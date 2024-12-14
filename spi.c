/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2020-2024 Pavel Nadein
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * STM32F10x open source driver
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#include <stm32f10x.h>
#include "spi.h"
#include "rcc.h"
#include "dma.h"
#include "gpio.h"

static SPI_TypeDef *spi_s[] = { SPI1, SPI2 };

static struct isr_t {
	struct msg_t *msg;
	void (*handler)(uint8_t spi_num, void *private_data);
	void *private_data;
	struct {
		GPIO_TypeDef *gpio;
		uint16_t pin;
	} cs;
	DMA_Channel_TypeDef *dma_tx;
	DMA_Channel_TypeDef *dma_rx;
	uint8_t spi_num;
	bool done;
} isrs[2] = { 0 };

inline static void spi_select(GPIO_TypeDef *GPIOx, uint16_t PINx)
{	
	GPIOx->BRR = PINx;
}

inline static void spi_release(GPIO_TypeDef *GPIOx, uint16_t PINx)
{
	GPIOx->BSRR = PINx;	
}

struct msg_t {
	uint8_t *tx;
	uint8_t *rx;
	uint16_t size;
	struct msg_t *next;
};

static void start_dma(uint8_t spi_num, struct msg_t *msg)
{
	const uint8_t dummy_byte = 0;
	SPI_TypeDef *spi = spi_s[spi_num];
	DMA_Channel_TypeDef *tx = isrs[spi_num].dma_tx;

	spi->CR1 &= ~SPI_CR1_SPE;
	spi->CR2 = SPI_CR2_TXDMAEN;

	tx->CPAR = (uint32_t)&spi->DR;
	tx->CNDTR = msg->size;

	if (msg->tx) {
		tx->CMAR = (uint32_t)msg->tx;
		tx->CCR = DMA_CCR1_MINC | DMA_CCR1_DIR | \
			DMA_CCR1_EN | (msg->rx ? 0 : DMA_CCR1_TCIE);
	} else {
		tx->CMAR = (uint32_t)&dummy_byte;
		tx->CCR = DMA_CCR1_DIR | DMA_CCR1_EN | \
			(msg->rx ? 0 : DMA_CCR1_TCIE);
	}

	if (msg->rx) {
		DMA_Channel_TypeDef *rx = isrs[spi_num].dma_rx;
		spi->CR2 |= SPI_CR2_RXDMAEN;
		rx->CPAR = (uint32_t)&spi->DR;
		rx->CMAR = (uint32_t)msg->rx;
		rx->CNDTR = msg->size;
		rx->CCR =  DMA_CCR1_MINC | DMA_CCR1_TCIE | DMA_CCR1_EN;
	}

	spi->CR1 |= SPI_CR1_SPE;
}

static void dma_isr(void *data)
{
	struct isr_t *isr = data;
	SPI_TypeDef *spi;

	isr->msg = isr->msg->next;

	if (isr->msg) {
		start_dma(isr->spi_num, isr->msg);
		return;
	}

	spi = spi_s[isr->spi_num];
	spi->CR2 = SPI_CR2_RXNEIE;
}

static void spi_isr(struct isr_t *isr)
{
	spi_release(isr->cs.gpio, isr->cs.pin);

	dma_release(isr->dma_tx);
	dma_release(isr->dma_rx);

	if (isr->handler)
		isr->handler(isr->spi_num, isr->private_data);
	isr->done = true;
}

static int transfer(uint8_t spi_num, struct msg_t **msg,
	GPIO_TypeDef *gpio, uint16_t pin,
	void (*handler)(uint8_t spi_num, void *private_data),
	void *private_data)
{
	const uint8_t tx_ch[] = { 3, 5 };
	const uint8_t rx_ch[] = { 2, 4 };
	struct msg_t *m = *msg;
	struct isr_t *isr;

	if (!spi_num || spi_num > 2)
		return -EINVAL;

	spi_num--;
	isr = &isrs[spi_num];

	if (isr->msg)
		return -EILSEQ;

	isr->dma_tx = get_dma_ch(tx_ch[spi_num], dma_isr, isr);
	isr->dma_rx = get_dma_ch(rx_ch[spi_num], dma_isr, isr);

	if (!isr->dma_tx || !isr->dma_rx) {
		dma_release(isr->dma_tx);
		dma_release(isr->dma_rx);
		return -EILSEQ;
	}

	isr->spi_num = spi_num;
	isr->done = false;
	isr->handler = handler;
	isr->private_data = private_data;

	/* first message */
	isr->msg = m;
	isr->cs.gpio = gpio;
	isr->cs.pin = pin;

	/* build a linked list */
	do {
		msg++;
		m->next = *msg;
		m = m->next;
	} while (m);

	spi_select(gpio, pin);
	start_dma(spi_num, isr->msg);

	/* wait for execution if no handler provided */
	if (!isr->handler)
		while (!isr->done) { }

	return 0;
}

static inline uint16_t calc_clock_div(uint32_t bus_freq, uint32_t expected_freq)
{
	uint32_t freq;
	uint16_t div = 1;
	uint16_t n = 0;

	do {
		div <<= 1;
		n++;
		freq = bus_freq / div;
	} while (freq > expected_freq);

	return n - 1;
}

void SPI1_IRQHandler(void)
{
	SPI1->DR;
	SPI1->CR2 = 0;
	spi_isr(&isrs[0]);
}

void SPI2_IRQHandler(void)
{
	SPI2->DR;
	SPI2->CR2 = 0;
	spi_isr(&isrs[1]);
}

#ifdef FREERTOS

#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

static struct params {
	SemaphoreHandle_t lock;
	SemaphoreHandle_t done;
} params[2];
#endif

int spi_init(uint8_t spi_num, uint32_t freq, bool clock_high)
{
	SPI_TypeDef *spi;
	uint16_t clock_div;
	uint16_t clock_mode = clock_high ? SPI_CR1_CPHA | SPI_CR1_CPOL : 0;
	struct system_clock_t *clocks = get_clocks();

	if (!spi_num || spi_num > 2)
		return -EINVAL;

	spi_num--;
	spi = spi_s[spi_num];

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	switch (spi_num) {
	case 0:
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		gpio_output_init(PA5, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		gpio_input_init(PA6, PULL_UP_INPUT);
		gpio_output_init(PA7, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		clock_div = calc_clock_div(clocks->apb2_freq, freq);
		NVIC_EnableIRQ(SPI1_IRQn);
		break;
	case 1:
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		gpio_output_init(PB13, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		gpio_input_init(PB14, PULL_UP_INPUT);
		gpio_output_init(PB15, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		clock_div = calc_clock_div(clocks->apb1_freq, freq);
		NVIC_EnableIRQ(SPI2_IRQn);
		break;
	default:
		return EINVAL;
	}

	spi->CR2 = 0;
	spi->CR1 = (clock_div << 3) | clock_mode | SPI_CR1_SSI | \
		SPI_CR1_SSM | SPI_CR1_MSTR;

#ifdef FREERTOS
	params[spi_num].lock = xSemaphoreCreateMutex();
	params[spi_num].done = xSemaphoreCreateBinary();
#endif

	return 0;
}

static int send_message(
	uint8_t spi_num,
	GPIO_TypeDef *gpio,
	uint16_t pin,
	uint8_t *reg,
	uint8_t reg_size,
	uint8_t *tx_data,
	uint8_t *rx_data,
	uint16_t size,
	void (*handler)(uint8_t spi_num, void *private_data),
	void *private_data)
{
	/* we use static variable to reduce the stack usage using RTOS */
	static struct msg_t msg[2];
	static struct msg_t *msgs[3];

	msg[0].tx = reg;
	msg[0].rx = 0;
	msg[0].size = reg_size;

	msg[1].tx = tx_data;
	msg[1].rx = rx_data;
	msg[1].size = size;

	msgs[0] = &msg[0];
	msgs[1] = &msg[1];
	msgs[2] = 0;

	return transfer(spi_num, msgs, gpio, pin, handler, private_data);
}

int spi_write_reg(uint8_t spi_num, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size)
{
	return send_message(spi_num, gpio, pin, &reg, 1,
		data, 0, size, 0, 0);
}

int spi_read_reg(uint8_t spi_num, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size)
{
	return send_message(spi_num, gpio, pin, &reg, 1,
		0, data, size, 0, 0);
}

#ifdef FREERTOS

static void handler(uint8_t spi_num, void *private_data)
{
	(void)private_data;
	xSemaphoreGiveFromISR(params[spi_num].done, NULL);
}

static int send_message_rtos(
	uint8_t spi_num,
	GPIO_TypeDef *gpio,
	uint16_t pin,
	uint8_t *reg,
	uint8_t reg_size,
	uint8_t *tx_data,
	uint8_t *rx_data,
	uint16_t size)
{
	int res;
	uint8_t n = spi_num - 1;

	xSemaphoreTake(params[n].lock, portMAX_DELAY);

	res = send_message(spi_num, gpio, pin, reg, reg_size,
		tx_data, rx_data, size, handler, &params[n]);
	if (!res)
		xSemaphoreTake(params[n].done, portMAX_DELAY);

	xSemaphoreGive(params[n].lock);

	return res;
}

int spi_write_reg_rtos(uint8_t spi_num, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size)
{
	return send_message_rtos(spi_num, gpio, pin, &reg, 1, data, 0, size);
}

int spi_read_reg_rtos(uint8_t spi_num, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size)
{
	return send_message_rtos(spi_num, gpio, pin, &reg, 1, 0, data, size);
}

#endif /* FREERTOS */
