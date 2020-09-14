/*
 * This file is provided under a MIT license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * MIT License
 *
 * Copyright (c) 2020 Pavel Nadein
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

#include "spi.h"
#include "rcc.h"
#include "gpio.h"

static SPI_TypeDef *spi_s[] = { SPI1, SPI2 };
static DMA_Channel_TypeDef *dma_ch_s[] = {
	DMA1_Channel1,
	DMA1_Channel2,
	DMA1_Channel3,
	DMA1_Channel4,
	DMA1_Channel5,
	DMA1_Channel6,
	DMA1_Channel7,
};

static struct isr_t {
	struct msg_t *msg;
	void (*handler)(uint8_t spi_num, void *private_data);
	void *private_data;
	struct {
		GPIO_TypeDef *gpio;
		uint16_t pin;
	} cs;
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

static int start_dma(uint8_t spi_num, struct msg_t *msg)
{
	/* available dma channels (-1) starting from 0 */
	const uint8_t rx_ch[] = { 1, 3 };
	const uint8_t tx_ch[] = { 2, 4 };
	SPI_TypeDef *spi = spi_s[spi_num];
	uint8_t ch_num_tx = tx_ch[spi_num];
	uint8_t ch_num_rx = rx_ch[spi_num];
	DMA_Channel_TypeDef *tx = dma_ch_s[ch_num_tx];
	DMA_Channel_TypeDef *rx = dma_ch_s[ch_num_rx];

	spi->CR1 |= SPI_CR1_SPE;

	if (msg->tx) {
		spi->CR2 |= SPI_CR2_TXDMAEN;	
		tx->CPAR = (uint32_t)&spi->DR;
		tx->CMAR = (uint32_t)msg->tx;
		tx->CNDTR = msg->size;
		tx->CCR = DMA_CCR1_PSIZE | DMA_CCR1_MINC | DMA_CCR1_DIR | \
			DMA_CCR1_EN | (msg->rx ? 0 : DMA_CCR1_TCIE);
	} else
		tx->CCR = 0;
	
	if (msg->rx) {
		spi->CR2 |= SPI_CR2_RXDMAEN;
		rx->CPAR = (uint32_t)&spi->DR;
		rx->CMAR = (uint32_t)msg->rx;
		rx->CNDTR = msg->size;
		rx->CCR = DMA_CCR1_PSIZE | DMA_CCR1_MINC | DMA_CCR1_TCIE | \
			DMA_CCR1_EN | DMA_CCR1_TCIE;
	} else
		rx->CCR = 0;

	return 0;
}

static int transfer(uint8_t spi_num, struct msg_t **msg,
	GPIO_TypeDef *gpio, uint16_t pin)
{
	struct msg_t *m = *msg;
	struct isr_t *isr;

	if (!spi_num || spi_num > 2)
		return -EINVAL;

	spi_num--;
	isr = &isrs[spi_num];

	if (isr->msg)
		return -EILSEQ;

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

	while (isr->msg) { }
	spi_release(gpio, pin);

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

int spi_init(uint8_t spi_num, uint32_t freq, bool clock_high)
{
	SPI_TypeDef *spi;
	uint16_t clock_div;
	uint16_t clock_mode = clock_high ? SPI_CR1_CPHA | SPI_CR1_CPOL : 0;
	struct system_clock_t *clocks = get_clocks();
	uint8_t i;

	if (!spi_num || spi_num > 2)
		return -EINVAL;

	spi_num--;
	spi = spi_s[spi_num];

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	for (i = 0; i != 7; i++)
		NVIC_EnableIRQ((enum IRQn)(DMA1_Channel1_IRQn + i));

	switch (spi_num) {
	case 0:
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		gpio_output_init(PA5, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		gpio_output_init(PA6, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		gpio_output_init(PA7, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		clock_div = calc_clock_div(clocks->apb2_freq, freq);
		break;
	case 1:
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		gpio_output_init(PB13, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		gpio_output_init(PB14, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		gpio_output_init(PB15, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		clock_div = calc_clock_div(clocks->apb1_freq, freq);
		break;
	default:
		return -EINVAL;
	}

	spi->CR2 = 0;
	spi->CR1 = (clock_div << 3) | clock_mode | SPI_CR1_SSI | \
		SPI_CR1_SSM | SPI_CR1_MSTR;

	return 0;
}

static void isr(void)
{
	struct isr_t *isr = &isrs[1];

	isr->msg = isr->msg->next;

	if (isr->msg)
		start_dma(1, isr->msg);
}

void DMA1_Channel1_IRQHandler(void)
{}
void DMA1_Channel2_IRQHandler(void)
{}
void DMA1_Channel3_IRQHandler(void)
{}
void DMA1_Channel4_IRQHandler(void)
{}
void DMA1_Channel5_IRQHandler(void)
{
	DMA1->IFCR = 0x0FFFFFFF;
	DMA1_Channel5->CCR = 0;
	isr();
}
void DMA1_Channel6_IRQHandler(void)
{
	DMA1->IFCR = 0x0FFFFFFF;
	isr();
}
void DMA1_Channel7_IRQHandler(void)
{}

int spi_write_reg(uint8_t spi_num, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size)
{
	struct msg_t msg[] = {
		{ &reg, 0, 1 },
		{ data, 0, size },
	};
	struct msg_t *msgs[] = { &msg[0], &msg[1], 0 };

	return transfer(spi_num, msgs, gpio, pin);
}	
