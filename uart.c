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

#include "uart.h"
#include "gpio.h"
#include "rcc.h"
#include "dma.h"
#include <errno.h>
#include <string.h>

static USART_TypeDef *uarts[] = { USART1, USART2, USART3 };
static struct rx_buf_t {
	char *buf;
	uint16_t size;
	uint16_t transfered;
	uart_rx_handler_t handler;
	void *private_data;
	bool ready;
} rx_buf[3] = { 0 };

static struct tx_buf_t {
	DMA_Channel_TypeDef *dma_ch;
	USART_TypeDef *uart;
	uart_tx_handler_t handler;
	void *private_data;
	bool done;
} tx_buf[3] = { 0 };

__INLINE static uint16_t UART_BRR_SAMPLING8(uint32_t _PCLK_, uint32_t _BAUD_)
{
    uint16_t Div = (_PCLK_ + _BAUD_ / 2) / _BAUD_;  
    return ((Div & ~0x7) << 1 | (Div & 0x07));
}

static void tx_isr(uint8_t uart_num, USART_TypeDef *uart)
{
	struct tx_buf_t *tx = &tx_buf[uart_num];

	uart->CR1 &= ~USART_CR1_TXEIE;

	if (tx->handler)
		tx->handler(tx->private_data);

	tx->done = true;
}

static void rx_isr(uint8_t uart_num, USART_TypeDef *uart)
{
	struct rx_buf_t *rx = &rx_buf[uart_num];
	char ch = uart->DR;

	rx->buf[rx->transfered] = ch;
	rx->transfered++;
	if (rx->transfered == rx->size || ch == '\r') {
		rx->ready = true;
		if (rx->handler) {
			rx->handler(uart_num + 1, rx->buf,
				rx->transfered, rx->private_data);
			rx->ready = false;
			rx->transfered = 0;
		}
	}
}

static void tx_dma_handler(void *data)
{
	struct tx_buf_t *tx = data;

	dma_release(tx->dma_ch);

	tx->uart->CR1 |= USART_CR1_TXEIE;
}

void uart_init(uint8_t uart_num, uint32_t freq)
{
	USART_TypeDef *uart = uarts[uart_num - 1];
	struct system_clock_t *clock = get_clocks();
	uint32_t clock_source;

	switch (uart_num) {
	case 1:
		clock_source = clock->apb2_freq;
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		gpio_output_init(PA9, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_2MHz);
		gpio_input_init(PA10, PULL_UP_INPUT);
		NVIC_EnableIRQ(USART1_IRQn);
		break;
	case 2:
		clock_source = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		gpio_output_init(PA2, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_2MHz);
		gpio_input_init(PA3, PULL_UP_INPUT);
		NVIC_EnableIRQ(USART2_IRQn);
		break;
	case 3:
		clock_source = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		gpio_output_init(PB10, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_2MHz);
		gpio_input_init(PB11, PULL_UP_INPUT);
		NVIC_EnableIRQ(USART3_IRQn);
		break;
	}

	uart->BRR = UART_BRR_SAMPLING8(clock_source, freq << 1);
	uart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	uart->CR2 = 0;
	uart->CR3 = USART_CR3_DMAT;
}

void uart_write(uint8_t uart_num, char ch)
{
	uart_num--;

	while (!(uarts[uart_num]->SR & USART_SR_TXE));

	uarts[uart_num]->DR = ch;
}

void uart_enable_rx_buffer(uint8_t uart_num, char *buf, uint16_t size,
	uart_rx_handler_t handler, void *private_data)
{
	USART_TypeDef *uart;
	struct rx_buf_t *rx;

	uart_num--;
	uart = uarts[uart_num];
	rx = &rx_buf[uart_num];

	rx->buf = buf;
	rx->size = size;
	rx->transfered = 0;
	rx->handler = handler;
	rx->private_data = private_data;
	rx->ready = false;

	uart->CR1 |= USART_CR1_RXNEIE;
}

void uart_disable_rx_buffer(uint8_t uart_num)
{
	USART_TypeDef *uart = uarts[uart_num - 1];

	uart->CR1 &= ~USART_CR1_RXNEIE;
}

int uart_send_data(uint8_t uart_num, char *buf, uint16_t size,
	uart_tx_handler_t handler, void *private_data)
{
	const uint8_t tx_ch[] = { 4, 7, 2 };
	USART_TypeDef *uart;
	struct tx_buf_t *tx;

	uart_num--;
	uart = uarts[uart_num];
	tx = &tx_buf[uart_num];

	tx->uart = uart;
	tx->dma_ch = get_dma_ch(tx_ch[uart_num], tx_dma_handler, tx);
	if (!tx->dma_ch)
		return -EINVAL;

	tx->handler = handler;
	tx->private_data = private_data;
	tx->dma_ch->CNDTR = size;
	tx->dma_ch->CPAR = (uint32_t)&uart->DR;
	tx->dma_ch->CMAR = (uint32_t)buf;
	tx->dma_ch->CCR = DMA_CCR1_MINC | DMA_CCR1_DIR | DMA_CCR1_TCIE | \
		DMA_CCR1_EN;

	/* without handler wait for execution */
	if (!handler)
		while (!tx->done) { }

	return 0;
}

void uart_send_string(uint8_t uart_num, const char *str)
{
	uart_send_data(uart_num, (char *)str, strlen(str), 0, 0);
}

uint16_t uart_received_bytes(uint8_t uart_num)
{
	return rx_buf[uart_num - 1].transfered;
}

bool uart_check_rx_buffer(uint8_t uart_num)
{
	return rx_buf[uart_num - 1].ready;
}

void uart_reset_rx_buffer(uint8_t uart_num)
{
	rx_buf[uart_num - 1].transfered = 0;
	rx_buf[uart_num - 1].ready = false;
}

char *uart_get_rx_buffer(uint8_t uart_num)
{
	return rx_buf[uart_num - 1].buf;
}

void USART1_IRQHandler(void)
{
	uint16_t sr = USART1->SR;

	if (sr & USART_SR_RXNE)
		rx_isr(0, USART1);

	if (sr & USART_SR_TXE)
		tx_isr(0, USART1);
}

void USART2_IRQHandler(void)
{
	uint16_t sr = USART2->SR;

	if (sr & USART_SR_RXNE)
		rx_isr(1, USART2);

	if (sr & USART_SR_TXE)
		tx_isr(1, USART2);
}

void USART3_IRQHandler(void)
{
	uint16_t sr = USART3->SR;

	if (sr & USART_SR_RXNE)
		rx_isr(2, USART3);

	if (sr & USART_SR_TXE)
		tx_isr(2, USART3);
}

#ifdef FREERTOS

static SemaphoreHandle_t tx_mutex[3] = { 0 };
static SemaphoreHandle_t rx_mutex[3] = { 0 };
static uint16_t bytes_received[3];

static void rtos_tx_handler(void *private_data)
{
	rtos_schedule_isr(private_data);
}

static void rtos_rx_handler(uint8_t uart_num, char *data, uint16_t size,
	void *private_data)
{
	bytes_received[uart_num - 1] = size;

	rtos_schedule_isr(private_data);
}

void uart_send_data_rtos(uint8_t uart_num, char *buf, uint16_t size)
{
	TaskHandle_t handle;

	if (!tx_mutex[uart_num - 1])
		tx_mutex[uart_num - 1] = xSemaphoreCreateMutex();

	handle = xTaskGetCurrentTaskHandle();

	xSemaphoreTake(tx_mutex[uart_num - 1], portMAX_DELAY);

	if (!uart_send_data(uart_num, buf, size, rtos_tx_handler, handle))
		vTaskSuspend(handle);

	xSemaphoreGive(tx_mutex[uart_num - 1]);
}

void uart_send_string_rtos(uint8_t uart_num, char *string)
{
	uart_send_data_rtos(uart_num, string, strlen(string));
}

uint16_t uart_receive_rtos(uint8_t uart_num, char *buf, uint16_t size)
{
	TaskHandle_t handle;

	if (!rx_mutex[uart_num - 1])
		rx_mutex[uart_num - 1] = xSemaphoreCreateMutex();

	xSemaphoreTake(rx_mutex[uart_num - 1], portMAX_DELAY);

	handle = xTaskGetCurrentTaskHandle();

	uart_enable_rx_buffer(uart_num, buf, size, rtos_rx_handler, handle);

	vTaskSuspend(handle);

	xSemaphoreGive(rx_mutex[uart_num - 1]);

	return bytes_received[uart_num - 1];
}

#endif /* FREERTOS */
