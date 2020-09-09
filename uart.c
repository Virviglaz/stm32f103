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
#include <errno.h>

static USART_TypeDef *uarts[] = { 0, USART1, USART2, USART3 };

#define UART_BRR_SAMPLING16(_PCLK_, _BAUD_) (((_PCLK_) + (_BAUD_) / 2) / _BAUD_)
__INLINE static uint16_t UART_BRR_SAMPLING8(uint32_t _PCLK_, uint32_t _BAUD_)
{
    uint16_t Div = (_PCLK_ + _BAUD_ / 2) / _BAUD_;  
    return ((Div & ~0x7) << 1 | (Div & 0x07));
}

int uart_init(uint8_t uart_num, uint32_t freq)
{
	USART_TypeDef *uart = uarts[uart_num];
	struct system_clock_t *clock = get_clocks();
	uint32_t clock_source;

	if (!uart_num || uart_num > ARRAY_SIZE(uarts))
		return -EINVAL;

	switch (uart_num) {
	case 1:
		clock_source = clock->apb2_freq;
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		gpio_output_init(PA9, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_2MHz);
		gpio_input_init(PA10, PULL_UP_INPUT);
		break;
	case 2:
		clock_source = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		gpio_output_init(PA2, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_2MHz);
		gpio_input_init(PA3, PULL_UP_INPUT);
		break;
	case 3:
		clock_source = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		gpio_output_init(PB10, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_2MHz);
		gpio_input_init(PB11, PULL_UP_INPUT);
		break;
	}

	uart->BRR = UART_BRR_SAMPLING8(clock_source, freq);
	uart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	uart->CR2 = 0;
	uart->CR3 = 0;

	return 0;
}

void uart_write(uint8_t uart_num, char ch)
{
	while (!(uarts[uart_num]->SR & USART_SR_TXE));

	uarts[uart_num]->DR = ch;
}

static struct {
	void (*handler)(uint8_t uart_num, char ch, void *data);
	void *data;
} isr[3];

int uart_enable_interrupt(uint8_t uart_num,
	void (*handler)(uint8_t uart_num, char ch, void *data), void *data)
{
	USART_TypeDef *uart = uarts[uart_num];
	if (!uart_num || uart_num > ARRAY_SIZE(uarts) ||
		!(uart->CR1 & USART_CR1_UE))
		return -EINVAL;

	isr[uart_num - 1].handler = handler;
	isr[uart_num - 1].data = data;

	uart->CR1 |= USART_CR1_RXNEIE;

	switch (uart_num) {
	case 1:
		NVIC_EnableIRQ(USART1_IRQn);
		break;
	case 2:
		NVIC_EnableIRQ(USART2_IRQn);
		break;
	case 3:
		NVIC_EnableIRQ(USART3_IRQn);
		break;
	}

	return 0;
}

void USART1_IRQHandler(void)
{
	isr[0].handler(1, USART1->DR, isr[0].data);
}

void USART2_IRQHandler(void)
{
	isr[1].handler(2, USART2->DR, isr[1].data);
}

void USART3_IRQHandler(void)
{
	isr[2].handler(3, USART3->DR, isr[2].data);
}
