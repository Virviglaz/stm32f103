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

#include "exti.h"
#include <errno.h>

#define NOF_EXTI_LINES		16

#define ENABLE_EXTI_LINE(pin, line) \
	AFIO->EXTICR[pin >> 2] |= line << ((pin % 4) << 2)
#define DISABLE_EXTI_LINE(pin, line) \
	AFIO->EXTICR[pin >> 2] &= ~(line << ((pin % 4) << 2))

static struct exit_isr_t {
	void (*handler)(void *private_data);
	void *data;
} isrs[NOF_EXTI_LINES];

static uint8_t gpio_to_index(GPIO_TypeDef *gpio)
{
	return (uint8_t)(((uint32_t)gpio - (uint32_t)GPIOA) >> 10);
}

static inline int set_pin_int(GPIO_TypeDef *gpio, uint8_t pin,
	void (*handler)(void *private_data),
		void *data, bool rising, bool falling)
{
	uint8_t line;
	struct exit_isr_t *isr;
	uint16_t pin_mask = BIT(pin);

	if (pin >= ARRAY_SIZE(isrs))
		return -EINVAL;

	line = gpio_to_index(gpio);
	isr = &isrs[pin];

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	/* Check line is occupied already */
	if (isr->handler || isr->data)
		return -EINVAL;

	isr->handler = handler;
	isr->data = data;

	ENABLE_EXTI_LINE(pin, line);

	EXTI->IMR |= pin_mask;

	if (rising)
		EXTI->RTSR |= pin_mask;
	else
		EXTI->RTSR &= ~pin_mask;

	if (falling)
		EXTI->FTSR |= pin_mask;
	else
		EXTI->FTSR &= ~pin_mask;

	if (pin < 5)
		NVIC_EnableIRQ((enum IRQn)(EXTI0_IRQn + pin));
	else if (pin < 10)
		NVIC_EnableIRQ(EXTI9_5_IRQn);
	else
		NVIC_EnableIRQ(EXTI15_10_IRQn);

	return 0;
}

static inline int clr_pin_int(GPIO_TypeDef *gpio, uint16_t pin)
{
	uint8_t line;
	struct exit_isr_t *isr;

	if (pin >= ARRAY_SIZE(isrs))
		return -EINVAL;

	line = gpio_to_index(gpio);
	isr = &isrs[pin];

	isr->handler = 0;
	isr->data = 0;

	DISABLE_EXTI_LINE(pin, line);

	if (pin < 5)
		NVIC_DisableIRQ((enum IRQn)(EXTI0_IRQn + pin));
	else if (pin < 10)
		NVIC_DisableIRQ(EXTI9_5_IRQn);
	else
		NVIC_DisableIRQ(EXTI15_10_IRQn);

	return 0;
}

int add_pinchange_interrupt(GPIO_TypeDef *gpio, uint16_t pin,
	void (*handler)(void *private_data),
		void *data, bool rising, bool falling)
{
	uint16_t i;
	int ret = -EINVAL;

	for (i = 0; i != 16; i++)
		if (pin & BIT(i)) {
			ret = set_pin_int(gpio, i, handler, data,
				rising, falling);
			if (ret)
				return ret;
		}
	return ret;
}

int remove_pinchange_interrupt(GPIO_TypeDef *gpio, uint16_t pin)
{
	uint16_t i;
	int ret = -EINVAL;

	for (i = 0; i != 16; i++)
		if (pin & BIT(i)) {
			ret = clr_pin_int(gpio, i);
			if (ret)
				return ret;
		}
	return ret;
}

static void isr_handler()
{
	uint8_t i;
	uint16_t source = EXTI->PR;

	/* clear all interrupts */
	EXTI->PR = 0xFFFFF;

	for (i = 0; i != NOF_EXTI_LINES; i++)
		if (source & (1 << i) && isrs[i].handler)
			isrs[i].handler(isrs[i].data);
}

void EXTI0_IRQHandler(void)
{
	isr_handler();
}

void EXTI1_IRQHandler(void)
{
	isr_handler();
}

void EXTI2_IRQHandler(void)
{
	isr_handler();
}

void EXTI3_IRQHandler(void)
{
	isr_handler();
}

void EXTI4_IRQHandler(void)
{
	isr_handler();
}

void EXTI9_5_IRQHandler(void)
{
	isr_handler();
}

void EXTI15_10_IRQHandler(void)
{
	isr_handler();
}
