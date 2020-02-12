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
 * STM32F103 open source driver
 *
 * Contact Information:
 * Pavel Nadein <pavelnadein@gmail.com>
 */

#include "stm32f103_gpio.h"

static void rcc_enable(GPIO_TypeDef *gpio)
{
	if (gpio == GPIOA)
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if (gpio == GPIOB)
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if (gpio == GPIOC)
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	else if (gpio == GPIOD)
		RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	else if (gpio == GPIOE)
		RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
	else if (gpio == GPIOF)
		RCC->APB2ENR |= RCC_APB2ENR_IOPFEN;
	else if (gpio == GPIOG)
		RCC->APB2ENR |= RCC_APB2ENR_IOPGEN;
}

static void output_init(GPIO_TypeDef *gpio, uint8_t pin,
	enum output_mode_t mode, enum freq_gpio_t freq)
{
	uint16_t cr = (uint16_t)freq | ((uint16_t)mode << 2);

	pin = pin << 2;

	if (pin <= 31) {
		gpio->CRL &= ~(0xF << pin);
		gpio->CRL |= cr << pin;
	} else {
		pin -= 31;
		gpio->CRH &= ~(0xF << pin);
		gpio->CRH |= cr << pin;
	}
}

static void input_init(GPIO_TypeDef *gpio, uint8_t pin, enum input_mode_t mode)
{
	uint16_t cr;

	switch (mode) {
	case ANALOG_INPUT:
		cr = 0 << 2;
		break;
	case DIGITAL_INPUT:
		cr = 1 << 2;
		break;
	case PULL_UP_INPUT:
		cr = 2 << 2;
		gpio->BSRR = 1 << pin;
		break;
	case PULL_DOWN_INPUT:
		cr = 2 << 2;
		gpio->BRR = 1 << pin;
		break;
	}

	pin = pin << 2;

	if (pin <= 31) {
		gpio->CRL &= ~(0xF << pin);
		gpio->CRL |= cr << pin;
	} else {
		pin -= 31;
		gpio->CRH &= ~(0xF << pin);
		gpio->CRH |= cr << pin;
	}
}

void gpio_output_init(GPIO_TypeDef *gpio, uint16_t pinmask,
	enum output_mode_t mode, enum freq_gpio_t freq)
{
	uint8_t i;

	rcc_enable(gpio);
	if (mode >= PUSHPULL_ALT_OUTPUT)
		RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	for (i = 0; i != 15; i++)
		if (pinmask & (1 << i))
			output_init(gpio, i, mode, freq);
}

void gpio_input_init(GPIO_TypeDef *gpio, uint16_t pinmask,
	enum input_mode_t mode)
{
	uint8_t i;

	rcc_enable(gpio);

	for (i = 0; i != 15; i++)
		if (pinmask & (1 << i))
			input_init(gpio, i, mode);
}

void gpio_set_state(GPIO_TypeDef *gpio, uint16_t pinmask, bool state)
{
	state ? gpio_set(gpio, pinmask) : gpio_reset(gpio, pinmask);
}
