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

#ifndef __STM32F103_GPIO_H__
#define __STM32F103_GPIO_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

enum input_mode_t {
	ANALOG_INPUT,
	DIGITAL_INPUT,
	PULL_UP_INPUT,
	PULL_DOWN_INPUT,
};

enum output_mode_t {
	PUSHPULL_OUTPUT,
	OPENDRAIN_OUTPUT,
	PUSHPULL_ALT_OUTPUT,
	OPENDRAIN_ALT_OUTPUT,
};

enum freq_gpio_t {
	GPIO_FREQ_2MHz,
	GPIO_FREQ_10MHz,
	GPIO_FREQ_50MHz,
};

void gpio_output_init(GPIO_TypeDef *gpio, uint16_t pinmask,
	enum output_mode_t mode, enum freq_gpio_t freq);
void gpio_input_init(GPIO_TypeDef *gpio, uint16_t pinmask,
	enum input_mode_t mode);
void gpio_set_state(GPIO_TypeDef *gpio, uint16_t pinmask, bool state);

static inline void gpio_set(GPIO_TypeDef *gpio, uint16_t pinmask)
{
	gpio->BSRR = pinmask;
}

static inline void gpio_reset(GPIO_TypeDef *gpio, uint16_t pinmask)
{
	gpio->BRR = pinmask;
}

static inline uint16_t gpio_read(GPIO_TypeDef *gpio, uint16_t pinmask)
{
	return gpio->IDR & pinmask;
}

#ifdef __cplusplus
}
#endif

#endif /* __STM32F103_GPIO_H__ */
