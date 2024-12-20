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

#ifndef __EXTI_H__
#define __EXTI_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>

/**
  * @brief  Initialize EXTI and install the interrupt handler.
  * @param  gpio: Pointer to GPIO perepherial.
  * @param  pin: Pin number for pin change interrupt.
  * @param  handler: Pointer to handler function.
  * @param  data: Pointer to private data if needed.
  * @param  rising: Enable rising edge detection.
  * @param  falling: Enable falling edge detection.
  *
  * @retval 0 if success.
  */
int add_pinchange_interrupt(GPIO_TypeDef *gpio, uint16_t pin,
	void (*handler)(void *private_data),
		void *data, bool rising, bool falling);

/**
  * @brief  Disable EXTI and remove the interrupt handler.
  * @param  gpio: Pointer to GPIO perepherial.
  * @param  pin: Pin number for pin change interrupt.
  *
  * @retval 0 if success.
  */
int remove_pinchange_interrupt(GPIO_TypeDef *gpio, uint16_t pin);

#ifdef FREERTOS

/**
  * @brief  Block the task until pinchange interrupt appears.
  * @param  gpio: Pointer to GPIO perepherial.
  * @param  pin: Pin number for pin change interrupt.
  * @param  rising: Enable rising edge detection.
  * @param  falling: Enable falling edge detection.
  * @param  waiter: Binary SemaphoreHandle_t to be used for a waiting state.
  *
  * @retval 0 if success.
  */
int wait_for_pinchange_rtos(GPIO_TypeDef *gpio, uint16_t pin,
	bool rising, bool falling, void *waiter);

#endif /* FREERTOS */

#ifdef __cplusplus
}
#endif

#endif /* __EXTI_H__ */
