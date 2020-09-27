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
 
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

/**
  * @brief  Initialize the timer.
  * @param  tim: Number of timer.
  * @param  freq: Timer frequency in MHz (0 if period is used)
  * @param  sec: Timer period in sec.
  *
  * @retval 0 if success.
  */
int timer_init(uint8_t tim, uint32_t freq, uint32_t period);

/**
  * @brief  Start the timer.
  * @param  tim: Number of timer.
  * @param  state: enable/disable the timer.
  *
  * @retval 0 if success.
  */
int timer_enable(uint8_t tim, bool state);

/**
  * @brief  Set timer period.
  * @param  tim: Number of timer.
  * @param  period: New period value.
  *
  * @retval 0 if success.
  */
int timer_set_period(uint8_t tim, uint16_t period);

/**
  * @brief  Enable PWM output.
  * @param  tim: Number of timer.
  * @param  ch: Channel number 1..4.
  * @param  duty: duty cycle value.
  *
  * @note   GPIO have to be initialized separatly
  *
  * @retval 0 if success.
  */
int timer_pwm_enable(uint8_t tim, uint8_t ch, u16 duty);

/**
  * @brief  Set PWM duty.
  * @param  tim: Number of timer.
  * @param  ch: Channel number 1..4.
  * @param  duty: duty cycle value.
  *
  * @retval 0 if success.
  */
int timer_pwm_set_duty(uint8_t tim, uint8_t ch, u16 duty);

/**
  * @brief  Enable timer interrupt.
  * @param  tim: Number of timer.
  * @param  handler: pointer to interrupt function.
  * @param  data: pointer to private data.
  *
  * @retval 0 if success.
  */
int timer_enable_interrupt(uint8_t tim,
	void (*handler)(uint8_t tim, void *data), void *data);

/**
  * @brief  Set timer period.
  * @param  tim: Number of timer.
  * @param  period: period value.
  *
  * @retval 0 if success.
  */
int set_timer_period(uint8_t tim, uint16_t period);

/**
  * @brief  Enable timer event TRGO.
  * @param  tim: Number of timer.
  * @param  state: Enable/disable TRGO.
  *
  * @retval 0 if success.
  */
int timer_enable_update_event(uint8_t tim, bool state);

/**
  * @brief  Disable the timer.
  * @param  tim: Number of timer.
  *
  * @retval 0 if success.
  */
int timer_deinit(uint8_t tim);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
