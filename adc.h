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

#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f10x.h>
#include <stdint.h>
#include "rtos.h"

/**
  * @brief  Init ADC and start the conversion.
  * @param  adc_num: 1 or 2 which ADC to use.
  * @param  channel: Channel number 0..16.
  * @param  sample_rate: Sampling factor 0..7.
  *
  * @retval 0 if success.
  */
int adc_start(uint8_t adc_num, uint8_t channel, uint8_t sample_rate);

/**
  * @brief  Wait for end of conversion and get a result.
  * @param  adc_num: 1 or 2 which ADC to use.
  *
  * @retval 12 bit raw adc value or negative error value.
  */
int adc_read(uint8_t adc_num);

/**
  * @brief  Perform single conversion.
  * @param  adc_num: 1 or 2 which ADC to use.
  * @param  channel: Channel number 0..16.
  * @param  sample_rate: Sampling factor 0..7.
  *
  * @retval 12 bit raw adc value or negative error value.
  */
int adc_single_conversion(uint8_t adc_num, uint8_t channel,
	uint8_t sample_rate);

/**
  * @brief  Enable interrupt.
  * @param  adc_num: 1 or 2 which ADC to use.
  * @param  handler: Pointer to function that will be executed in ISR.
  *
  * @retval 12 bit raw adc value or negative error value.
  */
void adc_enable_interrupt(uint8_t adc_num,
	void (*handler)(uint8_t adc_num, uint16_t value));

#ifdef FREERTOS

/**
  * @brief  Perform single conversion using FreeRTOS.
  * @param  adc_num: 1 or 2 which ADC to use.
  * @param  channel: Channel number 0..16.
  *
  * @retval 12 bit raw adc value or negative error value.
  */
int adc_single_conversion_rtos(uint8_t adc_num, uint8_t channel);

#endif /* FREERTOS */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
