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

#ifndef __DAC_H__
#define __DAC_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>
#include "rtos.h"

#if !defined(STM32F10X_HD) && !defined(STM32F10X_CL) && \
	!defined(STM32F10X_LD_VL) && !defined(STM32F10X_MD_VL) && \
	!defined(STM32F10X_HD_VL)
#error "No DAC available for this mcu!"
#endif

enum trig_t {
	TIM6_TRGO = 0,
	TIM3_TRGO = 1,
	TIM7_TRGO = 2,
	TIM5_TRGO = 3,
	TIM2_TRGO = 4,
	TIM4_TRGO = 5,
};

/**
  * @brief  Start DAC 12 bit conversion using DMA.
  * @param  ch: 1 or 2 DAC channel output.
  * @param  src: Pointer to data source.
  * @param  size: Size of data to transfer.
  * @param  trig: Trigger source. Choose available timer.
  * @param  freq: Frequency in Hz.
  *
  * @retval 0 if success.
  */
int dac_start_12bit(uint8_t ch, uint16_t *src, uint16_t size,
	enum trig_t trig, uint32_t freq);

/**
  * @brief  Enable the end of conversion interrupt.
  * @param  handler: Pointer to function. 0 to disable.
  * @param  data: Private data pointer.
  *
  * @retval None.
  */
void dac_enable_interrupt(void (*handler)(void *data), void *data);

#ifdef FREERTOS

/**
  * @brief  Start DAC 12 bit conversion using DMA and RTOS.
  * @param  ch: 1 or 2 DAC channel output.
  * @param  src: Pointer to data source.
  * @param  size: Size of data to transfer.
  * @param  trig: Trigger source. Choose available timer.
  * @param  freq: Frequency in Hz.
  *
  * @retval 0 if success.
  */
int dac_start_12bit_rtos(uint8_t ch, uint16_t *src, uint16_t size,
	enum trig_t trig, uint32_t freq);

#endif /* FREERTOS */

#ifdef __cplusplus
}
#endif

#endif /* __DAC_H__ */
