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

#include "adc.h"
#include <stdbool.h>
#include <stm32f10x.h>
#include <errno.h>

struct adc_params_t {
	void (*handler)(uint8_t adc_num, uint16_t value);
	bool init_done;
};

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && \
	!defined (STM32F10X_HD_VL)
#define ADC_DEV(x)		((x == 1) ? ADC1 : ADC2)
struct adc_params_t adc_params[2] = { 0 };
#else
#define ADC_DEV(x)		ADC1
struct adc_params_t adc_params[1] = { 0 };
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */

static inline void calibrate(ADC_TypeDef *adc)
{
	adc->CR2 = ADC_CR2_RSTCAL | ADC_CR2_ADON;
	while (adc->CR2 & ADC_CR2_RSTCAL);
	adc->CR2 = ADC_CR2_CAL | ADC_CR2_ADON;
	while (adc->CR2 & ADC_CR2_CAL);
}

static inline void init_rcc(uint8_t adc_num)
{
#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && \
	!defined (STM32F10X_HD_VL)
	if (adc_num == 0)
		RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	else
		RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
#else
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */
}

static void init(uint8_t adc_num, ADC_TypeDef *adc)
{
	if (adc_params[adc_num].init_done)
		return;

	init_rcc(adc_num);
	calibrate(adc);
	adc_params[adc_num].init_done = true;
#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && \
	!defined (STM32F10X_HD_VL)
	NVIC_EnableIRQ(ADC1_2_IRQn);
#else
	NVIC_EnableIRQ(ADC1_IRQn);
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */
}

int adc_start(uint8_t adc_num, uint8_t channel, uint8_t sample_rate)
{
	ADC_TypeDef *adc = ADC_DEV(adc_num);
	if (!adc_num || adc_num > sizeof(adc_params) / sizeof(adc_params[0]))
		return -EINVAL;

	/* adc index starting from 0 */
	adc_num--;
	adc->CR2 = 0;

	init(adc_num, adc);

	if (channel > 17)
		return -EINVAL;

	sample_rate &= 0x07;

	if (channel >= 10)
		adc->SMPR1 = sample_rate << (channel - 10);
	else
		adc->SMPR2 = sample_rate << channel;

	adc->SQR1 = 0;
	adc->SQR3 = channel;
	adc->CR2 = ADC_CR2_ADON;
	adc->CR2 = ADC_CR2_ADON;

	return 0;
}

int adc_read(uint8_t adc_num)
{
	ADC_TypeDef *adc = ADC_DEV(adc_num);

	if (!adc_num || adc_num > sizeof(adc_params) / sizeof(adc_params[0]))
		return -EINVAL;

	if (!(adc->SR & ADC_SR_EOC))
		return -EINVAL;

	return (int)adc->DR;
}

int adc_single_conversion(uint8_t adc_num, uint8_t channel, uint8_t sample_rate)
{
	ADC_TypeDef *adc = ADC_DEV(adc_num);
	int res;

	if (!adc_num || adc_num > sizeof(adc_params) / sizeof(adc_params[0]))
		return -EINVAL;

	res = adc_start(adc_num, channel, sample_rate);
	if (res)
		return res;

	while (!(adc->SR & ADC_SR_EOC)) { }

	return (int)adc->DR;
}

void adc_enable_interrupt(uint8_t adc_num,
	void (*handler)(uint8_t adc_num, uint16_t value))
{
	ADC_TypeDef *adc = ADC_DEV(adc_num);

	if (!adc_num || adc_num > sizeof(adc_params) / sizeof(adc_params[0]))
		return;

	adc_num--;

	init(adc_num, adc);

	adc_params[adc_num].handler = handler;

	adc->CR1 = ADC_CR1_EOCIE;
}

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && \
	!defined (STM32F10X_HD_VL)
void ADC1_2_IRQHandler(void)
{
	if (ADC1->SR & ADC_SR_EOC) {
		ADC1->CR2 = 0;
		adc_params[0].handler(1, ADC1->DR);
		return;
	}
	if (ADC2->SR & ADC_SR_EOC) {
		ADC2->CR2 = 0;
		adc_params[1].handler(2, ADC2->DR);
		return;
	}
}
#else
void ADC1_2_IRQHandler(void)
{
	ADC1->CR2 = 0;
	adc_params[0].handler(1, ADC1->DR);
}
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */

#ifdef FREERTOS

#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t lock;
static SemaphoreHandle_t done;
static uint16_t result;

static void rtos_handler(uint8_t adc_num, uint16_t value)
{
	(void)adc_num;
	result = value;
	xSemaphoreGiveFromISR(done, NULL);
}

uint16_t adc_single_conversion_rtos(uint8_t adc_num, uint8_t channel)
{
	if (!lock) { /* Race condition here when preemption is enabled. */
		lock = xSemaphoreCreateMutex();
		done = xSemaphoreCreateBinary();
	}

	xSemaphoreTake(lock, portMAX_DELAY);

	adc_enable_interrupt(adc_num, rtos_handler);
	adc_start(adc_num, channel, 7);

	xSemaphoreTake(done, portMAX_DELAY);
	xSemaphoreGive(lock);

	return result;
}

#endif /* FREERTOS */
