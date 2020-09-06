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

#include "adc.h"
#include <stdbool.h>
#include <errno.h>

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && \
	!defined (STM32F10X_HD_VL)
static bool init_done[2] = { false, false };
static struct {
	void (*handler)(uint8_t adc_num, uint16_t value);
} adc_isr[2];
#else
static bool init_done = false;
void (*handler)(uint8_t adc_num, uint16_t value);
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */

static inline void calibrate(ADC_TypeDef *adc)
{
	adc->CR2 = ADC_CR2_RSTCAL | ADC_CR2_ADON;
	while (adc->CR2 & ADC_CR2_RSTCAL);
	adc->CR2 = ADC_CR2_CAL | ADC_CR2_ADON;
	while (adc->CR2 & ADC_CR2_CAL);
}

static void init(ADC_TypeDef *adc)
{
#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && \
	!defined (STM32F10X_HD_VL)
	if (adc == ADC1)
		RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	else
		RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
#else
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */

	//adc->CR2 = ADC_CR2_ADON;
	calibrate(adc);
}

int adc_init(uint8_t adc_num, uint8_t channel, uint8_t sample_rate)
{
#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && \
	!defined (STM32F10X_HD_VL)
	ADC_TypeDef *adc = adc_num == 1 ? ADC1 : ADC2;

	if (adc_num < 1 || adc_num > 2)
		return -EINVAL;

	if (!init_done[adc_num - 1]) {
		init(adc);
		init_done[adc_num - 1] = true;
	}
#else
	ADC_TypeDef *adc = ADC1;
	(void)adc_num;

	if (!init_done) {
		init(adc);
		init_done = true;
	}
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */
	if (channel > 17)
		return -EINVAL;

	sample_rate &= 0x07;

	if (channel >= 10)
		adc->SMPR1 = sample_rate << (channel - 10);
	else
		adc->SMPR2 = sample_rate << channel;

	adc->SQR1 = 0;
	adc->SQR3 = channel;
	adc_start(adc_num);

	return 0;
}

int adc_read(uint8_t adc_num)
{
#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && \
	!defined (STM32F10X_HD_VL)
	ADC_TypeDef *adc = adc_num == 1 ? ADC1 : ADC2;
	if (adc_num < 1 || adc_num > 2)
		return -EINVAL;
#else
	ADC_TypeDef *adc = ADC1;
	(void)adc_num;
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */

	if (!(adc->SR & ADC_SR_EOC))
		return -EINVAL;
	return (int)adc->DR;
}

int adc_single_conversion(uint8_t adc_num, uint8_t channel, uint8_t sample_rate)
{
	int res;

	res = adc_init(adc_num, channel, sample_rate);
	if (res)
		return res;

	do {
		res = adc_read(adc_num);
	} while (res < 0);

	return res;
}

void adc_enable_interrupt(uint8_t adc_num,
	void (*handler)(uint8_t adc_num, uint16_t value))
{
#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && \
	!defined (STM32F10X_HD_VL)
	ADC_TypeDef *adc = adc_num == 1 ? ADC1 : ADC2;

	if (adc_num < 1 || adc_num > 2)
		return;
	adc_isr[adc_num - 1].handler = handler;
	adc->CR1 = ADC_CR1_EOCIE;
	NVIC_EnableIRQ(ADC1_2_IRQn);
#else
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */
}

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && \
	!defined (STM32F10X_HD_VL)
void ADC1_2_IRQHandler(void)
{
	if (ADC1->SR & ADC_SR_EOC)
		adc_isr[0].handler(1, ADC1->DR);
	if (ADC2->SR & ADC_SR_EOC)
		adc_isr[1].handler(2, ADC2->DR);
}
#else
void ADC1_IRQn(void)
{
	handler(1, ADC1->DR);
}
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */
