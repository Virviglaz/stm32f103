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

#include "tim.h"
#include "rcc.h"
#include <errno.h>

static struct {
	void (*handler)(uint8_t tim, void *data);
	void *data;
} tim_isr[17];

static TIM_TypeDef *get_tim_base(uint8_t tim)
{
	static const TIM_TypeDef *tims[] = { 0,
		TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM8, TIM9, TIM10,
		TIM11, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17 };

	return tim >= (sizeof(tims) / sizeof(void *)) ? 0 : (void *)tims[tim];
}

int timer_init(uint8_t tim, uint32_t freq, uint32_t period)
{
	TIM_TypeDef *base = get_tim_base(tim);
	struct system_clock_t *clock = get_clocks();
	uint32_t clock_freq, prc, arr;

	if (!base)
		return -EINVAL;

	switch (tim) {
	case 1:
		clock_freq = clock->apb2_freq;
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
		break;
	case 2:
		clock_freq = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		break;
	case 3:
		clock_freq = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		break;
#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
	case 4:
		clock_freq = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
		break;
#endif /* !STM32F10X_LD !STM32F10X_LD_VL */
#if defined (STM32F10X_HD) || defined  (STM32F10X_CL)
	case 5:
		clock_freq = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
		break;
	case 6:
		clock_freq = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
		break;
	case 7:
		clock_freq = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
		break;
#endif /* STM32F10X_HD STM32F10X_CL */
#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
	case 8:
		clock_freq = clock->apb2_freq;
		RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
		break;
#endif /* STM32F10X_HD STM32F10X_XL */
#ifdef STM32F10X_XL
	case 9:
		clock_freq = clock->apb2_freq;
		RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
		break;
	case 10:
		clock_freq = clock->apb2_freq;
		RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
		break;
	case 11:
		clock_freq = clock->apb2_freq;
		RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
		break;
#endif /* STM32F10X_XL */
#ifdef STM32F10X_HD_VL
	case 12:
		clock_freq = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;
		break;
	case 13:
		clock_freq = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
		break;
	case 14:
		clock_freq = clock->apb1_freq;
		RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
		break;
#endif /* STM32F10X_HD_VL */
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || \
	defined (STM32F10X_HD_VL)
	case 15:
		clock_freq = clock->apb2_freq;
		RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
		break;
	case 16:
		clock_freq = clock->apb2_freq;
		RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
		break;
	case 17:
		clock_freq = clock->apb2_freq;
		RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
		break;
#endif /* STM32F10X_LD_VL STM32F10X_MD_VL STM32F10X_HD_VL */
	default:
		return -EINVAL;
	};

	if (freq) {
		prc = 0;
		do {
			arr = clock_freq / freq / ++prc;
		} while (arr > 0xFFFF);
	} else {
		prc = 0xFFFF;
		do {
			arr = clock_freq * period / --prc;
		} while (arr > 0xFFFF);
	}

	base->PSC = prc - 1;
	base->ARR = arr;
	base->EGR = TIM_EGR_UG;
	base->CR1 = TIM_CR1_CEN;

	return 0;
}

int timer_enable(uint8_t tim, bool state)
{
	TIM_TypeDef *base = get_tim_base(tim);

	if (!base)
		return -EINVAL;

	base->CR1 = state ? TIM_CR1_CEN : 0;

	return 0;
}

int timer_set_period(uint8_t tim, uint16_t period)
{
	TIM_TypeDef *base = get_tim_base(tim);

	if (!base)
		return -EINVAL;

	base->ARR = period;
	base->EGR = TIM_EGR_UG;

	return 0;
}

int timer_pwm_enable(uint8_t tim, uint8_t ch, u16 duty)
{
	TIM_TypeDef *base = get_tim_base(tim);

	if (!base)
		return -EINVAL;

	switch (ch) {
	case 1:
		base->CCMR1 = (7 << 4) | TIM_CCMR1_OC1PE;
		base->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;
		base->CCR1 = duty;
		break;
	case 2:
		base->CCMR1 = (7 << 4) | TIM_CCMR1_OC2PE;
		base->CCER = TIM_CCER_CC2E | TIM_CCER_CC2P;
		base->CCR2 = duty;
		break;
	case 3:
		base->CCMR2 = (7 << 4) | TIM_CCMR2_OC3PE;
		base->CCER = TIM_CCER_CC3E | TIM_CCER_CC3P;
		base->CCR3 = duty;
		break;
	case 4:
		base->CCMR2 = (7 << 4) | TIM_CCMR2_OC4PE;
		base->CCER = TIM_CCER_CC4E | TIM_CCER_CC4P;
		base->CCR4 = duty;
		break;
	default:
		return -EINVAL;
	}

	base->RCR = 0;
	base->BDTR = TIM_BDTR_MOE;
	base->EGR = TIM_EGR_UG;
	base->CR1 = TIM_CR1_CEN;

	return 0;
}

int timer_pwm_set_duty(uint8_t tim, uint8_t ch, u16 duty)
{
	TIM_TypeDef *base = get_tim_base(tim);

	if (!base)
		return -EINVAL;

	switch (ch) {
	case 1:
		base->CCR1 = duty;
		break;
	case 2:
		base->CCR2 = duty;
		break;
	case 3:
		base->CCR3 = duty;
		break;
	case 4:
		base->CCR4 = duty;
		break;
	default:
		return -EINVAL;
	}

	base->EGR = TIM_EGR_UG;

	return 0;
}

int timer_enable_interrupt(uint8_t tim,
	void (*handler)(uint8_t tim, void *data), void *data)
{
	TIM_TypeDef *base = get_tim_base(tim);

	if (!base)
		return -EINVAL;

	base->DIER = TIM_DIER_UIE;

	tim_isr[tim - 1].handler = handler;
	tim_isr[tim - 1].data = data;

	switch (tim) {
	case 1:
#if defined(STM32F10X_LD) || defined(STM32F10X_MD) || defined(STM32F10X_HD) || \
	defined(STM32F10X_CL)
		NVIC_EnableIRQ(TIM1_UP_IRQn);
		break;
#endif /* STM32F10X_LD STM32F10X_MD STM32F10X_HD STM32F10X_CL */
	case 2:
		NVIC_EnableIRQ(TIM2_IRQn);
		break;
	case 3:
		NVIC_EnableIRQ(TIM3_IRQn);
		break;
	case 4:
		NVIC_EnableIRQ(TIM4_IRQn);
		break;
#if defined(STM32F10X_HD) || defined(STM32F10X_HD_VL) || defined(STM32F10X_XL) \
	|| defined(STM32F10X_CL)
	case 5:
		NVIC_EnableIRQ(TIM5_IRQn);
		break;
#endif /* STM32F10X_HD STM32F10X_HD_VL STM32F10X_XL STM32F10X_CL */
#if defined(STM32F10X_HD) || defined(STM32F10X_XL) || defined(STM32F10X_CL)
	case 6:
		NVIC_EnableIRQ(TIM6_IRQn);
		break;
	case 7:
		NVIC_EnableIRQ(TIM7_IRQn);
		break;
#endif /* STM32F10X_HD STM32F10X_XL STM32F10X_CL */
#ifdef STM32F10X_XL
	case 8:
		NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
		break;
	case 9:
		NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
		break;
	case 10:
		NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		break;
	case 11:
		NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
		break;
	case 12:
		NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
		break;
	case 13:
		NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
		break;
	case 14:
		NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
		break;
	case 15:
		NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
		break;
	case 16:
		NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
		break;
	case 17:
		NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

int set_timer_period(uint8_t tim, uint16_t period)
{
	TIM_TypeDef *base = get_tim_base(tim);

	if (!base)
		return -EINVAL;

	base->ARR = period;
	base->EGR = TIM_EGR_UG;

	return 0;
}

/* INTERRUPT VECTORS */
void TIM1_UP_IRQHandler(void)
{
	tim_isr[0].handler(1, tim_isr[0].data);
	TIM1->SR = 0;
}

void TIM2_IRQHandler(void)
{
	tim_isr[1].handler(2, tim_isr[1].data);
	TIM2->SR = 0;
}

void TIM3_IRQHandler(void)
{
	tim_isr[2].handler(3, tim_isr[2].data);
	TIM3->SR = 0;
}

void TIM4_IRQHandler(void)
{
	tim_isr[3].handler(4, tim_isr[3].data);
	TIM4->SR = 0;
}

void TIM5_IRQHandler(void)
{
	tim_isr[4].handler(5, tim_isr[4].data);
	TIM5->SR = 0;
}

void TIM6_IRQHandler(void)
{
	tim_isr[5].handler(6, tim_isr[5].data);
	TIM6->SR = 0;
}

void TIM7_IRQHandler(void)
{
	tim_isr[6].handler(7, tim_isr[6].data);
	TIM7->SR = 0;
}
	
#ifdef STM32F10X_XL
void TIM8_IRQHandler(void)
{
	tim_isr[7].handler(8, tim_isr[7].data);
	TIM8->SR = 0;
}


void TIM9_IRQHandler(void)
{
	tim_isr[8].handler(9, tim_isr[8].data);
	TIM9->SR = 0;
}

void TIM10_IRQHandler(void)
{
	tim_isr[9].handler(10, tim_isr[9].data);
	TIM10->SR = 0;
}

void TIM11_IRQHandler(void)
{
	tim_isr[10].handler(11, tim_isr[10].data);
	TIM11->SR = 0;
}

void TIM12_IRQHandler(void)
{
	tim_isr[11].handler(12, tim_isr[11].data);
	TIM12->SR = 0;
}

void TIM13_IRQHandler(void)
{
	tim_isr[12].handler(13, tim_isr[12].data);
	TIM13->SR = 0;
}

void TIM14_IRQHandler(void)
{
	tim_isr[13].handler(14, tim_isr[13].data);
	TIM14->SR = 0;
}

void TIM15_IRQHandler(void)
{
	tim_isr[14].handler(15, tim_isr[14].data);
	TIM15->SR = 0;
}

void TIM16_IRQHandler(void)
{
	tim_isr[15].handler(16, tim_isr[15].data);
	TIM16->SR = 0;
}

void TIM17_IRQHandler(void)
{
	tim_isr[16].handler(17, tim_isr[16].data);
	TIM17->SR = 0;
}

#endif /* STM32F10X_XL */
