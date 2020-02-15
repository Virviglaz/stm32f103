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

#include "stm32f103_rcc.h"

enum clock_t hsi_enable(void)
{
	RCC->CR |= RCC_CR_HSION;

	while (1) {
		if (RCC->CR & RCC_CR_HSIRDY)
			return HSI_CLOCK;
	};
}

enum clock_t hse_enable(void)
{
	RCC->CR |= RCC_CR_CSSON | RCC_CR_HSEON;

	while (1) {
		if (RCC->CR & RCC_CR_CSSON)
			return HSE_CLOCK;
	};
}

enum clock_t get_system_clock(void)
{
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI)
		return HSI_CLOCK;
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE)
		return HSE_CLOCK;
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
		return PLL_CLOCK;
	return INV_CLOCK;
}

enum clock_t set_system_clock(enum clock_t clk)
{
	switch (clk) {
	case HSI_CLOCK:
		RCC->CFGR &= ~RCC_CFGR_SW;
		break;
	case HSE_CLOCK:
		RCC->CFGR &= ~RCC_CFGR_SW;
		RCC->CFGR |= RCC_CFGR_SW_HSE;
		break;
	case PLL_CLOCK:
		RCC->CFGR &= ~RCC_CFGR_SW;
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		break;
	default:
		return INV_CLOCK;
	};

	return get_system_clock();
}

enum clock_t pll_enable(uint8_t pll, enum clock_t source)
{
	if (pll < 2 || pll > 16)
		return INV_CLOCK;

	switch (source) {
	case HSE_CLOCK:
		if (hse_enable() != HSE_CLOCK)
			return INV_CLOCK;
		RCC->CFGR &= ~RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC;
		RCC->CFGR |= RCC_CFGR_PLLSRC;
		break;
	case HSE_CLOCK_DIV2:
		if (hse_enable() != HSE_CLOCK)
			return INV_CLOCK;
		RCC->CFGR |= RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC;
		break;
	case HSI_CLOCK:
		if (hsi_enable() != HSI_CLOCK)
			return INV_CLOCK;
		RCC->CFGR &= ~RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC;
		break;
	default:
		return INV_CLOCK;
	};

	RCC->CFGR &= ~RCC_CFGR_PLLMULL;
	RCC->CFGR |= pll << 18;
	RCC->CR |=  RCC_CR_CSSON;

	while (1) {
		if (RCC->CR & RCC_CR_PLLRDY)
			return PLL_CLOCK;
	};
}

enum clock_t lse_enable(void)
{
	RCC->BDCR |= RCC_BDCR_LSEON;

	while (1) {
		if (RCC->BDCR & RCC_BDCR_LSERDY)
			break;
	};

	return LSE_CLOCK;
}

enum clock_t get_rtc_clock(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN;

	if ((RCC->BDCR & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_NOCLOCK)
		return NO_CLOCK;
	if ((RCC->BDCR & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_LSE)
		return LSE_CLOCK;
	if ((RCC->BDCR & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_LSI)
		return LSI_CLOCK;
	if ((RCC->BDCR & RCC_BDCR_RTCSEL) == RCC_BDCR_RTCSEL_HSE)
		return HSE_CLOCK_DIV128;
	return INV_CLOCK;
}

enum clock_t set_rtc_clock(enum clock_t source)
{
	if (get_rtc_clock() == source)
		return source;

	/* Reset the backup domain */
	RCC->BDCR |= RCC_BDCR_BDRST;
	RCC->BDCR &= ~RCC_BDCR_BDRST;

	/* Enable access to backup domain */
	PWR->CR |= PWR_CR_DBP;

	switch (source) {
	case NO_CLOCK:
		RCC->BDCR &= ~RCC_BDCR_RTCSEL;
		break;
	case LSE_CLOCK:
		if (lse_enable() != LSE_CLOCK)
			return INV_CLOCK;
		RCC->BDCR &= ~RCC_BDCR_RTCSEL;
		RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;
		break;
	case LSI_CLOCK:
		RCC->BDCR &= ~RCC_BDCR_RTCSEL;
		RCC->BDCR |= RCC_BDCR_RTCSEL_LSI;
		break;
	case HSE_CLOCK_DIV128:
		RCC->BDCR &= ~RCC_BDCR_RTCSEL;
		RCC->BDCR |= RCC_BDCR_RTCSEL_HSE;
		break;
	default:
		PWR->CR &= ~PWR_CR_DBP;
		return INV_CLOCK;
	};

	/* Enable RTC */
	RCC->BDCR |= RCC_BDCR_RTCEN;

	/* Disable access to backup domain */
	if (source != HSE_CLOCK_DIV128)
		/* Note: for HSE/128 this bit should remains enabled */
		PWR->CR &= ~PWR_CR_DBP;

	return get_rtc_clock();
}
