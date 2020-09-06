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

#ifndef __RCC_H__
#define __RCC_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"
#include <stdint.h>

enum clock_t {
	NO_CLOCK,
	HSI_CLOCK, /* High speed internal clock */
	HSE_CLOCK, /* High speed external clock */
	HSE_CLOCK_DIV2, /* High speed external divided by 2 */
	HSE_CLOCK_DIV128, /* High speed external divided by 128 */
	LSI_CLOCK, /* Low speed internal clock */
	LSE_CLOCK, /* Low speed external clock */
	PLL_CLOCK, /* PLL clock source */
	INV_CLOCK, /* Invalid value */
};

struct system_clock_t {
	uint32_t cpu_freq;
	uint32_t ahb_freq;
	uint32_t apb1_freq;
	uint32_t apb2_freq;
};

enum clock_t hsi_enable(void);
enum clock_t hse_enable(void);
enum clock_t get_system_clock(void);
enum clock_t set_system_clock(enum clock_t clk);
enum clock_t pll_enable(uint8_t pll, enum clock_t source);
enum clock_t lse_enable(void);
enum clock_t get_rtc_clock(void);
enum clock_t set_rtc_clock(enum clock_t source);

static inline enum clock_t get_pll_source(void)
{
	return RCC->CFGR & RCC_CFGR_PLLSRC ? HSE_CLOCK : HSI_CLOCK;
}

void set_hse_freq(uint32_t freq);
struct system_clock_t *get_clocks(void);

#ifdef __cplusplus
}
#endif

#endif /* __RCC_H__ */
