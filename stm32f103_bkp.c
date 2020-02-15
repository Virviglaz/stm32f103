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

#include "stm32f103_bkp.h"

static uint16_t *get_dr_reg(uint8_t n)
{
	static const uint16_t *dr_regs[] = {
		(void *)&BKP->DR1,
		(void *)&BKP->DR2,
		(void *)&BKP->DR3,
		(void *)&BKP->DR4,
		(void *)&BKP->DR5,
		(void *)&BKP->DR6,
		(void *)&BKP->DR7,
		(void *)&BKP->DR8,
		(void *)&BKP->DR9,
		(void *)&BKP->DR10,
		(void *)&BKP->DR11,
		(void *)&BKP->DR12,
		(void *)&BKP->DR13,
		(void *)&BKP->DR14,
		(void *)&BKP->DR15,
		(void *)&BKP->DR16,
		(void *)&BKP->DR17,
		(void *)&BKP->DR18,
		(void *)&BKP->DR19,
		(void *)&BKP->DR20,
		(void *)&BKP->DR21,
		(void *)&BKP->DR22,
		(void *)&BKP->DR23,
		(void *)&BKP->DR24,
		(void *)&BKP->DR25,
		(void *)&BKP->DR26,
		(void *)&BKP->DR27,
		(void *)&BKP->DR28,
		(void *)&BKP->DR29,
		(void *)&BKP->DR30,
		(void *)&BKP->DR31,
		(void *)&BKP->DR32,
		(void *)&BKP->DR33,
		(void *)&BKP->DR34,
		(void *)&BKP->DR35,
		(void *)&BKP->DR36,
		(void *)&BKP->DR37,
		(void *)&BKP->DR38,
		(void *)&BKP->DR39,
		(void *)&BKP->DR40,
		(void *)&BKP->DR41,
		(void *)&BKP->DR42,
	};

	RCC->APB1ENR |= RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN;

	return n < ARRAY_SIZE(dr_regs) ? (uint16_t *)dr_regs[n] : 0;
}

void bkp_write(uint8_t reg_num, uint16_t value)
{
	uint16_t *reg = get_dr_reg(reg_num - 1);

	PWR->CR |= PWR_CR_DBP;

	if (reg)
		*reg = value;

	PWR->CR &= ~PWR_CR_DBP;
}

uint16_t bkp_read(uint8_t reg_num)
{
	uint16_t *reg = get_dr_reg(reg_num - 1);

	return reg ? *reg : 0;
}
