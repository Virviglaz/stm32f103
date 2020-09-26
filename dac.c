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

#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "delay.h"
#include <errno.h>

struct dac_cr_t {
	uint8_t : 3;
	bool dma_enable	: 1;
	uint8_t mamp	: 4;
	uint8_t wave	: 2;
	uint8_t tsel	: 2;
	bool ten	: 1;
	bool boff	: 1;
	bool enable	: 1;
};

static volatile struct {
	bool done;
	void (*handler)(void *data);
	void *data;
} isr = { 0 };

static void handler(void *data)
{
	if (isr.handler)
		isr.handler(isr.data);
	isr.done = true;
}

int dac_start_12bit(uint8_t ch, uint16_t *src, uint16_t size,
	enum trig_t trig, uint32_t freq)
{
	const uint8_t dma_ch_num[] = { 3, 4 };
	const uint32_t dac_dst[] = {
		(uint32_t)&DAC->DHR12R1, (uint32_t)&DAC->DHR12R2 };
	const uint8_t timer_sources[] = { 6, 3, 7, 5, 2, 4 };
	DMA_Channel_TypeDef *dma_ch;
	uint16_t cr_val = (trig << 3) | \
		DAC_CR_DMAEN1 | DAC_CR_TEN1 | DAC_CR_EN1;
	int ret;
	uint8_t tim;

	RCC->APB1ENR |= RCC_APB1ENR_DACEN;

	if (!ch || ch > 2 || trig > TIM4_TRGO)
		return -EINVAL;

	ch--;

	/* setup the timer to get TRGO event */
	tim = timer_sources[trig];
	ret = timer_init(tim, freq, 0);
	if (ret)
		return ret;

	timer_enable_update_event(tim, true);
	isr.done = false;

	DAC->CR = cr_val << (ch ? 16 : 0);

	dma_ch = get_dma2_ch(dma_ch_num[ch], handler, 0);
	if (!dma_ch)
		return -EILSEQ;

	dma_ch->CMAR = (uint32_t)src;
	dma_ch->CPAR = dac_dst[ch];
	dma_ch->CNDTR = size;
	dma_ch->CCR = DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0 | DMA_CCR1_DIR | \
		DMA_CCR1_EN | DMA_CCR1_TCIE | DMA_CCR1_MINC;

	if (!isr.handler) /* if no handler wait for finish */
		while (isr.done == false) { }

	timer_enable_update_event(tim, false);

	return 0;
}

void dac_enable_interrupt(void (*handler)(void *data), void *data)
{
	isr.handler = handler;
	isr.data = data;
}

#ifdef FREERTOS

static void handler(void *data)
{
	rtos_schedule_isr(private_data);
}

int dac_start_12bit_rtos(uint8_t ch, uint16_t *src, uint16_t size,
	enum trig_t trig, uint32_t freq)
{
	static SemaphoreHandle_t mutex = 0;
	TaskHandle_t handle;
	int res;

	if (!mutex)
		mutex = xSemaphoreCreateMutex();

	xSemaphoreTake(mutex, portMAX_DELAY);

	handle = xTaskGetCurrentTaskHandle();

	dac_enable_interrupt(handler);

	res = dac_start_12bit(ch, src, sizeof, trig, freq);
	if (!res)
		vTaskSuspend(handle);

	dac_enable_interrupt(0);

	xSemaphoreGive(mutex);

	return res;
}

#endif /* FREERTOS */
