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

#include "dma.h"
#include <string.h>

#if defined(STM32F10X_HD) || defined(STM32F10X_CL) || defined(STM32F10X_LD_VL) \
	|| defined(STM32F10X_MD_VL) || defined(STM32F10X_HD_VL)
#define NOF_DMA_CHANNELS	(7 + 4) /* DMA1 + DMA2 */
#else
#define NOF_DMA_CHANNELS	(7 + 0) /* DMA2 not used */
#endif

static struct isr_t {
	void (*handler)(void *data);
	void *data;
} isrs[NOF_DMA_CHANNELS] = { 0 };

DMA_Channel_TypeDef *get_dma_ch(uint8_t channel,
	void (*handler)(void *data), void *data)
{
	const DMA_Channel_TypeDef *dev1_chs[] = {
		DMA1_Channel1,
		DMA1_Channel2,
		DMA1_Channel3,
		DMA1_Channel4,
		DMA1_Channel5,
		DMA1_Channel6,
		DMA1_Channel7,
	};
	uint8_t i;

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	if (channel > sizeof(dev1_chs) / sizeof(dev1_chs[0]))
		return 0;

	/* find specified channel */
	if (channel) {
		DMA_Channel_TypeDef *ch;
		channel--;
		ch = (void *)dev1_chs[channel];

		/* check channel is free */
		if (ch->CCR & DMA_CCR1_EN)
			return 0;

		/* assign handler */
		isrs[channel].handler = handler;
		isrs[channel].data = data;

		NVIC_EnableIRQ((enum IRQn)(DMA1_Channel1_IRQn + channel));

		return ch;
	}

	/* find first free channel */
	for (i = 0; i != sizeof(dev1_chs) / sizeof(dev1_chs[0]); i++) {
		DMA_Channel_TypeDef *ch = (void *)dev1_chs[i];
		if (ch->CCR & DMA_CCR1_EN)
			continue;

		/* assign handler */
		isrs[i].handler = handler;
		isrs[i].data = data;

		NVIC_EnableIRQ((enum IRQn)(DMA1_Channel1_IRQn + i));
		return ch;
	}

	/* no channels found */
	return 0;
}

#if defined(STM32F10X_HD) || defined(STM32F10X_CL) || defined(STM32F10X_LD_VL) \
	|| defined(STM32F10X_HD_VL)

DMA_Channel_TypeDef *get_dma2_ch(uint8_t channel,
	void (*handler)(void *data), void *data)
{
	const DMA_Channel_TypeDef *dev2_chs[] = {
		DMA2_Channel1,
		DMA2_Channel2,
		DMA2_Channel3,
		DMA2_Channel4,
		/* DMA2_Channel5: NOT USED FOR NOW */
	};
	DMA_Channel_TypeDef *ch;

	RCC->AHBENR |= RCC_AHBENR_DMA2EN;

	if (!channel || channel > sizeof(dev2_chs) / sizeof(dev2_chs[0]))
		return 0;

	channel--;
	ch = (void *)dev2_chs[channel];

	/* check channel is free */
	if (ch->CCR & DMA_CCR1_EN)
		return 0;

	NVIC_EnableIRQ((enum IRQn)(DMA2_Channel1_IRQn + channel));

	channel += NOF_DMA_CHANNELS - ARRAY_SIZE(dev2_chs);

	/* assign handler */
	isrs[channel].handler = handler;
	isrs[channel].data = data;

	return ch;
}

void DMA2_Channel1_IRQHandler(void)
{
	DMA2->IFCR = DMA_IFCR_CGIF1;
	dma_release(DMA2_Channel1);
	isrs[7].handler(isrs[7].data);
}

void DMA2_Channel2_IRQHandler(void)
{
	DMA2->IFCR = DMA_IFCR_CGIF2;
	dma_release(DMA2_Channel2);
	isrs[8].handler(isrs[8].data);
}

void DMA2_Channel3_IRQHandler(void)
{
	DMA2->IFCR = DMA_IFCR_CGIF3;
	dma_release(DMA2_Channel3);
	isrs[9].handler(isrs[9].data);
}

void DMA2_Channel4_5_IRQHandler(void)
{
	DMA2->IFCR = DMA_IFCR_CGIF4;
	dma_release(DMA2_Channel4);
	isrs[10].handler(isrs[10].data);
}

#endif /* DMA2 */

void DMA1_Channel1_IRQHandler(void)
{
	DMA1->IFCR = DMA_IFCR_CGIF1;
	dma_release(DMA1_Channel1);
	isrs[0].handler(isrs[0].data);
}

void DMA1_Channel2_IRQHandler(void)
{
	DMA1->IFCR = DMA_IFCR_CGIF2;
	dma_release(DMA1_Channel2);
	isrs[1].handler(isrs[1].data);
}

void DMA1_Channel3_IRQHandler(void)
{
	DMA1->IFCR = DMA_IFCR_CGIF3;
	dma_release(DMA1_Channel3);
	isrs[2].handler(isrs[2].data);
}

void DMA1_Channel4_IRQHandler(void)
{
	DMA1->IFCR = DMA_IFCR_CGIF4;
	dma_release(DMA1_Channel4);
	isrs[3].handler(isrs[3].data);
}

void DMA1_Channel5_IRQHandler(void)
{
	DMA1->IFCR = DMA_IFCR_CGIF5;
	dma_release(DMA1_Channel5);
	isrs[4].handler(isrs[4].data);
}

void DMA1_Channel6_IRQHandler(void)
{
	DMA1->IFCR = DMA_IFCR_CGIF6;
	dma_release(DMA1_Channel6);
	isrs[5].handler(isrs[5].data);
}

void DMA1_Channel7_IRQHandler(void)
{	DMA1->IFCR = DMA_IFCR_CGIF7;
	dma_release(DMA1_Channel7);
	isrs[6].handler(isrs[6].data);
}

#ifndef FREERTOS

static volatile int busy;
static void handler(void *data)
{
	busy = 0;
}

static void memcpy_dma(void *dst, const void *src, uint16_t size, uint16_t flag)
{
	DMA_Channel_TypeDef *ch;

	ch = get_dma_ch(0, handler, 0);

	if (!ch) /* failed to get channel, use cpu instead */
		memcpy(dst, src, size);
	else {
		busy = !0;
		ch->CNDTR = size;
		ch->CMAR = (uint32_t)src;
		ch->CPAR = (uint32_t)dst;
		ch->CCR = DMA_CCR1_MEM2MEM | DMA_CCR1_MINC | DMA_CCR1_PINC | \
			DMA_CCR1_DIR | DMA_CCR1_TCIE | DMA_CCR1_EN | flag;
		while(busy) { }
		dma_release(ch);
	}
}

#else /* FREERTOS */

static void handler(void *data)
{
	rtos_schedule_isr(data);
}

static void memcpy_dma(void *dst, const void *src, uint16_t size, uint16_t flag)
{
	static SemaphoreHandle_t mutex = 0;
	DMA_Channel_TypeDef *ch;
	TaskHandle_t handle;

	if (!mutex)
		mutex = xSemaphoreCreateMutex();

	xSemaphoreTake(mutex, portMAX_DELAY);

	handle = xTaskGetCurrentTaskHandle();

	ch = get_dma_ch(0, handler, handle);

	if (!ch) /* failed to get channel, use cpu instead */
		memcpy(dst, src, size);
	else {
		ch->CNDTR = size;
		ch->CMAR = (uint32_t)src;
		ch->CPAR = (uint32_t)dst;
		ch->CCR = DMA_CCR1_MEM2MEM | DMA_CCR1_MINC | DMA_CCR1_PINC | \
			DMA_CCR1_DIR | DMA_CCR1_TCIE | DMA_CCR1_EN | flag;
		vTaskSuspend(handle);
		dma_release(ch);
	}

	xSemaphoreGive(mutex);
}

#endif /* FREERTOS */

void memcpy_dma8(uint8_t *dst, const uint8_t *src, uint16_t size)
{
	memcpy_dma(dst, src, size, 0);
}

void memcpy_dma16(uint16_t *dst, const uint16_t *src, uint16_t size)
{
	memcpy_dma(dst, src, size, DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0);
}

void memcpy_dma32(uint32_t *dst, const uint32_t *src, uint16_t size)
{
	memcpy_dma(dst, src, size, DMA_CCR1_PSIZE_1 | DMA_CCR1_MSIZE_1);
}
