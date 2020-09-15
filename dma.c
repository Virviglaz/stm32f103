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

#include "dma.h"

#define NOF_DMA_CHANNELS	7

static DMA_Channel_TypeDef *dev1_chs[] = {
	DMA1_Channel1,
	DMA1_Channel2,
	DMA1_Channel3,
	DMA1_Channel4,
	DMA1_Channel5,
	DMA1_Channel6,
	DMA1_Channel7,
};

static struct isr_t {
	void (*handler)(void *data);
	void *data;
} isrs[NOF_DMA_CHANNELS] = { 0 };

DMA_Channel_TypeDef *get_dma_ch(uint8_t channel,
	void (*handler)(void *data), void *data)
{
	uint8_t i;

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	if (channel > NOF_DMA_CHANNELS)
		return 0;

	/* find specified channel */
	if (channel) {
		DMA_Channel_TypeDef *ch;
		channel--;
		ch = dev1_chs[channel];

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
	for (i = 0; i != NOF_DMA_CHANNELS; i++) {
		DMA_Channel_TypeDef *ch = dev1_chs[i];
		if (ch->CCR & DMA_CCR1_EN)
			continue;

		return ch;
	}

	/* no channels found */
	return 0;
}

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
