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

#include <stm32f10x.h>
#include "crc.h"
#include "dma.h"
#include <stdbool.h>

uint32_t crc32(uint8_t *buf, uint32_t size)
{
	RCC->AHBENR |= RCC_AHBENR_CRCEN;
	CRC->CR = CRC_CR_RESET;

	while (size--)
		CRC->DR = *buf++;

	return CRC->DR;
}

static struct {
	void (*handler)(void *data, uint32_t crc);
	void *data;
	DMA_Channel_TypeDef *dma_ch;
	uint32_t res;
	bool ready;
} isr;

static void crc_dma_handler(void *data)
{
	dma_release(isr.dma_ch);
	isr.res = CRC->DR;
	if (isr.handler)
		isr.handler(isr.data, isr.res);
	isr.ready = true;
}

uint32_t crc32dma8(uint8_t *buf, uint16_t size,
	void (*handler)(void *data, uint32_t crc), void *data)
{
	isr.handler = handler;
	isr.data = data;
	isr.ready = false;

	isr.dma_ch = get_dma_ch(0, crc_dma_handler, data);;
	if (!isr.dma_ch)
		return crc32(buf, size);

	RCC->AHBENR |= RCC_AHBENR_CRCEN;
	CRC->CR = CRC_CR_RESET;

	isr.dma_ch->CMAR = (uint32_t)buf;
	isr.dma_ch->CPAR = (uint32_t)&CRC->DR;
	isr.dma_ch->CNDTR = size;
	isr.dma_ch->CCR = DMA_CCR1_TCIE | DMA_CCR1_MINC | DMA_CCR1_MEM2MEM | \
		DMA_CCR1_DIR | DMA_CCR1_EN;

	if (!handler) {
		while (!isr.ready) { }
		return isr.res;
	}

	return 0;
}

#ifdef FREERTOS

#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t lock;
static SemaphoreHandle_t done;
static uint32_t result;

static void rtos_handler(void *data, uint32_t res)
{
	(void)data;
	result = res;
	xSemaphoreGiveFromISR(done, NULL);
}

uint32_t crc32rtos(uint8_t *buf, uint16_t size)
{
	if (!lock) { /* Race condition here when preemption is enabled. */
		lock = xSemaphoreCreateMutex();
		done = xSemaphoreCreateBinary();
	}

	xSemaphoreTake(lock, portMAX_DELAY);

	crc32dma8(buf, size, rtos_handler, NULL);

	xSemaphoreTake(done, portMAX_DELAY);

	xSemaphoreGive(lock);

	return result;
}

#endif /* FREERTOS */
