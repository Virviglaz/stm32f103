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

#include "spi.h"
#include "rcc.h"
#include "gpio.h"

inline static void spi_select(GPIO_TypeDef *GPIOx, uint16_t PINx)
{	
	GPIOx->BRR = PINx;
}

inline static void spi_release(SPI_TypeDef *SPIx,
	GPIO_TypeDef *GPIOx, uint16_t PINx)
{
	while (SPIx->SR & SPI_SR_BSY);
	GPIOx->BSRR = PINx;	
}

uint8_t spi_read_byte(SPI_TypeDef *SPIx, uint8_t value)
{
	/* Loop while DR register in not empty */
	while (!(SPIx->SR & SPI_SR_TXE));

	/* Send byte through the SPI peripheral */
	SPIx->DR = value;

	/* Wait to receive a byte */
	while (!(SPIx->SR & SPI_SR_RXNE));
	//while (SPIx->SR & SPI_SR_BSY);

	return SPIx->DR;
}

uint8_t spi_write_reg(SPI_TypeDef *SPIx, GPIO_TypeDef *GPIOx, uint16_t PINx,
	uint8_t reg, uint8_t *buf, uint16_t size)
{
	uint8_t ret;

	spi_select(GPIOx, PINx);

	ret = spi_read_byte(SPIx, reg);

	while(size--)
		spi_read_byte(SPIx, *buf++);

	spi_release(SPIx, GPIOx, PINx);

	return ret;
}

uint8_t spi_read_reg(SPI_TypeDef *SPIx, GPIO_TypeDef *GPIOx, uint16_t PINx,
	uint8_t reg, uint8_t *buf, uint16_t size)
{
	uint8_t ret;

	spi_select(GPIOx, PINx);	
	ret = spi_read_byte(SPIx, reg);

	while(size--)
		*buf++ = spi_read_byte(SPIx, 0x00);

	spi_release(SPIx, GPIOx, PINx);

	return ret;
}

static inline uint16_t calc_clock_div(uint32_t bus_freq, uint32_t expected_freq)
{
	uint32_t freq;
	uint16_t div = 1;
	uint16_t n = 0;

	do {
		div <<= 1;
		n++;
		freq = bus_freq / div;
	} while (freq > expected_freq);

	return n - 1;
}

void spi_init(SPI_TypeDef *SPIx, uint32_t freq, bool idle_clock_high)
{
	uint16_t clock_div;
	uint16_t clock_mode = idle_clock_high ? SPI_CR1_CPHA | SPI_CR1_CPOL : 0;
	struct system_clock_t *clocks = get_clocks();

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	if (SPIx == SPI1) {
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		//gpio_input_init(PA6, PULL_UP_INPUT);
		gpio_output_init(PA5, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		gpio_output_init(PA6, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		gpio_output_init(PA7, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		clock_div = calc_clock_div(clocks->apb2_freq, freq);
	} else if (SPIx == SPI2) {
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		//gpio_input_init(PB4, PULL_UP_INPUT);
		gpio_output_init(PB13, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		gpio_output_init(PB14, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		gpio_output_init(PB15, PUSHPULL_ALT_OUTPUT, GPIO_FREQ_50MHz);
		clock_div = calc_clock_div(clocks->apb1_freq, freq);
	} else
		return;

	SPIx->CR2 = 0;
	SPIx->CR1 = (clock_div << 3) | clock_mode | SPI_CR1_SSI | \
		SPI_CR1_SSM | SPI_CR1_MSTR | SPI_CR1_SPE;
}