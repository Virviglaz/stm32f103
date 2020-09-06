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
	*((__IO uint8_t *)&SPIx->DR) = value;

	/* Wait to receive a byte */
	while (!(SPIx->SR & SPI_SR_RXNE));

	return *((__IO uint8_t *)&SPIx->DR);
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

void spi_init(SPI_TypeDef *SPIx, enum spi_dir dir,
	enum spi_clockdiv div, enum spi_clockmode mode)
{
	if (SPIx == SPI1)
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	if (SPIx == SPI2)
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	SPIx->CR2 = 0;
	SPIx->CR1 = (uint16_t)dir | (uint16_t)div | (uint16_t)mode \
		| SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_MSTR | SPI_CR1_SPE;
}
