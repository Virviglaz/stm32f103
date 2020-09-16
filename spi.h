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
 
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include "rtos.h"

/**
  * @brief  Initialize SPI master.
  * @param  spi_num: Can be 1 or 2 to select the SPI peripheral.
  * @param  freq: Clock frequency in Hz.
  *
  * @retval 0 if success.
  */
int spi_init(uint8_t spi_num, uint32_t freq, bool clock_high);

/**
  * @brief  Write a sequence to register (waiting call).
  * @param  spi_num: Can be 1 or 2 to select the SPI peripheral.
  * @param  gpio: Chip select gpio.
  * @param  pin: Chip select pin.
  * @param  reg: Register number to write to.
  * @param  data: Pointer where the data is stored.
  * @param  size: Amount of bytes to write.
  *
  * @retval 0 if success.
  */
int spi_write_reg(uint8_t spi_num, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size);

/**
  * @brief  Read a sequence from register (waiting call).
  * @param  spi_num: Can be 1 or 2 to select the SPI peripheral.
  * @param  gpio: Chip select gpio.
  * @param  pin: Chip select pin.
  * @param  reg: Register number to read from.
  * @param  data: Pointer where the data will be stored.
  * @param  size: Amount of bytes to read.
  *
  * @retval 0 if success.
  */
int spi_read_reg(uint8_t spi_num, GPIO_TypeDef *gpio, uint16_t pin,
	uint8_t reg, uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
