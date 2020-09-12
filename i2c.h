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

#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_SUCCESS	0
#define I2C_ERR_NOACK	1

/**
  * @brief  Initialize I2C master in fast/normal mode.
  * @param  i2c_num: can be 1 or 2 to select the I2C peripheral.
  * @param  fast_mode: enables the 400kHz mode if true.
  *
  * @retval 0 if success
  */
int i2c_init(uint8_t i2c_num, bool fast_mode);

/**
  * @brief  Transfer the data buffer to reg
  * @param  i2c_num: can be 1 or 2 to select the I2C peripheral.
  * @param  addr: device i2c address
  * @param  data: pointer to buffer
  * @param  size: amount of bytes to transfer
  *
  * @retval 0 if success, I2C_ERR_NOACK if device not responds
  */
int i2c_write_reg(uint8_t i2c_num, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size);

/**
  * @brief  Receives the data to buffer starting from reg
  * @param  i2c_num: can be 1 or 2 to select the I2C peripheral.
  * @param  addr: device i2c address
  * @param  data: pointer to buffer
  * @param  size: amount of bytes to receive
  *
  * @retval 0 if success, I2C_ERR_NOACK if device not responds
  */
int i2c_read_reg(uint8_t i2c_num, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size);

/**
  * @brief  Transfer the data buffer to location
  * @param  i2c_num: can be 1 or 2 to select the I2C peripheral.
  * @param  addr: device i2c address
  * @param  pos: pointer to buffer location
  * @param  pos_size: size of location
  * @param  data: pointer to buffer
  * @param  size: amount of bytes to transfer
  *
  * @retval 0 if success, I2C_ERR_NOACK if device not responds
  */
int i2c_write(uint8_t i2c_num, uint8_t addr, uint8_t *pos, uint16_t pos_size,
	uint8_t *data, uint16_t size);

/**
  * @brief  Receives the data from location to buffer
  * @param  i2c_num: can be 1 or 2 to select the I2C peripheral.
  * @param  addr: device i2c address
  * @param  pos: pointer to buffer location
  * @param  pos_size: size of location
  * @param  data: pointer to buffer
  * @param  size: amount of bytes to receive
  *
  * @retval 0 if success, I2C_ERR_NOACK if device not responds
  */
int i2c_read(uint8_t i2c_num, uint8_t addr, uint8_t *pos, uint16_t pos_size,
	uint8_t *data, uint16_t size);

/**
  * @brief  Receives the data from location to buffer
  * @param  i2c_num: can be 1 or 2 to select the I2C peripheral.
  * @param  handler: pointer to function executed at the end of the transfer
  * @param  private_data: pointer private data for handler if needed
  *
  * @retval 0 if success
  */
int i2c_set_handler(uint8_t i2c_num, void (*handler)(void *data, uint8_t err),
	void *private_data);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
