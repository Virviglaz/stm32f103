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

#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stm32f10x.h>
#include <stdint.h>
#include <stdbool.h>
#include "rtos.h"

typedef void (*uart_tx_handler_t)(void *private_data);
typedef void (*uart_rx_handler_t) \
	(uint8_t uart_num, char *data, uint16_t size, void *private_data);

/**
  * @brief  initialize uart master in full duplex mode.
  * @param  uart_num: can be 1, or 3. Uart number.
  * @param  freq: uart frequency.
  *
  * @retval none.
  */

void uart_init(uint8_t uart_num, uint32_t freq);

/**
  * @brief  simple non-interrupt write.
  * @param  uart_num: can be 1, or 3. Uart number.
  * @param  ch: character to write.
  *
  * @retval none.
  */
void uart_write(uint8_t uart_num, char ch);

/**
  * @brief  enable interrupt based receiving.
  * @param  uart_num: can be 1, or 3. Uart number.
  * @param  buf: rx buffer where the data will be stored.
  * @param  size: size of rx buffer.
  * @param  handler: optional handler that executes when data is received.
  * @param  private_data: optional pointer to send to handler.
  *
  * @retval none.
  */
void uart_enable_rx_buffer(uint8_t uart_num, char *buf, uint16_t size,
	uart_rx_handler_t handler, void *private_data);

/**
  * @brief  disable interrupt based receiving.
  * @param  uart_num: can be 1, or 3. Uart number.
  *
  * @retval none.
  */
void uart_disable_rx_buffer(uint8_t uart_num);

/**
  * @brief  send data interrupt based.
  * @param  uart_num: can be 1, or 3. Uart number.
  * @param  buf: buffer to send.
  * @param  size: amount of bytes to send.
  * @param  handler: optional handler that executes when data is send.
  * @param  private_data: optional pointer to send to handler.
  *
  * @retval 0 if success.
  */
int uart_send_data(uint8_t uart_num, char *buf, uint16_t size,
	uart_tx_handler_t handler, void *private_data);

/**
  * @brief  send null terminated line to uart using waiting call.
  * @param  uart_num: can be 1, or 3. Uart number.
  * @param  str: pointer to character buffer.
  *
  * @retval none.
  */
void uart_send_string(uint8_t uart_num, const char *str);

/**
  * @brief  get amount of bytes received.
  * @param  uart_num: can be 1, or 3. Uart number.
  *
  * @retval amount of bytes in rx buffer.
  */
uint16_t uart_received_bytes(uint8_t uart_num);

/**
  * @brief  check data ready in rx buffer if handler is skipped.
  * @param  uart_num: can be 1, or 3. Uart number.
  *
  * @retval true if buffer have the data.
  */
bool uart_check_rx_buffer(uint8_t uart_num);

/**
  * @brief  reset rx buffer counter.
  * @param  uart_num: can be 1, or 3. Uart number.
  *
  * @retval none.
  */
void uart_reset_rx_buffer(uint8_t uart_num);

/**
  * @brief  get pointer to rx buffer.
  * @param  uart_num: can be 1, or 3. Uart number.
  *
  * @retval pointer to buffer.
  */
char *uart_get_rx_buffer(uint8_t uart_num);

#ifdef FREERTOS

/**
  * @brief  send data using RTOS.
  * @param  uart_num: can be 1, or 3. Uart number.
  * @param  buf: buffer to send.
  * @param  size: amount of bytes to send.
  *
  * @retval none.
  */
void uart_send_data_rtos(uint8_t uart_num, char *buf, uint16_t size);

/**
  * @brief  send null termitanted string using RTOS.
  * @param  uart_num: can be 1, or 3. Uart number.
  * @param  buf: buffer to send.
  *
  * @retval none.
  */
void uart_send_string_rtos(uint8_t uart_num, char *string);

/**
  * @brief  Wait for data from uart using RTOS.
  * @param  uart_num: can be 1, or 3. Uart number.
  * @param  buf: buffer to send.
  * @param  size: maximum buffer size.
  *
  * @retval amount of bytes received.
  */
uint16_t uart_receive_rtos(uint8_t uart_num, char *buf, uint16_t size);

#endif /* FREERTOS */

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */
