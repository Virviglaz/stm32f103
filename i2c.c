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
#include "i2c.h"
#include "gpio.h"
#include "rcc.h"
#include "dma.h"
#include <errno.h>

#define I2C_FAST_FREQ	400000
#define I2C_NORM_FREQ	100000

#define VALIDATE_I2C()	if (!i2c_num || i2c_num > 2) return -EINVAL;

/* task and error reporting */
enum task_t {
	NO_INIT = 0,
	IN_PROGRESS,
	DONE,
	ERR_NOACK,
	ERR_UNKNOWN,
};

enum dir_t {
	WRITING,
	READING,
};

/* every transfer is considered as the message */
struct msg_t {
	enum dir_t dir;
	uint8_t *data;
	uint16_t size;
	struct msg_t *next;
};

/* we support I2C1 and I2C2 only for now */
static I2C_TypeDef *i2c_s[] = { I2C1, I2C2 };

/* isr will keep all transfer parameters per bus */
static struct isr_t {
	I2C_TypeDef *i2c;
	void (*handler)(void *data, uint8_t err);
	void *private_data;
	enum task_t task;
	uint8_t addr;
	struct msg_t *msg;
	DMA_Channel_TypeDef *tx_dma;
	DMA_Channel_TypeDef *rx_dma;
	bool is_rx;
} isrs[2] = { 0 };

/* set start bit macro */
static inline void start(I2C_TypeDef *i2c)
{
	/* TODO: remove this STOP bit polling! */
	while (i2c->CR1 & I2C_CR1_STOP) { }
	i2c->CR1 |= I2C_CR1_START;
}

/* set stop bit macro */
static inline void stop(I2C_TypeDef *i2c)
{
	i2c->CR1 |= I2C_CR1_STOP;
}

static void release_bus(struct isr_t *isr)
{
	stop(isr->i2c);
	dma_release(isr->tx_dma);
	dma_release(isr->rx_dma);
	if (isr->handler)
		isr->handler(isr->private_data, I2C_SUCCESS);
	isr->task = DONE;
}

/* handles the messages */
static void start_next_message(struct isr_t *isr)
{
	struct msg_t *m = isr->msg;
	I2C_TypeDef *i2c = isr->i2c;

	/*
	 * Note: In transmitter mode bus is released from I2C interrupt.
	 * In receiver mode we release the bus from DMA interrupt.
	 */
	if (!m) {
		if (isr->is_rx)
			release_bus(isr);
		return;
	}

	if (m->dir == READING) {
		i2c->CR1 |= I2C_CR1_ACK;
		i2c->CR2 |= I2C_CR2_LAST;
		isr->rx_dma->CMAR = (uint32_t)m->data;
		isr->rx_dma->CPAR = (uint32_t)&i2c->DR;
		isr->rx_dma->CNDTR = m->size;
		isr->rx_dma->CCR = DMA_CCR1_EN | DMA_CCR1_TCIE | \
			DMA_CCR1_MINC;
	} else {
		i2c->CR1 &= ~I2C_CR1_ACK;
		i2c->CR2 &= ~I2C_CR2_LAST;
		isr->tx_dma->CMAR = (uint32_t)m->data;
		isr->tx_dma->CPAR = (uint32_t)&i2c->DR;
		isr->tx_dma->CNDTR = m->size;
		isr->tx_dma->CCR = DMA_CCR1_EN | DMA_CCR1_TCIE | \
			DMA_CCR1_MINC | DMA_CCR1_DIR;
	}
}

/* executed at the end of the dma transfer */
static void dma_isr(void *data)
{
	struct isr_t *isr = data;
	struct msg_t *m = isr->msg;

	isr->msg = m->next;
	start_next_message(isr);
}

/* generic transfer function. Condigure the bus and create a messages list */
static int transfer(uint8_t i2c_num, uint8_t addr, struct msg_t **msg)
{
	const uint8_t tx_ch[] = { 6, 4 };
	const uint8_t rx_ch[] = { 7, 5 };
	I2C_TypeDef *i2c;
	struct isr_t *isr;
	struct msg_t *m;

	VALIDATE_I2C();
	i2c_num--;
	i2c = i2c_s[i2c_num];
	isr = &isrs[i2c_num];

	if (!isr->i2c) {
		int ret = i2c_init(i2c_num + 1, false);
		if (ret)
			return ret;
	}

	if (isr->task == NO_INIT) {
		int res = i2c_init(i2c_num + 1, false);
		if (res)
			return res;
	}

	if (isr->task == IN_PROGRESS)
		return -EILSEQ;

	isr->task = IN_PROGRESS;
	isr->addr = addr << 1;
	isr->msg = *msg;
	isr->is_rx = false;
	if (!isr->msg)
		return -EINVAL;

	isr->tx_dma = get_dma_ch(tx_ch[i2c_num], dma_isr, isr);
	isr->rx_dma = get_dma_ch(rx_ch[i2c_num], dma_isr, isr);

	if (!isr->tx_dma || !isr->rx_dma) {
		dma_release(isr->tx_dma);
		dma_release(isr->rx_dma);
		return -EINVAL;
	}

	m = *msg; /* build a linked list */
	do {
		msg++;
		if (m->dir == READING)
			isr->is_rx = true;
		m->next = *msg;
		m = m->next;
	} while (m);

	i2c->CR2 &= ~I2C_CR2_LAST;
	i2c->CR2 |= I2C_CR2_ITERREN | I2C_CR2_ITEVTEN;

	/* initiate the transfer */
	start_next_message(isr);
	start(i2c);

	/* having the handler this is non-waiting call */
	if (!isr->handler)
		while (isr->task < DONE) { }
	else
		return 0;

	return isr->task == DONE ? 0 : -(int)isr->task;
}

static int send_message(uint8_t i2c_num, uint8_t addr, uint8_t *reg,
	uint8_t reg_size, uint8_t *data, uint16_t size, enum dir_t dir)
{
	/* we use static variable to reduce the stack usage using RTOS */
	static struct msg_t msg[2];
	static struct msg_t *msgs[3];

	msg[0].dir = WRITING;
	msg[0].data = reg;
	msg[0].size = reg_size;

	msg[1].dir = dir;
	msg[1].data = data;
	msg[1].size = size;

	msgs[0] = &msg[0];
	msgs[1] = &msg[1];
	msgs[2] = 0;

	return transfer(i2c_num, addr, msgs);
}

/* this ISR occures when address is send and initiate DMA transfer */
static void isr_handler(I2C_TypeDef *i2c, uint8_t i2c_num)
{
	struct isr_t *isr = &isrs[i2c_num];
	struct msg_t *m = isr->msg;
	uint16_t sr1 = i2c->SR1;

	/* start is set, sending address */
	if (sr1 & I2C_SR1_SB) {
		if (m->dir == READING) {
			i2c->CR1 |= I2C_CR1_ACK;
			i2c->DR = isr->addr | 1;
		} else {
			i2c->CR1 &= ~I2C_CR1_ACK;
			i2c->DR = isr->addr;
		}
		return;
	}

	if (sr1 & I2C_SR1_TXE && m->dir == READING)
		start(i2c);
	
	/* clear addr flag */
	i2c->SR2;

	/* all data send, execute handler and clean up */
	if (!m)
		release_bus(isr);
}

/* this ISR occures when no acknowledges being received from slave */
static void isr_err_handler(I2C_TypeDef *i2c, uint8_t i2c_num)
{
	struct isr_t *isr = &isrs[i2c_num];
	uint16_t sr1 = i2c->SR1;

	i2c->SR1 = 0;
	i2c->SR2 = 0;
	dma_release(isr->tx_dma);
	dma_release(isr->rx_dma);

	if (sr1 & I2C_SR1_AF)
		isr->task = ERR_NOACK;
	else
		isr->task = ERR_UNKNOWN;

	stop(i2c);
	if (isr->handler)
		isr->handler(isr->private_data, isr->task);
}

void I2C1_EV_IRQHandler(void)
{
	isr_handler(I2C1, 0);
}

void I2C1_ER_IRQHandler(void)
{
	isr_err_handler(I2C1, 0);
}

void I2C2_EV_IRQHandler(void)
{
	isr_handler(I2C2, 1);
}

void I2C2_ER_IRQHandler(void)
{
	isr_err_handler(I2C2, 1);
}

#ifdef FREERTOS

#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

static struct params {
	SemaphoreHandle_t lock;
	SemaphoreHandle_t done;
	uint8_t result;
} params[2];
#endif

int i2c_init(uint8_t i2c_num, bool fast_mode)
{
	I2C_TypeDef *i2c;
	struct isr_t *isr;
	struct system_clock_t *clock = get_clocks();
	uint16_t freqrange;

	VALIDATE_I2C();
	i2c_num--;

	switch (i2c_num) {
	case 0:
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		gpio_output_init(PB6, OPENDRAIN_ALT_OUTPUT, GPIO_FREQ_2MHz);
		gpio_output_init(PB7, OPENDRAIN_ALT_OUTPUT, GPIO_FREQ_2MHz);
		NVIC_EnableIRQ(I2C1_EV_IRQn);
		NVIC_EnableIRQ(I2C1_ER_IRQn);
		break;
	case 1:
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		gpio_output_init(PB10, OPENDRAIN_ALT_OUTPUT, GPIO_FREQ_2MHz);
		gpio_output_init(PB11, OPENDRAIN_ALT_OUTPUT, GPIO_FREQ_2MHz);
		NVIC_EnableIRQ(I2C2_EV_IRQn);
		NVIC_EnableIRQ(I2C2_ER_IRQn);
		break;
	default:
		return EINVAL;
	}

	i2c = i2c_s[i2c_num];
	isr = &isrs[i2c_num];

	if (fast_mode) {
		i2c->CCR = clock->apb1_freq / (I2C_FAST_FREQ * 3);
		i2c->CCR |= I2C_CCR_FS;
		if (!i2c->CCR)
			i2c->CCR = 1;
		i2c->TRISE = ((freqrange * 300) / 1000) + 1;
	} else {
		i2c->CCR = clock->apb1_freq / (I2C_NORM_FREQ * 2);
		if (i2c->CCR < 0x04) /* Set minimum allowed value */
			i2c->CCR = 0x04;
		i2c->TRISE = freqrange + 1;
	}

	freqrange = clock->apb1_freq / 1000000;

	i2c->CR2 = I2C_CR2_DMAEN | ((clock->apb1_freq / 1000000) & I2C_CR2_FREQ);
	i2c->CR1 = I2C_CR1_PE;

	isr->task = DONE;
	isr->i2c = i2c;

#ifdef FREERTOS
	params[i2c_num].lock = xSemaphoreCreateMutex();
	params[i2c_num].done = xSemaphoreCreateBinary();
#endif

	return 0;
}

/*
 * this functions using consistently 2 messages transfer.
 * first message holding the address of memory of register where
 * access is planned. Then the next message is a buffer reading/writing
 * followed by repeated start condition in case of reading.
 */
int i2c_write_reg(uint8_t i2c_num, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size)
{
	return send_message(i2c_num, addr, (void *)&reg, sizeof(reg),
		data, size, WRITING);
}

int i2c_read_reg(uint8_t i2c_num, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size)
{
	return send_message(i2c_num, addr, (void *)&reg, sizeof(reg),
		data, size, READING);
}

int i2c_write_eeprom_w(uint8_t i2c_num, uint8_t addr, uint16_t reg,
	uint8_t *data, uint16_t size)
{
	return send_message(i2c_num, addr, (void *)&reg, sizeof(reg),
		data, size, WRITING);
}

int i2c_read_eeprom_w(uint8_t i2c_num, uint8_t addr, uint16_t reg,
	uint8_t *data, uint16_t size)
{
	return send_message(i2c_num, addr, (void *)&reg, sizeof(reg),
		data, size, READING);
}

int i2c_write_eeprom_dw(uint8_t i2c_num, uint8_t addr, uint32_t reg,
	uint8_t *data, uint16_t size)
{
	return send_message(i2c_num, addr, (void *)&reg, sizeof(reg),
		data, size, WRITING);
}

int i2c_read_eeprom_dw(uint8_t i2c_num, uint8_t addr, uint32_t reg,
	uint8_t *data, uint16_t size)
{
	return send_message(i2c_num, addr, (void *)&reg, sizeof(reg),
		data, size, READING);
}

int i2c_set_handler(uint8_t i2c_num, void (*handler)(void *data, uint8_t err),
	void *private_data)
{
	VALIDATE_I2C();
	i2c_num--;

	isrs[i2c_num].handler = handler;
	isrs[i2c_num].private_data = private_data;

	return 0;
}

#ifdef FREERTOS

static void rtos_handler(void *args, uint8_t err)
{
	struct params *par = args;

	par->result = err;

	xSemaphoreGiveFromISR(par->done, NULL);
}

/*
 * this is rtos oriented i2c function. During the transfer the task is put
 * on hold and not using a cpu time. When transfer is finished the task
 * handler is reported to isr daemon resulting deferred interrupt technics.
 * isr daemon is waiking up calling task and execution of code continues.
 */
static int rw_reg8_rtos(uint8_t i2c_num, uint8_t addr, uint8_t *reg,
	uint8_t reg_size, uint8_t *data, uint16_t size, enum dir_t dir)
{
	int res;

	xSemaphoreTake(params[i2c_num - 1].lock, portMAX_DELAY);

	i2c_set_handler(i2c_num, rtos_handler, &params[i2c_num - 1]);

	res = send_message(i2c_num, addr, reg, reg_size, data, size, dir);
	if (!res)
		xSemaphoreTake(params[i2c_num - 1].done, portMAX_DELAY);

	xSemaphoreGive(params[i2c_num - 1].lock);

	return res ? res : params[i2c_num - 1].result;
}

int i2c_write_reg_rtos(uint8_t i2c_num, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size)
{
	return rw_reg8_rtos(i2c_num, addr, &reg, sizeof(reg),
		data, size, WRITING);
}

int i2c_read_reg_rtos(uint8_t i2c_num, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size)
{
	return rw_reg8_rtos(i2c_num, addr, &reg, sizeof(reg),
		data, size, READING);
}

#endif /* FREERTOS */
