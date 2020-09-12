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

#include "i2c.h"
#include "gpio.h"
#include "rcc.h"
#include <errno.h>

#define I2C_FAST_FREQ	400000
#define I2C_NORM_FREQ	100000

enum task_t {
	IN_PROGRESS,
	DONE,
	ERR_NOACK,
	ERR_UNKNOWN,
};

enum dir_r {
	WRITING,
	READING,
};

struct msg_t {
	enum dir_r dir;
	uint8_t *data;
	uint16_t size;
	struct msg_t *next;
};

static I2C_TypeDef *i2c_s[] = { I2C1, I2C2 };
static struct isr_t {
	void (*handler)(void *data, uint8_t err);
	void *private_data;
	enum task_t task;
	uint8_t addr;
	struct msg_t *msg;
} isr[2] = { 0 };

static inline void start(I2C_TypeDef *i2c)
{
	i2c->CR1 |= I2C_CR1_START;
}

static inline void stop(I2C_TypeDef *i2c)
{
	i2c->CR1 |= I2C_CR1_STOP;
}

static int i2c_transfer(uint8_t i2c_num, uint8_t addr, struct msg_t **msg)
{
	I2C_TypeDef *i2c;

	if (!i2c_num || i2c_num > 2)
		return -EINVAL;

	i2c_num--;
	i2c = i2c_s[i2c_num];

	if (isr[i2c_num].task == IN_PROGRESS)
		return -EILSEQ;

	isr[i2c_num].task = IN_PROGRESS;

	struct msg_t *m = *msg++;
	if (!m)
		return -EINVAL;

	m->next = *msg;
	isr[i2c_num].msg = m;
	isr[i2c_num].addr = addr << 1;

	/* initiate the transfer */
	start(i2c);

	/* having the handler this is non-waiting call */
	if (!isr[i2c_num].handler)
		while (isr[i2c_num].task < DONE) { }
	else
		return 0;

	return isr[i2c_num].task == DONE ? 0 : -(int)isr[i2c_num].task;
}

static void isr_process(I2C_TypeDef *i2c, uint8_t i2c_num)
{
	struct msg_t *m = isr[i2c_num].msg;

	if (!m) {
		i2c->SR1;
		stop(i2c);
		isr[i2c_num].task = DONE;
		if (isr[i2c_num].handler)
			isr[i2c_num].handler(isr[i2c_num].private_data,
				I2C_SUCCESS);
		return;
	}

	/* start is set, sending address */
	if (i2c->SR1 & I2C_SR1_SB) {
		if (m->dir == READING) {
			i2c->CR1 |= I2C_CR1_ACK;
			i2c->DR = isr[i2c_num].addr | 1;
		} else {
			i2c->CR1 &= ~I2C_CR1_ACK;
			i2c->DR = isr[i2c_num].addr;
		}
		return;
	}

	/* address is send, processing first message */
	if (i2c->SR1 & I2C_SR1_ADDR) {
		i2c->SR2;
		i2c->DR = *m->data;
		m->data++;
		m->size--;
		return;
	}

	/* data is send, processing next data */
	if (i2c->SR1 & I2C_SR1_TXE) {
		if (m->size) {
			i2c->DR = *m->data;
			m->data++;
			m->size--;
			return;
		}
		isr[i2c_num].msg = m->next;
		if (m->next && m->next->dir == READING)
			start(i2c);
		return;
	}

	/* receiving the data */
	if (i2c->SR1 & I2C_SR1_RXNE) {
		*m->data = i2c->DR;
		m->data++;
		m->size--;
		if (m->size == 1) {
			i2c->CR1 &= ~I2C_CR1_ACK;
			stop(i2c);
			isr[i2c_num].task = DONE;
			if (isr[i2c_num].handler)
				isr[i2c_num].handler(isr[i2c_num].private_data,
					I2C_SUCCESS);
		}
	}
}

static void isr_err_handler(I2C_TypeDef *i2c, uint8_t i2c_num)
{
	if (i2c->SR1 & I2C_SR1_AF) {
		i2c->SR1 = 0;
		isr[i2c_num].task = ERR_NOACK;
		stop(i2c);
		if (isr[i2c_num].handler)
			isr[i2c_num].handler(isr[i2c_num].private_data,
				I2C_ERR_NOACK);
		return;
	}

	/* rest of the errors */
	i2c->SR1 = 0;
	i2c->SR2 = 0;
	isr[i2c_num].task = ERR_UNKNOWN;
	stop(i2c);
	if (isr[i2c_num].handler)
			isr[i2c_num].handler(isr[i2c_num].private_data,
				ERR_UNKNOWN);
}

void I2C1_EV_IRQHandler(void)
{
	isr_process(I2C1, 0);
}

void I2C1_ER_IRQHandler(void)
{
	isr_err_handler(I2C1, 0);
}

void I2C2_EV_IRQHandler(void)
{
	isr_process(I2C2, 1);
}

void I2C2_ER_IRQHandler(void)
{
	isr_err_handler(I2C2, 1);
}

int i2c_init(uint8_t i2c_num, bool fast_mode)
{
	I2C_TypeDef *i2c;
	struct system_clock_t *clock = get_clocks();
	uint16_t freqrange;

	if (!i2c_num || i2c_num > 2)
		return -EINVAL;
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
	}

	i2c = i2c_s[i2c_num];

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

	i2c->CR2 = I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | \
		((clock->apb1_freq / 1000000) & I2C_CR2_FREQ);
	i2c->CR1 = I2C_CR1_PE;

	isr[i2c_num].task = DONE;

	return 0;
}

int i2c_write_reg(uint8_t i2c_num, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size)
{
	struct msg_t msg[] = {
		{ WRITING, &reg, 1},
		{ WRITING, data, size },
	};
	struct msg_t *msgs[] = { &msg[0], &msg[1], 0 };

	return i2c_transfer(i2c_num, addr, msgs);
}

int i2c_read_reg(uint8_t i2c_num, uint8_t addr, uint8_t reg,
	uint8_t *data, uint16_t size)
{
	struct msg_t msg[] = {
		{ WRITING, &reg, 1 },
		{ READING, data, size },
	};
	struct msg_t *msgs[] = { &msg[0], &msg[1], 0 };

	return i2c_transfer(i2c_num, addr, msgs);
}

int i2c_write(uint8_t i2c_num, uint8_t addr, uint8_t *pos, uint16_t pos_size,
	uint8_t *data, uint16_t size)
{
	struct msg_t msg[] = {
		{ WRITING, pos, pos_size },
		{ WRITING, data, size },
	};
	struct msg_t *msgs[] = { &msg[0], &msg[1], 0 };

	return i2c_transfer(i2c_num, addr, msgs);
}

int i2c_read(uint8_t i2c_num, uint8_t addr, uint8_t *pos, uint16_t pos_size,
	uint8_t *data, uint16_t size)
{
	struct msg_t msg[] = {
		{ WRITING, pos, pos_size},
		{ READING, data, size },
	};
	struct msg_t *msgs[] = { &msg[0], &msg[1], 0 };

	return i2c_transfer(i2c_num, addr, msgs);
}

int i2c_set_handler(uint8_t i2c_num, void (*handler)(void *data, uint8_t err),
	void *private_data)
{
	if (!i2c_num || i2c_num > 2)
		return -EINVAL;
	i2c_num--;

	isr[i2c_num].handler = handler;
	isr[i2c_num].private_data = private_data;

	return 0;
}
