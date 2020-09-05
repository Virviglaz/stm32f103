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
#include "rtc.h"
#include <errno.h>

#define FIRSTYEAR	2000			// start year
#define FIRSTDAY	6			// 0 = Sunday
#define RTC_LSB_MASK	((uint32_t)0x0000FFFF)	/*!< RTC LSB Mask */
#define PRLH_MSB_MASK	((uint32_t)0x000F0000)	/*!< RTC Prescaler MSB Mask */

static enum clock_t clock_source = NO_CLOCK;
static const uint8_t days_in_month[] =
	{ 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };


static uint8_t is_dst(struct rtc_t *t)
{
	uint8_t wday, month = t->month;

	if( month < 3 || month > 10 ) { /* month 1, 2, 11, 12 */
		return 0; // -> Winter
	}

	wday = t->wday;

	if (t->mday - wday >= 25 && (wday || t->hour >= 2) ) {
		// after last Sunday 2:00
		if( month == 10 ) {	// October -> Winter
			return 0;
		}
	} else {	// before last Sunday 2:00
		if( month == 3 ) {	// March -> Winter
			return 0;
		}
	}

	return 1;
}

static uint8_t adjustDST(struct rtc_t *t)
{
	uint8_t hour, day, wday, month;		// locals for faster access

	hour  = t->hour;
	day   = t->mday;
	wday  = t->wday;
	month = t->month;

	if ( is_dst(t) ) {
		t->dst = 1;
		hour++;				// add one hour
		if( hour == 24 ){		// next day
			hour = 0;
			wday++;			// next weekday
			if( wday == 7 ) {
				wday = 0;
			}
			if( day == days_in_month[month-1] ) {	// next month
				day = 0;
				month++;
			}
			day++;
		}
		t->month = month;
		t->hour  = hour;
		t->mday  = day;
		t->wday  = wday;
		return 1;
	} else {
		t->dst = 0;
		return 0;
	}
}

static void counter_to_struct(uint32_t sec, struct rtc_t *t)
{
	uint16_t day;
	uint8_t year;
	uint16_t dayofyear;
	uint8_t leap400;
	uint8_t month;

	t->sec = sec % 60;
	sec /= 60;
	t->min = sec % 60;
	sec /= 60;
	t->hour = sec % 24;
	day = (uint16_t)(sec / 24);

	t->wday = (day + FIRSTDAY) % 7;			// weekday

	year = FIRSTYEAR % 100;				// 0..99
	leap400 = 4 - ((FIRSTYEAR - 1) / 100 & 3);	// 4, 3, 2, 1

	for(;;) {
		dayofyear = 365;
		if (!(year & 3)) {
			dayofyear = 366;		// leap year
			if (year == 0 || year == 100 || year == 200) {
				// 100 year exception
				if (--leap400 ) {	// 400 year exception
					dayofyear = 365;
				}
			}
		}
		if (day < dayofyear)
			break;
		day -= dayofyear;
		year++;					// 00..136 / 99..235
	}
	t->year = year + FIRSTYEAR / 100 * 100;	// + century

	if(dayofyear & 1 && day > 58)// no leap year and after 28.2.
		day++;					// skip 29.2.

	for (month = 1; day >= days_in_month[month-1]; month++)
		day -= days_in_month[month-1];

	t->month = month;				// 1..12
	t->mday = day + 1;				// 1..31
}

static uint32_t struct_to_counter(const struct rtc_t *t)
{
	uint8_t i;
	uint32_t result = 0;
	uint16_t idx, year;

	year = t->year;

	/* Calculate days of years before */
	result = (uint32_t)year * 365;
	if (t->year >= 1) {
		result += (year + 3) / 4;
		result -= (year - 1) / 100;
		result += (year - 1) / 400;
	}

	/* Start with 2000 a.d. */
	result -= 730485UL;

	/* Make month an array index */
	idx = t->month - 1;

	/* Loop thru each month, adding the days */
	for (i = 0; i < idx; i++) {
		result += days_in_month[i];
	}

	/* Leap year? adjust February */
	if (year % 400 == 0 || (year % 4 == 0 && year % 100)) {
		;
	} else {
		if (t->month > 1)
			result--;
	}

	/* Add remaining days */
	result += t->mday;

	/* Convert to seconds, add all the other stuff */
	result = (result-1) * 86400L + (uint32_t)t->hour * 3600 + \
		(uint32_t)t->min * 60 + t->sec;

	return result;
}

static inline void wait_for_sync(void)
{
	RTC->CRL &= ~RTC_CRL_RSF;
	while (!(RTC->CRL & RTC_CRL_RSF));
}

static uint32_t get_cnt(void)
{
	wait_for_sync();
	return (RTC->CNTH << 16) | RTC->CNTL;
}

static inline void wait_for_write_finished(void)
{
	while (!(RTC->CRL & RTC_CRL_RTOFF));
}

static inline void config_mode_enable(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_DBP;
	RTC->CRL |= RTC_CRL_CNF;
}

static inline void config_mode_disable(void)
{
	RTC->CRL &= ~RTC_CRL_CNF;
	wait_for_write_finished();

	if (clock_source != HSE_CLOCK_DIV128)
		/* Note: for HSE/128 this bit should remains enabled */
		PWR->CR &= ~PWR_CR_DBP;
}

static inline void set_prescaler(uint32_t psc)
{
	config_mode_enable();
	RTC->PRLH = (psc & PRLH_MSB_MASK) >> 16;
	RTC->PRLL = (psc & RTC_LSB_MASK);
	config_mode_disable();
}

static void rtc_set_counter(uint32_t cnt)
{
	config_mode_enable();
	RTC->CNTH = cnt >> 16;
	RTC->CNTL = (cnt & RTC_LSB_MASK);
	config_mode_disable();
}

void rtc_gettime(struct rtc_t *rtc)
{
	counter_to_struct(get_cnt(), rtc);
	adjustDST(rtc);
}

void rtc_settime(const struct rtc_t *rtc)
{
	uint32_t cnt;
	struct rtc_t ts;

	cnt = struct_to_counter(rtc); // non-DST counter-value
	counter_to_struct(cnt, &ts);  // normalize struct (for weekday)
	if (is_dst(&ts)) {
		cnt -= 60 * 60; // Subtract one hour
	}

	rtc_set_counter(cnt);
}

int rtc_init(enum clock_t source, uint32_t prc)
{
	if (!(RCC->BDCR & RCC_BDCR_RTCEN) || get_rtc_clock() != source) {
		const struct rtc_t default_time = {
			.year = 2020,
			.month = 1,
			.mday = 1,
			.wday = 3, /* check your callendar */
			.hour = 12,
			.min = 0,
			.sec = 0,
			.dst = 0,
		};

		/* Enable LSE */
		if (set_rtc_clock(source) != source)
			return -EINVAL;

		wait_for_sync();

		/* Set RTC prescaler: set RTC period to 1sec */
		set_prescaler(prc - 1);

		rtc_settime(&default_time);
	}

	clock_source = source;
	return 0;
}

static void (*sec_irq_handler)(void);

void rtc_sec_interrupt(void (*handler)(void))
{
	sec_irq_handler = handler;

	RTC->CRH |= RTC_CRH_SECIE;

	NVIC_EnableIRQ(RTC_IRQn);
}

void RTC_IRQHandler(void)
{
	sec_irq_handler();
	RTC->CRL = 0;
}
