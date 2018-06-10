/* See COPYRIGHT for copyright information. */

#include <inc/x86.h>
#include <kern/kclock.h>
#include <inc/stdio.h>
#include <inc/time.h>
#include <inc/string.h>

static struct tm
read_time(void)
{
	return (struct tm){
		.tm_sec = BCD2BIN(mc146818_read(RTC_SEC)),
		.tm_min = BCD2BIN(mc146818_read(RTC_MIN)),
		.tm_hour = BCD2BIN(mc146818_read(RTC_HOUR)),
		.tm_mday = BCD2BIN(mc146818_read(RTC_DAY)),
		.tm_mon = BCD2BIN(mc146818_read(RTC_MON)) - 1,
		.tm_year = BCD2BIN(mc146818_read(RTC_YEAR)),
	};
}

int gettime(void)
{
	nmi_disable();

	struct tm time, time_chk;

	do {
		while (mc146818_read(RTC_AREG) & RTC_UPDATE_IN_PROGRESS);

		time = read_time();
		time_chk = read_time();
	} while (memcmp(&time, &time_chk, sizeof(time)));

	nmi_enable();

	return timestamp(&time);
}

void
rtc_init(void)
{
	nmi_disable();

	uint8_t value;

	outb(IO_RTC_CMND, RTC_AREG);
	value = inb(IO_RTC_DATA);
	outb(IO_RTC_DATA, (value & 0xF0) | 15);

	outb(IO_RTC_CMND, RTC_BREG);
	value = inb(IO_RTC_DATA);
	outb(IO_RTC_DATA, value | RTC_PIE);

	nmi_enable();
}

uint8_t
rtc_check_status(void)
{
	outb(IO_RTC_CMND, RTC_CREG);

	return inb(IO_RTC_DATA);
}

unsigned
mc146818_read(unsigned reg)
{
	outb(IO_RTC_CMND, reg);
	return inb(IO_RTC_DATA);
}

void
mc146818_write(unsigned reg, unsigned datum)
{
	outb(IO_RTC_CMND, reg);
	outb(IO_RTC_DATA, datum);
}

