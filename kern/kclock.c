/* See COPYRIGHT for copyright information. */

#include <inc/x86.h>
#include <kern/kclock.h>
#include <inc/stdio.h>

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

