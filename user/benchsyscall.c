#include <inc/lib.h>

#define COUNT 1000000

void
umain(int argc, char **argv)
{
	volatile int id;

	unsigned start = sys_get_mono_ts();

	for (unsigned i = 0; i < COUNT; ++i) {
		id = sys_getenvid();
	}

	unsigned delta = sys_get_mono_ts() - start;

	cprintf("count=%d spent=%dms op/ms=%d\n",
			COUNT, delta, COUNT/delta);
}
