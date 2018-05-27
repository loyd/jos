// Called from entry.S to get us going.
// entry.S already took care of defining envs, pages, uvpd, and uvpt.

#include <inc/lib.h>

extern void umain(int argc, char **argv);

const volatile struct Env *thisenv;
const char *binaryname = "<unknown>";

#ifdef JOS_PROG
void (* volatile sys_exit)(void);
#endif

void
libmain(int argc, char **argv)
{
	// set thisenv to point at our Env structure in envs[].
	envid_t eid = sys_getenvid();

	for (unsigned i = 0; i < NENV; ++i) {
		if (envs[i].env_id == eid) {
			thisenv = envs + i;
			break;
		}
	}

	// save the name of the program so that panic() can use it
	if (argc > 0)
		binaryname = argv[0];

	// call user main routine
	umain(argc, argv);

	// exit
#ifdef JOS_PROG
	sys_exit();
#else
	exit();
#endif
}

