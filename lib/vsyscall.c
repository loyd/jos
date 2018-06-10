#include <inc/vsyscall.h>
#include <inc/lib.h>

static inline int32_t
vsyscall(int num)
{
	return *((int32_t *)UVSYS + num);
}

int vsys_gettime(void)
{
	return vsyscall(VSYS_gettime);
}
