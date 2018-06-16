#include <inc/mmu.h>
#include <inc/memlayout.h>

.data
// Define the global symbols 'envs', 'pages', 'vsys', 'uvpt', and 'uvpd'
// so that they can be used in C as if they were ordinary global arrays.
	.globl envs
	.set envs, UENVS
	.globl pages
	.set pages, UPAGES
	.globl vsys
	.set vsys, UVSYS
	.globl uvpt
	.set uvpt, UVPT
	.globl uvpd
	.set uvpd, (UVPT+(UVPT>>12)*4)


// Entrypoint - this is where the kernel (or our parent environment)
// starts us running when we are initially loaded into a new environment.
.text
.globl _start
_start:
	// See if we were started with arguments on the stack
#ifndef CONFIG_KSPACE
	cmpl $USTACKTOP, %esp
	jne args_exist
#endif

	// If not, push dummy argc/argv arguments.
	// This happens when we are loaded by the kernel,
	// because the kernel does not know about passing arguments.
	pushl $0
	pushl $0

args_exist:
	call libmain
1:	jmp 1b


.globl fast_syscall_user
.type  fast_syscall_user, @function
.align 2
fast_syscall_user:
	pushl %ebp
	movl %esp, %ebp

	# Follow cdecl
	pushl %ebx
	pushl %edi
	pushl %esi

	# Pass arguments
	movl 8(%ebp), %eax
	movl 12(%ebp), %edx
	movl 16(%ebp), %ecx
	movl 20(%ebp), %ebx
	movl 24(%ebp), %edi
	movl 28(%ebp), %esi

	# Save esp and eip
	pushl $(fast_syscall_ret)
	movl %esp, %ebp

	sysenter

fast_syscall_ret:
	addl $4, %esp

	# Follow cdecl
	popl %esi
	popl %edi
	popl %ebx

	popl %ebp
	ret
