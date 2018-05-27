/* See COPYRIGHT for copyright information. */

#include <inc/mmu.h>
#include <inc/memlayout.h>
#include <inc/trap.h>

#include <kern/picirq.h>

#ifdef CONFIG_KSPACE
.comm intr_ret_eip, 4
.comm intr_ebp_reg, 4
.comm intr_esp_reg, 4
.comm intr_cs, 4
.comm intr_eflags, 4
#endif


###################################################################
# exceptions/interrupts
###################################################################

/* TRAPHANDLER defines a globally-visible function for handling a trap.
 * It pushes a trap number onto the stack, then jumps to _alltraps.
 * Use TRAPHANDLER for traps where the CPU automatically pushes an error code.
 *
 * You shouldn't call a TRAPHANDLER function from C, but you may
 * need to _declare_ one in C (for instance, to get a function pointer
 * during IDT setup).  You can declare the function with
 *   void NAME();
 * where NAME is the argument passed to TRAPHANDLER.
 */
#define TRAPHANDLER(name, num)						\
	.globl name;		/* define global symbol for 'name' */	\
	.type name, @function;	/* symbol type is function */		\
	.align 2;		/* align function definition */		\
	name:			/* function starts here */		\
	pushl $(num);							\
	jmp _alltraps

/* Use TRAPHANDLER_NOEC for traps where the CPU doesn't push an error code.
 * It pushes a 0 in place of the error code, so the trap frame has the same
 * format in either case.
 */
#define TRAPHANDLER_NOEC(name, num)					\
	.globl name;							\
	.type name, @function;						\
	.align 2;							\
	name:								\
	pushl $0;							\
	pushl $(num);							\
	jmp _alltraps


.text

.globl _alltraps
.type _alltraps, @function;
.align 2
_alltraps:
	push %ds
	push %es
	pushal

	mov $GD_KD, %ax
	mov %ax, %ds
	mov %ax, %es

	pushl %esp
	call trap

.globl clock_thdlr
.type clock_thdlr, @function;
.align 2;
#ifdef CONFIG_KSPACE
clock_thdlr:
	popl intr_ret_eip
	popl intr_cs
	popl intr_eflags
	movl %ebp, intr_ebp_reg
	movl %esp, intr_esp_reg
	movl $0x0,%ebp
	movl $(bootstacktop),%esp
	pushl $GD_KD
	pushl intr_esp_reg
	pushl intr_eflags
	pushl intr_cs
	pushl intr_ret_eip
	pushl $0
	pushl $(IRQ_OFFSET + IRQ_CLOCK)
	pushl %ds
	pushl %es

	pushl %eax
	pushl %ecx
	pushl %edx
	pushl %ebx
	pushl intr_esp_reg
	pushl intr_ebp_reg
	pushl %esi
	pushl %edi

	pushl %esp  /* trap(%esp) */
	call trap
	jmp .
#else
TRAPHANDLER_NOEC(clock_thdlr, IRQ_OFFSET + IRQ_CLOCK)
TRAPHANDLER_NOEC(traphandler0, 0)
TRAPHANDLER_NOEC(traphandler1, 1)
TRAPHANDLER_NOEC(traphandler2, 2)
TRAPHANDLER_NOEC(traphandler3, 3)
TRAPHANDLER_NOEC(traphandler4, 4)
TRAPHANDLER_NOEC(traphandler5, 5)
TRAPHANDLER_NOEC(traphandler6, 6)
TRAPHANDLER_NOEC(traphandler7, 7)
TRAPHANDLER(traphandler8, 8)
TRAPHANDLER_NOEC(traphandler9, 9)
TRAPHANDLER(traphandler10, 10)
TRAPHANDLER(traphandler11, 11)
TRAPHANDLER(traphandler12, 12)
TRAPHANDLER(traphandler13, 13)
TRAPHANDLER(traphandler14, 14)
TRAPHANDLER_NOEC(traphandler15, 15)
TRAPHANDLER_NOEC(traphandler16, 16)
TRAPHANDLER_NOEC(traphandler17, 17)
TRAPHANDLER_NOEC(traphandler18, 18)
TRAPHANDLER_NOEC(traphandler19, 19)
TRAPHANDLER_NOEC(traphandler20, 20)
TRAPHANDLER_NOEC(traphandler21, 21)
TRAPHANDLER_NOEC(traphandler22, 22)
TRAPHANDLER_NOEC(traphandler23, 23)
TRAPHANDLER_NOEC(traphandler24, 24)
TRAPHANDLER_NOEC(traphandler25, 25)
TRAPHANDLER_NOEC(traphandler26, 26)
TRAPHANDLER_NOEC(traphandler27, 27)
TRAPHANDLER_NOEC(traphandler28, 28)
TRAPHANDLER_NOEC(traphandler29, 29)
TRAPHANDLER_NOEC(traphandler30, 30)
TRAPHANDLER_NOEC(traphandler31, 31)
#endif

.data
.globl traphandlers
traphandlers:
	.long traphandler0
	.long traphandler1
	.long traphandler2
	.long traphandler3
	.long traphandler4
	.long traphandler5
	.long traphandler6
	.long traphandler7
	.long traphandler8
	.long traphandler9
	.long traphandler10
	.long traphandler11
	.long traphandler12
	.long traphandler13
	.long traphandler14
	.long traphandler15
	.long traphandler16
	.long traphandler17
	.long traphandler18
	.long traphandler19
	.long traphandler20
	.long traphandler21
	.long traphandler22
	.long traphandler23
	.long traphandler24
	.long traphandler25
	.long traphandler26
	.long traphandler27
	.long traphandler28
	.long traphandler29
	.long traphandler30
	.long traphandler31
