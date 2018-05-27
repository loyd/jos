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
TRAPHANDLER_NOEC(traphandler32, 32)
TRAPHANDLER_NOEC(traphandler33, 33)
TRAPHANDLER_NOEC(traphandler34, 34)
TRAPHANDLER_NOEC(traphandler35, 35)
TRAPHANDLER_NOEC(traphandler36, 36)
TRAPHANDLER_NOEC(traphandler37, 37)
TRAPHANDLER_NOEC(traphandler38, 38)
TRAPHANDLER_NOEC(traphandler39, 39)
TRAPHANDLER_NOEC(traphandler40, 40)
TRAPHANDLER_NOEC(traphandler41, 41)
TRAPHANDLER_NOEC(traphandler42, 42)
TRAPHANDLER_NOEC(traphandler43, 43)
TRAPHANDLER_NOEC(traphandler44, 44)
TRAPHANDLER_NOEC(traphandler45, 45)
TRAPHANDLER_NOEC(traphandler46, 46)
TRAPHANDLER_NOEC(traphandler47, 47)
TRAPHANDLER_NOEC(traphandler48, 48)
TRAPHANDLER_NOEC(traphandler49, 49)
TRAPHANDLER_NOEC(traphandler50, 50)
TRAPHANDLER_NOEC(traphandler51, 51)
TRAPHANDLER_NOEC(traphandler52, 52)
TRAPHANDLER_NOEC(traphandler53, 53)
TRAPHANDLER_NOEC(traphandler54, 54)
TRAPHANDLER_NOEC(traphandler55, 55)
TRAPHANDLER_NOEC(traphandler56, 56)
TRAPHANDLER_NOEC(traphandler57, 57)
TRAPHANDLER_NOEC(traphandler58, 58)
TRAPHANDLER_NOEC(traphandler59, 59)
TRAPHANDLER_NOEC(traphandler60, 60)
TRAPHANDLER_NOEC(traphandler61, 61)
TRAPHANDLER_NOEC(traphandler62, 62)
TRAPHANDLER_NOEC(traphandler63, 63)
TRAPHANDLER_NOEC(traphandler64, 64)
TRAPHANDLER_NOEC(traphandler65, 65)
TRAPHANDLER_NOEC(traphandler66, 66)
TRAPHANDLER_NOEC(traphandler67, 67)
TRAPHANDLER_NOEC(traphandler68, 68)
TRAPHANDLER_NOEC(traphandler69, 69)
TRAPHANDLER_NOEC(traphandler70, 70)
TRAPHANDLER_NOEC(traphandler71, 71)
TRAPHANDLER_NOEC(traphandler72, 72)
TRAPHANDLER_NOEC(traphandler73, 73)
TRAPHANDLER_NOEC(traphandler74, 74)
TRAPHANDLER_NOEC(traphandler75, 75)
TRAPHANDLER_NOEC(traphandler76, 76)
TRAPHANDLER_NOEC(traphandler77, 77)
TRAPHANDLER_NOEC(traphandler78, 78)
TRAPHANDLER_NOEC(traphandler79, 79)
TRAPHANDLER_NOEC(traphandler80, 80)
TRAPHANDLER_NOEC(traphandler81, 81)
TRAPHANDLER_NOEC(traphandler82, 82)
TRAPHANDLER_NOEC(traphandler83, 83)
TRAPHANDLER_NOEC(traphandler84, 84)
TRAPHANDLER_NOEC(traphandler85, 85)
TRAPHANDLER_NOEC(traphandler86, 86)
TRAPHANDLER_NOEC(traphandler87, 87)
TRAPHANDLER_NOEC(traphandler88, 88)
TRAPHANDLER_NOEC(traphandler89, 89)
TRAPHANDLER_NOEC(traphandler90, 90)
TRAPHANDLER_NOEC(traphandler91, 91)
TRAPHANDLER_NOEC(traphandler92, 92)
TRAPHANDLER_NOEC(traphandler93, 93)
TRAPHANDLER_NOEC(traphandler94, 94)
TRAPHANDLER_NOEC(traphandler95, 95)
TRAPHANDLER_NOEC(traphandler96, 96)
TRAPHANDLER_NOEC(traphandler97, 97)
TRAPHANDLER_NOEC(traphandler98, 98)
TRAPHANDLER_NOEC(traphandler99, 99)
TRAPHANDLER_NOEC(traphandler100, 100)
TRAPHANDLER_NOEC(traphandler101, 101)
TRAPHANDLER_NOEC(traphandler102, 102)
TRAPHANDLER_NOEC(traphandler103, 103)
TRAPHANDLER_NOEC(traphandler104, 104)
TRAPHANDLER_NOEC(traphandler105, 105)
TRAPHANDLER_NOEC(traphandler106, 106)
TRAPHANDLER_NOEC(traphandler107, 107)
TRAPHANDLER_NOEC(traphandler108, 108)
TRAPHANDLER_NOEC(traphandler109, 109)
TRAPHANDLER_NOEC(traphandler110, 110)
TRAPHANDLER_NOEC(traphandler111, 111)
TRAPHANDLER_NOEC(traphandler112, 112)
TRAPHANDLER_NOEC(traphandler113, 113)
TRAPHANDLER_NOEC(traphandler114, 114)
TRAPHANDLER_NOEC(traphandler115, 115)
TRAPHANDLER_NOEC(traphandler116, 116)
TRAPHANDLER_NOEC(traphandler117, 117)
TRAPHANDLER_NOEC(traphandler118, 118)
TRAPHANDLER_NOEC(traphandler119, 119)
TRAPHANDLER_NOEC(traphandler120, 120)
TRAPHANDLER_NOEC(traphandler121, 121)
TRAPHANDLER_NOEC(traphandler122, 122)
TRAPHANDLER_NOEC(traphandler123, 123)
TRAPHANDLER_NOEC(traphandler124, 124)
TRAPHANDLER_NOEC(traphandler125, 125)
TRAPHANDLER_NOEC(traphandler126, 126)
TRAPHANDLER_NOEC(traphandler127, 127)
TRAPHANDLER_NOEC(traphandler128, 128)
TRAPHANDLER_NOEC(traphandler129, 129)
TRAPHANDLER_NOEC(traphandler130, 130)
TRAPHANDLER_NOEC(traphandler131, 131)
TRAPHANDLER_NOEC(traphandler132, 132)
TRAPHANDLER_NOEC(traphandler133, 133)
TRAPHANDLER_NOEC(traphandler134, 134)
TRAPHANDLER_NOEC(traphandler135, 135)
TRAPHANDLER_NOEC(traphandler136, 136)
TRAPHANDLER_NOEC(traphandler137, 137)
TRAPHANDLER_NOEC(traphandler138, 138)
TRAPHANDLER_NOEC(traphandler139, 139)
TRAPHANDLER_NOEC(traphandler140, 140)
TRAPHANDLER_NOEC(traphandler141, 141)
TRAPHANDLER_NOEC(traphandler142, 142)
TRAPHANDLER_NOEC(traphandler143, 143)
TRAPHANDLER_NOEC(traphandler144, 144)
TRAPHANDLER_NOEC(traphandler145, 145)
TRAPHANDLER_NOEC(traphandler146, 146)
TRAPHANDLER_NOEC(traphandler147, 147)
TRAPHANDLER_NOEC(traphandler148, 148)
TRAPHANDLER_NOEC(traphandler149, 149)
TRAPHANDLER_NOEC(traphandler150, 150)
TRAPHANDLER_NOEC(traphandler151, 151)
TRAPHANDLER_NOEC(traphandler152, 152)
TRAPHANDLER_NOEC(traphandler153, 153)
TRAPHANDLER_NOEC(traphandler154, 154)
TRAPHANDLER_NOEC(traphandler155, 155)
TRAPHANDLER_NOEC(traphandler156, 156)
TRAPHANDLER_NOEC(traphandler157, 157)
TRAPHANDLER_NOEC(traphandler158, 158)
TRAPHANDLER_NOEC(traphandler159, 159)
TRAPHANDLER_NOEC(traphandler160, 160)
TRAPHANDLER_NOEC(traphandler161, 161)
TRAPHANDLER_NOEC(traphandler162, 162)
TRAPHANDLER_NOEC(traphandler163, 163)
TRAPHANDLER_NOEC(traphandler164, 164)
TRAPHANDLER_NOEC(traphandler165, 165)
TRAPHANDLER_NOEC(traphandler166, 166)
TRAPHANDLER_NOEC(traphandler167, 167)
TRAPHANDLER_NOEC(traphandler168, 168)
TRAPHANDLER_NOEC(traphandler169, 169)
TRAPHANDLER_NOEC(traphandler170, 170)
TRAPHANDLER_NOEC(traphandler171, 171)
TRAPHANDLER_NOEC(traphandler172, 172)
TRAPHANDLER_NOEC(traphandler173, 173)
TRAPHANDLER_NOEC(traphandler174, 174)
TRAPHANDLER_NOEC(traphandler175, 175)
TRAPHANDLER_NOEC(traphandler176, 176)
TRAPHANDLER_NOEC(traphandler177, 177)
TRAPHANDLER_NOEC(traphandler178, 178)
TRAPHANDLER_NOEC(traphandler179, 179)
TRAPHANDLER_NOEC(traphandler180, 180)
TRAPHANDLER_NOEC(traphandler181, 181)
TRAPHANDLER_NOEC(traphandler182, 182)
TRAPHANDLER_NOEC(traphandler183, 183)
TRAPHANDLER_NOEC(traphandler184, 184)
TRAPHANDLER_NOEC(traphandler185, 185)
TRAPHANDLER_NOEC(traphandler186, 186)
TRAPHANDLER_NOEC(traphandler187, 187)
TRAPHANDLER_NOEC(traphandler188, 188)
TRAPHANDLER_NOEC(traphandler189, 189)
TRAPHANDLER_NOEC(traphandler190, 190)
TRAPHANDLER_NOEC(traphandler191, 191)
TRAPHANDLER_NOEC(traphandler192, 192)
TRAPHANDLER_NOEC(traphandler193, 193)
TRAPHANDLER_NOEC(traphandler194, 194)
TRAPHANDLER_NOEC(traphandler195, 195)
TRAPHANDLER_NOEC(traphandler196, 196)
TRAPHANDLER_NOEC(traphandler197, 197)
TRAPHANDLER_NOEC(traphandler198, 198)
TRAPHANDLER_NOEC(traphandler199, 199)
TRAPHANDLER_NOEC(traphandler200, 200)
TRAPHANDLER_NOEC(traphandler201, 201)
TRAPHANDLER_NOEC(traphandler202, 202)
TRAPHANDLER_NOEC(traphandler203, 203)
TRAPHANDLER_NOEC(traphandler204, 204)
TRAPHANDLER_NOEC(traphandler205, 205)
TRAPHANDLER_NOEC(traphandler206, 206)
TRAPHANDLER_NOEC(traphandler207, 207)
TRAPHANDLER_NOEC(traphandler208, 208)
TRAPHANDLER_NOEC(traphandler209, 209)
TRAPHANDLER_NOEC(traphandler210, 210)
TRAPHANDLER_NOEC(traphandler211, 211)
TRAPHANDLER_NOEC(traphandler212, 212)
TRAPHANDLER_NOEC(traphandler213, 213)
TRAPHANDLER_NOEC(traphandler214, 214)
TRAPHANDLER_NOEC(traphandler215, 215)
TRAPHANDLER_NOEC(traphandler216, 216)
TRAPHANDLER_NOEC(traphandler217, 217)
TRAPHANDLER_NOEC(traphandler218, 218)
TRAPHANDLER_NOEC(traphandler219, 219)
TRAPHANDLER_NOEC(traphandler220, 220)
TRAPHANDLER_NOEC(traphandler221, 221)
TRAPHANDLER_NOEC(traphandler222, 222)
TRAPHANDLER_NOEC(traphandler223, 223)
TRAPHANDLER_NOEC(traphandler224, 224)
TRAPHANDLER_NOEC(traphandler225, 225)
TRAPHANDLER_NOEC(traphandler226, 226)
TRAPHANDLER_NOEC(traphandler227, 227)
TRAPHANDLER_NOEC(traphandler228, 228)
TRAPHANDLER_NOEC(traphandler229, 229)
TRAPHANDLER_NOEC(traphandler230, 230)
TRAPHANDLER_NOEC(traphandler231, 231)
TRAPHANDLER_NOEC(traphandler232, 232)
TRAPHANDLER_NOEC(traphandler233, 233)
TRAPHANDLER_NOEC(traphandler234, 234)
TRAPHANDLER_NOEC(traphandler235, 235)
TRAPHANDLER_NOEC(traphandler236, 236)
TRAPHANDLER_NOEC(traphandler237, 237)
TRAPHANDLER_NOEC(traphandler238, 238)
TRAPHANDLER_NOEC(traphandler239, 239)
TRAPHANDLER_NOEC(traphandler240, 240)
TRAPHANDLER_NOEC(traphandler241, 241)
TRAPHANDLER_NOEC(traphandler242, 242)
TRAPHANDLER_NOEC(traphandler243, 243)
TRAPHANDLER_NOEC(traphandler244, 244)
TRAPHANDLER_NOEC(traphandler245, 245)
TRAPHANDLER_NOEC(traphandler246, 246)
TRAPHANDLER_NOEC(traphandler247, 247)
TRAPHANDLER_NOEC(traphandler248, 248)
TRAPHANDLER_NOEC(traphandler249, 249)
TRAPHANDLER_NOEC(traphandler250, 250)
TRAPHANDLER_NOEC(traphandler251, 251)
TRAPHANDLER_NOEC(traphandler252, 252)
TRAPHANDLER_NOEC(traphandler253, 253)
TRAPHANDLER_NOEC(traphandler254, 254)
TRAPHANDLER_NOEC(traphandler255, 255)
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
	.long traphandler32
	.long traphandler33
	.long traphandler34
	.long traphandler35
	.long traphandler36
	.long traphandler37
	.long traphandler38
	.long traphandler39
	.long traphandler40
	.long traphandler41
	.long traphandler42
	.long traphandler43
	.long traphandler44
	.long traphandler45
	.long traphandler46
	.long traphandler47
	.long traphandler48
	.long traphandler49
	.long traphandler50
	.long traphandler51
	.long traphandler52
	.long traphandler53
	.long traphandler54
	.long traphandler55
	.long traphandler56
	.long traphandler57
	.long traphandler58
	.long traphandler59
	.long traphandler60
	.long traphandler61
	.long traphandler62
	.long traphandler63
	.long traphandler64
	.long traphandler65
	.long traphandler66
	.long traphandler67
	.long traphandler68
	.long traphandler69
	.long traphandler70
	.long traphandler71
	.long traphandler72
	.long traphandler73
	.long traphandler74
	.long traphandler75
	.long traphandler76
	.long traphandler77
	.long traphandler78
	.long traphandler79
	.long traphandler80
	.long traphandler81
	.long traphandler82
	.long traphandler83
	.long traphandler84
	.long traphandler85
	.long traphandler86
	.long traphandler87
	.long traphandler88
	.long traphandler89
	.long traphandler90
	.long traphandler91
	.long traphandler92
	.long traphandler93
	.long traphandler94
	.long traphandler95
	.long traphandler96
	.long traphandler97
	.long traphandler98
	.long traphandler99
	.long traphandler100
	.long traphandler101
	.long traphandler102
	.long traphandler103
	.long traphandler104
	.long traphandler105
	.long traphandler106
	.long traphandler107
	.long traphandler108
	.long traphandler109
	.long traphandler110
	.long traphandler111
	.long traphandler112
	.long traphandler113
	.long traphandler114
	.long traphandler115
	.long traphandler116
	.long traphandler117
	.long traphandler118
	.long traphandler119
	.long traphandler120
	.long traphandler121
	.long traphandler122
	.long traphandler123
	.long traphandler124
	.long traphandler125
	.long traphandler126
	.long traphandler127
	.long traphandler128
	.long traphandler129
	.long traphandler130
	.long traphandler131
	.long traphandler132
	.long traphandler133
	.long traphandler134
	.long traphandler135
	.long traphandler136
	.long traphandler137
	.long traphandler138
	.long traphandler139
	.long traphandler140
	.long traphandler141
	.long traphandler142
	.long traphandler143
	.long traphandler144
	.long traphandler145
	.long traphandler146
	.long traphandler147
	.long traphandler148
	.long traphandler149
	.long traphandler150
	.long traphandler151
	.long traphandler152
	.long traphandler153
	.long traphandler154
	.long traphandler155
	.long traphandler156
	.long traphandler157
	.long traphandler158
	.long traphandler159
	.long traphandler160
	.long traphandler161
	.long traphandler162
	.long traphandler163
	.long traphandler164
	.long traphandler165
	.long traphandler166
	.long traphandler167
	.long traphandler168
	.long traphandler169
	.long traphandler170
	.long traphandler171
	.long traphandler172
	.long traphandler173
	.long traphandler174
	.long traphandler175
	.long traphandler176
	.long traphandler177
	.long traphandler178
	.long traphandler179
	.long traphandler180
	.long traphandler181
	.long traphandler182
	.long traphandler183
	.long traphandler184
	.long traphandler185
	.long traphandler186
	.long traphandler187
	.long traphandler188
	.long traphandler189
	.long traphandler190
	.long traphandler191
	.long traphandler192
	.long traphandler193
	.long traphandler194
	.long traphandler195
	.long traphandler196
	.long traphandler197
	.long traphandler198
	.long traphandler199
	.long traphandler200
	.long traphandler201
	.long traphandler202
	.long traphandler203
	.long traphandler204
	.long traphandler205
	.long traphandler206
	.long traphandler207
	.long traphandler208
	.long traphandler209
	.long traphandler210
	.long traphandler211
	.long traphandler212
	.long traphandler213
	.long traphandler214
	.long traphandler215
	.long traphandler216
	.long traphandler217
	.long traphandler218
	.long traphandler219
	.long traphandler220
	.long traphandler221
	.long traphandler222
	.long traphandler223
	.long traphandler224
	.long traphandler225
	.long traphandler226
	.long traphandler227
	.long traphandler228
	.long traphandler229
	.long traphandler230
	.long traphandler231
	.long traphandler232
	.long traphandler233
	.long traphandler234
	.long traphandler235
	.long traphandler236
	.long traphandler237
	.long traphandler238
	.long traphandler239
	.long traphandler240
	.long traphandler241
	.long traphandler242
	.long traphandler243
	.long traphandler244
	.long traphandler245
	.long traphandler246
	.long traphandler247
	.long traphandler248
	.long traphandler249
	.long traphandler250
	.long traphandler251
	.long traphandler252
	.long traphandler253
	.long traphandler254
	.long traphandler255
