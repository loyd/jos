/* See COPYRIGHT for copyright information. */

#include <inc/x86.h>
#include <inc/mmu.h>
#include <inc/error.h>
#include <inc/string.h>
#include <inc/assert.h>
#include <inc/elf.h>

#include <kern/env.h>
#include <kern/trap.h>
#include <kern/monitor.h>
#include <kern/sched.h>
#include <kern/cpu.h>

struct Env env_array[NENV];
struct Env *curenv = NULL;
struct Env *envs = env_array;		// All environments
static struct Env *env_free_list = NULL;	// Free environment list
					// (linked by Env->env_link)

#define ENVGENSHIFT	12		// >= LOGNENV

extern unsigned int bootstacktop;

// Global descriptor table.
//
// Set up global descriptor table (GDT) with separate segments for
// kernel mode and user mode.  Segments serve many purposes on the x86.
// We don't use any of their memory-mapping capabilities, but we need
// them to switch privilege levels.
//
// The kernel and user segments are identical except for the DPL.
// To load the SS register, the CPL must equal the DPL.  Thus,
// we must duplicate the segments for the user and the kernel.
//
// In particular, the last argument to the SEG macro used in the
// definition of gdt specifies the Descriptor Privilege Level (DPL)
// of that descriptor: 0 for kernel and 3 for user.
//
struct Segdesc gdt[NCPU + 5] =
{
	// 0x0 - unused (always faults -- for trapping NULL far pointers)
	SEG_NULL,

	// 0x8 - kernel code segment
	[GD_KT >> 3] = SEG(STA_X | STA_R, 0x0, 0xffffffff, 0),

	// 0x10 - kernel data segment
	[GD_KD >> 3] = SEG(STA_W, 0x0, 0xffffffff, 0),

	// 0x18 - user code segment
	[GD_UT >> 3] = SEG(STA_X | STA_R, 0x0, 0xffffffff, 3),

	// 0x20 - user data segment
	[GD_UD >> 3] = SEG(STA_W, 0x0, 0xffffffff, 3),

	// Per-CPU TSS descriptors (starting from GD_TSS0) are initialized
	// in trap_init_percpu()
	[GD_TSS0 >> 3] = SEG_NULL
};

struct Pseudodesc gdt_pd = {
	sizeof(gdt) - 1, (unsigned long) gdt
};

//
// Converts an envid to an env pointer.
// If checkperm is set, the specified environment must be either the
// current environment or an immediate child of the current environment.
//
// RETURNS
//   0 on success, -E_BAD_ENV on error.
//   On success, sets *env_store to the environment.
//   On error, sets *env_store to NULL.
//
int
envid2env(envid_t envid, struct Env **env_store, bool checkperm)
{
	struct Env *e;

	// If envid is zero, return the current environment.
	if (envid == 0) {
		*env_store = curenv;
		return 0;
	}

	// Look up the Env structure via the index part of the envid,
	// then check the env_id field in that struct Env
	// to ensure that the envid is not stale
	// (i.e., does not refer to a _previous_ environment
	// that used the same slot in the envs[] array).
	e = &envs[ENVX(envid)];
	if (e->env_status == ENV_FREE || e->env_id != envid) {
		*env_store = 0;
		return -E_BAD_ENV;
	}

	// Check that the calling environment has legitimate permission
	// to manipulate the specified environment.
	// If checkperm is set, the specified environment
	// must be either the current environment
	// or an immediate child of the current environment.
	if (checkperm && e != curenv && e->env_parent_id != curenv->env_id) {
		*env_store = 0;
		return -E_BAD_ENV;
	}

	*env_store = e;
	return 0;
}

// Mark all environments in 'envs' as free, set their env_ids to 0,
// and insert them into the env_free_list.
// Make sure the environments are in the free list in the same order
// they are in the envs array (i.e., so that the first call to
// env_alloc() returns envs[0]).
//
void
env_init(void)
{
	// Set up envs array
	int n = NENV;

	while (n --> 0) {
		envs[n].env_status = ENV_FREE;
		envs[n].env_link = env_free_list;

		env_free_list = envs + n;
	}

	// Per-CPU part of the initialization
	env_init_percpu();
}

// Load GDT and segment descriptors.
void
env_init_percpu(void)
{
	lgdt(&gdt_pd);
	// The kernel never uses GS or FS, so we leave those set to
	// the user data segment.
	asm volatile("movw %%ax,%%gs" :: "a" (GD_UD|3));
	asm volatile("movw %%ax,%%fs" :: "a" (GD_UD|3));
	// The kernel does use ES, DS, and SS.  We'll change between
	// the kernel and user data segments as needed.
	asm volatile("movw %%ax,%%es" :: "a" (GD_KD));
	asm volatile("movw %%ax,%%ds" :: "a" (GD_KD));
	asm volatile("movw %%ax,%%ss" :: "a" (GD_KD));
	// Load the kernel text segment into CS.
	asm volatile("ljmp %0,$1f\n 1:\n" :: "i" (GD_KT));
	// For good measure, clear the local descriptor table (LDT),
	// since we don't use it.
	lldt(0);
}

//
// Allocates and initializes a new environment.
// On success, the new environment is stored in *newenv_store.
//
// Returns 0 on success, < 0 on failure.  Errors include:
//	-E_NO_FREE_ENV if all NENVS environments are allocated
//	-E_NO_MEM on memory exhaustion
//
int
env_alloc(struct Env **newenv_store, envid_t parent_id)
{
	int32_t generation;
	struct Env *e;

	if (!(e = env_free_list)) {
		return -E_NO_FREE_ENV;
	}

	// Generate an env_id for this environment.
	generation = (e->env_id + (1 << ENVGENSHIFT)) & ~(NENV - 1);
	if (generation <= 0)	// Don't create a negative env_id.
		generation = 1 << ENVGENSHIFT;
	e->env_id = generation | (e - envs);

	// Set the basic status variables.
	e->env_parent_id = parent_id;
#ifdef CONFIG_KSPACE
	e->env_type = ENV_TYPE_KERNEL;
#else
#endif
	e->env_status = ENV_RUNNABLE;
	e->env_runs = 0;

	// Clear out all the saved register state,
	// to prevent the register values
	// of a prior environment inhabiting this Env structure
	// from "leaking" into our new environment.
	memset(&e->env_tf, 0, sizeof(e->env_tf));

	// Set up appropriate initial values for the segment registers.
	// GD_UD is the user data (KD - kernel data) segment selector in the GDT, and
	// GD_UT is the user text (KT - kernel text) segment selector (see inc/memlayout.h).
	// The low 2 bits of each segment register contains the
	// Requestor Privilege Level (RPL); 3 means user mode, 0 - kernel mode.  When
	// we switch privilege levels, the hardware does various
	// checks involving the RPL and the Descriptor Privilege Level
	// (DPL) stored in the descriptors themselves.
#ifdef CONFIG_KSPACE
	e->env_tf.tf_ds = GD_KD | 0;
	e->env_tf.tf_es = GD_KD | 0;
	e->env_tf.tf_ss = GD_KD | 0;
	e->env_tf.tf_cs = GD_KT | 0;
	e->env_tf.tf_esp = 0x210000 + (e - envs) * 2 * PGSIZE;
#else
#endif
	// You will set e->env_tf.tf_eip later.

	// commit the allocation
	env_free_list = e->env_link;
	*newenv_store = e;

	cprintf("[%08x] new env %08x\n", curenv ? curenv->env_id : 0, e->env_id);
	return 0;
}

#ifdef CONFIG_KSPACE
static void
bind_functions(struct Env *e, struct Elf *elf)
{
	//find_function from kdebug.c should be used
	//LAB 3: Your code here.

	/*
	*((int *) 0x00231008) = (int) &cprintf;
	*((int *) 0x00221004) = (int) &sys_yield;
	*((int *) 0x00231004) = (int) &sys_yield;
	*((int *) 0x00241004) = (int) &sys_yield;
	*((int *) 0x0022100c) = (int) &sys_exit;
	*((int *) 0x00231010) = (int) &sys_exit;
	*((int *) 0x0024100c) = (int) &sys_exit;
	*/
}
#endif

//
// Set up the initial program binary, stack, and processor flags
// for a user process.
// This function is ONLY called during kernel initialization,
// before running the first environment.
//
// This function loads all loadable segments from the ELF binary image
// into the environment's user memory, starting at the appropriate
// virtual addresses indicated in the ELF program header.
// At the same time it clears to zero any portions of these segments
// that are marked in the program header as being mapped
// but not actually present in the ELF file - i.e., the program's bss section.
//
// All this is very similar to what our boot loader does, except the boot
// loader also needs to read the code from disk.  Take a look at
// boot/main.c to get ideas.
//
// load_icode panics if it encounters problems.
//  - How might load_icode fail?  What might be wrong with the given input?
//
static void
load_icode(struct Env *e, uint8_t *binary, size_t size)
{
	assert(e);
	assert(binary);

	struct Elf *elf = (struct Elf *)binary;

	if (elf->e_magic != ELF_MAGIC) {
		goto invalid_elf;
	}

	struct Proghdr *ph = (struct Proghdr *)(binary + elf->e_phoff);
	struct Proghdr *eph = ph + elf->e_phnum;

	for (; ph < eph; ph++) {
		if (ph->p_type != ELF_PROG_LOAD) {
			continue;
		}

		if (ph->p_filesz > ph->p_memsz) {
			goto invalid_elf;
		}

		memcpy((void *)ph->p_va, binary + ph->p_offset, ph->p_filesz);
		memset((void *)(ph->p_va + ph->p_filesz), 0, ph->p_memsz - ph->p_filesz);
	}

	e->env_tf.tf_eip = elf->e_entry;
	e->env_tf.tf_eflags = elf->e_flags;

	return;

#ifdef CONFIG_KSPACE
	// Uncomment this for task №5.
	//bind_functions();
#endif

invalid_elf:
	panic("Cannot load icode: invalid elf");
}

//
// Allocates a new env with env_alloc, loads the named elf
// binary into it with load_icode, and sets its env_type.
// This function is ONLY called during kernel initialization,
// before running the first user-mode environment.
// The new env's parent ID is set to 0.
//
void
env_create(uint8_t *binary, size_t size, enum EnvType type)
{
	int ret;
	struct Env *env;

	if ((ret = env_alloc(&env, 0))) {
		panic("Cannot create an env: %i", ret);
	}

	load_icode(env, binary, size);
	env->env_type = type;
}

//
// Frees env e and all memory it uses.
//
void
env_free(struct Env *e)
{
	// Note the environment's demise.
	cprintf("[%08x] free env %08x\n", curenv ? curenv->env_id : 0, e->env_id);

	// return the environment to the free list
	e->env_status = ENV_FREE;
	e->env_link = env_free_list;
	env_free_list = e;
}

//
// Frees environment e.
// If e was the current env, then runs a new environment (and does not return
// to the caller).
//
void
env_destroy(struct Env *e)
{
	env_free(e);

	if (e == curenv) {
		sched_yield();
	}

	// TODO: dead code?

	cprintf("Destroyed the only environment - nothing more to do!\n");
	while (1)
		monitor(NULL);
}

#ifdef CONFIG_KSPACE
void
csys_exit(void)
{
	env_destroy(curenv);
}

void
csys_yield(struct Trapframe *tf)
{
	memcpy(&curenv->env_tf, tf, sizeof(struct Trapframe));
	sched_yield();
}
#endif


//
// Restores the register values in the Trapframe with the 'ret' instruction.
// This exits the kernel and starts executing some environment's code.
//
// This function does not return.
//
void
env_pop_tf(struct Trapframe *tf)
{
#ifdef CONFIG_KSPACE
	static uintptr_t eip = 0;
	eip = tf->tf_eip;

	asm volatile (
		"mov %c[ebx](%[tf]), %%ebx \n\t"
		"mov %c[ecx](%[tf]), %%ecx \n\t"
		"mov %c[edx](%[tf]), %%edx \n\t"
		"mov %c[esi](%[tf]), %%esi \n\t"
		"mov %c[edi](%[tf]), %%edi \n\t"
		"mov %c[ebp](%[tf]), %%ebp \n\t"
		"mov %c[esp](%[tf]), %%esp \n\t"
		"pushl %c[eip](%[tf])	   \n\t"
		"pushl %c[eflags](%[tf])   \n\t"
		"mov %c[eax](%[tf]), %%eax \n\t"
		"popfl			   \n\t"
		"ret			   \n\t"
		:
		: [tf]"a"(tf),
		  [eip]"i"(offsetof(struct Trapframe, tf_eip)),
		  [eax]"i"(offsetof(struct Trapframe, tf_regs.reg_eax)),
		  [ebx]"i"(offsetof(struct Trapframe, tf_regs.reg_ebx)),
		  [ecx]"i"(offsetof(struct Trapframe, tf_regs.reg_ecx)),
		  [edx]"i"(offsetof(struct Trapframe, tf_regs.reg_edx)),
		  [esi]"i"(offsetof(struct Trapframe, tf_regs.reg_esi)),
		  [edi]"i"(offsetof(struct Trapframe, tf_regs.reg_edi)),
		  [ebp]"i"(offsetof(struct Trapframe, tf_regs.reg_ebp)),
		  [eflags]"i"(offsetof(struct Trapframe, tf_eflags)),
//		  [esp]"i"(offsetof(struct Trapframe, tf_regs.reg_oesp))
		  [esp]"i"(offsetof(struct Trapframe, tf_esp))
		: "cc", "memory", "ebx", "ecx", "edx", "esi", "edi" );
#else
#endif
	panic("BUG");  /* mostly to placate the compiler */
}

//
// Context switch from curenv to env e.
// Note: if this is the first call to env_run, curenv is NULL.
//
// This function does not return.
//
void
env_run(struct Env *e)
{
	assert(e);

#ifdef CONFIG_KSPACE
	cprintf("envrun %s: %d\n",
		e->env_status == ENV_RUNNING ? "RUNNING" :
		    e->env_status == ENV_RUNNABLE ? "RUNNABLE" : "(unknown)",
		ENVX(e->env_id));
#endif

	if (e->env_status != ENV_RUNNING) {
		assert(e->env_status == ENV_RUNNABLE);

		if (curenv && curenv->env_status == ENV_RUNNING) {
			curenv->env_status = ENV_RUNNABLE;
		}

		curenv = e;
		curenv->env_status = ENV_RUNNING;
		++curenv->env_runs;
	}

	env_pop_tf(&e->env_tf);
}

