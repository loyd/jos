/* See COPYRIGHT for copyright information. */

#include <inc/x86.h>
#include <inc/mmu.h>
#include <inc/error.h>
#include <inc/string.h>
#include <inc/assert.h>
#include <inc/elf.h>

#include <kern/env.h>
#include <kern/pmap.h>
#include <kern/trap.h>
#include <kern/monitor.h>
#include <kern/sched.h>
#include <kern/cpu.h>
#include <kern/kdebug.h>

#ifdef CONFIG_KSPACE
struct Env env_array[NENV];
struct Env *curenv = NULL;
struct Env *envs = env_array;		// All environments
#else
struct Env *envs = NULL;		// All environments
struct Env *curenv = NULL;		// The current env
#endif
static struct Env *env_free_list;	// Free environment list
					// (linked by Env->env_link)

#define ENVGENSHIFT	12		// >= LOGNENV

//extern unsigned int bootstacktop;

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
// Initialize the kernel virtual memory layout for environment e.
// Allocate a page directory, set e->env_pgdir accordingly,
// and initialize the kernel portion of the new environment's address space.
// Do NOT (yet) map anything into the user portion
// of the environment's virtual address space.
//
// Returns 0 on success, < 0 on error.  Errors include:
//	-E_NO_MEM if page directory or table could not be allocated.
//
static int
env_setup_vm(struct Env *e)
{
	struct PageInfo *p = NULL;

	// Allocate a page for the page directory
	if (!(p = page_alloc(ALLOC_ZERO)))
		return -E_NO_MEM;

	// In general, pp_ref is not maintained for
	// hysical pages mapped only above UTOP, but env_pgdir
	// is an exception -- you need to increment env_pgdir's
	// pp_ref for env_free to work correctly.
	++p->pp_ref;

	e->env_pgdir = (pde_t *)page2kva(p);
	memcpy(e->env_pgdir, kern_pgdir, PGSIZE);

	// UVPT maps the env's own page table read-only.
	// Permissions: kernel R, user R
	e->env_pgdir[PDX(UVPT)] = PADDR(e->env_pgdir) | PTE_P | PTE_U;

	return 0;
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
	int r;
	struct Env *e;

	if (!(e = env_free_list)) {
		return -E_NO_FREE_ENV;
	}

	// Allocate and set up the page directory for this environment.
	if ((r = env_setup_vm(e)) < 0)
		return r;

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
	e->env_type = ENV_TYPE_USER;
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
	e->env_tf.tf_ds = GD_UD | 3;
	e->env_tf.tf_es = GD_UD | 3;
	e->env_tf.tf_ss = GD_UD | 3;
	e->env_tf.tf_esp = USTACKTOP;
	e->env_tf.tf_cs = GD_UT | 3;
#endif

	e->env_tf.tf_eflags |= FL_IF;

	// You will set e->env_tf.tf_eip later.

	// Clear the page fault handler until user installs one.
	e->env_pgfault_upcall = 0;

	// Also clear the IPC receiving flag.
	e->env_ipc_recving = 0;

	// commit the allocation
	env_free_list = e->env_link;
	*newenv_store = e;

	cprintf("[%08x] new env %08x\n", curenv ? curenv->env_id : 0, e->env_id);
	return 0;
}

//
// Allocate len bytes of physical memory for environment env,
// and map it at virtual address va in the environment's address space.
// Does not zero or otherwise initialize the mapped pages in any way.
// Pages should be writable by user and kernel.
// Panic if any allocation attempt fails.
//
static void
region_alloc(struct Env *e, void *va, size_t len)
{
	void *from = ROUNDDOWN(va, PGSIZE);
	void *to = ROUNDUP(va + len, PGSIZE);

	for (void *it = from; it < to; it += PGSIZE) {
		struct PageInfo *page = page_alloc(0);

		if (!page) {
			panic("region_alloc: OOM");
		}

		if (page_insert(e->env_pgdir, page, it, PTE_W | PTE_U) == -E_NO_MEM) {
			panic("region_alloc: OOM");
		}
	}
}

#ifdef CONFIG_KSPACE
static void
bind_functions(struct Env *e, struct Elf *elf)
{
	if (!elf->e_shoff) {
		panic("Section header table is not found");
	}

	struct Secthdr *sh = (struct Secthdr *)((uint8_t *)elf + elf->e_shoff);

	for (int i = 0; i < elf->e_shnum; ++i) {
		if (sh[i].sh_type != ELF_SHT_SYMTAB) {
			continue;
		}

		int n = sh[i].sh_size / sh[i].sh_entsize;
		struct Elf32_Sym *sym = (struct Elf32_Sym *)((uint8_t *)elf + sh[i].sh_offset);
		char *strtab = (char *)((uint8_t *)elf + sh[sh[i].sh_link].sh_offset);

		for (; n --> 0; ++sym) {
			if (ELF32_ST_TYPE(sym->st_info) != STT_OBJECT) {
				continue;
			}

			const char *name = strtab + sym->st_name;
			uintptr_t addr = find_function(name);

			if (addr) {
				*((int *)sym->st_value) = addr;
			}
		}
	}
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
// Finally, this function maps one page for the program's initial stack.
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

	lcr3(PADDR(e->env_pgdir));

	for (; ph < eph; ph++) {
		if (ph->p_type != ELF_PROG_LOAD) {
			continue;
		}

		if (ph->p_filesz > ph->p_memsz) {
			goto invalid_elf;
		}

		region_alloc(e, (void *)ph->p_va, ph->p_memsz);
		memcpy((void *)ph->p_va, binary + ph->p_offset, ph->p_filesz);
		memset((void *)(ph->p_va + ph->p_filesz), 0, ph->p_memsz - ph->p_filesz);
	}

	lcr3(PADDR(kern_pgdir));

	// Now map one page for the program's initial stack
	// at virtual address USTACKTOP - PGSIZE.
	region_alloc(e, (void *)(USTACKTOP - PGSIZE), PGSIZE);

	e->env_tf.tf_eip = elf->e_entry;

#ifdef CONFIG_KSPACE
	bind_functions(e, elf);
#endif

	return;

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

	// If this is the file server (type == ENV_TYPE_FS) give it I/O privileges.
	// LAB 10: Your code here.
}

//
// Frees env e and all memory it uses.
//
void
env_free(struct Env *e)
{
#ifndef CONFIG_KSPACE
	pte_t *pt;
	uint32_t pdeno, pteno;
	physaddr_t pa;

	// If freeing the current environment, switch to kern_pgdir
	// before freeing the page directory, just in case the page
	// gets reused.
	if (e == curenv)
		lcr3(PADDR(kern_pgdir));
#endif

	// Note the environment's demise.
	cprintf("[%08x] free env %08x\n", curenv ? curenv->env_id : 0, e->env_id);

#ifndef CONFIG_KSPACE
	// Flush all mapped pages in the user portion of the address space
	static_assert(UTOP % PTSIZE == 0);
	for (pdeno = 0; pdeno < PDX(UTOP); pdeno++) {

		// only look at mapped page tables
		if (!(e->env_pgdir[pdeno] & PTE_P))
			continue;

		// find the pa and va of the page table
		pa = PTE_ADDR(e->env_pgdir[pdeno]);
		pt = (pte_t*) KADDR(pa);

		// unmap all PTEs in this page table
		for (pteno = 0; pteno <= PTX(~0); pteno++) {
			if (pt[pteno] & PTE_P)
				page_remove(e->env_pgdir, PGADDR(pdeno, pteno, 0));
		}

		// free the page table itself
		e->env_pgdir[pdeno] = 0;
		page_decref(pa2page(pa));
	}

	// free the page directory
	pa = PADDR(e->env_pgdir);
	e->env_pgdir = 0;
	page_decref(pa2page(pa));
#endif
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
	tf->tf_eflags |= FL_IF;

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
	__asm __volatile("movl %0,%%esp\n"
		"\tpopal\n"
		"\tpopl %%es\n"
		"\tpopl %%ds\n"
		"\taddl $0x8,%%esp\n" /* skip tf_trapno and tf_errcode */
		"\tiret"
		: : "g" (tf) : "memory");
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

	lcr3(PADDR(e->env_pgdir));
	env_pop_tf(&e->env_tf);
}

