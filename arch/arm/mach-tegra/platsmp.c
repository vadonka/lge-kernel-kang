/*
 *  linux/arch/arm/mach-tegra/platsmp.c
 *
 *  Copyright (C) 2002 ARM Ltd.
 *  All Rights Reserved
 *
 *  Copyright (C) 2009 Palm
 *  All Rights Reserved
 *
 *  Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <linux/cpu.h>
#include <linux/slab.h>

#include <asm/tlbflush.h>
#include <asm/smp_scu.h>
#include <asm/cpu.h>
#include <asm/mmu_context.h>
#include <asm/pgalloc.h>
#include <asm/hardware/gic.h>

#include <mach/iomap.h>

#include "power.h"

extern void tegra_secondary_startup(void);

static void __iomem *scu_base = IO_ADDRESS(TEGRA_ARM_PERIF_BASE);
extern void __cortex_a9_restore(void);
extern void __shut_off_mmu(void);

#ifdef CONFIG_HOTPLUG_CPU
extern void tegra_hotplug_startup(void);
#endif

static DECLARE_BITMAP(cpu_init_bits, CONFIG_NR_CPUS) __read_mostly;
const struct cpumask *const cpu_init_mask = to_cpumask(cpu_init_bits);
#define cpu_init_map (*(cpumask_t *)cpu_init_mask)

#define EVP_CPU_RESET_VECTOR \
	(IO_ADDRESS(TEGRA_EXCEPTION_VECTORS_BASE) + 0x100)
#define CLK_RST_CONTROLLER_CLK_CPU_CMPLX \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x4c)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x340)
#define CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR \
	(IO_ADDRESS(TEGRA_CLK_RESET_BASE) + 0x344)

unsigned long tegra_pgd_phys;  /* pgd used by hotplug & LP2 bootup */
#if defined(CONFIG_PM) || defined(CONFIG_HOTPLUG_CPU)
static pgd_t *tegra_pgd;
#endif
void *tegra_context_area = NULL;

void __cpuinit platform_secondary_init(unsigned int cpu)
{
#ifdef CONFIG_CPU_V7
	/* enable dynamic clock gating */
	unsigned int reg;
	asm volatile ("mrc p15, 0, %0, c15, c0, 0" : "=r" (reg) : : "cc");
	reg |= 1;
	asm volatile ("mcr p15, 0, %0, c15, c0, 0" : : "r" (reg) : "cc");
#endif

	gic_secondary_init(0);
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long boot_vector;
	u32 reg;

	/* set the reset vector to point to the secondary_startup routine */
#ifdef CONFIG_HOTPLUG_CPU
	if (cpumask_test_cpu(cpu, cpu_init_mask))
		boot_vector = virt_to_phys(tegra_hotplug_startup);
	else
#endif
		boot_vector = virt_to_phys(tegra_secondary_startup);

	smp_wmb();

	writel(boot_vector, EVP_CPU_RESET_VECTOR);

	/* enable cpu clock on cpu */
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg & ~(1<<(8+cpu)), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);

	reg = 0x1111<<cpu;
	writel(reg, CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR);

	/* unhalt the cpu */
	writel(0, IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + 0x14 + 0x8*(cpu-1));

	return 0;
}

/*
 * Initialise the CPU possible map early - this describes the CPUs
 * which may be present or become present in the system.
 */
void __init smp_init_cpus(void)
{
	unsigned int i, ncores = scu_get_core_count(scu_base);

	for (i = 0; i < ncores; i++)
		cpu_set(i, cpu_possible_map);

	set_smp_cross_call(gic_raise_softirq);
}

#if defined(CONFIG_PM) || defined(CONFIG_HOTPLUG_CPU)
static int create_suspend_pgtable(void)
{
	int i;
	pmd_t *pmd;
	/* arrays of virtual-to-physical mappings which must be
	 * present to safely boot hotplugged / LP2-idled CPUs.
	 * tegra_hotplug_startup (hotplug reset vector) is mapped
	 * VA=PA so that the translation post-MMU is the same as
	 * pre-MMU, IRAM is mapped VA=PA so that SDRAM self-refresh
	 * can safely disable the MMU */
	unsigned long addr_v[] = {
		PHYS_OFFSET,
		IO_IRAM_PHYS,
		(unsigned long)tegra_context_area,
		(unsigned long)virt_to_phys(tegra_hotplug_startup),
		(unsigned long)__cortex_a9_restore,
		(unsigned long)virt_to_phys(__shut_off_mmu),
	};
	unsigned long addr_p[] = {
		PHYS_OFFSET,
		IO_IRAM_PHYS,
		(unsigned long)virt_to_phys(tegra_context_area),
		(unsigned long)virt_to_phys(tegra_hotplug_startup),
		(unsigned long)virt_to_phys(__cortex_a9_restore),
		(unsigned long)virt_to_phys(__shut_off_mmu),
	};
	unsigned int flags = PMD_TYPE_SECT | PMD_SECT_AP_WRITE |
		PMD_SECT_WBWA | PMD_SECT_S;

	tegra_pgd = pgd_alloc(&init_mm);
	if (!tegra_pgd)
		return -ENOMEM;

	for (i=0; i<ARRAY_SIZE(addr_p); i++) {
		unsigned long v = addr_v[i];
		pmd = pmd_offset(tegra_pgd + pgd_index(v), v);
		*pmd = __pmd((addr_p[i] & PGDIR_MASK) | flags);
		flush_pmd_entry(pmd);
		outer_clean_range(__pa(pmd), __pa(pmd + 1));
	}

	tegra_pgd_phys = virt_to_phys(tegra_pgd);
	__cpuc_flush_dcache_area(&tegra_pgd_phys,
		sizeof(tegra_pgd_phys));
	outer_clean_range(__pa(&tegra_pgd_phys),
		__pa(&tegra_pgd_phys+1));

	__cpuc_flush_dcache_area(&tegra_context_area,
		sizeof(tegra_context_area));
	outer_clean_range(__pa(&tegra_context_area),
		__pa(&tegra_context_area+1));

	return 0;
}
#endif

void __init platform_smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int ncores = scu_get_core_count(scu_base);
	int i;

#if defined(CONFIG_PM) || defined(CONFIG_HOTPLUG_CPU)
	tegra_context_area = kzalloc(CONTEXT_SIZE_BYTES * ncores, GFP_KERNEL);

	if (tegra_context_area && create_suspend_pgtable()) {
		kfree(tegra_context_area);
		tegra_context_area = NULL;
	}
#endif

	/*
	 * Initialise the present map, which describes the set of CPUs
	 * actually populated at the present time.
	 */
	for (i = 0; i < max_cpus; i++)
		set_cpu_present(i, true);

		scu_enable(scu_base);
	}

#ifdef CONFIG_HOTPLUG_CPU

extern void vfp_sync_state(struct thread_info *thread);

void __cpuinit secondary_start_kernel(void);

int platform_cpu_kill(unsigned int cpu)
{
	unsigned int reg;

		do {
			reg = readl(CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET);
			cpu_relax();
		} while (!(reg & (1<<cpu)));

	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg | (1<<(8+cpu)), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);

	return 1;
}

void platform_cpu_die(unsigned int cpu)
{
#ifdef DEBUG
	unsigned int this_cpu = hard_smp_processor_id();

	if (cpu != this_cpu) {
		printk(KERN_CRIT "Eek! platform_cpu_die running on %u, should be %u\n",
			   this_cpu, cpu);
		BUG();
	}
#endif

	gic_cpu_exit(0);
	barrier();
	flush_cache_all();
	barrier();
	__cortex_a9_save(0);

	/* return happens from __cortex_a9_restore */
	barrier();
	writel(smp_processor_id(), EVP_CPU_RESET_VECTOR);
	/* cpu_die is called with preemption disabled, and jumps directly to
	 * secondary_start_kernel, which disables preemption (again), resulting
	 * in a scheduling while atomic BUG without this. */
	preempt_enable_no_resched();
}

int platform_cpu_disable(unsigned int cpu)
{
	/*
	 * we don't allow CPU 0 to be shutdown (it is still too special
	 * e.g. clock tick interrupts)
	 */
	if (unlikely(!tegra_context_area))
		return -ENXIO;

	return cpu == 0 ? -EPERM : 0;
}
#endif
