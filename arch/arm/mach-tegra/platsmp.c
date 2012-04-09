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
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <linux/cpu.h>

#include <asm/cacheflush.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/localtimer.h>
#include <asm/tlbflush.h>
#include <asm/smp_scu.h>
#include <asm/cpu.h>
#include <asm/mmu_context.h>
#include <asm/pgalloc.h>

#include <mach/iomap.h>

#include "power.h"

extern void tegra_secondary_startup(void);

static DEFINE_SPINLOCK(boot_lock);
static void __iomem *scu_base = IO_ADDRESS(TEGRA_ARM_PERIF_BASE);
extern void __cortex_a9_restore(void);
extern void __shut_off_mmu(void);

#ifdef CONFIG_HOTPLUG_CPU
static DEFINE_PER_CPU(struct completion, cpu_killed);
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

	trace_hardirqs_off();
	gic_cpu_init(0, IO_ADDRESS(TEGRA_ARM_PERIF_BASE) + 0x100);
	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
#ifdef CONFIG_HOTPLUG_CPU
	cpu_set(cpu, cpu_init_map);
	INIT_COMPLETION(per_cpu(cpu_killed, cpu));
#endif
	spin_unlock(&boot_lock);
}

int __cpuinit boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long old_boot_vector;
	unsigned long boot_vector;
	unsigned long timeout;
	u32 reg;

	/*
	 * set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	/* set the reset vector to point to the secondary_startup routine */
#ifdef CONFIG_HOTPLUG_CPU
	if (cpumask_test_cpu(cpu, cpu_init_mask))
		boot_vector = virt_to_phys(tegra_hotplug_startup);
	else
#endif
		boot_vector = virt_to_phys(tegra_secondary_startup);

	smp_wmb();

	old_boot_vector = readl(EVP_CPU_RESET_VECTOR);
	writel(boot_vector, EVP_CPU_RESET_VECTOR);

	/* enable cpu clock on cpu */
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg & ~(1<<(8+cpu)), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);

	reg = 0x1111<<cpu;
	writel(reg, CLK_RST_CONTROLLER_RST_CPU_CMPLX_CLR);

	/* unhalt the cpu */
	writel(0, IO_ADDRESS(TEGRA_FLOW_CTRL_BASE) + 0x14 + 0x8*(cpu-1));

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout)) {
		if (readl(EVP_CPU_RESET_VECTOR) != boot_vector)
			break;
		udelay(10);
	}

	/* put the old boot vector back */
	writel(old_boot_vector, EVP_CPU_RESET_VECTOR);

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */
	spin_unlock(&boot_lock);

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

void __init smp_prepare_cpus(unsigned int max_cpus)
{
	unsigned int ncores = scu_get_core_count(scu_base);
	unsigned int cpu = smp_processor_id();
	int i;

	smp_store_cpu_info(cpu);

	/*
	 * are we trying to boot more cores than exist?
	 */
	if (max_cpus > ncores)
		max_cpus = ncores;

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

#ifdef CONFIG_HOTPLUG_CPU
	for_each_present_cpu(i) {
		init_completion(&per_cpu(cpu_killed, i));
	}
#endif

	/*
	 * Initialise the SCU if there are more than one CPU and let
	 * them know where to start. Note that, on modern versions of
	 * MILO, the "poke" doesn't actually do anything until each
	 * individual core is sent a soft interrupt to get it out of
	 * WFI
	 */
	if (max_cpus > 1) {
		percpu_timer_setup();
		scu_enable(scu_base);
	}
}

#ifdef CONFIG_HOTPLUG_CPU

extern void vfp_sync_state(struct thread_info *thread);

void __cpuinit secondary_start_kernel(void);

int platform_cpu_kill(unsigned int cpu)
{
	unsigned int reg;
	int e;

	e = wait_for_completion_timeout(&per_cpu(cpu_killed, cpu), 100);
	printk(KERN_NOTICE "CPU%u: %s shutdown\n", cpu, (e) ? "clean":"forced");

	if (e) {
		do {
			reg = readl(CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET);
			cpu_relax();
		} while (!(reg & (1<<cpu)));
	} else {
		writel(0x1111<<cpu, CLK_RST_CONTROLLER_RST_CPU_CMPLX_SET);
		/* put flow controller in WAIT_EVENT mode */
		writel(2<<29, IO_ADDRESS(TEGRA_FLOW_CTRL_BASE)+0x14 + 0x8*(cpu-1));
	}
	spin_lock(&boot_lock);
	reg = readl(CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	writel(reg | (1<<(8+cpu)), CLK_RST_CONTROLLER_CLK_CPU_CMPLX);
	spin_unlock(&boot_lock);
	return e;
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
	complete(&per_cpu(cpu_killed, cpu));
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

int mach_cpu_disable(unsigned int cpu)
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
