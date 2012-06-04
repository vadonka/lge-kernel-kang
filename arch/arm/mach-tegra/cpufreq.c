/*
 * arch/arm/mach-tegra/cpufreq.c
 *
 * cpufreq driver for NVIDIA Tegra SoCs
 *
 * Copyright (c) 2008-2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/smp.h>
#include <linux/cpu.h>
#include <linux/freezer.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/suspend.h>
// 20100728 related deepsleep wakeup delay, (NVIDIA john moser) [START]
#include <linux/delay.h>
// 20100728  related deepsleep wakeup delay, (NVIDIA john moser) [END]
#include <linux/reboot.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <asm/smp_twd.h>

#include <mach/hardware.h>
#include <mach/nvrm_linux.h>
#include <mach/timex.h>

#include <nvrm_power.h>
#include <nvrm_power_private.h>

#ifdef CONFIG_OVERCLOCK
#include "nvrm/core/common/nvrm_clocks_limits_private.h"
#include "nvrm/core/common/nvrm_power_dfs.h"
#include "nvrm/core/common/nvrm_chipid.h"
#endif /* OVERCLOCK END */

#define KTHREAD_IRQ_PRIO (MAX_RT_PRIO>>1)

#ifdef CONFIG_OVERCLOCK
#define DEFAULT_CPU_NOMINAL_MV (1300)

#define define_ro_attr(_name)            \
    static ssize_t show_##_name(struct cpufreq_policy *policy,  \
                char *buf);  \
    static struct freq_attr _name =  __ATTR(_name, 0444,    \
                show_##_name, NULL)  \

#define define_rw_attr(_name)            \
    static ssize_t show_##_name(struct cpufreq_policy *policy,  \
                char *buf);  \
    static ssize_t store_##_name(struct cpufreq_policy *policy,  \
                const char *buf, size_t count);    \
    static struct freq_attr _name =  __ATTR(_name, 0664,    \
                show_##_name, store_##_name)
#endif /* OVERCLOCK END */

static NvRmDeviceHandle rm_cpufreq = NULL;
static struct task_struct *cpufreq_dfsd = NULL;
static struct clk *clk_cpu = NULL;

static DEFINE_MUTEX(init_mutex);

#ifdef CONFIG_HOTPLUG_CPU
static int disable_hotplug = 0;
extern atomic_t hotplug_policy;
#endif

static void tegra_cpufreq_hotplug(NvRmPmRequest req)
{
	int rc = 0;
#ifdef CONFIG_HOTPLUG_CPU
	unsigned int cpu;
	int policy = atomic_read(&hotplug_policy);

	smp_rmb();
	if (disable_hotplug)
		return;
	

	if (req & NvRmPmRequest_CpuOnFlag && (policy > 1 || !policy)) {
		struct cpumask m;

		cpumask_andnot(&m, cpu_present_mask, cpu_online_mask);
		cpu = cpumask_any(&m);

		if (cpu_present(cpu) && !cpu_online(cpu))
			rc = cpu_up(cpu);
	} else
	  if (req & NvRmPmRequest_CpuOffFlag && (policy < NR_CPUS || !policy)) {
		cpu = cpumask_any_but(cpu_online_mask, 0);

		if (cpu_present(cpu) && cpu_online(cpu))
			rc = cpu_down(cpu);
	}
#endif
	if (rc)
		pr_err("%s: error %d servicing hot plug request\n",
		       __func__, rc);
}

#ifdef CONFIG_OVERCLOCK
static struct cpufreq_frequency_table freq_table[] = {
  { 0, 216000 },
  { 1, 456000 },
  { 2, 760000 },
  { 3, 912000 },
  { 4, 1100000 },
  { 5, 1216000 },
  { 6, 1408000 },
  { 7, 1504000 },
  { 8, CPUFREQ_TABLE_END },
};

#define FT_SIZE ARRAY_SIZE(freq_table)

static int enforce_freq_table_bounds(struct cpufreq_policy *policy)
{
    unsigned int max_freq, min_freq;
    int ret;

    max_freq = policy->max;
    min_freq = policy->min;

    cpufreq_verify_within_limits(policy,
    freq_table[0].frequency, freq_table[FT_SIZE - 2].frequency);

    ret = 0;
    if (max_freq != policy->max || min_freq != policy->min)
        ret = 1;

    return ret;
}

extern NvRmCpuShmoo *ExposedCpuShmoo;
static int voltage_deltas[NVRM_VOLTAGE_STEPS] = { 0 };

define_ro_attr(cpu_temp);
define_ro_attr(cpu_voltage);
define_rw_attr(scaling_available_frequencies);
define_ro_attr(cpu_volt_max);
define_ro_attr(cpu_volt_min);
define_ro_attr(frequency_voltage_table);
define_rw_attr(scaling_step_freqs);
define_rw_attr(scaling_step_volts);
define_rw_attr(UV_mV_table);

static struct freq_attr *tegra_cpufreq_attrs[] = {
    &cpu_temp,
    &cpu_voltage,
    &scaling_available_frequencies,
    &cpu_volt_max,
    &cpu_volt_min,
    &frequency_voltage_table,
    &scaling_step_freqs,
    &scaling_step_volts,
    &UV_mV_table,
    NULL,
};

static int tegra_set_policy(struct cpufreq_policy *pol);

static ssize_t show_cpu_temp(struct cpufreq_policy *policy, char *buf)
{
    NvError e;
    int temperature;

    e = NvRmDiagGetTemperature(rm_cpufreq, NvRmTmonZoneId_Core,
                                                            &temperature);
    switch (e) {
    case NvSuccess:
            return scnprintf(buf, PAGE_SIZE, "%d\n", temperature);
    case NvError_Busy:
            return scnprintf(buf, PAGE_SIZE, "<unavailable>\n");
    default:
            return scnprintf(buf, PAGE_SIZE, "<unsupported>\n");
    }
}

static ssize_t show_cpu_voltage(struct cpufreq_policy *policy, char *buf)
{
    unsigned int low, cur;

    if (NvRmPrivIsCpuRailDedicated(rm_cpufreq) == NV_TRUE) {
        NvRmDfsGetLowVoltageThreshold(rm_cpufreq,
                    NvRmDfsVoltageRailId_Cpu, &low, &cur);
    } else {
        NvRmDfsGetLowVoltageThreshold(rm_cpufreq,
                    NvRmDfsVoltageRailId_Core, &low, &cur);
    }

    return scnprintf(buf, PAGE_SIZE, "%u\n", cur);
}

static ssize_t show_scaling_available_frequencies
                (struct cpufreq_policy *policy, char *buf)
{
    int i;
    ssize_t n, size;

    size = 0;
    for (i = 0; i < FT_SIZE - 1; i++) {
        n = scnprintf(buf + size, PAGE_SIZE - size, "%u ",
        freq_table[i].frequency);
        size += n;
    }
    n = scnprintf(buf + size, PAGE_SIZE - size, "\n");
    size += n;

    return size;
}

static ssize_t store_scaling_available_frequencies
                (struct cpufreq_policy *policy, const char *buf, size_t count)
{
    int i;
    ssize_t n, size;
    unsigned int *freqs_in;
    unsigned int f;

    freqs_in = kmalloc(sizeof(unsigned int) * FT_SIZE, GFP_TEMPORARY);
    if (freqs_in == NULL)
        return -ENOMEM;

    size = 0;
    for (i = 0; i < FT_SIZE - 1; i++) {
        if (size < count && buf[size] != '\n') {
            if (sscanf(buf + size, "%u%n",
                            freqs_in + i, &n) != 1) {
                size = -EINVAL;
                goto err_free;
            }
            size += n;
            continue;
        }
    freqs_in[i] = freq_table[i].frequency;
    }

    freqs_in[FT_SIZE - 1] = CPUFREQ_TABLE_END;

    f = min(ExposedCpuShmoo->pScaledCpuLimits->MaxKHzList
                            [ExposedCpuShmoo->ShmooVmaxIndex],
                                    policy->cpuinfo.max_freq);

    if (freqs_in[0] < policy->cpuinfo.min_freq ||
                            freqs_in[FT_SIZE - 2] > f) {
        size = -EINVAL;
        goto err_free;
    }

    for (i = 0; i < FT_SIZE - 2; i++) {
        if (freqs_in[i] > freqs_in[i + 1]) {
            size = -EINVAL;
            goto err_free;
        }
    }

    for (i = 0; i < FT_SIZE - 1; i++)
        freq_table[i].frequency = freqs_in[i];

    enforce_freq_table_bounds(policy);
    tegra_set_policy(policy);
    NvRmDvsForceUpdate(rm_cpufreq);
    size = count;

    err_free:
    kfree(freqs_in);
    return size;
}

static ssize_t show_cpu_volt_max(struct cpufreq_policy *policy, char *buf)
{
    unsigned int vmax, vmin;

    NvRmDvsGetCpuVoltageThresholds(rm_cpufreq, &vmin, &vmax);

    return scnprintf(buf, PAGE_SIZE, "%u\n", vmax);
}

static ssize_t show_cpu_volt_min(struct cpufreq_policy *policy, char *buf)
{
    unsigned int vmax, vmin;

    NvRmDvsGetCpuVoltageThresholds(rm_cpufreq, &vmin, &vmax);

    return scnprintf(buf, PAGE_SIZE, "%u\n", vmin);
}

static ssize_t show_frequency_voltage_table(struct cpufreq_policy *policy,
                                                                char *buf)
{
    int i;
    ssize_t n, size;

    size = 0;
    for (i = ExposedCpuShmoo->ShmooVmaxIndex; i >= 0; i--) {
        if (size >= PAGE_SIZE) {
            buf[PAGE_SIZE - 1] = '\n';
            break;
        }

        n = scnprintf(buf + size, PAGE_SIZE - size, "%u ",
                ExposedCpuShmoo->pScaledCpuLimits->MaxKHzList[i]);
        size += n;

        n = scnprintf(buf + size, PAGE_SIZE - size, "%u ",
                            ExposedCpuShmoo->ShmooVoltages[i] +
                                            voltage_deltas[i]);
        size += n;

        n = scnprintf(buf + size, PAGE_SIZE - size, "%u\n",
                            ExposedCpuShmoo->ShmooVoltages[i]);
        size += n;
    }

    return size;
}

static ssize_t show_scaling_step_freqs(struct cpufreq_policy *policy,
                                                            char *buf)
{
    int i;
    ssize_t n, size;

    size = 0;
    for (i = 0; i <= ExposedCpuShmoo->ShmooVmaxIndex; i++) {
        n = scnprintf(buf + size, PAGE_SIZE - size, "%u ",
                ExposedCpuShmoo->pScaledCpuLimits->MaxKHzList[i]);
        size += n;
    }
    n = scnprintf(buf + size, PAGE_SIZE - size, "\n");
    size += n;

    return size;
}

static ssize_t store_scaling_step_freqs(struct cpufreq_policy *policy,
                                            const char *buf, size_t count)
{
    int i;
    ssize_t n, size;
    unsigned int *freqs_in, *freqs_out;

    freqs_in = kmalloc(sizeof(unsigned int) *
        (ExposedCpuShmoo->ShmooVmaxIndex + 1), GFP_TEMPORARY);
    if (freqs_in == NULL)
        return -ENOMEM;

    freqs_out = (unsigned int *)
        (ExposedCpuShmoo->pScaledCpuLimits->MaxKHzList);

    size = 0;
    for (i = 0; i <= ExposedCpuShmoo->ShmooVmaxIndex; i++) {
        if (size < count && buf[size] != '\n') {
            if (sscanf(buf + size, "%u%n",
                        freqs_in + i, &n) != 1) {
                size = -EINVAL;
                goto err_free;
            }
            size += n;
            continue;
        }
    freqs_in[i] = freqs_out[i];
    }

    if (freqs_in[0] < NvRmPrivDfsGetMinKHz(NvRmDfsClockId_Cpu) ||
                        freqs_in[ExposedCpuShmoo->ShmooVmaxIndex] >
                        NvRmPrivDfsGetMaxKHz(NvRmDfsClockId_Cpu)) {
        size = -EINVAL;
        goto err_free;
    }

    for (i = 0; i < ExposedCpuShmoo->ShmooVmaxIndex; i++) {
        if (freqs_in[i] > freqs_in[i + 1]) {
            size = -EINVAL;
            goto err_free;
        }
    }

    memcpy(freqs_out, freqs_in, sizeof(unsigned int) *
                        (ExposedCpuShmoo->ShmooVmaxIndex + 1));

    for (i = 0; i < FT_SIZE - 1; i++) {
        if (freq_table[i].frequency >
                freqs_in[ExposedCpuShmoo->ShmooVmaxIndex])
            freq_table[i].frequency =
                freqs_in[ExposedCpuShmoo->ShmooVmaxIndex];
    }

    enforce_freq_table_bounds(policy);
    tegra_set_policy(policy);
    NvRmDvsForceUpdate(rm_cpufreq);
    size = count;

    err_free:
    kfree(freqs_in);
    return size;
}

static ssize_t show_scaling_step_volts(struct cpufreq_policy *policy,
                                                            char *buf)
{
    int i;
    ssize_t n, size;

    size = 0;
    for (i = 0; i <= ExposedCpuShmoo->ShmooVmaxIndex; i++) {
        n = scnprintf(buf + size, PAGE_SIZE - size, "%u ",
                            ExposedCpuShmoo->ShmooVoltages[i]);
        size += n;
    }
    n = scnprintf(buf + size, PAGE_SIZE - size, "\n");
    size += n;

    return size;
}

static ssize_t store_scaling_step_volts(struct cpufreq_policy *policy,
                                            const char *buf, size_t count)
{
    int i;
    ssize_t n, size;
    unsigned int v1, v2;
    unsigned int *volts_in, *volts_out;

    volts_in = kmalloc(sizeof(unsigned int) *
            (ExposedCpuShmoo->ShmooVmaxIndex + 1), GFP_TEMPORARY);

    if (volts_in == NULL)
        return -ENOMEM;

    volts_out = (unsigned int *)(ExposedCpuShmoo->ShmooVoltages);

    size = 0;
    for (i = 0; i <= ExposedCpuShmoo->ShmooVmaxIndex; i++) {
        if (size < count && buf[size] != '\n') {
            if (sscanf(buf + size, "%u%n",
                       volts_in + i, &n) != 1) {
                size = -EINVAL;
                goto err_free;
            }
            size += n;

            v1 = volts_in[i] % NVRM_CORE_RESOLUTION_MV;
            if (v1)
                volts_in[i] -= v1;

            continue;
        }

    volts_in[i] = volts_out[i];
    }

    NvRmDvsGetCpuVoltageThresholds(rm_cpufreq, &v1, &v2);
    if (volts_in[0] < v1 ||
            volts_in[ExposedCpuShmoo->ShmooVmaxIndex] > v2) {
        size = -EINVAL;
        goto err_free;
    }

    for (i = 0; i <= ExposedCpuShmoo->ShmooVmaxIndex; i++)
        voltage_deltas[i] += volts_out[i] - volts_in[i];

    memcpy(volts_out, volts_in, sizeof(unsigned int) *
                    (ExposedCpuShmoo->ShmooVmaxIndex + 1));

    NvRmDvsForceUpdate(rm_cpufreq);
    size = count;

    err_free:
    kfree(volts_in);
    return size;
}

static ssize_t show_UV_mV_table(struct cpufreq_policy *policy, char *buf)
{
    int i;
    ssize_t n, size;

    size = 0;
    for (i = ExposedCpuShmoo->ShmooVmaxIndex; i >= 0; i--) {
        n = scnprintf(buf + size, PAGE_SIZE - size, "%d ",
                                            voltage_deltas[i]);
        size += n;
    }

    n = scnprintf(buf + size, PAGE_SIZE - size, "\n");
    size += n;

    return size;
}

static ssize_t store_UV_mV_table(struct cpufreq_policy *policy,
                                    const char *buf, size_t count)
{
    int i;
    ssize_t n, size;
    int v;
    int *deltas_in;
    char *b;

    deltas_in = kmalloc(sizeof(int) *
        (ExposedCpuShmoo->ShmooVmaxIndex + 1), GFP_TEMPORARY);
    if (deltas_in == NULL)
        return -ENOMEM;

    b = kmalloc(PAGE_SIZE, GFP_TEMPORARY);
    if (b == NULL) {
        size = -ENOMEM;
        goto err_free_deltas;
    }

    size = 0;
    for (i = ExposedCpuShmoo->ShmooVmaxIndex; i >= 0; i--) {
        if (size < count && buf[size] != '\n') {
            if (sscanf(buf + size, "%d%n",
                        deltas_in + i, &n) != 1) {
                size = -EINVAL;
                goto err_free_all;
            }
            size += n;

            v = deltas_in[i] % NVRM_CORE_RESOLUTION_MV;
            if (v)
                deltas_in[i] -= v;

            continue;
        }
    deltas_in[i] = voltage_deltas[i];
    }

    size = 0;
    for (i = 0; i <= ExposedCpuShmoo->ShmooVmaxIndex; i++) {
        n = scnprintf(b + size, PAGE_SIZE - size, "%u ",
            ExposedCpuShmoo->ShmooVoltages[i] +
            voltage_deltas[i] - deltas_in[i]);
        size += n;
    }

    i = store_scaling_step_volts(policy, b, size);
    if (i == size)
        size = count;
    else
        size = i;

    err_free_all:
    kfree(b);

    err_free_deltas:
    kfree(deltas_in);
    return size;
}

#endif /* OVERCLOCK END */

#ifdef CONFIG_HOTPLUG_CPU
static int tegra_cpufreq_pm_notifier(struct notifier_block *nfb,
				     unsigned long event, void *data)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		disable_hotplug = 1;
		smp_wmb();
		break;
	case PM_POST_SUSPEND:
		disable_hotplug = 0;
		smp_wmb();
		break;
	default:
		pr_err("%s: unknown event %lu\n", __func__, event);
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}
#endif

static int dfs_reboot_notify(struct notifier_block *nb,
			     unsigned long event, void *data)
{
	switch (event) {
	case SYS_RESTART:
	case SYS_HALT:
	case SYS_POWER_OFF:
#ifdef CONFIG_OVERCLOCK
                NvRmDvsSetCpuVoltageThresholds(rm_cpufreq, 0,
                                        DEFAULT_CPU_NOMINAL_MV);
#endif /* OVERCLOCK END */
		/* Warm boot setting at max voltages works for any reboot */
		NvRmPrivDfsSuspend(NvOdmSocPowerState_DeepSleep);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block dfs_reboot_nb = {
	.notifier_call = dfs_reboot_notify,
	.next = NULL,
	.priority = 0
};

static int tegra_cpufreq_dfsd(void *arg)
{
	unsigned long rate, last_rate;
	NvRmPmRequest req = 0;

	BUG_ON(!clk_cpu);

	preset_lpj = loops_per_jiffy;
	rate = clk_get_rate(clk_cpu);
	last_rate = rate;

	NvRmDfsSetState(rm_cpufreq, NvRmDfsRunState_ClosedLoop);
	set_freezable();

	while (!kthread_should_stop() && !(req & NvRmPmRequest_ExitFlag)) {

		req = NvRmPrivPmThread();

		if (try_to_freeze())
			continue;

		tegra_cpufreq_hotplug(req);

#ifdef CONFIG_USE_ARM_TWD_PRESCALER
		rate = clk_get_rate(clk_cpu);
		if (rate != last_rate) {
			local_timer_rescale(rate / 1000);
			smp_wmb();
			on_each_cpu(twd_set_prescaler, NULL, true);
			last_rate = rate;
		}
#endif
	}
	pr_info("dvfs thead shutdown\n");

	return 0;
}

static int tegra_verify_speed(struct cpufreq_policy *policy)
{
#ifdef CONFIG_OVERCLOCK
	enforce_freq_table_bounds(policy);
#else
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
		policy->cpuinfo.max_freq);
#endif /* OVERCLOCK END */
	return 0;
}

static unsigned int tegra_get_speed(unsigned int cpu)
{
	unsigned long rate;

	rate = clk_get_rate(clk_cpu);
	return rate / 1000;
}

static int tegra_set_policy(struct cpufreq_policy *pol)
{
	NvError e = NvRmDfsSetCpuEnvelope(rm_cpufreq, pol->min, pol->max);

	if (e) {
		pr_err("%s: error 0x%08x \n", __func__, e);
		return -EINVAL;
	}
	return 0;
}

int tegra_start_dvfsd(void) {
	int rc = 0;
	static bool started = false;

	mutex_lock(&init_mutex);
	if (cpufreq_dfsd && !started) {
		wake_up_process(cpufreq_dfsd);
		started = true;
	} else
		rc = -ENOSYS;
	mutex_unlock(&init_mutex);

	return rc;
}

static int tegra_cpufreq_init_once(void)
{
	struct sched_param sp;
	int rc = 0;

	mutex_lock(&init_mutex);

	if (rm_cpufreq)
		goto clean;

	if (NvRmOpenNew(&rm_cpufreq)!=NvSuccess) {
		pr_err("%s: unable to open NvRm\n", __func__);
		rc = -ENOSYS;
		goto clean;
	}

#ifdef CONFIG_OVERCLOCK
	NvRmDvsSetCpuVoltageThresholdsToLimits(rm_cpufreq);
#endif /* OVERCLOCK END */

	clk_cpu = clk_get_sys(NULL, "cpu");
	if (IS_ERR(clk_cpu)) {
		rc = PTR_ERR(clk_cpu);
		clk_cpu = NULL;
		goto clean;
	}

	rc = register_reboot_notifier(&dfs_reboot_nb);
	if (rc) {
		pr_err("%s: unable to regsiter DVFS reboot notifier\n", __func__);
		goto clean;
	}

	cpufreq_dfsd = kthread_create(tegra_cpufreq_dfsd, NULL, "cpufreq-dvfsd");
	if (IS_ERR(cpufreq_dfsd)) {
		pr_err("%s: unable to start DVFS daemon\n", __func__);
		rc = PTR_ERR(cpufreq_dfsd);
		cpufreq_dfsd = NULL;
		goto clean;
	}

	sp.sched_priority = KTHREAD_IRQ_PRIO + 1;
	if (sched_setscheduler_nocheck(cpufreq_dfsd, SCHED_FIFO, &sp) < 0)
		pr_err("%s: unable to elevate DVFS daemon priority\n",__func__);

clean:
	if (rc) {
		if (rm_cpufreq)
			NvRmClose(rm_cpufreq);
		if (clk_cpu)
			clk_put(clk_cpu);
		clk_cpu = NULL;
		rm_cpufreq = NULL;
		unregister_reboot_notifier(&dfs_reboot_nb);
	}

	mutex_unlock(&init_mutex);
	return rc;
}

static int tegra_cpufreq_driver_init(struct cpufreq_policy *pol)
{
	NvRmDfsClockUsage usage;
	NvError e;
	int rc;

	rc = tegra_cpufreq_init_once();
	if (rc)
		return rc;

	e = NvRmDfsGetClockUtilization(rm_cpufreq, NvRmDfsClockId_Cpu, &usage);

	if (e != NvSuccess) {
		WARN_ON(1);
		return -ENXIO;
	}

	pr_debug("%s: min: %u max: %u current: %u\n",
		 __func__, usage.MinKHz, usage.MaxKHz, usage.CurrentKHz);

	pol->min = usage.LowCornerKHz;
	pol->max = usage.HighCornerKHz;
	pol->cur = usage.CurrentKHz;

	pol->cpuinfo.min_freq = usage.MinKHz;
	pol->cpuinfo.max_freq = usage.MaxKHz;
	pol->cpuinfo.transition_latency = 0;

#ifdef CONFIG_OVERCLOCK
	switch (NvRmPrivGetChipId(rm_cpufreq)->Id) {
	case 0x20:
		cpumask_copy(pol->cpus, cpu_possible_mask);
		break;
	default:
	break;
	}
#endif /* OVERCLOCK END */

	return 0;
}

static struct cpufreq_driver s_tegra_cpufreq_driver = {
	.flags		= CPUFREQ_CONST_LOOPS,
	.verify		= tegra_verify_speed,
	.setpolicy	= tegra_set_policy,
	.get		= tegra_get_speed,
	.init		= tegra_cpufreq_driver_init,
	.name		= "tegra_cpufreq",
	.owner		= THIS_MODULE,
#ifdef CONFIG_OVERCLOCK
	.attr		= tegra_cpufreq_attrs,
#endif /* OVERCLOCK END */
};

static int __init tegra_cpufreq_init(void)
{
#ifdef CONFIG_HOTPLUG_CPU
	pm_notifier(tegra_cpufreq_pm_notifier, 0);
#endif
	return cpufreq_register_driver(&s_tegra_cpufreq_driver);
}

static void __exit tegra_cpufreq_exit(void)
{
	kthread_stop(cpufreq_dfsd);
	clk_put(clk_cpu);
	unregister_reboot_notifier(&dfs_reboot_nb);

	cpufreq_unregister_driver(&s_tegra_cpufreq_driver);
}

MODULE_DESCRIPTION("CPU frequency driver for the Tegra SOC");
MODULE_LICENSE("GPL");
module_init(tegra_cpufreq_init);
module_exit(tegra_cpufreq_exit);
