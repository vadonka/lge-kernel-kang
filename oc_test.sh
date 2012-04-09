#!/bin/bash

./gitdiff --srcbranch gb_3.0.y --destbranch remotes/hei1125/p99x-ics-3.0.y --output oc.patch --file arch/arm/mach-tegra/Kconfig.lge
./gitdiff --srcbranch gb_3.0.y --destbranch remotes/hei1125/p99x-ics-3.0.y --output oc.patch --file arch/arm/mach-tegra/nvrm/core/ap20/ap20rm_clock_config.c
./gitdiff --srcbranch gb_3.0.y --destbranch remotes/hei1125/p99x-ics-3.0.y --output oc.patch --file arch/arm/mach-tegra/nvrm/core/ap20/ap20rm_power_dfs.h
./gitdiff --srcbranch gb_3.0.y --destbranch remotes/hei1125/p99x-ics-3.0.y --output oc.patch --file arch/arm/mach-tegra/nvrm/core/common/nvrm_clocks_limits.c
./gitdiff --srcbranch gb_3.0.y --destbranch remotes/hei1125/p99x-ics-3.0.y --output oc.patch --file arch/arm/mach-tegra/nvrm/core/common/nvrm_power_dfs.c
./gitdiff --srcbranch gb_3.0.y --destbranch remotes/hei1125/p99x-ics-3.0.y --output oc.patch --file arch/arm/mach-tegra/odm_kit/star/adaptations/pmu/max8907/max8907.c
./gitdiff --srcbranch gb_3.0.y --destbranch remotes/hei1125/p99x-ics-3.0.y --output oc.patch --file arch/arm/mach-tegra/odm_kit/star/query/nvodm_query.c
./gitdiff --srcbranch gb_3.0.y --destbranch remotes/hei1125/p99x-ics-3.0.y --output oc.patch --file drivers/cpufreq/cpufreq.c
