/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define CPUFREQ_TABLE_END ~1

static unsigned int transition_latency;
static DEFINE_PER_CPU(struct clk *, cpu_clks);
struct cpufreq_frequency_table *ftbl;


static struct cpufreq_frequency_table *cpufreq_parse_dt(struct device *dev,
						char *tbl_name, int cpu)
{
	u32 ret, nf, i;
	u32 *data;

	/* Parse list of usable CPU frequencies. */
	if (!of_find_property(dev->of_node, tbl_name, &nf))
		return ERR_PTR(-EINVAL);
	nf /= sizeof(*data);

	if (nf == 0)
		return ERR_PTR(-EINVAL);

	data = devm_kzalloc(dev, nf * sizeof(*data), GFP_KERNEL);
	if (!data)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32_array(dev->of_node, tbl_name, data, nf);
	if (ret)
		return ERR_PTR(ret);

	ftbl = devm_kzalloc(dev, (nf + 1) * sizeof(*ftbl), GFP_KERNEL);
	if (!ftbl)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < nf; i++) {
		unsigned long f;
		struct clk *cpu_clk;

		cpu_clk = per_cpu(cpu_clks, cpu);

		f = clk_round_rate(cpu_clk, data[i] * 1000);
		if (IS_ERR_VALUE(f))
			break;
		f /= 1000;

		/*
		 * Check if this is the last feasible frequency in the table.
		 *
		 * The table listing frequencies higher than what the HW can
		 * support is not an error since the table might be shared
		 * across CPUs in different speed bins. It's also not
		 * sufficient to check if the rounded rate is lower than the
		 * requested rate as it doesn't cover the following example:
		 *
		 * Table lists: 2.2 GHz and 2.5 GHz.
		 * Rounded rate returns: 2.2 GHz and 2.3 GHz.
		 *
		 * In this case, we can CPUfreq to use 2.2 GHz and 2.3 GHz
		 * instead of rejecting the 2.5 GHz table entry.
		 */
		if (i > 0 && f <= ftbl[i-1].frequency)
			break;

		ftbl[i].driver_data = i;
		ftbl[i].frequency = f;
	}

	ftbl[i].driver_data = i;
	ftbl[i].frequency = CPUFREQ_TABLE_END;

	devm_kfree(dev, data);

	return ftbl;
}

static int ipq40xx_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	int ret;
	unsigned int old_freq, new_freq;
	long freq_Hz, freq_exact;
	struct clk *cpu_clk;

	cpu_clk = per_cpu(cpu_clks, policy->cpu);
	freq_Hz = clk_round_rate(cpu_clk, ftbl[index].frequency * 1000);
	if (freq_Hz <= 0)
		freq_Hz = ftbl[index].frequency * 1000;

	freq_exact = freq_Hz;
	new_freq = freq_Hz;
	old_freq = clk_get_rate(cpu_clk);

	ret = clk_set_rate(cpu_clk, freq_exact);
	if (ret)
		pr_err("failed to set clock rate: %d\n", ret);

	return ret;
}

static unsigned int ipq40xx_get_target(unsigned int cpu)
{
	struct clk *cpu_clk;

	cpu_clk = per_cpu(cpu_clks, cpu);
	if (cpu_clk)
		return clk_get_rate(cpu_clk) / 1000;

	return 0;
}

static int ipq40xx_cpufreq_init(struct cpufreq_policy *policy)
{
	policy->clk = per_cpu(cpu_clks, policy->cpu);

	if (!ftbl) {
		pr_err("Freq table not initialized.\n");
		return -ENODEV;
	}
	cpufreq_frequency_table_get_attr(ftbl, 0);
	return cpufreq_generic_init(policy, ftbl, transition_latency);
}

static struct cpufreq_driver qca_ipq40xx_cpufreq_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = ipq40xx_set_target,
	.get = ipq40xx_get_target,
	.init = ipq40xx_cpufreq_init,
	.name = "ipq40xx_freq",
	.attr = cpufreq_generic_attr,
};

static int __init ipq40xx_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np = NULL;
	struct device *dev;
	struct clk *clk;
	unsigned int cpu;
	int ret;

	for_each_possible_cpu(cpu) {
		dev = get_cpu_device(cpu);
		if (!dev) {
			pr_err("failed to get A7 device\n");
			ret = -ENOENT;
			goto out_put_node;
		}
		per_cpu(cpu_clks, cpu) = clk = devm_clk_get(dev, NULL);
		if (IS_ERR(clk)) {
			pr_err("failed to get clk device\n");
			ret = PTR_ERR(clk);
			goto out_put_node;
		}
	}

	ftbl = cpufreq_parse_dt(&pdev->dev, "qcom,cpufreq-table", 0);

	np = of_node_get(pdev->dev.of_node);
	of_property_read_u32(np, "clock-latency", &transition_latency);

	if (!transition_latency) {
		pr_info("%s: Clock latency not found. Defaults...\n"
			, __func__);
		transition_latency = CPUFREQ_ETERNAL;
	}

	ret = cpufreq_register_driver(&qca_ipq40xx_cpufreq_driver);
	if (ret) {
		pr_err("failed register driver: %d\n", ret);
		goto out_put_node;
	}
	return 0;

out_put_node:
	of_node_put(np);
	return ret;
}

static int __exit ipq40xx_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&qca_ipq40xx_cpufreq_driver);
	return 0;
}

static struct of_device_id ipq40xx_match_table[] = {
	{.compatible = "qca,ipq40xx_freq"},
	{},
};

static struct platform_driver qca_ipq40xx_cpufreq_platdrv = {
	.probe		= ipq40xx_cpufreq_probe,
	.remove		= ipq40xx_cpufreq_remove,
	.driver = {
		.name	= "cpufreq-ipq40xx",
		.owner	= THIS_MODULE,
		.of_match_table = ipq40xx_match_table,
	},
};

module_platform_driver(qca_ipq40xx_cpufreq_platdrv);

MODULE_DESCRIPTION("QCA IPQ40XX CPUfreq driver");
