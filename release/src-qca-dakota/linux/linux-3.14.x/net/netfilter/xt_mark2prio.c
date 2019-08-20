/* Copyright (c) 2013,2016 The Linux Foundation. All rights reserved.
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
 */

#include <linux/module.h>
#include <linux/netfilter/x_tables.h>

/* do our -amazing- copy */
static unsigned int
mark2prio(struct sk_buff *skb, const struct xt_action_param *par)
{
	/* only override priority if it hasn't been set yet.
	 * this prevents mark2prio from stomping on the CLASSIFY target.
	 */
	if (skb->priority == 0)
		skb->priority = skb->mark;

	return NF_ACCEPT;
}

/* always return 0, because there is no reason we can't
 * copy skb->mark to skb->priority
 */
static int check_mark2prio(const struct xt_tgchk_param *par)
{
	return 0;
}

static struct xt_target mark2prio_target[] __read_mostly = {
	{
		.name		= "mark2prio",
		.family		= NFPROTO_IPV4,
		.target		= mark2prio,
		.targetsize	= 0,
		.table		= "mangle",
		.hooks		= (1 << NF_INET_POST_ROUTING),
		.checkentry	= check_mark2prio,
		.me		= THIS_MODULE,
	},
	{
		.name		= "mark2prio",
		.family		= NFPROTO_IPV6,
		.target		= mark2prio,
		.targetsize	= 0,
		.table		= "mangle",
		.hooks		= (1 << NF_INET_POST_ROUTING),
		.checkentry	= check_mark2prio,
		.me		= THIS_MODULE,
	},
};

static int __init mark2prio_init(void)
{
	return xt_register_targets(mark2prio_target,
				   ARRAY_SIZE(mark2prio_target));
}

static void __exit mark2prio_exit(void)
{
	xt_unregister_targets(mark2prio_target, ARRAY_SIZE(mark2prio_target));
}

module_init(mark2prio_init);
module_exit(mark2prio_exit);
