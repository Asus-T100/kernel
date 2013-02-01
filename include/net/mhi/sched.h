/*
* File: mhi/sched.h
*
* MHI queue definitions
*
* Copyright (C) 2012 Renesas Mobile Corporation. All rights reserved.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. Â See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
 */

#ifndef MHI_SCHED_H
#define MHI_SCHED_H

#define MHI_NOTIFY_QUEUE_LOW     19
#define MHI_NOTIFY_QUEUE_HIGH    20

extern int
mhi_register_queue_notifier(struct Qdisc *sch,
			struct notifier_block *nb,
			unsigned long cl);

extern int
mhi_unregister_queue_notifier(struct Qdisc *sch,
			struct notifier_block *nb,
			unsigned long cl);

#endif /* MHI_SCHED_H */
