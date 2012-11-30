/* Module signature checker
 *
 * Copyright (C) 2012 Red Hat, Inc. All Rights Reserved.
 * Written by David Howells (dhowells@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version
 * 2 of the Licence, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include "module-internal.h"

/*
 * Verify the signature on a module.
 */
int mod_verify_sig(const void *mod, unsigned long modlen,
		   const void *sig, unsigned long siglen)
{
	return -ENOKEY;
}
