/*-
 * Copyright (c) 2005 Doug Rabson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	$FreeBSD: src/lib/libgssapi/gss_release_name.c,v 1.1 2005/12/29 14:40:20 dfr Exp $
 */

#include "mech_locl.h"
RCSID("$Id: gss_release_name.c,v 1.1.1.1 2011/06/10 09:34:42 andrew Exp $");

OM_uint32 GSSAPI_LIB_FUNCTION
gss_release_name(OM_uint32 *minor_status,
    gss_name_t *input_name)
{
	struct _gss_name *name;

	*minor_status = 0;

	if (input_name == NULL || *input_name == NULL)
	    return GSS_S_COMPLETE;

	name = (struct _gss_name *) *input_name;

	if (name->gn_type.elements)
		free(name->gn_type.elements);
	while (SLIST_FIRST(&name->gn_mn)) {
		struct _gss_mechanism_name *mn;
		mn = SLIST_FIRST(&name->gn_mn);
		SLIST_REMOVE_HEAD(&name->gn_mn, gmn_link);
		mn->gmn_mech->gm_release_name(minor_status,
					      &mn->gmn_name);
		free(mn);
	}
	gss_release_buffer(minor_status, &name->gn_value);
	free(name);
	*input_name = GSS_C_NO_NAME;

	return (GSS_S_COMPLETE);
}
