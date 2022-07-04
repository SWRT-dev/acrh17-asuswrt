/*
 * Copyright (c) 1997 - 2001 Kungliga Tekniska Högskolan
 * (Royal Institute of Technology, Stockholm, Sweden).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/* $Id: parse_units.h,v 1.1.1.1 2011/06/10 09:34:42 andrew Exp $ */

#ifndef __PARSE_UNITS_H__
#define __PARSE_UNITS_H__

#include <stdio.h>
#include <stddef.h>

#ifndef ROKEN_LIB_FUNCTION
#ifdef _WIN32
#define ROKEN_LIB_FUNCTION _stdcall
#else
#define ROKEN_LIB_FUNCTION
#endif
#endif

struct units {
    const char *name;
    unsigned mult;
};

int ROKEN_LIB_FUNCTION
parse_units (const char *s, const struct units *units,
	     const char *def_unit);

void ROKEN_LIB_FUNCTION
print_units_table (const struct units *units, FILE *f);

int ROKEN_LIB_FUNCTION
parse_flags (const char *s, const struct units *units,
	     int orig);

int ROKEN_LIB_FUNCTION
unparse_units (int num, const struct units *units, char *s, size_t len);

int ROKEN_LIB_FUNCTION
unparse_units_approx (int num, const struct units *units, char *s,
		      size_t len);

int ROKEN_LIB_FUNCTION
unparse_flags (int num, const struct units *units, char *s, size_t len);

void ROKEN_LIB_FUNCTION
print_flags_table (const struct units *units, FILE *f);

#endif /* __PARSE_UNITS_H__ */
