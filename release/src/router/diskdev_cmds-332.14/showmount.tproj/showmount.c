/*
 * Copyright (c) 1999 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * "Portions Copyright (c) 1999 Apple Computer, Inc.  All Rights
 * Reserved.  This file contains Original Code and/or Modifications of
 * Original Code as defined in and that are subject to the Apple Public
 * Source License Version 1.0 (the 'License').  You may not use this file
 * except in compliance with the License.  Please obtain a copy of the
 * License at http://www.apple.com/publicsource and read it before using
 * this file.
 * 
 * The Original Code and all software distributed under the License are
 * distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
 * License for the specific language governing rights and limitations
 * under the License."
 * 
 * @APPLE_LICENSE_HEADER_END@
 */
/*
 * Copyright (c) 1989, 1993, 1995
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Rick Macklem at The University of Guelph.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */


#include <sys/types.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/socketvar.h>

#include <netdb.h>
#include <rpc/rpc.h>
#include <rpc/pmap_clnt.h>
#include <rpc/pmap_prot.h>
#include <nfs/rpcv2.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "showmount.h"

struct mountlist *mntdump;
struct exportslist *exports;
int type = 0;
int rpcs = 0, mntvers = 1;

void	print_dump(struct mountlist *);
void	usage(void);
int	xdr_mntdump(XDR *, struct mountlist **);
int	xdr_exports(XDR *, struct exportslist **);
void	do_print(char *host);
extern void browse();

/*
 * This command queries the NFS mount daemon for it's mount list and/or
 * it's exports list and prints them out.
 * See "NFS: Network File System Protocol Specification, RFC1094, Appendix A"
 * and the "Network File System Protocol XXX.."
 * for detailed information on the protocol.
 */
int
main(argc, argv)
	int argc;
	char **argv;
{
	int ch;
	char *host;
	int do_browse = 0;

	while ((ch = getopt(argc, argv, "Aade3")) != EOF)
		switch((char)ch) {
		case 'A': do_browse = 1; break;
		case 'a':
			if (type == 0) {
				type = ALL;
				rpcs |= DODUMP;
			} else
				usage();
			break;
		case 'd':
			if (type == 0) {
				type = DIRS;
				rpcs |= DODUMP;
			} else
				usage();
			break;
		case 'e':
			rpcs |= DOEXPORTS;
			break;
		case '3':
			mntvers = 3;
			break;
		case '?':
		default:
			usage();
		}
	argc -= optind;
	argv += optind;

	if( do_browse ) {
		browse();
		exit(0);
	}

	if (argc > 0)
		host = *argv;
	else
		host = "localhost";

	do_print(host);
	exit(0);
}

/*
 * Xdr routine for retrieving the mount dump list
 */
int
xdr_mntdump(xdrsp, mlp)
	XDR *xdrsp;
	struct mountlist **mlp;
{
	struct mountlist *mp, **otp, *tp;
	int bool, val, val2;
	char *strp;

	*mlp = (struct mountlist *)0;
	if (!xdr_bool(xdrsp, &bool))
		return (0);
	while (bool) {
		mp = (struct mountlist *)malloc(sizeof(struct mountlist));
		if (mp == NULL)
			return (0);
		mp->ml_left = mp->ml_right = (struct mountlist *)0;
		strp = mp->ml_host;
		if (!xdr_string(xdrsp, &strp, RPCMNT_NAMELEN))
			return (0);
		strp = mp->ml_dirp;
		if (!xdr_string(xdrsp, &strp, RPCMNT_PATHLEN))
			return (0);

		/*
		 * Build a binary tree on sorted order of either host or dirp.
		 * Drop any duplications.
		 */
		if (*mlp == NULL) {
			*mlp = mp;
		} else {
			tp = *mlp;
			while (tp) {
				val = strcmp(mp->ml_host, tp->ml_host);
				val2 = strcmp(mp->ml_dirp, tp->ml_dirp);
				switch (type) {
				case ALL:
					if (val == 0) {
						if (val2 == 0) {
							free((caddr_t)mp);
							goto next;
						}
						val = val2;
					}
					break;
				case DIRS:
					if (val2 == 0) {
						free((caddr_t)mp);
						goto next;
					}
					val = val2;
					break;
				default:
					if (val == 0) {
						free((caddr_t)mp);
						goto next;
					}
					break;
				};
				if (val < 0) {
					otp = &tp->ml_left;
					tp = tp->ml_left;
				} else {
					otp = &tp->ml_right;
					tp = tp->ml_right;
				}
			}
			*otp = mp;
		}
next:
		if (!xdr_bool(xdrsp, &bool))
			return (0);
	}
	return (1);
}

/*
 * Xdr routine to retrieve exports list
 */
int
xdr_exports(xdrsp, exp)
	XDR *xdrsp;
	struct exportslist **exp;
{
	struct exportslist *ep;
	struct grouplist *gp;
	int bool, grpbool;
	char *strp;

	*exp = (struct exportslist *)0;
	if (!xdr_bool(xdrsp, &bool))
		return (0);
	while (bool) {
		ep = (struct exportslist *)malloc(sizeof(struct exportslist));
		if (ep == NULL)
			return (0);
		ep->ex_groups = (struct grouplist *)0;
		strp = ep->ex_dirp;
		if (!xdr_string(xdrsp, &strp, RPCMNT_PATHLEN))
			return (0);
		if (!xdr_bool(xdrsp, &grpbool))
			return (0);
		while (grpbool) {
			gp = (struct grouplist *)malloc(sizeof(struct grouplist));
			if (gp == NULL)
				return (0);
			strp = gp->gr_name;
			if (!xdr_string(xdrsp, &strp, RPCMNT_NAMELEN))
				return (0);
			gp->gr_next = ep->ex_groups;
			ep->ex_groups = gp;
			if (!xdr_bool(xdrsp, &grpbool))
				return (0);
		}
		ep->ex_next = *exp;
		*exp = ep;
		if (!xdr_bool(xdrsp, &bool))
			return (0);
	}
	return (1);
}

void
usage()
{

	fprintf(stderr, "usage: showmount [-ade] host\n");
	exit(1);
}

/*
 * Print the binary tree in inorder so that output is sorted.
 */
void
print_dump(mp)
	struct mountlist *mp;
{

	if (mp == NULL)
		return;
	if (mp->ml_left)
		print_dump(mp->ml_left);
	switch (type) {
	case ALL:
		printf("%s:%s\n", mp->ml_host, mp->ml_dirp);
		break;
	case DIRS:
		printf("%s\n", mp->ml_dirp);
		break;
	default:
		printf("%s\n", mp->ml_host);
		break;
	};
	if (mp->ml_right)
		print_dump(mp->ml_right);
}

void do_print(char *host)
{
	struct exportslist *exp;
	struct grouplist *grp;
	int estat;

	if (rpcs == 0)
		rpcs = DODUMP;

	if (rpcs & DODUMP)
		if ((estat = callrpc(host, RPCPROG_MNT, mntvers,
			RPCMNT_DUMP, xdr_void, (char *)0,
			xdr_mntdump, (char *)&mntdump)) != 0) {
			clnt_perrno(estat);
			fprintf(stderr, ": Can't do Mountdump rpc\n");
			exit(1);
		}
	if (rpcs & DOEXPORTS)
		if ((estat = callrpc(host, RPCPROG_MNT, mntvers,
			RPCMNT_EXPORT, xdr_void, (char *)0,
			xdr_exports, (char *)&exports)) != 0) {
			clnt_perrno(estat);
			fprintf(stderr, ": Can't do Exports rpc\n");
			exit(1);
		}

	/* Now just print out the results */
	if (rpcs & DODUMP) {
		switch (type) {
		case ALL:
			printf("All mount points on %s:\n", host);
			break;
		case DIRS:
			printf("Directories on %s:\n", host);
			break;
		default:
			printf("Hosts on %s:\n", host);
			break;
		};
		print_dump(mntdump);
	}
	if (rpcs & DOEXPORTS) {
		printf("Exports list on %s:\n", host);
		exp = exports;
		while (exp) {
			printf("%-35s", exp->ex_dirp);
			grp = exp->ex_groups;
			if (grp == NULL) {
				printf(" Everyone\n"); // allow space
			} else {
				while (grp) {
					printf("%s ", grp->gr_name);
					grp = grp->gr_next;
				}
				printf("\n");
			}
			exp = exp->ex_next;
		}
	}

}
