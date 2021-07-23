/*
 * Copyright (c) 2000 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * "Portions Copyright (c) 2000 Apple Computer, Inc.  All Rights
 * Reserved.  This file contains Original Code and/or Modifications of
 * Original Code as defined in and that are subject to the Apple Public
 * Source License Version 1.1 (the 'License').  You may not use this file
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
 * Copyright (C) 1995, 1996, 1997 Wolfgang Solfrank
 * Copyright (c) 1995 Martin Husemann
 * Some structure declaration borrowed from Paul Popelka
 * (paulp@uts.amdahl.com), see /sys/msdosfs/ for reference.
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
 *	This product includes software developed by Martin Husemann
 *	and Wolfgang Solfrank.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <sys/cdefs.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <sys/param.h>

#include "ext.h"
#include "fsutil.h"

#define	SLOT_EMPTY	0x00		/* slot has never been used */
#define	SLOT_E5		0x05		/* the real value is 0xe5 */
#define	SLOT_DELETED	0xe5		/* file in this slot deleted */

#define	ATTR_NORMAL	0x00		/* normal file */
#define	ATTR_READONLY	0x01		/* file is readonly */
#define	ATTR_HIDDEN	0x02		/* file is hidden */
#define	ATTR_SYSTEM	0x04		/* file is a system file */
#define	ATTR_VOLUME	0x08		/* entry is a volume label */
#define	ATTR_DIRECTORY	0x10		/* entry is a directory name */
#define	ATTR_ARCHIVE	0x20		/* file is new or modified */

#define	ATTR_WIN95	0x0f		/* long name record */

/*
 * This is the format of the contents of the deTime field in the direntry
 * structure.
 * We don't use bitfields because we don't know how compilers for
 * arbitrary machines will lay them out.
 */
#define DT_2SECONDS_MASK	0x1F	/* seconds divided by 2 */
#define DT_2SECONDS_SHIFT	0
#define DT_MINUTES_MASK		0x7E0	/* minutes */
#define DT_MINUTES_SHIFT	5
#define DT_HOURS_MASK		0xF800	/* hours */
#define DT_HOURS_SHIFT		11

/*
 * This is the format of the contents of the deDate field in the direntry
 * structure.
 */
#define DD_DAY_MASK		0x1F	/* day of month */
#define DD_DAY_SHIFT		0
#define DD_MONTH_MASK		0x1E0	/* month */
#define DD_MONTH_SHIFT		5
#define DD_YEAR_MASK		0xFE00	/* year - 1980 */
#define DD_YEAR_SHIFT		9


/* dir.c */
static struct dosDirEntry *newDosDirEntry __P((void));
static void freeDosDirEntry __P((struct dosDirEntry *));
static struct dirTodoNode *newDirTodo __P((void));
static void freeDirTodo __P((struct dirTodoNode *));
static char *fullpath __P((struct dosDirEntry *));
static u_char calcShortSum __P((u_char *));
static int delete __P((int, struct bootblock *, struct fatEntry *, cl_t, int,
    cl_t, int, int));
static int removede __P((int, struct bootblock *, struct fatEntry *, u_char *,
    u_char *, cl_t, cl_t, cl_t, char *, int));
static int checksize __P((struct bootblock *, struct fatEntry *, u_char *,
    struct dosDirEntry *));
static int readDosDirSection __P((int, struct bootblock *, struct fatEntry *,
    struct dosDirEntry *));

/*
 * Manage free dosDirEntry structures.
 */
static struct dosDirEntry *freede;

static struct dosDirEntry *
newDosDirEntry()
{
	struct dosDirEntry *de;

	if (!(de = freede)) {
		if (!(de = (struct dosDirEntry *)malloc(sizeof *de)))
			return 0;
	} else
		freede = de->next;
	return de;
}

static void
freeDosDirEntry(de)
	struct dosDirEntry *de;
{
	de->next = freede;
	freede = de;
}

/*
 * The same for dirTodoNode structures.
 */
static struct dirTodoNode *freedt;

static struct dirTodoNode *
newDirTodo()
{
	struct dirTodoNode *dt;

	if (!(dt = freedt)) {
		if (!(dt = (struct dirTodoNode *)malloc(sizeof *dt)))
			return 0;
	} else
		freedt = dt->next;
	return dt;
}

static void
freeDirTodo(dt)
	struct dirTodoNode *dt;
{
	dt->next = freedt;
	freedt = dt;
}

/*
 * The stack of unread directories
 */
struct dirTodoNode *pendingDirectories = NULL;

/*
 * Return the full pathname for a directory entry.
 */
static char *
fullpath(dir)
	struct dosDirEntry *dir;
{
	static char namebuf[MAXPATHLEN + 1];
	char *cp, *np;
	int nl;

	cp = namebuf + sizeof namebuf - 1;
	*cp = '\0';
	do {
		np = dir->lname[0] ? dir->lname : dir->name;
		nl = strlen(np);
		if ((cp -= nl) <= namebuf + 1)
			break;
		memcpy(cp, np, nl);
		*--cp = '/';
	} while ((dir = dir->parent) != NULL);
	if (dir)
		*--cp = '?';
	else
		cp++;
	return cp;
}

/*
 * Calculate a checksum over an 8.3 alias name
 */
static u_char
calcShortSum(p)
	u_char *p;
{
	u_char sum = 0;
	int i;

	for (i = 0; i < 11; i++) {
		sum = (sum << 7)|(sum >> 1);	/* rotate right */
		sum += p[i];
	}

	return sum;
}

/*
 * Global variables temporarily used during a directory scan
 */
static char longName[DOSLONGNAMELEN] = "";
static u_char *buffer = NULL;
static u_char *delbuf = NULL;

struct dosDirEntry *rootDir;
static struct dosDirEntry *lostDir;

/*
 * Init internal state for a new directory scan.
 */
int
resetDosDirSection(boot, fat)
	struct bootblock *boot;
	struct fatEntry *fat;
{
	int b1, b2;
	cl_t cl;
	int ret = FSOK;

	b1 = boot->RootDirEnts * 32;
	b2 = boot->SecPerClust * boot->BytesPerSec;

	if (!(buffer = malloc(b1 > b2 ? b1 : b2))
	    || !(delbuf = malloc(b2))
	    || !(rootDir = newDosDirEntry())) {
		perr("No space for directory");
		return FSFATAL;
	}
	memset(rootDir, 0, sizeof *rootDir);
	if (boot->flags & FAT32) {
		if (boot->RootCl < CLUST_FIRST || boot->RootCl >= boot->NumClusters) {
			pfatal("Root directory starts with cluster out of range(%u)\n",
			       boot->RootCl);
			return FSFATAL;
		}
		cl = fat[boot->RootCl].next;
		if (cl < CLUST_FIRST
		    || (cl >= CLUST_RSRVD && cl< CLUST_EOFS)
		    || fat[boot->RootCl].head != boot->RootCl) {
			if (cl == CLUST_FREE)
				pwarn("Root directory starts with free cluster\n");
			else if (cl >= CLUST_RSRVD)
				pwarn("Root directory starts with cluster marked %s\n",
				      rsrvdcltype(cl));
			else {
				pfatal("Root directory doesn't start a cluster chain\n");
				return FSFATAL;
			}
			if (ask(1, "Fix")) {
				fat[boot->RootCl].next = CLUST_FREE;
				ret = FSFATMOD;
			} else
				ret = FSFATAL;
		}

		fat[boot->RootCl].in_use = 1;
		rootDir->head = boot->RootCl;
	}

	return ret;
}

/*
 * Cleanup after a directory scan
 */
void
finishDosDirSection()
{
	struct dirTodoNode *p, *np;
	struct dosDirEntry *d, *nd;

	for (p = pendingDirectories; p; p = np) {
		np = p->next;
		freeDirTodo(p);
	}
	pendingDirectories = 0;
	for (d = rootDir; d; d = nd) {
		if ((nd = d->child) != NULL) {
			d->child = 0;
			continue;
		}
		if (!(nd = d->next))
			nd = d->parent;
		freeDosDirEntry(d);
	}
	rootDir = lostDir = NULL;
	free(buffer);
	free(delbuf);
	buffer = NULL;
	delbuf = NULL;
}

/*
 * Delete directory entries between startcl, startoff and endcl, endoff.
 */
static int
delete(f, boot, fat, startcl, startoff, endcl, endoff, notlast)
	int f;
	struct bootblock *boot;
	struct fatEntry *fat;
	cl_t startcl;
	int startoff;
	cl_t endcl;
	int endoff;
	int notlast;
{
	u_char *s, *e;
	off_t off;
	int clsz = boot->SecPerClust * boot->BytesPerSec;

	s = delbuf + startoff;
	e = delbuf + clsz;
	while (startcl >= CLUST_FIRST && startcl < boot->NumClusters) {
		if (startcl == endcl) {
			if (notlast)
				break;
			e = delbuf + endoff;
		}
		off = startcl * boot->SecPerClust + boot->ClusterOffset;
		off *= boot->BytesPerSec;
		if (lseek(f, off, SEEK_SET) != off
		    || read(f, delbuf, clsz) != clsz) {
			perr("Unable to read directory");
			return FSFATAL;
		}
		while (s < e) {
			*s = SLOT_DELETED;
			s += 32;
		}
		if (lseek(f, off, SEEK_SET) != off
		    || write(f, delbuf, clsz) != clsz) {
			perr("Unable to write directory");
			return FSFATAL;
		}
		if (startcl == endcl)
			break;
		startcl = fat[startcl].next;
		s = delbuf;
	}
	return FSOK;
}

static int
removede(f, boot, fat, start, end, startcl, endcl, curcl, path, type)
	int f;
	struct bootblock *boot;
	struct fatEntry *fat;
	u_char *start;
	u_char *end;
	cl_t startcl;
	cl_t endcl;
	cl_t curcl;
	char *path;
	int type;
{
	switch (type) {
	case 0:
		pwarn("Invalid long filename entry for %s\n", path);
		break;
	case 1:
		pwarn("Invalid long filename entry at end of directory %s\n", path);
		break;
	case 2:
		pwarn("Invalid long filename entry for volume label\n");
		break;
	}
	if (ask(0, "Remove")) {
		if (startcl != curcl) {
			if (delete(f, boot, fat,
				   startcl, start - buffer,
				   endcl, end - buffer,
				   endcl == curcl) == FSFATAL)
				return FSFATAL;
			start = buffer;
		}
		if (endcl == curcl)
			for (; start < end; start += 32)
				*start = SLOT_DELETED;
		return FSDIRMOD;
	}
	return FSERROR;
}

/*
 * Check an in-memory file entry
 */
static int
checksize(boot, fat, p, dir)
	struct bootblock *boot;
	struct fatEntry *fat;
	u_char *p;
	struct dosDirEntry *dir;
{
	/*
	 * Check size on ordinary files
	 */
	int32_t physicalSize;

	if (dir->head == CLUST_FREE)
		physicalSize = 0;
	else {
		if (dir->head < CLUST_FIRST || dir->head >= boot->NumClusters)
			return FSERROR;
		physicalSize = chainlength(boot, fat, dir->head) * boot->ClusterSize;
	}
	if (physicalSize < dir->size) {
		pwarn("size of %s is %u, should at most be %u\n",
		      fullpath(dir), dir->size, physicalSize);
		if (ask(1, "Truncate")) {
			dir->size = physicalSize;
			p[28] = (u_char)physicalSize;
			p[29] = (u_char)(physicalSize >> 8);
			p[30] = (u_char)(physicalSize >> 16);
			p[31] = (u_char)(physicalSize >> 24);
			return FSDIRMOD;
		} else
			return FSERROR;
	} else if (physicalSize - dir->size >= boot->ClusterSize) {
		pwarn("%s has too many clusters allocated\n",
		      fullpath(dir));
		if (ask(1, "Drop superfluous clusters")) {
			cl_t cl;
			u_int32_t sz = 0;

			for (cl = dir->head; (sz += boot->ClusterSize) < dir->size;)
				cl = fat[cl].next;
			clearchain(boot, fat, fat[cl].next);
			fat[cl].next = CLUST_EOF;
			return FSFATMOD;
		} else
			return FSERROR;
	}
	return FSOK;
}

/*
 * Read a directory and
 *   - resolve long name records
 *   - enter file and directory records into the parent's list
 *   - push directories onto the todo-stack
 */
static int
readDosDirSection(f, boot, fat, dir)
	int f;
	struct bootblock *boot;
	struct fatEntry *fat;
	struct dosDirEntry *dir;
{
	struct dosDirEntry dirent, *d;
	u_char *p, *vallfn, *invlfn, *empty;
	off_t off;
	int i, j, k, last;
	cl_t cl, valcl = ~0, invcl = ~0, empcl = ~0;
	char *t;
	u_int lidx = 0;
	int shortSum;
	int mod = FSOK;
#define	THISMOD	0x8000			/* Only used within this routine */

	cl = dir->head;
	if (dir->parent && (cl < CLUST_FIRST || cl >= boot->NumClusters)) {
		/*
		 * Already handled somewhere else.
		 */
		return FSOK;
	}
	shortSum = -1;
	vallfn = invlfn = empty = NULL;
	do {
		if (!(boot->flags & FAT32) && !dir->parent) {
			last = boot->RootDirEnts * 32;
			off = boot->ResSectors + boot->FATs * boot->FATsecs;
		} else {
			last = boot->SecPerClust * boot->BytesPerSec;
			off = cl * boot->SecPerClust + boot->ClusterOffset;
		}

		off *= boot->BytesPerSec;
		if (lseek(f, off, SEEK_SET) != off
		    || read(f, buffer, last) != last) {
			perr("Unable to read directory");
			return FSFATAL;
		}
		last /= 32;
		/*
		 * Check `.' and `..' entries here?			XXX
		 */
		for (p = buffer, i = 0; i < last; i++, p += 32) {
			if (dir->fsckflags & DIREMPWARN) {
				*p = SLOT_EMPTY;
				continue;
			}

			if (*p == SLOT_EMPTY || *p == SLOT_DELETED) {
				if (*p == SLOT_EMPTY) {
					dir->fsckflags |= DIREMPTY;
					empty = p;
					empcl = cl;
				}
				continue;
			}

			if (dir->fsckflags & DIREMPTY) {
				if (!(dir->fsckflags & DIREMPWARN)) {
					pwarn("%s has entries after end of directory\n",
					      fullpath(dir));
					if (ask(1, "Truncate"))
						dir->fsckflags |= DIREMPWARN;
					else if (ask(0, "Extend")) {
						u_char *q;

						dir->fsckflags &= ~DIREMPTY;
						if (delete(f, boot, fat,
							   empcl, empty - buffer,
							   cl, p - buffer, 1) == FSFATAL)
							return FSFATAL;
						q = empcl == cl ? empty : buffer;
						for (; q < p; q += 32)
							*q = SLOT_DELETED;
						mod |= THISMOD|FSDIRMOD;
					} 
				}
				if (dir->fsckflags & DIREMPWARN) {
					*p = SLOT_DELETED;
					mod |= THISMOD|FSDIRMOD;
					continue;
				} else if (dir->fsckflags & DIREMPTY)
					mod |= FSERROR;
				empty = NULL;
			}

			if (p[11] == ATTR_WIN95) {
				if (*p & LRFIRST) {
					if (shortSum != -1) {
						if (!invlfn) {
							invlfn = vallfn;
							invcl = valcl;
						}
					}
					memset(longName, 0, sizeof longName);
					shortSum = p[13];
					vallfn = p;
					valcl = cl;
				} else if (shortSum != p[13]
					   || lidx != (*p & LRNOMASK)) {
					if (!invlfn) {
						invlfn = vallfn;
						invcl = valcl;
					}
					if (!invlfn) {
						invlfn = p;
						invcl = cl;
					}
					vallfn = NULL;
				}
				lidx = *p & LRNOMASK;
				t = longName + --lidx * 13;
				for (k = 1; k < 11 && t < longName + sizeof(longName); k += 2) {
					if (!p[k] && !p[k + 1])
						break;
					*t++ = p[k];
					/*
					 * Warn about those unusable chars in msdosfs here?	XXX
					 */
					if (p[k + 1])
						t[-1] = '?';
				}
				if (k >= 11)
					for (k = 14; k < 26 && t < longName + sizeof(longName); k += 2) {
						if (!p[k] && !p[k + 1])
							break;
						*t++ = p[k];
						if (p[k + 1])
							t[-1] = '?';
					}
				if (k >= 26)
					for (k = 28; k < 32 && t < longName + sizeof(longName); k += 2) {
						if (!p[k] && !p[k + 1])
							break;
						*t++ = p[k];
						if (p[k + 1])
							t[-1] = '?';
					}
				if (t >= longName + sizeof(longName)) {
					pwarn("long filename too long\n");
					if (!invlfn) {
						invlfn = vallfn;
						invcl = valcl;
					}
					vallfn = NULL;
				}
				if (p[26] | (p[27] << 8)) {
					pwarn("long filename record cluster start != 0\n");
					if (!invlfn) {
						invlfn = vallfn;
						invcl = cl;
					}
					vallfn = NULL;
				}
				continue;	/* long records don't carry further
						 * information */
			}

			/*
			 * This is a standard msdosfs directory entry.
			 */
			memset(&dirent, 0, sizeof dirent);

			/*
			 * it's a short name record, but we need to know
			 * more, so get the flags first.
			 */
			dirent.flags = p[11];

			/*
			 * Translate from 850 to ISO here		XXX
			 */
			for (j = 0; j < 8; j++)
				dirent.name[j] = p[j];
			dirent.name[8] = '\0';
			for (k = 7; k >= 0 && dirent.name[k] == ' '; k--)
				dirent.name[k] = '\0';
			if (dirent.name[k] != '\0')
				k++;
			if (dirent.name[0] == SLOT_E5)
				dirent.name[0] = 0xe5;

			if (dirent.flags & ATTR_VOLUME) {
				if (vallfn || invlfn) {
					mod |= removede(f, boot, fat,
							invlfn ? invlfn : vallfn, p,
							invlfn ? invcl : valcl, -1, 0,
							fullpath(dir), 2);
					vallfn = NULL;
					invlfn = NULL;
				}
				continue;
			}

			if (p[8] != ' ')
				dirent.name[k++] = '.';
			for (j = 0; j < 3; j++)
				dirent.name[k++] = p[j+8];
			dirent.name[k] = '\0';
			for (k--; k >= 0 && dirent.name[k] == ' '; k--)
				dirent.name[k] = '\0';

			if (vallfn && shortSum != calcShortSum(p)) {
				if (!invlfn) {
					invlfn = vallfn;
					invcl = valcl;
				}
				vallfn = NULL;
			}
			dirent.head = p[26] | (p[27] << 8);
			if (boot->ClustMask == CLUST32_MASK)
				dirent.head |= (p[20] << 16) | (p[21] << 24);
			dirent.size = p[28] | (p[29] << 8) | (p[30] << 16) | (p[31] << 24);
			if (vallfn) {
				strcpy(dirent.lname, longName);
				longName[0] = '\0';
				shortSum = -1;
			}

			dirent.parent = dir;
			dirent.next = dir->child;

			if (invlfn) {
				mod |= k = removede(f, boot, fat,
						    invlfn, vallfn ? vallfn : p,
						    invcl, vallfn ? valcl : cl, cl,
						    fullpath(&dirent), 0);
				if (mod & FSFATAL)
					return FSFATAL;
				if (vallfn
				    ? (valcl == cl && vallfn != buffer)
				    : p != buffer)
					if (k & FSDIRMOD)
						mod |= THISMOD;
			}

			vallfn = NULL; /* not used any longer */
			invlfn = NULL;

			if (dirent.size == 0 && !(dirent.flags & ATTR_DIRECTORY)) {
				if (dirent.head != 0) {
					pwarn("%s has clusters, but size 0\n",
					      fullpath(&dirent));
					if (ask(1, "Drop allocated clusters")) {
						p[26] = p[27] = 0;
						if (boot->ClustMask == CLUST32_MASK)
							p[20] = p[21] = 0;
						clearchain(boot, fat, dirent.head);
						dirent.head = 0;
						mod |= THISMOD|FSDIRMOD|FSFATMOD;
					} else
						mod |= FSERROR;
				}
			} else if (dirent.head == 0
				   && !strcmp(dirent.name, "..")
				   && dir->parent			/* XXX */
				   && !dir->parent->parent) {
				/*
				 *  Do nothing, the parent is the root
				 */
			} else if (dirent.head < CLUST_FIRST
				   || dirent.head >= boot->NumClusters
				   || fat[dirent.head].next == CLUST_FREE
				   || (fat[dirent.head].next >= CLUST_RSRVD
				       && fat[dirent.head].next < CLUST_EOFS)
				   || fat[dirent.head].head != dirent.head) {
				if (dirent.head == 0)
					pwarn("%s has no clusters\n",
					      fullpath(&dirent));
				else if (dirent.head < CLUST_FIRST
					 || dirent.head >= boot->NumClusters)
					pwarn("%s starts with cluster out of range(%u)\n",
					      fullpath(&dirent),
					      dirent.head);
				else if (fat[dirent.head].next == CLUST_FREE)
					pwarn("%s starts with free cluster\n",
					      fullpath(&dirent));
				else if (fat[dirent.head].next >= CLUST_RSRVD)
					pwarn("%s starts with cluster marked %s\n",
					      fullpath(&dirent),
					      rsrvdcltype(fat[dirent.head].next));
				else
					pwarn("%s doesn't start a cluster chain\n",
					      fullpath(&dirent));
				if (dirent.flags & ATTR_DIRECTORY) {
					if (ask(0, "Remove")) {
						*p = SLOT_DELETED;
						mod |= THISMOD|FSDIRMOD;
					} else
						mod |= FSERROR;
					continue;
				} else {
					if (ask(1, "Truncate")) {
						p[28] = p[29] = p[30] = p[31] = 0;
						p[26] = p[27] = 0;
						if (boot->ClustMask == CLUST32_MASK)
							p[20] = p[21] = 0;
						dirent.size = 0;
						mod |= THISMOD|FSDIRMOD;
					} else
						mod |= FSERROR;
				}
			}

			if (dirent.head >= CLUST_FIRST && dirent.head < boot->NumClusters)
				fat[dirent.head].in_use = 1;

			if (dirent.flags & ATTR_DIRECTORY) {
				/*
				 * gather more info for directories
				 */
				struct dirTodoNode *n;

				if (dirent.size) {
					pwarn("Directory %s has size != 0\n",
					      fullpath(&dirent));
					if (ask(1, "Correct")) {
						p[28] = p[29] = p[30] = p[31] = 0;
						dirent.size = 0;
						mod |= THISMOD|FSDIRMOD;
					} else
						mod |= FSERROR;
				}
				/*
				 * handle `.' and `..' specially
				 */
				if (strcmp(dirent.name, ".") == 0) {
					if (dirent.head != dir->head) {
						pwarn("`.' entry in %s has incorrect start cluster\n",
						      fullpath(dir));
						if (ask(1, "Correct")) {
							dirent.head = dir->head;
							p[26] = (u_char)dirent.head;
							p[27] = (u_char)(dirent.head >> 8);
							if (boot->ClustMask == CLUST32_MASK) {
								p[20] = (u_char)(dirent.head >> 16);
								p[21] = (u_char)(dirent.head >> 24);
							}
							mod |= THISMOD|FSDIRMOD;
						} else
							mod |= FSERROR;
					}
					continue;
				}
				if (strcmp(dirent.name, "..") == 0) {
					if (dir->parent) {		/* XXX */
						if (!dir->parent->parent) {
							if (dirent.head) {
								pwarn("`..' entry in %s has non-zero start cluster\n",
								      fullpath(dir));
								if (ask(1, "Correct")) {
									dirent.head = 0;
									p[26] = p[27] = 0;
									if (boot->ClustMask == CLUST32_MASK)
										p[20] = p[21] = 0;
									mod |= THISMOD|FSDIRMOD;
								} else
									mod |= FSERROR;
							}
						} else if (dirent.head != dir->parent->head) {
							pwarn("`..' entry in %s has incorrect start cluster\n",
							      fullpath(dir));
							if (ask(1, "Correct")) {
								dirent.head = dir->parent->head;
								p[26] = (u_char)dirent.head;
								p[27] = (u_char)(dirent.head >> 8);
								if (boot->ClustMask == CLUST32_MASK) {
									p[20] = (u_char)(dirent.head >> 16);
									p[21] = (u_char)(dirent.head >> 24);
								}
								mod |= THISMOD|FSDIRMOD;
							} else
								mod |= FSERROR;
						}
					}
					continue;
				}

				/* create directory tree node */
				if (!(d = newDosDirEntry())) {
					perr("No space for directory");
					return FSFATAL;
				}
				memcpy(d, &dirent, sizeof(struct dosDirEntry));
				/* link it into the tree */
				dir->child = d;

				/* Enter this directory into the todo list */
				if (!(n = newDirTodo())) {
					perr("No space for todo list");
					return FSFATAL;
				}
				n->next = pendingDirectories;
				n->dir = d;
				pendingDirectories = n;
			} else {
				mod |= k = checksize(boot, fat, p, &dirent);
				if (k & FSDIRMOD)
					mod |= THISMOD;
			}
			boot->NumFiles++;
		}
		if (mod & THISMOD) {
			last *= 32;
			if (lseek(f, off, SEEK_SET) != off
			    || write(f, buffer, last) != last) {
				perr("Unable to write directory");
				return FSFATAL;
			}
			mod &= ~THISMOD;
		}
	} while ((cl = fat[cl].next) >= CLUST_FIRST && cl < boot->NumClusters);
	if (invlfn || vallfn)
		mod |= removede(f, boot, fat,
				invlfn ? invlfn : vallfn, p,
				invlfn ? invcl : valcl, -1, 0,
				fullpath(dir), 1);
	return mod & ~THISMOD;
}

int
handleDirTree(dosfs, boot, fat)
	int dosfs;
	struct bootblock *boot;
	struct fatEntry *fat;
{
	int mod;

	mod = readDosDirSection(dosfs, boot, fat, rootDir);
	if (mod & FSFATAL)
		return FSFATAL;

	/*
	 * process the directory todo list
	 */
	while (pendingDirectories) {
		struct dosDirEntry *dir = pendingDirectories->dir;
		struct dirTodoNode *n = pendingDirectories->next;

		/*
		 * remove TODO entry now, the list might change during
		 * directory reads
		 */
		freeDirTodo(pendingDirectories);
		pendingDirectories = n;

		/*
		 * handle subdirectory
		 */
		mod |= readDosDirSection(dosfs, boot, fat, dir);
		if (mod & FSFATAL)
			return FSFATAL;
	}

	return mod;
}

/*
 * Try to reconnect a FAT chain into dir
 */
static u_char *lfbuf;
static cl_t lfcl;
static off_t lfoff;

int
reconnect(dosfs, boot, fat, head)
	int dosfs;
	struct bootblock *boot;
	struct fatEntry *fat;
	cl_t head;
{
	struct dosDirEntry d;
	u_char *p;

	if (!ask(1, "Reconnect"))
		return FSERROR;

	if (!lostDir) {
		for (lostDir = rootDir->child; lostDir; lostDir = lostDir->next) {
			if (!strcmp(lostDir->name, LOSTDIR))
				break;
		}
		if (!lostDir) {		/* Create LOSTDIR?		XXX */
			pwarn("No %s directory\n", LOSTDIR);
			return FSERROR;
		}
	}
	if (!lfbuf) {
		lfbuf = malloc(boot->ClusterSize);
		if (!lfbuf) {
			perr("No space for buffer");
			return FSFATAL;
		}
		p = NULL;
	} else
		p = lfbuf;
	while (1) {
		if (p)
			for (; p < lfbuf + boot->ClusterSize; p += 32)
				if (*p == SLOT_EMPTY
				    || *p == SLOT_DELETED)
					break;
		if (p && p < lfbuf + boot->ClusterSize)
			break;
		lfcl = p ? fat[lfcl].next : lostDir->head;
		if (lfcl < CLUST_FIRST || lfcl >= boot->NumClusters) {
			/* Extend LOSTDIR?				XXX */
			pwarn("No space in %s\n", LOSTDIR);
			return FSERROR;
		}
		lfoff = lfcl * boot->ClusterSize
		    + boot->ClusterOffset * boot->BytesPerSec;
		if (lseek(dosfs, lfoff, SEEK_SET) != lfoff
		    || read(dosfs, lfbuf, boot->ClusterSize) != boot->ClusterSize) {
			perr("could not read LOST.DIR");
			return FSFATAL;
		}
		p = lfbuf;
	}

	boot->NumFiles++;
	/* Ensure uniqueness of entry here!				XXX */
	memset(&d, 0, sizeof d);
	(void)snprintf(d.name, sizeof(d.name), "%u", head);
	d.flags = 0;
	d.head = head;
	d.size = chainlength(boot, fat, head) * boot->ClusterSize;

	memset(p, 0, 32);
	memset(p, ' ', 11);
	memcpy(p, d.name, strlen(d.name));
	p[26] = (u_char)d.head;
	p[27] = (u_char)(d.head >> 8);
	if (boot->ClustMask == CLUST32_MASK) {
		p[20] = (u_char)(d.head >> 16);
		p[21] = (u_char)(d.head >> 24);
	}
	p[28] = (u_char)d.size;
	p[29] = (u_char)(d.size >> 8);
	p[30] = (u_char)(d.size >> 16);
	p[31] = (u_char)(d.size >> 24);
	fat[head].in_use = 1;
	if (lseek(dosfs, lfoff, SEEK_SET) != lfoff
	    || write(dosfs, lfbuf, boot->ClusterSize) != boot->ClusterSize) {
		perr("could not write LOST.DIR");
		return FSFATAL;
	}
	return FSDIRMOD;
}

void
finishlf()
{
	if (lfbuf)
		free(lfbuf);
	lfbuf = NULL;
}
