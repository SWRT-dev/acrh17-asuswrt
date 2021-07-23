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
 * Copyright (c) 1998 Robert Nordier
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <sys/param.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/disklabel.h>
#include <sys/mount.h>

#include <ctype.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int dkdisklabel __P((int fd, struct disklabel * lp));

#define MAXU16	  0xffff	/* maximum unsigned 16-bit quantity */
#define BPN	  4		/* bits per nibble */
#define NPB	  2		/* nibbles per byte */

#define DOSMAGIC  0xaa55	/* DOS magic number */
#define MINBPS	  128		/* minimum bytes per sector */
#define MAXSPC	  128		/* maximum sectors per cluster */
#define MAXNFT	  16		/* maximum number of FATs */
#define DEFBLK	  4096		/* default block size */
#define DEFBLK16  2048		/* default block size FAT16 */
#define DEFRDE	  512		/* default root directory entries */
#define RESFTE	  2		/* reserved FAT entries */

/*
 * [2873845]  FAT12 volumes can have 1..4084 clusters.  FAT16 can have
 * 4085..65524 clusters.  FAT32 is 65525 clusters or more.
 * Since many other implementations are off by 1, 2, 4, 8, 10, or 16,
 * Microsoft recommends staying at least 16 clusters away from these
 * boundary points.  They also recommend that FAT32 volumes avoid
 * making the bad cluster mark an allocatable cluster number.
 *
 * So, the minimum and maximum values listed below aren't the strict
 * limits (smaller or larger values may work on more robust implementations).
 * The limits below are safe limits that should be compatible with a
 * wide variety of implementations.
 */
#define MINCLS12  1		/* minimum FAT12 clusters */
#define MINCLS16  4085		/* minimum FAT16 clusters */
#define MINCLS32  65525		/* minimum FAT32 clusters */
#define MAXCLS12  4084 		/* maximum FAT12 clusters */
#define MAXCLS16  65524		/* maximum FAT16 clusters */
#define MAXCLS32  0x0FFFFFF5	/* maximum FAT32 clusters */

#define BACKUP_BOOT_SECTOR 6	/* Default location for backup boot sector on FAT32 */
#define FAT32_RESERVED_SECTORS 32

#define mincls(fat)  ((fat) == 12 ? MINCLS12 :	\
		      (fat) == 16 ? MINCLS16 :	\
				    MINCLS32)

#define maxcls(fat)  ((fat) == 12 ? MAXCLS12 :	\
		      (fat) == 16 ? MAXCLS16 :	\
				    MAXCLS32)

#define mk1(p, x)				\
    (p) = (u_int8_t)(x)

#define mk2(p, x)				\
    (p)[0] = (u_int8_t)(x),			\
    (p)[1] = (u_int8_t)((x) >> 010)

#define mk4(p, x)				\
    (p)[0] = (u_int8_t)(x),			\
    (p)[1] = (u_int8_t)((x) >> 010),		\
    (p)[2] = (u_int8_t)((x) >> 020),		\
    (p)[3] = (u_int8_t)((x) >> 030)

#define argto1(arg, lo, msg)  argtou(arg, lo, 0xff, msg)
#define argto2(arg, lo, msg)  argtou(arg, lo, 0xffff, msg)
#define argto4(arg, lo, msg)  argtou(arg, lo, 0xffffffff, msg)
#define argtox(arg, lo, msg)  argtou(arg, lo, UINT_MAX, msg)

struct bs {
    u_int8_t jmp[3];		/* bootstrap entry point */
    u_int8_t oem[8];		/* OEM name and version */
};

struct bsbpb {
    u_int8_t bps[2];		/* bytes per sector */
    u_int8_t spc;		/* sectors per cluster */
    u_int8_t res[2];		/* reserved sectors */
    u_int8_t nft;		/* number of FATs */
    u_int8_t rde[2];		/* root directory entries */
    u_int8_t sec[2];		/* total sectors */
    u_int8_t mid;		/* media descriptor */
    u_int8_t spf[2];		/* sectors per FAT */
    u_int8_t spt[2];		/* sectors per track */
    u_int8_t hds[2];		/* drive heads */
    u_int8_t hid[4];		/* hidden sectors */
    u_int8_t bsec[4];		/* big total sectors */
};

struct bsxbpb {
    u_int8_t bspf[4];		/* big sectors per FAT */
    u_int8_t xflg[2];		/* FAT control flags */
    u_int8_t vers[2];		/* file system version */
    u_int8_t rdcl[4];		/* root directory start cluster */
    u_int8_t infs[2];		/* file system info sector */
    u_int8_t bkbs[2];		/* backup boot sector */
    u_int8_t rsvd[12];		/* reserved */
};

struct bsx {
    u_int8_t drv;		/* drive number */
    u_int8_t rsvd;		/* reserved */
    u_int8_t sig;		/* extended boot signature */
    u_int8_t volid[4];		/* volume ID number */
    u_int8_t label[11]; 	/* volume label */
    u_int8_t type[8];		/* file system type */
};

struct de {
    u_int8_t namext[11];	/* name and extension */
    u_int8_t attr;		/* attributes */
    u_int8_t rsvd[10];		/* reserved */
    u_int8_t time[2];		/* creation time */
    u_int8_t date[2];		/* creation date */
    u_int8_t clus[2];		/* starting cluster */
    u_int8_t size[4];		/* size */
};

struct bpb {
    u_int bps;			/* bytes per sector */
    u_int spc;			/* sectors per cluster */
    u_int res;			/* reserved sectors */
    u_int nft;			/* number of FATs */
    u_int rde;			/* root directory entries */
    u_int sec;			/* total sectors */
    u_int mid;			/* media descriptor */
    u_int spf;			/* sectors per FAT */
    u_int spt;			/* sectors per track */
    u_int hds;			/* drive heads */
    u_int hid;			/* hidden sectors */
    u_int bsec; 		/* big total sectors */
    u_int bspf; 		/* big sectors per FAT */
    u_int rdcl; 		/* root directory start cluster */
    u_int infs; 		/* file system info sector */
    u_int bkbs; 		/* backup boot sector */
};

static struct {
    const char *name;
    struct bpb bpb;
} stdfmt[] = {
    {"160",  {512, 1, 1, 2,  64,  320, 0xfe, 1,  8, 1}},
    {"180",  {512, 1, 1, 2,  64,  360, 0xfc, 2,  9, 1}},
    {"320",  {512, 2, 1, 2, 112,  640, 0xff, 1,  8, 2}},
    {"360",  {512, 2, 1, 2, 112,  720, 0xfd, 2,  9, 2}},
    {"640",  {512, 2, 1, 2, 112, 1280, 0xfb, 2,  8, 2}},    
    {"720",  {512, 2, 1, 2, 112, 1440, 0xf9, 3,  9, 2}},
    {"1200", {512, 1, 1, 2, 224, 2400, 0xf9, 7, 15, 2}},
    {"1232", {1024,1, 1, 2, 192, 1232, 0xfe, 2,  8, 2}},    
    {"1440", {512, 1, 1, 2, 224, 2880, 0xf0, 9, 18, 2}},
    {"2880", {512, 2, 1, 2, 240, 5760, 0xf0, 9, 36, 2}}
};

static u_int8_t bootcode[] = {
    0xfa,			/* cli		    */
    0x31, 0xc0, 		/* xor	   ax,ax    */
    0x8e, 0xd0, 		/* mov	   ss,ax    */
    0xbc, 0x00, 0x7c,		/* mov	   sp,7c00h */
    0xfb,			/* sti		    */
    0x8e, 0xd8, 		/* mov	   ds,ax    */
    0xe8, 0x00, 0x00,		/* call    $ + 3    */
    0x5e,			/* pop	   si	    */
    0x83, 0xc6, 0x19,		/* add	   si,+19h  */
    0xbb, 0x07, 0x00,		/* mov	   bx,0007h */
    0xfc,			/* cld		    */
    0xac,			/* lodsb	    */
    0x84, 0xc0, 		/* test    al,al    */
    0x74, 0x06, 		/* jz	   $ + 8    */
    0xb4, 0x0e, 		/* mov	   ah,0eh   */
    0xcd, 0x10, 		/* int	   10h	    */
    0xeb, 0xf5, 		/* jmp	   $ - 9    */
    0x30, 0xe4, 		/* xor	   ah,ah    */
    0xcd, 0x16, 		/* int	   16h	    */
    0xcd, 0x19, 		/* int	   19h	    */
    0x0d, 0x0a,
    'N', 'o', 'n', '-', 's', 'y', 's', 't',
    'e', 'm', ' ', 'd', 'i', 's', 'k',
    0x0d, 0x0a,
    'P', 'r', 'e', 's', 's', ' ', 'a', 'n',
    'y', ' ', 'k', 'e', 'y', ' ', 't', 'o',
    ' ', 'r', 'e', 'b', 'o', 'o', 't',
    0x0d, 0x0a,
    0
};

/*
 * [2873851] Tables of default cluster sizes for FAT16 and FAT32.
 * These constants come from Microsoft's documentation.
 */

#define MAX_SEC_FAT12 8400	/* (4 MB) Maximum number of sectors to default to FAT12 */
#define MAX_SEC_FAT16 1048576	/* (512 MB) Maximum number of sectors t odefault to FAT16 */

struct DiskSizeToClusterSize {
    u_int diskSectors;		/* input: maximum bpb.bsec */
    u_int sectorsPerCluster;	/* output: desired bpb.spc */
};

struct DiskSizeToClusterSize fat16Sizes[] = {
    {   8400,  0},	/* Disks up to 4.1 MB; the 0 triggers an error */
    {  32680,  2},	/* Disks up to  16 MB => 1 KB cluster */
    { 262144,  4},	/* Disks up to 128 MB => 2 KB cluster */
    { 524288,  8},	/* Disks up to 256 MB => 4 KB cluster */
    {1048576, 16},	/* Disks up to 512 MB => 8 KB cluster */
    /* The following entries are used only if FAT16 is forced */
    {2097152, 32},	/* Disks up to 1 GB => 16 KB cluster */
    {0xFFFFFFFF, 64}	/* Disks over 2 GB => 32KB cluster (total size may be limited) */
};
struct DiskSizeToClusterSize fat32Sizes[] = {
    {   66600,  0},	/* Disks up to 32.5 MB; the 0 triggers an error */
    {  532480,  1},	/* Disks up to 260 MB => 512 byte cluster; not used unles FAT32 forced */
    {16777216,  8},	/* Disks up to   8 GB =>  4 KB cluster */
    {33554432, 16},	/* Disks up to  16 GB =>  8 KB cluster */
    {67108864, 32},	/* Disks up to  32 GB => 16 KB cluster */
    {0xFFFFFFFF, 64}	/* Disks over 32 GB => 32 KB cluster */
};

static void check_mounted(const char *, mode_t);
static void getstdfmt(const char *, struct bpb *);
static void getdiskinfo(int, const char *, const char *, int,
			struct bpb *);
static void print_bpb(struct bpb *);
static u_int ckgeom(const char *, u_int, const char *);
static u_int argtou(const char *, u_int, u_int, const char *);
static int oklabel(const char *);
static void mklabel(u_int8_t *, const char *);
static void setstr(u_int8_t *, const char *, size_t);
static void usage(void);

/*
 * Construct a FAT12, FAT16, or FAT32 file system.
 */
int
main(int argc, char *argv[])
{
    static char opts[] = "NB:F:I:O:S:a:b:c:e:f:h:i:k:m:n:o:r:s:u:v:";
    static const char *opt_B, *opt_v, *opt_O, *opt_f;
    static u_int opt_F, opt_I, opt_S, opt_a, opt_b, opt_c, opt_e;
    static u_int opt_h, opt_i, opt_k, opt_m, opt_n, opt_o, opt_r;
    static u_int opt_s, opt_u;
    static int opt_N;
    static int Iflag, mflag, oflag;
    char buf[MAXPATHLEN];
    struct stat sb;
    struct timeval tv;
    struct bpb bpb;
    struct tm *tm;
    struct bs *bs;
    struct bsbpb *bsbpb;
    struct bsxbpb *bsxbpb;
    struct bsx *bsx;
    struct de *de;
    u_int8_t *img;
    const char *fname, *dtype, *bname;
    ssize_t n;
    time_t now;
    u_int fat, bss, rds, cls, dir, lsn, x, x1, x2;
    int ch, fd, fd1;

    while ((ch = getopt(argc, argv, opts)) != -1)
	switch (ch) {
	case 'N':
	    opt_N = 1;
	    break;
	case 'B':
	    opt_B = optarg;
	    break;
	case 'F':
	    if (strcmp(optarg, "12") &&
		strcmp(optarg, "16") &&
		strcmp(optarg, "32"))
		errx(1, "%s: bad FAT type", optarg);
	    opt_F = atoi(optarg);
	    break;
	case 'I':
	    opt_I = argto4(optarg, 0, "volume ID");
	    Iflag = 1;
	    break;
	case 'O':
	    if (strlen(optarg) > 8)
		errx(1, "%s: bad OEM string", optarg);
	    opt_O = optarg;
	    break;
	case 'S':
	    opt_S = argto2(optarg, 1, "bytes/sector");
	    break;
	case 'a':
	    opt_a = argto4(optarg, 1, "sectors/FAT");
	    break;
	case 'b':
	    opt_b = argtox(optarg, 1, "block size");
	    opt_c = 0;
	    break;
	case 'c':
	    opt_c = argto1(optarg, 1, "sectors/cluster");
	    opt_b = 0;
	    break;
	case 'e':
	    opt_e = argto2(optarg, 1, "directory entries");
	    break;
	case 'f':
	    opt_f = optarg;
	    break;
	case 'h':
	    opt_h = argto2(optarg, 1, "drive heads");
	    break;
	case 'i':
	    opt_i = argto2(optarg, 1, "info sector");
	    break;
	case 'k':
	    opt_k = argto2(optarg, 1, "backup sector");
	    break;
	case 'm':
	    opt_m = argto1(optarg, 0, "media descriptor");
	    mflag = 1;
	    break;
	case 'n':
	    opt_n = argto1(optarg, 1, "number of FATs");
	    break;
	case 'o':
	    opt_o = argto4(optarg, 0, "hidden sectors");
	    oflag = 1;
	    break;
	case 'r':
	    opt_r = argto2(optarg, 1, "reserved sectors");
	    break;
	case 's':
	    opt_s = argto4(optarg, 1, "file system size");
	    break;
	case 'u':
	    opt_u = argto2(optarg, 1, "sectors/track");
	    break;
	case 'v':
	    if (!oklabel(optarg))
		errx(1, "%s: bad volume name", optarg);
	    opt_v = optarg;
	    break;
	default:
	    usage();
	}
    argc -= optind;
    argv += optind;
    if (argc < 1 || argc > 2)
	usage();
    fname = *argv++;
    if (!strchr(fname, '/')) {
	snprintf(buf, sizeof(buf), "%sr%s", _PATH_DEV, fname);
	if (stat(buf, &sb))
	    snprintf(buf, sizeof(buf), "%s%s", _PATH_DEV, fname);
	if (!(fname = strdup(buf)))
	    err(1, NULL);
    }
    dtype = *argv;
    if ((fd = open(fname, opt_N ? O_RDONLY : O_RDWR)) == -1 ||
	fstat(fd, &sb))
	err(1, "%s", fname);
    if (!opt_N)
	check_mounted(fname, sb.st_mode);
    if (!S_ISCHR(sb.st_mode))
	warnx("warning: %s is not a character device", fname);
    memset(&bpb, 0, sizeof(bpb));
    if (opt_f) {
	getstdfmt(opt_f, &bpb);
	bpb.bsec = bpb.sec;
	bpb.sec = 0;
	bpb.bspf = bpb.spf;
	bpb.spf = 0;
    }
    if (opt_h)
	bpb.hds = opt_h;
    if (opt_u)
	bpb.spt = opt_u;
    if (opt_S)
	bpb.bps = opt_S;
    if (opt_s)
	bpb.bsec = opt_s;
    if (oflag)
	bpb.hid = opt_o;
    if (!(opt_f || (opt_h && opt_u && opt_S && opt_s && oflag)))
	getdiskinfo(fd, fname, dtype, oflag, &bpb);
    if (!powerof2(bpb.bps))
	errx(1, "bytes/sector (%u) is not a power of 2", bpb.bps);
    if (bpb.bps < MINBPS)
	errx(1, "bytes/sector (%u) is too small; minimum is %u",
	     bpb.bps, MINBPS);
    if (!(fat = opt_F)) {
	if (opt_f)
	    fat = 12;
	else if (!opt_e && (opt_i || opt_k))
	    fat = 32;
    }
    if ((fat == 32 && opt_e) || (fat != 32 && (opt_i || opt_k)))
	errx(1, "-%c is not a legal FAT%s option",
	     fat == 32 ? 'e' : opt_i ? 'i' : 'k',
	     fat == 32 ? "32" : "12/16");
    if (opt_f && fat == 32)
	bpb.rde = 0;
    if (opt_b) {
	if (!powerof2(opt_b))
	    errx(1, "block size (%u) is not a power of 2", opt_b);
	if (opt_b < bpb.bps)
	    errx(1, "block size (%u) is too small; minimum is %u",
		 opt_b, bpb.bps);
	if (opt_b > bpb.bps * MAXSPC)
	    errx(1, "block size (%u) is too large; maximum is %u",
		 opt_b, bpb.bps * MAXSPC);
	bpb.spc = opt_b / bpb.bps;
    }
    if (opt_c) {
	if (!powerof2(opt_c))
	    errx(1, "sectors/cluster (%u) is not a power of 2", opt_c);
	bpb.spc = opt_c;
    }
    if (opt_r)
	bpb.res = opt_r;
    if (opt_n) {
	if (opt_n > MAXNFT)
	    errx(1, "number of FATs (%u) is too large; maximum is %u",
		 opt_n, MAXNFT);
	bpb.nft = opt_n;
    }
    if (opt_e)
	bpb.rde = opt_e;
    if (mflag) {
	if (opt_m < 0xf0)
	    errx(1, "illegal media descriptor (%#x)", opt_m);
	bpb.mid = opt_m;
    }
    if (opt_a)
	bpb.bspf = opt_a;
    if (opt_i)
	bpb.infs = opt_i;
    if (opt_k)
	bpb.bkbs = opt_k;
    bss = 1;
    bname = NULL;
    fd1 = -1;
    if (opt_B) {
	bname = opt_B;
	if (!strchr(bname, '/')) {
	    snprintf(buf, sizeof(buf), "/boot/%s", bname);
	    if (!(bname = strdup(buf)))
		err(1, NULL);
	}
	if ((fd1 = open(bname, O_RDONLY)) == -1 || fstat(fd1, &sb))
	    err(1, "%s", bname);
	if (!S_ISREG(sb.st_mode) || sb.st_size % bpb.bps ||
	    sb.st_size < bpb.bps || sb.st_size > bpb.bps * MAXU16)
	    errx(1, "%s: inappropriate file type or format", bname);
	bss = sb.st_size / bpb.bps;
    }
    if (!bpb.nft)
	bpb.nft = 2;

    /*
     * [2873851] If the FAT type or sectors per cluster were not explicitly specified,
     * set them to default values.
     */
    if (!bpb.spc)
    {
	/*
	 * If the user didn't specify the FAT type, then pick a default based on the
	 * number of sectors on the volume.
	 */
	if (!fat)
	{
	    if (bpb.bsec <= MAX_SEC_FAT12)
		fat = 12;
	    else if (bpb.bsec <= MAX_SEC_FAT16)
		fat = 16;
	    else
		fat = 32;
	}

	switch (fat)
	{
	case 12:
	    /*
	     * There is no general table for FAT12, so try all possible
	     * sector-per-cluster values until it all fits, or we try the
	     * maximum cluster size.
	     */
	    for (bpb.spc=1; bpb.spc<64; bpb.spc*=2)
	    {
		/* Start with number of reserved sectors */
		x = bpb.res ? bpb.res : bss;
		/* Plus number of sectors used by FAT */
		x = howmany((RESFTE+MAXCLS12+1)*(12/BPN), bpb.bps*NPB) * bpb.nft;
		/* Plus root directory */
		x += howmany(bpb.rde ? bpb.rde : DEFRDE, bpb.bps / sizeof(struct de));
		/* Plus data clusters */
		x += (MAXCLS12+1) * bpb.spc;

		/*
		 * We now know how many sectors the volume would occupy with the given
		 * sectors per cluster, and the maximum number of FAT12 clusters.  If
		 * this is as big as or bigger than the actual volume, we've found the
		 * minimum sectors per cluster.
		 */
		if (x >= bpb.bsec)
		    break;
	    }
	    break;
	case 16:
	    for (x=0; bpb.bsec > fat16Sizes[x].diskSectors; ++x)
		;
	    bpb.spc = fat16Sizes[x].sectorsPerCluster;
	    break;
	case 32:
	    for (x=0; bpb.bsec > fat32Sizes[x].diskSectors; ++x)
		;
	    bpb.spc = fat32Sizes[x].sectorsPerCluster;
	    break;
	default:
	    errx(1, "Invalid FAT type: %d", fat);
	    break;
	}
	
	if (bpb.spc == 0)
	    errx(1, "FAT%d is impossible with %lu sectors", fat, bpb.bsec);
    }
    else
    {
	/*
	 * User explicitly specified sectors per cluster.  If they didn't
	 * specify the FAT type, pick one that uses up the available sectors.
	 */
	if (!fat)
	{
	    /* See if a maximum number of FAT clusters would fill it up. */
	    if (bpb.bsec < (bpb.res ? bpb.res : bss) +
		howmany((RESFTE+MAXCLS12+1) * (12/BPN), bpb.bps * BPN) * bpb.nft +
		howmany(bpb.rde ? bpb.rde : DEFRDE, bpb.bps / sizeof(struct de)) +
		(MAXCLS12+1) * bpb.spc)
	    {
		fat = 12;
	    }
	    else if (bpb.bsec < (bpb.res ? bpb.res : bss) +
		howmany((RESFTE+MAXCLS16) * 2, bpb.bps) * bpb.nft +
		howmany(bpb.rde ? bpb.rde : DEFRDE, bpb.bps / sizeof(struct de)) +
		(MAXCLS16+1) * bpb.spc)
	    {
		fat = 16;
	    }
	    else
	    {
		fat = 32;
	    }
	}
    }
    
    x = bss;
    if (fat == 32) {
	if (!bpb.infs) {
	    if (x == MAXU16 || x == bpb.bkbs)
		errx(1, "no room for info sector");
	    bpb.infs = x;
	}
	if (bpb.infs != MAXU16 && x <= bpb.infs)
	    x = bpb.infs + 1;
	if (!bpb.bkbs) {
	    if (x == MAXU16)
		errx(1, "no room for backup sector");
	    if (x <= BACKUP_BOOT_SECTOR)
		bpb.bkbs = BACKUP_BOOT_SECTOR;
	    else
		bpb.bkbs = x;
	} else if (bpb.bkbs != MAXU16 && bpb.bkbs == bpb.infs)
	    errx(1, "backup sector would overwrite info sector");
	if (bpb.bkbs != MAXU16 && x <= bpb.bkbs)
	    x = bpb.bkbs + 1;
    }
    if (!bpb.res)
	bpb.res = fat == 32 ? MAX(x, FAT32_RESERVED_SECTORS) : x;
    else if (bpb.res < x)
	errx(1, "too few reserved sectors");
    if (fat != 32 && !bpb.rde)
	bpb.rde = DEFRDE;
    rds = howmany(bpb.rde, bpb.bps / sizeof(struct de));
    if (fat != 32 && bpb.bspf > MAXU16)
	errx(1, "too many sectors/FAT for FAT12/16");
    x1 = bpb.res + rds;
    x = bpb.bspf ? bpb.bspf : 1;
    if (x1 + (u_int64_t)x * bpb.nft > bpb.bsec)
	errx(1, "meta data exceeds file system size");
    x1 += x * bpb.nft;
    x = (u_int64_t)(bpb.bsec - x1) * bpb.bps * NPB /
	(bpb.spc * bpb.bps * NPB + fat / BPN * bpb.nft);
    x2 = howmany((RESFTE + MIN(x, maxcls(fat))) * (fat / BPN),
		 bpb.bps * NPB);
    if (!bpb.bspf) {
	bpb.bspf = x2;
	x1 += (bpb.bspf - 1) * bpb.nft;
    }
    cls = (bpb.bsec - x1) / bpb.spc;
    x = (u_int64_t)bpb.bspf * bpb.bps * NPB / (fat / BPN) - RESFTE;
    if (cls > x)
    {
    	/* 
    	 * This indicates that there are more sectors available
    	 * for data clusters than there are usable entries in the
    	 * FAT.  In this case, we need to limit the number of
    	 * clusters, and also reduce the number of sectors.
    	 */
	bpb.bsec = bpb.res + bpb.bspf*bpb.nft + rds + x*bpb.spc;
	warnx("warning: sectors/FAT limits sectors to %u, clusters to %u", bpb.bsec, x);
	cls = x;
    }
    if (bpb.bspf < x2)
	warnx("warning: sectors/FAT limits file system to %u clusters",
	      cls);
    if (cls < mincls(fat))
	errx(1, "%u clusters too few clusters for FAT%u, need %u", cls, fat,
	    mincls(fat));
    if (cls > maxcls(fat)) {
	cls = maxcls(fat);
	bpb.bsec = x1 + (cls + 1) * bpb.spc - 1;
	warnx("warning: FAT type limits file system to %u sectors",
	      bpb.bsec);
    }
    printf("%s: %u sector%s in %u FAT%u cluster%s "
	   "(%u bytes/cluster)\n", fname, cls * bpb.spc,
	   cls * bpb.spc == 1 ? "" : "s", cls, fat,
	   cls == 1 ? "" : "s", bpb.bps * bpb.spc);
    if (!bpb.mid)
        bpb.mid = !bpb.hid ? 0xf0 : 0xf8;
    if (fat == 32)
        bpb.rdcl = RESFTE;
    if (bpb.hid + bpb.bsec <= MAXU16) {
        bpb.sec = bpb.bsec;
        bpb.bsec = 0;
    }
    if (fat != 32) {
        bpb.spf = bpb.bspf;
        bpb.bspf = 0;
    } else {
        if (bpb.bsec == 0)
            bpb.bsec = bpb.sec;
        bpb.spf = 0;
        bpb.sec = 0;
    }
    print_bpb(&bpb);
    if (!opt_N) {
        gettimeofday(&tv, NULL);
        now = tv.tv_sec;
        tm = localtime(&now);
        if (!(img = malloc(bpb.bps)))
            err(1, NULL);
        dir = bpb.res + (bpb.spf ? bpb.spf : bpb.bspf) * bpb.nft;
	for (lsn = 0; lsn < dir + (fat == 32 ? bpb.spc : rds); lsn++) {
	    x = lsn;
	    if (opt_B &&
		fat == 32 && bpb.bkbs != MAXU16 &&
		bss <= bpb.bkbs && x >= bpb.bkbs) {
		x -= bpb.bkbs;
		if (!x && lseek(fd1, 0, SEEK_SET))
		    err(1, "%s", bname);
	    }
	    if (opt_B && x < bss) {
		if ((n = read(fd1, img, bpb.bps)) == -1)
		    err(1, "%s", bname);
		if (n != bpb.bps)
		    errx(1, "%s: can't read sector %u", bname, x);
	    } else
		memset(img, 0, bpb.bps);
	    if (!lsn ||
	      (fat == 32 && bpb.bkbs != MAXU16 && lsn == bpb.bkbs)) {
		x1 = sizeof(struct bs);
		bsbpb = (struct bsbpb *)(img + x1);
		mk2(bsbpb->bps, bpb.bps);
		mk1(bsbpb->spc, bpb.spc);
		mk2(bsbpb->res, bpb.res);
		mk1(bsbpb->nft, bpb.nft);
		mk2(bsbpb->rde, bpb.rde);
		mk2(bsbpb->sec, bpb.sec);
		mk1(bsbpb->mid, bpb.mid);
		mk2(bsbpb->spf, bpb.spf);
		mk2(bsbpb->spt, bpb.spt);
		mk2(bsbpb->hds, bpb.hds);
		mk4(bsbpb->hid, bpb.hid);
		mk4(bsbpb->bsec, bpb.bsec);
		x1 += sizeof(struct bsbpb);
		if (fat == 32) {
		    bsxbpb = (struct bsxbpb *)(img + x1);
		    mk4(bsxbpb->bspf, bpb.bspf);
		    mk2(bsxbpb->xflg, 0);
		    mk2(bsxbpb->vers, 0);
		    mk4(bsxbpb->rdcl, bpb.rdcl);
		    mk2(bsxbpb->infs, bpb.infs);
		    mk2(bsxbpb->bkbs, bpb.bkbs);
		    x1 += sizeof(struct bsxbpb);
		}
		bsx = (struct bsx *)(img + x1);
		mk1(bsx->sig, 0x29);
		if (Iflag)
		    x = opt_I;
		else
		    x = (((u_int)(1 + tm->tm_mon) << 8 |
			  (u_int)tm->tm_mday) +
			 ((u_int)tm->tm_sec << 8 |
			  (u_int)(tv.tv_usec / 10))) << 16 |
			((u_int)(1900 + tm->tm_year) +
			 ((u_int)tm->tm_hour << 8 |
			  (u_int)tm->tm_min));
		mk4(bsx->volid, x);
		mklabel(bsx->label, opt_v ? opt_v : "NO NAME");
		sprintf(buf, "FAT%u", fat);
		setstr(bsx->type, buf, sizeof(bsx->type));
		if (!opt_B) {
		    x1 += sizeof(struct bsx);
		    bs = (struct bs *)img;
		    mk1(bs->jmp[0], 0xeb);
		    mk1(bs->jmp[1], x1 - 2);
		    mk1(bs->jmp[2], 0x90);
		    setstr(bs->oem, opt_O ? opt_O : "BSD  4.4",
			   sizeof(bs->oem));
		    memcpy(img + x1, bootcode, sizeof(bootcode));
		    mk2(img + bpb.bps - 2, DOSMAGIC);
		}
	    } else if (fat == 32 && bpb.infs != MAXU16 &&
		       (lsn == bpb.infs ||
			(bpb.bkbs != MAXU16 &&
			 lsn == bpb.bkbs + bpb.infs))) {
		mk4(img, 0x41615252);
		mk4(img + 484, 0x61417272);
		mk4(img + 488, 0xffffffff);
		mk4(img + 492, bpb.rdcl+1);
		/* Offsets 508-509 remain zero */
		mk2(img + 510, DOSMAGIC);
	    } else if (lsn >= bpb.res && lsn < dir &&
		       !((lsn - bpb.res) %
			 (bpb.spf ? bpb.spf : bpb.bspf))) {
		mk1(img[0], bpb.mid);
		for (x = 1; x < fat * (fat == 32 ? 3 : 2) / 8; x++)
		    mk1(img[x], fat == 32 && x % 4 == 3 ? 0x0f : 0xff);
	    } else if (lsn == dir && opt_v && *opt_v) {
		de = (struct de *)img;
		mklabel(de->namext, opt_v);
		mk1(de->attr, 050);
		x = (u_int)tm->tm_hour << 11 |
		    (u_int)tm->tm_min << 5 |
		    (u_int)tm->tm_sec >> 1;
		mk2(de->time, x);
		x = (u_int)(tm->tm_year - 80) << 9 |
		    (u_int)(tm->tm_mon + 1) << 5 |
		    (u_int)tm->tm_mday;
		mk2(de->date, x);
	    }
	    if ((n = write(fd, img, bpb.bps)) == -1)
		err(1, "%s", fname);
	    if (n != bpb.bps)
		errx(1, "%s: can't write sector %u", fname, lsn);
	}
    }
    return 0;
}

/*
 * Exit with error if file system is mounted.
 */
static void
check_mounted(const char *fname, mode_t mode)
{
    struct statfs *mp;
    const char *s1, *s2;
    size_t len;
    int n, r;

    if (!(n = getmntinfo(&mp, MNT_NOWAIT)))
	err(1, "getmntinfo");
    len = strlen(_PATH_DEV);
    s1 = fname;
    if (!strncmp(s1, _PATH_DEV, len))
	s1 += len;
    r = S_ISCHR(mode) && s1 != fname && *s1 == 'r';
    for (; n--; mp++) {
	s2 = mp->f_mntfromname;
	if (!strncmp(s2, _PATH_DEV, len))
	    s2 += len;
	if ((r && s2 != mp->f_mntfromname && !strcmp(s1 + 1, s2)) ||
	    !strcmp(s1, s2))
	    errx(1, "%s is mounted on %s", fname, mp->f_mntonname);
    }
}

/*
 * Get a standard format.
 */
static void
getstdfmt(const char *fmt, struct bpb *bpb)
{
    u_int x, i;

    x = sizeof(stdfmt) / sizeof(stdfmt[0]);
    for (i = 0; i < x && strcmp(fmt, stdfmt[i].name); i++);
    if (i == x)
	errx(1, "%s: unknown standard format", fmt);
    *bpb = stdfmt[i].bpb;
}

/*
 * Get disk partition, and geometry information.
 */
static void
getdiskinfo(int fd, const char *fname, const char *dtype, int oflag,
	    struct bpb *bpb)
{
	struct disklabel lab;

	if (dkdisklabel(fd, &lab) < 0) {
		warn("ioctl (GDINFO)");
		errx(1, "%s: can't figure out partition info", fname);
	}

	if (!oflag)
		bpb->hid = lab.d_partitions[0].p_offset;
	if (!bpb->bsec)
		bpb->bsec = lab.d_partitions[0].p_size;

	if (!bpb->bps)
		bpb->bps = ckgeom(fname, lab.d_secsize, "bytes/sector");
	if (!bpb->spt)
		bpb->spt = ckgeom(fname, lab.d_nsectors, "sectors/track");
	if (!bpb->hds)
		bpb->hds = ckgeom(fname, lab.d_ntracks, "drive heads");
}

/*
 * Print out BPB values.
 */
static void
print_bpb(struct bpb *bpb)
{
    printf("bps=%u spc=%u res=%u nft=%u", bpb->bps, bpb->spc, bpb->res,
	   bpb->nft);
    if (bpb->rde)
	printf(" rde=%u", bpb->rde);
    if (bpb->sec)
	printf(" sec=%u", bpb->sec);
    printf(" mid=%#x", bpb->mid);
    if (bpb->spf)
	printf(" spf=%u", bpb->spf);
    printf(" spt=%u hds=%u hid=%u", bpb->spt, bpb->hds, bpb->hid);
    if (bpb->bsec)
	printf(" bsec=%u", bpb->bsec);
    if (!bpb->spf) {
	printf(" bspf=%u rdcl=%u", bpb->bspf, bpb->rdcl);
	printf(" infs=");
	printf(bpb->infs == MAXU16 ? "%#x" : "%u", bpb->infs);
	printf(" bkbs=");
	printf(bpb->bkbs == MAXU16 ? "%#x" : "%u", bpb->bkbs);
    }
    printf("\n");
}

/*
 * Check a disk geometry value.
 */
static u_int
ckgeom(const char *fname, u_int val, const char *msg)
{
    if (!val)
	errx(1, "%s: no default %s", fname, msg);
    if (val > MAXU16)
	errx(1, "%s: illegal %s", fname, msg);
    return val;
}

/*
 * Convert and check a numeric option argument.
 */
static u_int
argtou(const char *arg, u_int lo, u_int hi, const char *msg)
{
    char *s;
    u_long x;

    errno = 0;
    x = strtoul(arg, &s, 0);
    if (errno || !*arg || *s || x < lo || x > hi)
	errx(1, "%s: bad %s", arg, msg);
    return x;
}

/*
 * Check a volume label.
 */
static int
oklabel(const char *src)
{
    int c, i;

    for (i = 0; i <= 11; i++) {
	c = (u_char)*src++;
	if (c < ' ' + !i || strchr("\"*+,./:;<=>?[\\]|", c))
	    break;
    }
    return !c;
}

/*
 * Make a volume label.
 */
static void
mklabel(u_int8_t *dest, const char *src)
{
    int c, i;

    for (i = 0; i < 11; i++) {
	c = *src ? toupper(*src++) : ' ';
	*dest++ = !i && c == '\xe5' ? 5 : c;
    }
}

/*
 * Copy string, padding with spaces.
 */
static void
setstr(u_int8_t *dest, const char *src, size_t len)
{
    while (len--)
	*dest++ = *src ? *src++ : ' ';
}

/*
 * Print usage message.
 */
static void
usage(void)
{
    fprintf(stderr,
	    "usage: newfs_msdos [ -options ] special [disktype]\n");
    fprintf(stderr, "where the options are:\n");
    fprintf(stderr, "\t-N don't create file system: "
	    "just print out parameters\n");
    fprintf(stderr, "\t-B get bootstrap from file\n");
    fprintf(stderr, "\t-F FAT type (12, 16, or 32)\n");
    fprintf(stderr, "\t-I volume ID\n");
    fprintf(stderr, "\t-O OEM string\n");
    fprintf(stderr, "\t-S bytes/sector\n");
    fprintf(stderr, "\t-a sectors/FAT\n");
    fprintf(stderr, "\t-b block size\n");
    fprintf(stderr, "\t-c sectors/cluster\n");
    fprintf(stderr, "\t-e root directory entries\n");
    fprintf(stderr, "\t-f standard format\n");
    fprintf(stderr, "\t-h drive heads\n");
    fprintf(stderr, "\t-i file system info sector\n");
    fprintf(stderr, "\t-k backup boot sector\n");
    fprintf(stderr, "\t-m media descriptor\n");
    fprintf(stderr, "\t-n number of FATs\n");
    fprintf(stderr, "\t-o hidden sectors\n");
    fprintf(stderr, "\t-r reserved sectors\n");
    fprintf(stderr, "\t-s file system size (sectors)\n");
    fprintf(stderr, "\t-u sectors/track\n");
    fprintf(stderr, "\t-v filesystem/volume name\n");
    exit(1);
}
