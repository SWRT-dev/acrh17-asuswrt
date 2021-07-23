/*
 * Copyright (c) 2000 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * The contents of this file constitute Original Code as defined in and
 * are subject to the Apple Public Source License Version 1.1 (the
 * "License").  You may not use this file except in compliance with the
 * License.  Please obtain a copy of the License at
 * http://www.apple.com/publicsource and read it before using this file.
 * 
 * This Original Code and all software distributed under the License are
 * distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
 * License for the specific language governing rights and limitations
 * under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 */
#ifndef __HFS_ENDIAN_H__
#define __HFS_ENDIAN_H__

/*
 * hfs_endian.h
 *
 * This file prototypes endian swapping routines for the HFS/HFS Plus
 * volume format.
 */
#include <architecture/byte_order.h>

/*********************/
/* BIG ENDIAN Macros */
/*********************/
#if BYTE_ORDER == BIG_ENDIAN
    /* HFS is always big endian, make swaps into no-ops */
    #define SWAP_BE16(__a) (__a)
    #define SWAP_BE32(__a) (__a)
    #define SWAP_BE64(__a) (__a)

/************************/
/* LITTLE ENDIAN Macros */
/************************/
#elif BYTE_ORDER == LITTLE_ENDIAN
    /* HFS is always big endian, make swaps actually swap */
    #define SWAP_BE16(__a) 							NXSwapBigShortToHost (__a)
    #define SWAP_BE32(__a) 							NXSwapBigLongToHost (__a)
    #define SWAP_BE64(__a) 							NXSwapBigLongLongToHost (__a)

#else
#warning Unknown byte order
#error
#endif

#endif /* __HFS_ENDIAN__ */
