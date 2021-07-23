/*
 * Copyright (c) 2000, 2005 Apple Computer, Inc. All rights reserved.
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
#include <hfs/hfs_format.h>
#if LINUX
#include <endian.h>
#include <byteswap.h>
#else
#include <architecture/byte_order.h>
#endif
#include "SRuntime.h"

/*********************/
/* BIG ENDIAN Macros */
/*********************/
#define SWAP_BE16(__a) 							OSSwapBigToHostInt16 (__a)
#define SWAP_BE32(__a) 							OSSwapBigToHostInt32 (__a)
#define SWAP_BE64(__a) 							OSSwapBigToHostInt64 (__a)

#if BYTE_ORDER == BIG_ENDIAN
    
    /* HFS is always big endian, no swapping needed */
    #define SWAP_HFSMDB(__a)
    #define SWAP_HFSPLUSVH(__a)

/************************/
/* LITTLE ENDIAN Macros */
/************************/
#elif BYTE_ORDER == LITTLE_ENDIAN

    #define SWAP_HFSMDB(__a)				hfs_swap_HFSMasterDirectoryBlock ((__a))
    #define SWAP_HFSPLUSVH(__a)				hfs_swap_HFSPlusVolumeHeader ((__a))

#else
#warning Unknown byte order
#error
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Constants for the "unswap" argument to hfs_swap_BTNode:
 */
enum HFSBTSwapDirection {
	kSwapBTNodeBigToHost		=	0,
	kSwapBTNodeHostToBig		=	1,

	/*
	 * kSwapBTNodeHeaderRecordOnly is used to swap just the header record
	 * of a header node from big endian (on disk) to host endian (in memory).
	 * It does not swap the node descriptor (forward/backward links, record
	 * count, etc.).  It assumes the header record is at offset 0x000E.
	 *
	 * Since HFS Plus doesn't have fixed B-tree node sizes, we have to read
	 * the header record to determine the actual node size for that tree
	 * before we can set up the B-tree control block.  We read it initially
	 * as 512 bytes, then re-read it once we know the correct node size.  Since
	 * we may not have read the entire header node the first time, we can't
	 * swap the record offsets, other records, or do most sanity checks.
	 */
	kSwapBTNodeHeaderRecordOnly	=	3
};

void hfs_swap_HFSMasterDirectoryBlock (void *buf);
void hfs_swap_HFSPlusVolumeHeader (void *buf);
int  hfs_swap_BTNode (BlockDescriptor *src, SFCB *fcb, enum HFSBTSwapDirection direction);

#ifdef __cplusplus
}
#endif

#endif /* __HFS_FORMAT__ */
