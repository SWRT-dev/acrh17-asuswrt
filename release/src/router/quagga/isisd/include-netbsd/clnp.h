/*	$NetBSD: clnp.h,v 1.13 2001/08/20 12:00:54 wiz Exp $	*/

/*-
 * Copyright (c) 1991, 1993, 1994
 *	The Regents of the University of California.  All rights reserved.
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
 *
 *	@(#)clnp.h	8.2 (Berkeley) 4/16/94
 */

/***********************************************************
		Copyright IBM Corporation 1987

                      All Rights Reserved

Permission to use, copy, modify, and distribute this software and its
documentation for any purpose and without fee is hereby granted,
provided that the above copyright notice appear in all copies and that
both that copyright notice and this permission notice appear in
supporting documentation, and that the name of IBM not be
used in advertising or publicity pertaining to distribution of the
software without specific, written prior permission.

IBM DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING
ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO EVENT SHALL
IBM BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR
ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
SOFTWARE.

******************************************************************/

/*
 * ARGO Project, Computer Sciences Dept., University of Wisconsin - Madison
 */

/* should be config option but cpp breaks with too many #defines */
#define	DECBIT

/*
 *	Return true if the mbuf is a cluster mbuf
 */
#define	IS_CLUSTER(m)	((m)->m_flags & M_EXT)

/*
 *	Move the halfword into the two characters
 */
#define	HTOC(msb, lsb, hword)\
	(msb) = (u_char)((hword) >> 8);\
	(lsb) = (u_char)((hword) & 0xff)
/*
 *	Move the two charcters into the halfword
 */
#define	CTOH(msb, lsb, hword)\
	(hword) = ((msb) << 8) | (lsb)

/*
 *	Return true if the checksum has been set - ie. the checksum is
 *	not zero
 */
#define	CKSUM_REQUIRED(clnp)\
	(((clnp)->cnf_cksum_msb != 0) || ((clnp)->cnf_cksum_lsb != 0))

/*
 *	Fixed part of clnp header
 */
struct clnp_fixed {
	u_char          cnf_proto_id;	/* network layer protocol identifier */
	u_char          cnf_hdr_len;	/* length indicator (octets) */
	u_char          cnf_vers;	/* version/protocol identifier
					 * extension */
	u_char          cnf_ttl;/* lifetime (500 milliseconds) */
	u_char          cnf_type;	/* type code */
	/* Includes err_ok, more_segs, and seg_ok */
	u_char          cnf_seglen_msb;	/* pdu segment length (octets) high
					 * byte */
	u_char          cnf_seglen_lsb;	/* pdu segment length (octets) low
					 * byte */
	u_char          cnf_cksum_msb;	/* checksum high byte */
	u_char          cnf_cksum_lsb;	/* checksum low byte */
} __attribute__((packed));
#define CNF_TYPE	0x1f
#define CNF_ERR_OK	0x20
#define CNF_MORE_SEGS	0x40
#define CNF_SEG_OK	0x80

#define CLNP_CKSUM_OFF	0x07	/* offset of checksum */

#define	clnl_fixed	clnp_fixed

/*
 *	Segmentation part of clnp header
 */
struct clnp_segment {
	u_short         cng_id;	/* data unit identifier */
	u_short         cng_off;/* segment offset */
	u_short         cng_tot_len;	/* total length */
};

/*
 *	Clnp fragment reassembly structures:
 *
 *	All packets undergoing reassembly are linked together in
 *	clnp_fragl structures. Each clnp_fragl structure contains a
 *	pointer to the original clnp packet header, as well as a
 *	list of packet fragments. Each packet fragment
 *	is headed by a clnp_frag structure. This structure contains the
 *	offset of the first and last byte of the fragment, as well as
 *	a pointer to the data (an mbuf chain) of the fragment.
 */

/*
 *	NOTE:
 *		The clnp_frag structure is stored in an mbuf immedately
 *		preceding the fragment data. Since there are words in
 *		this struct, it must be word aligned.
 *
 *	NOTE:
 *		All the fragment code assumes that the entire clnp header is
 *		contained in the first mbuf.
 */
struct clnp_frag {
	u_int           cfr_first;	/* offset of first byte of this frag */
	u_int           cfr_last;	/* offset of last byte of this frag */
	u_int           cfr_bytes;	/* bytes to shave to get to data */
	struct mbuf    *cfr_data;	/* ptr to data for this frag */
	struct clnp_frag *cfr_next;	/* next fragment in list */
};

struct clnp_fragl {
	struct iso_addr cfl_src;/* source of the pkt */
	struct iso_addr cfl_dst;/* destination of the pkt */
	u_short         cfl_id;	/* id of the pkt */
	u_char          cfl_ttl;/* current ttl of pkt */
	u_short         cfl_last;	/* offset of last byte of packet */
	struct mbuf    *cfl_orighdr;	/* ptr to original header */
	struct clnp_frag *cfl_frags;	/* linked list of fragments for pkt */
	struct clnp_fragl *cfl_next;	/* next pkt being reassembled */
};

/*
 *	The following structure is used to index into an options section
 *	of a clnp datagram. These values can be used without worry that
 *	offset or length fields are invalid or too big, etc. That is,
 *	the consistancy of the options will be guaranteed before this
 *	structure is filled in. Any pointer (field ending in p) is
 *	actually the offset from the beginning of the mbuf the option
 *	is contained in.  A value of NULL for any pointer
 *	means that the option is not present. The length any option
 *	does not include the option code or option length fields.
 */
struct clnp_optidx {
	u_short         cni_securep;	/* ptr to start of security option */
	char            cni_secure_len;	/* length of entire security option */

	u_short         cni_srcrt_s;	/* offset of start of src rt option */
	u_short         cni_srcrt_len;	/* length of entire src rt option */

	u_short         cni_recrtp;	/* ptr to beginning of recrt option */
	char            cni_recrt_len;	/* length of entire recrt option */

	char            cni_priorp;	/* ptr to priority option */

	u_short         cni_qos_formatp;	/* ptr to format of qos
						 * option */
	char            cni_qos_len;	/* length of entire qos option */

	u_char          cni_er_reason;	/* reason from ER pdu option */

	/* ESIS options */

	u_short         cni_esct;	/* value from ISH ESCT option */

	u_short         cni_netmaskp;	/* ptr to beginning of netmask option */
	char            cni_netmask_len;	/* length of entire netmask
						 * option */

	u_short         cni_snpamaskp;	/* ptr to start of snpamask option */
	char            cni_snpamask_len;	/* length of entire snpamask
						 * option */

};

#define	ER_INVALREAS	0xff	/* code for invalid ER pdu discard reason */

/* given an mbuf and addr of option, return offset from data of mbuf */
#define CLNP_OPTTOOFF(m, opt) ((u_short) (opt - mtod(m, caddr_t)))

/* given an mbuf and offset of option, return address of option */
#define CLNP_OFFTOOPT(m, off) ((caddr_t) (mtod(m, caddr_t) + off))

/* return true iff src route is valid */
#define	CLNPSRCRT_VALID(oidx) ((oidx) && (oidx->cni_srcrt_s))

/* return the offset field of the src rt */
#define CLNPSRCRT_OFF(oidx, options)\
	(*((u_char *)(CLNP_OFFTOOPT(options, oidx->cni_srcrt_s) + 1)))

/* return the type field of the src rt */
#define CLNPSRCRT_TYPE(oidx, options)\
	((u_char)(*(CLNP_OFFTOOPT(options, oidx->cni_srcrt_s))))

/* return the length of the current address */
#define CLNPSRCRT_CLEN(oidx, options)\
	((u_char)(*(CLNP_OFFTOOPT(options, oidx->cni_srcrt_s) + CLNPSRCRT_OFF(oidx, options) - 1)))

/* return the address of the current address */
#define CLNPSRCRT_CADDR(oidx, options)\
	((caddr_t)(CLNP_OFFTOOPT(options, oidx->cni_srcrt_s) + CLNPSRCRT_OFF(oidx, options)))

/*
 * return true if the src route has run out of routes this is true if the
 * offset of next route is greater than the end of the rt
 */
#define	CLNPSRCRT_TERM(oidx, options)\
	(CLNPSRCRT_OFF(oidx, options) > oidx->cni_srcrt_len)

/*
 *	Options a user can set/get
 */
#define	CLNPOPT_FLAGS	0x01	/* flags: seg permitted, no er xmit, etc  */
#define	CLNPOPT_OPTS	0x02	/* datagram options */

/*
 *	Values for particular datagram options
 */
#define	CLNPOVAL_PAD		0xcc	/* padding */
#define	CLNPOVAL_SECURE		0xc5	/* security */
#define	CLNPOVAL_SRCRT		0xc8	/* source routing */
#define	CLNPOVAL_RECRT		0xcb	/* record route */
#define	CLNPOVAL_QOS		0xc3	/* quality of service */
#define	CLNPOVAL_PRIOR		0xcd	/* priority */
#define CLNPOVAL_ERREAS		0xc1	/* ER PDU ONLY: reason for discard */

#define	CLNPOVAL_SRCSPEC	0x40	/* source address specific */
#define	CLNPOVAL_DSTSPEC	0x80	/* destination address specific */
#define	CLNPOVAL_GLOBAL		0xc0	/* globally unique */

/* Globally Unique QOS */
#define	CLNPOVAL_SEQUENCING	0x10	/* sequencing preferred */
#define CLNPOVAL_CONGESTED	0x08	/* congestion experienced */
#define CLNPOVAL_LOWDELAY	0x04	/* low transit delay */

#define	CLNPOVAL_PARTRT		0x00	/* partial source routing */
#define CLNPOVAL_COMPRT		0x01	/* complete source routing */

/*
 *	Clnp flags used in a control block flags field.
 *	NOTE: these must be out of the range of bits defined in ../net/raw_cb.h
 */
#define	CLNP_NO_SEG		0x010	/* segmentation not permitted */
#define	CLNP_NO_ER		0x020	/* do not generate ERs */
#define CLNP_SEND_RAW		0x080	/* send pkt as RAW DT not TP DT */
#define	CLNP_NO_CKSUM		0x100	/* don't use clnp checksum */
#define CLNP_ECHO		0x200	/* send echo request */
#define	CLNP_NOCACHE		0x400	/* don't store cache information */
#define CLNP_ECHOR		0x800	/* send echo reply */

/* valid clnp flags */
#define CLNP_VFLAGS \
	(CLNP_SEND_RAW|CLNP_NO_SEG|CLNP_NO_ER|CLNP_NO_CKSUM|\
	 CLNP_ECHO|CLNP_NOCACHE|CLNP_ECHOR)

/*
 * Constants used by clnp
 */
#define	CLNP_HDR_MIN	(sizeof (struct clnp_fixed))
#define	CLNP_HDR_MAX	(254)
#define	CLNP_TTL_UNITS	2	/* 500 milliseconds */
#define CLNP_TTL	15*CLNP_TTL_UNITS	/* time to live (seconds) */
#define	ISO8473_V1	0x01

/*
 *	Clnp packet types
 *	In order to test raw clnp and tp/clnp simultaneously, a third type of
 *	packet has been defined: CLNP_RAW. This is done so that the input
 *	routine can switch to the correct input routine (rclnp_input or
 *	tpclnp_input) based on the type field. If clnp had a higher level
 *	protocol field, this would not be necessary.
 */
#define	CLNP_DT			0x1C	/* normal data */
#define	CLNP_ER			0x01	/* error report */
#define	CLNP_RAW		0x1D	/* debug only */
#define CLNP_EC			0x1E	/* echo packet */
#define CLNP_ECR		0x1F	/* echo reply */

/*
 *	ER pdu error codes
 */
#define GEN_NOREAS		0x00	/* reason not specified */
#define GEN_PROTOERR		0x01	/* protocol procedure error */
#define GEN_BADCSUM		0x02	/* incorrect checksum */
#define GEN_CONGEST		0x03	/* pdu discarded due to congestion */
#define GEN_HDRSYNTAX		0x04	/* header syntax error */
#define GEN_SEGNEEDED		0x05	/* need segmentation but not allowed */
#define GEN_INCOMPLETE		0x06	/* incomplete pdu received */
#define GEN_DUPOPT		0x07	/* duplicate option */

/* address errors */
#define ADDR_DESTUNREACH	0x80	/* destination address unreachable */
#define ADDR_DESTUNKNOWN	0x81	/* destination address unknown */

/* source routing */
#define SRCRT_UNSPECERR		0x90	/* unspecified src rt error */
#define SRCRT_SYNTAX		0x91	/* syntax error in src rt field */
#define SRCRT_UNKNOWNADDR	0x92	/* unknown addr in src rt field */
#define SRCRT_BADPATH		0x93	/* path not acceptable */

/* lifetime */
#define TTL_EXPTRANSIT		0xa0	/* lifetime expired during transit */
#define TTL_EXPREASS		0xa1	/* lifetime expired during reassembly */

/* pdu discarded */
#define DISC_UNSUPPOPT		0xb0	/* unsupported option not specified? */
#define DISC_UNSUPPVERS		0xb1	/* unsupported protocol version */
#define DISC_UNSUPPSECURE	0xb2	/* unsupported security option */
#define DISC_UNSUPPSRCRT	0xb3	/* unsupported src rt option */
#define DISC_UNSUPPRECRT	0xb4	/* unsupported rec rt option */

/* reassembly */
#define REASS_INTERFERE		0xc0	/* reassembly interference */
#define CLNP_ERRORS		22


#ifdef CLNP_ER_CODES
u_char          clnp_er_codes[CLNP_ERRORS] = {
	GEN_NOREAS, GEN_PROTOERR, GEN_BADCSUM, GEN_CONGEST,
	GEN_HDRSYNTAX, GEN_SEGNEEDED, GEN_INCOMPLETE, GEN_DUPOPT,
	ADDR_DESTUNREACH, ADDR_DESTUNKNOWN,
	SRCRT_UNSPECERR, SRCRT_SYNTAX, SRCRT_UNKNOWNADDR, SRCRT_BADPATH,
	TTL_EXPTRANSIT, TTL_EXPREASS,
	DISC_UNSUPPOPT, DISC_UNSUPPVERS, DISC_UNSUPPSECURE,
	DISC_UNSUPPSRCRT, DISC_UNSUPPRECRT, REASS_INTERFERE
};
#endif

#ifdef	TROLL

#define	TR_DUPEND		0x01	/* duplicate end of fragment */
#define TR_DUPPKT		0x02	/* duplicate entire packet */
#define	TR_DROPPKT		0x04	/* drop packet on output */
#define TR_TRIM			0x08	/* trim bytes from packet */
#define TR_CHANGE		0x10	/* change bytes in packet */
#define TR_MTU			0x20	/* delta to change device mtu */
#define	TR_CHUCK		0x40	/* drop packet in rclnp_input */
#define	TR_BLAST		0x80	/* force rclnp_output to blast many
					 * packet */
#define	TR_RAWLOOP		0x100	/* make if_loop call clnpintr
					 * directly */
struct troll {
	int             tr_ops;	/* operations to perform */
	float           tr_dup_size;	/* % to duplicate */
	float           tr_dup_freq;	/* frequency to duplicate packets */
	float           tr_drop_freq;	/* frequence to drop packets */
	int             tr_mtu_adj;	/* delta to adjust if mtu */
	int             tr_blast_cnt;	/* # of pkts to blast out */
};

#define	SN_OUTPUT(clcp, m)\
	troll_output(clcp->clc_ifp, m, clcp->clc_firsthop, clcp->clc_rt)

#define	SN_MTU(ifp, rt) (((rt && rt->rt_rmx.rmx_mtu) ?\
	rt->rt_rmx.rmx_mtu : clnp_badmtu(ifp, rt, __LINE__, __FILE__))\
		- trollctl.tr_mtu_adj)

#ifdef _KERNEL
extern float    troll_random;
#endif

#else				/* NO TROLL */

#define	SN_OUTPUT(clcp, m)\
	(*clcp->clc_ifp->if_output)(clcp->clc_ifp, m, clcp->clc_firsthop, \
				    clcp->clc_rt)

#define	SN_MTU(ifp, rt) (((rt && rt->rt_rmx.rmx_mtu) ?\
	rt->rt_rmx.rmx_mtu : clnp_badmtu(ifp, rt, __LINE__, __FILE__)))

#endif				/* TROLL */

/*
 *	Macro to remove an address from a clnp header
 */
#define CLNP_EXTRACT_ADDR(isoa, hoff, hend)\
	{\
		isoa.isoa_len = (u_char)*hoff;\
		if ((((++hoff) + isoa.isoa_len) > hend) ||\
			(isoa.isoa_len > 20) || (isoa.isoa_len == 0)) {\
			hoff = (caddr_t)0;\
		} else {\
			(void) bcopy(hoff, (caddr_t)isoa.isoa_genaddr, \
				     isoa.isoa_len);\
			hoff += isoa.isoa_len;\
		}\
	}

/*
 *	Macro to insert an address into a clnp header
 */
#define CLNP_INSERT_ADDR(hoff, isoa)\
	*hoff++ = (isoa).isoa_len;\
	(void) bcopy((caddr_t)((isoa).isoa_genaddr), hoff, (isoa).isoa_len);\
	hoff += (isoa).isoa_len;

/*
 *	Clnp hdr cache.	Whenever a clnp packet is sent, a copy of the
 *	header is made and kept in this cache. In addition to a copy of
 *	the cached clnp hdr, the cache contains
 *	information necessary to determine whether the new packet
 *	to send requires a new header to be built.
 */
struct clnp_cache {
	/* these fields are used to check the validity of the cache */
	struct iso_addr clc_dst;/* destination of packet */
	struct mbuf    *clc_options;	/* ptr to options mbuf */
	int             clc_flags;	/* flags passed to clnp_output */

	/* these fields are state that clnp_output requires to finish the pkt */
	int             clc_segoff;	/* offset of seg part of header */
	struct rtentry *clc_rt;	/* ptr to rtentry (points into the route
				 * structure) */
	struct sockaddr *clc_firsthop;	/* first hop of packet */
	struct ifnet   *clc_ifp;/* ptr to interface structure */
	struct iso_ifaddr
	               *clc_ifa;/* ptr to interface address */
	struct mbuf    *clc_hdr;/* cached pkt hdr (finally)! */
};

#ifdef	_KERNEL
struct iso_addr;
struct sockaddr_iso;
struct mbuf;
struct clnp_segment;
struct sockaddr;
struct rt_entry;
struct clnp_fragl;
struct clnp_optidx;
struct isopcb;
struct snpa_hdr;
struct iso_ifaddr;
struct route_iso;

/* clnp_debug.c */
char *clnp_hexp __P((char *, int, char *));
char *clnp_iso_addrp __P((struct iso_addr *));
char *clnp_saddr_isop __P((struct sockaddr_iso *));

/* clnp_er.c */
void clnp_er_input __P((struct mbuf *, struct iso_addr *, u_int));
void clnp_discard __P((struct mbuf *, u_int));
void clnp_emit_er __P((struct mbuf *, u_int));
int clnp_er_index __P((u_int));

int clnp_fragment __P((struct ifnet *, struct mbuf *, struct sockaddr *,
		       int, int, int, struct rtentry *));
struct mbuf *clnp_reass __P((struct mbuf *, struct iso_addr *,
			     struct iso_addr *, struct clnp_segment *));
int clnp_newpkt __P((struct mbuf *, struct iso_addr *, struct iso_addr *,
		     struct clnp_segment *));
void clnp_insert_frag __P((struct clnp_fragl *, struct mbuf *,
			   struct clnp_segment *));
struct mbuf    *clnp_comp_pdu __P((struct clnp_fragl *));
#ifdef TROLL
float troll_random __P((void));
int troll_output __P((struct ifnet *, struct mbuf *, struct sockaddr *,
		      struct rtentry *));
#endif

/* clnp_input.c */
void clnp_init  __P((void));
void clnlintr    __P((void));
void clnp_input __P((struct mbuf *, ...));

/* clnp_options.c */
void clnp_update_srcrt __P((struct mbuf *, struct clnp_optidx *));
void clnp_dooptions __P((struct mbuf *, struct clnp_optidx *, struct ifnet *,
			 struct iso_addr *));
int clnp_set_opts __P((struct mbuf **, struct mbuf **));
int clnp_opt_sanity __P((struct mbuf *, caddr_t, int, struct clnp_optidx *));

/* clnp_output.c */
int clnp_output __P((struct mbuf *, ...));
void clnp_ctloutput __P((void));

/* clnp_raw.c */
void rclnp_input __P((struct mbuf *, ...));
int rclnp_output __P((struct mbuf *, ...));
int rclnp_ctloutput __P((int, struct socket *, int, int, struct mbuf **));
int clnp_usrreq __P((struct socket *, int, struct mbuf *, struct mbuf *,
		     struct mbuf *, struct proc *));

/* clnp_subr.c */
struct mbuf    *clnp_data_ck __P((struct mbuf *, int));
caddr_t clnp_extract_addr __P((caddr_t, int, struct iso_addr *,
			       struct iso_addr *));
int clnp_ours   __P((struct iso_addr *));
void clnp_forward __P((struct mbuf *, int, struct iso_addr *,
		       struct clnp_optidx *, int, struct snpa_hdr *));
caddr_t clnp_insert_addr __P((caddr_t, struct iso_addr *, struct iso_addr *));
int clnp_route  __P((struct iso_addr *, struct route_iso *, int,
		     struct sockaddr **, struct iso_ifaddr **));
int clnp_srcroute __P((struct mbuf *, struct clnp_optidx *, struct route_iso *,
		       struct sockaddr **, struct iso_ifaddr **,
		       struct iso_addr *));
int clnp_echoreply __P((struct mbuf *, int, struct sockaddr_iso *,
		        struct sockaddr_iso *, struct clnp_optidx *));
int clnp_badmtu __P((struct ifnet *, struct rtentry *, int, char *));
void clnp_ypocb  __P((caddr_t, caddr_t, u_int));

/* clnp_timer.c */
struct clnp_fragl *clnp_freefrags __P((struct clnp_fragl *));
void clnp_slowtimo __P((void));
void clnp_drain __P((void));

#ifdef	TROLL
struct troll    trollctl;
#endif /* TROLL */

#endif /* _KERNEL */
