/*
 *  Unix SMB/CIFS implementation.
 *  Internal DNS query structures
 *  Copyright (C) Gerald Carter                2006.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _ADS_DNS_H
#define _ADS_DNS_H

/* DNS query section in replies */

struct dns_query {
	const char *hostname;
	uint16 type;
	uint16 in_class;
};

/* DNS RR record in reply */

struct dns_rr {
	const char *hostname;
	uint16 type;
	uint16 in_class;
	uint32 ttl;
	uint16 rdatalen;
	uint8 *rdata;
};

/* SRV records */

struct dns_rr_srv {
	const char *hostname;
	uint16 priority;
	uint16 weight;
	uint16 port;
	size_t num_ips;
	struct sockaddr_storage *ss_s;	/* support multi-homed hosts */
};

/* NS records */

struct dns_rr_ns {
	const char *hostname;
	struct sockaddr_storage ss;
};


#endif	/* _ADS_DNS_H */
