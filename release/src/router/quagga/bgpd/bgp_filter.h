/* AS path filter list.
   Copyright (C) 1999 Kunihiro Ishiguro

This file is part of GNU Zebra.

GNU Zebra is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option) any
later version.

GNU Zebra is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with GNU Zebra; see the file COPYING.  If not, write to the Free
Software Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
02111-1307, USA.  */

#ifndef _QUAGGA_BGP_FILTER_H
#define _QUAGGA_BGP_FILTER_H

enum as_filter_type
{
  AS_FILTER_DENY,
  AS_FILTER_PERMIT
};

extern void bgp_filter_init (void);
extern void bgp_filter_reset (void);

extern enum as_filter_type as_list_apply (struct as_list *, void *);

extern struct as_list *as_list_lookup (const char *);
extern void as_list_add_hook (void (*func) (void));
extern void as_list_delete_hook (void (*func) (void));

#endif /* _QUAGGA_BGP_FILTER_H */
