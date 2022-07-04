/************************************************************************ 
 * RSTP library - Rapid Spanning Tree (802.1t, 802.1w) 
 * Copyright (C) 2001-2003 Optical Access 
 * Author: Alex Rozin 
 * 
 * This file is part of RSTP library. 
 * 
 * RSTP library is free software; you can redistribute it and/or modify it 
 * under the terms of the GNU Lesser General Public License as published by the 
 * Free Software Foundation; version 2.1 
 * 
 * RSTP library is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser 
 * General Public License for more details. 
 * 
 * You should have received a copy of the GNU Lesser General Public License 
 * along with RSTP library; see the file COPYING.  If not, write to the Free 
 * Software Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 
 * 02111-1307, USA. 
 **********************************************************************/

/* Port Role Selection state machine : 17.22 */

#ifndef _STP_ROLES_SELECT_H
#define _STP_ROLES_SELECT_H

void
STP_rolesel_enter_state (STATE_MACH_T* s);

Bool
STP_rolesel_check_conditions (STATE_MACH_T* s);

void
STP_rolesel_update_stpm (struct stpm_t* this);

char*
STP_rolesel_get_state_name (int state);

#endif /* _STP_ROLES_SELECT_H */

