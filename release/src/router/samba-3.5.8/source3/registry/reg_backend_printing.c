/*
 *  Unix SMB/CIFS implementation.
 *  Virtual Windows Registry Layer
 *  Copyright (C) Gerald Carter                     2002-2005
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

/* Implementation of registry virtual views for printing information */

#include "includes.h"

#undef DBGC_CLASS
#define DBGC_CLASS DBGC_REGISTRY

/* registrt paths used in the print_registry[] */

#define KEY_MONITORS		"HKLM/SYSTEM/CURRENTCONTROLSET/CONTROL/PRINT/MONITORS"
#define KEY_FORMS		"HKLM/SYSTEM/CURRENTCONTROLSET/CONTROL/PRINT/FORMS"
#define KEY_CONTROL_PRINTERS	"HKLM/SYSTEM/CURRENTCONTROLSET/CONTROL/PRINT/PRINTERS"
#define KEY_ENVIRONMENTS	"HKLM/SYSTEM/CURRENTCONTROLSET/CONTROL/PRINT/ENVIRONMENTS"
#define KEY_CONTROL_PRINT	"HKLM/SYSTEM/CURRENTCONTROLSET/CONTROL/PRINT"
#define KEY_WINNT_PRINTERS	"HKLM/SOFTWARE/MICROSOFT/WINDOWS NT/CURRENTVERSION/PRINT/PRINTERS"
#define KEY_WINNT_PRINT		"HKLM/SOFTWARE/MICROSOFT/WINDOWS NT/CURRENTVERSION/PRINT"
#define KEY_PORTS		"HKLM/SOFTWARE/MICROSOFT/WINDOWS NT/CURRENTVERSION/PORTS"

/* callback table for various registry paths below the ones we service in this module */

struct reg_dyn_tree {
	/* full key path in normalized form */
	const char *path;

	/* callbscks for fetch/store operations */
	int ( *fetch_subkeys) ( const char *path, struct regsubkey_ctr *subkeys );
	bool (*store_subkeys) ( const char *path, struct regsubkey_ctr *subkeys );
	int  (*fetch_values)  ( const char *path, struct regval_ctr *values );
	bool (*store_values)  ( const char *path, struct regval_ctr *values );
};

/*********************************************************************
 *********************************************************************
 ** Utility Functions
 *********************************************************************
 *********************************************************************/

/***********************************************************************
 simple function to prune a pathname down to the basename of a file
 **********************************************************************/

static const char *dos_basename(const char *path)
{
	const char *p;

	if (!(p = strrchr( path, '\\'))) {
		p = path;
	} else {
		p++;
	}

	return p;
}

/*********************************************************************
 *********************************************************************
 ** "HKLM/SYSTEM/CURRENTCONTROLSET/CONTROL/PRINT/FORMS"
 *********************************************************************
 *********************************************************************/

static int key_forms_fetch_keys(const char *key, struct regsubkey_ctr *subkeys)
{
	char *p = reg_remaining_path(talloc_tos(), key + strlen(KEY_FORMS));

	/* no keys below Forms */

	if (p) {
		return -1;
	}

	return 0;
}

/**********************************************************************
 *********************************************************************/

static int key_forms_fetch_values(const char *key, struct regval_ctr *values)
{
	uint32 		data[8];
	int		i, num_values, form_index = 1;
	nt_forms_struct *forms_list = NULL;
	nt_forms_struct *form;

	DEBUG(10,("print_values_forms: key=>[%s]\n", key ? key : "NULL" ));

	num_values = get_ntforms( &forms_list );

	DEBUG(10,("hive_forms_fetch_values: [%d] user defined forms returned\n",
		num_values));

	/* handle user defined forms */

	for ( i=0; i<num_values; i++ ) {
		form = &forms_list[i];

		data[0] = form->width;
		data[1] = form->length;
		data[2] = form->left;
		data[3] = form->top;
		data[4] = form->right;
		data[5] = form->bottom;
		data[6] = form_index++;
		data[7] = form->flag;

		regval_ctr_addvalue( values, form->name, REG_BINARY, (char*)data, sizeof(data) );	
	}

	SAFE_FREE( forms_list );
	forms_list = NULL;

	/* handle built-on forms */

	num_values = get_builtin_ntforms( &forms_list );

	DEBUG(10,("print_subpath_values_forms: [%d] built-in forms returned\n",
		num_values));

	for ( i=0; i<num_values; i++ ) {
		form = &forms_list[i];

		data[0] = form->width;
		data[1] = form->length;
		data[2] = form->left;
		data[3] = form->top;
		data[4] = form->right;
		data[5] = form->bottom;
		data[6] = form_index++;
		data[7] = form->flag;

		regval_ctr_addvalue(values, form->name, REG_BINARY, (char*)data, sizeof(data) );
	}

	SAFE_FREE(forms_list);

	return regval_ctr_numvals(values);
}

/*********************************************************************
 *********************************************************************
 ** "HKLM/SYSTEM/CURRENTCONTROLSET/CONTROL/PRINT/PRINTERS"
 ** "HKLM/SOFTWARE/MICROSOFT/WINDOWS NT/CURRENTVERSION/PRINT/PRINTERS"
 *********************************************************************
 *********************************************************************/

/*********************************************************************
 strip off prefix for printers key.  DOes return a pointer to static
 memory.
 *********************************************************************/

static char *strip_printers_prefix(const char *key)
{
	char *subkeypath = NULL;
	char *path = NULL;
	TALLOC_CTX *ctx = talloc_tos();

	path = talloc_strdup(ctx, key);
	if (!path) {
		return NULL;
	}
	path = normalize_reg_path(ctx, path);
	if (!path) {
		return NULL;
	}

	/* normalizing the path does not change length, just key delimiters and case */

	if (strncmp(path, KEY_WINNT_PRINTERS, strlen(KEY_WINNT_PRINTERS)) == 0) {
		subkeypath = reg_remaining_path(ctx, key + strlen(KEY_WINNT_PRINTERS));
	} else {
		subkeypath = reg_remaining_path(ctx, key + strlen(KEY_CONTROL_PRINTERS));
	}

	TALLOC_FREE(path);
	return subkeypath;
}

/*********************************************************************
 *********************************************************************/

static int key_printers_fetch_keys( const char *key, struct regsubkey_ctr *subkeys )
{
	int n_services = lp_numservices();
	int snum;
	fstring sname;
	int i;
	int num_subkeys = 0;
	char *printers_key;
	char *printername, *printerdatakey;
	NT_PRINTER_INFO_LEVEL *printer = NULL;
	fstring *subkey_names = NULL;

	DEBUG(10,("key_printers_fetch_keys: key=>[%s]\n", key ? key : "NULL" ));

	printers_key = strip_printers_prefix( key );

	if ( !printers_key ) {
		/* enumerate all printers */

		for (snum=0; snum<n_services; snum++) {
			if ( !(lp_snum_ok(snum) && lp_print_ok(snum) ) )
				continue;

			/* don't report the [printers] share */

			if ( strequal( lp_servicename(snum), PRINTERS_NAME ) )
				continue;

			fstrcpy( sname, lp_servicename(snum) );

			regsubkey_ctr_addkey( subkeys, sname );
		}

		num_subkeys = regsubkey_ctr_numkeys( subkeys );
		goto done;
	}

	/* get information for a specific printer */

	if (!reg_split_path( printers_key, &printername, &printerdatakey )) {
		return -1;
	}

	/* validate the printer name */

	for (snum=0; snum<n_services; snum++) {
		if ( !lp_snum_ok(snum) || !lp_print_ok(snum) )
			continue;
		if (strequal( lp_servicename(snum), printername ) )
			break;
	}

	if ( snum>=n_services
		|| !W_ERROR_IS_OK( get_a_printer(NULL, &printer, 2, printername) ) ) 
	{
		return -1;
	}

	num_subkeys = get_printer_subkeys( printer->info_2->data, printerdatakey?printerdatakey:"", &subkey_names );
	
	for ( i=0; i<num_subkeys; i++ )
		regsubkey_ctr_addkey( subkeys, subkey_names[i] );
	
	free_a_printer( &printer, 2 );
			
	/* no other subkeys below here */

done:	
	SAFE_FREE( subkey_names );
	
	return num_subkeys;
}

/**********************************************************************
 Take a list of names and call add_printer_hook() if necessary
 Note that we do this a little differently from Windows since the 
 keyname is the sharename and not the printer name.
 *********************************************************************/

static bool add_printers_by_registry( struct regsubkey_ctr *subkeys )
{
	int i, num_keys, snum;
	char *printername;
	NT_PRINTER_INFO_LEVEL_2 info2;
	NT_PRINTER_INFO_LEVEL printer;
	
	ZERO_STRUCT( info2 );
	printer.info_2 = &info2;
	
	num_keys = regsubkey_ctr_numkeys( subkeys );
	
	become_root();
	for ( i=0; i<num_keys; i++ ) {
		printername = regsubkey_ctr_specific_key( subkeys, i );
		snum = find_service( printername );
		
		/* just verify a valied snum for now */
		if ( snum == -1 ) {
			fstrcpy( info2.printername, printername );
			fstrcpy( info2.sharename, printername );
			if ( !add_printer_hook(talloc_tos(), NULL, &printer ) ) {
				DEBUG(0,("add_printers_by_registry: Failed to add printer [%s]\n",
					printername));
			}	
		}
	}
	unbecome_root();

	return True;
}

/**********************************************************************
 *********************************************************************/

static bool key_printers_store_keys( const char *key, struct regsubkey_ctr *subkeys )
{
	char *printers_key;
	char *printername, *printerdatakey;
	NT_PRINTER_INFO_LEVEL *printer = NULL;
	int i, num_subkeys, num_existing_keys;
	char *subkeyname;
	fstring *existing_subkeys = NULL;
	
	printers_key = strip_printers_prefix( key );
	
	if ( !printers_key ) {
		/* have to deal with some new or deleted printer */
		return add_printers_by_registry( subkeys );
	}
	
	if (!reg_split_path( printers_key, &printername, &printerdatakey )) {
		return False;
	}
	
	/* lookup the printer */
	
	if ( !W_ERROR_IS_OK(get_a_printer(NULL, &printer, 2, printername)) ) {
		DEBUG(0,("key_printers_store_keys: Tried to store subkey for bad printername %s\n", 
			printername));
		return False;
	}
	
	/* get the top level printer keys */
	
	num_existing_keys = get_printer_subkeys( printer->info_2->data, "", &existing_subkeys );
	
	for ( i=0; i<num_existing_keys; i++ ) {
	
		/* remove the key if it has been deleted */
		
		if ( !regsubkey_ctr_key_exists( subkeys, existing_subkeys[i] ) ) {
			DEBUG(5,("key_printers_store_keys: deleting key %s\n", 
				existing_subkeys[i]));
			delete_printer_key( printer->info_2->data, existing_subkeys[i] );
		}
	}

	num_subkeys = regsubkey_ctr_numkeys( subkeys );
	for ( i=0; i<num_subkeys; i++ ) {
		subkeyname = regsubkey_ctr_specific_key(subkeys, i);
		/* add any missing printer keys */
		if ( lookup_printerkey(printer->info_2->data, subkeyname) == -1 ) {
			DEBUG(5,("key_printers_store_keys: adding key %s\n", 
				existing_subkeys[i]));
			if ( add_new_printer_key( printer->info_2->data, subkeyname ) == -1 ) {
				SAFE_FREE( existing_subkeys );
				return False;
			}
		}
	}
	
	/* write back to disk */
	
	mod_a_printer( printer, 2 );
	
	/* cleanup */
	
	free_a_printer( &printer, 2 );

	SAFE_FREE( existing_subkeys );

	return True;
}

/**********************************************************************
 *********************************************************************/

static void fill_in_printer_values(NT_PRINTER_INFO_LEVEL_2 *info2, struct regval_ctr *values)
{
	struct spoolss_DeviceMode *devmode;
	char 		*p;
	uint32 printer_status = PRINTER_STATUS_OK;
	
	regval_ctr_addvalue( values, "Attributes",       REG_DWORD, (char*)&info2->attributes,       sizeof(info2->attributes) );
	regval_ctr_addvalue( values, "Priority",         REG_DWORD, (char*)&info2->priority,         sizeof(info2->attributes) );
	regval_ctr_addvalue( values, "ChangeID",         REG_DWORD, (char*)&info2->changeid,         sizeof(info2->changeid) );
	regval_ctr_addvalue( values, "Default Priority", REG_DWORD, (char*)&info2->default_priority, sizeof(info2->default_priority) );
	
	/* lie and say everything is ok since we don't want to call print_queue_length() to get the real status */
	regval_ctr_addvalue( values, "Status",           REG_DWORD, (char*)&printer_status,          sizeof(info2->status) );

	regval_ctr_addvalue( values, "StartTime",        REG_DWORD, (char*)&info2->starttime,        sizeof(info2->starttime) );
	regval_ctr_addvalue( values, "UntilTime",        REG_DWORD, (char*)&info2->untiltime,        sizeof(info2->untiltime) );

	/* strip the \\server\ from this string */
	if ( !(p = strrchr( info2->printername, '\\' ) ) )
		p = info2->printername;
	else
		p++;

	regval_ctr_addvalue_sz(values, "Name", p);
	regval_ctr_addvalue_sz(values, "Location", info2->location);
	regval_ctr_addvalue_sz(values, "Description", info2->comment);
	regval_ctr_addvalue_sz(values, "Parameters", info2->parameters);
	regval_ctr_addvalue_sz(values, "Port", info2->portname);
	regval_ctr_addvalue_sz(values, "Share Name", info2->sharename);
	regval_ctr_addvalue_sz(values, "Printer Driver", info2->drivername);
	regval_ctr_addvalue_sz(values, "Separator File", info2->sepfile);
	regval_ctr_addvalue_sz(values, "Print Processor", "WinPrint");
	regval_ctr_addvalue_sz(values, "Datatype", "RAW");

	/* stream the device mode */

	devmode = construct_dev_mode(values,info2->sharename);
	if (devmode) {
		DATA_BLOB blob;
		enum ndr_err_code ndr_err;

		ndr_err = ndr_push_struct_blob(&blob, values, NULL, devmode,
				(ndr_push_flags_fn_t)ndr_push_spoolss_DeviceMode);

		if (NDR_ERR_CODE_IS_SUCCESS(ndr_err)) {
			regval_ctr_addvalue(values, "Default Devmode", REG_BINARY,
					    (const char *)blob.data, blob.length);
		}
	}

	/* stream the printer security descriptor */

	if (info2->secdesc_buf &&
	    info2->secdesc_buf->sd &&
	    info2->secdesc_buf->sd_size)
	{
		NTSTATUS status;
		DATA_BLOB blob;

		status = marshall_sec_desc(values, info2->secdesc_buf->sd,
					   &blob.data, &blob.length);
		if (NT_STATUS_IS_OK(status)) {
			regval_ctr_addvalue(values, "Security", REG_BINARY,
					    (const char *)blob.data, blob.length);
		}
	}

	return;
}

/**********************************************************************
 *********************************************************************/

static int key_printers_fetch_values(const char *key, struct regval_ctr *values)
{
	int 		num_values;
	char		*printers_key;
	char		*printername, *printerdatakey;
	NT_PRINTER_INFO_LEVEL 	*printer = NULL;
	NT_PRINTER_DATA	*p_data;
	int		i, key_index;
	
	printers_key = strip_printers_prefix( key );	
	
	/* top level key values stored in the registry has no values */
	
	if ( !printers_key ) {
		/* normalize to the 'HKLM\SOFTWARE\...\Print\Printers' key */
		return regdb_fetch_values( KEY_WINNT_PRINTERS, values );
	}
	
	/* lookup the printer object */
	
	if (!reg_split_path( printers_key, &printername, &printerdatakey )) {
		return -1;
	}
	
	if ( !W_ERROR_IS_OK( get_a_printer(NULL, &printer, 2, printername) ) )
		goto done;
		
	if ( !printerdatakey ) {
		fill_in_printer_values( printer->info_2, values );
		goto done;
	}
		
	/* iterate over all printer data keys and fill the regval container */
	
	p_data = printer->info_2->data;
	if ( (key_index = lookup_printerkey( p_data, printerdatakey )) == -1  ) {
		/* failure....should never happen if the client has a valid open handle first */
		DEBUG(10,("key_printers_fetch_values: Unknown keyname [%s]\n", printerdatakey));
		free_a_printer( &printer, 2 );
		return -1;
	}
	
	num_values = regval_ctr_numvals( p_data->keys[key_index].values );	
	for ( i=0; i<num_values; i++ )
		regval_ctr_copyvalue( values, regval_ctr_specific_value(p_data->keys[key_index].values, i) );
			

done:
	if ( printer )
		free_a_printer( &printer, 2 );
		
	return regval_ctr_numvals( values );
}

/**********************************************************************
 *********************************************************************/

#define REG_IDX_ATTRIBUTES		1
#define REG_IDX_PRIORITY		2
#define REG_IDX_DEFAULT_PRIORITY	3
#define REG_IDX_CHANGEID		4
#define REG_IDX_STATUS			5
#define REG_IDX_STARTTIME		6
#define REG_IDX_NAME			7
#define REG_IDX_LOCATION		8
#define REG_IDX_DESCRIPTION		9
#define REG_IDX_PARAMETERS		10
#define REG_IDX_PORT			12
#define REG_IDX_SHARENAME		13
#define REG_IDX_DRIVER			14
#define REG_IDX_SEP_FILE		15
#define REG_IDX_PRINTPROC		16
#define REG_IDX_DATATYPE		17
#define REG_IDX_DEVMODE			18
#define REG_IDX_SECDESC			19
#define REG_IDX_UNTILTIME		20

struct {
	const char *name;
	int index;	
} printer_values_map[] = {
	{ "Attributes", 	REG_IDX_ATTRIBUTES },
	{ "Priority", 		REG_IDX_PRIORITY },
	{ "Default Priority", 	REG_IDX_DEFAULT_PRIORITY },
	{ "ChangeID", 		REG_IDX_CHANGEID },
	{ "Status", 		REG_IDX_STATUS },
	{ "StartTime", 		REG_IDX_STARTTIME },
	{ "UntilTime",	 	REG_IDX_UNTILTIME },
	{ "Name", 		REG_IDX_NAME },
	{ "Location", 		REG_IDX_LOCATION },
	{ "Description", 	REG_IDX_DESCRIPTION },
	{ "Parameters", 	REG_IDX_PARAMETERS },
	{ "Port", 		REG_IDX_PORT },
	{ "Share Name", 	REG_IDX_SHARENAME },
	{ "Printer Driver", 	REG_IDX_DRIVER },
	{ "Separator File", 	REG_IDX_SEP_FILE },
	{ "Print Processor", 	REG_IDX_PRINTPROC },
	{ "Datatype", 		REG_IDX_DATATYPE },
	{ "Default Devmode", 	REG_IDX_DEVMODE },
	{ "Security", 		REG_IDX_SECDESC },
	{ NULL, -1 }
};


static int find_valuename_index( const char *valuename )
{
	int i;
	
	for ( i=0; printer_values_map[i].name; i++ ) {
		if ( strequal( valuename, printer_values_map[i].name ) )
			return printer_values_map[i].index;
	}
	
	return -1;
}

/**********************************************************************
 *********************************************************************/

static void pull_reg_sz_fstring(TALLOC_CTX *mem_ctx, const DATA_BLOB *blob, fstring s)
{
	const char *str;
	pull_reg_sz(mem_ctx, blob, &str);
	fstrcpy(s, str);
}

static void convert_values_to_printer_info_2(TALLOC_CTX *mem_ctx,
					     NT_PRINTER_INFO_LEVEL_2 *printer2,
					     struct regval_ctr *values)
{
	int num_values = regval_ctr_numvals( values );
	uint32 value_index;
	struct regval_blob *val;
	int i;
	
	for ( i=0; i<num_values; i++ ) {
		DATA_BLOB blob;
		val = regval_ctr_specific_value( values, i );
		value_index = find_valuename_index( regval_name( val ) );

		blob = data_blob_const(regval_data_p(val), regval_size(val));
		
		switch( value_index ) {
			case REG_IDX_ATTRIBUTES:
				printer2->attributes = (uint32)(*regval_data_p(val));
				break;
			case REG_IDX_PRIORITY:
				printer2->priority = (uint32)(*regval_data_p(val));
				break;
			case REG_IDX_DEFAULT_PRIORITY:
				printer2->default_priority = (uint32)(*regval_data_p(val));
				break;
			case REG_IDX_CHANGEID:
				printer2->changeid = (uint32)(*regval_data_p(val));
				break;
			case REG_IDX_STARTTIME:
				printer2->starttime = (uint32)(*regval_data_p(val));
				break;
			case REG_IDX_UNTILTIME:
				printer2->untiltime = (uint32)(*regval_data_p(val));
				break;
			case REG_IDX_NAME:
				pull_reg_sz_fstring(mem_ctx, &blob, printer2->printername);
				break;
			case REG_IDX_LOCATION:
				pull_reg_sz_fstring(mem_ctx, &blob, printer2->location);
				break;
			case REG_IDX_DESCRIPTION:
				pull_reg_sz_fstring(mem_ctx, &blob, printer2->comment);
				break;
			case REG_IDX_PARAMETERS:
				pull_reg_sz_fstring(mem_ctx, &blob, printer2->parameters);
				break;
			case REG_IDX_PORT:
				pull_reg_sz_fstring(mem_ctx, &blob, printer2->portname);
				break;
			case REG_IDX_SHARENAME:
				pull_reg_sz_fstring(mem_ctx, &blob, printer2->sharename);
				break;
			case REG_IDX_DRIVER:
				pull_reg_sz_fstring(mem_ctx, &blob, printer2->drivername);
				break;
			case REG_IDX_SEP_FILE:
				pull_reg_sz_fstring(mem_ctx, &blob, printer2->sepfile);
				break;
			case REG_IDX_PRINTPROC:
				pull_reg_sz_fstring(mem_ctx, &blob, printer2->printprocessor);
				break;
			case REG_IDX_DATATYPE:
				pull_reg_sz_fstring(mem_ctx, &blob, printer2->datatype);
				break;
			case REG_IDX_DEVMODE:
				break;
			case REG_IDX_SECDESC:
				break;		
			default:
				/* unsupported value...throw away */
				DEBUG(8,("convert_values_to_printer_info_2: Unsupported registry value [%s]\n", 
					regval_name( val ) ));
		}
	}
	
	return;
}	

/**********************************************************************
 *********************************************************************/

static bool key_printers_store_values(const char *key, struct regval_ctr *values)
{
	char *printers_key;
	char *printername, *keyname;
	NT_PRINTER_INFO_LEVEL   *printer = NULL;
	WERROR result;
	TALLOC_CTX *mem_ctx = talloc_init("key_printers_store_values");
	
	printers_key = strip_printers_prefix( key );
	
	/* values in the top level key get stored in the registry */

	if ( !printers_key ) {
		/* normalize on the 'HKLM\SOFTWARE\....\Print\Printers' key */
		return regdb_store_values( KEY_WINNT_PRINTERS, values );
	}
	
	if (!reg_split_path( printers_key, &printername, &keyname )) {
		return False;
	}

	if ( !W_ERROR_IS_OK(get_a_printer(NULL, &printer, 2, printername) ) )
		return False;

	/* deal with setting values directly under the printername */

	if ( !keyname ) {
		convert_values_to_printer_info_2(mem_ctx, printer->info_2, values );
	}
	else {
		int num_values = regval_ctr_numvals( values );
		int i;
		struct regval_blob *val;
		
		delete_printer_key( printer->info_2->data, keyname );
		
		/* deal with any subkeys */
		for ( i=0; i<num_values; i++ ) {
			val = regval_ctr_specific_value( values, i );
			result = set_printer_dataex( printer, keyname, 
				regval_name( val ),
				regval_type( val ),
				regval_data_p( val ),
				regval_size( val ) );
			if ( !W_ERROR_IS_OK(result) ) {
				DEBUG(0,("key_printers_store_values: failed to set printer data [%s]!\n",
					keyname));
				free_a_printer( &printer, 2 );
				talloc_destroy(mem_ctx);
				return False;
			}
		}
	}

	result = mod_a_printer( printer, 2 );

	free_a_printer( &printer, 2 );
	talloc_destroy(mem_ctx);

	return W_ERROR_IS_OK(result);
}

/*********************************************************************
 *********************************************************************
 ** "HKLM/SYSTEM/CURRENTCONTROLSET/CONTROL/PRINT/ENVIRONMENTS"
 *********************************************************************
 *********************************************************************/

static int key_driver_fetch_keys( const char *key, struct regsubkey_ctr *subkeys )
{
	const char *environments[] = {
		"Windows 4.0",
		"Windows NT x86",
		"Windows NT R4000",
		"Windows NT Alpha_AXP",
		"Windows NT PowerPC",
		"Windows IA64",
		"Windows x64",
		NULL };
	fstring *drivers = NULL;
	int i, env_index, num_drivers;
	char *keystr, *base, *subkeypath;
	char *key2 = NULL;
	int num_subkeys = -1;
	int version;
	TALLOC_CTX *ctx = talloc_tos();

	DEBUG(10,("key_driver_fetch_keys key=>[%s]\n", key ? key : "NULL" ));

	keystr = reg_remaining_path(ctx, key + strlen(KEY_ENVIRONMENTS) );

	/* list all possible architectures */

	if ( !keystr ) {
		for ( num_subkeys=0; environments[num_subkeys]; num_subkeys++ )
			regsubkey_ctr_addkey( subkeys, 	environments[num_subkeys] );

		return num_subkeys;
	}

	/* we are dealing with a subkey of "Environments */
	key2 = talloc_strdup(ctx, keystr);
	if (!key2) {
		return -1;
	}
	keystr = key2;
	if (!reg_split_path(keystr, &base, &subkeypath )) {
		return -1;
	}

	/* sanity check */

	for ( env_index=0; environments[env_index]; env_index++ ) {
		if ( strequal( environments[env_index], base ) )
			break;
	}
	if ( !environments[env_index] )
		return -1;

	/* ...\Print\Environements\...\ */

	if ( !subkeypath ) {
		regsubkey_ctr_addkey( subkeys, "Drivers" );
		regsubkey_ctr_addkey( subkeys, "Print Processors" );

		return 2;
	}

	/* more of the key path to process */

	keystr = subkeypath;
	if (!reg_split_path( keystr, &base, &subkeypath )) {
		return -1;
	}

	/* ...\Print\Environements\...\Drivers\ */

	if ( !subkeypath ) {
		if ( strequal(base, "Drivers") ) {
			switch ( env_index ) {
				case 0:	/* Win9x */
					regsubkey_ctr_addkey( subkeys, "Version-0" );
					break;
				default: /* Windows NT based systems */
					regsubkey_ctr_addkey( subkeys, "Version-2" );
					regsubkey_ctr_addkey( subkeys, "Version-3" );
					break;
			}

			return regsubkey_ctr_numkeys( subkeys );
		} else if ( strequal(base, "Print Processors") ) {
			if ( env_index == 1 || env_index == 5 || env_index == 6 )


			return regsubkey_ctr_numkeys( subkeys );
		} else
			return -1;	/* bad path */
	}

	/* we finally get to enumerate the drivers */

	/* only one possible subkey below PrintProc key */

	if ( strequal(base, "Print Processors") ) {
		keystr = subkeypath;
		if (!reg_split_path( keystr, &base, &subkeypath )) {
			return -1;
		}

		/* no subkeys below this point */

		if ( subkeypath )
			return -1;

		/* only allow one keyname here -- 'winprint' */

		return strequal( base, "winprint" ) ? 0 : -1;
	}

	/* only dealing with drivers from here on out */

	keystr = subkeypath;
	if (!reg_split_path( keystr, &base, &subkeypath )) {
		return -1;
	}

	version = atoi(&base[strlen(base)-1]);

	switch (env_index) {
	case 0:
		if ( version != 0 )
			return -1;
		break;
	default:
		if ( version != 2 && version != 3 )
			return -1;
		break;
	}


	if ( !subkeypath ) {
		num_drivers = get_ntdrivers( &drivers, environments[env_index], version );
		for ( i=0; i<num_drivers; i++ )
			regsubkey_ctr_addkey( subkeys, drivers[i] );

		return regsubkey_ctr_numkeys( subkeys );
	}

	/* if anything else left, just say if has no subkeys */

	DEBUG(1,("key_driver_fetch_keys unhandled key [%s] (subkey == %s)\n",
		key, subkeypath ));

	return 0;
}


/**********************************************************************
 *********************************************************************/

static void fill_in_driver_values(const struct spoolss_DriverInfo8 *r,
				  struct regval_ctr *values)
{
	char *buffer = NULL;
	int buffer_size = 0;
	int i, length;
	const char *filename;
	DATA_BLOB data;

	filename = dos_basename(r->driver_path);
	regval_ctr_addvalue_sz(values, "Driver", filename);

	filename = dos_basename(r->config_file);
	regval_ctr_addvalue_sz(values, "Configuration File", filename);

	filename = dos_basename(r->data_file);
	regval_ctr_addvalue_sz(values, "Data File", filename);

	filename = dos_basename(r->help_file);
	regval_ctr_addvalue_sz(values, "Help File", filename);

	regval_ctr_addvalue_sz(values, "Data Type", r->default_datatype);

	regval_ctr_addvalue( values, "Version", REG_DWORD, (char*)&r->version,
		sizeof(r->version) );

	if (r->dependent_files) {
		/* place the list of dependent files in a single
		   character buffer, separating each file name by
		   a NULL */

		for (i=0; r->dependent_files[i] && strcmp(r->dependent_files[i], ""); i++) {
			/* strip the path to only the file's base name */

			filename = dos_basename(r->dependent_files[i]);

			length = strlen(filename);

			buffer = (char *)SMB_REALLOC( buffer, buffer_size + (length + 1)*sizeof(uint16) );
			if ( !buffer ) {
				break;
			}

			push_reg_sz(talloc_tos(), &data, filename);
			memcpy( buffer+buffer_size, (char*)data.data, data.length);

			buffer_size += (length + 1)*sizeof(uint16);
		}

		/* terminated by double NULL.  Add the final one here */

		buffer = (char *)SMB_REALLOC( buffer, buffer_size + 2 );
		if ( !buffer ) {
			buffer_size = 0;
		} else {
			buffer[buffer_size++] = '\0';
			buffer[buffer_size++] = '\0';
		}
	}

	regval_ctr_addvalue( values, "Dependent Files",    REG_MULTI_SZ, buffer, buffer_size );

	SAFE_FREE( buffer );

	return;
}

/**********************************************************************
 *********************************************************************/

static int driver_arch_fetch_values(char *key, struct regval_ctr *values)
{
	char 		*keystr, *base, *subkeypath;
	fstring		arch_environment;
	fstring		driver;
	int		version;
	struct spoolss_DriverInfo8 *driver_ctr;
	WERROR		w_result;

	if (!reg_split_path( key, &base, &subkeypath )) {
		return -1;
	}

	/* no values in 'Environments\Drivers\Windows NT x86' */

	if ( !subkeypath )
		return 0;

	/* We have the Architecture string and some subkey name:
	   Currently we only support
	   * Drivers
	   * Print Processors
	   Anything else is an error.
	   */

	fstrcpy( arch_environment, base );

	keystr = subkeypath;
	if (!reg_split_path( keystr, &base, &subkeypath )) {
		return -1;
	}

	if ( strequal(base, "Print Processors") )
		return 0;

	/* only Drivers key can be left */

	if ( !strequal(base, "Drivers") )
		return -1;

	if ( !subkeypath )
		return 0;

	/* We know that we have Architechure\Drivers with some subkey name
	   The subkey name has to be Version-XX */

	keystr = subkeypath;
	if (!reg_split_path( keystr, &base, &subkeypath )) {
		return -1;
	}

	if ( !subkeypath )
		return 0;

	version = atoi(&base[strlen(base)-1]);

	/* BEGIN PRINTER DRIVER NAME BLOCK */

	keystr = subkeypath;
	if (!reg_split_path( keystr, &base, &subkeypath )) {
		return -1;
	}

	/* don't go any deeper for now */

	fstrcpy( driver, base );

	w_result = get_a_printer_driver(talloc_tos(), &driver_ctr, driver, arch_environment, version);

	if ( !W_ERROR_IS_OK(w_result) )
		return -1;

	fill_in_driver_values(driver_ctr, values);

	free_a_printer_driver(driver_ctr);

	/* END PRINTER DRIVER NAME BLOCK */


	DEBUG(8,("key_driver_fetch_values: Exit\n"));

	return regval_ctr_numvals( values );
}

/**********************************************************************
 *********************************************************************/

static int key_driver_fetch_values(const char *key, struct regval_ctr *values)
{
	char *keystr = NULL;
	char *subkey = NULL;
	TALLOC_CTX *ctx = talloc_tos();

	DEBUG(8,("key_driver_fetch_values: Enter key => [%s]\n", key ? key : "NULL"));

	/* no values in the Environments key */

	if (!(keystr = reg_remaining_path(ctx, key + strlen(KEY_ENVIRONMENTS))))
		return 0;

	subkey = talloc_strdup(ctx, keystr);
	if (!subkey) {
		return 0;
	}

	/* pass off to handle subkeys */

	return driver_arch_fetch_values( subkey, values );
}

/*********************************************************************
 *********************************************************************
 ** "HKLM/SYSTEM/CURRENTCONTROLSET/CONTROL/PRINT"
 *********************************************************************
 *********************************************************************/

static int key_print_fetch_keys( const char *key, struct regsubkey_ctr *subkeys )
{
	int key_len = strlen(key);

	/* no keys below 'Print' handled here */

	if ( (key_len != strlen(KEY_CONTROL_PRINT)) && (key_len != strlen(KEY_WINNT_PRINT)) )
		return -1;

	regsubkey_ctr_addkey( subkeys, "Environments" );
	regsubkey_ctr_addkey( subkeys, "Monitors" );
	regsubkey_ctr_addkey( subkeys, "Forms" );
	regsubkey_ctr_addkey( subkeys, "Printers" );

	return regsubkey_ctr_numkeys( subkeys );
}

/**********************************************************************
 *********************************************************************
 ** Structure to hold dispatch table of ops for various printer keys.
 ** Make sure to always store deeper keys along the same path first so
 ** we ge a more specific match.
 *********************************************************************
 *********************************************************************/

static struct reg_dyn_tree print_registry[] = {
/* just pass the monitor onto the registry tdb */
{ KEY_MONITORS,
	&regdb_fetch_keys,
	&regdb_store_keys,
	&regdb_fetch_values,
	&regdb_store_values },
{ KEY_FORMS,
	&key_forms_fetch_keys,
	NULL,
	&key_forms_fetch_values,
	NULL },
{ KEY_CONTROL_PRINTERS,
	&key_printers_fetch_keys,
	&key_printers_store_keys,
	&key_printers_fetch_values,
	&key_printers_store_values },
{ KEY_ENVIRONMENTS,
	&key_driver_fetch_keys,
	NULL,
	&key_driver_fetch_values,
	NULL },
{ KEY_CONTROL_PRINT,
	&key_print_fetch_keys,
	NULL,
	NULL,
	NULL },
{ KEY_WINNT_PRINTERS,
	&key_printers_fetch_keys,
	&key_printers_store_keys,
	&key_printers_fetch_values,
	&key_printers_store_values },
{ KEY_PORTS,
	&regdb_fetch_keys,
	&regdb_store_keys,
	&regdb_fetch_values,
	&regdb_store_values },

{ NULL, NULL, NULL, NULL, NULL }
};


/**********************************************************************
 *********************************************************************
 ** Main reg_printing interface functions
 *********************************************************************
 *********************************************************************/

/***********************************************************************
 Lookup a key in the print_registry table, returning its index.
 -1 on failure
 **********************************************************************/

static int match_registry_path(const char *key)
{
	int i;
	char *path = NULL;
	TALLOC_CTX *ctx = talloc_tos();

	if ( !key )
		return -1;

	path = talloc_strdup(ctx, key);
	if (!path) {
		return -1;
	}
	path = normalize_reg_path(ctx, path);
	if (!path) {
		return -1;
	}

	for ( i=0; print_registry[i].path; i++ ) {
		if (strncmp( path, print_registry[i].path, strlen(print_registry[i].path) ) == 0 )
			return i;
	}

	return -1;
}

/***********************************************************************
 **********************************************************************/

static int regprint_fetch_reg_keys( const char *key, struct regsubkey_ctr *subkeys )
{
	int i = match_registry_path( key );

	if ( i == -1 )
		return -1;

	if ( !print_registry[i].fetch_subkeys )
		return -1;

	return print_registry[i].fetch_subkeys( key, subkeys );
}

/**********************************************************************
 *********************************************************************/

static bool regprint_store_reg_keys( const char *key, struct regsubkey_ctr *subkeys )
{
	int i = match_registry_path( key );

	if ( i == -1 )
		return False;

	if ( !print_registry[i].store_subkeys )
		return False;

	return print_registry[i].store_subkeys( key, subkeys );
}

/**********************************************************************
 *********************************************************************/

static int regprint_fetch_reg_values(const char *key, struct regval_ctr *values)
{
	int i = match_registry_path( key );

	if ( i == -1 )
		return -1;

	/* return 0 values by default since we know the key had
	   to exist because the client opened a handle */

	if ( !print_registry[i].fetch_values )
		return 0;

	return print_registry[i].fetch_values( key, values );
}

/**********************************************************************
 *********************************************************************/

static bool regprint_store_reg_values(const char *key, struct regval_ctr *values)
{
	int i = match_registry_path( key );

	if ( i == -1 )
		return False;

	if ( !print_registry[i].store_values )
		return False;

	return print_registry[i].store_values( key, values );
}

/*
 * Table of function pointers for accessing printing data
 */

struct registry_ops printing_ops = {
	.fetch_subkeys = regprint_fetch_reg_keys,
	.fetch_values = regprint_fetch_reg_values,
	.store_subkeys = regprint_store_reg_keys,
	.store_values = regprint_store_reg_values,
};
