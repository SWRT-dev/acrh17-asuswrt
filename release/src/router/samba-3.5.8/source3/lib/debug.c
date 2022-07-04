/*
   Unix SMB/CIFS implementation.
   Samba utility functions
   Copyright (C) Andrew Tridgell 1992-1998
   Copyright (C) Elrond               2002
   Copyright (C) Simo Sorce           2002

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "includes.h"

/* -------------------------------------------------------------------------- **
 * Defines...
 *
 *  FORMAT_BUFR_MAX - Index of the last byte of the format buffer;
 *                    format_bufr[FORMAT_BUFR_MAX] should always be reserved
 *                    for a terminating null byte.
 */

#define FORMAT_BUFR_SIZE 1024
#define FORMAT_BUFR_MAX (FORMAT_BUFR_SIZE - 1)

/* -------------------------------------------------------------------------- **
 * This module implements Samba's debugging utility.
 *
 * The syntax of a debugging log file is represented as:
 *
 *  <debugfile> :== { <debugmsg> }
 *
 *  <debugmsg>  :== <debughdr> '\n' <debugtext>
 *
 *  <debughdr>  :== '[' TIME ',' LEVEL ']' [ [FILENAME ':'] [FUNCTION '()'] ]
 *
 *  <debugtext> :== { <debugline> }
 *
 *  <debugline> :== TEXT '\n'
 *
 * TEXT     is a string of characters excluding the newline character.
 * LEVEL    is the DEBUG level of the message (an integer in the range 0..10).
 * TIME     is a timestamp.
 * FILENAME is the name of the file from which the debug message was generated.
 * FUNCTION is the function from which the debug message was generated.
 *
 * Basically, what that all means is:
 *
 * - A debugging log file is made up of debug messages.
 *
 * - Each debug message is made up of a header and text.  The header is
 *   separated from the text by a newline.
 *
 * - The header begins with the timestamp and debug level of the message
 *   enclosed in brackets.  The filename and function from which the
 *   message was generated may follow.  The filename is terminated by a
 *   colon, and the function name is terminated by parenthesis.
 *
 * - The message text is made up of zero or more lines, each terminated by
 *   a newline.
 */

/* -------------------------------------------------------------------------- **
 * External variables.
 *
 *  dbf           - Global debug file handle.
 *  debugf        - Debug file name.
 *  DEBUGLEVEL    - System-wide debug message limit.  Messages with message-
 *                  levels higher than DEBUGLEVEL will not be processed.
 */

XFILE   *dbf        = NULL;
static char *debugf = NULL;
bool    debug_warn_unknown_class = True;
bool    debug_auto_add_unknown_class = True;
bool    AllowDebugChange = True;

/*
   used to check if the user specified a
   logfile on the command line
*/
bool    override_logfile;

/*
 * This is to allow assignment to DEBUGLEVEL before the debug
 * system has been initialized.
 */
static int debug_all_class_hack = 1;
static bool debug_all_class_isset_hack = True;

static int debug_num_classes = 0;
int     *DEBUGLEVEL_CLASS = &debug_all_class_hack;
bool    *DEBUGLEVEL_CLASS_ISSET = &debug_all_class_isset_hack;

/* DEBUGLEVEL is #defined to *debug_level */
int     DEBUGLEVEL = &debug_all_class_hack;


/* -------------------------------------------------------------------------- **
 * Internal variables.
 *
 *  stdout_logging  - Default False, if set to True then dbf will be set to
 *                    stdout and debug output will go to dbf only, and not
 *                    to syslog.  Set in setup_logging() and read in Debug1().
 *
 *  debug_count     - Number of debug messages that have been output.
 *                    Used to check log size.
 *
 *  syslog_level    - Internal copy of the message debug level.  Written by
 *                    dbghdr() and read by Debug1().
 *
 *  format_bufr     - Used to format debug messages.  The dbgtext() function
 *                    prints debug messages to a string, and then passes the
 *                    string to format_debug_text(), which uses format_bufr
 *                    to build the formatted output.
 *
 *  format_pos      - Marks the first free byte of the format_bufr.
 * 
 *
 *  log_overflow    - When this variable is True, never attempt to check the
 *                    size of the log. This is a hack, so that we can write
 *                    a message using DEBUG, from open_logs() when we
 *                    are unable to open a new log file for some reason.
 */

static bool    stdout_logging = False;
static int     debug_count    = 0;
#ifdef WITH_SYSLOG
static int     syslog_level   = 0;
#endif
static char *format_bufr = NULL;
static size_t     format_pos     = 0;
static bool    log_overflow   = False;

/*
 * Define all the debug class selection names here. Names *MUST NOT* contain 
 * white space. There must be one name for each DBGC_<class name>, and they 
 * must be in the table in the order of DBGC_<class name>.. 
 */
static const char *default_classname_table[] = {
	"all",               /* DBGC_ALL; index refs traditional DEBUGLEVEL */
	"tdb",               /* DBGC_TDB	  */
	"printdrivers",      /* DBGC_PRINTDRIVERS */
	"lanman",            /* DBGC_LANMAN       */
	"smb",               /* DBGC_SMB          */
	"rpc_parse",         /* DBGC_RPC_PARSE    */
	"rpc_srv",           /* DBGC_RPC_SRV      */
	"rpc_cli",           /* DBGC_RPC_CLI      */
	"passdb",            /* DBGC_PASSDB       */
	"sam",               /* DBGC_SAM          */
	"auth",              /* DBGC_AUTH         */
	"winbind",           /* DBGC_WINBIND      */
	"vfs",		     /* DBGC_VFS	  */
	"idmap",	     /* DBGC_IDMAP	  */
	"quota",	     /* DBGC_QUOTA	  */
	"acls",		     /* DBGC_ACLS	  */
	"locking",	     /* DBGC_LOCKING	  */
	"msdfs",	     /* DBGC_MSDFS	  */
	"dmapi",	     /* DBGC_DMAPI	  */
	"registry",          /* DBGC_REGISTRY     */
	NULL
};

static char **classname_table = NULL;


/* -------------------------------------------------------------------------- **
 * Functions...
 */

/***************************************************************************
 Free memory pointed to by global pointers.
****************************************************************************/

static bool initialized;

void gfree_debugsyms(void)
{
	int i;

	if ( classname_table ) {
		for ( i = 0; i < debug_num_classes; i++ ) {
			SAFE_FREE( classname_table[i] );
		}
		SAFE_FREE( classname_table );
	}

	if ( DEBUGLEVEL_CLASS != &debug_all_class_hack ) {
		SAFE_FREE( DEBUGLEVEL_CLASS );
		DEBUGLEVEL_CLASS = &debug_all_class_hack;
	}

	if ( DEBUGLEVEL_CLASS_ISSET != &debug_all_class_isset_hack ) {
		SAFE_FREE( DEBUGLEVEL_CLASS_ISSET );
		DEBUGLEVEL_CLASS_ISSET = &debug_all_class_isset_hack;
	}

	SAFE_FREE(format_bufr);

	debug_num_classes = 0;

	debug_level = DEBUGLEVEL_CLASS;

	initialized = false;
}

/****************************************************************************
utility lists registered debug class names's
****************************************************************************/

#define MAX_CLASS_NAME_SIZE 1024

static char *debug_list_class_names_and_levels(void)
{
	int i, dim;
	char **list;
	char *buf = NULL;
	char *b;
	bool err = False;

	if (DEBUGLEVEL_CLASS == &debug_all_class_hack) {
		return NULL;
	}

	list = SMB_CALLOC_ARRAY(char *, debug_num_classes + 1);
	if (!list) {
		return NULL;
	}

	/* prepare strings */
	for (i = 0, dim = 0; i < debug_num_classes; i++) {
		int l = asprintf(&list[i],
				"%s:%d ",
				classname_table[i],
				DEBUGLEVEL_CLASS_ISSET[i]?DEBUGLEVEL_CLASS[i]:DEBUGLEVEL);
		if (l < 0 || l > MAX_CLASS_NAME_SIZE) {
			err = True;
			goto done;
		}
		dim += l;
	}

	/* create single string list - add space for newline */
	b = buf = (char *)SMB_MALLOC(dim+1);
	if (!buf) {
		err = True;
		goto done;
	}
	for (i = 0; i < debug_num_classes; i++) {
		int l = strlen(list[i]);
		strncpy(b, list[i], l);
		b = b + l;
	}
	b[-1] = '\n'; /* replace last space with newline */
	b[0] = '\0';  /* null terminate string */

done:
	/* free strings list */
	for (i = 0; i < debug_num_classes; i++) {
		SAFE_FREE(list[i]);
	}
	SAFE_FREE(list);

	if (err) {
		return NULL;
	} else {
		return buf;
	}
}

/****************************************************************************
 Utility access to debug class names's.
****************************************************************************/

const char *debug_classname_from_index(int ndx)
{
	if (ndx < 0 || ndx >= debug_num_classes)
		return NULL;
	else
		return classname_table[ndx];
}

/****************************************************************************
 Utility to translate names to debug class index's (internal version).
****************************************************************************/

static int debug_lookup_classname_int(const char* classname)
{
	int i;

	if (!classname) return -1;

	for (i=0; i < debug_num_classes; i++) {
		if (strcmp(classname, classname_table[i])==0)
			return i;
	}
	return -1;
}

/****************************************************************************
 Add a new debug class to the system.
****************************************************************************/

int debug_add_class(const char *classname)
{
	int ndx;
	void *new_ptr;

	if (!classname)
		return -1;

	/* check the init has yet been called */
	debug_init();

	ndx = debug_lookup_classname_int(classname);
	if (ndx >= 0)
		return ndx;
	ndx = debug_num_classes;

	new_ptr = DEBUGLEVEL_CLASS;
	if (DEBUGLEVEL_CLASS == &debug_all_class_hack) {
		/* Initial loading... */
		new_ptr = NULL;
	}
	new_ptr = SMB_REALLOC_ARRAY(new_ptr, int, debug_num_classes + 1);
	if (!new_ptr)
		return -1;
	DEBUGLEVEL_CLASS = (int *)new_ptr;
	DEBUGLEVEL_CLASS[ndx] = 0;

	/* debug_level is the pointer used for the DEBUGLEVEL-thingy */
	if (ndx==0) {
		/* Transfer the initial level from debug_all_class_hack */
		DEBUGLEVEL_CLASS[ndx] = DEBUGLEVEL;
	}
	debug_level = DEBUGLEVEL_CLASS;

	new_ptr = DEBUGLEVEL_CLASS_ISSET;
	if (new_ptr == &debug_all_class_isset_hack) {
		new_ptr = NULL;
	}
	new_ptr = SMB_REALLOC_ARRAY(new_ptr, bool, debug_num_classes + 1);
	if (!new_ptr)
		return -1;
	DEBUGLEVEL_CLASS_ISSET = (bool *)new_ptr;
	DEBUGLEVEL_CLASS_ISSET[ndx] = False;

	new_ptr = SMB_REALLOC_ARRAY(classname_table, char *, debug_num_classes + 1);
	if (!new_ptr)
		return -1;
	classname_table = (char **)new_ptr;

	classname_table[ndx] = SMB_STRDUP(classname);
	if (! classname_table[ndx])
		return -1;
	
	debug_num_classes++;

	return ndx;
}

/****************************************************************************
 Utility to translate names to debug class index's (public version).
****************************************************************************/

int debug_lookup_classname(const char *classname)
{
	int ndx;
       
	if (!classname || !*classname)
		return -1;

	ndx = debug_lookup_classname_int(classname);

	if (ndx != -1)
		return ndx;

	if (debug_warn_unknown_class) {
		DEBUG(0, ("debug_lookup_classname(%s): Unknown class\n",
			  classname));
	}
	if (debug_auto_add_unknown_class) {
		return debug_add_class(classname);
	}
	return -1;
}

/****************************************************************************
 Dump the current registered debug levels.
****************************************************************************/

static void debug_dump_status(int level)
{
	int q;

	DEBUG(level, ("INFO: Current debug levels:\n"));
	for (q = 0; q < debug_num_classes; q++) {
		DEBUGADD(level, ("  %s: %s/%d\n",
				 classname_table[q],
				 (DEBUGLEVEL_CLASS_ISSET[q]
				  ? "True" : "False"),
				 DEBUGLEVEL_CLASS[q]));
	}
}

/****************************************************************************
 parse the debug levels from smbcontrol. Example debug level parameter:
 printdrivers:7
****************************************************************************/

static bool debug_parse_params(char **params)
{
	int   i, ndx;
	char *class_name;
	char *class_level;

	if (!params)
		return False;

	/* Allow DBGC_ALL to be specified w/o requiring its class name e.g."10"  
	 * v.s. "all:10", this is the traditional way to set DEBUGLEVEL 
	 */
	if (isdigit((int)params[0][0])) {
		DEBUGLEVEL_CLASS[DBGC_ALL] = atoi(params[0]);
		DEBUGLEVEL_CLASS_ISSET[DBGC_ALL] = True;
		i = 1; /* start processing at the next params */
	} else {
		i = 0; /* DBGC_ALL not specified OR class name was included */
	}

	/* Fill in new debug class levels */
	for (; i < debug_num_classes && params[i]; i++) {
		char *saveptr;
		if ((class_name = strtok_r(params[i],":", &saveptr)) &&
			(class_level = strtok_r(NULL, "\0", &saveptr)) &&
            ((ndx = debug_lookup_classname(class_name)) != -1)) {
				DEBUGLEVEL_CLASS[ndx] = atoi(class_level);
				DEBUGLEVEL_CLASS_ISSET[ndx] = True;
		} else {
			DEBUG(0,("debug_parse_params: unrecognized debug class name or format [%s]\n", params[i]));
			return False;
		}
	}

	return True;
}

/****************************************************************************
 Parse the debug levels from smb.conf. Example debug level string:
  3 tdb:5 printdrivers:7
 Note: the 1st param has no "name:" preceeding it.
****************************************************************************/

bool debug_parse_levels(const char *params_str)
{
	char **params;

	/* Just in case */
	debug_init();

	if (AllowDebugChange == False)
		return True;

	params = str_list_make_v3(talloc_tos(), params_str, NULL);

	if (debug_parse_params(params)) {
		debug_dump_status(5);
		TALLOC_FREE(params);
		return True;
	} else {
		TALLOC_FREE(params);
		return False;
	}
}

/****************************************************************************
 Receive a "set debug level" message.
****************************************************************************/

void debug_message(struct messaging_context *msg_ctx,
			  void *private_data, 
			  uint32_t msg_type, 
			  struct server_id src,
			  DATA_BLOB *data)
{
	const char *params_str = (const char *)data->data;

	/* Check, it's a proper string! */
	if (params_str[(data->length)-1] != '\0') {
		DEBUG(1, ("Invalid debug message from pid %u to pid %u\n",
			  (unsigned int)procid_to_pid(&src),
			  (unsigned int)getpid()));
		return;
	}

	DEBUG(3, ("INFO: Remote set of debug to `%s'  (pid %u from pid %u)\n",
		  params_str, (unsigned int)getpid(),
		  (unsigned int)procid_to_pid(&src)));

	debug_parse_levels(params_str);
}

/****************************************************************************
 Return current debug level.
****************************************************************************/

static void debuglevel_message(struct messaging_context *msg_ctx,
			       void *private_data, 
			       uint32_t msg_type, 
			       struct server_id src,
			       DATA_BLOB *data)
{
	char *message = debug_list_class_names_and_levels();

	if (!message) {
		DEBUG(0,("debuglevel_message - debug_list_class_names_and_levels returned NULL\n"));
		return;
	}

	DEBUG(1,("INFO: Received REQ_DEBUGLEVEL message from PID %s\n",
		 procid_str_static(&src)));
	messaging_send_buf(msg_ctx, src, MSG_DEBUGLEVEL,
			   (uint8 *)message, strlen(message) + 1);

	SAFE_FREE(message);
}

/****************************************************************************
Init debugging (one time stuff)
****************************************************************************/

void debug_init(void)
{
	const char **p;

	if (initialized)
		return;

	initialized = true;

	for(p = default_classname_table; *p; p++) {
		debug_add_class(*p);
	}
	format_bufr = (char *)SMB_MALLOC(FORMAT_BUFR_SIZE);
	if (!format_bufr) {
		smb_panic("debug_init: unable to create buffer");
	}
}

void debug_register_msgs(struct messaging_context *msg_ctx)
{
	messaging_register(msg_ctx, NULL, MSG_DEBUG, debug_message);
	messaging_register(msg_ctx, NULL, MSG_REQ_DEBUGLEVEL,
			   debuglevel_message);
}

/***************************************************************************
 Get ready for syslog stuff
**************************************************************************/

void setup_logging(const char *pname, bool interactive)
{
	debug_init();

	/* reset to allow multiple setup calls, going from interactive to
	   non-interactive */
	stdout_logging = False;
	if (dbf) {
		x_fflush(dbf);
                if (dbf != x_stdout) {
                        (void) x_fclose(dbf);
                }
	}

	dbf = NULL;

	if (interactive) {
		stdout_logging = True;
		dbf = x_stdout;
		x_setbuf( x_stdout, NULL );
	}
#ifdef WITH_SYSLOG
	else {
		const char *p = strrchr_m( pname,'/' );
		if (p)
			pname = p + 1;
#ifdef LOG_DAEMON
		openlog( pname, LOG_PID, SYSLOG_FACILITY );
#else
		/* for old systems that have no facility codes. */
		openlog( pname, LOG_PID );
#endif
	}
#endif
}

/**
   Just run logging to stdout for this program 
*/
_PUBLIC_ void setup_logging_stdout(void)
{
	setup_logging(NULL, True);
}


/***************************************************************************
 Set the logfile name.
**************************************************************************/

void debug_set_logfile(const char *name)
{
	SAFE_FREE(debugf);
	debugf = SMB_STRDUP(name);
}

/**************************************************************************
 reopen the log files
 note that we now do this unconditionally
 We attempt to open the new debug fp before closing the old. This means
 if we run out of fd's we just keep using the old fd rather than aborting.
 Fix from dgibson@linuxcare.com.
**************************************************************************/

bool reopen_logs( void )
{
	char *fname = NULL;
	mode_t oldumask;
	XFILE *new_dbf = NULL;
	XFILE *old_dbf = NULL;
	bool ret = True;

	if (stdout_logging)
		return True;

	oldumask = umask( 022 );

	fname = debugf;
	if (!fname) {
		return false;
	}
	debugf = NULL;

	if (lp_loaded()) {
		char *logfname;

		logfname = lp_logfile();
		if (*logfname) {
			SAFE_FREE(fname);
			fname = SMB_STRDUP(logfname);
			if (!fname) {
				return false;
			}
		}
	}

	debugf = fname;
	new_dbf = x_fopen( debugf, O_WRONLY|O_APPEND|O_CREAT, 0644);

	if (!new_dbf) {
		log_overflow = True;
		DEBUG(0, ("Unable to open new log file %s: %s\n", debugf, strerror(errno)));
		log_overflow = False;
		if (dbf)
			x_fflush(dbf);
		ret = False;
	} else {
		x_setbuf(new_dbf, NULL);
		old_dbf = dbf;
		dbf = new_dbf;
		if (old_dbf)
			(void) x_fclose(old_dbf);
	}

	/* Fix from klausr@ITAP.Physik.Uni-Stuttgart.De
	 * to fix problem where smbd's that generate less
	 * than 100 messages keep growing the log.
	 */
	force_check_log_size();
	(void)umask(oldumask);

	/* Take over stderr to catch output into logs */
	if (dbf && dup2(x_fileno(dbf), 2) == -1) {
		close_low_fds(True); /* Close stderr too, if dup2 can't point it
					at the logfile */
	}

	return ret;
}

/**************************************************************************
 Force a check of the log size.
 ***************************************************************************/

void force_check_log_size( void )
{
	debug_count = 100;
}

/***************************************************************************
 Check to see if there is any need to check if the logfile has grown too big.
**************************************************************************/

bool need_to_check_log_size( void )
{
	int maxlog;

	if( debug_count < 100 )
		return( False );

	maxlog = lp_max_log_size() * 1024;
	if( !dbf || maxlog <= 0 ) {
		debug_count = 0;
		return(False);
	}
	return( True );
}

/**************************************************************************
 Check to see if the log has grown to be too big.
 **************************************************************************/

void check_log_size( void )
{
	int         maxlog;
	SMB_STRUCT_STAT st;

	/*
	 *  We need to be root to check/change log-file, skip this and let the main
	 *  loop check do a new check as root.
	 */

	if( geteuid() != 0 )
		return;

	if(log_overflow || !need_to_check_log_size() )
		return;

	maxlog = lp_max_log_size() * 1024;

	if(sys_fstat(x_fileno(dbf), &st, false) == 0
	   && st.st_ex_size > maxlog ) {
		(void)reopen_logs();
		if( dbf && get_file_size( debugf ) > maxlog ) {
			char *name = NULL;

			if (asprintf(&name, "%s.old", debugf ) < 0) {
				return;
			}
			(void)rename(debugf, name);

			if (!reopen_logs()) {
				/* We failed to reopen a log - continue using the old name. */
				(void)rename(name, debugf);
			}
			SAFE_FREE(name);
		}
	}

	/*
	 * Here's where we need to panic if dbf == NULL..
	 */

	if(dbf == NULL) {
		/* This code should only be reached in very strange
		 * circumstances. If we merely fail to open the new log we
		 * should stick with the old one. ergo this should only be
		 * reached when opening the logs for the first time: at
		 * startup or when the log level is increased from zero.
		 * -dwg 6 June 2000
		 */
		dbf = x_fopen( "/dev/console", O_WRONLY, 0);
		if(dbf) {
			DEBUG(0,("check_log_size: open of debug file %s failed - using console.\n",
					debugf ));
		} else {
			/*
			 * We cannot continue without a debug file handle.
			 */
			abort();
		}
	}
	debug_count = 0;
}

/*************************************************************************
 Write an debug message on the debugfile.
 This is called by dbghdr() and format_debug_text().
************************************************************************/

 int Debug1( const char *format_str, ... )
{
	va_list ap;
	int old_errno = errno;

	debug_count++;

	if( stdout_logging ) {
		va_start( ap, format_str );
		if(dbf)
			(void)x_vfprintf( dbf, format_str, ap );
		va_end( ap );
		errno = old_errno;
		goto done;
	}

	/* prevent recursion by checking if reopen_logs() has temporaily
	   set the debugf string to NULL */
	if( debugf == NULL)
		goto done;

#ifdef WITH_SYSLOG
	if( !lp_syslog_only() )
#endif
	{
		if( !dbf ) {
			mode_t oldumask = umask( 022 );

			dbf = x_fopen( debugf, O_WRONLY|O_APPEND|O_CREAT, 0644 );
			(void)umask( oldumask );
			if( dbf ) {
				x_setbuf( dbf, NULL );
			} else {
				errno = old_errno;
				goto done;
			}
		}
	}

#ifdef WITH_SYSLOG
	if( syslog_level < lp_syslog() ) {
		/* map debug levels to syslog() priorities
		 * note that not all DEBUG(0, ...) calls are
		 * necessarily errors */
		static const int priority_map[4] = {
			LOG_ERR,     /* 0 */
			LOG_WARNING, /* 1 */
			LOG_NOTICE,  /* 2 */
			LOG_INFO,    /* 3 */
		};
		int     priority;
		char *msgbuf = NULL;
		int ret;

		if( syslog_level >= ARRAY_SIZE(priority_map) || syslog_level < 0)
			priority = LOG_DEBUG;
		else
			priority = priority_map[syslog_level];

		/*
		 * Specify the facility to interoperate with other syslog
		 * callers (vfs_full_audit for example).
		 */
		priority |= SYSLOG_FACILITY;

		va_start(ap, format_str);
		ret = vasprintf(&msgbuf, format_str, ap);
		va_end(ap);

		if (ret != -1) {
			syslog(priority, "%s", msgbuf);
		}
		SAFE_FREE(msgbuf);
	}
#endif

	check_log_size();

#ifdef WITH_SYSLOG
	if( !lp_syslog_only() )
#endif
	{
		va_start( ap, format_str );
		if(dbf)
			(void)x_vfprintf( dbf, format_str, ap );
		va_end( ap );
		if(dbf)
			(void)x_fflush( dbf );
	}

 done:
	errno = old_errno;

	return( 0 );
}


/**************************************************************************
 Print the buffer content via Debug1(), then reset the buffer.
 Input:  none
 Output: none
****************************************************************************/

static void bufr_print( void )
{
	format_bufr[format_pos] = '\0';
	(void)Debug1( "%s", format_bufr );
	format_pos = 0;
}

/***************************************************************************
 Format the debug message text.

 Input:  msg - Text to be added to the "current" debug message text.

 Output: none.

 Notes:  The purpose of this is two-fold.  First, each call to syslog()
         (used by Debug1(), see above) generates a new line of syslog
         output.  This is fixed by storing the partial lines until the
         newline character is encountered.  Second, printing the debug
         message lines when a newline is encountered allows us to add
         spaces, thus indenting the body of the message and making it
         more readable.
**************************************************************************/

static void format_debug_text( const char *msg )
{
	size_t i;
	bool timestamp = (!stdout_logging && (lp_timestamp_logs() || !(lp_loaded())));

	if (!format_bufr) {
		debug_init();
	}

	for( i = 0; msg[i]; i++ ) {
		/* Indent two spaces at each new line. */
		if(timestamp && 0 == format_pos) {
			format_bufr[0] = format_bufr[1] = ' ';
			format_pos = 2;
		}

		/* If there's room, copy the character to the format buffer. */
		if( format_pos < FORMAT_BUFR_MAX )
			format_bufr[format_pos++] = msg[i];

		/* If a newline is encountered, print & restart. */
		if( '\n' == msg[i] )
			bufr_print();

		/* If the buffer is full dump it out, reset it, and put out a line
		 * continuation indicator.
		 */
		if( format_pos >= FORMAT_BUFR_MAX ) {
			bufr_print();
			(void)Debug1( " +>\n" );
		}
	}

	/* Just to be safe... */
	format_bufr[format_pos] = '\0';
}

/***************************************************************************
 Flush debug output, including the format buffer content.

 Input:  none
 Output: none
***************************************************************************/

void dbgflush( void )
{
	bufr_print();
	if(dbf)
		(void)x_fflush( dbf );
}

/***************************************************************************
 Print a Debug Header.

 Input:  level - Debug level of the message (not the system-wide debug
                  level. )
	  cls   - Debuglevel class of the calling module.
          file  - Pointer to a string containing the name of the file
                  from which this function was called, or an empty string
                  if the __FILE__ macro is not implemented.
          func  - Pointer to a string containing the name of the function
                  from which this function was called, or an empty string
                  if the __FUNCTION__ macro is not implemented.
         line  - line number of the call to dbghdr, assuming __LINE__
                 works.

  Output: Always True.  This makes it easy to fudge a call to dbghdr()
          in a macro, since the function can be called as part of a test.
          Eg: ( (level <= DEBUGLEVEL) && (dbghdr(level,"",line)) )

  Notes:  This function takes care of setting syslog_level.

****************************************************************************/

bool dbghdrclass(int level, int cls, const char *location, const char *func)
{
	/* Ensure we don't lose any real errno value. */
	int old_errno = errno;

	if( format_pos ) {
		/* This is a fudge.  If there is stuff sitting in the format_bufr, then
		 * the *right* thing to do is to call
		 *   format_debug_text( "\n" );
		 * to write the remainder, and then proceed with the new header.
		 * Unfortunately, there are several places in the code at which
		 * the DEBUG() macro is used to build partial lines.  That in mind,
		 * we'll work under the assumption that an incomplete line indicates
		 * that a new header is *not* desired.
		 */
		return( True );
	}

#ifdef WITH_SYSLOG
	/* Set syslog_level. */
	syslog_level = level;
#endif

	/* Don't print a header if we're logging to stdout. */
	if( stdout_logging )
		return( True );

	/* Print the header if timestamps are turned on.  If parameters are
	 * not yet loaded, then default to timestamps on.
	 */
	if( lp_timestamp_logs() || lp_debug_prefix_timestamp() || !(lp_loaded()) ) {
		char header_str[200];

		header_str[0] = '\0';

		if( lp_debug_pid())
			slprintf(header_str,sizeof(header_str)-1,", pid=%u",(unsigned int)sys_getpid());

		if( lp_debug_uid()) {
			size_t hs_len = strlen(header_str);
			slprintf(header_str + hs_len,
			sizeof(header_str) - 1 - hs_len,
				", effective(%u, %u), real(%u, %u)",
				(unsigned int)geteuid(), (unsigned int)getegid(),
				(unsigned int)getuid(), (unsigned int)getgid()); 
		}

		if (lp_debug_class() && (cls != DBGC_ALL)) {
			size_t hs_len = strlen(header_str);
			slprintf(header_str + hs_len,
				 sizeof(header_str) -1 - hs_len,
				 ", class=%s",
				 default_classname_table[cls]);
		}
  
		/* Print it all out at once to prevent split syslog output. */
		if( lp_debug_prefix_timestamp() ) {
		    (void)Debug1( "[%s, %2d%s] ",
			current_timestring(talloc_tos(),
					   lp_debug_hires_timestamp()),
			level, header_str);
		} else {
		    (void)Debug1( "[%s, %2d%s] %s(%s)\n",
			current_timestring(talloc_tos(),
					   lp_debug_hires_timestamp()),
			level, header_str, location, func );
		}
	}

	errno = old_errno;
	return( True );
}

bool dbghdr(int level, const char *location, const char *func)
{
	/* For compatibility with Samba 4, which doesn't have debug classes */
	return dbghdrclass(level, 0, location, func);
}

/***************************************************************************
 Add text to the body of the "current" debug message via the format buffer.

  Input:  format_str  - Format string, as used in printf(), et. al.
          ...         - Variable argument list.

  ..or..  va_alist    - Old style variable parameter list starting point.

  Output: Always True.  See dbghdr() for more info, though this is not
          likely to be used in the same way.

***************************************************************************/

 bool dbgtext( const char *format_str, ... )
{
	va_list ap;
	char *msgbuf = NULL;
	bool ret = true;
	int res;

	va_start(ap, format_str);
	res = vasprintf(&msgbuf, format_str, ap);
	va_end(ap);

	if (res != -1) {
		format_debug_text(msgbuf);
	} else {
		ret = false;
	}
	SAFE_FREE(msgbuf);
	return ret;
}
