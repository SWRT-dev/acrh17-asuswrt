/*
 * Copyright (c) 1997 - 2004 Kungliga Tekniska Högskolan
 * (Royal Institute of Technology, Stockholm, Sweden).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "krb5_locl.h"

/* Gaah! I want a portable funopen */
struct fileptr {
    const char *s;
    FILE *f;
};

static char *
config_fgets(char *str, size_t len, struct fileptr *ptr)
{
    /* XXX this is not correct, in that they don't do the same if the
       line is longer than len */
    if(ptr->f != NULL)
	return fgets(str, len, ptr->f);
    else {
	/* this is almost strsep_copy */
	const char *p;
	ssize_t l;
	if(*ptr->s == '\0')
	    return NULL;
	p = ptr->s + strcspn(ptr->s, "\n");
	if(*p == '\n')
	    p++;
	l = min(len, p - ptr->s);
	if(len > 0) {
	    memcpy(str, ptr->s, l);
	    str[l] = '\0';
	}
	ptr->s = p;
	return str;
    }
}

static krb5_error_code parse_section(char *p, krb5_config_section **s,
				     krb5_config_section **res,
				     const char **error_message);
static krb5_error_code parse_binding(struct fileptr *f, unsigned *lineno, char *p,
				     krb5_config_binding **b,
				     krb5_config_binding **parent,
				     const char **error_message);
static krb5_error_code parse_list(struct fileptr *f, unsigned *lineno,
				  krb5_config_binding **parent,
				  const char **error_message);

static krb5_config_section *
get_entry(krb5_config_section **parent, const char *name, int type)
{
    krb5_config_section **q;

    for(q = parent; *q != NULL; q = &(*q)->next)
	if(type == krb5_config_list &&
	   type == (*q)->type &&
	   strcmp(name, (*q)->name) == 0)
	    return *q;
    *q = calloc(1, sizeof(**q));
    if(*q == NULL)
	return NULL;
    (*q)->name = strdup(name);
    (*q)->type = type;
    if((*q)->name == NULL) {
	free(*q);
	*q = NULL;
	return NULL;
    }
    return *q;
}

/*
 * Parse a section:
 *
 * [section]
 *	foo = bar
 *	b = {
 *		a
 *	    }
 * ...
 *
 * starting at the line in `p', storing the resulting structure in
 * `s' and hooking it into `parent'.
 * Store the error message in `error_message'.
 */

static krb5_error_code
parse_section(char *p, krb5_config_section **s, krb5_config_section **parent,
	      const char **error_message)
{
    char *p1;
    krb5_config_section *tmp;

    p1 = strchr (p + 1, ']');
    if (p1 == NULL) {
	*error_message = "missing ]";
	return KRB5_CONFIG_BADFORMAT;
    }
    *p1 = '\0';
    tmp = get_entry(parent, p + 1, krb5_config_list);
    if(tmp == NULL) {
	*error_message = "out of memory";
	return KRB5_CONFIG_BADFORMAT;
    }
    *s = tmp;
    return 0;
}

/*
 * Parse a brace-enclosed list from `f', hooking in the structure at
 * `parent'.
 * Store the error message in `error_message'.
 */

static krb5_error_code
parse_list(struct fileptr *f, unsigned *lineno, krb5_config_binding **parent,
	   const char **error_message)
{
    char buf[BUFSIZ];
    krb5_error_code ret;
    krb5_config_binding *b = NULL;
    unsigned beg_lineno = *lineno;

    while(config_fgets(buf, sizeof(buf), f) != NULL) {
	char *p;

	++*lineno;
	buf[strcspn(buf, "\r\n")] = '\0';
	p = buf;
	while(isspace((unsigned char)*p))
	    ++p;
	if (*p == '#' || *p == ';' || *p == '\0')
	    continue;
	while(isspace((unsigned char)*p))
	    ++p;
	if (*p == '}')
	    return 0;
	if (*p == '\0')
	    continue;
	ret = parse_binding (f, lineno, p, &b, parent, error_message);
	if (ret)
	    return ret;
    }
    *lineno = beg_lineno;
    *error_message = "unclosed {";
    return KRB5_CONFIG_BADFORMAT;
}

/*
 *
 */

static krb5_error_code
parse_binding(struct fileptr *f, unsigned *lineno, char *p,
	      krb5_config_binding **b, krb5_config_binding **parent,
	      const char **error_message)
{
    krb5_config_binding *tmp;
    char *p1, *p2;
    krb5_error_code ret = 0;

    p1 = p;
    while (*p && *p != '=' && !isspace((unsigned char)*p))
	++p;
    if (*p == '\0') {
	*error_message = "missing =";
	return KRB5_CONFIG_BADFORMAT;
    }
    p2 = p;
    while (isspace((unsigned char)*p))
	++p;
    if (*p != '=') {
	*error_message = "missing =";
	return KRB5_CONFIG_BADFORMAT;
    }
    ++p;
    while(isspace((unsigned char)*p))
	++p;
    *p2 = '\0';
    if (*p == '{') {
	tmp = get_entry(parent, p1, krb5_config_list);
	if (tmp == NULL) {
	    *error_message = "out of memory";
	    return KRB5_CONFIG_BADFORMAT;
	}
	ret = parse_list (f, lineno, &tmp->u.list, error_message);
    } else {
	tmp = get_entry(parent, p1, krb5_config_string);
	if (tmp == NULL) {
	    *error_message = "out of memory";
	    return KRB5_CONFIG_BADFORMAT;
	}
	p1 = p;
	p = p1 + strlen(p1);
	while(p > p1 && isspace((unsigned char)*(p-1)))
	    --p;
	*p = '\0';
	tmp->u.string = strdup(p1);
    }
    *b = tmp;
    return ret;
}

/*
 * Parse the config file `fname', generating the structures into `res'
 * returning error messages in `error_message'
 */

static krb5_error_code
krb5_config_parse_debug (struct fileptr *f,
			 krb5_config_section **res,
			 unsigned *lineno,
			 const char **error_message)
{
    krb5_config_section *s = NULL;
    krb5_config_binding *b = NULL;
    char buf[BUFSIZ];
    krb5_error_code ret;

    while (config_fgets(buf, sizeof(buf), f) != NULL) {
	char *p;

	++*lineno;
	buf[strcspn(buf, "\r\n")] = '\0';
	p = buf;
	while(isspace((unsigned char)*p))
	    ++p;
	if (*p == '#' || *p == ';')
	    continue;
	if (*p == '[') {
	    ret = parse_section(p, &s, res, error_message);
	    if (ret)
		return ret;
	    b = NULL;
	} else if (*p == '}') {
	    *error_message = "unmatched }";
	    return EINVAL;	/* XXX */
	} else if(*p != '\0') {
	    if (s == NULL) {
		*error_message = "binding before section";
		return EINVAL;
	    }
	    ret = parse_binding(f, lineno, p, &b, &s->u.list, error_message);
	    if (ret)
		return ret;
	}
    }
    return 0;
}

krb5_error_code KRB5_LIB_FUNCTION
krb5_config_parse_string_multi(krb5_context context,
			       const char *string,
			       krb5_config_section **res)
{
    const char *str;
    unsigned lineno = 0;
    krb5_error_code ret;
    struct fileptr f;
    f.f = NULL;
    f.s = string;

    ret = krb5_config_parse_debug (&f, res, &lineno, &str);
    if (ret) {
	krb5_set_error_message (context, ret, "%s:%u: %s",
				"<constant>", lineno, str);
	return ret;
    }
    return 0;
}

/**
 * Parse a configuration file and add the result into res. This
 * interface can be used to parse several configuration files into one
 * resulting krb5_config_section by calling it repeatably.
 *
 * @param context a Kerberos 5 context.
 * @param fname a file name to a Kerberos configuration file
 * @param res the returned result, must be free with krb5_free_config_files().
 * @return Return an error code or 0, see krb5_get_error_message().
 *
 * @ingroup krb5_support
 */

krb5_error_code KRB5_LIB_FUNCTION
krb5_config_parse_file_multi (krb5_context context,
			      const char *fname,
			      krb5_config_section **res)
{
    const char *str;
    char *newfname = NULL;
    unsigned lineno = 0;
    krb5_error_code ret;
    struct fileptr f;

    /**
     * If the fname starts with "~/" parse configuration file in the
     * current users home directory. The behavior can be disabled and
     * enabled by calling krb5_set_home_dir_access().
     */
    if (_krb5_homedir_access(context) && fname[0] == '~' && fname[1] == '/') {
	const char *home = NULL;

	if(!issuid())
	    home = getenv("HOME");

	if (home == NULL) {
	    struct passwd *pw = getpwuid(getuid());	
	    if(pw != NULL)
		home = pw->pw_dir;
	}
	if (home) {
	    asprintf(&newfname, "%s%s", home, &fname[1]);
	    if (newfname == NULL) {
		krb5_set_error_message(context, ENOMEM,
				       N_("malloc: out of memory", ""));
		return ENOMEM;
	    }
	    fname = newfname;
	}
    }

    f.f = fopen(fname, "r");
    f.s = NULL;
    if(f.f == NULL) {
	ret = errno;
	krb5_set_error_message (context, ret, "open %s: %s",
				fname, strerror(ret));
	if (newfname)
	    free(newfname);
	return ret;
    }

    ret = krb5_config_parse_debug (&f, res, &lineno, &str);
    fclose(f.f);
    if (ret) {
	krb5_set_error_message (context, ret, "%s:%u: %s", fname, lineno, str);
	if (newfname)
	    free(newfname);
	return ret;
    }
    if (newfname)
	free(newfname);
    return 0;
}

krb5_error_code KRB5_LIB_FUNCTION
krb5_config_parse_file (krb5_context context,
			const char *fname,
			krb5_config_section **res)
{
    *res = NULL;
    return krb5_config_parse_file_multi(context, fname, res);
}

static void
free_binding (krb5_context context, krb5_config_binding *b)
{
    krb5_config_binding *next_b;

    while (b) {
	free (b->name);
	if (b->type == krb5_config_string)
	    free (b->u.string);
	else if (b->type == krb5_config_list)
	    free_binding (context, b->u.list);
	else
	    krb5_abortx(context, "unknown binding type (%d) in free_binding",
			b->type);
	next_b = b->next;
	free (b);
	b = next_b;
    }
}

krb5_error_code KRB5_LIB_FUNCTION
krb5_config_file_free (krb5_context context, krb5_config_section *s)
{
    free_binding (context, s);
    return 0;
}

krb5_error_code
_krb5_config_copy(krb5_context context,
		  krb5_config_section *c,
		  krb5_config_section **head)
{
    krb5_config_binding *d, *previous = NULL;

    *head = NULL;

    while (c) {
	d = calloc(1, sizeof(*d));

	if (*head == NULL)
	    *head = d;

	d->name = strdup(c->name);
	d->type = c->type;
	if (d->type == krb5_config_string)
	    d->u.string = strdup(c->u.string);
	else if (d->type == krb5_config_list)
	    _krb5_config_copy (context, c->u.list, &d->u.list);
	else
	    krb5_abortx(context,
			"unknown binding type (%d) in krb5_config_copy",
			d->type);
	if (previous)
	    previous->next = d;

	previous = d;
	c = c->next;
    }
    return 0;
}



const void *
krb5_config_get_next (krb5_context context,
		      const krb5_config_section *c,
		      const krb5_config_binding **pointer,
		      int type,
		      ...)
{
    const char *ret;
    va_list args;

    va_start(args, type);
    ret = krb5_config_vget_next (context, c, pointer, type, args);
    va_end(args);
    return ret;
}

static const void *
vget_next(krb5_context context,
	  const krb5_config_binding *b,
	  const krb5_config_binding **pointer,
	  int type,
	  const char *name,
	  va_list args)
{
    const char *p = va_arg(args, const char *);
    while(b != NULL) {
	if(strcmp(b->name, name) == 0) {
	    if(b->type == type && p == NULL) {
		*pointer = b;
		return b->u.generic;
	    } else if(b->type == krb5_config_list && p != NULL) {
		return vget_next(context, b->u.list, pointer, type, p, args);
	    }
	}
	b = b->next;
    }
    return NULL;
}

const void *
krb5_config_vget_next (krb5_context context,
		       const krb5_config_section *c,
		       const krb5_config_binding **pointer,
		       int type,
		       va_list args)
{
    const krb5_config_binding *b;
    const char *p;

    if(c == NULL)
	c = context->cf;

    if (c == NULL)
	return NULL;

    if (*pointer == NULL) {
	/* first time here, walk down the tree looking for the right
           section */
	p = va_arg(args, const char *);
	if (p == NULL)
	    return NULL;
	return vget_next(context, c, pointer, type, p, args);
    }

    /* we were called again, so just look for more entries with the
       same name and type */
    for (b = (*pointer)->next; b != NULL; b = b->next) {
	if(strcmp(b->name, (*pointer)->name) == 0 && b->type == type) {
	    *pointer = b;
	    return b->u.generic;
	}
    }
    return NULL;
}

const void *
krb5_config_get (krb5_context context,
		 const krb5_config_section *c,
		 int type,
		 ...)
{
    const void *ret;
    va_list args;

    va_start(args, type);
    ret = krb5_config_vget (context, c, type, args);
    va_end(args);
    return ret;
}

const void *
krb5_config_vget (krb5_context context,
		  const krb5_config_section *c,
		  int type,
		  va_list args)
{
    const krb5_config_binding *foo = NULL;

    return krb5_config_vget_next (context, c, &foo, type, args);
}

const krb5_config_binding *
krb5_config_get_list (krb5_context context,
		      const krb5_config_section *c,
		      ...)
{
    const krb5_config_binding *ret;
    va_list args;

    va_start(args, c);
    ret = krb5_config_vget_list (context, c, args);
    va_end(args);
    return ret;
}

const krb5_config_binding *
krb5_config_vget_list (krb5_context context,
		       const krb5_config_section *c,
		       va_list args)
{
    return krb5_config_vget (context, c, krb5_config_list, args);
}

const char* KRB5_LIB_FUNCTION
krb5_config_get_string (krb5_context context,
			const krb5_config_section *c,
			...)
{
    const char *ret;
    va_list args;

    va_start(args, c);
    ret = krb5_config_vget_string (context, c, args);
    va_end(args);
    return ret;
}

const char* KRB5_LIB_FUNCTION
krb5_config_vget_string (krb5_context context,
			 const krb5_config_section *c,
			 va_list args)
{
    return krb5_config_vget (context, c, krb5_config_string, args);
}

const char* KRB5_LIB_FUNCTION
krb5_config_vget_string_default (krb5_context context,
				 const krb5_config_section *c,
				 const char *def_value,
				 va_list args)
{
    const char *ret;

    ret = krb5_config_vget_string (context, c, args);
    if (ret == NULL)
	ret = def_value;
    return ret;
}

const char* KRB5_LIB_FUNCTION
krb5_config_get_string_default (krb5_context context,
				const krb5_config_section *c,
				const char *def_value,
				...)
{
    const char *ret;
    va_list args;

    va_start(args, def_value);
    ret = krb5_config_vget_string_default (context, c, def_value, args);
    va_end(args);
    return ret;
}

char ** KRB5_LIB_FUNCTION
krb5_config_vget_strings(krb5_context context,
			 const krb5_config_section *c,
			 va_list args)
{
    char **strings = NULL;
    int nstr = 0;
    const krb5_config_binding *b = NULL;
    const char *p;

    while((p = krb5_config_vget_next(context, c, &b,
				     krb5_config_string, args))) {
	char *tmp = strdup(p);
	char *pos = NULL;
	char *s;
	if(tmp == NULL)
	    goto cleanup;
	s = strtok_r(tmp, " \t", &pos);
	while(s){
	    char **tmp2 = realloc(strings, (nstr + 1) * sizeof(*strings));
	    if(tmp2 == NULL)
		goto cleanup;
	    strings = tmp2;
	    strings[nstr] = strdup(s);
	    nstr++;
	    if(strings[nstr-1] == NULL)
		goto cleanup;
	    s = strtok_r(NULL, " \t", &pos);
	}
	free(tmp);
    }
    if(nstr){
	char **tmp = realloc(strings, (nstr + 1) * sizeof(*strings));
	if(tmp == NULL)
	    goto cleanup;
	strings = tmp;
	strings[nstr] = NULL;
    }
    return strings;
cleanup:
    while(nstr--)
	free(strings[nstr]);
    free(strings);
    return NULL;

}

char**
krb5_config_get_strings(krb5_context context,
			const krb5_config_section *c,
			...)
{
    va_list ap;
    char **ret;
    va_start(ap, c);
    ret = krb5_config_vget_strings(context, c, ap);
    va_end(ap);
    return ret;
}

void KRB5_LIB_FUNCTION
krb5_config_free_strings(char **strings)
{
    char **s = strings;
    while(s && *s){
	free(*s);
	s++;
    }
    free(strings);
}

krb5_boolean KRB5_LIB_FUNCTION
krb5_config_vget_bool_default (krb5_context context,
			       const krb5_config_section *c,
			       krb5_boolean def_value,
			       va_list args)
{
    const char *str;
    str = krb5_config_vget_string (context, c, args);
    if(str == NULL)
	return def_value;
    if(strcasecmp(str, "yes") == 0 ||
       strcasecmp(str, "true") == 0 ||
       atoi(str)) return TRUE;
    return FALSE;
}

krb5_boolean KRB5_LIB_FUNCTION
krb5_config_vget_bool  (krb5_context context,
			const krb5_config_section *c,
			va_list args)
{
    return krb5_config_vget_bool_default (context, c, FALSE, args);
}

krb5_boolean KRB5_LIB_FUNCTION
krb5_config_get_bool_default (krb5_context context,
			      const krb5_config_section *c,
			      krb5_boolean def_value,
			      ...)
{
    va_list ap;
    krb5_boolean ret;
    va_start(ap, def_value);
    ret = krb5_config_vget_bool_default(context, c, def_value, ap);
    va_end(ap);
    return ret;
}

krb5_boolean KRB5_LIB_FUNCTION
krb5_config_get_bool (krb5_context context,
		      const krb5_config_section *c,
		      ...)
{
    va_list ap;
    krb5_boolean ret;
    va_start(ap, c);
    ret = krb5_config_vget_bool (context, c, ap);
    va_end(ap);
    return ret;
}

int KRB5_LIB_FUNCTION
krb5_config_vget_time_default (krb5_context context,
			       const krb5_config_section *c,
			       int def_value,
			       va_list args)
{
    const char *str;
    krb5_deltat t;

    str = krb5_config_vget_string (context, c, args);
    if(str == NULL)
	return def_value;
    if (krb5_string_to_deltat(str, &t))
	return def_value;
    return t;
}

int KRB5_LIB_FUNCTION
krb5_config_vget_time  (krb5_context context,
			const krb5_config_section *c,
			va_list args)
{
    return krb5_config_vget_time_default (context, c, -1, args);
}

int KRB5_LIB_FUNCTION
krb5_config_get_time_default (krb5_context context,
			      const krb5_config_section *c,
			      int def_value,
			      ...)
{
    va_list ap;
    int ret;
    va_start(ap, def_value);
    ret = krb5_config_vget_time_default(context, c, def_value, ap);
    va_end(ap);
    return ret;
}

int KRB5_LIB_FUNCTION
krb5_config_get_time (krb5_context context,
		      const krb5_config_section *c,
		      ...)
{
    va_list ap;
    int ret;
    va_start(ap, c);
    ret = krb5_config_vget_time (context, c, ap);
    va_end(ap);
    return ret;
}


int KRB5_LIB_FUNCTION
krb5_config_vget_int_default (krb5_context context,
			      const krb5_config_section *c,
			      int def_value,
			      va_list args)
{
    const char *str;
    str = krb5_config_vget_string (context, c, args);
    if(str == NULL)
	return def_value;
    else {
	char *endptr;
	long l;
	l = strtol(str, &endptr, 0);
	if (endptr == str)
	    return def_value;
	else
	    return l;
    }
}

int KRB5_LIB_FUNCTION
krb5_config_vget_int  (krb5_context context,
		       const krb5_config_section *c,
		       va_list args)
{
    return krb5_config_vget_int_default (context, c, -1, args);
}

int KRB5_LIB_FUNCTION
krb5_config_get_int_default (krb5_context context,
			     const krb5_config_section *c,
			     int def_value,
			     ...)
{
    va_list ap;
    int ret;
    va_start(ap, def_value);
    ret = krb5_config_vget_int_default(context, c, def_value, ap);
    va_end(ap);
    return ret;
}

int KRB5_LIB_FUNCTION
krb5_config_get_int (krb5_context context,
		     const krb5_config_section *c,
		     ...)
{
    va_list ap;
    int ret;
    va_start(ap, c);
    ret = krb5_config_vget_int (context, c, ap);
    va_end(ap);
    return ret;
}
