/*
 * Copyright (c) 1997 - 2005 Kungliga Tekniska Högskolan
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

#include "kdc_locl.h"

#ifdef KRB4

#include <krb5-v4compat.h>
#include <rx.h>

#define KA_AUTHENTICATION_SERVICE 731
#define KA_TICKET_GRANTING_SERVICE 732
#define KA_MAINTENANCE_SERVICE 733

#define AUTHENTICATE_OLD	 1
#define CHANGEPASSWORD		 2
#define GETTICKET_OLD		 3
#define SETPASSWORD		 4
#define SETFIELDS		 5
#define CREATEUSER		 6
#define DELETEUSER		 7
#define GETENTRY		 8
#define LISTENTRY		 9
#define GETSTATS		10
#define DEBUG			11
#define GETPASSWORD		12
#define GETRANDOMKEY		13
#define AUTHENTICATE		21
#define AUTHENTICATE_V2		22
#define GETTICKET		23

/* XXX - Where do we get these? */

#define RXGEN_OPCODE (-455)

#define KADATABASEINCONSISTENT                   (180480L)
#define KAEXIST                                  (180481L)
#define KAIO                                     (180482L)
#define KACREATEFAIL                             (180483L)
#define KANOENT                                  (180484L)
#define KAEMPTY                                  (180485L)
#define KABADNAME                                (180486L)
#define KABADINDEX                               (180487L)
#define KANOAUTH                                 (180488L)
#define KAANSWERTOOLONG                          (180489L)
#define KABADREQUEST                             (180490L)
#define KAOLDINTERFACE                           (180491L)
#define KABADARGUMENT                            (180492L)
#define KABADCMD                                 (180493L)
#define KANOKEYS                                 (180494L)
#define KAREADPW                                 (180495L)
#define KABADKEY                                 (180496L)
#define KAUBIKINIT                               (180497L)
#define KAUBIKCALL                               (180498L)
#define KABADPROTOCOL                            (180499L)
#define KANOCELLS                                (180500L)
#define KANOCELL                                 (180501L)
#define KATOOMANYUBIKS                           (180502L)
#define KATOOMANYKEYS                            (180503L)
#define KABADTICKET                              (180504L)
#define KAUNKNOWNKEY                             (180505L)
#define KAKEYCACHEINVALID                        (180506L)
#define KABADSERVER                              (180507L)
#define KABADUSER                                (180508L)
#define KABADCPW                                 (180509L)
#define KABADCREATE                              (180510L)
#define KANOTICKET                               (180511L)
#define KAASSOCUSER                              (180512L)
#define KANOTSPECIAL                             (180513L)
#define KACLOCKSKEW                              (180514L)
#define KANORECURSE                              (180515L)
#define KARXFAIL                                 (180516L)
#define KANULLPASSWORD                           (180517L)
#define KAINTERNALERROR                          (180518L)
#define KAPWEXPIRED                              (180519L)
#define KAREUSED                                 (180520L)
#define KATOOSOON                                (180521L)
#define KALOCKED                                 (180522L)


static krb5_error_code
decode_rx_header (krb5_storage *sp,
		  struct rx_header *h)
{
    krb5_error_code ret;

    ret = krb5_ret_uint32(sp, &h->epoch);
    if (ret) return ret;
    ret = krb5_ret_uint32(sp, &h->connid);
    if (ret) return ret;
    ret = krb5_ret_uint32(sp, &h->callid);
    if (ret) return ret;
    ret = krb5_ret_uint32(sp, &h->seqno);
    if (ret) return ret;
    ret = krb5_ret_uint32(sp, &h->serialno);
    if (ret) return ret;
    ret = krb5_ret_uint8(sp,  &h->type);
    if (ret) return ret;
    ret = krb5_ret_uint8(sp,  &h->flags);
    if (ret) return ret;
    ret = krb5_ret_uint8(sp,  &h->status);
    if (ret) return ret;
    ret = krb5_ret_uint8(sp,  &h->secindex);
    if (ret) return ret;
    ret = krb5_ret_uint16(sp, &h->reserved);
    if (ret) return ret;
    ret = krb5_ret_uint16(sp, &h->serviceid);
    if (ret) return ret;

    return 0;
}

static krb5_error_code
encode_rx_header (struct rx_header *h,
		  krb5_storage *sp)
{
    krb5_error_code ret;

    ret = krb5_store_uint32(sp, h->epoch);
    if (ret) return ret;
    ret = krb5_store_uint32(sp, h->connid);
    if (ret) return ret;
    ret = krb5_store_uint32(sp, h->callid);
    if (ret) return ret;
    ret = krb5_store_uint32(sp, h->seqno);
    if (ret) return ret;
    ret = krb5_store_uint32(sp, h->serialno);
    if (ret) return ret;
    ret = krb5_store_uint8(sp,  h->type);
    if (ret) return ret;
    ret = krb5_store_uint8(sp,  h->flags);
    if (ret) return ret;
    ret = krb5_store_uint8(sp,  h->status);
    if (ret) return ret;
    ret = krb5_store_uint8(sp,  h->secindex);
    if (ret) return ret;
    ret = krb5_store_uint16(sp, h->reserved);
    if (ret) return ret;
    ret = krb5_store_uint16(sp, h->serviceid);
    if (ret) return ret;

    return 0;
}

static void
init_reply_header (struct rx_header *hdr,
		   struct rx_header *reply_hdr,
		   u_char type,
		   u_char flags)
{
    reply_hdr->epoch     = hdr->epoch;
    reply_hdr->connid    = hdr->connid;
    reply_hdr->callid    = hdr->callid;
    reply_hdr->seqno     = 1;
    reply_hdr->serialno  = 1;
    reply_hdr->type      = type;
    reply_hdr->flags     = flags;
    reply_hdr->status    = 0;
    reply_hdr->secindex  = 0;
    reply_hdr->reserved  = 0;
    reply_hdr->serviceid = hdr->serviceid;
}

/*
 * Create an error `reply´ using for the packet `hdr' with the error
 * `error´ code.
 */
static void
make_error_reply (struct rx_header *hdr,
		  uint32_t error,
		  krb5_data *reply)

{
    struct rx_header reply_hdr;
    krb5_error_code ret;
    krb5_storage *sp;

    init_reply_header (hdr, &reply_hdr, HT_ABORT, HF_LAST);
    sp = krb5_storage_emem();
    if (sp == NULL)
	return;
    ret = encode_rx_header (&reply_hdr, sp);
    if (ret)
	return;
    krb5_store_int32(sp, error);
    krb5_storage_to_data (sp, reply);
    krb5_storage_free (sp);
}

static krb5_error_code
krb5_ret_xdr_data(krb5_storage *sp,
		  krb5_data *data)
{
    int ret;
    int size;
    ret = krb5_ret_int32(sp, &size);
    if(ret)
	return ret;
    if(size < 0)
	return ERANGE;
    data->length = size;
    if (size) {
	u_char foo[4];
	size_t pad = (4 - size % 4) % 4;

	data->data = malloc(size);
	if (data->data == NULL)
	    return ENOMEM;
	ret = krb5_storage_read(sp, data->data, size);
	if(ret != size)
	    return (ret < 0)? errno : KRB5_CC_END;
	if (pad) {
	    ret = krb5_storage_read(sp, foo, pad);
	    if (ret != pad)
		return (ret < 0)? errno : KRB5_CC_END;
	}
    } else
	data->data = NULL;
    return 0;
}

static krb5_error_code
krb5_store_xdr_data(krb5_storage *sp,
		    krb5_data data)
{
    u_char zero[4] = {0, 0, 0, 0};
    int ret;
    size_t pad;

    ret = krb5_store_int32(sp, data.length);
    if(ret < 0)
	return ret;
    ret = krb5_storage_write(sp, data.data, data.length);
    if(ret != data.length){
	if(ret < 0)
	    return errno;
	return KRB5_CC_END;
    }
    pad = (4 - data.length % 4) % 4;
    if (pad) {
	ret = krb5_storage_write(sp, zero, pad);
	if (ret != pad) {
	    if (ret < 0)
		return errno;
	    return KRB5_CC_END;
	}
    }
    return 0;
}


static krb5_error_code
create_reply_ticket (krb5_context context,
		     struct rx_header *hdr,
		     Key *skey,
		     char *name, char *instance, char *realm,
		     struct sockaddr_in *addr,
		     int life,
		     int kvno,
		     int32_t max_seq_len,
		     const char *sname, const char *sinstance,
		     uint32_t challenge,
		     const char *label,
		     krb5_keyblock *key,
		     krb5_data *reply)
{
    krb5_error_code ret;
    krb5_data ticket;
    krb5_keyblock session;
    krb5_storage *sp;
    krb5_data enc_data;
    struct rx_header reply_hdr;
    char zero[8];
    size_t pad;
    unsigned fyrtiosjuelva;

    /* create the ticket */

    krb5_generate_random_keyblock(context, ETYPE_DES_PCBC_NONE, &session);

    _krb5_krb_create_ticket(context,
			    0,
			    name,
			    instance,
			    realm,
			    addr->sin_addr.s_addr,
			    &session,
			    life,
			    kdc_time,
			    sname,
			    sinstance,
			    &skey->key,
			    &ticket);

    /* create the encrypted part of the reply */
    sp = krb5_storage_emem ();
    krb5_generate_random_block(&fyrtiosjuelva, sizeof(fyrtiosjuelva));
    fyrtiosjuelva &= 0xffffffff;
    krb5_store_int32 (sp, fyrtiosjuelva);
    krb5_store_int32 (sp, challenge);
    krb5_storage_write  (sp, session.keyvalue.data, 8);
    krb5_free_keyblock_contents(context, &session);
    krb5_store_int32 (sp, kdc_time);
    krb5_store_int32 (sp, kdc_time + _krb5_krb_life_to_time (0, life));
    krb5_store_int32 (sp, kvno);
    krb5_store_int32 (sp, ticket.length);
    krb5_store_stringz (sp, name);
    krb5_store_stringz (sp, instance);
#if 1 /* XXX - Why shouldn't the realm go here? */
    krb5_store_stringz (sp, "");
#else
    krb5_store_stringz (sp, realm);
#endif
    krb5_store_stringz (sp, sname);
    krb5_store_stringz (sp, sinstance);
    krb5_storage_write (sp, ticket.data, ticket.length);
    krb5_storage_write (sp, label, strlen(label));

    /* pad to DES block */
    memset (zero, 0, sizeof(zero));
    pad = (8 - krb5_storage_seek (sp, 0, SEEK_CUR) % 8) % 8;
    krb5_storage_write (sp, zero, pad);

    krb5_storage_to_data (sp, &enc_data);
    krb5_storage_free (sp);

    if (enc_data.length > max_seq_len) {
	krb5_data_free (&enc_data);
	make_error_reply (hdr, KAANSWERTOOLONG, reply);
	return 0;
    }

    /* encrypt it */
    {
        DES_key_schedule schedule;
	DES_cblock deskey;
	
	memcpy (&deskey, key->keyvalue.data, sizeof(deskey));
	DES_set_key_unchecked (&deskey, &schedule);
	DES_pcbc_encrypt (enc_data.data,
			  enc_data.data,
			  enc_data.length,
			  &schedule,
			  &deskey,
			  DES_ENCRYPT);
	memset (&schedule, 0, sizeof(schedule));
	memset (&deskey, 0, sizeof(deskey));
    }

    /* create the reply packet */
    init_reply_header (hdr, &reply_hdr, HT_DATA, HF_LAST);
    sp = krb5_storage_emem ();
    ret = encode_rx_header (&reply_hdr, sp);
    krb5_store_int32 (sp, max_seq_len);
    krb5_store_xdr_data (sp, enc_data);
    krb5_data_free (&enc_data);
    krb5_storage_to_data (sp, reply);
    krb5_storage_free (sp);
    return 0;
}

static krb5_error_code
unparse_auth_args (krb5_storage *sp,
		   char **name,
		   char **instance,
		   time_t *start_time,
		   time_t *end_time,
		   krb5_data *request,
		   int32_t *max_seq_len)
{
    krb5_data data;
    int32_t tmp;

    krb5_ret_xdr_data (sp, &data);
    *name = malloc(data.length + 1);
    if (*name == NULL)
	return ENOMEM;
    memcpy (*name, data.data, data.length);
    (*name)[data.length] = '\0';
    krb5_data_free (&data);

    krb5_ret_xdr_data (sp, &data);
    *instance = malloc(data.length + 1);
    if (*instance == NULL) {
	free (*name);
	return ENOMEM;
    }
    memcpy (*instance, data.data, data.length);
    (*instance)[data.length] = '\0';
    krb5_data_free (&data);

    krb5_ret_int32 (sp, &tmp);
    *start_time = tmp;
    krb5_ret_int32 (sp, &tmp);
    *end_time = tmp;
    krb5_ret_xdr_data (sp, request);
    krb5_ret_int32 (sp, max_seq_len);
    /* ignore the rest */
    return 0;
}

static void
do_authenticate (krb5_context context,
		 krb5_kdc_configuration *config,
		 struct rx_header *hdr,
		 krb5_storage *sp,
		 struct sockaddr_in *addr,
		 const char *from,
		 krb5_data *reply)
{
    krb5_error_code ret;
    char *name = NULL;
    char *instance = NULL;
    time_t start_time;
    time_t end_time;
    krb5_data request;
    int32_t max_seq_len;
    hdb_entry_ex *client_entry = NULL;
    hdb_entry_ex *server_entry = NULL;
    Key *ckey = NULL;
    Key *skey = NULL;
    krb5_storage *reply_sp;
    time_t max_life;
    uint8_t life;
    int32_t chal;
    char client_name[256];
    char server_name[256];
	
    krb5_data_zero (&request);

    ret = unparse_auth_args (sp, &name, &instance, &start_time, &end_time,
			     &request, &max_seq_len);
    if (ret != 0 || request.length < 8) {
	make_error_reply (hdr, KABADREQUEST, reply);
	goto out;
    }

    snprintf (client_name, sizeof(client_name), "%s.%s@%s",
	      name, instance, config->v4_realm);
    snprintf (server_name, sizeof(server_name), "%s.%s@%s",
	      "krbtgt", config->v4_realm, config->v4_realm);

    kdc_log(context, config, 0, "AS-REQ (kaserver) %s from %s for %s",
	    client_name, from, server_name);

    ret = _kdc_db_fetch4 (context, config, name, instance,
			  config->v4_realm, HDB_F_GET_CLIENT,
			  &client_entry);
    if (ret) {
	kdc_log(context, config, 0, "Client not found in database: %s: %s",
		client_name, krb5_get_err_text(context, ret));
	make_error_reply (hdr, KANOENT, reply);
	goto out;
    }

    ret = _kdc_db_fetch4 (context, config, "krbtgt",
			  config->v4_realm, config->v4_realm,
			  HDB_F_GET_KRBTGT, &server_entry);
    if (ret) {
	kdc_log(context, config, 0, "Server not found in database: %s: %s",
		server_name, krb5_get_err_text(context, ret));
	make_error_reply (hdr, KANOENT, reply);
	goto out;
    }

    ret = kdc_check_flags (context, config,
			   client_entry, client_name,
			   server_entry, server_name,
			   TRUE);
    if (ret) {
	make_error_reply (hdr, KAPWEXPIRED, reply);
	goto out;
    }

    /* find a DES key */
    ret = _kdc_get_des_key(context, client_entry, FALSE, TRUE, &ckey);
    if(ret){
	kdc_log(context, config, 0, "no suitable DES key for client");
	make_error_reply (hdr, KANOKEYS, reply);
	goto out;
    }

    /* find a DES key */
    ret = _kdc_get_des_key(context, server_entry, TRUE, TRUE, &skey);
    if(ret){
	kdc_log(context, config, 0, "no suitable DES key for server");
	make_error_reply (hdr, KANOKEYS, reply);
	goto out;
    }

    {
	DES_cblock key;
	DES_key_schedule schedule;
	
	/* try to decode the `request' */
	memcpy (&key, ckey->key.keyvalue.data, sizeof(key));
	DES_set_key_unchecked (&key, &schedule);
	DES_pcbc_encrypt (request.data,
			  request.data,
			  request.length,
			  &schedule,
			  &key,
			  DES_DECRYPT);
	memset (&schedule, 0, sizeof(schedule));
	memset (&key, 0, sizeof(key));
    }

    /* check for the magic label */
    if (memcmp ((char *)request.data + 4, "gTGS", 4) != 0) {
	kdc_log(context, config, 0, "preauth failed for %s", client_name);
	make_error_reply (hdr, KABADREQUEST, reply);
	goto out;
    }

    reply_sp = krb5_storage_from_mem (request.data, 4);
    krb5_ret_int32 (reply_sp, &chal);
    krb5_storage_free (reply_sp);

    if (abs(chal - kdc_time) > context->max_skew) {
	make_error_reply (hdr, KACLOCKSKEW, reply);
	goto out;
    }

    /* life */
    max_life = end_time - kdc_time;
    /* end_time - kdc_time can sometimes be non-positive due to slight
       time skew between client and server. Let's make sure it is postive */
    if(max_life < 1)
	max_life = 1;
    if (client_entry->entry.max_life)
	max_life = min(max_life, *client_entry->entry.max_life);
    if (server_entry->entry.max_life)
	max_life = min(max_life, *server_entry->entry.max_life);

    life = krb_time_to_life(kdc_time, kdc_time + max_life);

    create_reply_ticket (context,
			 hdr, skey,
			 name, instance, config->v4_realm,
			 addr, life, server_entry->entry.kvno,
			 max_seq_len,
			 "krbtgt", config->v4_realm,
			 chal + 1, "tgsT",
			 &ckey->key, reply);

 out:
    if (request.length) {
	memset (request.data, 0, request.length);
	krb5_data_free (&request);
    }
    if (name)
	free (name);
    if (instance)
	free (instance);
    if (client_entry)
	_kdc_free_ent (context, client_entry);
    if (server_entry)
	_kdc_free_ent (context, server_entry);
}

static krb5_error_code
unparse_getticket_args (krb5_storage *sp,
			int *kvno,
			char **auth_domain,
			krb5_data *ticket,
			char **name,
			char **instance,
			krb5_data *times,
			int32_t *max_seq_len)
{
    krb5_data data;
    int32_t tmp;

    krb5_ret_int32 (sp, &tmp);
    *kvno = tmp;

    krb5_ret_xdr_data (sp, &data);
    *auth_domain = malloc(data.length + 1);
    if (*auth_domain == NULL)
	return ENOMEM;
    memcpy (*auth_domain, data.data, data.length);
    (*auth_domain)[data.length] = '\0';
    krb5_data_free (&data);

    krb5_ret_xdr_data (sp, ticket);

    krb5_ret_xdr_data (sp, &data);
    *name = malloc(data.length + 1);
    if (*name == NULL) {
	free (*auth_domain);
	return ENOMEM;
    }
    memcpy (*name, data.data, data.length);
    (*name)[data.length] = '\0';
    krb5_data_free (&data);

    krb5_ret_xdr_data (sp, &data);
    *instance = malloc(data.length + 1);
    if (*instance == NULL) {
	free (*auth_domain);
	free (*name);
	return ENOMEM;
    }
    memcpy (*instance, data.data, data.length);
    (*instance)[data.length] = '\0';
    krb5_data_free (&data);

    krb5_ret_xdr_data (sp, times);

    krb5_ret_int32 (sp, max_seq_len);
    /* ignore the rest */
    return 0;
}

static void
do_getticket (krb5_context context,
	      krb5_kdc_configuration *config,
	      struct rx_header *hdr,
	      krb5_storage *sp,
	      struct sockaddr_in *addr,
	      const char *from,
	      krb5_data *reply)
{
    krb5_error_code ret;
    int kvno;
    char *auth_domain = NULL;
    krb5_data aticket;
    char *name = NULL;
    char *instance = NULL;
    krb5_data times;
    int32_t max_seq_len;
    hdb_entry_ex *server_entry = NULL;
    hdb_entry_ex *client_entry = NULL;
    hdb_entry_ex *krbtgt_entry = NULL;
    Key *kkey = NULL;
    Key *skey = NULL;
    DES_cblock key;
    DES_key_schedule schedule;
    DES_cblock session;
    time_t max_life;
    int8_t life;
    time_t start_time, end_time;
    char server_name[256];
    char client_name[256];
    struct _krb5_krb_auth_data ad;

    krb5_data_zero (&aticket);
    krb5_data_zero (&times);

    memset(&ad, 0, sizeof(ad));

    unparse_getticket_args (sp, &kvno, &auth_domain, &aticket,
			    &name, &instance, &times, &max_seq_len);
    if (times.length < 8) {
	make_error_reply (hdr, KABADREQUEST, reply);
	goto out;
	
    }

    snprintf (server_name, sizeof(server_name),
	      "%s.%s@%s", name, instance, config->v4_realm);

    ret = _kdc_db_fetch4 (context, config, name, instance,
			  config->v4_realm, HDB_F_GET_SERVER, &server_entry);
    if (ret) {
	kdc_log(context, config, 0, "Server not found in database: %s: %s",
		server_name, krb5_get_err_text(context, ret));
	make_error_reply (hdr, KANOENT, reply);
	goto out;
    }

    ret = _kdc_db_fetch4 (context, config, "krbtgt",
		     config->v4_realm, config->v4_realm, HDB_F_GET_KRBTGT, &krbtgt_entry);
    if (ret) {
	kdc_log(context, config, 0,
		"Server not found in database: %s.%s@%s: %s",
		"krbtgt", config->v4_realm,  config->v4_realm,
		krb5_get_err_text(context, ret));
	make_error_reply (hdr, KANOENT, reply);
	goto out;
    }

    /* find a DES key */
    ret = _kdc_get_des_key(context, krbtgt_entry, TRUE, TRUE, &kkey);
    if(ret){
	kdc_log(context, config, 0, "no suitable DES key for krbtgt");
	make_error_reply (hdr, KANOKEYS, reply);
	goto out;
    }

    /* find a DES key */
    ret = _kdc_get_des_key(context, server_entry, TRUE, TRUE, &skey);
    if(ret){
	kdc_log(context, config, 0, "no suitable DES key for server");
	make_error_reply (hdr, KANOKEYS, reply);
	goto out;
    }

    /* decrypt the incoming ticket */
    memcpy (&key, kkey->key.keyvalue.data, sizeof(key));

    /* unpack the ticket */
    {
	char *sname = NULL;
	char *sinstance = NULL;

	ret = _krb5_krb_decomp_ticket(context, &aticket, &kkey->key,
				      config->v4_realm, &sname,
				      &sinstance, &ad);
	if (ret) {
	    kdc_log(context, config, 0,
		    "kaserver: decomp failed for %s.%s with %d",
		    sname, sinstance, ret);
	    make_error_reply (hdr, KABADTICKET, reply);
	    goto out;
	}

	if (strcmp (sname, "krbtgt") != 0
	    || strcmp (sinstance, config->v4_realm) != 0) {
	    kdc_log(context, config, 0, "no TGT: %s.%s for %s.%s@%s",
		    sname, sinstance,
		    ad.pname, ad.pinst, ad.prealm);
	    make_error_reply (hdr, KABADTICKET, reply);
	    free(sname);
	    free(sinstance);
	    goto out;
	}
	free(sname);
	free(sinstance);

	if (kdc_time > _krb5_krb_life_to_time(ad.time_sec, ad.life)) {
	    kdc_log(context, config, 0, "TGT expired: %s.%s@%s",
		    ad.pname, ad.pinst, ad.prealm);
	    make_error_reply (hdr, KABADTICKET, reply);
	    goto out;
	}
    }

    snprintf (client_name, sizeof(client_name),
	      "%s.%s@%s", ad.pname, ad.pinst, ad.prealm);

    kdc_log(context, config, 0, "TGS-REQ (kaserver) %s from %s for %s",
	    client_name, from, server_name);

    ret = _kdc_db_fetch4 (context, config,
			  ad.pname, ad.pinst, ad.prealm, HDB_F_GET_CLIENT,
			  &client_entry);
    if(ret && ret != HDB_ERR_NOENTRY) {
	kdc_log(context, config, 0,
		"Client not found in database: (krb4) %s: %s",
		client_name, krb5_get_err_text(context, ret));
	make_error_reply (hdr, KANOENT, reply);
	goto out;
    }
    if (client_entry == NULL && strcmp(ad.prealm, config->v4_realm) == 0) {
	kdc_log(context, config, 0,
		"Local client not found in database: (krb4) "
		"%s", client_name);
	make_error_reply (hdr, KANOENT, reply);
	goto out;
    }

    ret = kdc_check_flags (context, config,
			   client_entry, client_name,
			   server_entry, server_name,
			   FALSE);
    if (ret) {
	make_error_reply (hdr, KAPWEXPIRED, reply);
	goto out;
    }

    /* decrypt the times */
    memcpy(&session, ad.session.keyvalue.data, sizeof(session));
    DES_set_key_unchecked (&session, &schedule);
    DES_ecb_encrypt (times.data,
		     times.data,
		     &schedule,
		     DES_DECRYPT);
    memset (&schedule, 0, sizeof(schedule));
    memset (&session, 0, sizeof(session));

    /* and extract them */
    {
	krb5_storage *tsp;
	int32_t tmp;

	tsp = krb5_storage_from_mem (times.data, times.length);
	krb5_ret_int32 (tsp, &tmp);
	start_time = tmp;
	krb5_ret_int32 (tsp, &tmp);
	end_time = tmp;
	krb5_storage_free (tsp);
    }

    /* life */
    max_life = end_time - kdc_time;
    /* end_time - kdc_time can sometimes be non-positive due to slight
       time skew between client and server. Let's make sure it is postive */
    if(max_life < 1)
	max_life = 1;
    if (krbtgt_entry->entry.max_life)
	max_life = min(max_life, *krbtgt_entry->entry.max_life);
    if (server_entry->entry.max_life)
	max_life = min(max_life, *server_entry->entry.max_life);
    /* if this is a cross realm request, the client_entry will likely
       be NULL */
    if (client_entry && client_entry->entry.max_life)
	max_life = min(max_life, *client_entry->entry.max_life);

    life = _krb5_krb_time_to_life(kdc_time, kdc_time + max_life);

    create_reply_ticket (context,
			 hdr, skey,
			 ad.pname, ad.pinst, ad.prealm,
			 addr, life, server_entry->entry.kvno,
			 max_seq_len,
			 name, instance,
			 0, "gtkt",
			 &ad.session, reply);

 out:
    _krb5_krb_free_auth_data(context, &ad);
    if (aticket.length) {
	memset (aticket.data, 0, aticket.length);
	krb5_data_free (&aticket);
    }
    if (times.length) {
	memset (times.data, 0, times.length);
	krb5_data_free (&times);
    }
    if (auth_domain)
	free (auth_domain);
    if (name)
	free (name);
    if (instance)
	free (instance);
    if (krbtgt_entry)
	_kdc_free_ent (context, krbtgt_entry);
    if (server_entry)
	_kdc_free_ent (context, server_entry);
}

krb5_error_code
_kdc_do_kaserver(krb5_context context,
		 krb5_kdc_configuration *config,
		 unsigned char *buf,
		 size_t len,
		 krb5_data *reply,
		 const char *from,
		 struct sockaddr_in *addr)
{
    krb5_error_code ret = 0;
    struct rx_header hdr;
    uint32_t op;
    krb5_storage *sp;

    if (len < RX_HEADER_SIZE)
	return -1;
    sp = krb5_storage_from_mem (buf, len);

    ret = decode_rx_header (sp, &hdr);
    if (ret)
	goto out;
    buf += RX_HEADER_SIZE;
    len -= RX_HEADER_SIZE;

    switch (hdr.type) {
    case HT_DATA :
	break;
    case HT_ACK :
    case HT_BUSY :
    case HT_ABORT :
    case HT_ACKALL :
    case HT_CHAL :
    case HT_RESP :
    case HT_DEBUG :
    default:
	/* drop */
	goto out;
    }


    if (hdr.serviceid != KA_AUTHENTICATION_SERVICE
	&& hdr.serviceid != KA_TICKET_GRANTING_SERVICE) {
	ret = -1;
	goto out;
    }

    ret = krb5_ret_uint32(sp, &op);
    if (ret)
	goto out;
    switch (op) {
    case AUTHENTICATE :
    case AUTHENTICATE_V2 :
	do_authenticate (context, config, &hdr, sp, addr, from, reply);
	break;
    case GETTICKET :
	do_getticket (context, config, &hdr, sp, addr, from, reply);
	break;
    case AUTHENTICATE_OLD :
    case CHANGEPASSWORD :
    case GETTICKET_OLD :
    case SETPASSWORD :
    case SETFIELDS :
    case CREATEUSER :
    case DELETEUSER :
    case GETENTRY :
    case LISTENTRY :
    case GETSTATS :
    case DEBUG :
    case GETPASSWORD :
    case GETRANDOMKEY :
    default :
	make_error_reply (&hdr, RXGEN_OPCODE, reply);
	break;
    }

out:
    krb5_storage_free (sp);
    return ret;
}

#endif /* KRB4 */
