/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
/*
 * ASUS Home Gateway Reference Design
 * Web Page Configuration Support Routines
 *
 * Copyright 2004, ASUSTeK Inc.
 * All Rights Reserved.
 * 
 * THIS SOFTWARE IS OFFERED "AS IS", AND BROADCOM GRANTS NO WARRANTIES OF ANY
 * KIND, EXPRESS OR IMPLIED, BY STATUTE, COMMUNICATION OR OTHERWISE. BROADCOM
 * SPECIFICALLY DISCLAIMS ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A SPECIFIC PURPOSE OR NONINFRINGEMENT CONCERNING THIS SOFTWARE.
 */

#ifdef WEBS
#include <webs.h>
#include <uemf.h>
#include <ej.h>
#else /* !WEBS */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <unistd.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <assert.h>
#include <httpd.h>
#endif /* WEBS */
#include <typedefs.h>
#include <bcmnvram.h>
#include <bcmutils.h>
#include <shutils.h>
#include <alpine.h>
#include <iwlib.h>
//#include <stapriv.h>
#include <shared.h>
#include <sys/mman.h>
#ifndef O_BINARY
#define O_BINARY 	0
#endif
#ifndef MAP_FAILED
#define MAP_FAILED (-1)
#endif

#define wan_prefix(unit, prefix)	snprintf(prefix, sizeof(prefix), "wan%d_", unit)
//static char * rfctime(const time_t *timep);
//static char * reltime(unsigned int seconds);
void reltime(unsigned int seconds, char *buf);
static int wl_status(int eid, webs_t wp, int argc, char_t **argv, int unit);

#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/klog.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/sockios.h>
#include <net/if_arp.h>

#include <dirent.h>

const char *get_wifname(int band)
{
	if (band)
		return WIF_5G;
	else
		return WIF_2G;
}

typedef struct _WPS_CONFIGURED_VALUE {
	unsigned short 	Configured;	// 1:un-configured/2:configured
	char		BSSID[18];
	char 		SSID[32 + 1];
	char		AuthMode[16];	// Open System/Shared Key/WPA-Personal/WPA2-Personal/WPA-Enterprise/WPA2-Enterprise
	char 		Encryp[8];	// None/WEP/TKIP/AES
	char 		DefaultKeyIdx;
	char 		WPAKey[64 + 1];
} WPS_CONFIGURED_VALUE;

/* shared/sysdeps/api-qca.c */
extern u_int ieee80211_mhz2ieee(u_int freq);
extern int get_channel_list_via_driver(int unit, char *buffer, int len);
extern int get_channel_list_via_country(int unit, const char *country_code, char *buffer, int len);

#define WL_A		(1U << 0)
#define WL_B		(1U << 1)
#define WL_G		(1U << 2)
#define WL_N		(1U << 3)
#define WL_AC		(1U << 4)

static const struct mode_s {
	unsigned int mask;
	char *mode;
} mode_tbl[] = {
	{ WL_A,	"a" },
	{ WL_B,	"b" },
	{ WL_G, "g" },
	{ WL_N, "n" },
	{ WL_AC, "ac" },
	{ 0, NULL },
};

/**
 * Run "iwpriv XXX get_XXX" and return string behind colon.
 * Expected result is shown below:
 * ath1      get_mode:11ACVHT40
 *                    ^^^^^^^^^
 * @iface:	interface name
 * @cmd:	get cmd
 * @buf:	pointer to memory area which is used to keep result.
 * 		it is guarantee as valid/empty string, if it is valid pointer
 * @buf_len:	length of @buf
 * @return:
 * 	0:	success
 *     -1:	invalid parameter
 *     -2:	run iwpriv command fail
 *     -3:	read result strin fail
 */
static int __iwpriv_get(const char *iface, char *cmd, char *buf, unsigned int buf_len)
{
	int len;
	FILE *fp;
	char *p = NULL, iwpriv_cmd[64], tmp[128];

	if (!iface || *iface == '\0' || !cmd || *cmd == '\0' || !buf || buf_len <= 1)
		return -1;

	if (strncmp(cmd, "get_", 4) && strncmp(cmd, "g_", 2)) {
		dbg("%s: iface [%s] cmd [%s] may not be supported!\n",
			__func__, iface, cmd);
	}

	*buf = '\0';
	sprintf(iwpriv_cmd, "iwpriv %s %s", iface, cmd);
	if (!(fp = popen(iwpriv_cmd, "r")))
		return -2;

	len = fread(tmp, 1, sizeof(tmp), fp);
	pclose(fp);
	if (len < 1)
		return -3;

	tmp[len-1] = '\0';
	if (!(p = strchr(tmp, ':'))) {
		dbg("%s: parsing [%s] of cmd [%s] error!", __func__, tmp, cmd);
		return -4;
	}
	p++;
	chomp(p);
	strlcpy(buf, p, buf_len);

	return 0;
}

/**
 * Run "iwpriv XXX get_XXX" and return string behind colon.
 * Expected result is shown below:
 * ath1      get_mode:11ACVHT40
 *                    ^^^^^^^^^ result
 * @iface:	interface name
 * @cmd:	get cmd
 * @return:
 * 	NULL	invalid parameter or error.
 *  otherwise:	success
 */
static char *iwpriv_get(const char *iface, char *cmd)
{
	static char result[256];

	if (__iwpriv_get(iface, cmd, result, sizeof(result)))
		return NULL;

	return result;
}

static void getWPSConfig(int unit, WPS_CONFIGURED_VALUE *result)
{
	char buf[128];
	FILE *fp;

	memset(result, 0, sizeof(WPS_CONFIGURED_VALUE));

	snprintf(buf, sizeof(buf), "hostapd_cli -i%s get_config", get_wifname(unit));
	fp = popen(buf, "r");
	if (fp) {
		while (fgets(buf, sizeof(buf), fp) != NULL) {
			char *pt1, *pt2;

			chomp(buf);
			//BSSID
			if ((pt1 = strstr(buf, "bssid="))) {
				pt2 = pt1 + strlen("bssid=");
				strlcpy(result->BSSID, pt2, sizeof(result->BSSID));
			}
			//SSID
			if ((pt1 = strstr(buf, "ssid="))) {
				pt2 = pt1 + strlen("ssid=");
				strlcpy(result->SSID, pt2, sizeof(result->SSID));
			}
			//Configured
			else if ((pt1 = strstr(buf, "wps_state="))) {
				pt2 = pt1 + strlen("wps_state=");
				if (!strcmp(pt2, "configured") ||
				    (!strcmp(pt2, "disabled") && nvram_get_int("w_Setting"))
				   )
					result->Configured = 2;
				else
					result->Configured = 1;
			}
			//WPAKey
			else if ((pt1 = strstr(buf, "passphrase="))) {
				pt2 = pt1 + strlen("passphrase=");
				strlcpy(result->WPAKey, pt2, sizeof(result->WPAKey));
			}
			//AuthMode
			else if ((pt1 = strstr(buf, "key_mgmt="))) {
				pt2 = pt1 + strlen("key_mgmt=");
				strlcpy(result->AuthMode, pt2, sizeof(result->AuthMode));/* FIXME: NEED TRANSFORM CONTENT */
			}
			//Encryp
			else if ((pt1 = strstr(buf, "rsn_pairwise_cipher="))) {
				pt2 = pt1 + strlen("rsn_pairwise_cipher=");
				if (!strcmp(pt2, "NONE"))
					strlcpy(result->Encryp, "None", sizeof(result->Encryp));
				else if (!strncmp(pt2, "WEP", 3))
					strlcpy(result->Encryp, "WEP", sizeof(result->Encryp));
				else if (!strcmp(pt2, "TKIP"))
					strlcpy(result->Encryp, "TKIP", sizeof(result->Encryp));
				else if (!strncmp(pt2, "CCMP", 4))
					strlcpy(result->Encryp, "AES", sizeof(result->Encryp));
			}
		}
		pclose(fp);
	}
	//dbg("%s: SSID[%s], Configured[%d], WPAKey[%s], AuthMode[%s], Encryp[%s]\n", __FUNCTION__, result->SSID, result->Configured, result->WPAKey, result->AuthMode, result->Encryp);
}

char *get_staifname(int band)
{
	return (char*) ((!band)? STA_2G:STA_5G);
}

char *get_vphyifname(int band)
{
	return (char*) ((!band)? VPHY_2G:VPHY_5G);
}


static char *__get_wlifname(int band, int subunit, char *buf, int buf_len)
{
	if (!buf)
		return buf;

	if (!subunit)
		strlcpy(buf, (!band)? WIF_2G:WIF_5G, buf_len);
	else
		snprintf(buf, buf_len, "%s%02d", (!band)? WIF_2G:WIF_5G, subunit);

	return buf;
}

static int get_wlsubnet(int band, const char *ifname)
{
	int subnet, sidx;
	char buf[32];

	for (subnet = 0, sidx = 0; subnet < MAX_NO_MSSID; subnet++)
	{
		if(!nvram_match(wl_nvname("bss_enabled", band, subnet), "1")) {
			if (!subnet)
				sidx++;
			continue;
		}

		if(strcmp(ifname, __get_wlifname(band, sidx, buf, sizeof(buf))) == 0)
			return subnet;

		sidx++;
	}
	return -1;
}

char *getAPPhyModebyIface(const char *iface)
{
	static char result[sizeof("11b/g/nXXXXXX")] = "";
	const struct mode_s *q;
	char *mode, *puren, *p, *sep;
	unsigned int m = 0;
	int sta = 0;

	if (!iface)
		return "";
	mode = iwpriv_get(iface, "get_mode");
	if (!mode)
		return "";

	/* Ref to phymode_strings of qca-wifi driver. */
	if (!strcmp(mode, "11A") || !strcmp(mode, "TA"))
		m = WL_A;
	else if (!strcmp(mode, "11G") || !strcmp(mode, "TG"))
		m = WL_G | WL_B;
	else if (!strcmp(mode, "11B"))
		m = WL_B;
	else if (!strncmp(mode, "11NA", 4))
		m = WL_N | WL_A;
	else if (!strncmp(mode, "11NG", 4))
		m = WL_N | WL_G | WL_B;
	else if (!strncmp(mode, "11ACVHT", 7))
		m = WL_AC | WL_N | WL_A;
	else if (!strncmp(mode, "AUTO", 4)) {
		if (!strcmp(iface, get_staifname(0))) {
			sta = 1;
			m = WL_N | WL_G | WL_B;
		}
		else if (!strcmp(iface, get_staifname(1)) ||
			 !strcmp(iface, get_staifname(2))	/* FIXME: for 2-nd 5GHz */
			) {
			sta = 1;
			m = WL_AC | WL_N | WL_A;
		}
		else
			dbg("%s: Unknown interface [%s] in AUTO mode\n", __func__, iface);
	}
	else {
		dbg("%s: Unknown mode [%s]\n", __func__, mode);
	}

	/* If puren is enabled, remove a/g/b. */
	puren = iwpriv_get(iface, "get_puren");
	if (!sta && atoi(puren))
		m &= ~(WL_A | WL_B | WL_G);

	p = result;
	*p = '\0';
	sep = "11";
	for (q = &mode_tbl[0]; m > 0 && q->mask; ++q) {
		if (!(m & q->mask))
			continue;

		m &= ~q->mask;
		strcat(p, sep);
		p += strlen(sep);
		strcat(p, q->mode);
		p += strlen(q->mode);
		sep = "/";
	}

	return result;
}

char *getAPPhyMode(int unit)
{
	return getAPPhyModebyIface(get_wifname(unit));
}

unsigned int getAPChannelbyIface(const char *ifname)
{
	char buf[8192];
	FILE *fp;
	int len, i = 0;
	char *pt1, *pt2, ch_mhz[5] = {0}, ch_mhz_t[5] = {0};

	if (!ifname || *ifname == '\0') {
		dbg("%S: got invalid ifname %p\n", __func__, ifname);
		return 0;
	}

	snprintf(buf, sizeof(buf), "iwconfig %s", ifname);
	fp = popen(buf, "r");
	if (fp) {
		memset(buf, 0, sizeof(buf));
		len = fread(buf, 1, sizeof(buf), fp);
		pclose(fp);
		if (len > 1) {
			buf[len-1] = '\0';
			pt1 = strstr(buf, "Frequency:");
			if (pt1) {
				pt2 = pt1 + strlen("Frequency:");
				pt1 = strstr(pt2, " GHz");
				if (pt1) {
					*pt1 = '\0';
					memset(ch_mhz, 0, sizeof(ch_mhz));
					len = strlen(pt2);
					for (i = 0; i < 5; i++) {
						if (i < len) {
							if (pt2[i] == '.')
								continue;

							snprintf(ch_mhz_t, sizeof(ch_mhz_t), "%s%c", ch_mhz, pt2[i]);
							strlcpy(ch_mhz, ch_mhz_t, sizeof(ch_mhz));
						}
						else{
							snprintf(ch_mhz_t, sizeof(ch_mhz_t), "%s0", ch_mhz);
							strlcpy(ch_mhz, ch_mhz_t, sizeof(ch_mhz));
						}
					}
					//dbg("Frequency:%s MHz\n", ch_mhz);
					return ieee80211_mhz2ieee((unsigned int)atoi(ch_mhz));
				}
			}
		}
	}
	return 0;
}

unsigned int getAPChannel(int unit)
{
	return getAPChannelbyIface(get_wifname(unit));
}

/**
 * Return SSID of a interface.
 * @return:	Don't return NULL even interface name is invalid or interface absent.
 */
char* getSSIDbyIFace(const char *ifname)
{
	static char ssid[33] = "";
	char buf[8192] = "";
	FILE *fp;
	int len;
	char *pt1, *pt2, *pt3;

	if (!ifname || *ifname == '\0') {
		dbg("%S: got invalid ifname %p\n", __func__, ifname);
		return ssid;
	}

	snprintf(buf, sizeof(buf), "iwconfig %s", ifname);
	if (!(fp = popen(buf, "r")))
		return ssid;

	len = fread(buf, 1, sizeof(buf), fp);
	pclose(fp);
	if (len <= 0)
		return ssid;

	buf[len] = '\0';
	pt1 = strstr(buf, "ESSID:");
	if (!pt1)
		return ssid;

	pt2 = pt1 + strlen("ESSID:") + 1;	/* skip leading " */
	pt1 = strchr(pt2, '\n');
	if (!pt1 || (pt1 - pt2) <= 1)
		return ssid;

	/* Remove trailing " */
	*pt1 = '\0';
	pt3 = strrchr(pt2, '"');
	if (pt3)
		*pt3 = '\0';

	strlcpy(ssid, pt2, sizeof(ssid));

	return ssid;
}

long getSTAConnTime(char *ifname, char *bssid)
{
	char buf[8192];
	FILE *fp;
	int len;
	char *pt1,*pt2;

	snprintf(buf, sizeof(buf), "hostapd_cli -i%s sta %s", ifname, bssid);
	fp = popen(buf, "r");
	if (fp) {
		memset(buf, 0, sizeof(buf));
		len = fread(buf, 1, sizeof(buf), fp);
		pclose(fp);
		if (len > 1) {
			buf[len-1] = '\0';
			pt1 = strstr(buf, "connected_time=");
			if (pt1) {
				pt2 = pt1 + strlen("connected_time=");
				chomp(pt2);
				return atol(pt2);
			}
		}
	}
	return 0;
}

typedef struct _WLANCONFIG_LIST {
	char addr[18];
	unsigned int aid;
	unsigned int chan;
	char txrate[7];
	char rxrate[10];
	unsigned int rssi;
	unsigned int idle;
	unsigned int txseq;
	unsigned int rxseq;
	char caps[12];
	char acaps[10];
	char erp[7];
	char state_maxrate[20];
	char wps[4];
	char conn_time[12];
	char rsn[4];
	char wme[4];
	char mode[31];
	char ie[32];
	char htcaps[10];
	unsigned int u_acaps;
	unsigned int u_erp;
	unsigned int u_state_maxrate;
	unsigned int u_psmode;
	int subunit;
} WLANCONFIG_LIST;

#if defined(RTCONFIG_WIFI_QCA9990_QCA9990) || defined(RTCONFIG_WIFI_QCA9994_QCA9994)
#define MAX_STA_NUM 512
#else
#define MAX_STA_NUM 256
#endif
typedef struct _WIFI_STA_TABLE {
	int Num;
	WLANCONFIG_LIST Entry[ MAX_STA_NUM ];
} WIFI_STA_TABLE;


static int getSTAInfo(int unit, WIFI_STA_TABLE *sta_info)
{
	#define STA_INFO_PATH "/tmp/wlanconfig_athX_list"
	FILE *fp;
	int ret = 0, l2_offset, subunit;
	char *unit_name;
	char *p, *ifname, *l2, *l3;
	char *wl_ifnames;
	char line_buf[300]; // max 14x

	memset(sta_info, 0, sizeof(*sta_info));
	unit_name = strdup(get_wifname(unit));
	if (!unit_name)
		return ret;
	wl_ifnames = strdup(nvram_safe_get("lan_ifnames"));
	if (!wl_ifnames) {
		free(unit_name);
		return ret;
	}
	p = wl_ifnames;
	while ((ifname = strsep(&p, " ")) != NULL) {
		while (*ifname == ' ') ++ifname;
		if (*ifname == 0) break;
		if(strncmp(ifname,unit_name,strlen(unit_name)))
			continue;

		subunit = get_wlsubnet(unit, ifname);
		if (subunit < 0)
			subunit = 0;

		doSystem("wlanconfig %s list > %s", ifname, STA_INFO_PATH);
		fp = fopen(STA_INFO_PATH, "r");
		if (fp) {
/* wlanconfig ath1 list
ADDR               AID CHAN TXRATE RXRATE RSSI IDLE  TXSEQ  RXSEQ  CAPS        ACAPS     ERP    STATE MAXRATE(DOT11) HTCAPS ASSOCTIME    IEs   MODE PSMODE
00:10:18:55:cc:08    1  149  55M   1299M   63    0      0   65535               0        807              0              Q 00:10:33 IEEE80211_MODE_11A  0
08:60:6e:8f:1e:e6    2  149 159M    866M   44    0      0   65535     E         0          b              0           WPSM 00:13:32 WME IEEE80211_MODE_11AC_VHT80  0
08:60:6e:8f:1e:e8    1  157 526M    526M   51 4320      0   65535    EP         0          b              0          AWPSM 00:00:10 RSN WME IEEE80211_MODE_11AC_VHT80 0
*/
			//fseek(fp, 131, SEEK_SET);	// ignore header
			fgets(line_buf, sizeof(line_buf), fp); // ignore header
			l2 = strstr(line_buf, "ACAPS");
			if (l2 != NULL)
				l2_offset = (int)(l2 - line_buf);
			else {
				l2_offset = 79;
				l2 = line_buf + l2_offset;
			}
			while ( fgets(line_buf, sizeof(line_buf), fp) ) {
				WLANCONFIG_LIST *r = &sta_info->Entry[sta_info->Num++];

				r->subunit = subunit;
				/* IEs may be empty string, find IEEE80211_MODE_ before parsing mode and psmode. */
				l3 = strstr(line_buf, "IEEE80211_MODE_");
				if (l3) {
					*(l3 - 1) = '\0';
					sscanf(l3, "IEEE80211_MODE_%s %d", r->mode, &r->u_psmode);
				}
				*(l2 - 1) = '\0';
				sscanf(line_buf, "%17s%u%u%9s%6s%u%u%u%u%[^\n]",
					r->addr, &r->aid, &r->chan, r->txrate,
					r->rxrate, &r->rssi, &r->idle, &r->txseq,
					&r->rxseq, r->caps);
				sscanf(l2, "%u%x%u%11s%31s%[^\n]",
					&r->u_acaps, &r->u_erp, &r->u_state_maxrate, r->htcaps, r->conn_time, r->ie);
				if (strlen(r->rxrate) >= 6)
					strlcpy(r->rxrate, "0M", sizeof(r->rxrate));
#if 0
				dbg("[%s][%u][%u][%s][%s][%u][%u][%u][%u][%s]"
					"[%u][%u][%x][%s][%s][%s][%d]\n",
					r->addr, r->aid, r->chan, r->txrate, r->rxrate,
					r->rssi, r->idle, r->txseq, r->rxseq, r->caps,
					r->u_acaps, r->u_erp, r->u_state_maxrate, r->htcaps, r->ie,
					r->mode, r->u_psmode);
#endif
			}

			fclose(fp);
			unlink(STA_INFO_PATH);
		}
	}
	free(wl_ifnames);
	free(unit_name);
	return ret;
}

char* GetBW(int BW)
{
	switch(BW)
	{
		case BW_10:
			return "10M";

		case BW_20:
			return "20M";

		case BW_40:
			return "40M";

#if defined(RTAC52U) || defined(RTAC51U)
		case BW_80:
			return "80M";
#endif

		default:
			return "N/A";
	}
}

char* GetPhyMode(int Mode)
{
	switch(Mode)
	{
		case MODE_CCK:
			return "CCK";

		case MODE_OFDM:
			return "OFDM";
		case MODE_HTMIX:
			return "HTMIX";

		case MODE_HTGREENFIELD:
			return "GREEN";

#if defined(RTAC52U) || defined(RTAC51U)
		case MODE_VHT:
			return "VHT";
#endif

		default:
			return "N/A";
	}
}

int MCSMappingRateTable[] =
	{2,  4,   11,  22, // CCK
	12, 18,   24,  36, 48, 72, 96, 108, // OFDM
	13, 26,   39,  52,  78, 104, 117, 130, 26,  52,  78, 104, 156, 208, 234, 260, // 20MHz, 800ns GI, MCS: 0 ~ 15
	39, 78,  117, 156, 234, 312, 351, 390,										  // 20MHz, 800ns GI, MCS: 16 ~ 23
	27, 54,   81, 108, 162, 216, 243, 270, 54, 108, 162, 216, 324, 432, 486, 540, // 40MHz, 800ns GI, MCS: 0 ~ 15
	81, 162, 243, 324, 486, 648, 729, 810,										  // 40MHz, 800ns GI, MCS: 16 ~ 23
	14, 29,   43,  57,  87, 115, 130, 144, 29, 59,   87, 115, 173, 230, 260, 288, // 20MHz, 400ns GI, MCS: 0 ~ 15
	43, 87,  130, 173, 260, 317, 390, 433,										  // 20MHz, 400ns GI, MCS: 16 ~ 23
	30, 60,   90, 120, 180, 240, 270, 300, 60, 120, 180, 240, 360, 480, 540, 600, // 40MHz, 400ns GI, MCS: 0 ~ 15
	90, 180, 270, 360, 540, 720, 810, 900,
	13, 26,   39,  52,  78, 104, 117, 130, 156, /* 11ac: 20Mhz, 800ns GI, MCS: 0~8 */
	27, 54,   81, 108, 162, 216, 243, 270, 324, 360, /*11ac: 40Mhz, 800ns GI, MCS: 0~9 */
	59, 117, 176, 234, 351, 468, 527, 585, 702, 780, /*11ac: 80Mhz, 800ns GI, MCS: 0~9 */
	14, 29,   43,  57,  87, 115, 130, 144, 173, /* 11ac: 20Mhz, 400ns GI, MCS: 0~8 */
	30, 60,   90, 120, 180, 240, 270, 300, 360, 400, /*11ac: 40Mhz, 400ns GI, MCS: 0~9 */
	65, 130, 195, 260, 390, 520, 585, 650, 780, 867 /*11ac: 80Mhz, 400ns GI, MCS: 0~9 */
	};


#define FN_GETRATE(_fn_, _st_)						\
_fn_(_st_ HTSetting)							\
{									\
	int rate_count = sizeof(MCSMappingRateTable)/sizeof(int);	\
	int rate_index = 0;						\
									\
	if (HTSetting.field.MODE >= MODE_VHT)				\
	{								\
		if (HTSetting.field.BW == BW_20) {			\
			rate_index = 108 +				\
			((unsigned char)HTSetting.field.ShortGI * 29) +	\
			((unsigned char)HTSetting.field.MCS);		\
		}							\
		else if (HTSetting.field.BW == BW_40) {			\
			rate_index = 117 +				\
			((unsigned char)HTSetting.field.ShortGI * 29) +	\
			((unsigned char)HTSetting.field.MCS);		\
		}							\
		else if (HTSetting.field.BW == BW_80) {			\
			rate_index = 127 +				\
			((unsigned char)HTSetting.field.ShortGI * 29) +	\
			((unsigned char)HTSetting.field.MCS);		\
		}							\
	}								\
	else								\
	if (HTSetting.field.MODE >= MODE_HTMIX)				\
	{								\
		rate_index = 12 + ((unsigned char)HTSetting.field.BW *24) + ((unsigned char)HTSetting.field.ShortGI *48) + ((unsigned char)HTSetting.field.MCS);	\
	}								\
	else								\
	if (HTSetting.field.MODE == MODE_OFDM)				\
		rate_index = (unsigned char)(HTSetting.field.MCS) + 4;	\
	else if (HTSetting.field.MODE == MODE_CCK)			\
		rate_index = (unsigned char)(HTSetting.field.MCS);	\
									\
	if (rate_index < 0)						\
		rate_index = 0;						\
									\
	if (rate_index >= rate_count)					\
		rate_index = rate_count-1;				\
									\
	return (MCSMappingRateTable[rate_index] * 5)/10;		\
}

int FN_GETRATE(getRate,      MACHTTRANSMIT_SETTING)		//getRate(MACHTTRANSMIT_SETTING)
int FN_GETRATE(getRate_2g,   MACHTTRANSMIT_SETTING_2G)		//getRate_2g(MACHTTRANSMIT_SETTING_2G)
#if defined(RTAC52U) || defined(RTAC51U)
int FN_GETRATE(getRate_11ac, MACHTTRANSMIT_SETTING_11AC)	//getRate_11ac(MACHTTRANSMIT_SETTING_11AC)
#endif



static int
show_wliface_info(webs_t wp, int unit, char *ifname, char *op_mode)
{
	int ret = 0;
	FILE *fp;
	unsigned char mac_addr[ETHER_ADDR_LEN];
	char tmpstr[1024], cmd[] = "iwconfig staXYYYYYY";
	char *p, ap_bssid[] = "00:00:00:00:00:00XXX";

	if (unit < 0 || !ifname || !op_mode)
		return 0;

	memset(&mac_addr, 0, sizeof(mac_addr));
	get_iface_hwaddr(ifname, mac_addr);
	ret += websWrite(wp, "=======================================================================================\n"); // separator
	ret += websWrite(wp, "OP Mode		: %s\n", op_mode);
	ret += websWrite(wp, "SSID		: %s\n", getSSIDbyIFace(ifname));
	snprintf(cmd, sizeof(cmd), "iwconfig %s", ifname);
	if ((fp = popen(cmd, "r")) != NULL && fread(tmpstr, 1, sizeof(tmpstr), fp) > 1) {
		pclose(fp);
		*(tmpstr + sizeof(tmpstr) - 1) = '\0';
		*ap_bssid = '\0';
		if ((p = strstr(tmpstr, "Access Point: ")) != NULL) {
			strncpy(ap_bssid, p + 14, 17);
			ap_bssid[17] = '\0';
		}
		ret += websWrite(wp, "BSSID		: %s\n", ap_bssid);
	}
	ret += websWrite(wp, "MAC address	: %02X:%02X:%02X:%02X:%02X:%02X\n",
		mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
	*tmpstr = '\0';
	strlcpy(tmpstr, getAPPhyModebyIface(ifname), sizeof(tmpstr));
	ret += websWrite(wp, "Phy Mode	: %s\n", tmpstr);
	ret += websWrite(wp, "Channel		: %u\n", getAPChannelbyIface(ifname));

	return ret;
}

static int
wl_status(int eid, webs_t wp, int argc, char_t **argv, int unit)
{
	int ret = 0, wl_mode_x, i;
	WIFI_STA_TABLE *sta_info;
	char tmp[128], prefix[] = "wlXXXXXXXXXX_", *ifname, *op_mode;
	char subunit_str[8];

#if defined(RTCONFIG_WIRELESSREPEATER) && defined(RTCONFIG_PROXYSTA)
	if (mediabridge_mode()) {
		/* Media bridge mode */
		snprintf(prefix, sizeof(prefix), "wl%d.1_", unit);
		ifname = nvram_safe_get(strcat_r(prefix, "ifname", tmp));
		if (unit != nvram_get_int("wlc_band")) {
			snprintf(prefix, sizeof(prefix), "wl%d_", unit);
			ret += websWrite(wp, "%s radio is disabled\n",
				nvram_match(strcat_r(prefix, "nband", tmp), "1") ? "5 GHz" : "2.4 GHz");
			return ret;
		}
		ret += show_wliface_info(wp, unit, ifname, "Media Bridge");
	} else {
#endif
		/* Router mode, Repeater and AP mode */
#if defined(RTCONFIG_WIRELESSREPEATER)
		if (!unit && repeater_mode()) {
			/* Show P-AP information first, if we are about to show 2.4G information in repeater mode. */
			snprintf(prefix, sizeof(prefix), "wl%d.1_", nvram_get_int("wlc_band"));
			ifname = nvram_safe_get(strcat_r(prefix, "ifname", tmp));
			ret += show_wliface_info(wp, unit, ifname, "Repeater");
			ret += websWrite(wp, "\n");
		}
#endif
		snprintf(prefix, sizeof(prefix), "wl%d_", unit);
		ifname = nvram_safe_get(strcat_r(prefix, "ifname", tmp));
		if (!get_radio_status(ifname)) {
#if defined(BAND_2G_ONLY)
			ret += websWrite(wp, "2.4 GHz radio is disabled\n");
#else
			ret += websWrite(wp, "%s radio is disabled\n",
				nvram_match(strcat_r(prefix, "nband", tmp), "1") ? "5 GHz" : "2.4 GHz");
#endif
			return ret;
		}

		wl_mode_x = nvram_get_int(strcat_r(prefix, "mode_x", tmp));
		op_mode = "AP";
		if (wl_mode_x == 1)
			op_mode = "WDS Only";
		else if (wl_mode_x == 2)
			op_mode = "Hybrid";
		ret += show_wliface_info(wp, unit, ifname, op_mode);
		ret += websWrite(wp, "\nStations List\n");
		ret += websWrite(wp, "----------------------------------------------------------------\n");
#if 0 //barton++
		ret += websWrite(wp, "%-18s%-4s%-8s%-4s%-4s%-4s%-5s%-5s%-12s\n",
				   "MAC", "PSM", "PhyMode", "BW", "MCS", "SGI", "STBC", "Rate", "Connect Time");
#else
		ret += websWrite(wp, "%-3s %-17s %-15s %-6s %-6s %-12s\n",
			"idx", "MAC", "PhyMode", "TXRATE", "RXRATE", "Connect Time");
#endif

		if ((sta_info = malloc(sizeof(*sta_info))) != NULL) {
			getSTAInfo(unit, sta_info);
			for(i = 0; i < sta_info->Num; i++) {
				*subunit_str = '\0';
				if (sta_info->Entry[i].subunit)
					snprintf(subunit_str, sizeof(subunit_str), "%d", sta_info->Entry[i].subunit);
				ret += websWrite(wp, "%3s %-17s %-15s %6s %6s %12s\n",
					subunit_str,
					sta_info->Entry[i].addr,
					sta_info->Entry[i].mode,
					sta_info->Entry[i].txrate,
					sta_info->Entry[i].rxrate,
					sta_info->Entry[i].conn_time
					);
			}
			free(sta_info);
		}
#if defined(RTCONFIG_WIRELESSREPEATER) && defined(RTCONFIG_PROXYSTA)
	}
#endif

	return ret;
}

static int ej_wl_sta_list(int unit, webs_t wp)
{
	WIFI_STA_TABLE *sta_info;
	char *value;
	int firstRow = 1;
	int i;
	int from_app = 0;

	from_app = check_user_agent(user_agent);

	if ((sta_info = malloc(sizeof(*sta_info))) != NULL)
	{
		getSTAInfo(unit, sta_info);
		for(i = 0; i < sta_info->Num; i++)
		{
			if (firstRow == 1)
				firstRow = 0;
			else
				websWrite(wp, ", ");

			if (from_app == 0)
				websWrite(wp, "[");

			websWrite(wp, "\"%s\"", sta_info->Entry[i].addr);

			if (from_app != 0) {
				websWrite(wp, ":{");
				websWrite(wp, "\"isWL\":");
			}

			value = "Yes";
			if (from_app == 0)
				websWrite(wp, ", \"%s\"", value);
			else
				websWrite(wp, "\"%s\"", value);

			value = "";

			if (from_app == 0)
				websWrite(wp, ", \"%s\"", value);
	
			if (from_app != 0) {
				websWrite(wp, ",\"rssi\":");
			}

			if (from_app == 0)
				websWrite(wp, ", \"%d\"", sta_info->Entry[i].rssi);
			else
				websWrite(wp, "\"%d\"", sta_info->Entry[i].rssi);

			if (from_app == 0)
				websWrite(wp, "]");
			else
				websWrite(wp, "}");
		}
		free(sta_info);
	}
	return 0;
}

#if defined(RTCONFIG_STAINFO)
/**
 * Format:
 * 	[ MAC, TX_RATE, RX_RATE, CONNECT_TIME, IDX ]
 * IDX:	main/GN1/GN2/GN3
 */
static int wl_stainfo_list(int unit, webs_t wp)
{
	WIFI_STA_TABLE *sta_info;
	WLANCONFIG_LIST *r;
	char idx_str[8];
	int i, s, firstRow = 1;

	if ((sta_info = malloc(sizeof(*sta_info))) == NULL)
		return 0 ;

	getSTAInfo(unit, sta_info);
	for(i = 0, r = &sta_info->Entry[0]; i < sta_info->Num; i++, r++) {
		if (firstRow == 1)
			firstRow = 0;
		else
			websWrite(wp, ", ");

		websWrite(wp, "[");
		websWrite(wp, "\"%s\"", r->addr);
		websWrite(wp, ", \"%s\"", r->txrate);
		websWrite(wp, ", \"%s\"", r->rxrate);
		websWrite(wp, ", \"%s\"", r->conn_time);
		s = r->subunit;
		if (s < 0 || s > 7)
			s = 0;
		if (!s)
			strlcpy(idx_str, "main", sizeof(idx_str));
		else
			snprintf(idx_str, sizeof(idx_str), "GN%d", s);
		websWrite(wp, ", \"%s\"", idx_str);
		websWrite(wp, "]");
	}
	free(sta_info);
	return 0;
}

#endif	/* RTCONFIG_STAINFO */

char *getWscStatus(int unit)
{
	char buf[512];
	FILE *fp;
	int len;
	char *pt1,*pt2;

	snprintf(buf, sizeof(buf), "hostapd_cli -i%s wps_get_status", get_wifname(unit));
	fp = popen(buf, "r");
	if (fp) {
		memset(buf, 0, sizeof(buf));
		len = fread(buf, 1, sizeof(buf), fp);
		pclose(fp);
		if (len > 1) {
			buf[len-1] = '\0';
			pt1 = strstr(buf, "Last WPS result: ");
			if (pt1) {
				pt2 = pt1 + strlen("Last WPS result: ");
				pt1 = strstr(pt2, "Peer Address: ");
				if (pt1) {
					*pt1 = '\0';
					chomp(pt2);
				}
				return pt2;
			}
		}
	}
	return "";
}

char *getAPPIN(int unit)
{
	static char buffer[128];
	char cmd[64];
	FILE *fp;
	int len;

	buffer[0] = '\0';
	snprintf(cmd, sizeof(cmd), "hostapd_cli -i%s wps_ap_pin get", get_wifname(unit));
	fp = popen(cmd, "r");
	if (fp) {
		len = fread(buffer, 1, sizeof(buffer), fp);
		pclose(fp);
		if (len > 1) {
			buffer[len] = '\0';
			//dbg("%s: AP PIN[%s]\n", __FUNCTION__, buffer);
			if(!strncmp(buffer,"FAIL",4))
			   strlcpy(buffer,nvram_get("secret_code"), sizeof(buffer));
			return buffer;
		}
	}
	return "";
}

#if 0
static void convertToUpper(char *str)
{
	if(str == NULL)
		return;
	while(*str)
	{
		if(*str >= 'a' && *str <= 'z')
		{
			*str &= (unsigned char)~0x20;
		}
		str++;
	}
}
#endif

#if 1
#define target 7
char str[target][40]={"Address:","ESSID:","Frequency:","Quality=","Encryption key:","IE:","Authentication Suites"};
static int wl_scan(int eid, webs_t wp, int argc, char_t **argv, int unit)
{
   	int apCount=0,retval=0;
	char header[128];
	char tmp[128], prefix[] = "wlXXXXXXXXXX_";
	char cmd[300];
	FILE *fp;
	char buf[target][200];
	int i,fp_len;
	char *pt1,*pt2;
	char a1[10],a2[10];
	char ssid_str[256];
	char ch[4],ssid[33],address[18],enc[9],auth[16],sig[9],wmode[8];
	int  lock;

	dbg("Please wait...");
	lock = file_lock("nvramcommit");
	system("rm -f /tmp/wlist");
	snprintf(prefix, sizeof(prefix), "wl%d_", unit);
	snprintf(cmd, sizeof(cmd), "iwlist %s scanning >> /tmp/wlist",nvram_safe_get(strcat_r(prefix, "ifname", tmp)));
	system(cmd);
	file_unlock(lock);
	
	if((fp= fopen("/tmp/wlist", "r"))==NULL) 
	   return -1;
	
	memset(header, 0, sizeof(header));
	snprintf(header, sizeof(header), "%-4s%-33s%-18s%-9s%-16s%-9s%-8s\n", "Ch", "SSID", "BSSID", "Enc", "Auth", "Siganl(%)", "W-Mode");

	dbg("\n%s", header);

	retval += websWrite(wp, "[");
	while(1)
	{
		memset(buf,0,sizeof(buf));
		fp_len=0;
		for(i=0;i<target;i++)
		{
		   	while(fgets(buf[i], sizeof(buf[i]), fp))
			{
				fp_len += strlen(buf[i]);  	
				if(i!=0 && strstr(buf[i],"Cell") && strstr(buf[i],"Address"))
				{
					fseek(fp,-fp_len, SEEK_CUR);
					fp_len=0;
					break;
				}
				else
			  	{ 	   
					if(strstr(buf[i],str[i]))
					{
					 	fp_len =0;  	
						break;
					}	
					else
						memset(buf[i],0,sizeof(buf[i]));
				}	

			}
		        	
	      		//dbg("buf[%d]=%s\n",i,buf[i]);
		}

  		if(feof(fp)) 
		   break;

		apCount++;

		dbg("\napCount=%d\n",apCount);
		//ch
	        pt1 = strstr(buf[2], "Channel ");	
		if(pt1)
		{

			pt2 = strstr(pt1,")");
		   	memset(ch,0,sizeof(ch));
			strncpy(ch,pt1+strlen("Channel "),pt2-pt1-strlen("Channel "));
		}   

		//ssid
	        pt1 = strstr(buf[1], "ESSID:");	
		if(pt1)
		{
		   	memset(ssid,0,sizeof(ssid));
			strncpy(ssid,pt1+strlen("ESSID:")+1,strlen(buf[1])-2-(pt1+strlen("ESSID:")+1-buf[1]));
		}   


		//bssid
	        pt1 = strstr(buf[0], "Address: ");	
		if(pt1)
		{
		   	memset(address,0,sizeof(address));
			strncpy(address,pt1+strlen("Address: "),strlen(buf[0])-(pt1+strlen("Address: ")-buf[0])-1);
		}   
	

		//enc
		pt1=strstr(buf[4],"Encryption key:");
		if(pt1)
		{   
			if(strstr(pt1+strlen("Encryption key:"),"on"))
			{  	
				snprintf(enc, sizeof(enc),"ENC");
		
			} 
			else
				snprintf(enc, sizeof(enc),"NONE");
		}

		//auth
		memset(auth,0,sizeof(auth));
		snprintf(auth, sizeof(auth),"N/A");

		//sig
	        pt1 = strstr(buf[3], "Quality=");	
		pt2 = strstr(pt1,"/");
		if(pt1 && pt2)
		{
			memset(sig,0,sizeof(sig));
			memset(a1,0,sizeof(a1));
			memset(a2,0,sizeof(a2));
			strncpy(a1,pt1+strlen("Quality="),pt2-pt1-strlen("Quality="));
			strncpy(a2,pt2+1,strstr(pt2," ")-(pt2+1));
			snprintf(sig, sizeof(sig),"%d",atoi(a1)/atoi(a2));

		}   

		//wmode
		memset(wmode,0,sizeof(wmode));
		snprintf(wmode, sizeof(wmode),"11b/g/n");


#if 1
		dbg("%-4s%-33s%-18s%-9s%-16s%-9s%-8s\n",ch,ssid,address,enc,auth,sig,wmode);
#endif	


		memset(ssid_str, 0, sizeof(ssid_str));
		char_to_ascii(ssid_str, trim_r(ssid));
		if (apCount==1)
			retval += websWrite(wp, "[\"%s\", \"%s\"]", ssid_str, address);
		else
			retval += websWrite(wp, ", [\"%s\", \"%s\"]", ssid_str, address);

	}

	retval += websWrite(wp, "]");
	fclose(fp);
	return 0;
}   
#else
static int wl_scan(int eid, webs_t wp, int argc, char_t **argv, int unit)
{
	int retval = 0, i = 0, apCount = 0;
	char data[8192];
	char ssid_str[256];
	char header[128];
	struct iwreq wrq;
	SSA *ssap;
	char tmp[128], prefix[] = "wlXXXXXXXXXX_";
	int lock;

	snprintf(prefix, sizeof(prefix), "wl%d_", unit);
	memset(data, 0x00, 255);
	strlcpy(data, "SiteSurvey=1", sizeof(data));
	wrq.u.data.length = strlen(data)+1; 
	wrq.u.data.pointer = data; 
	wrq.u.data.flags = 0; 

	lock = file_lock("nvramcommit");
	if (wl_ioctl(nvram_safe_get(strcat_r(prefix, "ifname", tmp)), RTPRIV_IOCTL_SET, &wrq) < 0)
	{
		file_unlock(lock);
		dbg("Site Survey fails\n");
		return 0;
	}
	file_unlock(lock);
	dbg("Please wait");
	sleep(1);
	dbg(".");
	sleep(1);
	dbg(".");
	sleep(1);
	dbg(".");
	sleep(1);
	dbg(".\n\n");
	memset(data, 0, 8192);
	strlcpy(data, "", sizeof(data));
	wrq.u.data.length = 8192;
	wrq.u.data.pointer = data;
	wrq.u.data.flags = 0;
	if (wl_ioctl(nvram_safe_get(strcat_r(prefix, "ifname", tmp)), RTPRIV_IOCTL_GSITESURVEY, &wrq) < 0)
	{
		dbg("errors in getting site survey result\n");
		return 0;
	}
	memset(header, 0, sizeof(header));
	//snprintf(header, sizeof(header), "%-3s%-33s%-18s%-8s%-15s%-9s%-8s%-2s\n", "Ch", "SSID", "BSSID", "Enc", "Auth", "Siganl(%)", "W-Mode", "NT");
#if 0// defined(RTN14U)
	snprintf(header, sizeof(header), "%-4s%-33s%-18s%-9s%-16s%-9s%-8s%-4s%-5s\n", "Ch", "SSID", "BSSID", "Enc", "Auth", "Siganl(%)", "W-Mode"," WPS", " DPID");
#else
	snprintf(header, sizeof(header), "%-4s%-33s%-18s%-9s%-16s%-9s%-8s\n", "Ch", "SSID", "BSSID", "Enc", "Auth", "Siganl(%)", "W-Mode");
#endif
	dbg("\n%s", header);
	if (wrq.u.data.length > 0)
	{
#if defined(RTN65U)
		if (unit == 0 && get_model() == MODEL_RTN65U)
		{
			char *encryption;
			SITE_SURVEY_RT3352_iNIC *pSsap, *ssAP;

			pSsap = ssAP = (SITE_SURVEY_RT3352_iNIC *) (1 /* '\n' */ + wrq.u.data.pointer +  sizeof(SITE_SURVEY_RT3352_iNIC) /* header */);
			while(((unsigned int)wrq.u.data.pointer + wrq.u.data.length) > (unsigned int) ssAP)
			{
				ssAP->channel   [sizeof(ssAP->channel)    -1] = '\0';
				ssAP->ssid      [32                         ] = '\0';
				ssAP->bssid     [17                         ] = '\0';
				ssAP->encryption[sizeof(ssAP->encryption) -1] = '\0';
				if((encryption = strchr(ssAP->authmode, '/')) != NULL)
				{
					memmove(ssAP->encryption, encryption +1, sizeof(ssAP->encryption) -1);
					memset(encryption, ' ', sizeof(ssAP->authmode) - (encryption - ssAP->authmode));
					*encryption = '\0';
				}
				ssAP->authmode  [sizeof(ssAP->authmode)   -1] = '\0';
				ssAP->signal    [sizeof(ssAP->signal)     -1] = '\0';
				ssAP->wmode     [sizeof(ssAP->wmode)      -1] = '\0';
				ssAP->extch     [sizeof(ssAP->extch)      -1] = '\0';
				ssAP->nt        [sizeof(ssAP->nt)         -1] = '\0';
				ssAP->wps       [sizeof(ssAP->wps)        -1] = '\0';
				ssAP->dpid      [sizeof(ssAP->dpid)       -1] = '\0';

				convertToUpper(ssAP->bssid);
				ssAP++;
				apCount++;
			}

			if (apCount)
			{
				retval += websWrite(wp, "[");
				for (i = 0; i < apCount; i++)
				{
					dbg("%-4s%-33s%-18s%-9s%-16s%-9s%-8s\n",
						pSsap[i].channel,
						pSsap[i].ssid,
						pSsap[i].bssid,
						pSsap[i].encryption,
						pSsap[i].authmode,
						pSsap[i].signal,
						pSsap[i].wmode
					);

					memset(ssid_str, 0, sizeof(ssid_str));
					char_to_ascii(ssid_str, trim_r(pSsap[i].ssid));

					if (!i)
						retval += websWrite(wp, "[\"%s\", \"%s\"]", ssid_str, pSsap[i].bssid);
					else
						retval += websWrite(wp, ", [\"%s\", \"%s\"]", ssid_str, pSsap[i].bssid);
				}
				retval += websWrite(wp, "]");
				dbg("\n");
			}
			else
				retval += websWrite(wp, "[]");
			return retval;
		}
#endif
		ssap=(SSA *)(wrq.u.data.pointer+strlen(header)+1);
		int len = strlen(wrq.u.data.pointer+strlen(header))-1;
		char *sp, *op;
 		op = sp = wrq.u.data.pointer+strlen(header)+1;
		while (*sp && ((len - (sp-op)) >= 0))
		{
			ssap->SiteSurvey[i].channel[3] = '\0';
			ssap->SiteSurvey[i].ssid[32] = '\0';
			ssap->SiteSurvey[i].bssid[17] = '\0';
			ssap->SiteSurvey[i].encryption[8] = '\0';
			ssap->SiteSurvey[i].authmode[15] = '\0';
			ssap->SiteSurvey[i].signal[8] = '\0';
			ssap->SiteSurvey[i].wmode[7] = '\0';
#if 0//defined(RTN14U)
			ssap->SiteSurvey[i].wps[3] = '\0';
			ssap->SiteSurvey[i].dpid[4] = '\0';
#endif
			sp+=strlen(header);
			apCount=++i;
		}
		if (apCount)
		{
			retval += websWrite(wp, "[");
			for (i = 0; i < apCount; i++)
			{
			   	dbg("\napCount=%d\n",i);
				dbg(
#if 0//defined(RTN14U)
				"%-4s%-33s%-18s%-9s%-16s%-9s%-8s%-4s%-5s\n",
#else
				"%-4s%-33s%-18s%-9s%-16s%-9s%-8s\n",
#endif
					ssap->SiteSurvey[i].channel,
					(char*)ssap->SiteSurvey[i].ssid,
					ssap->SiteSurvey[i].bssid,
					ssap->SiteSurvey[i].encryption,
					ssap->SiteSurvey[i].authmode,
					ssap->SiteSurvey[i].signal,
					ssap->SiteSurvey[i].wmode
#if 0//defined(RTN14U)
					, ssap->SiteSurvey[i].wps
					, ssap->SiteSurvey[i].dpid
#endif
				);

				memset(ssid_str, 0, sizeof(ssid_str));
				char_to_ascii(ssid_str, trim_r(ssap->SiteSurvey[i].ssid));

				if (!i)
//					retval += websWrite(wp, "\"%s\"", ssap->SiteSurvey[i].bssid);
					retval += websWrite(wp, "[\"%s\", \"%s\"]", ssid_str, ssap->SiteSurvey[i].bssid);
				else
//					retval += websWrite(wp, ", \"%s\"", ssap->SiteSurvey[i].bssid);
					retval += websWrite(wp, ", [\"%s\", \"%s\"]", ssid_str, ssap->SiteSurvey[i].bssid);
			}
			retval += websWrite(wp, "]");
			dbg("\n");
		}
		else
			retval += websWrite(wp, "[]");
	}
	return retval;
}

#endif
int
ej_wl_scan(int eid, webs_t wp, int argc, char_t **argv)
{
	return wl_scan(eid, wp, argc, argv, 0);
}

static int ej_wl_channel_list(int eid, webs_t wp, int argc, char_t **argv, int unit)
{
	int retval = 0;
	char tmp[128], prefix[] = "wlXXXXXXXXXX_";
	char *country_code;
	char chList[256];
	int band;

	snprintf(prefix, sizeof(prefix), "wl%d_", unit);
	country_code = nvram_get(strcat_r(prefix, "country_code", tmp));
	band = unit;

	if (country_code == NULL || strlen(country_code) != 2) return retval;

	if (band != 0 && band != 1) return retval;

	//try getting channel list via wifi driver first
	if(get_channel_list_via_driver(unit, chList, sizeof(chList)) > 0)
	{
		retval += websWrite(wp, "[%s]", chList);
	}
	else if(get_channel_list_via_country(unit, country_code, chList, sizeof(chList)) > 0)
	{
		retval += websWrite(wp, "[%s]", chList);
	}
	return retval;
}


int
ej_nat_accel_status(int eid, webs_t wp, int argc, char_t **argv)
{
	int retval = 0;

	/* always 1, alpine has no HW NAT */
	retval += websWrite(wp, "1");

	return retval;
}

#if 0
#ifdef RTCONFIG_PROXYSTA
int
ej_wl_auth_psta(int eid, webs_t wp, int argc, char_t **argv)
{
	int retval = 0;

	if(nvram_match("wlc_state", "2"))	//connected
		retval += websWrite(wp, "wlc_state=1;wlc_state_auth=0;");
	//else if(?)				//authorization failed
	//	retval += websWrite(wp, "wlc_state=2;wlc_state_auth=1;");
	else					//disconnected
		retval += websWrite(wp, "wlc_state=0;wlc_state_auth=0;");

	return retval;
}
#endif
#endif
