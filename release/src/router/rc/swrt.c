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
 *
 * Copyright 2018-2022, SWRT.
 * Copyright 2018-2022, paldier <paldier@hotmail.com>.
 * Copyright 2018-2022, lostlonger<lostlonger.g@gmail.com>.
 * All Rights Reserved.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "rc.h"
#include <shared.h>
#include <version.h>
#include <shutils.h>
#include <json.h>
#include "swrt.h"
#include <curl/curl.h>
#include <sys/mount.h>
#include <sys/stat.h>
#ifndef MNT_DETACH
#define MNT_DETACH	0x00000002
#endif

#if defined(RTCONFIG_SOFTCENTER)
static void firmware_ver(void)
{
    char tmp[6] = {0};
    strncpy(tmp, RT_FWVER, 5);//5.x.x[beta]
	doSystem("dbus set softcenter_firmware_version='%s'",tmp);
}
#endif

void swrt_insmod(){
	eval("insmod", "nfnetlink");
	eval("insmod", "ip_set");
	eval("insmod", "ip_set_bitmap_ip");
	eval("insmod", "ip_set_bitmap_ipmac");
	eval("insmod", "ip_set_bitmap_port");
	eval("insmod", "ip_set_hash_ip");
	eval("insmod", "ip_set_hash_ipport");
	eval("insmod", "ip_set_hash_ipportip");
	eval("insmod", "ip_set_hash_ipportnet");
	eval("insmod", "ip_set_hash_ipmac");
	eval("insmod", "ip_set_hash_ipmark");
	eval("insmod", "ip_set_hash_net");
	eval("insmod", "ip_set_hash_netport");
	eval("insmod", "ip_set_hash_netiface");
	eval("insmod", "ip_set_hash_netnet");
	eval("insmod", "ip_set_hash_netportnet");
	eval("insmod", "ip_set_hash_mac");
	eval("insmod", "ip_set_list_set");
	eval("insmod", "nf_tproxy_core");
	eval("insmod", "xt_TPROXY");
	eval("insmod", "xt_set");
	modprobe("cifs");
}

void swrt_init()
{
	_dprintf("############################ SWRT init #################################\n");
#if defined(RTCONFIG_SOFTCENTER)
	nvram_set("sc_wan_sig", "0");
	nvram_set("sc_nat_sig", "0");
	nvram_set("sc_mount_sig", "0");
	nvram_set("sc_services_start_sig", "0");
	nvram_set("sc_services_stop_sig", "0");
	nvram_set("sc_unmount_sig", "0");
#endif
#if defined(RTCONFIG_ENTWARE)
	nvram_set("entware_wan_sig", "0");
	nvram_set("entware_stop_sig", "0");
#endif
	swrt_insmod();
	swrt_init_model();
#if defined(RTCONFIG_ROG_UI)
	nvram_set("swrt_rog", "1");
#endif
}

#if defined(K3)
static int k3screena(void){
	_dprintf("....k3screen start a....\n");
	char fwver[32];
	FILE *fpu;
	if (!nvram_get("hd_version"))
		nvram_set("hd_version", "A1/A2");

	if (!nvram_get("product"))
		nvram_set("product", "K3");
	if(nvram_get_int("sw_mode") == 1){
		sprintf(fwver, "%s_%s", nvram_get("buildno"), nvram_get("extendno"));
		nvram_set("fw_version", fwver);
		nvram_set("sw_version", fwver);
		nvram_set("wan_ifname", "vlan2");
	} else {
		sprintf(fwver, "AP:%s", nvram_safe_get("lan_ipaddr"));
		nvram_set("fw_version", fwver);
		nvram_set("sw_version", fwver);
		nvram_set("wan_ifname", "vlan1");
	}
	if (!nvram_get("wan_pppoe_ifname"))
		nvram_set("wan_pppoe_ifname", "ppp0");
	if (nvram_get_int("bsd_role") != 0)
		nvram_set_int("bsd_role", 0);
	if (!nvram_get("pingcheck"))
		nvram_set("pingcheck", "1");
	if (nvram_get_int("get_wan_port_status") != 1)
		nvram_set_int("get_wan_port_status", 1);
	if (!nvram_get("screen_time"))
		nvram_set("screen_time", "20");
	if (!nvram_get("screen_2G5G_pwd_en"))
		nvram_set("screen_2G5G_pwd_en", "1");
	if (!nvram_get("city_ch"))
		nvram_set("city_ch", "北京");
	if (!nvram_get("county_ch"))
		nvram_set("county_ch", "北京");
	if (!nvram_get("city_id"))
		nvram_set("city_id", "WX4FBXXFKE4F");
	if (!nvram_get("vis_ssid_enable"))
		nvram_set("vis_ssid_enable", "1");
	if (!nvram_get("screen_guest_pwd_en"))
		nvram_set("screen_guest_pwd_en", "1");
	if (!nvram_get("vis_ssid"))
		nvram_set("vis_ssid", "ASUS_GUEST");
	if (!nvram_get("vis_ssid_pwd"))
		nvram_set("vis_ssid_pwd", "1234567890");
	nvram_commit();
	doSystem("killall -q -9 phi_speed wl_cr uhmi k3screenctrl update_weather 2>/dev/null");
	if ((fpu = fopen("/tmp/uhmi.sh", "w"))){
		fprintf(fpu, "#!/bin/sh\n");
		fprintf(fpu, "mkdir -p /jffs/softcenter/lib\n");
		fprintf(fpu, "ln -sf /usr/share/libwlcr.so /jffs/softcenter/lib/libwlcr.so\n");
		fprintf(fpu, "devmem 0x1800c1c1 32 0x00001f0f\n");
		fprintf(fpu, "[ -n \"$(echo $LD_LIBRARY_PATH |grep \"jffs\")\" ] || export LD_LIBRARY_PATH=/jffs/softcenter/lib:/lib:/usr/lib\n");
		fprintf(fpu, "phi_speed &\n");
		fprintf(fpu, "wl_cr &\n");
		fprintf(fpu, "uhmi &\n");
		fclose(fpu);
	}
	doSystem("chmod +x /tmp/uhmi.sh");
	doSystem("/tmp/uhmi.sh &");
	doSystem("echo '#!/bin/sh' > /tmp/update_weather");
	doSystem("echo 'while [ 1 ]' >> /tmp/update_weather");
	doSystem("echo 'do' >> /tmp/update_weather");
	doSystem("echo 'weather update_weather' >> /tmp/update_weather");
	doSystem("echo 'sleep 3600' >> /tmp/update_weather");
	doSystem("echo 'done' >> /tmp/update_weather");
	doSystem("chmod +x /tmp/update_weather");
	doSystem("/tmp/update_weather &");
	_dprintf("....k3screen ok....\n");
	return 0;
}

static int k3screenb(void){
	_dprintf("....k3screen start b....\n");
	char *timeout;
	doSystem("mkdir -p /tmp/k3screenctrl");
	doSystem("killall -q -9 phi_speed wl_cr uhmi k3screenctrl update_weather k3screend 2>/dev/null");
	//doSystem("/usr/sbin/k3screend &");
	//doSystem("chmod +x /tmp/k3screenctrl/*.sh");
	if (nvram_get_int("k3screen_timeout")==1)
		timeout = "-m0";
	else
		timeout = "-m30";
	char *k3screenctrl_argv[] = { "k3screenctrl", timeout,NULL };
	char *k3screend_argv[] = { "k3screend",NULL };
	pid_t pid;
	_eval(k3screend_argv, NULL, 0, &pid);
	_eval(k3screenctrl_argv, NULL, 0, &pid);
	_dprintf("....k3screen ok....\n");
	return 0;
}
int start_k3screen(void){
	logmessage("K3", "屏幕支援程序开始启动");
	if (!nvram_get("k3screen")){
		nvram_set("k3screen", "b");
		nvram_commit();
	}
	_dprintf("....k3screen start....\n");
	if ((strcmp(nvram_get("k3screen"), "A")==0) || (strcmp(nvram_get("k3screen"), "a")==0))
		return k3screena();
	else if ((strcmp(nvram_get("k3screen"), "B")==0) || (strcmp(nvram_get("k3screen"), "b")==0))
		return k3screenb();
	else {
		nvram_set("k3screen", "b");
		nvram_commit();
		return k3screenb();
		}
	return 0;
}
#endif

#if defined(TUFAX3000) || defined(RTAX58U)
void enable_4t4r_ax58(void)
{
//ensure that the hardware support 4t4r
	if(!strcmp(nvram_get("1:sw_rxchain_mask"), "0xf") ){
//4t4r
		nvram_set("1:sw_txchain_mask", "0xf");
		nvram_commit();
	} else {
//2t2r
		nvram_set("1:sw_txchain_mask", "0x9");
		nvram_commit();
	}
}
void enable_4t4r(void)
{
//hack asus watchdog and modify this to enable 4t4r
//ensure that the hardware support 4t4r
	if(!strcmp(cfe_nvram_get("1:sw_rxchain_mask"), "0xf") && strcmp(cfe_nvram_get("1:sw_txchain_mask"), "0xf")){
		doSystem("envrams");
		doSystem("envram set 1:sw_txchain_mask=0xf");
		doSystem("envram commit");
	}
}
#endif

void swrt_init_done(){
	_dprintf("############################ SWRT init done #################################\n");
#if defined(RTCONFIG_SOFTCENTER)
//cifs
	if(nvram_match("sc_mount","2")){
		char *path = nvram_get("sc_cifs_url");
		char *user = nvram_safe_get("sc_cifs_user");
		char *pw = nvram_safe_get("sc_cifs_pw");
		char opt[410] = {0};
		snprintf(opt, sizeof(opt), "username=%s%s%s", *user ? user : "guest", *pw ? ",password=" : "", pw);
		mount(path, "/jffs/softcenter", "cifs", MS_NOATIME|MS_NODIRATIME, opt);
//jffs > 30MB
	}else if(!f_exists("/jffs/softcenter/scripts/ks_tar_install.sh") && nvram_match("sc_mount","0")){
		doSystem("/usr/sbin/jffsinit.sh &");
		logmessage("Softcenter/软件中心", "Installing/开始安装......");
		logmessage("Softcenter/软件中心", "Wait a minute/1分钟后完成安装");
		_dprintf("....softcenter ok....\n");
	}else if(f_exists("/jffs/softcenter/scripts/ks_tar_install.sh") && nvram_match("sc_mount","0"))
		nvram_set("sc_installed","1");
//usb
	//else if (!f_exists("/jffs/softcenter/scripts/ks_tar_intall.sh") && nvram_match("sc_mount","1"))
		//nvram_set("sc_installed","0");
	if(f_exists("/jffs/.asusrouter")){
		unlink("/jffs/.asusrouter");
		doSystem("sed -i '/softcenter-wan.sh/d' /jffs/scripts/wan-start");
		doSystem("sed -i '/softcenter-net.sh/d' /jffs/scripts/nat-start");
		doSystem("sed -i '/softcenter-mount.sh/d' /jffs/scripts/post-mount");
	}
	firmware_ver();
#endif
#if defined(RTCONFIG_QCA) || defined(RTCONFIG_RALINK)
	if(!nvram_get("bl_ver"))
		nvram_set("bl_ver", nvram_safe_get("blver"));
#elif defined(RTCONFIG_LANTIQ)
#if !defined(K3C)
	if(!nvram_get("bl_ver"))
		doSystem("nvram set bl_ver=`uboot_env --get --name bl_ver`");
#endif
#endif
	if(!nvram_get("modelname"))
//non asus
#if defined(SBRAC1900P)
		nvram_set("modelname", "SBRAC1900P");
#elif defined(EA6700)
		nvram_set("modelname", "EA6700");
#elif defined(DIR868L)
		nvram_set("modelname", "DIR868L");
#elif defined(R6300V2)
		nvram_set("modelname", "R6300V2");
#elif defined(SBRAC3200P)
		nvram_set("modelname", "SBRAC3200P");
#elif defined(K3)
		nvram_set("modelname", "K3");
#elif defined(XWR3100)
		nvram_set("modelname", "XWR3100");
#elif defined(R7000P)
		nvram_set("modelname", "R7000P");
#elif defined(R8500)
		nvram_set("modelname", "R8500");
#elif defined(R8000P)
		nvram_set("modelname", "R8000P");
#elif defined(RAX20)
		nvram_set("modelname", "RAX20");
#elif defined(RAX70)
		nvram_set("modelname", "RAX70");
#elif defined(RAX80)
		nvram_set("modelname", "RAX80");
#elif defined(RAX120)
		nvram_set("modelname", "RAX120");
#elif defined(RAX200)
		nvram_set("modelname", "RAX200");
#elif defined(TY6201_BCM)
		nvram_set("modelname", "TY6201_BCM");
#elif defined(TY6201_RTK)
		nvram_set("modelname", "TY6201_RTK");
#elif defined(TY6202)
		nvram_set("modelname", "TY6202");
#elif defined(K3C)
		nvram_set("modelname", "K3C");
#elif defined(MR60)
		nvram_set("modelname", "MR60");
#elif defined(MS60)
		nvram_set("modelname", "MS60");
#elif defined(SWRT360V6)
		nvram_set("modelname", "360V6");
#elif defined(GLAX1800)
		nvram_set("modelname", "GLAX1800");
//asus
#elif defined(RTAC68U)
		nvram_set("modelname", "RTAC68U");
#elif defined(RTAC3200)
		nvram_set("modelname", "RTAC3200");
#elif defined(RTAC3100)
		nvram_set("modelname", "RTAC3100");
#elif defined(RTAC88U)
		nvram_set("modelname", "RTAC88U");
#elif defined(RTAC5300)
		nvram_set("modelname", "RTAC5300");
#elif defined(RTAC86U)
		nvram_set("modelname", "RTAC86U");
#elif defined(GTAC2900)
		nvram_set("modelname", "GTAC2900");
#elif defined(GTAC5300)
		nvram_set("modelname", "GTAC5300");
#elif defined(RTAX53U)
		nvram_set("modelname", "RTAX53U");
#elif defined(RTAX54)
		nvram_set("modelname", "RTAX54");
#elif defined(RTAX55) || defined(RTAX1800)
		nvram_set("modelname", "RTAX55");
#elif defined(RTAX56U)
		nvram_set("modelname", "RTAX56U");
#elif defined(RTAX58U) || defined(RTAX3000)
		nvram_set("modelname", "RTAX58U");
#elif defined(RTAX58U_V2)
		nvram_set("modelname", "RTAX58UV2");
#elif defined(TUFAX3000)
		nvram_set("modelname", "TUFAX3000");
#elif defined(TUFAX5400)
		nvram_set("modelname", "TUFAX5400");
#elif defined(GSAX3000)
		nvram_set("modelname", "GSAX3000");
#elif defined(GSAX5400)
		nvram_set("modelname", "GSAX5400");
#elif defined(RTAX68U)
		nvram_set("modelname", "RTAX68U");
#elif defined(RTAX82U)
		nvram_set("modelname", "RTAX82U");
#elif defined(RTAX86U)
		nvram_set("modelname", "RTAX86U");
#elif defined(RTAX88U)
		nvram_set("modelname", "RTAX88U");
#elif defined(GTAX6000)
		nvram_set("modelname", "GTAX6000");
#elif defined(GTAX11000)
		nvram_set("modelname", "GTAX11000");
#elif defined(GTAX11000_PRO)
		nvram_set("modelname", "GTAX11000PRO");
#elif defined(GTAXE11000)
		nvram_set("modelname", "GTAXE11000");
#elif defined(GTAXE16000)
		nvram_set("modelname", "GTAXE16000");
#elif defined(BLUECAVE)
		nvram_set("modelname", "BLUECAVE");
#elif defined(RTAC82U)
		nvram_set("modelname", "RTACRH17");
#elif defined(RTAX89U)
		nvram_set("modelname", "RTAX89X");
#elif defined(RTAC85P)
		nvram_set("modelname", "RTAC85P");
#elif defined(RMAC2100)
		nvram_set("modelname", "RMAC2100");
#elif defined(R6800)
		nvram_set("modelname", "R6800");
#elif defined(TUFAC1750)
		nvram_set("modelname", "TUFAC1750");
#elif defined(RTAC95U)
		nvram_set("modelname", "ZENWIFICT8");
#elif defined(RTAX56XD4)
		nvram_set("modelname", "ZENWIFIXD4");
#elif defined(RTAX82_XD6)
		nvram_set("modelname", "ZENWIFIXD6");
#elif defined(RTAX95Q)
		nvram_set("modelname", "ZENWIFIXT8");
#elif defined(RTAXE95Q)
		nvram_set("modelname", "ZENWIFIET8");
#elif defined(XT12)
		nvram_set("modelname", "ZENWIFIXT12");
#elif defined(ET12)
		nvram_set("modelname", "ZENWIFIET12");
#elif defined(XD4PRO)
		nvram_set("modelname", "ZENWIFIXD4PRO");
#elif defined(XT8PRO)
		nvram_set("modelname", "ZENWIFIXT8PRO");
#elif defined(ET8PRO)
		nvram_set("modelname", "ZENWIFIET8PRO");
#elif defined(PLAX56_XP4)
		nvram_set("modelname", "ZENWIFIXP4");
#elif defined(ETJ)
		nvram_set("modelname", "ZENWIFIET10");
#endif
#if defined(R8000P)
	nvram_set("ping_target", "www.taobao.com");
#endif
	nvram_commit();
#if defined(K3)
	start_k3screen();
#endif
#if defined(SWRT_VER_MAJOR_B)
	del_rc_support("amasRouter");
	del_rc_support("amas");
#endif
	gen_swrtid();
#if defined(R8000P)
    add_rc_support("uu_accel");
#endif
#if defined(RAX120)
	rax120_lanled();
#endif
#if defined(RTCONFIG_SWRT_LED)
	swrt_ledon();//to fix LED state
#endif
}


#define FWUPDATE_DBG(fmt,args...) \
        if(1) { \
                char info[1024]; \
                snprintf(info, sizeof(info), "echo \"[FWUPDATE][%s:(%d)]"fmt"\" >> /tmp/webs_upgrade.log", __FUNCTION__, __LINE__, ##args); \
                system(info); \
        }

int str_split(char* buf, char** s, int s_size) {
	int curr = 0;
	char* token = strtok(buf, ".");
	while(token && curr < s_size) {
		s[curr++] = token;
		token = strtok(NULL, ".");
	}
	return curr;
}

int versioncmp(char *cur_fwver, char *fwver) {
	char buf1[20] = {0}, buf2[20] = {0};
	char* s1[20] = {0};
	char* s2[20] = {0};
	char* end;
	int i, n1, n2, s1_len, s2_len, s_max, rlt;
	strncpy(buf1, cur_fwver, strlen(cur_fwver));
	strncpy(buf2, fwver, strlen(fwver));
	//fprintf(stderr, "%s\n", buf1);
	//fprintf(stderr, "%s\n", buf2);

	s1_len = str_split(buf1, s1, 10);
	s2_len = str_split(buf2, s2, 10);

	//fprintf(stderr, "s1=%d, s2=%d\n", s1_len, s2_len);

	s_max = (s1_len > s2_len ? s1_len: s2_len);

	rlt = 0;
	for(i = 0; i < s_max; i++) {
		if(!s1[i]) {
			s1[i] = "0";
		}
		if(!s2[i]) {
			s2[i] = "0";
		}

		n1 = strtol(s1[i], &end, 10);
		if(*end) {
			//parse error, but not break hear
			rlt = 1;
		}

		n2 = strtol(s2[i], &end, 10);
		if(*end) {
			if (1 == rlt) {
				//both error
				rlt = 0;
				break;
			}
			rlt = -1;
			break;
		}

		if(1 == rlt) {
			break;
		}

		if (n1 == n2) {
			continue;
		} else if(n1 < n2) {
			rlt = 1;
			break;
		} else {
			rlt = -1;
			break;
		}
	}
	return rlt;
}

size_t wirtefunc(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
	return fwrite(ptr, size, nmemb, stream);
}

int curl_download_swrt(const char *url, const char *file_path, long timeout)
{
	FILE *fp;
	CURL *curl = NULL;
	CURLcode ret;

	if(url == NULL || file_path == NULL)
		return -1;

	curl_global_init(CURL_GLOBAL_ALL);
	curl = curl_easy_init();

	if(!curl)
		return -3;

	unlink(file_path);
	if ((fp = fopen(file_path, "wb")) == NULL)
		return -2;

	curl_easy_setopt(curl, CURLOPT_URL, url);
	curl_easy_setopt(curl, CURLOPT_PROTOCOLS, CURLPROTO_HTTPS);
	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, timeout);
	curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
	curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, wirtefunc);

	//curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1);
	curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);

	ret = curl_easy_perform(curl);
	fclose(fp);
	curl_easy_cleanup(curl);

	if (ret != CURLE_OK)
		return -4;
	return 0;
}

int swrt_firmware_check_update_main(int argc, char *argv[])
{
	//char notetxt[]="/tmp/release_note.txt";
	//char downloadphp[]="download.php";
	//char sqdownloadphp[]="SQ_download.php";
	//char wlanutdatetxt[]="/tmp/wlan_utdate.txt";
	//char wlan_update_mrflag1[]="wlan_update_mrflag1.zip";
	FILE *fpupdate;
	int download;
	char url[100];
	char log[200];
	char serverurl[]="https://update.paldier.com";
	char serverupdate[]="wlan_update_v2.zip";
	char localupdate[]="/tmp/wlan_update.txt";
	char releasenote[]="/tmp/release_note0.txt";
	char model[20], modelname[20], fsver[10], fwver[10], tag[10];
	char cur_fwver[10];
	char info[100];
	nvram_set("webs_state_update", "0");
	nvram_set("webs_state_flag", "0");
	nvram_set("webs_state_error", "0");
	nvram_set("webs_state_odm", "0");
	nvram_set("webs_state_url", "");
#ifdef RTCONFIG_AMAS
	nvram_set("cfg_check", "0");
	nvram_set("cfg_upgrade", "0");
#endif
	unlink("/tmp/webs_upgrade.log");
	unlink("/tmp/wlan_update.txt");
	unlink("/tmp/release_note0.txt");
	memset(cur_fwver, 0, sizeof(cur_fwver));
    strncpy(cur_fwver, RT_FWVER, 5);//5.x.x[beta]

	snprintf(url, sizeof(url), "%s/%s", serverurl, serverupdate);

	FWUPDATE_DBG("---- update dl_path_info for general %s/%s ----", serverurl, serverupdate);
	download=curl_download_swrt(url, localupdate, 8);
	if(!download)
	{
		fpupdate = fopen(localupdate, "r");
		if (!fpupdate)
			goto GODONE;
		//BLUECAVE#K3C#3004384#R7.0#g13e704e
		char buffer[1024];
		while(NULL != fgets(buffer, sizeof(buffer), fpupdate)){
			sscanf(buffer, "%[A-Z0-9-]#%[A-Z0-9]#%[0-9]#%[A-Z0-9.]#%[a-z0-9]", model, modelname, fsver, fwver, tag);
			_dprintf("%s#%s#%s#%s\n", model, modelname, fsver, fwver);
			if(!strcmp(model, nvram_get("productid")) && !strcmp(modelname, nvram_safe_get("modelname"))){
				if(strstr(fwver, "B") || strstr(fwver, "R") || strstr(fwver, "X")){
					_dprintf("%s#%s\n",fwver,cur_fwver);
					if(versioncmp((cur_fwver),(fwver+1)) == 1){
						nvram_set("webs_state_url", "");
						if(!strcmp(nvram_get("firmver"), "3.0.0.4"))
							snprintf(info, sizeof(info), "3004_%s_%s_%s-%s", nvram_get("buildno"), modelname, fwver, tag);
						else if(!strcmp(nvram_get("firmver"), "4.0.0.4"))
							snprintf(info, sizeof(info), "4004_%s_%s_%s-%s", nvram_get("buildno"), modelname, fwver, tag);
						else
							snprintf(info, sizeof(info), "5004_%s_%s_%s-%s", nvram_get("buildno"), modelname, fwver, tag);
						FWUPDATE_DBG("---- current version : %s ----", nvram_get("extendno"));
						FWUPDATE_DBG("---- productid : %s_%s-%s ----", modelname, fwver, tag);
						nvram_set("webs_state_info", info);
						nvram_set("webs_state_REQinfo", info);
						nvram_set("webs_state_flag", "1");
						nvram_set("webs_state_update", "1");
#ifdef RTCONFIG_AMAS
//						nvram_set("cfg_check", "9");
//						nvram_set("cfg_upgrade", "0");
#endif
						memset(url,'\0',sizeof(url));
						memset(log,'\0',sizeof(log));
						char releasenote_file[100];
						snprintf(releasenote_file, sizeof(releasenote_file), "%s_%s_%s_note.zip", nvram_get("productid"), nvram_get("webs_state_info"),nvram_get("preferred_lang"));
						snprintf(url, sizeof(url), "%s/%s", serverurl, releasenote_file);
						//snprintf(log, sizeof(log), "echo \"[FWUPDATE]---- download real release note %s/%s ----\" >> /tmp/webs_upgrade.log", serverurl, releasenote_file);
						//system(log);
						FWUPDATE_DBG("---- download real release note %s/%s ----", serverurl, releasenote_file);
						download=curl_download_swrt(url, releasenote, 8);
						if(download ==0 ){
							memset(url,'\0',sizeof(url));
							snprintf(releasenote_file, sizeof(releasenote_file), "%s_%s_US_note.zip", nvram_get("productid"), nvram_get("webs_state_info"));
							snprintf(url, sizeof(url), "%s/%s", serverurl, releasenote_file);
							//snprintf(log, sizeof(log), "echo \"[FWUPDATE]---- download real release note %s/%s ----\" >> /tmp/webs_upgrade.log", serverurl, releasenote_file);
							//system(log);
							FWUPDATE_DBG("---- download real release note %s/%s ----", serverurl, releasenote_file);
							curl_download_swrt(url, releasenote, 8);
						}
						FWUPDATE_DBG("---- firmware check update finish ----");
						return 0;
#if 0
						if(strstr(nt_center,"nt_center")){
							if((!nvram_get("webs_last_info"))||(strcmp(nvram_get("webs_last_info"), nvram_get("webs_state_info")))){
								system("Notify_Event2NC \"$SYS_FW_NWE_VERSION_AVAILABLE_EVENT\" \"{\\\"fw_ver\\\":\\\"$update_webs_state_info\\\"}\"");
								nvram_set("webs_last_info", nvram_get("webs_state_info"));
							}
						}
#endif
					}
				}
			}
		}
	}

GODONE:
	if(!strcmp(nvram_get("firmver"), "3.0.0.4"))
		snprintf(info, sizeof(info), "3004_%s_%s_%s-%s", nvram_get("buildno"), modelname, fwver, tag);
	else if(!strcmp(nvram_get("firmver"), "4.0.0.4"))
		snprintf(info, sizeof(info), "4004_%s_%s_%s-%s", nvram_get("buildno"), modelname, fwver, tag);
	else
		snprintf(info, sizeof(info), "5004_%s_%s_%s-%s", nvram_get("buildno"), modelname, fwver, tag);
	nvram_set("webs_state_url", "");
	nvram_set("webs_state_flag", "0");
	nvram_set("webs_state_error", "1");
	nvram_set("webs_state_odm", "0");
	nvram_set("webs_state_update", "1");
	nvram_set("webs_state_info", info);
	nvram_set("webs_state_REQinfo", info);
	nvram_set("webs_state_upgrade", "");
#ifdef RTCONFIG_AMAS
	nvram_set("cfg_check", "9");
	nvram_set("cfg_upgrade", "0");
#endif
	FWUPDATE_DBG("---- firmware check update finish ----");
	return 0;
}

#ifdef RTCONFIG_UUPLUGIN
void start_uu(void)
{
	stop_uu();

	if(getpid()!=1) {
		notify_rc("start_uu");
		return;
	}

	if(nvram_get_int("uu_enable"))
#if defined(RTCONFIG_SWRT_UU)
		exec_uu_swrt();
#else
		exec_uu();
#endif
}

void stop_uu(void)
{
	doSystem("killall uuplugin_monitor.sh");
	if (pidof("uuplugin") > 0)
		doSystem("killall uuplugin");
}
#endif
#if defined(RTCONFIG_SWRT_UU)
void exec_uu_swrt()
{
	FILE *fp;
	char buf[128];
	int download,i;
	char *g, *gg;
	char p[2][100];
	if(nvram_get_int("sw_mode") == 1){
		add_rc_support("uu_accel");
		mkdir("/tmp/uu", 0755);
		download = system("wget -t 2 -T 30 --dns-timeout=120 --header=Accept:text/plain -q --no-check-certificate 'https://router.uu.163.com/api/script/monitor?type=asuswrt-merlin' -O /tmp/uu/script_url");
		if (!download){
			_dprintf("download uuplugin script info successfully\n");
			if ((fp = fopen("/tmp/uu/script_url", "r"))!=NULL){
				fgets(buf, 128, fp);
				fclose(fp);
				unlink("/tmp/uu/script_url");
				i=0;
				g = strdup(buf);
				gg = strtok(g, ",");
				while(gg != NULL){
						strcpy(p[i], gg);
						i++;
						++download;
						gg = strtok(NULL, ",");
				}
				if (download > 0){
					_dprintf("URL: %s\n",p[0]);
					_dprintf("MD5: %s\n",p[1]);
					if ( !doSystem("wget -t 2 -T 30 --dns-timeout=120 --header=Accept:text/plain -q --no-check-certificate %s -O /tmp/uu/uuplugin_monitor.sh", p[0]))
					{
						_dprintf("download uuplugin script successfully\n");
						if ((fp = fopen("/tmp/uu/uuplugin_monitor.config", "w"))){
							fprintf(fp, "router=asuswrt-merlin\n");
							fprintf(fp, "model=\n");
							fclose(fp);
						}
						if((fp=popen("md5sum /tmp/uu/uuplugin_monitor.sh | sed 's/[ ][ ]*/ /g' | cut -d' ' -f1", "r")))
						{
							memset(buf,'\0',sizeof(buf));
							if((fread(buf, 1, 128, fp)))
							{
								buf[32]='\0';
								buf[33]='\0';
								if ( !strcasecmp(buf, p[1]))
								{
									pid_t pid;
									char *uu_argv[] = { "/tmp/uu/uuplugin_monitor.sh", NULL };
									_dprintf("prepare to execute uuplugin stript...\n");
									chmod("/tmp/uu/uuplugin_monitor.sh", 0755);
									_eval(uu_argv, NULL, 0, &pid);
								}
							}
							pclose(fp);
						}
					}
				}
			}
			free(g);
		}
	}
}
#endif

#if defined(RTCONFIG_SOFTCENTER)
void softcenter_trigger(int sig)
{
	pid_t pid;
	char path[100], action[10], sc[]="/jffs/softcenter/bin";
	if(SOFTCENTER_WAN == sig){
		snprintf(path, sizeof(path), "%s/softcenter-wan.sh", sc);
		snprintf(action, sizeof(action), "start");
	} else if (SOFTCENTER_NAT == sig){
		snprintf(path, sizeof(path), "%s/softcenter-net.sh", sc);
		snprintf(action, sizeof(action), "start_nat");
	} else if (SOFTCENTER_MOUNT == sig){
		snprintf(path, sizeof(path), "%s/softcenter-mount.sh", sc);
		snprintf(action, sizeof(action), "start");
	} else if (SOFTCENTER_SERVICES_START == sig){
		snprintf(path, sizeof(path), "%s/softcenter-services.sh", sc);
		snprintf(action, sizeof(action), "start");
	} else if (SOFTCENTER_SERVICES_STOP == sig){//only reboot
		char *eval_argv[] = { "/jffs/softcenter/bin/softcenter-mount.sh", "stop", NULL };
		_eval(eval_argv, NULL, 0, &pid);
		eval_argv[0] = "/jffs/softcenter/bin/softcenter-services.sh";
		_eval(eval_argv, NULL, 0, &pid);
		eval_argv[0] = "/jffs/softcenter/bin/softcenter-wan.sh";
		_eval(eval_argv, NULL, 0, &pid);
		return;
	} else if (SOFTCENTER_UNMOUNT == sig){
		snprintf(path, sizeof(path), "%s/softcenter-unmount.sh", sc);
		snprintf(action, sizeof(action), "unmount");
	} else if (SOFTCENTER_CIFS_MOUNT == sig){
		char *path = nvram_get("sc_cifs_url");
		char *user = nvram_safe_get("sc_cifs_user");
		char *pw = nvram_safe_get("sc_cifs_pw");
		char opt[999] = {0};
		snprintf(opt, sizeof(opt), "username=%s%s%s", *user ? user : "guest", *pw ? ",password=" : "", pw);
		mount(path, "/jffs/softcenter", "cifs", MS_NOATIME|MS_NODIRATIME, opt);
		return;
	} else {
		logmessage("Softcenter", "sig=%d, bug?",sig);
		return;
	}
	char *eval_argv[] = { path, action, NULL };
	_eval(eval_argv, NULL, 0, &pid);
}
#endif

#if defined(RTCONFIG_ENTWARE)
void stop_entware(void)
{
	nvram_set_int("entware_busy", 0);
	nvram_unset("entware_app");
	nvram_unset("entware_action");
	nvram_unset("entware_arg");
}

void init_entware(void)
{
	if(strlen(nvram_get("apps_mounted_path"))){
		logmessage("[Entware]", "Downloadmaster/Aicloud is installed already! Entware can't install!\n");
		return;
	}
	if(nvram_get_int("entware_mount") == 0)
		return;
	/* /opt->/tmp/opt->/jffs/opt->/tmp/mnt/sda/opt */
	unlink("/tmp/opt");
	symlink("/jffs/opt", "/tmp/opt");
#if defined(RTCONFIG_LANTIQ)
	doSystem("mount --bind /tmp/opt /opt");
#endif
	stop_entware();
}

#define ENTWARE_ACT_INSTALL		1
#define ENTWARE_ACT_UPDATE		2
#define ENTWARE_ACT_REMOVE		4
#define ENTWARE_ACT_START		8
#define ENTWARE_ACT_STOP		16
#define	ENTWARE_ACT_MASK (ENTWARE_ACT_INSTALL | ENTWARE_ACT_UPDATE | ENTWARE_ACT_REMOVE)
#define	ENTWARE_ACT_MASK2 (ENTWARE_ACT_START | ENTWARE_ACT_STOP)

void start_entware(void)
{
	char *ent_app, *ent_arg;
	int ent_action;
	char cmd[128], app[32];
	
	if (nvram_get_int("entware_busy") != 1)
		return;
	
	nvram_set_int("entware_busy", 2);
	ent_action = nvram_get_int("entware_action");
	ent_app = nvram_safe_get("entware_app");
	ent_arg = nvram_safe_get("entware_arg");
	
	if (strcmp(ent_app, "entware") == 0)
	{
		if (ent_action & ENTWARE_ACT_INSTALL)
		{
			snprintf(cmd, sizeof(cmd), "wget http://bin.entware.net/%s/installer/%s.sh -O /tmp/doentware.sh", nvram_get("entware_arch"), ent_arg);
			system(cmd);
			system("chmod +x /tmp/doentware.sh");
			system("/tmp/doentware.sh");
			nvram_set("entware_installed", "1");
		}
		else if (ent_action & ENTWARE_ACT_UPDATE)
		{
			system("/opt/bin/opkg update");
			system("/opt/bin/opkg upgrade");
		}
		else
		{
			logmessage("[Entware]", "Unregistered action\n");
		}
	}
	else if (ent_action & ENTWARE_ACT_MASK2)
	{
		snprintf(app, sizeof(app), "/opt/etc/init.d/%s", ent_app);
		if (f_exists(app))
		{
			if (ent_action & ENTWARE_ACT_STOP)
			{
				snprintf(cmd, sizeof(cmd), "%s stop", app);
				system(cmd);
			}
			if (ent_action & ENTWARE_ACT_START)
			{
				snprintf(cmd, sizeof(cmd), "%s start", app);
				system(cmd);
			}
		}
		else
		{
			logmessage("[Entware]", "Nonexistent service\n");
		}
	}
	else if (ent_action & ENTWARE_ACT_MASK)
	{
		if (ent_app)
		{
			if (ent_action & ENTWARE_ACT_REMOVE)
			{
				snprintf(cmd, sizeof(cmd), "/opt/bin/opkg %s remove %s --force-space", ent_arg, ent_app);
				system(cmd);
			}
			if (ent_action & ENTWARE_ACT_UPDATE)
			{
				system("/opt/bin/opkg update");
				snprintf(cmd, sizeof(cmd), "/opt/bin/opkg %s upgrade %s --force-space", ent_arg, ent_app);
				system(cmd);
			}
			if (ent_action & ENTWARE_ACT_INSTALL)
			{
				system("/opt/bin/opkg update");
				snprintf(cmd, sizeof(cmd), "/opt/bin/opkg %s install %s --force-space", ent_arg, ent_app);
				system(cmd);
			}
		}
	}
	else
	{
		logmessage("[Entware]", "Nonexistent app and Unregistered action\n");
	}
	nvram_set_int("entware_busy", 0);
	nvram_unset("entware_app");
	nvram_unset("entware_action");
	nvram_unset("entware_arg");
}
#endif

#if defined(R6800)
#define NETGEAR_BOARD_MAC 0xB0		//6
#define NETGEAR_BOARD_SN 0xB7		//42
#define NETGEAR_BOARD_PIN 0xE1		//12
#define NETGEAR_BOARD_DOMAIN 0xF2	//2
#define NETGEAR_BOARD_PCBA_SN 0xF6	//12
#define NETGEAR_BOARD_SSID 0x106	//20
#define NETGEAR_BOARD_PASSWD 0x120	//64
#define NETGEAR_BOARD_MODULE 0x200	//2
void show_boraddata(void)
{
	FILE *fp = NULL;
	int mtd_part = 0, mtd_size = 0, i;
	char dev_mtd[] = "/dev/mtdblockXXX";
	unsigned char factory_var_buf[256];

	mtd_getinfo("boarddata", &mtd_part, &mtd_size);
	snprintf(dev_mtd, sizeof(dev_mtd), "/dev/mtdblock%d", mtd_part);
	if((fp = fopen(dev_mtd, "rb")) != NULL){
		memset(factory_var_buf, 0, sizeof(factory_var_buf));
		fseek(fp, NETGEAR_BOARD_MAC, SEEK_SET);
		fread(factory_var_buf, 1, 6, fp);
		if(factory_var_buf[0] == 0x0 || factory_var_buf[0] == 0xff)
			printf("mac is invalid, type 'toolbox fix MAC AA1122334455' to fix it, 17 bytes\n");
		else
			printf("mac:%s\n", factory_var_buf);
		memset(factory_var_buf, 0, sizeof(factory_var_buf));
		fseek(fp, NETGEAR_BOARD_SN, SEEK_SET);
		fread(factory_var_buf, 1, 42, fp);
		if(factory_var_buf[0] == 0x0 || factory_var_buf[0] == 0xff)
			printf("sn is invalid, type 'toolbox fix SN 123456789' to fix it\n");
		else{
			for(i = 0; i < 42; i++){
				if(factory_var_buf[i] == 0xff)
					factory_var_buf[i] = 0x0;
			}
			printf("sn:%s\n", factory_var_buf);
		}
		memset(factory_var_buf, 0, sizeof(factory_var_buf));
		fseek(fp, NETGEAR_BOARD_PIN, SEEK_SET);
		fread(factory_var_buf, 1, 12, fp);
		if(factory_var_buf[0] == 0x0 || factory_var_buf[0] == 0xff)
			printf("pin is invalid, type 'toolbox fix PIN 12345678' to fix it, 8 bytes\n");
		else{
			for(i = 0; i < 12; i++){
				if(factory_var_buf[i] == 0xff)
					factory_var_buf[i] = 0x0;
			}
			printf("pin:%s\n", factory_var_buf);
		}
		memset(factory_var_buf, 0, sizeof(factory_var_buf));
		fseek(fp, NETGEAR_BOARD_DOMAIN, SEEK_SET);
		fread(factory_var_buf, 1, 2, fp);
		if(factory_var_buf[0] == 0x0 || factory_var_buf[0] == 0xff)
			printf("domain is invalid, type 'toolbox fix DOMAIN 11' to fix it, 2 bytes\n");
		else{
			for(i = 0; i < 2; i++){
				if(factory_var_buf[i] == 0xff)
					factory_var_buf[i] = 0x0;
			}
			printf("domain:%d%d\n", factory_var_buf[0], factory_var_buf[1]);
		}
		memset(factory_var_buf, 0, sizeof(factory_var_buf));
		fseek(fp, NETGEAR_BOARD_PCBA_SN, SEEK_SET);
		fread(factory_var_buf, 1, 12, fp);
		if(factory_var_buf[0] == 0x0 || factory_var_buf[0] == 0xff)
			printf("PCBA_SN is invalid, type 'toolbox fix PCBA_SN R.BZV8AE0A4D' to fix it, 12 bytes\n");
		else{
			for(i = 0; i < 12; i++){
				if(factory_var_buf[i] == 0xff)
					factory_var_buf[i] = 0x0;
			}
			printf("PCBA_SN:%s\n", factory_var_buf);
		}
		memset(factory_var_buf, 0, sizeof(factory_var_buf));
		fseek(fp, NETGEAR_BOARD_SSID, SEEK_SET);
		fread(factory_var_buf, 1, 20, fp);
		if(factory_var_buf[0] == 0x0 || factory_var_buf[0] == 0xff)
			printf("ssid is invalid, type 'toolbox fix SSID NETGEAR28' to fix it\n");
		else{
			for(i = 0; i < 20; i++){
				if(factory_var_buf[i] == 0xff)
					factory_var_buf[i] = 0x0;
			}
			printf("ssid:%s\n", factory_var_buf);
		}
		memset(factory_var_buf, 0, sizeof(factory_var_buf));
		fseek(fp, NETGEAR_BOARD_PASSWD, SEEK_SET);
		fread(factory_var_buf, 1, 64, fp);
		if(factory_var_buf[0] == 0x0 || factory_var_buf[0] == 0xff)
			printf("password is invalid, type 'toolbox fix PSWD slowjungle499' to fix it\n");
		else{
			for(i = 0; i < 64; i++){
				if(factory_var_buf[i] == 0xff)
					factory_var_buf[i] = 0x0;
			}
			printf("password:%s\n", factory_var_buf);
		}
		memset(factory_var_buf, 0, sizeof(factory_var_buf));
		fseek(fp, NETGEAR_BOARD_MODULE, SEEK_SET);
		fread(factory_var_buf, 1, 2, fp);
		if(factory_var_buf[0] == 0x0 || factory_var_buf[0] == 0xff)
			printf("MODULE is invalid, type 'toolbox fix MODULE 09' to fix it, 2 bytes\n");
		else{
			for(i = 0; i < 2; i++){
				if(factory_var_buf[i] == 0xff)
					factory_var_buf[i] = 0x0;
			}
			printf("MODULE:%s\n", factory_var_buf);
		}
		fclose(fp);
	}else
		printf("can't open boarddata\n");
}

void fix_boraddata(char *key, char *value)
{
	FILE *fp = NULL;
	int mtd_part = 0, mtd_size = 0, i;
	char dev_mtd[] = "/dev/mtdblockXXX";
	unsigned char factory_var_buf[256];

	if(key == NULL || value == NULL){
		printf("key or value is null\n");
		printf("for example:toolbox fix MODULE 09\n");
		return;
	}
	memset(factory_var_buf, 0, sizeof(factory_var_buf));
	mtd_getinfo("boarddata", &mtd_part, &mtd_size);
	snprintf(dev_mtd, sizeof(dev_mtd), "/dev/mtdblock%d", mtd_part);
	if(!strcmp(key, "MAC")){
		int l;
		unsigned char mac_binary[6];
		l = strlen(value);
		if(l != 17 && l != 12){
			printf("The mac is wrong:%s\ntoolbox fix MAC AABB22334455\n", value);
			return -1;
		}
		for(i=0; i < l; i++){
			if(!isxdigit(value[i]) && value[i] != ':' && value[i] != '-'){
				printf("The mac is wrong:%s\ntoolbox fix MAC AABB22334455\n", value);
				return -1;
			}
		}
		ether_atoe(value, mac_binary);
		if((fp = fopen(dev_mtd, "rb")) != NULL){
			fseek(fp, NETGEAR_BOARD_MAC, SEEK_SET);
			fwrite(mac_binary, 1, 6, fp);
			fclose(fp);
			
		}
	}else if(!strcmp(key, "SN")){
		snprintf(factory_var_buf, sizeof(factory_var_buf), "%s", value);
		for(i = 0; i < 42; i++){
			if(factory_var_buf[i] == 0x0)
				factory_var_buf[i] = 0xff;
		}
		if((fp = fopen(dev_mtd, "rb")) != NULL){
			fseek(fp, NETGEAR_BOARD_SN, SEEK_SET);
			fwrite(factory_var_buf, 1, 42, fp);
			fclose(fp);
		}
	}else if(!strcmp(key, "PIN")){
		snprintf(factory_var_buf, sizeof(factory_var_buf), "%s", value);
		for(i = 0; i < 12; i++){
			if(factory_var_buf[i] == 0x0 || i > 7)
				factory_var_buf[i] = 0xff;
		}
		if((fp = fopen(dev_mtd, "rb")) != NULL){
			fseek(fp, NETGEAR_BOARD_PIN, SEEK_SET);
			fwrite(factory_var_buf, 1, 12, fp);
			fclose(fp);
		}
	}else if(!strcmp(key, "DOMAIN")){
		i = atoi(value);
		if(i > 254)
			i = 254;
		factory_var_buf[1] = i;
		if((fp = fopen(dev_mtd, "rb")) != NULL){
			fseek(fp, NETGEAR_BOARD_DOMAIN, SEEK_SET);
			fwrite(factory_var_buf, 1, 2, fp);
			fclose(fp);
		}
	}else if(!strcmp(key, "PCBA_SN")){
		snprintf(factory_var_buf, sizeof(factory_var_buf), "%s", value);
		for(i = 0; i < 12; i++){
			if(factory_var_buf[i] == 0x0)
				factory_var_buf[i] = 0xff;
		}
		if((fp = fopen(dev_mtd, "rb")) != NULL){
			fseek(fp, NETGEAR_BOARD_PCBA_SN, SEEK_SET);
			fwrite(factory_var_buf, 1, 12, fp);
			fclose(fp);
		}
	}else if(!strcmp(key, "SSID")){
		snprintf(factory_var_buf, sizeof(factory_var_buf), "%s", value);
		for(i = 0; i < 20; i++){
			if(factory_var_buf[i] == 0x0)
				factory_var_buf[i] = 0xff;
		}
		if((fp = fopen(dev_mtd, "rb")) != NULL){
			fseek(fp, NETGEAR_BOARD_SSID, SEEK_SET);
			fwrite(factory_var_buf, 1, 20, fp);
			fclose(fp);
		}
	}else if(!strcmp(key, "PSWD")){
		snprintf(factory_var_buf, sizeof(factory_var_buf), "%s", value);
		for(i = 0; i < 64; i++){
			if(factory_var_buf[i] == 0x0)
				factory_var_buf[i] = 0xff;
		}
		if((fp = fopen(dev_mtd, "rb")) != NULL){
			fseek(fp, NETGEAR_BOARD_PASSWD, SEEK_SET);
			fwrite(factory_var_buf, 1, 64, fp);
			fclose(fp);
		}
	}else if(!strcmp(key, "MODULE")){
		snprintf(factory_var_buf, sizeof(factory_var_buf), "%s", value);
		for(i = 0; i < 2; i++){
			if(factory_var_buf[i] == 0x0)
				factory_var_buf[i] = 0xff;
		}
		if((fp = fopen(dev_mtd, "rb")) != NULL){
			fseek(fp, NETGEAR_BOARD_MODULE, SEEK_SET);
			fwrite(factory_var_buf, 1, 2, fp);
			fclose(fp);
		}
	}
}
#endif
