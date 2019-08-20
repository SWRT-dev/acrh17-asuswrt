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
#include <arpa/inet.h>
#include <httpd.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <sys/wait.h>
#include <dirent.h>
#include <sys/ioctl.h>

#include <typedefs.h>
#include <bcmutils.h>
#include <shutils.h>
#include <bcmnvram.h>
#include <bcmnvram_f.h>
#include <common.h>
#include <shared.h>
#include <rtstate.h>

#include <wlioctl.h>

#include <wlutils.h>
#include <sys/sysinfo.h>
#include <sys/statvfs.h>
#include <linux/version.h>

#ifdef RTCONFIG_USB
#include <disk_io_tools.h>
#include <disk_initial.h>
#include <disk_share.h>

#else
typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;
#endif

#include "sysinfo.h"

unsigned int get_phy_temperature(int radio);


#define MBYTES 1024 / 1024
#define KBYTES 1024

#define SI_WL_QUERY_ASSOC 1
#define SI_WL_QUERY_AUTHE 2
#define SI_WL_QUERY_AUTHO 3


int ej_show_sysinfo(int eid, webs_t wp, int argc, char_t ** argv)
{
	char *type;
	char result[2048];
	int retval = 0;
	struct sysinfo sys;
	char *tmp;

	strcpy(result,"None");

	if (ejArgs(argc, argv, "%s", &type) < 1) {
		websError(wp, 400, "Insufficient args\n");
		return retval;
	}

	if (type) {
		if (strcmp(type,"cpu.model") == 0) {
			char *buffer = read_whole_file("/proc/cpuinfo");

			if (buffer) {
				char model[64];

				tmp = strstr(buffer, ":");

				if (tmp)
					sscanf(tmp, ":%[^\n]", model);
				else {
#if defined(RTCONFIG_LANTIQ)
					strcpy(model, "GRX500 rev 1.2");
#elif  defined(RTCONFIG_QCA)
					strcpy(model, "Unknow");
#endif
				}

				strcpy(result, model);
				free(buffer);
			}

		} else if(strcmp(type,"cpu.freq") == 0) {

			char freq[64];
			char *buffer;

			buffer = read_whole_file("/proc/cpuinfo");

			if (buffer) {
				tmp = strstr(buffer, "cpu MHz			:");
				if (tmp)
					sscanf(tmp, "cpu MHz			:%s", freq);
				else {
#if defined(RTCONFIG_LANTIQ)
					strcpy(freq, "800");
#elif  defined(RTCONFIG_QCA)
					strcpy(freq, "700");
#endif
				}
				strcpy(result, freq);
				free(buffer);
			}
		} else if(strcmp(type,"memory.total") == 0) {
			sysinfo(&sys);
			sprintf(result,"%.2f",(sys.totalram/(float)MBYTES));
		} else if(strcmp(type,"memory.free") == 0) {
			sysinfo(&sys);
			sprintf(result,"%.2f",(sys.freeram/(float)MBYTES));
		} else if(strcmp(type,"memory.buffer") == 0) {
			sysinfo(&sys);
			sprintf(result,"%.2f",(sys.bufferram/(float)MBYTES));
		} else if(strcmp(type,"memory.swap.total") == 0) {
			sysinfo(&sys);
			sprintf(result,"%.2f",(sys.totalswap/(float)MBYTES));
		} else if(strcmp(type,"memory.swap.used") == 0) {
			sysinfo(&sys);
			sprintf(result,"%.2f",((sys.totalswap - sys.freeswap) / (float)MBYTES));
		} else if(strcmp(type,"memory.cache") == 0) {
			int size = 0;
			char *buffer = read_whole_file("/proc/meminfo");

			if (buffer) {
				tmp = strstr(buffer, "Cached");
				if (tmp)
					sscanf(tmp, "Cached:            %d kB\n", &size);
				free(buffer);
				sprintf(result,"%.2f", (size / (float)KBYTES));
			} else {
				strcpy(result,"??");
			}
		} else if(strcmp(type,"cpu.load.1") == 0) {
			sysinfo(&sys);
			sprintf(result,"%.2f",(sys.loads[0] / (float)(1<<SI_LOAD_SHIFT)));
		} else if(strcmp(type,"cpu.load.5") == 0) {
			sysinfo(&sys);
			sprintf(result,"%.2f",(sys.loads[1] / (float)(1<<SI_LOAD_SHIFT)));
		} else if(strcmp(type,"cpu.load.15") == 0) {
			sysinfo(&sys);
			sprintf(result,"%.2f",(sys.loads[2] / (float)(1<<SI_LOAD_SHIFT)));
		} else if(strcmp(type,"nvram.total") == 0) {
			sprintf(result,"%d",NVRAM_SPACE);
		} else if(strcmp(type,"nvram.used") == 0) {
			char *buf;
			int size = 0;

			buf = malloc(NVRAM_SPACE);
			if (buf) {
				nvram_getall(buf, NVRAM_SPACE);
				tmp = buf;
				while (*tmp) tmp += strlen(tmp) +1;

				size = sizeof(struct nvram_header) + (int) tmp - (int) buf;
				free(buf);
			}
			sprintf(result,"%d",size);

		} else if(strcmp(type,"jffs.usage") == 0) {
			struct statvfs fiData;

			char *mount_info = read_whole_file("/proc/mounts");

			if ((mount_info) && (strstr(mount_info, "/jffs")) && (statvfs("/jffs",&fiData) == 0 )) {
				sprintf(result,"%.2f / %.2f MB",((fiData.f_blocks-fiData.f_bfree) * fiData.f_frsize / (float)MBYTES) ,(fiData.f_blocks * fiData.f_frsize / (float)MBYTES));
			} else {
				strcpy(result,"<i>Unmounted</i>");
			}

			if (mount_info) free(mount_info);

		} else if(strncmp(type,"temperature",11) == 0) {

			unsigned int temperature;
			int radio;

			if (sscanf(type,"temperature.%d", &radio) != 1)
				temperature = 0;
			else
			{
				temperature = get_phy_temperature(radio);
			}
			if (temperature == 0)
				strcpy(result,"<i>disabled</i>");
			else
				sprintf(result,"%u&deg;C", temperature);

		} else if(strcmp(type,"conn.total") == 0) {
			FILE* fp;

			fp = fopen ("/proc/sys/net/ipv4/netfilter/ip_conntrack_count", "r");
			if (fp) {
				if (fgets(result, sizeof(result), fp) == NULL)
					strcpy(result, "error");
				else
					result[strcspn(result, "\n")] = 0;
				fclose(fp);
			}
		} else if(strcmp(type,"conn.active") == 0) {
			char buf[256];
			FILE* fp;
			unsigned int established = 0;

			fp = fopen("/proc/net/nf_conntrack", "r");
			if (fp) {
				while (fgets(buf, sizeof(buf), fp) != NULL) {
				if (strstr(buf,"ESTABLISHED") || ((strstr(buf,"udp")) && (strstr(buf,"ASSURED"))))
					established++;
				}
				fclose(fp);
			}
			sprintf(result,"%u",established);

		} else if(strcmp(type,"conn.max") == 0) {
			FILE* fp;

			fp = fopen ("/proc/sys/net/ipv4/netfilter/ip_conntrack_max", "r");
			if (fp) {
				if (fgets(result, sizeof(result), fp) == NULL)
					strcpy(result, "error");
				else
					result[strcspn(result, "\n")] = 0;
				fclose(fp);
			}
		} else if(strncmp(type,"conn.wifi",9) == 0) {
			strcpy(result,"<i>off</i>");
/*			int count, radio;
			char command[10];

			sscanf(type,"conn.wifi.%d.%9s", &radio, command);

			if (strcmp(command,"autho") == 0) {
				count = get_wifi_clients(radio,SI_WL_QUERY_AUTHO);
			} else if (strcmp(command,"authe") == 0) {
				count = get_wifi_clients(radio,SI_WL_QUERY_AUTHE);
			} else if (strcmp(command,"assoc") == 0) {
				count = get_wifi_clients(radio,SI_WL_QUERY_ASSOC);
			} else {
				count = 0;
			}
			if (count == -1)
				strcpy(result,"<i>off</i>");
			else
				sprintf(result,"%d",count);
*/
		} else if(strcmp(type,"driver_version") == 0 ) {
#if defined(RTCONFIG_LANTIQ)
			char *buffer = read_whole_file("/rom/opt/lantiq/etc/wave_components.ver");

			if (buffer) {
				tmp = strstr(buffer, "wave_release_minor=");
				if (tmp)
					sscanf(tmp, "wave_release_minor=%s", result);
				else
					strcpy(result,"K3C merlin");

				free(buffer);
			}
			unlink("/rom/opt/lantiq/etc/wave_components.ver");
#elif  defined(RTCONFIG_QCA)
					strcpy(result,"Unknow");
#endif
		} else if(strncmp(type,"pid",3) ==0 ) {
			char service[32];
			sscanf(type, "pid.%31s", service);

			if (strlen(service))
				sprintf(result, "%d", pidof(service));

		} else if(strncmp(type, "vpnip",5) == 0 ) {
			int instance = 1;
			int fd;
			struct ifreq ifr;
			char buf[18];

			strcpy(result, "0.0.0.0");

			fd = socket(AF_INET, SOCK_DGRAM, 0);
			if (fd) {
				ifr.ifr_addr.sa_family = AF_INET;
				sscanf(type,"vpnip.%d", &instance);
				snprintf(ifr.ifr_name, IFNAMSIZ - 1, "tun1%d", instance);
				if (ioctl(fd, SIOCGIFADDR, &ifr) == 0) {
					strlcpy(result, inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr), sizeof result);

					snprintf(buf, sizeof buf, "vpn_client%d_rip", instance);
					if (!strlen(nvram_safe_get(buf))) {
						sprintf(buf, "%d", instance);
						eval("/usr/sbin/gettunnelip.sh", buf);
					}
				}
				close(fd);
			}

		} else if(strncmp(type,"vpnstatus",9) == 0 ) {
			int num = 0;
			char service[10], buf[256];

			sscanf(type,"vpnstatus.%9[^.].%d", service, &num);

			if ((strlen(service)) && (num > 0) )
			{
				snprintf(buf, sizeof(buf), "vpn%s%d", service, num);
				if (pidof(buf) > 0) {

					// Read the status file and repeat it verbatim to the caller
					sprintf(buf,"/etc/openvpn/%s%d/status", service, num);

					// Give it some time if it doesn't exist yet
					if (!check_if_file_exist(buf))
					{
						sleep(5);
					}

					char *buffer = read_whole_file(buf);
					if (buffer)
					{
						replace_char(buffer, '\n', '>');
						strlcpy(result, buffer, sizeof(result));
						free(buffer);
					}
				}
			}

		} else if(strcmp(type,"ethernet") == 0 ) {
			strcpy(result,"<i>off</i>");
/*			int len, j;

			system("/usr/sbin/robocfg showports >/tmp/output.txt");

			char *buffer = read_whole_file("/tmp/output.txt");
			if (buffer) {
				len = strlen(buffer);

				for (j=0; (j < len); j++) {
					if (buffer[j] == '\n') buffer[j] = '>';
				}
#ifdef RTCONFIG_QTN
				j = GetPhyStatus_qtn();
				snprintf(result, sizeof result, (j > 0 ? "%sPort 10: %dFD enabled stp: none vlan: 1 jumbo: off mac: 00:00:00:00:00:00>" :
							 "%sPort 10: DOWN enabled stp: none vlan: 1 jumbo: off mac: 00:00:00:00:00:00>"),
							  buffer, j);
#else
                                strlcpy(result, buffer, sizeof result);
#endif
                                free(buffer);

			}
			unlink("/tmp/output.txt");*/
//		} else if(strcmp(type,"wifitxpower") == 0) {
//			unsigned int txpower;
		} else {
			strcpy(result,"Not implemented");
		}

	}

	retval += websWrite(wp, result);
	return retval;
}


unsigned int get_phy_temperature(int radio)
{
#if defined(RTCONFIG_LANTIQ)
	int temp = 0, retval = 0;
	char *tmp;
	if (radio == 2) {
		system("iwpriv wlan0 gTemperature >/tmp/output.txt");
	} else if (radio == 5) {
		system("iwpriv wlan2 gTemperature >/tmp/output.txt");
	} else if (radio == 7) {
		system("cut -c25-26 /sys/kernel/debug/ltq_tempsensor/allsensors >/tmp/output.txt");
	} else {
		return retval;
	}
	char *buffer = read_whole_file("/tmp/output.txt");

	if (buffer) {
		if (radio == 7) {
			sscanf(buffer, "%d", &temp);
			free(buffer);
			retval = temp;
		} else {
			tmp = strstr(buffer, "gTemperature:");
			if (tmp) {
				sscanf(tmp, "gTemperature:%d", &temp);
				free(buffer);			
				retval = temp;
			} else {
				free(buffer);
				retval = 0;
			}
		}
	} else { retval = 99; }
	unlink("/tmp/output.txt");
	return retval;
#elif  defined(RTCONFIG_QCA)
    return 99;
#endif
}

int ej_kool_info(int eid, webs_t wp, int argc, char_t ** argv)
{
	char *type;
	char result[2048];
	int retval = 0;
	strcpy(result,"None");

	if (ejArgs(argc, argv, "%s", &type) < 1) {
		websError(wp, 400, "Insufficient args\n");
		return retval;
	}
	if (type) {
		if (strcmp(type,"version") == 0) {
			if (nvram_get_int("adblock_mode") == 0) {
				char pathad[2048];
				sprintf(pathad,"/tmp/mnt/%s/adbyby/adbyby", nvram_get("k3c_disk"));
				//if (!d_exists("/tmp/mnt/%s/adbyby"))
				if((access(pathad, F_OK)) != -1) {
					eval("/tmp/mnt/%s/adbyby/adbyby", "--version|awk '{print $3}'|awk -F '(' '{print $1}' > /tmp/kool.ver 2>/dev/null", nvram_get("k3c_disk"));
					char *buffer = read_whole_file("/tmp/kool.ver");

					if (buffer) {
						sscanf(buffer, "%s", result);
						free(buffer);
					} else {
						strcpy(result,"None");
					}
					unlink("/tmp/kool.ver");
				}
				else
					strcpy(result,"None");
			} else {
				if((access("/tmp/koolproxy/koolproxy", F_OK) != -1)) {
					eval("/tmp/koolproxy/koolproxy", "-v > /tmp/kool.ver 2>/dev/null");
					char *buffer = read_whole_file("/tmp/kool.ver");

					if (buffer) {
						sscanf(buffer, "%s", result);
						free(buffer);
					} else {
						strcpy(result,"None");
					}
					unlink("/tmp/kool.ver");
				}
				else
					strcpy(result,"None");
			}
		}
	}
	retval += websWrite(wp, result);
	return retval;
}


