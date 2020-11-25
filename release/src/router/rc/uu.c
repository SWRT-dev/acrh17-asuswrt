#include "rc.h"
#include "merlinr.h"

void start_uu(void)
{
	stop_uu();

	if(getpid()!=1) {
		notify_rc("start_uu");
		return;
	}

	if(nvram_get_int("uu_enable"))
#if defined(R8000P) || defined(RTAC3200) || defined(RTAC3100) || defined(EA6700) || defined(RAX20) || defined(SBRAC1900P) || defined(R7000P) || defined(RMAC2100) || defined(TY6201_BCM) || defined(TY6201_RTK)
		exec_uu_merlinr();
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
