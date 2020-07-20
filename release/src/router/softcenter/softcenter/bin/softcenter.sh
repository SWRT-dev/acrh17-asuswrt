#!/bin/sh

# this scripts used for .asusrouer to start softcenter
source /jffs/softcenter/scripts/base.sh
nvram set jffs2_scripts=1
nvram commit
dbus set softcenter_firmware_version=`nvram get extendno|cut -d "_" -f2|cut -d "-" -f1|cut -c2-6`


