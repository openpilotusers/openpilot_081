#!/usr/bin/bash

if [ ! -f "/data/no_ota_updates" ]; then
    /usr/bin/touch /data/no_ota_updates
fi

ALIAS_CHECK=$(/usr/bin/grep gitpull /system/comma/home/.bash_profile)

if [ "$ALIAS_CHECK" == "" ]; then
    sleep 3
    mount -o remount,rw /system
    echo "alias gi='/data/openpilot/gitpull.sh'" >> /system/comma/home/.bash_profile
    mount -o remount,r /system
fi

if [ ! -f "/system/fonts/opensans_regular.ttf" ]; then
    sleep 3
    mount -o remount,rw /system
  	cp -rf /data/openpilot/selfdrive/assets/fonts/opensans* /system/fonts/
    cp -rf /data/openpilot/selfdrive/assets/addon/font/fonts.xml /system/etc/fonts.xml
    chmod 644 /system/etc/fonts.xml
  	chmod 644 /system/fonts/opensans*
    mount -o remount,r /system
fi

if [ ! -f "/data/KRSet" ]; then
    setprop persist.sys.locale ko-KR
    setprop persist.sys.local ko-KR
    setprop persist.sys.timezone Asia/Seoul
    /usr/bin/touch /data/KRSet
fi

cp -f /data/openpilot/selfdrive/assets/addon/xab /data/openpilot/selfdrive/assets/addon/x2ab
cat /data/openpilot/selfdrive/assets/addon/xa* > /data/openpilot/selfdrive/assets/addon/com.locnall.KimGiSa.apk
cp -f /data/openpilot/selfdrive/assets/addon/com.locnall.KimGiSa.apk /data/openpilot/apk/

if [ ! -f "/data/openpilot/selfdrive/assets/addon/x2ab" ]; then
    cp -f /data/openpilot/selfdrive/assets/addon/xab /data/openpilot/selfdrive/assets/addon/x2ab
    cat /data/openpilot/selfdrive/assets/addon/xa* > /data/openpilot/selfdrive/assets/addon/com.locnall.KimGiSa.apk
    cp -f /data/openpilot/selfdrive/assets/addon/com.locnall.KimGiSa.apk /data/openpilot/apk/
elif [ -f "/data/openpilot/selfdrive/assets/addon/x2ab" ]; then
    DIFF=$(diff -q xab x2ab)
    if [ "$DIFF" != "" ]; then
        cp -f /data/openpilot/selfdrive/assets/addon/xab /data/openpilot/selfdrive/assets/addon/x2ab
        cat /data/openpilot/selfdrive/assets/addon/xa* > /data/openpilot/selfdrive/assets/addon/com.locnall.KimGiSa.apk
        cp -f /data/openpilot/selfdrive/assets/addon/com.locnall.KimGiSa.apk /data/openpilot/apk/
    fi
fi

export PASSIVE="0"
exec ./launch_chffrplus.sh

