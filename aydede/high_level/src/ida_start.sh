#!/bin/bash
# 1. Donanım ve haberleşme engellerini temizle, chroot dizinlerini mount et
su -c "sh /data/data/com.termux/files/home/aydede/high_level/src/optimize_system.sh"
su -c "sh /data/data/com.termux/files/home/aydede/phone_assets/chroot_mount.sh mount"

# 2. Chroot Ubuntu'ya gir ve otonomiyi arka planda (detach) başlat
su -c "chroot /data/local/ubuntu /bin/bash -c 'cd /aydede/high_level/src/ && python3 main.py /dev/ttyACM0 115200'" &
