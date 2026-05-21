#!/bin/bash
# ==============================================================================
# İDA Chroot Mount/Unmount Yardımcı Scripti
# ==============================================================================

CHROOT_DIR="/data/local/ubuntu"

mount_chroot() {
    echo "[+] Diskler/Dizinler chroot ortamına bağlanıyor..."
    
    # Mount /proc
    if ! mountpoint -q "$CHROOT_DIR/proc"; then
        mount -t proc proc "$CHROOT_DIR/proc"
    fi
    
    # Mount /sys
    if ! mountpoint -q "$CHROOT_DIR/sys"; then
        mount -t sysfs sysfs "$CHROOT_DIR/sys"
    fi
    
    # Mount /dev
    if ! mountpoint -q "$CHROOT_DIR/dev"; then
        mount -o bind /dev "$CHROOT_DIR/dev"
    fi
    
    # Mount /dev/pts
    if ! mountpoint -q "$CHROOT_DIR/dev/pts"; then
        mount -t devpts devpts "$CHROOT_DIR/dev/pts"
    fi
    
    echo "[✔] Mount işlemleri tamamlandı."
}

unmount_chroot() {
    echo "[*] Disk bağlantıları kesiliyor..."
    
    # Sırayla unmount et (hata almamak için)
    umount -l "$CHROOT_DIR/dev/pts" 2>/dev/null
    umount -l "$CHROOT_DIR/dev" 2>/dev/null
    umount -l "$CHROOT_DIR/sys" 2>/dev/null
    umount -l "$CHROOT_DIR/proc" 2>/dev/null
    
    echo "[✔] Tüm bağlantılar kesildi."
}

case "$1" in
    mount)
        mount_chroot
        ;;
    unmount)
        unmount_chroot
        ;;
    *)
        echo "Kullanım: $0 {mount|unmount}"
        exit 1
        ;;
esac
