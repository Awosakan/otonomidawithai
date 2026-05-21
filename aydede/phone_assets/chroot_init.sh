#!/bin/bash
# ==============================================================================
# İDA Chroot Ubuntu Yapılandırma Scripti (Chroot İçinde Çalışır)
# ==============================================================================

echo "===================================================="
echo "          Chroot Ubuntu Yapılandırması              "
echo "===================================================="

export DEBIAN_FRONTEND=noninteractive

# Çevrimiçi modda paket depolarını güncelle ve gerekli kütüphaneleri kur
echo "[+] Paket listeleri güncelleniyor..."
apt-get update

echo "[+] Temel sistem paketleri yükleniyor (Python3, Pip, Serial)..."
apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-setuptools \
    setserial \
    nano \
    ca-certificates \
    libglib2.0-0

# Python kütüphanelerini kur (Çevrimdışı paketler varsa oradan, yoksa PyPI üzerinden)
if [ -d "/aydede/phone_assets/pip_packages" ] && [ "$(ls -A /aydede/phone_assets/pip_packages 2>/dev/null)" ]; then
    echo "[+] Çevrimdışı Python paketleri yerel dizinden yükleniyor (Çevrimdışı Kurulum)..."
    pip3 install --no-index --find-links="/aydede/phone_assets/pip_packages" numpy pyserial opencv-python-headless
else
    echo "[+] İnternet bağlantısı aktif. Python paketleri PyPI üzerinden yükleniyor..."
    pip3 install numpy pyserial opencv-python-headless
fi

# Gereksiz apt dosyalarını temizle (Yer tasarrufu için)
apt-get clean
rm -rf /var/lib/apt/lists/*

echo "[✔] Chroot Ubuntu yapılandırması başarıyla tamamlandı!"
echo "===================================================="
