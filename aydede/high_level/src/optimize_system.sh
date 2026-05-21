#!/bin/bash
# ==============================================================================
# İDA Snapdragon 845 (OnePlus 6) Linux Performans ve Kararlılık Optimizasyonu
# ==============================================================================
# Bu betik, otonom yazılımın OnePlus 6 üzerinde en az 24 FPS görüntü işleme hızıyla
# ve gecikmesiz çalışması için gerekli çekirdek ve sürücü ayarlarını yapar.

if [ "$EUID" -ne 0 ]; then
  echo "Lütfen bu optimizasyon betiğini root (sudo) yetkileriyle çalıştırın."
  exit 1
fi

echo "[*] İDA Sistem Optimizasyonu Başlatılıyor..."

# 1. CPU Frekans Regülatörü (Governor) Ayarı
# Snapdragon 845'in 4 adet Kryo Gold (büyük) çekirdeğini maksimum frekansa kilitler.
# Cores 0-3: Kryo Silver (Küçük çekirdekler - Güç Tasarrufu)
# Cores 4-7: Kryo Gold (Büyük çekirdekler - Performans)
echo "[*] CPU Governor ayarları 'performance' moduna alınıyor..."
for cpu in /sys/devices/system/cpu/cpu[0-7]; do
  if [ -f "$cpu/cpufreq/scaling_governor" ]; then
    echo "performance" > "$cpu/cpufreq/scaling_governor"
  fi
done

# 2. USB Seri Port Gecikme Sönümleyici (USB Serial Latency Timer)
# Linux çekirdeği varsayılan olarak USB-Serial arayüzünde verileri 16ms tamponlar.
# Bu gecikme otonom kontrol döngüsünde (25Hz) ciddi sapmalara yol açar.
# Aşağıdaki kod, gecikme süresini 1ms'ye indirerek anlık haberleşme sağlar.
echo "[*] USB Seri port gecikme süreleri (Latency Timer) 1ms'ye düşürülüyor..."
for dev in /sys/bus/usb-serial/devices/ttyUSB* /sys/class/tty/ttyACM*; do
  dev_name=$(basename "$dev")
  # ttyUSB veya ttyACM cihazının latency_timer yolunu bul
  latency_path="/sys/bus/usb-serial/devices/$dev_name/latency_timer"
  if [ -f "$latency_path" ]; then
    echo 1 > "$latency_path"
    echo " -> $dev_name gecikmesi 1ms'ye düşürüldü."
  fi
  
  # ACM/CDC cihazları için (direkt USB'den bağlı STM32) latency ayarı
  if [[ "$dev_name" == ttyACM* ]]; then
    # CDC ACM sürücüleri için alternatif latency düşürme setserial komutu
    if command -v setserial &> /dev/null; then
      setserial "/dev/$dev_name" low_latency
      echo " -> /dev/$dev_name low_latency olarak yapılandırıldı."
    fi
  fi
done

# 3. CPU Sıcaklık Limiti & Thermal Daemon Engelleme
# OnePlus 6 gövdesinde fan olmadığı için thermal-daemon işlemciyi erken yavaşlatır.
# Yarışma süresince (yaklaşık 10-15 dakika) frekans düşüşünü (throttling) ertelemek için:
if systemctl is-active --quiet thermal-engine; then
  echo "[!] UYARI: Termal yavaşlama daemon'ı (thermal-engine) geçici olarak kapatılıyor..."
  systemctl stop thermal-engine
fi

# 4. USB Güç Tasarrufu Ayarlarının Devre Dışı Bırakılması
# Suda titreşim veya dalga anında USB'nin askıya alınmasını (autosuspend) önler.
echo "[*] USB Autosuspend korumaları kapatılıyor..."
for power in /sys/bus/usb/devices/*/power/control; do
  if [ -f "$power" ]; then
    echo "on" > "$power"
  fi
done

# 5. Kablosuz Haberleşme ve Frekans İhlali Engelleme (Teknofest Şartnamesi Madde 4.1)
# İDA üzerindeki dahili Wi-Fi, Bluetooth ve Hücresel modem sinyallerini spektrum analizörüne
# yakalanmamak ve doğrudan elenme riskini ortadan kaldırmak için tamamen kapatır.
echo "[*] Kablosuz haberleşme donanımları (Wi-Fi, Bluetooth, LTE) kapatılıyor..."
if command -v rfkill &> /dev/null; then
  rfkill block wifi
  rfkill block bluetooth
  rfkill block wwan
  echo " -> rfkill ile tüm kablosuz vericiler kapatıldı (Uçak Modu)."
else
  # Alternatif olarak network manager veya ip link üzerinden kapatmayı dene
  nmcli radio wifi off &>/dev/null
  nmcli radio wwan off &>/dev/null
  ip link set wlan0 down &>/dev/null
  echo " -> Alternatif yöntemle Wi-Fi/LTE kapatılmaya çalışıldı."
fi

echo "[+] Performans optimizasyonu tamamlandı. Yazılım çalışmaya hazır."
