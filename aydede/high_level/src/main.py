import sys
import os
import time
import threading
import logging
import cv2
import serial
import gc

# Modüllerimizi içe aktaralım
# Python'ın dosyayı doğrudan çalıştırma durumunu desteklemek için path eklemesi yapalım
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.protocol import IDAParser, MSG_STM32_TELEMETRY, unpack_stm32_telemetry, pack_heartbeat, MSG_HEARTBEAT, IDAPacket
from src.telemetry_logger import AsyncLoggerManager
from src.detector import BuoyDetector
from src.costmap import LocalCostmap
from src.mission_control import MissionController, STATE_PARKUR1

# Logger Setup
logging.basicConfig(level=logging.INFO, format="[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s")
logger = logging.getLogger("IDA_Main")

class MockSerial:
    """
    Test bilgisayarlarında STM32 bağlı değilken yazılımın çökmemesini ve
    test edilebilmesini sağlayan sahte seri port sınıfı (Failsafe & Simülasyon).
    """
    def write(self, data):
        pass
    def read(self, size=1):
        time.sleep(0.01)
        return b""
    def close(self):
        pass

class IDANode:
    def __init__(self, serial_port: str = "/dev/ttyACM0", baudrate: int = 115200, 
                 model_path: str = None, video_source=0):
        
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.video_source = video_source
        self.running = False
        
        # 1. Asenkron Loglama Yöneticisi (Şartnamedeki 3 Dosya Çıktısı İçin)
        self.logger_manager = AsyncLoggerManager(output_dir="./ida_logs")
        
        # 2. Seri Port Bağlantısı
        self.ser = None
        self._init_serial()
        
        # 3. Seri Protokol Parser
        self.parser = IDAParser(callback=self.on_packet_received)
        
        # 4. Görev Kontrolcü
        self.mission = MissionController(self.logger_manager, self)
        
        # 5. Duba Dedektörü (Yedekli Model + HSV)
        self.detector = BuoyDetector(model_path=model_path, image_width=640, image_height=480)
        
        # 6. Yerel Engel Haritası (Costmap)
        self.costmap = LocalCostmap(size_m=40.0, resolution=0.25, inflation_radius_m=1.0)
        
        # Görev noktalarını tanımla (Şartnameye uygun örnek rotalar)
        # Parkur 1: Nokta Takip (Örnek Coğrafi Noktalar)
        p1_wps = [
            [40.732501, 29.831201],
            [40.732702, 29.831502],
            [40.732903, 29.831203],
            [40.732704, 29.830904]
        ]
        # Parkur 2: Engelli Nokta Takip
        p2_wps = [
            [40.733100, 29.831500],
            [40.733500, 29.831500]
        ]
        # Ev konumu (Manuel kontrol sonrası botun döneceği yer)
        home_wp = [40.732501, 29.831201]
        
        self.mission.set_waypoints(p1_wps, p2_wps, home_wp)

    def _init_serial(self):
        # [Performans Optimizasyonu]: Linux'ta USB seri gecikmesini 1ms'ye indir (Düşük Gecikmeli Seri Haberleşme)
        if sys.platform.startswith('linux'):
            try:
                dev_name = os.path.basename(self.serial_port)
                latency_path = f"/sys/bus/usb-serial/devices/{dev_name}/latency_timer"
                if os.path.exists(latency_path):
                    with open(latency_path, "w") as f:
                        f.write("1")
                    logger.info(f"Performans Optimizasyonu: USB Seri gecikme süresi {dev_name} için 1ms olarak ayarlandı.")
            except Exception as e:
                logger.warning(f"USB Gecikme süresi otomatik ayarlanamadı (Sudo yetkisi gerekebilir): {e}")

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            logger.info(f"Seri port bağlantısı başarılı: {self.serial_port}")
        except Exception as e:
            logger.error(f"Seri port açılamadı ({e}). Sahte (Mock) seri haberleşme başlatılıyor.")
            self.ser = MockSerial()

    def send_packet(self, payload: bytes, msg_id: int = 0x02):
        """
        Gövdeyi paketleyip seri porttan STM32'ye gönderir.
        """
        packet = IDAPacket(msg_id, payload)
        try:
            self.ser.write(packet.pack())
        except Exception as e:
            logger.error(f"Packet send error: {e}")

    def on_packet_received(self, msg_id: int, payload: bytes):
        """
        Seri porttan geçerli bir paket ayrıştırıldığında çağrılan callback.
        """
        if msg_id == MSG_STM32_TELEMETRY:
            try:
                telemetry = unpack_stm32_telemetry(payload)
                self.mission.update_telemetry(telemetry)
            except Exception as e:
                logger.error(f"Failed to unpack telemetry: {e}")

    def _serial_read_loop(self):
        """
        Seri porttan sürekli veri okuyan ve parser'a besleyen thread.
        Hata durumunda otomatik yeniden bağlanma (reconnect) mantığı içerir.
        """
        while self.running:
            try:
                # Sahte (Mock) seri modunda ise basitçe bekle ve veri beslemeyi sürdür
                if isinstance(self.ser, MockSerial):
                    data = self.ser.read(32)
                    if data:
                        self.parser.feed_data(data)
                    time.sleep(0.04) # ~25Hz
                    continue

                data = self.ser.read(32)
                if data:
                    self.parser.feed_data(data)
            except Exception as e:
                logger.error(f"Seri port okuma hatası: {e}. Yeniden bağlanmaya çalışılıyor...")
                try:
                    self.ser.close()
                except Exception:
                    pass
                time.sleep(1.0)
                # Yeniden bağlanma (reconnect) girişimi
                try:
                    if sys.platform.startswith('linux'):
                        dev_name = os.path.basename(self.serial_port)
                        latency_path = f"/sys/bus/usb-serial/devices/{dev_name}/latency_timer"
                        if os.path.exists(latency_path):
                            with open(latency_path, "w") as f:
                                f.write("1")
                    self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
                    logger.info("Seri port bağlantısı başarıyla yeniden kuruldu.")
                except Exception as recon_err:
                    logger.error(f"Seri port yeniden bağlanma başarısız: {recon_err}")

    def _heartbeat_loop(self):
        """
        STM32'ye 10 Hz frekansta kalp atışı (heartbeat) paketi gönderir (F-35 Failsafe standardı).
        """
        rate = 1.0 / 10.0 # 10 Hz
        while self.running:
            # 1 status (OK), 1 auto mode (aktif göreve göre)
            sys_status = 1
            sys_mode = 1 if "PARKUR" in self.mission.state else 0
            hb_payload = pack_heartbeat(sys_status, sys_mode)
            self.send_packet(hb_payload, msg_id=MSG_HEARTBEAT)
            time.sleep(rate)

    def start(self):
        self.running = True
        
        # [Performans Optimizasyonu]: Ana otonomi thread'ini Snapdragon 845'in Kryo Gold (büyük) çekirdeklerine kilitle
        if hasattr(os, "sched_setaffinity"):
            try:
                # Cores 4-7: Kryo Gold (Büyük performans çekirdekleri)
                os.sched_setaffinity(0, {4, 5, 6, 7})
                logger.info("Performans Optimizasyonu: Ana otonomi iş parçacığı büyük CPU çekirdeklerine (4-7) kilitlendi.")
            except Exception as e:
                logger.warning(f"CPU Çekirdek kilitlemesi başarısız oldu: {e}")
        
        # [Performans Optimizasyonu]: Bellek sızıntılarını ve OOM (Hafıza Tükenmesi) durumlarını önlemek için GC açık tutulur.
        gc.enable()
        gc.collect()
        logger.info("Performans Optimizasyonu: Otomatik Çöp Toplayıcı (GC) bellek sızıntılarını engellemek amacıyla aktif tutuldu.")
        
        # 1. Logları Başlat
        self.logger_manager.start(frame_width=640, frame_height=480, fps=24.0)
        
        # 2. Seri Okuma Threadini Başlat
        self.read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self.read_thread.start()
        
        # 3. Heartbeat Threadini Başlat
        self.hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.hb_thread.start()
        
        # 4. Kamera Başlatılması
        cap = cv2.VideoCapture(self.video_source)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 24) # Şartname minimum 24 FPS
        
        if not cap.isOpened():
            logger.error("Kamera açılamadı! Sistem yedek (simüle) görüntü moduna geçiyor.")
            
        logger.info("İDA otonomi düğümü başlatıldı. Görev tetiklenmesi bekleniyor...")
        
        # Otonomi Döngüsü (24 FPS Kontrol)
        frame_time = 1.0 / 24.0
        
        # Test amaçlı otomatik göreve başlama komutu (Normalde YKİ'den veya RC'den gelir)
        # 3 saniye sonra otomatik Parkur 1'i başlatalım
        start_time = time.time()
        auto_started = False
        
        try:
            while self.running:
                loop_start = time.time()
                
                # Test/Simülasyon başlatma tetiği
                if not auto_started and (loop_start - start_time > 3.0):
                    logger.info("Otonom Görev Tetiklendi!")
                    self.mission.transition_to(STATE_PARKUR1)
                    auto_started = True
                
                ret, frame = cap.read() if cap.isOpened() else (True, self._create_test_frame())
                
                if ret:
                    # Görüntü İşleme ve Duba Tespiti
                    detections = self.detector.detect(frame)
                    
                    # Görev Durum Makinesi Adımı (Görüntü + Harita + Planlama)
                    self.mission.process_step(detections, self.costmap)
                    
                    # Tespitleri ekrana çiz (MP4 video kaydı için)
                    annotated_frame = self.detector.draw_detections(frame, detections)
                    
                    # Log kuyruğuna çerçeveyi asenkron yazılmak üzere ekle
                    self.logger_manager.log_frame(annotated_frame)
                    
                    # Görsel arayüz (Ekranlı testler için - telefonda arka planda çalışırken kapatılabilir)
                    if "DISPLAY" in os.environ:
                        cv2.imshow("IDA Autonomy Monitor", annotated_frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                            
                # 24 FPS kararlılığı için bekleme süresini ayarla
                elapsed = time.time() - loop_start
                sleep_time = frame_time - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            logger.info("Kullanıcı tarafından durduruldu.")
        finally:
            self.stop()
            if cap.isOpened():
                cap.release()
            cv2.destroyAllWindows()

    def _create_test_frame(self):
        """
        Kamera bağlı değilken boş test çerçevesi üreterek programın çalışmasını sağlar.
        """
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # Ortaya yapay bir turuncu duba çizelim (Test amaçlı görsel algılama testi)
        # Turuncu BGR: (0, 165, 255)
        cv2.circle(frame, (320, 240), 20, (0, 165, 255), -1)
        # Bir sarı duba çizelim
        cv2.circle(frame, (150, 200), 15, (0, 255, 255), -1)
        return frame

    def stop(self):
        logger.info("Sistem kapatılıyor, güvenli moda geçiliyor...")
        self.running = False
        
        # Logları kapat
        self.logger_manager.stop()
        
        # Seri portu kapat
        if self.ser is not None:
            self.ser.close()
            
        # [Performans Optimizasyonu]: GC'yi tekrar aç ve elle temizle
        gc.enable()
        gc.collect()
        logger.info("Performans Optimizasyonu: Çöp toplayıcı (GC) yeniden etkinleştirildi ve manuel temizlik yapıldı.")
            
        logger.info("Sistem başarıyla durduruldu.")

if __name__ == "__main__":
    # Örnek çalıştırma parametreleri
    # OnePlus 6 üzerinde çalışırken: python main.py /dev/ttyACM0 115200
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    # Otomatik model algılama
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_model = os.path.join(script_dir, "en_iyi_duba_modeli.onnx")
    model_path = default_model if os.path.exists(default_model) else None
    
    if model_path:
        logger.info(f"Otomatik duba tespit modeli bulundu ve yüklenecek: {model_path}")
    else:
        logger.warning("YOLO ONNX model dosyası ('en_iyi_duba_modeli.onnx') bulunamadı. HSV yedek modunda başlatılıyor.")
        
    node = IDANode(serial_port=port, baudrate=baud, model_path=model_path)
    node.start()
