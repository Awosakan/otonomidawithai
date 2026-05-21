import os
import csv
import json
import time
import queue
import threading
import logging
import cv2

# Logger Setup
logger = logging.getLogger("IDA_Logger")
logger.setLevel(logging.INFO)

class AsyncLoggerManager:
    """
    Tüm loglama işlemlerini asenkron olarak arka planda yöneten sınıf.
    Ana döngünün disk I/O işlemlerinden dolayı duraksamasını (lag) önler.
    """
    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # Dosya yolları
        self.csv_path = os.path.join(output_dir, "dosya2_telemetri.csv")
        self.costmap_path = os.path.join(output_dir, "dosya3_costmap.jsonl")
        self.video_path = os.path.join(output_dir, "dosya1_kamera.mp4")
        
        # Telemetri ve Costmap kuyrukları
        self.telemetry_queue = queue.Queue()
        self.costmap_queue = queue.Queue()
        
        # Çalışma bayrağı
        self.running = False
        
        # CSV Başlıkları
        self.csv_headers = [
            "Timestamp", "Latitude", "Longitude", "Speed", 
            "Roll", "Pitch", "Heading", 
            "SpeedSetpoint", "HeadingSetpoint"
        ]
        
        # Video yazıcı bileşenleri
        self.video_writer = None
        self.video_queue = queue.Queue(maxsize=100) # OOM önlemek için maks 100 kare (yaklaşık 4 saniye tampon)
        
        # Yazıcı threadleri
        self.writer_thread = None
        self.video_thread = None

    def start(self, frame_width=640, frame_height=480, fps=24.0):
        self.running = True
        
        # CSV dosyasını başlat ve başlıkları yaz
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_headers)
                
        # Video yazıcıyı başlat
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(self.video_path, fourcc, fps, (frame_width, frame_height))
        
        # Threadleri başlat
        self.writer_thread = threading.Thread(target=self._telemetry_writer_loop, daemon=True)
        self.video_thread = threading.Thread(target=self._video_writer_loop, daemon=True)
        
        self.writer_thread.start()
        self.video_thread.start()
        logger.info("Asenkron loglama sistemi başlatıldı.")

    def log_telemetry(self, lat: float, lon: float, speed: float, 
                      roll: float, pitch: float, heading: float, 
                      speed_sp: float, heading_sp: float):
        """
        Telemetri verisini kuyruğa ekler (>= 1 Hz çağrılmalıdır).
        """
        timestamp = time.time()
        data = [timestamp, lat, lon, speed, roll, pitch, heading, speed_sp, heading_sp]
        self.telemetry_queue.put(data)

    def log_costmap(self, grid_data: list, origin_x: float, origin_y: float, 
                    resolution: float, width: int, height: int):
        """
        Costmap / Engel haritasını kuyruğa ekler (>= 1 Hz çağrılmalıdır).
        """
        timestamp = time.time()
        payload = {
            "timestamp": timestamp,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "resolution": resolution,
            "width": width,
            "height": height,
            "grid": grid_data  # 1D veya 2D engel matrisi/koordinat listesi
        }
        self.costmap_queue.put(payload)

    def log_frame(self, frame):
        """
        Görüntü çerçevesini kuyruğa ekler. Çerçeveye asenkron olarak zaman damgası basılacaktır.
        """
        if frame is not None:
            try:
                # Görüntünün kopyasını kuyruğa ekle (referans sorunlarını önlemek için)
                # Kuyruk doluysa otonomi döngüsünü geciktirmemek için put_nowait kullanılır
                self.video_queue.put_nowait((time.time(), frame.copy()))
            except queue.Full:
                # Disk yazma hızı yetişemediğinde hafızanın şişmesini (OOM) engellemek için kareyi atla (drop)
                pass

    def _telemetry_writer_loop(self):
        """
        Telemetri ve Costmap verilerini diske yazan arka plan döngüsü.
        """
        while self.running or not self.telemetry_queue.empty() or not self.costmap_queue.empty():
            try:
                # Telemetri Yazımı
                try:
                    data = self.telemetry_queue.get(timeout=0.1)
                    with open(self.csv_path, mode='a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(data)
                    self.telemetry_queue.task_done()
                except queue.Empty:
                    pass
                
                # Costmap Yazımı
                try:
                    map_data = self.costmap_queue.get(timeout=0.1)
                    with open(self.costmap_path, mode='a') as f:
                        f.write(json.dumps(map_data) + "\n")
                    self.costmap_queue.task_done()
                except queue.Empty:
                    pass
                    
            except Exception as e:
                logger.error(f"Error in telemetry writer loop: {e}")
                time.sleep(0.1)

    def _video_writer_loop(self):
        """
        Görüntülere zaman damgası basan ve MP4 formatında kaydeden arka plan döngüsü.
        """
        while self.running or not self.video_queue.empty():
            try:
                t, frame = self.video_queue.get(timeout=0.1)
                
                # Zaman damgası yazısını oluştur (Örn: 2026-05-20 17:09:46.123)
                local_time = time.localtime(t)
                milliseconds = int((t - int(t)) * 1000)
                time_str = time.strftime("%Y-%m-%d %H:%M:%S", local_time) + f".{milliseconds:03d}"
                
                # Zaman etiketini videonun sol üst köşesine yaz
                # Font, konum, boyut ve renk ayarları
                font = cv2.FONT_HERSHEY_SIMPLEX
                position = (10, 30)
                font_scale = 0.8
                color = (0, 255, 0)  # Yeşil renk
                thickness = 2
                
                # Arka plan için siyah bir kutu çiz (okunabilirliği artırmak için)
                text_size, _ = cv2.getTextSize(time_str, font, font_scale, thickness)
                cv2.rectangle(frame, (5, 5), (15 + text_size[0], 40), (0, 0, 0), -1)
                
                cv2.putText(frame, time_str, position, font, font_scale, color, thickness, cv2.LINE_AA)
                
                # Videoya yaz
                if self.video_writer is not None:
                    self.video_writer.write(frame)
                    
                self.video_queue.task_done()
            except queue.Empty:
                pass
            except Exception as e:
                logger.error(f"Error in video writer loop: {e}")
                time.sleep(0.1)

    def stop(self):
        self.running = False
        
        # Threadlerin bitmesini bekle
        if self.writer_thread is not None:
            self.writer_thread.join()
        if self.video_thread is not None:
            self.video_thread.join()
            
        # Video yazıcıyı kapat
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            
        logger.info("Loglama sistemi durduruldu ve dosyalar kapatıldı.")
