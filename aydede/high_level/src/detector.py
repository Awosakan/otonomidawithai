import cv2
import numpy as np
import logging
import math

logger = logging.getLogger("IDA_Detector")
logger.setLevel(logging.INFO)

class BuoyDetector:
    """
    Duba algılama ve konum kestirim sınıfı.
    YOLO ONNX ve HSV Renk Eşikleme olmak üzere çift kanallı yedekli çalışır (F-35 Failsafe standardı).
    """
    def __init__(self, model_path: str = None, 
                 image_width: int = 640, 
                 image_height: int = 480,
                 hfov: float = 80.0,  # Derece cinsinden Yatay Görüş Açısı (Horizontal Field of View)
                 conf_threshold: float = 0.35,
                 nms_threshold: float = 0.4):
        
        self.image_width = image_width
        self.image_height = image_height
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        
        # Kamera Parametreleri (Odak Uzaklığı - Focal Length Hesaplama)
        self.hfov_rad = math.radians(hfov)
        self.focal_length_px = (self.image_width / 2.0) / math.tan(self.hfov_rad / 2.0)
        
        # Fiziksel Duba Boyutları (Şartnameye göre çapı 30 cm = 0.3 metre)
        self.BUOY_REAL_WIDTH_M = 0.30 
        
        # Sınıflar (YOLO ve Renk Filtrelemede Ortak)
        self.classes = {
            0: "orange_gate",      # Şartname Turuncu Duba (RAL 2003)
            1: "yellow_obstacle",  # Şartname Sarı Duba (RAL 1026)
            2: "target_red",       # Parkur 3 Kamikaze Hedef Kırmızı
            3: "target_green",     # Parkur 3 Kamikaze Hedef Yeşil
            4: "target_blue"       # Parkur 3 Kamikaze Hedef Mavi
        }
        
        # Model Yükleme
        self.net = None
        self.use_fallback = True
        
        if model_path:
            try:
                # OpenCV DNN ile ONNX modelini yükle
                self.net = cv2.dnn.readNetFromONNX(model_path)
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                
                # [GPU Hızlandırma Optimizasyonu]: Adreno 630 GPU üzerinde OpenCL veya Vulkan ile çalıştır
                try:
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_OPENCL)
                    logger.info("Performans Optimizasyonu: YOLO Çıkarımı GPU (OpenCL) üzerine yönlendirildi.")
                except Exception:
                    try:
                        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_VULKAN)
                        logger.info("Performans Optimizasyonu: YOLO Çıkarımı GPU (Vulkan) üzerine yönlendirildi.")
                    except Exception:
                        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                        logger.info("Performans Optimizasyonu: GPU hedef atanamadı, çıkarım CPU (ARM NEON) üzerinde yapılacak.")
                
                self.use_fallback = False
                logger.info(f"YOLO ONNX modeli başarıyla yüklendi: {model_path}")
            except Exception as e:
                logger.error(f"YOLO modeli yüklenemedi: {e}. HSV Renk Filtreleme moduna geçiliyor.")
                self.use_fallback = True
        else:
            logger.info("Model dosyası belirtilmedi. HSV Renk Filtreleme modunda çalışılıyor.")
            self.use_fallback = True

        # --- Gelişmiş Emniyet Filtreleri (Suda 10 Kötü Senaryo Önlemleri) ---
        # Senaryo 5 (Su Sıçraması / Kamera Kapanması) Kontrolü
        self.frame_count = 0
        self.camera_blocked = False
        
        # Senaryo 6 (Geçici Duba Kaybı / Yanlış Pozitif Engelleme): Zamansal Filtre (Temporal Filter)
        # Her bir duba sınıfı için son konumu ve kaç karedir aralıksız görüldüğünü tutar.
        # Format: {class_name: {"history": [(dist, bearing), ...], "confirmed": bool}}
        self.detection_tracks = {}

    def check_lens_obstruction(self, frame) -> bool:
        """
        [Kötü Senaryo 5]: Merceğe su gelmesi veya kameranın tamamen kapanması durumunu kontrol eder.
        Görüntüdeki renk varyansını (kontrastı) ve ortalama parlaklığı ölçer.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_brightness = np.mean(gray)
        std_deviation = np.std(gray)
        
        # Lens kapandıysa veya su damlasından dolayı görüntü aşırı bulanıklaştıysa standart sapma çok düşer.
        if std_deviation < 5.0 or mean_brightness < 8.0:
            if not self.camera_blocked:
                logger.warning(f"ACİL DURUM: Kamera merceği kapandı veya su sıçradı! Kontrast: {std_deviation:.1f}, Parlaklık: {mean_brightness:.1f}")
                self.camera_blocked = True
            return True
            
        self.camera_blocked = False
        return False

    def temporal_filter(self, raw_detections: list) -> list:
        """
        [Kötü Senaryo 6]: Kamera gürültüsü ve su sıçramalarından ötürü anlık sahte duba tespitlerini filtreler.
        Bir dubanın 'kesin' kabul edilmesi için son 5 karede en az 3 kez benzer koordinatlarda görülmesi gerekir.
        """
        confirmed_detections = []
        new_tracks = {}

        for det in raw_detections:
            cls = det["class"]
            dist = det["distance"]
            bearing = det["bearing"]
            
            # Eski izler arasında en yakın olanı bul
            matched = False
            if cls in self.detection_tracks:
                track = self.detection_tracks[cls]
                last_dist, last_bearing = track["history"][-1]
                
                # Mesafe ve açı farkı makul sınırlar içindeyse aynı duba kabul et
                dist_diff = abs(dist - last_dist)
                bearing_diff = abs(bearing - last_bearing)
                
                if dist_diff < 3.0 and bearing_diff < math.radians(15.0):
                    history = track["history"] + [(dist, bearing)]
                    # Son 5 kareyi tut
                    if len(history) > 5:
                        history.pop(0)
                        
                    hits = len(history)
                    confirmed = hits >= 3 # 5 karede en az 3 kez görüldüyse doğrula
                    
                    new_tracks[cls] = {"history": history, "confirmed": confirmed}
                    matched = True
                    
                    if confirmed:
                        confirmed_detections.append(det)
            
            if not matched:
                # Yeni bir iz başlat (İlk karede onaylanmamış kabul et)
                new_tracks[cls] = {"history": [(dist, bearing)], "confirmed": False}

        self.detection_tracks = new_tracks
        return confirmed_detections

    def detect(self, frame) -> list:
        """
        Görüntüde duba algılar ve açı/mesafe hesaplar. Emniyet filtrelerinden geçirir.
        """
        if frame is None:
            return []
            
        self.frame_count += 1
        
        # [Senaryo 5 Önlemi] Lens tıkanıklık kontrolü
        if self.check_lens_obstruction(frame):
            return []
            
        if self.use_fallback:
            raw_dets = self._detect_hsv(frame)
        else:
            raw_dets = self._detect_yolo(frame)
            
        # [Senaryo 6 Önlemi] Zamansal doğrulama filtresi uygula
        return self.temporal_filter(raw_dets)

    def _detect_yolo(self, frame) -> list:
        """
        OpenCV DNN ile YOLOv8 ONNX modeli kullanarak çıkarım yapar.
        """
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (640, 640), swapRB=True, crop=False)
        self.net.setInput(blob)
        
        outputs = self.net.forward()
        outputs = np.transpose(outputs[0], (1, 0))
        
        boxes = []
        confidences = []
        class_ids = []
        
        for row in outputs:
            classes_scores = row[4:]
            class_id = np.argmax(classes_scores)
            confidence = classes_scores[class_id]
            
            if confidence >= self.conf_threshold:
                x_center, y_center, w, h = row[0:4]
                x_factor = self.image_width / 640.0
                y_factor = self.image_height / 640.0
                
                x = int((x_center - w/2) * x_factor)
                y = int((y_center - h/2) * y_factor)
                width = int(w * x_factor)
                height = int(h * y_factor)
                
                boxes.append([x, y, width, height])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)
        
        detections = []
        for i in indices:
            idx = i[0] if isinstance(i, (list, np.ndarray)) else i
            box = boxes[idx]
            class_id = class_ids[idx]
            conf = confidences[idx]
            
            distance, bearing = self.estimate_distance_and_bearing(box)
            
            detections.append({
                "class": self.classes.get(class_id, "unknown"),
                "bbox": box,
                "confidence": conf,
                "distance": distance,
                "bearing": bearing
            })
            
        return detections

    def _detect_hsv(self, frame) -> list:
        """
        Yedek algılama mekanizması: HSV renk eşikleme ve kontur analizi.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detections = []
        
        color_ranges = {
            "orange_gate": [((0, 120, 100), (15, 255, 255)), ((165, 120, 100), (180, 255, 255))],
            "yellow_obstacle": [((20, 100, 100), (35, 255, 255))],
            "target_red": [((0, 150, 80), (10, 255, 255)), ((170, 150, 80), (180, 255, 255))],
            "target_green": [((40, 80, 80), (80, 255, 255))],
            "target_blue": [((100, 120, 80), (130, 255, 255))]
        }
        
        for name, ranges in color_ranges.items():
            mask = None
            for lower, upper in ranges:
                m = cv2.inRange(hsv, np.array(lower), np.array(upper))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
                
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 150:
                    continue
                    
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / float(h)
                if aspect_ratio < 0.2 or aspect_ratio > 1.8:
                    continue
                    
                box = [x, y, w, h]
                distance, bearing = self.estimate_distance_and_bearing(box)
                
                detections.append({
                    "class": name,
                    "bbox": box,
                    "confidence": 0.85,
                    "distance": distance,
                    "bearing": bearing
                })
                
        return detections

    def estimate_distance_and_bearing(self, bbox: list) -> tuple:
        x, y, w, h = bbox
        w_px = max(1, w)
        distance = (self.focal_length_px * self.BUOY_REAL_WIDTH_M) / w_px
        box_center_x = x + w / 2.0
        offset_x = box_center_x - (self.image_width / 2.0)
        bearing = math.atan2(offset_x, self.focal_length_px)
        return distance, bearing

    def draw_detections(self, frame, detections: list):
        for det in detections:
            x, y, w, h = det["bbox"]
            label = f"{det['class']} ({det['confidence']:.2f})"
            dist_label = f"Dist: {det['distance']:.2f}m, Ang: {math.degrees(det['bearing']):.1f}deg"
            
            if "orange" in det["class"]:
                color = (0, 165, 255)
            elif "yellow" in det["class"]:
                color = (0, 255, 255)
            elif "red" in det["class"]:
                color = (0, 0, 255)
            elif "green" in det["class"]:
                color = (0, 255, 0)
            elif "blue" in det["class"]:
                color = (255, 0, 0)
            else:
                color = (255, 255, 255)
                
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, label, (x, y - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.putText(frame, dist_label, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return frame
