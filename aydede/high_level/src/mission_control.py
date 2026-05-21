import time
import math
import logging
from .protocol import (
    MODE_IDLE, MODE_AUTO, MODE_FAILSAFE, MODE_EMERGENCY,
    pack_phone_commands, pack_heartbeat
)
from .planner import APFPlanner, gps_to_meters

logger = logging.getLogger("IDA_Mission")
logger.setLevel(logging.INFO)

# Durum Tanımları (States)
STATE_IDLE = "IDLE"
STATE_PARKUR1 = "PARKUR1_NOKTA_TAKIP"
STATE_PARKUR2 = "PARKUR2_ENGEL_KACINMA"
STATE_PARKUR3 = "PARKUR3_KAMIKAZE"
STATE_RETURN = "RETURN_HOME"
STATE_FAILSAFE = "FAILSAFE"

class MissionController:
    """
    İDA Otonom Görev Durum Makinesi.
    Algılama, haritalama ve rota planlama bileşenlerini koordine eder, motor komutlarını üretir.
    [Senaryo 10]: 100 Metrelik Sanal Çit (Geofence) koruması içerir.
    """
    def __init__(self, logger_manager, serial_client):
        self.state = STATE_IDLE
        self.logger_manager = logger_manager
        self.serial_client = serial_client
        
        # Rota Planlayıcı
        self.planner = APFPlanner(waypoint_tolerance_m=2.2, nominal_speed_ms=1.3, max_speed_ms=2.0)
        
        # Görev Parametreleri
        self.parkur1_waypoints = []
        self.parkur2_waypoints = []
        self.home_waypoint = None     # Başlangıç noktası
        
        self.current_wp_idx = 0
        self.target_color = "target_red"
        
        # Telemetri ve Güvenlik Durumu
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.battery_voltage = 12.0
        self.gps_lock = 0
        self.last_telemetry_time = 0.0
        
        # Kamikaze Sayaçları
        self.kamikaze_lock_time = 0.0
        self.kamikaze_hit_detected = False
        self.last_step_time = 0.0     # Adım adım zaman farkı hesabı için sayac

    def set_waypoints(self, p1_wps: list, p2_wps: list, home_wp: list):
        self.parkur1_waypoints = p1_wps
        self.parkur2_waypoints = p2_wps
        self.home_waypoint = home_wp

    def update_telemetry(self, telemetry_data: dict):
        self.current_lat = telemetry_data["lat"]
        self.current_lon = telemetry_data["lon"]
        self.current_yaw = telemetry_data["yaw"]
        self.current_speed = telemetry_data["sog"]
        self.battery_voltage = telemetry_data["battery"]
        self.gps_lock = telemetry_data["gps_lock"]
        self.last_telemetry_time = time.time()

    def process_step(self, detections: list, costmap) -> dict:
        """
        Otonomi döngüsünün ana adımı. 24+ Hz hızda çağrılmalıdır.
        """
        now = time.time()
        
        # Dinamik Zaman Farkı (dt) Hesabı (Hata 3 düzeltmesi)
        dt = 0.04166 # Varsayılan 24 FPS zaman farkı
        if self.last_step_time > 0.0:
            dt = now - self.last_step_time
            # Kararsızlık durumlarında dt sınırlandırılır (1ms ile 1.0sn arası)
            dt = max(0.001, min(1.0, dt))
        self.last_step_time = now
        
        # 1. Donanımsal Failsafe Kontrolleri (F-35 Seviyesi Güvenlik)
        if self.state != STATE_IDLE and self.state != STATE_FAILSAFE:
            # [Senaryo 7]: Telemetri İletişim Kesintisi (1.5 saniyeden fazla veri gelmemesi)
            if now - self.last_telemetry_time > 1.5:
                logger.error("Failsafe: STM32 telemetri bağlantısı koptu!")
                self.transition_to(STATE_FAILSAFE)
            # [Senaryo 1]: GPS Kilidi Kaybı
            elif self.gps_lock == 0:
                logger.warning("Failsafe: GPS kilidi kayboldu!")
                self.transition_to(STATE_FAILSAFE)
            # [Senaryo 8]: Batarya Voltajı Kritik Sınırı (Batarya sag koruması stm32'de filtrelenir)
            elif self.battery_voltage < 10.5:
                logger.error(f"Failsafe: Batarya voltajı kritik seviyede: {self.battery_voltage}V!")
                self.transition_to(STATE_FAILSAFE)
            # [Kötü Senaryo 5]: Kamera Merceği Tıkanması/Su Sıçraması Koruması
            elif getattr(self.serial_client, "detector", None) and self.serial_client.detector.camera_blocked:
                logger.error("Failsafe: Kamera merceği kapandı veya aşırı bulanıklaştı!")
                self.transition_to(STATE_FAILSAFE)
                
            # [Kötü Senaryo 10]: Sanal Çit (Geofence) Güvenliği (Predictive Geofence)
            # İDA'nın hızı ve ataleti hesaba katılarak 2 saniye sonra çiti aşacağı öngörülüyorsa
            # veya mevcut mesafe 100 metreyi aştıysa motorları kilitleyip failsafe durumuna geçer.
            if self.home_waypoint:
                dx_h, dy_h = gps_to_meters(self.home_waypoint[0], self.home_waypoint[1], self.current_lat, self.current_lon)
                dist_from_home = math.sqrt(dx_h**2 + dy_h**2)
                predicted_dist = dist_from_home + max(0.0, self.current_speed) * 2.0
                
                if predicted_dist > 100.0:
                    logger.error(f"ACİL DURUM: Tahmini Sanal Çit İhlali! Mevcut: {dist_from_home:.1f}m, 2sn Sonraki: {predicted_dist:.1f}m (Limit 100m). Failsafe aktif edildi.")
                    self.transition_to(STATE_FAILSAFE)

        # Hedef komut değişkenleri
        target_speed = 0.0
        target_heading = self.current_yaw
        reached_all = False
        
        # 2. Durum Makinesi Davranışları
        if self.state == STATE_IDLE:
            target_speed = 0.0
            target_heading = self.current_yaw
            
        elif self.state == STATE_PARKUR1:
            costmap.update(detections)
            # Sürüklenme düzeltmesi için önceki hedef noktayı belirle
            prev_wp = self.home_waypoint if self.current_wp_idx == 0 else self.parkur1_waypoints[self.current_wp_idx - 1]
            
            target_speed, target_heading, self.current_wp_idx, reached_all = self.planner.plan(
                self.current_lat, self.current_lon, self.current_yaw, self.current_speed,
                self.parkur1_waypoints, self.current_wp_idx, costmap, prev_wp, dt
            )
            
            if reached_all:
                logger.info("Parkur 1 başarıyla tamamlandı! Parkur 2'ye geçiliyor.")
                self.current_wp_idx = 0
                costmap.reset()
                self.transition_to(STATE_PARKUR2)
                
        elif self.state == STATE_PARKUR2:
            costmap.update(detections)
            # Sürüklenme düzeltmesi için önceki hedef noktayı belirle
            prev_wp = self.parkur1_waypoints[-1] if self.current_wp_idx == 0 else self.parkur2_waypoints[self.current_wp_idx - 1]
            
            target_speed, target_heading, self.current_wp_idx, reached_all = self.planner.plan(
                self.current_lat, self.current_lon, self.current_yaw, self.current_speed,
                self.parkur2_waypoints, self.current_wp_idx, costmap, prev_wp, dt
            )
            
            if reached_all:
                logger.info("Parkur 2 başarıyla tamamlandı! Parkur 3 Kamikaze görevine geçiliyor.")
                costmap.reset()
                self.transition_to(STATE_PARKUR3)
                
        elif self.state == STATE_PARKUR3:
            target_buoy = None
            for det in detections:
                if det["class"] == self.target_color:
                    target_buoy = det
                    break
                    
            if target_buoy is not None:
                bearing_deg = math.degrees(target_buoy["bearing"])
                distance = target_buoy["distance"]
                target_heading = (self.current_yaw + bearing_deg) % 360.0
                target_speed = 1.2
                
                logger.info(f"Kamikaze hedefi ({self.target_color}) kilitlendi! Mesafe: {distance:.2f}m")
                
                if distance < 0.7:
                    if self.kamikaze_lock_time == 0.0:
                        self.kamikaze_lock_time = now
                    elif now - self.kamikaze_lock_time > 3.0:
                        self.kamikaze_hit_detected = True
            else:
                target_speed = 0.5
                target_heading = self.current_yaw
                
            if self.kamikaze_hit_detected:
                logger.info("Kamikaze hedefi vuruldu! Görev bitti, eve dönülüyor.")
                self.transition_to(STATE_RETURN)
                
        elif self.state == STATE_RETURN:
            if self.home_waypoint:
                prev_wp = self.parkur2_waypoints[-1] if len(self.parkur2_waypoints) > 0 else self.home_waypoint
                target_speed, target_heading, _, reached_all = self.planner.plan(
                    self.current_lat, self.current_lon, self.current_yaw, self.current_speed,
                    [self.home_waypoint], 0, costmap, prev_wp, dt
                )
                if reached_all:
                    logger.info("Başlangıç noktasına geri dönüldü. Motorlar kapatılıyor.")
                    self.transition_to(STATE_IDLE)
            else:
                target_speed = 0.0
                
        elif self.state == STATE_FAILSAFE:
            target_speed = 0.0
            target_heading = self.current_yaw

        # 3. STM32 Kontrol Paketini Gönder (Hız verisi 0.0 - 1.0 motor güç yüzdesi arasına çekilir)
        cmd_mode = 1 
        normalized_speed = max(0.0, min(1.0, target_speed / self.planner.max_speed_ms))
        cmd_packet = pack_phone_commands(cmd_mode, normalized_speed, target_heading)
        self.serial_client.send_packet(cmd_packet)
        
        # 4. Asenkron Dosya Loglama
        self.logger_manager.log_telemetry(
            self.current_lat, self.current_lon, self.current_speed,
            0.0, 0.0, self.current_yaw,
            target_speed, target_heading
        )
        self.logger_manager.log_costmap(
            costmap.get_serialized_grid(), 0.0, 0.0, 
            costmap.resolution, costmap.grid_size, costmap.grid_size
        )
        
        return {
            "state": self.state,
            "target_speed": target_speed,
            "target_heading": target_heading
        }

    def transition_to(self, new_state: str):
        logger.info(f"Durum geçişi: {self.state} -> {new_state}")
        self.state = new_state
        sys_status = 1 if new_state != STATE_FAILSAFE else 2
        sys_mode = MODE_AUTO if "PARKUR" in new_state else MODE_IDLE
        hb_payload = pack_heartbeat(sys_status, sys_mode)
        self.serial_client.send_packet(hb_payload, msg_id=1)
