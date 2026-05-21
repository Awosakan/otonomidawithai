import math
import logging

logger = logging.getLogger("IDA_Planner")
logger.setLevel(logging.INFO)

def gps_to_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple:
    """
    WGS-84 referans elipsoidi kullanarak iki GPS koordinatı arasındaki mesafeyi metre cinsinden hesaplar.
    dx: Doğu (East) yönünde mesafe (m)
    dy: Kuzey (North) yönünde mesafe (m)
    """
    lat_avg = math.radians((lat1 + lat2) / 2.0)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    
    R = 6378137.0
    dy = dlat * R
    dx = dlon * R * math.cos(lat_avg)
    return dx, dy

class APFPlanner:
    """
    Yapay Potansiyel Alanlar (Artificial Potential Field) rota planlayıcı.
    Hedefe doğru çekici kuvvet (attractive), engellerden zıt yönde itici kuvvet (repulsive) üretir.
    Akıntı sürüklenmesine karşı enine sapma entegral (Cross-Track Error Integral) terimini içerir (Senaryo 4).
    """
    def __init__(self, waypoint_tolerance_m: float = 2.5, 
                 nominal_speed_ms: float = 1.5, 
                 max_speed_ms: float = 2.5):
        
        self.waypoint_tolerance_m = waypoint_tolerance_m
        self.nominal_speed_ms = nominal_speed_ms
        self.max_speed_ms = max_speed_ms
        
        # Çekici ve itici kuvvet katsayıları
        self.K_attractive = 2.0
        
        # --- Akıntı ve Rüzgar Sapma Düzeltmesi (Cross-Track Error) ---
        self.K_cte_i = 0.05       # CTE Entegral kazancı (sürüklenme düzeltmesi)
        self.cte_integrator = 0.0 # Birikmiş sürüklenme hatası
        self.max_cte_i = 1.5     # Maksimum düzeltme doyumu (windup engelleme)

    def plan(self, current_lat: float, current_lon: float, current_yaw_deg: float, current_speed: float,
             waypoints: list, current_wp_idx: int, costmap, prev_wp_gps: list = None, dt: float = 0.04) -> tuple:
        """
        Rota ve hız planlaması yapar.
        """
        if not waypoints or current_wp_idx >= len(waypoints):
            return 0.0, current_yaw_deg, current_wp_idx, True
            
        target_lat, target_lon = waypoints[current_wp_idx]
        
        # 1. Hedef noktaya olan mesafeyi ve bağıl konumu metre cinsinden hesapla
        dx_m, dy_m = gps_to_meters(current_lat, current_lon, target_lat, target_lon)
        dist_to_wp = math.sqrt(dx_m**2 + dy_m**2)
        
        # Noktaya ulaşıldı mı kontrolü
        if dist_to_wp < self.waypoint_tolerance_m:
            logger.info(f"Waypoint {current_wp_idx} ulaşıldı! Bir sonraki noktaya geçiliyor.")
            current_wp_idx += 1
            self.cte_integrator = 0.0 # Yeni hedef noktada sürüklenme entegralini sıfırla
            if current_wp_idx >= len(waypoints):
                return 0.0, current_yaw_deg, current_wp_idx, True
            # Yeni hedef noktayı oku
            target_lat, target_lon = waypoints[current_wp_idx]
            dx_m, dy_m = gps_to_meters(current_lat, current_lon, target_lat, target_lon)
            dist_to_wp = math.sqrt(dx_m**2 + dy_m**2)

        # 2. [Senaryo 4 Önlemi]: Enine Sapma (Cross-Track Error) Hesaplama ve Entegral Düzeltmesi
        # İki nokta arasındaki ideal rota çizgisine olan dikey sapmayı hesaplar.
        cte_offset_x = 0.0
        cte_offset_y = 0.0
        
        if prev_wp_gps:
            # Önceki WP ile hedef WP arasındaki rota hattı vektörü (Metre cinsinden)
            line_dx, line_dy = gps_to_meters(prev_wp_gps[0], prev_wp_gps[1], target_lat, target_lon)
            line_len = math.sqrt(line_dx**2 + line_dy**2)
            
            if line_len > 1.0:
                # İdeal rota hattının birim vektörü
                u_x = line_dx / line_len
                u_y = line_dy / line_len
                
                # Botun önceki WP'ye göre bağıl konumu (Metre)
                boat_dx, boat_dy = gps_to_meters(prev_wp_gps[0], prev_wp_gps[1], current_lat, current_lon)
                
                # Enine sapma mesafesi (Cross-Track Error) - Rota hattına dik olan mesafe
                # Vektörel çarpım (2D cross product): boat_vector x line_unit_vector
                cte = boat_dx * (-u_y) + boat_dy * u_x
                
                # Enine sapma yönünde entegral düzeltme biriktir (dinamik dt kullanılır)
                self.cte_integrator += cte * dt
                # Anti-windup koruması
                self.cte_integrator = max(-self.max_cte_i, min(self.cte_integrator, self.max_cte_i))
                
                # Düzeltme yönü ideal rotaya çekmek için hattın dik birim vektörüdür
                # Sürüklenme yönünün tersine çekici kuvvet uyguluyoruz
                cte_offset_x = (u_y) * (self.cte_integrator * self.K_cte_i)
                cte_offset_y = (-u_x) * (self.cte_integrator * self.K_cte_i)

        # 3. Koordinat Dönüşümü: Doğu-Kuzey (EN) koordinatlarından Bot Gövde Koordinatlarına (Body Frame)
        yaw_rad = math.radians(current_yaw_deg)
        
        # Çekici kuvvet yönüne enine sapma düzeltmesini ekle (Dünya koordinatlarında)
        total_dx = dx_m + cte_offset_x
        total_dy = dy_m + cte_offset_y
        total_dist = math.sqrt(total_dx**2 + total_dy**2)
        
        # Bot gövde eksenindeki ileri (x_body) ve sağ (y_body) çekici yön
        x_body = total_dx * math.sin(yaw_rad) + total_dy * math.cos(yaw_rad)
        y_body = total_dx * math.cos(yaw_rad) - total_dy * math.sin(yaw_rad)
        
        # 4. Çekici Kuvvet (Attractive Force) Hesabı
        if total_dist > 0.1:
            att_x = self.K_attractive * (x_body / total_dist)
            att_y = self.K_attractive * (y_body / total_dist)
        else:
            att_x, att_y = 0.0, 0.0
            
        # 5. İtici Kuvvet (Repulsive Force) Hesabı
        rep_x, rep_y = costmap.get_obstacle_forces()
        
        # 6. Bileşke Kuvvet
        total_force_x = att_x + rep_x
        total_force_y = att_y + rep_y
        
        # 7. Kontrol Komutları Üretimi
        steer_angle_rad = math.atan2(total_force_y, total_force_x)
        target_heading_deg = (current_yaw_deg + math.degrees(steer_angle_rad)) % 360.0
        
        # Hız kontrolü:
        angle_factor = math.cos(steer_angle_rad)
        if angle_factor < 0:
            target_speed = 0.2 
        else:
            target_speed = self.nominal_speed_ms * angle_factor
            if dist_to_wp < 5.0:
                target_speed = min(target_speed, 0.5 + 0.2 * dist_to_wp)
                
        target_speed = max(0.2, min(target_speed, self.max_speed_ms))
        
        # Eğer rotada hiç engel yoksa ve hedefe gidiyorsak tam hıza çıkabiliriz
        if abs(rep_x) < 0.1 and abs(rep_y) < 0.1 and dist_to_wp > 8.0:
            target_speed = self.nominal_speed_ms
            
        return target_speed, target_heading_deg, current_wp_idx, False
