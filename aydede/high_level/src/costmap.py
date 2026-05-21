import numpy as np
import math
import logging

logger = logging.getLogger("IDA_Costmap")
logger.setLevel(logging.INFO)

class LocalCostmap:
    """
    İDA merkezli çift katmanlı yerel engel haritası (Occupancy Grid / Costmap).
    Turuncu kapı dubaları ve Sarı engel dubaları için iki ayrı katman tutar.
    - Kapı Dubaları (Orange Gates): Kapının tam ortasından geçmek için simetrik itme uygular.
    - Sarı Engeller (Yellow Obstacles): COLREGs kuralları gereği sağdan (sancak) geçmek için asimetrik itme uygular.
    """
    def __init__(self, size_m: float = 40.0, resolution: float = 0.25, inflation_radius_m: float = 1.0):
        self.size_m = size_m
        self.resolution = resolution
        self.grid_size = int(size_m / resolution)
        self.center_idx = self.grid_size // 2
        
        # Çift Katmanlı Harita Izgarası
        self.grid_gates = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        self.grid_obstacles = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        
        # Birleşik görsel ızgara (Loglama ve görselleştirme için)
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        
        self.inflation_radius_cells = int(inflation_radius_m / resolution)
        self.decay_factor = 0.85
        self.min_cost_threshold = 15

    def update(self, detections: list):
        """
        Gelen yeni duba tespitlerinin sınıfına göre ilgili katmanı günceller.
        """
        # 1. Eski harita katmanlarını sönümle (Decay)
        self.grid_gates = (self.grid_gates * self.decay_factor).astype(np.uint8)
        self.grid_gates[self.grid_gates < self.min_cost_threshold] = 0
        
        self.grid_obstacles = (self.grid_obstacles * self.decay_factor).astype(np.uint8)
        self.grid_obstacles[self.grid_obstacles < self.min_cost_threshold] = 0
        
        # 2. Yeni tespitleri sınıflarına göre dağıt
        for det in detections:
            cls = det["class"]
            distance = det["distance"]
            bearing = det["bearing"]
            
            x_rel = distance * math.cos(bearing)
            y_rel = distance * math.sin(bearing)
            
            row = self.center_idx - int(x_rel / self.resolution)
            col = self.center_idx + int(y_rel / self.resolution)
            
            if 0 <= row < self.grid_size and 0 <= col < self.grid_size:
                if cls == "orange_gate":
                    self.grid_gates[row, col] = 100
                    self._inflate_obstacle(self.grid_gates, row, col)
                else:
                    # Sarı dubalar ve diğer hedefler engel kabul edilir
                    self.grid_obstacles[row, col] = 100
                    self._inflate_obstacle(self.grid_obstacles, row, col)
                    
        # Loglama ve uyumluluk için birleşik haritayı güncelle
        self.grid = np.maximum(self.grid_gates, self.grid_obstacles)

    def _inflate_obstacle(self, target_grid, row: int, col: int):
        r_cells = self.inflation_radius_cells
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                dist_cells = math.sqrt(dr**2 + dc**2)
                if dist_cells <= r_cells:
                    target_row = row + dr
                    target_col = col + dc
                    
                    if 0 <= target_row < self.grid_size and 0 <= target_col < self.grid_size:
                        cost = int(100 * (1.0 - (dist_cells / (r_cells + 1.0))))
                        current_cost = target_grid[target_row, target_col]
                        target_grid[target_row, target_col] = max(current_cost, cost)

    def get_obstacle_forces(self) -> tuple:
        """
        Yapay Potansiyel Alanlar (APF) için bileşke itici kuvveti hesaplar.
        - Kapı dubaları için tam simetrik itme (orta hattı korur).
        - Engeller için asimetrik (sağa kaçış) itme uygular.
        """
        rep_x = 0.0
        rep_y = 0.0
        
        K_repulsive = 5.0
        influence_distance = 6.0 
        
        # 1. Kapı Dubaları (Orange Gates) İtme Hesabı (Simetrik - Dubaların Ortasından Geçiş Sağlar)
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                cost = self.grid_gates[r, c]
                if cost > 30:
                    dx_m = (self.center_idx - r) * self.resolution
                    dy_m = (c - self.center_idx) * self.resolution
                    dist = math.sqrt(dx_m**2 + dy_m**2)
                    if dist < 0.1: continue
                    
                    if dist <= influence_distance:
                        force_mag = K_repulsive * (cost / 100.0) * ((1.0 / dist) - (1.0 / influence_distance)) * (1.0 / dist**2)
                        # Simetrik itme (Botu tam zıt yöne iter, böylece sol ve sağ duba kuvvetleri ortada dengelenir)
                        rep_x += - (dx_m / dist) * force_mag
                        rep_y += - (dy_m / dist) * force_mag
                        
        # 2. Sarı Engeller (Yellow Obstacles) İtme Hesabı (Asimetrik COLREGs - Sağa Sancak Kaçışı Sağlar)
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                cost = self.grid_obstacles[r, c]
                if cost > 30:
                    dx_m = (self.center_idx - r) * self.resolution
                    dy_m = (c - self.center_idx) * self.resolution
                    dist = math.sqrt(dx_m**2 + dy_m**2)
                    if dist < 0.1: continue
                    
                    if dist <= influence_distance:
                        force_mag = K_repulsive * (cost / 100.0) * ((1.0 / dist) - (1.0 / influence_distance)) * (1.0 / dist**2)
                        
                        ux = - (dx_m / dist)
                        uy = - (dy_m / dist)
                        
                        # Eğer engel önümüzde ise sağa kaçışı (sancak) tetikleyecek asimetrik itme uyguluyoruz
                        if dx_m > 0.0:
                            # ~22 derecelik rotasyon (cos(22) = 0.927, sin(22) = 0.374)
                            cos_t = 0.927
                            sin_t = 0.374
                            ux_rot = ux * cos_t + uy * sin_t
                            uy_rot = -ux * sin_t + uy * cos_t
                            ux, uy = ux_rot, uy_rot
                            
                        rep_x += ux * force_mag
                        rep_y += uy * force_mag
                        
        return rep_x, rep_y

    def get_serialized_grid(self) -> list:
        rows, cols = np.where(self.grid > 0)
        serialized = []
        for r, c in zip(rows, cols):
            serialized.append([int(r), int(c), int(self.grid[r, c])])
        return serialized

    def reset(self):
        self.grid_gates.fill(0)
        self.grid_obstacles.fill(0)
        self.grid.fill(0)
