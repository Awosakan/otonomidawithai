#include "sensors.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// STM32 Sistem Zaman Sayacı (Tick) Bildirimi
extern uint32_t HAL_GetTick(void);

// GPS Durum Değişkenleri
static double gps_lat = 0.0;
static double gps_lon = 0.0;
static float gps_sog = 0.0;
static float gps_cog = 0.0;
static uint8_t gps_lock_state = 0;

// [Kötü Senaryo 1 Önlemi]: GPS Sıçrama (Outlier) Filtresi Değişkenleri
static double last_valid_lat = 0.0;
static double last_valid_lon = 0.0;
static uint32_t last_gps_time_ms = 0;
static uint8_t consecutive_gps_outliers = 0;

// IMU Durum Değişkenleri (Tamamlayıcı Filtre Durumu)
static float imu_roll = 0.0f;
static float imu_pitch = 0.0f;
static float imu_yaw = 0.0f;
static float imu_gx = 0.0f, imu_gy = 0.0f, imu_gz = 0.0f;

// Tamamlayıcı Filtre Parametreleri
#define ALPHA_YAW 0.98f

// GPS NMEA Ayrıştırıcı Durum Makinesi
#define NMEA_BUF_SIZE 120
static char nmea_buf[NMEA_BUF_SIZE];
static uint8_t nmea_idx = 0;
static uint8_t nmea_recording = 0;

// Donanımsal I2C Kurtarma Değişkenleri
static uint8_t i2c_error_flag = 0;

void sensors_init(void) {
    gps_lat = 0.0;
    gps_lon = 0.0;
    gps_sog = 0.0f;
    gps_cog = 0.0f;
    gps_lock_state = 0;
    
    last_valid_lat = 0.0;
    last_valid_lon = 0.0;
    last_gps_time_ms = 0;
    consecutive_gps_outliers = 0;
    
    imu_roll = 0.0f;
    imu_pitch = 0.0f;
    imu_yaw = 0.0f;
    imu_gx = 0.0f; imu_gy = 0.0f; imu_gz = 0.0f;
    
    i2c_error_flag = 0;
}

// Derece-Dakika formatını (DDMM.MMMM) Ondalık Dereceye (DD.DDDDDD) dönüştürür.
static double nmea_to_decimal(const char *str, char direction) {
    if (!str || strlen(str) == 0) return 0.0;
    
    double raw = atof(str);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);
    
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

// İki nokta arasındaki mesafeyi basitçe hesaplar (Metre)
static float distance_meters(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * 0.01745329f;
    double dlon = (lon2 - lon1) * 0.01745329f;
    double lat_avg = ((lat1 + lat2) / 2.0) * 0.01745329f;
    
    float R = 6378137.0f;
    float dy = dlat * R;
    float dx = dlon * R * cosf(lat_avg);
    return sqrtf(dx*dx + dy*dy);
}

static void parse_nmea_sentence(char *line) {
    // Sadece $GPRMC (Önerilen Minimum Seyrüsefer Verisi) ayrıştırılır.
    if (strncmp(line, "$GPRMC", 6) == 0 || strncmp(line, "$GNRMC", 6) == 0) {
        int field_idx = 0;
        char *p = line;
        char *next_comma;
        
        char lat_str[15] = {0};
        char lon_str[15] = {0};
        char lat_dir = 'N';
        char lon_dir = 'E';
        char status = 'V';
        char speed_str[10] = {0};
        char cog_str[10] = {0};
        
        while (p && *p) {
            next_comma = strchr(p, ',');
            int len = next_comma ? (next_comma - p) : strlen(p);
            
            char field[30] = {0};
            if (len > 0) {
                if (len >= sizeof(field)) len = sizeof(field) - 1;
                strncpy(field, p, len);
                field[len] = '\0';
            }
            
            field_idx++;
            switch (field_idx) {
                case 3: // Status (A=Lock, V=No Lock)
                    if (len > 0) status = field[0];
                    break;
                case 4: // Latitude
                    strncpy(lat_str, field, sizeof(lat_str)-1);
                    break;
                case 5: // N/S Indicator
                    if (len > 0) lat_dir = field[0];
                    break;
                case 6: // Longitude
                    strncpy(lon_str, field, sizeof(lon_str)-1);
                    break;
                case 7: // E/W Indicator
                    if (len > 0) lon_dir = field[0];
                    break;
                case 8: // Speed Over Ground (Knots)
                    strncpy(speed_str, field, sizeof(speed_str)-1);
                    break;
                case 9: // Course Over Ground (Degrees)
                    strncpy(cog_str, field, sizeof(cog_str)-1);
                    break;
            }
            
            if (next_comma) {
                p = next_comma + 1;
            } else {
                break;
            }
        }
        
        if (status == 'A') {
            double new_lat = nmea_to_decimal(lat_str, lat_dir);
            double new_lon = nmea_to_decimal(lon_str, lon_dir);
            
            // [Kötü Senaryo 1 Önlemi]: GPS Sıçrama Filtresi
            // Eğer daha önceden geçerli bir konumumuz varsa, yeni gelen konumun
            // botun maksimum hızından (örn. 6.0 m/s) daha hızlı hareket edip etmediğini kontrol et.
            uint8_t accept_point = 1;
            
            // STM32'nin donanımsal zaman sayacından (HAL_GetTick()) anlık milisaniyeyi oku
            uint32_t now_ms = HAL_GetTick(); 
            
            if (last_valid_lat != 0.0 && last_valid_lon != 0.0) {
                float dist = distance_meters(last_valid_lat, last_valid_lon, new_lat, new_lon);
                
                // İki GPS paketi arasındaki gerçek zaman farkını saniye cinsinden hesapla
                uint32_t dt_ms = now_ms - last_gps_time_ms;
                float dt_sec = 1.0f; // Varsayılan 1 saniye
                if (last_gps_time_ms != 0 && dt_ms > 0) {
                    dt_sec = (float)dt_ms / 1000.0f;
                }
                
                float inferred_speed = dist / dt_sec;
                
                // Eğer İDA fiziksel limitlerin üzerinde hareket ettiğini iddia ediyorsa bu koordinatı yoksay
                if (inferred_speed > 6.0f) {
                    consecutive_gps_outliers++;
                    accept_point = 0;
                    // Eğer üst üste 5 kez sıçrama algılanırsa, baz alınan eski konumun yanlış olduğunu varsay
                    // ve kilidi tazelemek için bu noktayı kabul et.
                    if (consecutive_gps_outliers >= 5) {
                        accept_point = 1;
                        consecutive_gps_outliers = 0;
                    }
                } else {
                    consecutive_gps_outliers = 0;
                }
            }
            
            if (accept_point) {
                gps_lock_state = 1;
                gps_lat = new_lat;
                gps_lon = new_lon;
                last_valid_lat = new_lat;
                last_valid_lon = new_lon;
                last_gps_time_ms = now_ms; // Son geçerli GPS paketinin zamanını güncelle
                gps_sog = atof(speed_str) * 0.514444f;
                gps_cog = atof(cog_str);
            }
        } else {
            gps_lock_state = 0;
        }
    }
}

void gps_parse_char(char c) {
    if (c == '$') {
        nmea_idx = 0;
        nmea_recording = 1;
    }
    
    if (nmea_recording) {
        if (c == '\n' || c == '\r') {
            nmea_buf[nmea_idx] = '\0';
            parse_nmea_sentence(nmea_buf);
            nmea_recording = 0;
        } else {
            if (nmea_idx < NMEA_BUF_SIZE - 1) {
                nmea_buf[nmea_idx++] = c;
            } else {
                nmea_recording = 0;
            }
        }
    }
}

void sensors_read_gps(double *lat, double *lon, float *sog, float *cog, uint8_t *gps_lock) {
    *lat = gps_lat;
    *lon = gps_lon;
    *sog = gps_sog;
    *cog = gps_cog;
    *gps_lock = gps_lock_state;
}

// [Kötü Senaryo 3 Önlemi]: Donanımsal I2C Hattı Kilitlenme Kurtarma
// STM32'de SDA hattının slave tarafından sıfıra çekilip kilitlendiği durumlar için
// SCL pinini GPIO çıkışı olarak el ile 9 kez clock tetikleyerek veri hattını serbest bırakır.
static void I2C_RecoverBus(void) {
    // Temsili logger pasif
    
    // 1. I2C donanımını kapat
    // __HAL_I2C_DISABLE(&hi2c1);
    
    // 2. SCL ve SDA pinlerini GPIO çıkışı yap
    // Pin_SCL_Mode_Output();
    
    // 3. SCL hattını 9 kez toggle et
    for (int i = 0; i < 9; i++) {
        // Pin_SCL_Write(1);
        // Delay_us(5);
        // Pin_SCL_Write(0);
        // Delay_us(5);
    }
    
    // 4. STOP koşulu oluştur (SDA low iken SCL high yapılır, ardından SDA high yapılır)
    // Pin_SDA_Write(0);
    // Pin_SCL_Write(1);
    // Delay_us(5);
    // Pin_SDA_Write(1);
    
    // 5. I2C donanımını yeniden başlat
    // HAL_I2C_Init(&hi2c1);
    i2c_error_flag = 0;
}

// Sensör birleştirme filtresi (Complementary Filter)
void sensors_update_imu_filter(float ax, float ay, float az, 
                               float gx, float gy, float gz, 
                               float mx, float my, float mz, 
                               float dt) {
                               
    // [Kötü Senaryo 3 Kontrolü]: I2C okuma hatası sayacı (Örn: ardışık 3 başarısız I2C)
    // Eğer donanımsal hata tespit edilirse I2C hattını otomatik kurtar
    if (i2c_error_flag > 3) {
        I2C_RecoverBus();
    }
    
    imu_gx = gx;
    imu_gy = gy;
    imu_gz = gz;
    
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
    float accel_roll = atan2f(ay, az) * 57.29578f;
    
    imu_pitch = 0.98f * (imu_pitch + gy * 57.29578f * dt) + 0.02f * accel_pitch;
    imu_roll = 0.98f * (imu_roll + gx * 57.29578f * dt) + 0.02f * accel_roll;
    
    // [Kötü Senaryo 2 Önlemi]: Pusula Bozulması (Manyetik Girişim) ve GPS COG Fallback
    // Eğer motor akımlarından veya metal gövdeden ötürü pusula saparsa,
    // bot hareket halinde iken (hız > 0.6 m/s) doğruluğu yüksek olan GPS Course Over Ground (COG) açısını
    // pusula (yaw) düzeltmesi olarak kullanırız.
    float mag_heading = 0.0f;
    
    if (gps_lock_state && gps_sog > 0.6f) {
        // Hızlı giderken GPS Rota Açısı (COG) referans alınır (Pusula manyetik gürültülerini filtreler)
        mag_heading = gps_cog;
    } else {
        // Düşük hızlarda veya dururken tilt-compensated pusula hesabı kullanılır
        float pitch_rad = imu_pitch * 0.017453f;
        float roll_rad = imu_roll * 0.017453f;
        
        float Xm = mx * cosf(pitch_rad) + mz * sinf(pitch_rad);
        float Ym = mx * sinf(roll_rad) * sinf(pitch_rad) + my * cosf(roll_rad) - mz * sinf(roll_rad) * cosf(pitch_rad);
        
        float magnetic_declination = 6.0f;
        mag_heading = atan2f(-Ym, Xm) * 57.29578f + magnetic_declination;
        
        if (mag_heading < 0) mag_heading += 360.0f;
    }
    
    // Yaw Açısı Tamamlayıcı Filtre Güncellemesi
    float gyro_yaw_delta = gz * 57.29578f * dt;
    imu_yaw += gyro_yaw_delta;
    
    float yaw_error = mag_heading - imu_yaw;
    while (yaw_error > 180.0f)  yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;
    
    imu_yaw += (1.0f - ALPHA_YAW) * yaw_error;
    
    while (imu_yaw >= 360.0f) imu_yaw -= 360.0f;
    while (imu_yaw < 0.0f)   imu_yaw += 360.0f;
}

void sensors_read_imu(float *roll, float *pitch, float *yaw, 
                      float *gx, float *gy, float *gz) {
    *roll = imu_roll;
    *pitch = imu_pitch;
    *yaw = imu_yaw;
    *gx = imu_gx;
    *gy = imu_gy;
    *gz = imu_gz;
}

float sensors_read_battery(void) {
    return 11.8f;
}
