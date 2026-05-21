#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Sensör Donanımlarının Başlatılması
void sensors_init(void);

// IMU ve Pusula Okuma (Complementary/Kalman Filtresi Çıktısı)
// roll, pitch, yaw: Derece cinsinden yönelimler
// gx, gy, gz: Radyan/sn cinsinden açısal hızlar
void sensors_read_imu(float *roll, float *pitch, float *yaw, 
                      float *gx, float *gy, float *gz);

// GPS Verilerini Okuma
// lat, lon: Coğrafi koordinatlar
// sog: Hız (m/s), cog: Rota açısı (derece)
// gps_lock: 0 = Kilit yok, 1 = 2D/3D Kilit var
void sensors_read_gps(double *lat, double *lon, float *sog, float *cog, uint8_t *gps_lock);

// Batarya Voltajını Okuma (ADC Arayüzü)
float sensors_read_battery(void);

// GPS Seri Portundan Gelen Karakterleri NMEA Ayrıştırıcıya Besler
void gps_parse_char(char c);

#ifdef __cplusplus
}
#endif

#endif // SENSORS_H
