#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Emniyet Modları
typedef enum {
    SAFE_MODE_IDLE = 0,
    SAFE_MODE_AUTO,
    SAFE_MODE_MANUAL,
    SAFE_MODE_FAILSAFE,
    SAFE_MODE_EMERGENCY
} SafetyMode_t;

// Emniyet Sistemi Başlatma
void safety_init(void);

// Telefon ve RC bağlantı takip sinyalleri (Heartbeat)
void safety_feed_phone_heartbeat(void);
void safety_feed_rc_link(void);
void safety_feed_phone_heartbeat_tick(uint32_t current_time_ms);
void safety_feed_rc_link_tick(uint32_t current_time_ms);

// Fiziksel acil durum kesmesi (Interrupt tetiklemeli)
void safety_trigger_physical_cutoff(void);

// Her çevrimde (örneğin 100 Hz veya 1000 Hz) çağrılarak durum kontrolü yapar
// [Kötü Senaryo 8 ve 9 Korumaları İçin Yeni Parametreler]
// Geri dönüş değeri: 1 = Failsafe aktif (motorları durdur), 0 = Normal
uint8_t safety_check(uint32_t current_time_ms, float raw_battery_v, 
                     float yaw_rate_deg_s, float left_motor_cmd, float right_motor_cmd);

// Anlık çalışma modunu döner
SafetyMode_t safety_get_mode(void);

// Yazılımsal manuel kontrol geçiş tetiği
void safety_set_mode(SafetyMode_t new_mode);

#ifdef __cplusplus
}
#endif

#endif // SAFETY_H
