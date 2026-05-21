#include "safety.h"
#include <math.h>

// Zaman Aşımı Limitleri (milisaniye)
#define PHONE_HEARTBEAT_TIMEOUT_MS  500
#define RC_LINK_TIMEOUT_MS         1000

// Durum Değişkenleri
static uint32_t last_phone_hb_time = 0;
static uint32_t last_rc_link_time = 0;
static volatile uint8_t physical_cutoff_active = 0;
static uint8_t hardware_failsafe_active = 0; // Donanımsal hata ile failsafe'e düşme kilidi

static SafetyMode_t current_safety_mode = SAFE_MODE_IDLE;

// [Kötü Senaryo 8]: Batarya Voltaj Düşümü (Sag) Filtresi Değişkenleri
static float filtered_voltage = 12.0f;
static uint32_t low_voltage_start_time = 0;

// [Kötü Senaryo 9]: Yosun Dolanması / Motor Kilitlenme Takip Sayaçları
static uint32_t stall_timer_ms = 0;

void safety_init(void) {
    last_phone_hb_time = 0;
    last_rc_link_time = 0;
    physical_cutoff_active = 0;
    hardware_failsafe_active = 0;
    current_safety_mode = SAFE_MODE_IDLE;
    filtered_voltage = 12.0f;
    low_voltage_start_time = 0;
    stall_timer_ms = 0;
}

void safety_feed_phone_heartbeat(void) {
    // heartbeats can be fed via the tick function below
}

void safety_feed_rc_link(void) {
    // rc link can be fed via the tick function below
}

void safety_trigger_physical_cutoff(void) {
    physical_cutoff_active = 1;
    current_safety_mode = SAFE_MODE_EMERGENCY;
}

void safety_set_mode(SafetyMode_t new_mode) {
    if (current_safety_mode == SAFE_MODE_EMERGENCY && new_mode != SAFE_MODE_EMERGENCY) {
        return; 
    }
    if (hardware_failsafe_active && new_mode == SAFE_MODE_AUTO) {
        return; 
    }
    current_safety_mode = new_mode;
}

SafetyMode_t safety_get_mode(void) {
    return current_safety_mode;
}

uint8_t safety_check(uint32_t current_time_ms, float raw_battery_v, 
                     float yaw_rate_deg_s, float left_motor_cmd, float right_motor_cmd) {
    
    // 1. Donanımsal Kesme Butonu Kontrolü (En yüksek öncelik)
    if (physical_cutoff_active) {
        current_safety_mode = SAFE_MODE_EMERGENCY;
        return 1;
    }
    
    if (current_safety_mode == SAFE_MODE_EMERGENCY) {
        return 1;
    }

    // İlk çalıştırma zaman aşımlarını eşitle
    if (last_phone_hb_time == 0) last_phone_hb_time = current_time_ms;
    if (last_rc_link_time == 0)   last_rc_link_time = current_time_ms;

    // 2. [Kötü Senaryo 8 Önlemi]: Batarya Voltaj Düşüş (Sag) Filtresi
    // Motorlar aniden tam güce çıktığında batarya voltajı geçici olarak çöker.
    // EMA (Exponential Moving Average) filtresi ile voltaj gürültülerini süzeriz.
    filtered_voltage = 0.98f * filtered_voltage + 0.02f * raw_battery_v;
    
    if (filtered_voltage < 10.5f) { // 3S Lipo için sınır
        if (low_voltage_start_time == 0) {
            low_voltage_start_time = current_time_ms;
        } else if (current_time_ms - low_voltage_start_time > 3000) {
            // Voltaj 3 saniyeden uzun süre filtrelenmiş olarak düşük kaldıysa failsafe tetikle
            current_safety_mode = SAFE_MODE_FAILSAFE;
            hardware_failsafe_active = 1;
            return 1;
        }
    } else {
        low_voltage_start_time = 0; // Voltaj normale döndüyse sayacı sıfırla
    }

    // 3. [Kötü Senaryo 9 Önlemi]: Yosun Dolanması / Motor Kilitlenme Koruması
    // Eğer sol ve sağ motorlar arasında belirgin bir itki farkı varsa (steer_cmd yüksekse)
    // ancak tekne jiroskop verisine göre dönmüyorsa (yaw_rate < 2.0 derece/sn), pervaneye yosun dolanmış demektir.
    float diff_thrust = fabsf(left_motor_cmd - right_motor_cmd);
    
    if (current_safety_mode == SAFE_MODE_AUTO && diff_thrust > 0.5f && fabsf(yaw_rate_deg_s) < 2.0f) {
        if (stall_timer_ms == 0) {
            stall_timer_ms = current_time_ms;
        } else if (current_time_ms - stall_timer_ms > 4000) {
            // 4 saniye boyunca komuta rağmen bot dönemediyse motoru/ESC'yi korumak için failsafe'e al
            current_safety_mode = SAFE_MODE_FAILSAFE;
            hardware_failsafe_active = 1;
            return 1;
        }
    } else {
        stall_timer_ms = 0;
    }

    // 4. Telefon Seri İletişim Zaman Aşımı Kontrolü
    if (current_safety_mode == SAFE_MODE_AUTO) {
        if (current_time_ms - last_phone_hb_time > PHONE_HEARTBEAT_TIMEOUT_MS) {
            current_safety_mode = SAFE_MODE_FAILSAFE;
            return 1;
        }
    }
    
    // 5. RC Kumanda Sinyal Zaman Aşımı Kontrolü
    if (current_safety_mode == SAFE_MODE_MANUAL) {
        if (current_time_ms - last_rc_link_time > RC_LINK_TIMEOUT_MS) {
            current_safety_mode = SAFE_MODE_FAILSAFE;
            return 1;
        }
    }

    if (current_safety_mode == SAFE_MODE_FAILSAFE) {
        return 1;
    }

    return 0;
}

void safety_feed_phone_heartbeat_tick(uint32_t current_time_ms) {
    last_phone_hb_time = current_time_ms;
    // Kalp atışı failsafe durumunu sadece haberleşme kaynaklıysa sıfırlayabilir (donanımsal hata yoksa)
    if (current_safety_mode == SAFE_MODE_FAILSAFE && !physical_cutoff_active && !hardware_failsafe_active) {
        current_safety_mode = SAFE_MODE_AUTO;
    }
}

void safety_feed_rc_link_tick(uint32_t current_time_ms) {
    last_rc_link_time = current_time_ms;
    // Alıcı sinyali failsafe durumunu sadece haberleşme kaynaklıysa sıfırlayabilir (donanımsal hata yoksa)
    if (current_safety_mode == SAFE_MODE_FAILSAFE && !physical_cutoff_active && !hardware_failsafe_active) {
        current_safety_mode = SAFE_MODE_MANUAL;
    }
}
