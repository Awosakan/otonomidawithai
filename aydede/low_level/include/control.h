#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// PID Kontrol Yapısı
typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float last_error;
    float max_integrator;
} PID_t;

// Diferansiyel İtki Çıktıları
typedef struct {
    float left_thrust;  // -1.0 ile 1.0 arasında (sol motor yüzdesi)
    float right_thrust; // -1.0 ile 1.0 arasında (sağ motor yüzdesi)
} MotorOutput_t;

// Kontrolcü başlatma
void control_init(void);

// PID Katsayılarını Güncelle
void control_set_pid_gains(float kp, float ki, float kd);

// Yönelim Sabitleme ve Diferansiyel İtki Hesaplama Döngüsü
// current_yaw: Anlık pusula açısı (0-360 derece)
// target_yaw: Telefondan gelen hedef rota açısı (0-360 derece)
// target_speed: Telefondan gelen hedef ileri hız komutu (0.0 ile 1.0 arası güç yüzdesi)
MotorOutput_t control_update(float current_yaw, float target_yaw, float target_speed, float dt);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_H
