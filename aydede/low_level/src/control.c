#include "control.h"
#include <math.h>

static PID_t yaw_pid;

void control_init(void) {
    // Varsayılan PID katsayıları (Saha testlerinde optimize edilebilir)
    yaw_pid.kp = 0.8f;
    yaw_pid.ki = 0.05f;
    yaw_pid.kd = 0.2f;
    yaw_pid.integrator = 0.0f;
    yaw_pid.last_error = 0.0f;
    yaw_pid.max_integrator = 0.3f; // Integrator doyumu (Anti-windup)
}

void control_set_pid_gains(float kp, float ki, float kd) {
    yaw_pid.kp = kp;
    yaw_pid.ki = ki;
    yaw_pid.kd = kd;
    yaw_pid.integrator = 0.0f;
}

MotorOutput_t control_update(float current_yaw, float target_yaw, float target_speed, float dt) {
    MotorOutput_t output;
    
    if (dt <= 0.0f) {
        output.left_thrust = 0.0f;
        output.right_thrust = 0.0f;
        return output;
    }

    // 1. Açısal Hata Hesaplama ve Sarmalama (Yaw Wrapping)
    // 359 derece ile 1 derece arasındaki hatanın 358 değil, -2 derece olmasını sağlar.
    float error = target_yaw - current_yaw;
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 2. Oransal Terim (Proportional)
    float p_term = yaw_pid.kp * error;

    // 3. İntegral Terim (Integral) ve Anti-Windup (Doyum Sınırı)
    yaw_pid.integrator += error * dt;
    if (yaw_pid.integrator > yaw_pid.max_integrator) {
        yaw_pid.integrator = yaw_pid.max_integrator;
    } else if (yaw_pid.integrator < -yaw_pid.max_integrator) {
        yaw_pid.integrator = -yaw_pid.max_integrator;
    }
    float i_term = yaw_pid.ki * yaw_pid.integrator;

    // 4. Türev Terim (Derivative)
    float derivative = (error - yaw_pid.last_error) / dt;
    float d_term = yaw_pid.kd * derivative;
    
    yaw_pid.last_error = error;

    // 5. Toplam Dümen Düzeltme Komutu (Steering Command)
    float steer_cmd = p_term + i_term + d_term;
    
    // Dümen düzeltmesini makul limitlerde sınırla (-0.5 ile 0.5 arası)
    if (steer_cmd > 0.6f)  steer_cmd = 0.6f;
    if (steer_cmd < -0.6f) steer_cmd = -0.6f;

    // 6. Diferansiyel İtki Eşleme (Differential Thrust Mapping)
    // Katamaran için sol ve sağ motorların güçlerini hesapla:
    // Sol motor hızı artırılıp sağ motor azaltılırsa bot SAĞA döner (steer_cmd pozitif ise)
    // Sol motor: İleri Hız + Dümen Düzeltmesi
    // Sağ motor: İleri Hız - Dümen Düzeltmesi
    output.left_thrust = target_speed + steer_cmd;
    output.right_thrust = target_speed - steer_cmd;

    // Sınırlandırma (Clamping) -> Motor çıkışlarının [-1.0, 1.0] aralığında olmasını garantiler
    if (output.left_thrust > 1.0f)  output.left_thrust = 1.0f;
    if (output.left_thrust < -1.0f) output.left_thrust = -1.0f;
    
    if (output.right_thrust > 1.0f)  output.right_thrust = 1.0f;
    if (output.right_thrust < -1.0f) output.right_thrust = -1.0f;

    // Emniyet Koruması: Eğer hedef ileri hız sıfır ise ve yön değişimi gereksiz küçükse motorları kapat
    if (target_speed < 0.05f && fabs(error) < 5.0f) {
        output.left_thrust = 0.0f;
        output.right_thrust = 0.0f;
    }

    return output;
}
