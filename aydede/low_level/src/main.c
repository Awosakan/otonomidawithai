#include <stdint.h>
#include <string.h>
#include <math.h>

// Autopilot Alt Modülleri
#include "protocol.h"
#include "safety.h"
#include "control.h"
#include "sensors.h"

// STM32 Donanımsal Zaman Sayacı (Tick)
extern uint32_t HAL_GetTick(void);

// STM32 Temsili Donanım Register Tanımları (CMSIS Yapısı)
// 168 MHz SYSCLK (Saniyenin 168 milyonda biri hassasiyetinde saat çözünürlüğü)
#define FLASH_ACR_LATENCY_5WS   (5UL << 0)
#define FLASH_ACR_PRFTEN        (1UL << 8)
#define FLASH_ACR_ICEN          (1UL << 9)
#define FLASH_ACR_DCEN          (1UL << 10)

typedef struct {
    volatile uint32_t ACR;
} FLASH_TypeDef;
#define FLASH ((FLASH_TypeDef *) 0x40023C00)

typedef struct {
    volatile uint32_t CPACR;
} SCB_TypeDef;
#define SCB ((SCB_TypeDef *) 0xE000ED00)

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
} RCC_TypeDef;
#define RCC ((RCC_TypeDef *) 0x40023800)

// DMA & USART Register Yapıları (Zero-CPU Gecikmeli Seri Haberleşme)
typedef struct {
    volatile uint32_t CR;
    volatile uint32_t NDTR;
    volatile uint32_t PAR;
    volatile uint32_t M0AR;
    volatile uint32_t M1AR;
    volatile uint32_t FCR;
} DMA_Stream_TypeDef;

#define DMA1_Stream5 ((DMA_Stream_TypeDef *) 0x40026084)
#define DMA2_Stream5 ((DMA_Stream_TypeDef *) 0x40026488)
#define DMA_SxCR_EN             (1UL << 0)
#define DMA_SxCR_CIRC           (1UL << 8)
#define DMA_SxCR_MINC           (1UL << 10)
#define USART_CR1_UE            (1UL << 13)
#define USART_CR1_RE            (1UL << 2)
#define USART_CR1_TE            (1UL << 3)
#define USART_CR1_IDLEIE        (1UL << 4) // Boşta Hattı Kesmesi (Packet End Detection)
#define USART_CR3_DMAR          (1UL << 6) // DMA Alıcı Aktif

// Ring Buffer (Dairesel Seri Tampon) Boyutu
#define RX_DMA_BUF_SIZE 256
static uint8_t usart1_rx_dma_buffer[RX_DMA_BUF_SIZE];
static uint32_t last_dma_read_ptr = 0;

// Seri Port (USART) Fonksiyon Tanımları
void USART_SendBytes(const uint8_t *data, uint16_t len);
uint8_t USART_ReceiveByte(uint8_t *b);

// PWM Zamanlayıcı (TIM) Ayarları
void TIM_SetPWM_LeftMotor(uint32_t pulse_us);
void TIM_SetPWM_RightMotor(uint32_t pulse_us);

// FreeRTOS Task Temsilleri
#define pdMS_TO_TICKS(ms) (ms)
void vTaskDelay(uint32_t ticks);
uint32_t xTaskGetTickCount(void);

// Küresel Durum Değişkenleri
static ProtocolParser_t phone_parser;
static Telemetry_t current_telemetry;
static PhoneCommands_t last_phone_command;
static float current_left_thrust = 0.0f;
static float current_right_thrust = 0.0f;

// --- Donanım Suyunu Çıkarma (Register Düzeyinde Donanım Aktivasyonu) ---

void Hardware_Extreme_Optimize_Init(void) {
    // 1. Donanımsal FPU (Coprocessor 10 & 11) Etkinleştirme
    // floating-point hesaplamalarını (PID, Tamamlayıcı Filtre) tek çevrimde donanımsal yapar.
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2)); // Full access to CP10 and CP11 FPU
    
    // 2. 168 MHz SYSCLK Saat Hızı ve PLL Ayarları
    // HSE (Harici Kristal) = 8 MHz. PLL_M=8, PLL_N=336, PLL_P=2 -> 168 MHz SYSCLK.
    RCC->PLLCFGR = (8UL << 0) | (336UL << 6) | (0UL << 16) | (1UL << 22) | (7UL << 24);
    // PLL aktif et ve SYSCLK kaynağı olarak seç
    
    // 3. ART Accelerator (Flash Prefetch, Instruction & Data Cache) Etkinleştirme
    // 168 MHz'de çalışırken Flash okuma gecikmesini (Latency) 5 Wait State olarak ayarlar
    // ve Cache tamponlarını açarak CPU'nun Flash üzerinde beklemesini (stall) engeller.
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    
    // 4. USART1 DMA Dairesel Mod Yapılandırması
    // Gelen seri verileri CPU müdahalesi olmadan doğrudan RAM tamponuna akar.
    // DMA1_Stream5->CR |= DMA_SxCR_EN | DMA_SxCR_CIRC | DMA_SxCR_MINC;
    // USART1->CR3 |= USART_CR3_DMAR; // Enable USART1 DMA Receiver
    // USART1->CR1 |= USART_CR1_IDLEIE; // IDLE Hattı kesmesini aç (Paket bittiğinde uyarır)
}

// Görev 1: Seri İletişim ve Sıfır Gecikmeli DMA Ayrıştırma (10 Hz)
void vTelemetryTask(void *argument) {
    protocol_parser_init(&phone_parser);
    Hardware_Extreme_Optimize_Init();
    
    while (1) {
        // 1. Telemetriyi doldur ve gönder
        sensors_read_gps(&current_telemetry.lat, &current_telemetry.lon, 
                         &current_telemetry.sog, &current_telemetry.cog, 
                         &current_telemetry.gps_lock);
                         
        sensors_read_imu(&current_telemetry.roll, &current_telemetry.pitch, &current_telemetry.yaw,
                         &current_telemetry.roll_rate, &current_telemetry.pitch_rate, &current_telemetry.yaw_rate);
                         
        current_telemetry.battery = sensors_read_battery();
        current_telemetry.mode = (uint8_t)safety_get_mode();
        
        uint8_t payload_buf[60];
        memcpy(payload_buf, &current_telemetry, sizeof(Telemetry_t));
        IDAPacket packet;
        packet.msg_id = MSG_STM32_TELEMETRY;
        packet.payload_len = sizeof(Telemetry_t);
        
        uint8_t tx_buf[70];
        tx_buf[0] = SYNC_BYTE_1;
        tx_buf[1] = SYNC_BYTE_2;
        tx_buf[2] = packet.msg_id;
        tx_buf[3] = packet.payload_len;
        memcpy(tx_buf + 4, payload_buf, sizeof(Telemetry_t));
        uint16_t crc = calculate_crc16(tx_buf, 4 + sizeof(Telemetry_t));
        tx_buf[4 + sizeof(Telemetry_t)] = (uint8_t)(crc & 0xFF);
        tx_buf[5 + sizeof(Telemetry_t)] = (uint8_t)((crc >> 8) & 0xFF);
        
        USART_SendBytes(tx_buf, 6 + sizeof(Telemetry_t));
        
        // 2. [Sıfır Gecikmeli DMA Okuma]:
        // DMA dairesel tamponunu tarayarak gelen baytları doğrudan parser'a besler.
        // DMA çevre biriminin yazma imlecini (NDTR registeri) kontrol ederek verileri okur.
        // Gerçek DMA NDTR imleç hesabı (DMA2_Stream5, USART1 RX için kullanılır):
        uint32_t current_dma_ptr = RX_DMA_BUF_SIZE - DMA2_Stream5->NDTR;
        
        while (last_dma_read_ptr != current_dma_ptr) {
            uint8_t rx_byte = usart1_rx_dma_buffer[last_dma_read_ptr];
            
            if (protocol_parser_feed(&phone_parser, rx_byte)) {
                if (phone_parser.msg_id == MSG_HEARTBEAT) {
                    safety_feed_phone_heartbeat_tick(HAL_GetTick());
                } 
                else if (phone_parser.msg_id == MSG_PHONE_COMMANDS) {
                    memcpy(&last_phone_command, phone_parser.payload, sizeof(PhoneCommands_t));
                    if (safety_get_mode() == SAFE_MODE_IDLE || safety_get_mode() == SAFE_MODE_FAILSAFE) {
                        safety_set_mode(SAFE_MODE_AUTO);
                    }
                }
                else if (phone_parser.msg_id == MSG_PID_TUNING) {
                    PIDTuning_t pid;
                    memcpy(&pid, phone_parser.payload, sizeof(PIDTuning_t));
                    control_set_pid_gains(pid.kp, pid.ki, pid.kd);
                }
            }
            
            last_dma_read_ptr = (last_dma_read_ptr + 1) % RX_DMA_BUF_SIZE;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    }
}

// Görev 2: Navigasyon ve FPU Hızlandırılmış PID (50 Hz)
void vNavigationTask(void *argument) {
    const float dt = 0.02f; // 20ms
    control_init();
    
    while (1) {
        // Temsili I2C sensör okumaları (FPU sayesinde float işlemler donanımsal yürütülür)
        float ax = 0.0f, ay = 0.0f, az = 9.8f;
        float gx = 0.0f, gy = 0.0f, gz = 0.0f;
        float mx = 10.0f, my = 5.0f, mz = -2.0f;
        
        float roll, pitch, yaw, dummy_g;
        sensors_read_imu(&roll, &pitch, &yaw, &dummy_g, &dummy_g, &dummy_g);
        
        if (safety_get_mode() == SAFE_MODE_AUTO) {
            MotorOutput_t motors = control_update(yaw, last_phone_command.target_heading, 
                                                  last_phone_command.target_speed, dt);
            
            current_left_thrust = motors.left_thrust;
            current_right_thrust = motors.right_thrust;
                                                  
            uint32_t left_pulse = (uint32_t)(1500.0f + motors.left_thrust * 500.0f);
            uint32_t right_pulse = (uint32_t)(1500.0f + motors.right_thrust * 500.0f);
            
            TIM_SetPWM_LeftMotor(left_pulse);
            TIM_SetPWM_RightMotor(right_pulse);
        } else {
            current_left_thrust = 0.0f;
            current_right_thrust = 0.0f;
            TIM_SetPWM_LeftMotor(1500);
            TIM_SetPWM_RightMotor(1500);
        }
        
        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
    }
}

// Görev 3: Güvenlik, Acil Kesme (100 Hz)
void vSafetyTask(void *argument) {
    safety_init();
    
    while (1) {
        uint32_t now = xTaskGetTickCount();
        
        float roll, pitch, yaw, gx, gy, gz;
        sensors_read_imu(&roll, &pitch, &yaw, &gx, &gy, &gz);
        float bat = sensors_read_battery();
        float yaw_rate_deg = gz * 57.29578f;
        
        if (safety_check(now, bat, yaw_rate_deg, current_left_thrust, current_right_thrust)) {
            TIM_SetPWM_LeftMotor(1500);
            TIM_SetPWM_RightMotor(1500);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

// STM32 HAL Temsili Sürücü Gövdeleri
void USART_SendBytes(const uint8_t *data, uint16_t len) {
    // UART1 TX DMA aktarımı başlatılır (Sıfır işlemci beklemesi)
}

uint8_t USART_ReceiveByte(uint8_t *b) {
    return 0;
}

void TIM_SetPWM_LeftMotor(uint32_t pulse_us) {
}

void TIM_SetPWM_RightMotor(uint32_t pulse_us) {
}
