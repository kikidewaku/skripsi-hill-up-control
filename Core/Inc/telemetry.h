/**
 * @file    telemetry.h
 * @brief   Modul telemetri, logging serial UART, dan monitor sistem terpusat
 * @details Menyediakan struktur data tunggal untuk STM32CubeIDE Live Expressions
 *          serta transmisi string serial UART berformat integer-breakdown (kompatibel
 *          dengan GCC Newlib-nano tanpa floating-point printf).
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "mpu6050.h"
#include "pitch_filter.h"
#include "vesc_interface.h"
#include <stdint.h>

/**
 * @brief Struktur data terpusat untuk memantau seluruh parameter sistem di Live Expressions
 */
typedef struct {
  // 1. Data Akselerometer & Giroskop Fisik (g & deg/s)
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;

  // 2. Estimasi Sudut Pitch & Filter (derajat)
  float pitch_accel_raw;
  float pitch_calibrated;
  float pitch_complementary;
  float kalman_pitch;
  float kalman_gyro_bias;

  // 3. Kontrol & User Interface
  float motor_cmd_current;  // Target arus ke VESC (Ampere)
  uint8_t btn_pc13_pressed; // 1 = Ditekan (4A), 0 = Dilepas (0A)

  // 4. Feedback Telemetri VESC (via CAN Bus)
  int32_t vesc_rpm;          // Putaran motor (RPM)
  float vesc_current_actual; // Arus motor riil terbaca (Ampere)
  float vesc_duty_cycle;     // Duty cycle (-1.0 s.d. 1.0)
  float vesc_battery_volt;   // Tegangan baterai (Volt)
  float vesc_fet_temp;       // Suhu MOSFET VESC (deg C)
  float vesc_motor_temp;     // Suhu motor BLDC (deg C)

  // 5. Diagnostik Loop & Sistem
  float dt_ms;               // Periode loop aktual (ms)
  uint32_t loop_counter;     // Counter detak loop
  uint8_t mpu6050_connected; // 1 = Terhubung normal, 0 = Gagal

  // 6. Diagnostik CAN Bus
  uint32_t can_rx_packets;   // Jumlah total paket CAN yang masuk
  uint32_t can_last_rx_id;   // Extended ID paket CAN terakhir
  uint32_t can_error_code;   // Raw register ESR
  uint8_t can_lec;           // Last Error: 0=OK, 1=Stuff, 2=Form, 3=ACK, dst.
  uint8_t can_tec;           // Transmit Error Counter (0 s.d. 255)
  uint8_t can_rec;           // Receive Error Counter (0 s.d. 255)
  uint8_t can_boff;          // 1 = Bus-Off (Mati), 0 = Normal/Aktif
} System_Monitor_t;

/**
 * @brief  Memperbarui data di struktur System_Monitor_t dari seluruh modul subsistem
 * @param  mon: Pointer ke struktur System_Monitor_t
 * @param  imu: Pointer ke data MPU6050
 * @param  filter: Pointer ke data filter pitch
 * @param  vesc: Pointer ke feedback telemetri VESC
 * @param  can_diag: Pointer ke diagnostik CAN
 * @param  cmd_current: Target arus motor yang dikirimkan (Ampere)
 * @param  btn_pressed: Status tombol PC13 (1 = Ditekan, 0 = Dilepas)
 * @param  dt_ms: Durasi periode loop aktual (milidetik)
 */
void Telemetry_Update(System_Monitor_t *mon,
                      const MPU6050_t *imu,
                      const Pitch_Filter_t *filter,
                      const VESC_Telemetry_t *vesc,
                      const CAN_Diagnostics_t *can_diag,
                      float cmd_current,
                      uint8_t btn_pressed,
                      float dt_ms);

/**
 * @brief  Format string telemetri serial dan kirim melalui UART1
 * @param  huart: Pointer ke handle USART/UART STM32
 * @param  mon: Pointer ke struktur System_Monitor_t
 */
void Telemetry_TransmitUART(UART_HandleTypeDef *huart, const System_Monitor_t *mon);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
