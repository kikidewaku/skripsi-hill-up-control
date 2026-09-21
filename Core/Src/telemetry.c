/**
 * @file    telemetry.c
 * @brief   Implementasi modul telemetri dan logging UART
 */

#include "telemetry.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

void Telemetry_Update(System_Monitor_t *mon,
                      const MPU6050_t *imu,
                      const Pitch_Filter_t *filter,
                      const VESC_Telemetry_t *vesc,
                      const CAN_Diagnostics_t *can_diag,
                      float cmd_current,
                      uint8_t btn_pressed,
                      float dt_ms) {
  if (mon == NULL) {
    return;
  }

  // 1. Update data IMU MPU6050
  if (imu != NULL) {
    mon->accel_x = imu->ax;
    mon->accel_y = imu->ay;
    mon->accel_z = imu->az;
    mon->gyro_x  = imu->gx;
    mon->gyro_y  = imu->gy;
    mon->gyro_z  = imu->gz;
    mon->pitch_accel_raw = imu->pitch_accel;
    mon->mpu6050_connected = imu->is_connected;
  }

  // 2. Update data Filter Estimasi Pitch
  if (filter != NULL) {
    mon->pitch_calibrated    = filter->pitch_calibrated;
    mon->pitch_complementary = filter->comp_pitch;
    mon->kalman_pitch        = filter->kalman_pitch;
    mon->kalman_gyro_bias    = filter->kalman_bias;
  }

  // 3. Update Kontrol & Tombol
  mon->motor_cmd_current = cmd_current;
  mon->btn_pc13_pressed  = btn_pressed;

  // 4. Update Feedback VESC
  if (vesc != NULL) {
    mon->vesc_rpm            = vesc->rpm;
    mon->vesc_current_actual = vesc->current_actual;
    mon->vesc_duty_cycle     = vesc->duty_cycle;
    mon->vesc_battery_volt   = vesc->battery_voltage;
    mon->vesc_fet_temp       = vesc->fet_temp;
    mon->vesc_motor_temp     = vesc->motor_temp;
  }

  // 5. Update Diagnostik CAN Bus
  if (can_diag != NULL) {
    mon->can_rx_packets = can_diag->rx_packet_count;
    mon->can_last_rx_id = can_diag->last_rx_id;
    mon->can_error_code = can_diag->error_code;
    mon->can_lec        = can_diag->lec;
    mon->can_tec        = can_diag->tec;
    mon->can_rec        = can_diag->rec;
    mon->can_boff       = can_diag->boff;
  }

  // 6. Diagnostik Loop
  mon->dt_ms = dt_ms;
  mon->loop_counter++;
}

void Telemetry_TransmitUART(UART_HandleTypeDef *huart, const System_Monitor_t *mon) {
  char uart_buf[160];

  if (huart == NULL || mon == NULL) {
    return;
  }

  // Integer breakdown untuk sprintf (mencegah bug nano-printf float disabled di GCC STM32)
  int ax_i = (int)mon->accel_x, ax_f = (int)(fabsf(mon->accel_x - (float)ax_i) * 100.0f);
  int ay_i = (int)mon->accel_y, ay_f = (int)(fabsf(mon->accel_y - (float)ay_i) * 100.0f);
  int az_i = (int)mon->accel_z, az_f = (int)(fabsf(mon->accel_z - (float)az_i) * 100.0f);

  int gx_i = (int)mon->gyro_x, gx_f = (int)(fabsf(mon->gyro_x - (float)gx_i) * 10.0f);
  int gy_i = (int)mon->gyro_y, gy_f = (int)(fabsf(mon->gyro_y - (float)gy_i) * 10.0f);
  int gz_i = (int)mon->gyro_z, gz_f = (int)(fabsf(mon->gyro_z - (float)gz_i) * 10.0f);

  int raw_i = (int)mon->pitch_accel_raw,
      raw_f = (int)(fabsf(mon->pitch_accel_raw - (float)raw_i) * 100.0f);
  int comp_i = (int)mon->pitch_complementary,
      comp_f = (int)(fabsf(mon->pitch_complementary - (float)comp_i) * 100.0f);
  int kal_i = (int)mon->kalman_pitch,
      kal_f = (int)(fabsf(mon->kalman_pitch - (float)kal_i) * 100.0f);
  int bias_i = (int)mon->kalman_gyro_bias,
      bias_f = (int)(fabsf(mon->kalman_gyro_bias - (float)bias_i) * 1000.0f);

  const char *ax_sgn = (mon->accel_x < 0.0f && ax_i == 0) ? "-" : "";
  const char *ay_sgn = (mon->accel_y < 0.0f && ay_i == 0) ? "-" : "";
  const char *az_sgn = (mon->accel_z < 0.0f && az_i == 0) ? "-" : "";

  const char *gx_sgn = (mon->gyro_x < 0.0f && gx_i == 0) ? "-" : "";
  const char *gy_sgn = (mon->gyro_y < 0.0f && gy_i == 0) ? "-" : "";
  const char *gz_sgn = (mon->gyro_z < 0.0f && gz_i == 0) ? "-" : "";

  const char *raw_sgn  = (mon->pitch_accel_raw < 0.0f && raw_i == 0) ? "-" : "";
  const char *comp_sgn = (mon->pitch_complementary < 0.0f && comp_i == 0) ? "-" : "";
  const char *kal_sgn  = (mon->kalman_pitch < 0.0f && kal_i == 0) ? "-" : "";
  const char *bias_sgn = (mon->kalman_gyro_bias < 0.0f && bias_i == 0) ? "-" : "";

  // Format data serial UART (Termasuk Raw, Comp, Kalman Pitch & Bias)
  snprintf(uart_buf, sizeof(uart_buf),
          "Ax:%s%d.%02dg Ay:%s%d.%02dg Az:%s%d.%02dg | Gx:%s%d.%01d "
          "Gy:%s%d.%01d Gz:%s%d.%01d | RawP:%s%d.%02d CompP:%s%d.%02d "
          "KalP:%s%d.%02d Bias:%s%d.%03d | Pitch:%s%d.%02d deg\r\n",
          ax_sgn, ax_i, ax_f, ay_sgn, ay_i, ay_f, az_sgn, az_i, az_f, gx_sgn,
          gx_i, gx_f, gy_sgn, gy_i, gy_f, gz_sgn, gz_i, gz_f, raw_sgn, raw_i,
          raw_f, comp_sgn, comp_i, comp_f, kal_sgn, kal_i, kal_f, bias_sgn,
          bias_i, bias_f, kal_sgn, kal_i, kal_f);

  // Kirim string telemetri via USART1 UART (115200 baud)
  HAL_UART_Transmit(huart, (uint8_t *)uart_buf, strlen(uart_buf), 100);
}
