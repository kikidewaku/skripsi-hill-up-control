/**
 * @file    mpu6050.h
 * @brief   Driver modul IMU MPU6050 (I2C) untuk STM32F4
 * @details Menangani inisialisasi, pembacaan 14-byte data sensor mentah,
 *          konversi ke besaran fisik (g dan deg/s), serta kalkulasi pitch dasar.
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <math.h>

/* Alamat I2C & Register MPU6050 */
#define MPU6050_ADDR                 0xD0
#define MPU6050_WHO_AM_I_REG         0x75
#define MPU6050_WHO_AM_I_VAL         0x68
#define MPU6050_PWR_MGMT_1_REG       0x6B
#define MPU6050_ACCEL_CONFIG_REG     0x1C
#define MPU6050_GYRO_CONFIG_REG      0x1B
#define MPU6050_ACCEL_XOUT_H_REG     0x3B

/* Faktor Skala Konversi */
#define MPU6050_ACCEL_SCALE_2G       16384.0f  // LSB/g untuk range +-2g
#define MPU6050_GYRO_SCALE_250DPS    131.0f    // LSB/(deg/s) untuk range +-250 deg/s
#define RAD_TO_DEG                   57.2957795f

/**
 * @brief Struktur data pembacaan dan status sensor MPU6050
 */
typedef struct {
  // Data mentah 16-bit signed integer dari register
  int16_t accel_x_raw;
  int16_t accel_y_raw;
  int16_t accel_z_raw;
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;

  // Nilai fisik terkonversi (g dan deg/s)
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;

  // Estimasi sudut pitch akselerometer (arctan2(-Ax, Az))
  float pitch_accel;

  // Status konektivitas (1 = Terhubung, 0 = Gagal/Terputus)
  uint8_t is_connected;
} MPU6050_t;

/**
 * @brief  Inisialisasi konfigurasi awal sensor MPU6050
 * @param  hi2c: Pointer ke handle I2C STM32
 * @param  dev: Pointer ke struktur data MPU6050_t
 * @retval 1 jika berhasil, 0 jika gagal
 */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_t *dev);

/**
 * @brief  Membaca 14 byte register data sensor (Accel, Temp, Gyro) secara burst
 * @param  hi2c: Pointer ke handle I2C STM32
 * @param  dev: Pointer ke struktur data MPU6050_t
 * @retval 1 jika pembacaan berhasil, 0 jika komunikasi timeout/gagal
 */
uint8_t MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *dev);

/**
 * @brief  Menangani rekoneksi otomatis sensor MPU6050 jika sempat terputus
 * @param  hi2c: Pointer ke handle I2C STM32
 * @param  dev: Pointer ke struktur data MPU6050_t
 * @param  now_ms: Nilai timestamp sistem saat ini (HAL_GetTick)
 * @param  interval_ms: Interval percobaan inisialisasi ulang (misal 500 ms)
 */
void MPU6050_AutoReconnect(I2C_HandleTypeDef *hi2c, MPU6050_t *dev, uint32_t now_ms, uint32_t interval_ms);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
