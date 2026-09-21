/**
 * @file    mpu6050.c
 * @brief   Implementasi driver modul IMU MPU6050 untuk STM32F4
 */

#include "mpu6050.h"

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_t *dev) {
  uint8_t check = 0;
  uint8_t data = 0;

  if (dev == NULL || hi2c == NULL) {
    return 0;
  }

  // 1. Baca WHO_AM_I register (0x75). MPU6050 harus mengembalikan 0x68
  if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I_REG, 1, &check, 1, 100) == HAL_OK &&
      check == MPU6050_WHO_AM_I_VAL) {

    // 2. Bangunkan MPU6050 dari Sleep Mode dengan menulis 0x00 ke PWR_MGMT_1 (0x6B)
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 1, &data, 1, 100) != HAL_OK) {
      dev->is_connected = 0;
      return 0;
    }

    // 3. Set Accel Full Scale Range +-2g (Register 0x1C: 0x00)
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 1, &data, 1, 100) != HAL_OK) {
      dev->is_connected = 0;
      return 0;
    }

    // 4. Set Gyro Full Scale Range +-250 deg/s (Register 0x1B: 0x00)
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 1, &data, 1, 100) != HAL_OK) {
      dev->is_connected = 0;
      return 0;
    }

    dev->is_connected = 1;
    return 1;
  }

  dev->is_connected = 0;
  return 0;
}

uint8_t MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *dev) {
  uint8_t rec_data[14];

  if (dev == NULL || hi2c == NULL) {
    return 0;
  }

  // Baca 14 byte sekaligus mulai dari ACCEL_XOUT_H (0x3B) dengan timeout 30 ms
  if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H_REG, 1, rec_data, 14, 30) == HAL_OK) {
    // Gabungkan byte H dan L untuk 16-bit signed integer
    dev->accel_x_raw = (int16_t)(rec_data[0] << 8 | rec_data[1]);
    dev->accel_y_raw = (int16_t)(rec_data[2] << 8 | rec_data[3]);
    dev->accel_z_raw = (int16_t)(rec_data[4] << 8 | rec_data[5]);

    dev->gyro_x_raw  = (int16_t)(rec_data[8] << 8 | rec_data[9]);
    dev->gyro_y_raw  = (int16_t)(rec_data[10] << 8 | rec_data[11]);
    dev->gyro_z_raw  = (int16_t)(rec_data[12] << 8 | rec_data[13]);

    // Konversi ke satuan fisik (g dan deg/s)
    dev->ax = dev->accel_x_raw / MPU6050_ACCEL_SCALE_2G;
    dev->ay = dev->accel_y_raw / MPU6050_ACCEL_SCALE_2G;
    dev->az = dev->accel_z_raw / MPU6050_ACCEL_SCALE_2G;

    dev->gx = dev->gyro_x_raw / MPU6050_GYRO_SCALE_250DPS;
    dev->gy = dev->gyro_y_raw / MPU6050_GYRO_SCALE_250DPS;
    dev->gz = dev->gyro_z_raw / MPU6050_GYRO_SCALE_250DPS;

    // Hitung Sudut Pitch dari Akselerometer: arctan2(-Ax, Az) * 180 / PI
    dev->pitch_accel = atan2f(-dev->ax, dev->az) * RAD_TO_DEG;
    dev->is_connected = 1;
    return 1;
  } else {
    dev->is_connected = 0; // Tandai jika I2C terputus
    return 0;
  }
}

void MPU6050_AutoReconnect(I2C_HandleTypeDef *hi2c, MPU6050_t *dev, uint32_t now_ms, uint32_t interval_ms) {
  static uint32_t last_attempt = 0;

  if (dev == NULL || dev->is_connected) {
    return;
  }

  if (now_ms - last_attempt >= interval_ms) {
    last_attempt = now_ms;
    MPU6050_Init(hi2c, dev);
  }
}
