/**
 * @file    pitch_filter.h
 * @brief   Modul estimasi sudut kemiringan (Pitch Estimation & Sensor Fusion)
 * @details Mengimplementasikan Zero-Tare Offset Calibration, Complementary Filter,
 *          dan 2-State Discrete Kalman Filter untuk pelacakan sudut pitch & gyro bias.
 */

#ifndef PITCH_FILTER_H
#define PITCH_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Struktur data parameter dan state filter estimasi kemiringan
 */
typedef struct {
  // Kalibrasi Zero-Tare Offset
  float pitch_offset;
  float pitch_calibrated;

  // Parameter & State Complementary Filter
  float alpha;
  float comp_pitch;

  // State 2-State Kalman Filter
  float kalman_pitch;
  float kalman_bias;

  // Matriks Kovarians Error Kalman Filter
  float P[2][2];

  // Parameter Tuning Stokastik Kalman Filter
  float Q_angle;    // Noise kovarians dinamika proses sudut
  float Q_bias;     // Noise kovarians drift gyro bias Y
  float R_measure;  // Kovarians noise pengukuran akselerometer
} Pitch_Filter_t;

/**
 * @brief  Inisialisasi konfigurasi filter kemiringan dengan nilai awal
 * @param  filter: Pointer ke struktur Pitch_Filter_t
 * @param  pitch_offset: Offset tare kemiringan awal (derajat)
 * @param  alpha: Bobot Complementary filter (misal 0.98f)
 * @param  q_angle: Noise proses sudut (misal 0.0010f)
 * @param  q_bias: Noise proses bias (misal 0.0005f)
 * @param  r_measure: Noise pengukuran akselerometer (misal 0.0350f)
 */
void Pitch_Filter_Init(Pitch_Filter_t *filter, float pitch_offset, float alpha,
                       float q_angle, float q_bias, float r_measure);

/**
 * @brief  Update estimasi sudut dengan Complementary Filter Orde-1
 * @param  filter: Pointer ke struktur Pitch_Filter_t
 * @param  accel_pitch: Sudut pitch dari akselerometer (derajat)
 * @param  gyro_rate: Laju sudut sumbu Y dari giroskop (deg/s)
 * @param  dt: Waktu cuplik loop (detik)
 * @retval Sudut pitch hasil filter komplementer
 */
float Pitch_Filter_Complementary(Pitch_Filter_t *filter, float accel_pitch, float gyro_rate, float dt);

/**
 * @brief  Update estimasi sudut dengan 2-State Kalman Filter
 * @param  filter: Pointer ke struktur Pitch_Filter_t
 * @param  accel_pitch: Sudut pitch dari akselerometer (derajat)
 * @param  gyro_rate: Laju sudut sumbu Y dari giroskop (deg/s)
 * @param  dt: Waktu cuplik loop (detik)
 */
void Pitch_Filter_Kalman2State(Pitch_Filter_t *filter, float accel_pitch, float gyro_rate, float dt);

/**
 * @brief  Eksekusi lengkap seluruh alur filter (Tare Offset -> Complementary -> Kalman Filter)
 * @param  filter: Pointer ke struktur Pitch_Filter_t
 * @param  raw_pitch: Sudut pitch akselerometer mentah (derajat)
 * @param  gyro_y: Kecepatan sudut sumbu Y giroskop (deg/s)
 * @param  dt: Waktu cuplik loop (detik)
 */
void Pitch_Filter_Update(Pitch_Filter_t *filter, float raw_pitch, float gyro_y, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PITCH_FILTER_H */
