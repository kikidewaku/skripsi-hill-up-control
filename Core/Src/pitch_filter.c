/**
 * @file    pitch_filter.c
 * @brief   Implementasi modul estimasi sudut kemiringan (Sensor Fusion)
 */

#include "pitch_filter.h"
#include <string.h>

void Pitch_Filter_Init(Pitch_Filter_t *filter, float pitch_offset, float alpha,
                       float q_angle, float q_bias, float r_measure) {
  if (filter == NULL) {
    return;
  }

  filter->pitch_offset = pitch_offset;
  filter->pitch_calibrated = 0.0f;

  filter->alpha = alpha;
  filter->comp_pitch = 0.0f;

  filter->kalman_pitch = 0.0f;
  filter->kalman_bias = 0.0f;

  // Inisialisasi matriks kovarians error P = I
  filter->P[0][0] = 1.0f;
  filter->P[0][1] = 0.0f;
  filter->P[1][0] = 0.0f;
  filter->P[1][1] = 1.0f;

  filter->Q_angle = q_angle;
  filter->Q_bias = q_bias;
  filter->R_measure = r_measure;
}

float Pitch_Filter_Complementary(Pitch_Filter_t *filter, float accel_pitch, float gyro_rate, float dt) {
  if (filter == NULL) {
    return 0.0f;
  }

  filter->comp_pitch = filter->alpha * (filter->comp_pitch + gyro_rate * dt) +
                       (1.0f - filter->alpha) * accel_pitch;
  return filter->comp_pitch;
}

void Pitch_Filter_Kalman2State(Pitch_Filter_t *filter, float accel_pitch, float gyro_rate, float dt) {
  if (filter == NULL) {
    return;
  }

  // 1. TAHAP PREDIKSI (Predict Step)
  // x_priori = x + (u - bias) * dt
  filter->kalman_pitch += (gyro_rate - filter->kalman_bias) * dt;

  // Update Kovarians Error P_priori = F * P * F^T + Q
  filter->P[0][0] += dt * (dt * filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
  filter->P[0][1] -= dt * filter->P[1][1];
  filter->P[1][0] -= dt * filter->P[1][1];
  filter->P[1][1] += filter->Q_bias * dt;

  // 2. TAHAP PEMBARUAN (Update Step / Measurement Update)
  // Inovasi residual y = z - H*x
  float y = accel_pitch - filter->kalman_pitch;

  // Kovarians Inovasi S = H*P*H^T + R
  float S = filter->P[0][0] + filter->R_measure;

  // Gain Kalman K = P * H^T / S
  float K0 = filter->P[0][0] / S;
  float K1 = filter->P[1][0] / S;

  // Update State Posteriori
  filter->kalman_pitch += K0 * y;
  filter->kalman_bias  += K1 * y;

  // Update Kovarians Error P_posteriori = (I - K*H) * P
  float P00_temp = filter->P[0][0];
  float P01_temp = filter->P[0][1];

  filter->P[0][0] -= K0 * P00_temp;
  filter->P[0][1] -= K0 * P01_temp;
  filter->P[1][0] -= K1 * P00_temp;
  filter->P[1][1] -= K1 * P01_temp;
}

void Pitch_Filter_Update(Pitch_Filter_t *filter, float raw_pitch, float gyro_y, float dt) {
  if (filter == NULL) {
    return;
  }

  // 1. Kalibrasi Zero-Tare Offset (-7.91 deg)
  filter->pitch_calibrated = raw_pitch - filter->pitch_offset;

  // 2. Update Complementary Filter
  Pitch_Filter_Complementary(filter, filter->pitch_calibrated, gyro_y, dt);

  // 3. Update 2-State Kalman Filter
  Pitch_Filter_Kalman2State(filter, filter->pitch_calibrated, gyro_y, dt);
}
