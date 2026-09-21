/**
 * @file    vesc_interface.h
 * @brief   Modul antarmuka komunikasi CAN Bus untuk VESC (CAN1 bxCAN STM32F4)
 * @details Menangani inisialisasi filter hardware CAN, penerimaan dan parsing paket
 *          telemetri VESC, transmisi perintah arus, serta diagnosa bus.
 */

#ifndef VESC_INTERFACE_H
#define VESC_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "vesc_can_protocol.h"
#include <stdint.h>

/**
 * @brief Struktur data feedback telemetri gabungan dari VESC
 */
typedef struct {
  int32_t rpm;              // Kecepatan putar motor BLDC (RPM)
  float current_actual;     // Arus motor riil terbaca (Ampere)
  float duty_cycle;         // Duty cycle motor (-1.0 s.d. 1.0)
  float battery_voltage;    // Tegangan baterai / input bus (Volt)
  float fet_temp;           // Suhu MOSFET controller (deg C)
  float motor_temp;         // Suhu lilitan motor (deg C)
} VESC_Telemetry_t;

/**
 * @brief Struktur data diagnostik register dan lalu lintas CAN Bus
 */
typedef struct {
  uint32_t rx_packet_count; // Jumlah total frame CAN yang berhasil diterima
  uint32_t last_rx_id;      // Extended ID paket terakhir yang masuk
  uint32_t error_code;      // Raw register ESR (Error Status Register)
  uint8_t lec;              // Last Error Code: 0=OK, 1=Stuff, 2=Form, 3=ACK, dst.
  uint8_t tec;              // Transmit Error Counter
  uint8_t rec;              // Receive Error Counter
  uint8_t boff;             // Bus-Off indicator: 1 = Bus-off, 0 = Normal
} CAN_Diagnostics_t;

/**
 * @brief  Konfigurasi hardware filter CAN1 (Accept-All ke FIFO0)
 * @param  hcan: Pointer ke handle CAN STM32
 * @retval HAL_OK jika sukses, HAL_ERROR jika gagal
 */
HAL_StatusTypeDef VESC_Interface_InitCANFilter(CAN_HandleTypeDef *hcan);

/**
 * @brief  Memproses seluruh antrean paket masuk di FIFO0 dan memperbarui telemetri VESC
 * @param  hcan: Pointer ke handle CAN STM32
 * @param  telemetry: Pointer ke struktur telemetri VESC
 * @param  diag: Pointer ke struktur diagnostik CAN
 */
void VESC_Interface_ProcessRx(CAN_HandleTypeDef *hcan, VESC_Telemetry_t *telemetry, CAN_Diagnostics_t *diag);

/**
 * @brief  Mengirimkan perintah arus (torsi) ke VESC melalui CAN Bus
 * @param  hcan: Pointer ke handle CAN STM32
 * @param  controller_id: ID target VESC (misal 17)
 * @param  current: Nilai target arus (Ampere)
 */
void VESC_Interface_SetCurrent(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current);

#ifdef __cplusplus
}
#endif

#endif /* VESC_INTERFACE_H */
