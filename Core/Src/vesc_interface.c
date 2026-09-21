/**
 * @file    vesc_interface.c
 * @brief   Implementasi modul antarmuka komunikasi CAN Bus VESC
 */

#include "vesc_interface.h"

HAL_StatusTypeDef VESC_Interface_InitCANFilter(CAN_HandleTypeDef *hcan) {
  if (hcan == NULL) {
    return HAL_ERROR;
  }

  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000; // Menerima seluruh frame ID
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  return HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
}

void VESC_Interface_ProcessRx(CAN_HandleTypeDef *hcan, VESC_Telemetry_t *telemetry, CAN_Diagnostics_t *diag) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (hcan == NULL || telemetry == NULL || diag == NULL) {
    return;
  }

  // 1. Kuras dan proses seluruh paket yang ada di FIFO0
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
      diag->rx_packet_count++;
      diag->last_rx_id = rx_header.ExtId;

      uint8_t cmd_id = (uint8_t)(rx_header.ExtId >> 8);

      if (cmd_id == CAN_PACKET_STATUS) {
        VESC_Status_1_t s1;
        VESC_CAN_UnpackStatus1(rx_data, &s1);
        telemetry->rpm = s1.rpm;
        telemetry->current_actual = s1.total_current;
        telemetry->duty_cycle = s1.duty_cycle;
      } else if (cmd_id == CAN_PACKET_STATUS_4) {
        VESC_Status_4_t s4;
        VESC_CAN_UnpackStatus4(rx_data, &s4);
        telemetry->fet_temp = s4.fet_temp;
        telemetry->motor_temp = s4.motor_temp;
      } else if (cmd_id == CAN_PACKET_STATUS_5) {
        VESC_Status_5_t s5;
        VESC_CAN_UnpackStatus5(rx_data, &s5);
        telemetry->battery_voltage = s5.input_voltage;
      }
    }
  }

  // 2. Baca register diagnostik ESR (Error Status Register)
  diag->error_code = hcan->Instance->ESR;
  diag->lec  = (uint8_t)((hcan->Instance->ESR >> 4) & 0x07);
  diag->tec  = (uint8_t)((hcan->Instance->ESR >> 16) & 0xFF);
  diag->rec  = (uint8_t)((hcan->Instance->ESR >> 24) & 0xFF);
  diag->boff = (uint8_t)((hcan->Instance->ESR >> 2) & 0x01);
}

void VESC_Interface_SetCurrent(CAN_HandleTypeDef *hcan, uint8_t controller_id, float current) {
  VESC_CAN_SetCurrent(hcan, controller_id, current);
}
