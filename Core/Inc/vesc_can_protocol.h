/**
 * @file vesc_can_protocol.h
 * @brief VESC CAN Protocol implementation for STM32F4 (bxCAN)
 * @details This file provides functions to encode and decode CAN messages
 * according to the VESC6 CAN protocol document. It handles the command
 * enumeration, data scaling, and Big-Endian byte ordering.
 *
 * Adapted for STM32F446RET6 (bxCAN - CAN_HandleTypeDef).
 *
 * Based on VESC 6 CAN Formats Version 0.1.
 */

#ifndef VESC_CAN_PROTOCOL_H
#define VESC_CAN_PROTOCOL_H

#include "main.h"  // For CAN_HandleTypeDef (STM32F4 HAL)
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Hardware-specific transmit function for STM32F4 bxCAN.
 */
static inline void CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t extended_id,
                                uint8_t *data, uint8_t dlc) {
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;

  tx_header.IDE = CAN_ID_EXT;
  tx_header.ExtId = extended_id;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = dlc;
  tx_header.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
    HAL_CAN_AddTxMessage(hcan, &tx_header, data, &tx_mailbox);
  }
}

/**
 * @brief Enumeration of VESC CAN packet command IDs.
 * From "List of Command Numbers", Page 4.
 */
typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT = 1,
  CAN_PACKET_SET_CURRENT_BRAKE = 2,
  CAN_PACKET_SET_RPM = 3,
  CAN_PACKET_SET_POS = 4,
  CAN_PACKET_FILL_RX_BUFFER = 5,
  CAN_PACKET_FILL_RX_BUFFER_LONG = 6,
  CAN_PACKET_PROCESS_RX_BUFFER = 7,
  CAN_PACKET_PROCESS_SHORT_BUFFER = 8,
  CAN_PACKET_STATUS = 9,
  CAN_PACKET_SET_CURRENT_REL = 10,
  CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
  CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
  CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13,
  CAN_PACKET_STATUS_2 = 14,
  CAN_PACKET_STATUS_3 = 15,
  CAN_PACKET_STATUS_4 = 16,
  CAN_PACKET_PING = 17,
  CAN_PACKET_PONG = 18,
  CAN_PACKET_DETECT_APPLY_ALL_FOC = 19,
  CAN_PACKET_DETECT_APPLY_ALL_FOC_RES = 20,
  CAN_PACKET_CONF_CURRENT_LIMITS = 21,
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS = 22,
  CAN_PACKET_CONF_CURRENT_LIMITS_IN = 23,
  CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN = 24,
  CAN_PACKET_CONF_FOC_ERPMS = 25,
  CAN_PACKET_CONF_STORE_FOC_ERPMS = 26,
  CAN_PACKET_STATUS_5 = 27
} CAN_PACKET_ID;

/**
 * @brief Helper function to place a 32-bit integer into a buffer in Big-Endian format.
 * @param buffer The destination buffer.
 * @param value The 32-bit value to place.
 */
static inline void buffer_append_int32(uint8_t *buffer, int32_t value) {
  buffer[0] = (uint8_t)(value >> 24);
  buffer[1] = (uint8_t)(value >> 16);
  buffer[2] = (uint8_t)(value >> 8);
  buffer[3] = (uint8_t)value;
}

/**
 * @brief Helper function to place a 16-bit integer into a buffer in Big-Endian format.
 * @param buffer The destination buffer.
 * @param value The 16-bit value to place.
 */
static inline void buffer_append_int16(uint8_t *buffer, int16_t value) {
  buffer[0] = (uint8_t)(value >> 8);
  buffer[1] = (uint8_t)value;
}

/**
 * @brief Commands a duty cycle from -1.0 to 1.0.
 * From "Command Duty Cycle", Page 5.
 * @param hcan Pointer to CAN handle (e.g., &hcan1)
 * @param controller_id The ID of the target VESC.
 * @param duty The duty cycle (-1.0 to 1.0).
 */
static inline void VESC_CAN_SetDuty(CAN_HandleTypeDef *hcan,
                                    uint8_t controller_id, float duty) {
  uint32_t extended_id = ((uint32_t)CAN_PACKET_SET_DUTY << 8) | controller_id;
  uint8_t data[4];
  int32_t scaled_duty = (int32_t)(duty * 100000.0f);
  buffer_append_int32(data, scaled_duty);
  CAN_Transmit(hcan, extended_id, data, sizeof(data));
}

/**
 * @brief Commands a current in Amperes.
 * From "Command Set Current", Page 6.
 * @param hcan Pointer to CAN handle (e.g., &hcan1)
 * @param controller_id The ID of the target VESC.
 * @param current The current in Amperes.
 */
static inline void VESC_CAN_SetCurrent(CAN_HandleTypeDef *hcan,
                                       uint8_t controller_id, float current) {
  uint32_t extended_id = ((uint32_t)CAN_PACKET_SET_CURRENT << 8) | controller_id;
  uint8_t data[4];
  int32_t scaled_current = (int32_t)(current * 1000.0f);
  buffer_append_int32(data, scaled_current);
  CAN_Transmit(hcan, extended_id, data, sizeof(data));
}

/**
 * @brief Commands a brake current in Amperes.
 * From "Command Set Current Brake", Page 7.
 * @param hcan Pointer to CAN handle (e.g., &hcan1)
 * @param controller_id The ID of the target VESC.
 * @param brake_current The brake current in Amperes.
 */
static inline void VESC_CAN_SetCurrentBrake(CAN_HandleTypeDef *hcan,
                                            uint8_t controller_id,
                                            float brake_current) {
  uint32_t extended_id = ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8) | controller_id;
  uint8_t data[4];
  int32_t scaled_brake_current = (int32_t)(brake_current * 1000.0f);
  buffer_append_int32(data, scaled_brake_current);
  CAN_Transmit(hcan, extended_id, data, sizeof(data));
}

/**
 * @brief Commands an angular velocity in RPM.
 * From "Command Set RPM", Page 8.
 * @param hcan Pointer to CAN handle (e.g., &hcan1)
 * @param controller_id The ID of the target VESC.
 * @param rpm The desired RPM.
 */
static inline void VESC_CAN_SetRPM(CAN_HandleTypeDef *hcan,
                                   uint8_t controller_id, int32_t rpm) {
  uint32_t extended_id = ((uint32_t)CAN_PACKET_SET_RPM << 8) | controller_id;
  uint8_t data[4];
  buffer_append_int32(data, rpm);
  CAN_Transmit(hcan, extended_id, data, sizeof(data));
}

/**
 * @brief Commands a position. Units are in encoder/hall steps.
 * From "Command Set POS", Page 9.
 * @param hcan Pointer to CAN handle (e.g., &hcan1)
 * @param controller_id The ID of the target VESC.
 * @param pos The desired position.
 */
static inline void VESC_CAN_SetPosition(CAN_HandleTypeDef *hcan,
                                        uint8_t controller_id, int32_t pos) {
  uint32_t extended_id = ((uint32_t)CAN_PACKET_SET_POS << 8) | controller_id;
  uint8_t data[4];
  int32_t scaled_pos = pos * 1000000;
  buffer_append_int32(data, scaled_pos);
  CAN_Transmit(hcan, extended_id, data, sizeof(data));
}

/**
 * @brief Sets the motor and input current limits.
 * From "Set Current Limits", Page 12.
 * @param hcan Pointer to CAN handle (e.g., &hcan1)
 * @param controller_id The ID of the target VESC.
 * @param store If true, saves the limits to NVM (EEPROM).
 * @param max_current The maximum motor current in Amperes.
 * @param min_current The minimum (braking) motor current in Amperes.
 */
static inline void VESC_CAN_SetCurrentLimits(CAN_HandleTypeDef *hcan,
                                             uint8_t controller_id, bool store,
                                             float max_current,
                                             float min_current) {
  CAN_PACKET_ID cmd = store ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS
                            : CAN_PACKET_CONF_CURRENT_LIMITS;
  uint32_t extended_id = ((uint32_t)cmd << 8) | controller_id;
  uint8_t data[8];

  buffer_append_int32(data, (int32_t)(max_current * 1000.0f));
  buffer_append_int32(data + 4, (int32_t)(min_current * 1000.0f));

  CAN_Transmit(hcan, extended_id, data, sizeof(data));
}

/**
 * @brief Sets the input current limits.
 * From "Set Input Current Limits", Page 13.
 * @param hcan Pointer to CAN handle (e.g., &hcan1)
 * @param controller_id The ID of the target VESC.
 * @param store If true, saves the limits to NVM (EEPROM).
 * @param max_input_current The maximum input current in Amperes.
 * @param min_input_current The minimum (regenerative) input current in Amperes.
 */
static inline void VESC_CAN_SetInputCurrentLimits(CAN_HandleTypeDef *hcan,
                                                  uint8_t controller_id,
                                                  bool store,
                                                  float max_input_current,
                                                  float min_input_current) {
  CAN_PACKET_ID cmd = store ? CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN
                            : CAN_PACKET_CONF_CURRENT_LIMITS_IN;
  uint32_t extended_id = ((uint32_t)cmd << 8) | controller_id;
  uint8_t data[8];

  buffer_append_int32(data, (int32_t)(max_input_current * 1000.0f));
  buffer_append_int32(data + 4, (int32_t)(min_input_current * 1000.0f));

  CAN_Transmit(hcan, extended_id, data, sizeof(data));
}

// --- Telemetry Data Structures ---

/**
 * @brief Structure for Status Message 1.
 * From "Status Message 1", Page 15.
 */
typedef struct {
  int32_t rpm;
  float total_current; // in Amperes
  float duty_cycle;    // from -1.0 to 1.0
} VESC_Status_1_t;

/**
 * @brief Structure for Status Message 2.
 * From "Status Message 2", Page 16.
 */
typedef struct {
  float amp_hours;
  float amp_hours_charged;
} VESC_Status_2_t;

/**
 * @brief Structure for Status Message 3.
 * From "Status Message 3", Page 17.
 */
typedef struct {
  float watt_hours;
  float watt_hours_charged;
} VESC_Status_3_t;

/**
 * @brief Structure for Status Message 4.
 * From "Status Message 4", Page 18.
 */
typedef struct {
  float fet_temp;            // in degrees C
  float motor_temp;          // in degrees C
  float total_input_current; // in Amperes
  float pid_pos;             // units unknown
} VESC_Status_4_t;

/**
 * @brief Structure for Status Message 5.
 * From "Status Message 5", Page 19.
 */
typedef struct {
  int32_t tachometer;
  float input_voltage;
} VESC_Status_5_t;

/**
 * @brief Unpacks data from CAN_PACKET_STATUS.
 * @param data Pointer to the 8-byte data payload.
 * @param status Pointer to the destination struct.
 */
static inline void VESC_CAN_UnpackStatus1(const uint8_t *data,
                                          VESC_Status_1_t *status) {
  status->rpm =
      (int32_t)(((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                ((uint32_t)data[2] << 8) | (uint32_t)data[3]);
  status->total_current = (float)((int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5])) / 10.0f;
  status->duty_cycle = (float)((int16_t)(((uint16_t)data[6] << 8) | (uint16_t)data[7])) / 1000.0f;
}

/**
 * @brief Unpacks data from CAN_PACKET_STATUS_2.
 * @param data Pointer to the 8-byte data payload.
 * @param status Pointer to the destination struct.
 */
static inline void VESC_CAN_UnpackStatus2(const uint8_t *data,
                                          VESC_Status_2_t *status) {
  status->amp_hours = (float)((int32_t)(((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                                        ((uint32_t)data[2] << 8) | (uint32_t)data[3])) /
                      10000.0f;
  status->amp_hours_charged = (float)((int32_t)(((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                                                ((uint32_t)data[6] << 8) | (uint32_t)data[7])) /
                              10000.0f;
}

/**
 * @brief Unpacks data from CAN_PACKET_STATUS_3.
 * @param data Pointer to the 8-byte data payload.
 * @param status Pointer to the destination struct.
 */
static inline void VESC_CAN_UnpackStatus3(const uint8_t *data,
                                          VESC_Status_3_t *status) {
  status->watt_hours = (float)((int32_t)(((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                                         ((uint32_t)data[2] << 8) | (uint32_t)data[3])) /
                       10000.0f;
  status->watt_hours_charged = (float)((int32_t)(((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                                                 ((uint32_t)data[6] << 8) | (uint32_t)data[7])) /
                               10000.0f;
}

/**
 * @brief Unpacks data from CAN_PACKET_STATUS_4.
 * @param data Pointer to the 8-byte data payload.
 * @param status Pointer to the destination struct.
 */
static inline void VESC_CAN_UnpackStatus4(const uint8_t *data,
                                          VESC_Status_4_t *status) {
  status->fet_temp = (float)((int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1])) / 10.0f;
  status->motor_temp = (float)((int16_t)(((uint16_t)data[2] << 8) | (uint16_t)data[3])) / 10.0f;
  status->total_input_current =
      (float)((int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5])) / 10.0f;
  status->pid_pos = (float)((int16_t)(((uint16_t)data[6] << 8) | (uint16_t)data[7])) / 50.0f;
}

/**
 * @brief Unpacks data from CAN_PACKET_STATUS_5.
 * @param data Pointer to the 8-byte data payload.
 * @param status Pointer to the destination struct.
 */
static inline void VESC_CAN_UnpackStatus5(const uint8_t *data,
                                          VESC_Status_5_t *status) {
  status->tachometer =
      (int32_t)(((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                ((uint32_t)data[2] << 8) | (uint32_t)data[3]);
  status->input_voltage = (float)((int16_t)(((uint16_t)data[4] << 8) | (uint16_t)data[5])) / 10.0f;
  // Bytes 6 and 7 are reserved
}

#endif // VESC_CAN_PROTOCOL_H
