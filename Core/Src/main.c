/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vesc_can_protocol.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
 * @brief Struktur data terpusat untuk memantau seluruh parameter sistem di Live
 * Expressions
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
  uint32_t loop_counter;     // Counter detak loop 50 Hz
  uint8_t mpu6050_connected; // 1 = Terhubung normal, 0 = Gagal

  // 6. Diagnostik CAN Bus
  uint32_t can_rx_packets; // Jumlah total paket CAN yang masuk
  uint32_t can_last_rx_id; // Extended ID paket CAN terakhir
  uint32_t can_error_code; // Raw register ESR
  uint8_t can_lec;         // Last Error: 0=OK, 1=Stuff, 2=Form, 3=ACK, 4=BitRec, 5=BitDom
  uint8_t can_tec;         // Transmit Error Counter (0 s.d. 255)
  uint8_t can_rec;         // Receive Error Counter (0 s.d. 255)
  uint8_t can_boff;        // 1 = Bus-Off (Mati), 0 = Normal/Aktif
} System_Monitor_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0
#define PWR_MGMT_1_REG 0x6B
#define ACCEL_XOUT_H_REG 0x3B
#define VESC_ID 17
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az;
float Gx, Gy, Gz;
float pitch_accel = 0.0f;
float pitch_comp = 0.0f;

// Variabel 2-State Kalman Filter
float Kalman_pitch = 0.0f;
float Kalman_bias = 0.0f;
float P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};

// Parameter Tuning Stokastik Hasil Uji Eksperimen Riil (50 Hz)
float Q_angle = 0.0010f;   // Noise kovarians dinamika proses sudut
float Q_bias = 0.0005f;    // Noise kovarians drift gyro bias Y
float R_measure = 0.0350f; // Kovarians noise pengukuran akselerometer pitch
float alpha = 0.98f;       // Bobot Complementary Filter

// Kalibrasi Zero-Tare Offset (Hasil eksperimen diam -7.91 deg)
float pitch_offset = -7.9073f;
float pitch_calibrated = 0.0f;

uint32_t last_time = 0;
char uart_buf[160];
uint8_t mpu6050_status = 0; // 1 = Terhubung (OK), 0 = Gagal

// Variabel Feedback Telemetri dari VESC
VESC_Status_1_t vesc_status1 = {0};
VESC_Status_4_t vesc_status4 = {0};
VESC_Status_5_t vesc_status5 = {0};
CAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data[8];

// Target Arus Motor (Torque Request untuk VESC)
float motor_cmd_current = 0.0f;

// Variabel Tunggal Terpusat untuk Live Expressions
System_Monitor_t monitor = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void MPU6050_Read_All(void);
float Complementary_Filter(float accel_pitch, float gyro_rate, float dt);
void Kalman_Filter_2State(float accel_pitch, float gyro_rate, float dt,
                          float *out_pitch, float *out_bias);
void CAN_Filter_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100); // Beri waktu 100 ms agar rail daya & sensor MPU6050 stabil
  MPU6050_Init();
  CAN_Filter_Init();     // Aktifkan Filter CAN
  HAL_CAN_Start(&hcan1); // Mulai modul CAN1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    float dt = (now - last_time) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f)
      dt = 0.02f; // Fallback default 50 Hz
    last_time = now;

    // Jika MPU6050 terputus, coba inisialisasi ulang otomatis setiap ~500ms
    if (mpu6050_status == 1) {
      MPU6050_Read_All();
    } else {
      static uint32_t last_reinit = 0;
      if (now - last_reinit > 500) {
        last_reinit = now;
        MPU6050_Init();
      }
    }

    // Kalibrasi Zero Tare Offset (-7.91 deg)
    pitch_calibrated = pitch_accel - pitch_offset;

    // Update Complementary Filter
    pitch_comp = Complementary_Filter(pitch_calibrated, Gy, dt);

    // Update 2-State Kalman Filter (Estimasi Pitch & Gyro Bias Y)
    Kalman_Filter_2State(pitch_calibrated, Gy, dt, &Kalman_pitch, &Kalman_bias);

    // 1. Terima Feedback Telemetri dari VESC (jika ada paket masuk di FIFO0)
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
      if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header,
                               can_rx_data) == HAL_OK) {
        monitor.can_rx_packets++;
        monitor.can_last_rx_id = can_rx_header.ExtId;

        uint8_t cmd_id = (uint8_t)(can_rx_header.ExtId >> 8);

        // Ekstrak paket status VESC (terima baik ID 0 maupun ID 1)
        if (cmd_id == CAN_PACKET_STATUS) {
          VESC_CAN_UnpackStatus1(can_rx_data, &vesc_status1);
        } else if (cmd_id == CAN_PACKET_STATUS_4) {
          VESC_CAN_UnpackStatus4(can_rx_data, &vesc_status4);
        } else if (cmd_id == CAN_PACKET_STATUS_5) {
          VESC_CAN_UnpackStatus5(can_rx_data, &vesc_status5);
        }
      }
    }
    monitor.can_error_code = hcan1.Instance->ESR;
    monitor.can_lec = (uint8_t)((hcan1.Instance->ESR >> 4) & 0x07);
    monitor.can_tec = (uint8_t)((hcan1.Instance->ESR >> 16) & 0xFF);
    monitor.can_rec = (uint8_t)((hcan1.Instance->ESR >> 24) & 0xFF);
    monitor.can_boff = (uint8_t)((hcan1.Instance->ESR >> 2) & 0x01);

    // 2. Baca Tombol User di Pin PC13 (User KEY WeAct: Active-HIGH)
    // Sesuai skematik, tombol K2 menghubungkan PC13 ke VDD33 saat ditekan
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
      motor_cmd_current =
          4.0f; // Tombol DITEKAN (Logika 1 / 3.3V): Kirim 4.0 Ampere
    } else {
      motor_cmd_current =
          0.0f; // Tombol DILEPAS (Logika 0 / Pulldown): 0.0 Ampere (Safety)
    }

    // 3. Kirim Perintah Arus ke VESC via CAN (50 Hz)
    VESC_CAN_SetCurrent(&hcan1, VESC_ID, motor_cmd_current);

    // 4. Perbarui Struktur Terpusat untuk Live Expressions (Cukup panggil
    // 'monitor')
    monitor.accel_x = Ax;
    monitor.accel_y = Ay;
    monitor.accel_z = Az;
    monitor.gyro_x = Gx;
    monitor.gyro_y = Gy;
    monitor.gyro_z = Gz;
    monitor.pitch_accel_raw = pitch_accel;
    monitor.pitch_calibrated = pitch_calibrated;
    monitor.pitch_complementary = pitch_comp;
    monitor.kalman_pitch = Kalman_pitch;
    monitor.kalman_gyro_bias = Kalman_bias;
    monitor.motor_cmd_current = motor_cmd_current;
    monitor.btn_pc13_pressed =
        (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) ? 1 : 0;
    monitor.vesc_rpm = vesc_status1.rpm;
    monitor.vesc_current_actual = vesc_status1.total_current;
    monitor.vesc_duty_cycle = vesc_status1.duty_cycle;
    monitor.vesc_battery_volt = vesc_status5.input_voltage;
    monitor.vesc_fet_temp = vesc_status4.fet_temp;
    monitor.vesc_motor_temp = vesc_status4.motor_temp;
    monitor.dt_ms = dt * 1000.0f;
    monitor.loop_counter++;
    monitor.mpu6050_connected = mpu6050_status;

    // Integer breakdown untuk sprintf (mencegah bug nano-printf float disabled
    // di GCC STM32)
    int ax_i = (int)Ax, ax_f = (int)(fabsf(Ax - (float)ax_i) * 100.0f);
    int ay_i = (int)Ay, ay_f = (int)(fabsf(Ay - (float)ay_i) * 100.0f);
    int az_i = (int)Az, az_f = (int)(fabsf(Az - (float)az_i) * 100.0f);

    int gx_i = (int)Gx, gx_f = (int)(fabsf(Gx - (float)gx_i) * 10.0f);
    int gy_i = (int)Gy, gy_f = (int)(fabsf(Gy - (float)gy_i) * 10.0f);
    int gz_i = (int)Gz, gz_f = (int)(fabsf(Gz - (float)gz_i) * 10.0f);

    int raw_i = (int)pitch_accel,
        raw_f = (int)(fabsf(pitch_accel - (float)raw_i) * 100.0f);
    int comp_i = (int)pitch_comp,
        comp_f = (int)(fabsf(pitch_comp - (float)comp_i) * 100.0f);
    int kal_i = (int)Kalman_pitch,
        kal_f = (int)(fabsf(Kalman_pitch - (float)kal_i) * 100.0f);
    int bias_i = (int)Kalman_bias,
        bias_f = (int)(fabsf(Kalman_bias - (float)bias_i) * 1000.0f);

    const char *ax_sgn = (Ax < 0.0f && ax_i == 0) ? "-" : "";
    const char *ay_sgn = (Ay < 0.0f && ay_i == 0) ? "-" : "";
    const char *az_sgn = (Az < 0.0f && az_i == 0) ? "-" : "";

    const char *gx_sgn = (Gx < 0.0f && gx_i == 0) ? "-" : "";
    const char *gy_sgn = (Gy < 0.0f && gy_i == 0) ? "-" : "";
    const char *gz_sgn = (Gz < 0.0f && gz_i == 0) ? "-" : "";

    const char *raw_sgn = (pitch_accel < 0.0f && raw_i == 0) ? "-" : "";
    const char *comp_sgn = (pitch_comp < 0.0f && comp_i == 0) ? "-" : "";
    const char *kal_sgn = (Kalman_pitch < 0.0f && kal_i == 0) ? "-" : "";
    const char *bias_sgn = (Kalman_bias < 0.0f && bias_i == 0) ? "-" : "";

    // Format data serial UART (Termasuk Raw, Comp, Kalman Pitch & Bias)
    sprintf(uart_buf,
            "Ax:%s%d.%02dg Ay:%s%d.%02dg Az:%s%d.%02dg | Gx:%s%d.%01d "
            "Gy:%s%d.%01d Gz:%s%d.%01d | RawP:%s%d.%02d CompP:%s%d.%02d "
            "KalP:%s%d.%02d Bias:%s%d.%03d | Pitch:%s%d.%02d deg\r\n",
            ax_sgn, ax_i, ax_f, ay_sgn, ay_i, ay_f, az_sgn, az_i, az_f, gx_sgn,
            gx_i, gx_f, gy_sgn, gy_i, gy_f, gz_sgn, gz_i, gz_f, raw_sgn, raw_i,
            raw_f, comp_sgn, comp_i, comp_f, kal_sgn, kal_i, kal_f, bias_sgn,
            bias_i, bias_f, kal_sgn, kal_i, kal_f);

    // Kirim string telemetri via USART1 UART (115200 baud)
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, strlen(uart_buf), 100);

    // Sampling Rate 50 Hz (20 ms delay)
    HAL_Delay(20);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

  /* USER CODE BEGIN CAN1_Init 0 */
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_CanStruct = {0};
  GPIO_CanStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_CanStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_CanStruct.Pull = GPIO_NOPULL;
  GPIO_CanStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_CanStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOA, &GPIO_CanStruct);
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure GPIO pin : PC13 (User Button) */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull =
      GPIO_PULLDOWN; // Wajib PULLDOWN karena tombol terhubung ke VDD33
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Inisialisasi sensor MPU6050 via I2C1
 * @retval None
 */
void MPU6050_Init(void) {
  uint8_t check = 0;
  uint8_t data = 0;

  // Baca WHO_AM_I register (0x75). MPU6050 harus mengembalikan 0x68
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 100) ==
          HAL_OK &&
      check == 0x68) {
    // Bangunkan MPU6050 dari Sleep Mode dengan menulis 0x00 ke PWR_MGMT_1
    // (0x6B)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);

    // Set Accel Full Scale Range +-2g (Register 0x1C)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &data, 1, 100);

    // Set Gyro Full Scale Range +-250 deg/s (Register 0x1B)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);

    mpu6050_status = 1;
  } else {
    mpu6050_status = 0;
  }
}

/**
 * @brief  Membaca 14 byte register mentah (Accel + Temp + Gyro) MPU6050 &
 * menghitung pitch
 * @retval None
 */
void MPU6050_Read_All(void) {
  uint8_t Rec_Data[14];

  // Baca 14 byte sekaligus mulai dari ACCEL_XOUT_H (0x3B) dengan timeout 30 ms
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14,
                       30) == HAL_OK) {
    // Gabungkan byte H dan L untuk 16-bit signed integer
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // Konversi data mentah ke satuan fisik (g-force dan deg/s)
    Ax = Accel_X_RAW / 16384.0f;
    Ay = Accel_Y_RAW / 16384.0f;
    Az = Accel_Z_RAW / 16384.0f;

    Gx = Gyro_X_RAW / 131.0f;
    Gy = Gyro_Y_RAW / 131.0f;
    Gz = Gyro_Z_RAW / 131.0f;

    // Hitung Sudut Pitch dari Akselerometer: arctan2(-Ax, Az) * 180 / PI
    pitch_accel = atan2f(-Ax, Az) * 57.2957795f;
  } else {
    mpu6050_status = 0; // Tandai jika I2C terputus
  }
}

/**
 * @brief  Complementary Filter Orde-1 (alpha = 0.98)
 */
float Complementary_Filter(float accel_pitch, float gyro_rate, float dt) {
  static float comp_state = 0.0f;
  comp_state =
      alpha * (comp_state + gyro_rate * dt) + (1.0f - alpha) * accel_pitch;
  return comp_state;
}

/**
 * @brief  2-State Kalman Filter (Estimasi Sudut Pitch & Gyro Bias Y)
 */
void Kalman_Filter_2State(float accel_pitch, float gyro_rate, float dt,
                          float *out_pitch, float *out_bias) {
  // 1. TAHAP PREDIKSI (Predict Step)
  // pitch_priori = pitch + (gyro_rate - bias) * dt
  Kalman_pitch += (gyro_rate - Kalman_bias) * dt;

  // Update Kovarians Error P_priori = F * P * F^T + Q
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // 2. TAHAP PEMBARUAN (Update Step / Measurement Update)
  // Inovasi residual y = z - H*x
  float y = accel_pitch - Kalman_pitch;

  // Kovarians Inovasi S = H*P*H^T + R
  float S = P[0][0] + R_measure;

  // Gain Kalman K = P * H^T / S
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  // Update State Posteriori
  Kalman_pitch += K0 * y;
  Kalman_bias += K1 * y;

  // Update Kovarians Error P_posteriori = (I - K*H) * P
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K0 * P00_temp;
  P[0][1] -= K0 * P01_temp;
  P[1][0] -= K1 * P00_temp;
  P[1][1] -= K1 * P01_temp;

  *out_pitch = Kalman_pitch;
  *out_bias = Kalman_bias;
}

/**
 * @brief  Konfigurasi Filter Hardware CAN1 untuk menerima paket VESC
 * @retval None
 */
void CAN_Filter_Init(void) {
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000; // Terima semua paket ID
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
