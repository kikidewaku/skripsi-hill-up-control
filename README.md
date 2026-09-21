# Firmware Sistem Hill-Up Control (Hill-Start Assist) - STM32F446RET6

Repositori ini berisi firmware embedded untuk sistem kendali **Hill-Up Control (Hill-Start Assist)** berbasis mikrokontroler **STM32F446RET6** (ARM Cortex-M4) pada kendaraan uji sasis/powertrain 10 kg dengan motor penggerak BLDC yang dikendalikan oleh controller **VESC** melalui antarmuka **CAN Bus**.

---

## 📌 Fitur Utama Firmware

1. **Arsitektur Modular Berlapis (Clean Embedded C Architecture):**
   - Pemisahan tanggung jawab kode (*Separation of Concerns*) ke dalam modul mandiri di `Core/Inc/` dan `Core/Src/`.
   - Kode utama [main.c](Core/Src/main.c) ringkas, terstruktur, dan hanya berfokus pada inisialisasi perifer serta penjadwalan task.

2. **Estimasi Sudut Kemiringan (Pitch Estimation & Sensor Fusion):**
   - Akuisisi sensor IMU **MPU6050** via antarmuka I2C1 (Fast Mode 400 kHz) dengan fitur *auto-reconnect*.
   - Kalibrasi *Zero-Tare Offset* ($-7.9073^\circ$).
   - Filter Komplementer orde-1 (bobot $\alpha = 0.98$).
   - **2-State Discrete Kalman Filter** untuk pelacakan sudut kemiringan $\theta$ dan estimasi *gyro bias drift* sumbu Y secara realtime.

3. **Penjadwalan Real-Time FreeRTOS & Sampling Rate Dinamis:**
   - Loop kontrol berjalan deterministik di dalam task FreeRTOS `StartDefaultTask` (stack 512 words).
   - Frekuensi sampling dapat diatur dinamis melalui variabel `sample_rate_hz` (default **50.0 Hz** / 20 ms delay). Nilai ini dapat diubah langsung saat runtime via debugger STM32CubeIDE tanpa perlu compile ulang.

4. **Komunikasi CAN Bus Powertrain (VESC CAN Protocol):**
   - Konfigurasi filter hardware 32-bit CAN1 ke FIFO0.
   - Transmisi perintah arus (*torque request*) ke VESC (ID 17).
   - Penerimaan dan parsing paket telemetri VESC: RPM, arus aktual, duty cycle, tegangan baterai, serta temperatur MOSFET/motor.
   - Pemantauan register status diagnostik hardware CAN (`ESR`, `TEC`, `REC`, `LEC`, `BOFF`).

5. **Output Telemetri Serial UART & Live Expressions:**
   - Struktur terpusat `System_Monitor_t monitor` di memori global untuk pemantauan seluruh variabel secara instan melalui STM32CubeIDE *Live Expressions*.
   - Transmisi data serial 115200 bps ke PC / Python Telemetry Dashboard menggunakan format *integer breakdown* (`%d.%02d`) yang aman untuk GCC Newlib-nano tanpa overhead floating-point printf.

---

## 🛠️ Spesifikasi Perangkat Lunak & Kebutuhan

- **Target MCU:** STM32F446RET6 (ARM Cortex-M4, Clock 168 MHz / 180 MHz, Flash 512 KB, SRAM 128 KB)
- **Framework & RTOS:** STM32Cube HAL + FreeRTOS (CMSIS-RTOS v1)
- **IDE:** STM32CubeIDE (v1.14.0 atau lebih baru)
- **Toolchain:** GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`)

---

## 📂 Struktur Direktori Proyek

```
SKRIPSI/
├── Core/
│   ├── Inc/
│   │   ├── main.h                 # Definisi pin, macro, & konstanta hardware CubeMX
│   │   ├── FreeRTOSConfig.h       # Konfigurasi kernel FreeRTOS & alokasi heap
│   │   ├── mpu6050.h              # [BARU] Header driver IMU MPU6050 & tipe MPU6050_t
│   │   ├── pitch_filter.h         # [BARU] Header estimasi kemiringan & Kalman Filter
│   │   ├── vesc_interface.h       # [BARU] Header antarmuka CAN VESC & diagnostik bus
│   │   ├── telemetry.h            # [BARU] Header System_Monitor_t & logging UART
│   │   ├── vesc_can_protocol.h    # Protokol CAN biner VESC6 tingkat rendah (byte packing)
│   │   ├── stm32f4xx_hal_conf.h   # Konfigurasi modul HAL driver STM32
│   │   └── stm32f4xx_it.h         # Header Interrupt Service Routine (ISR)
│   └── Src/
│       ├── main.c                 # Inisialisasi sistem & FreeRTOS task controller
│       ├── mpu6050.c              # [BARU] Implementasi driver MPU6050 (I2C burst read)
│       ├── pitch_filter.c         # [BARU] Implementasi algoritma Kalman & Complementary
│       ├── vesc_interface.c       # [BARU] Implementasi filter hardware CAN & VESC RX/TX
│       ├── telemetry.c            # [BARU] Implementasi format string telemetri serial
│       ├── freertos.c             # Callback FreeRTOS idle task memory
│       ├── stm32f4xx_hal_msp.c    # Low-level HAL MSP hardware init (Clock, GPIO, NVIC)
│       └── stm32f4xx_it.c         # ISR handler interrupt (CAN, I2C, SysTick)
├── Drivers/                       # STM32 HAL Driver & ARM CMSIS Library
├── SKRIPSI.ioc                    # File Konfigurasi Pinout & Clock STM32CubeMX
├── STM32F446RETX_FLASH.ld         # Linker script Flash memory
├── .gitignore                     # Aturan ignore direktori build (Debug/Release)
├── ARCHITECTURE_AND_ROADMAP.md    # [BARU] Dokumentasi arsitektur & panduan teknis pengembang / AI
└── README.md                      # Dokumentasi umum firmware
```

---

## 🧩 Rincian Modul Header (`.h`) & Source (`.c`) Baru

| Modul | File Header & Source | Tanggung Jawab & Fungsi Kunci |
| :--- | :--- | :--- |
| **IMU MPU6050** | [Core/Inc/mpu6050.h](Core/Inc/mpu6050.h)<br>[Core/Src/mpu6050.c](Core/Src/mpu6050.c) | - Inisialisasi I2C1 (range $\pm 2g$, $\pm 250^\circ/\text{s}$).<br>- `MPU6050_Read_All()`: burst read 14 byte dan konversi ke satuan fisik.<br>- `MPU6050_AutoReconnect()`: rekoneksi otomatis jika sensor terlepas. |
| **Pitch Filter** | [Core/Inc/pitch_filter.h](Core/Inc/pitch_filter.h)<br>[Core/Src/pitch_filter.c](Core/Src/pitch_filter.c) | - Kalibrasi zero-tare offset ($-7.9073^\circ$).<br>- `Pitch_Filter_Complementary()`: filter komplementer $\alpha = 0.98$.<br>- `Pitch_Filter_Kalman2State()`: filter Kalman 2-state untuk pelacakan pitch dan estimasi gyro bias drift.<br>- `Pitch_Filter_Update()`: eksekusi terpadu. |
| **VESC CAN** | [Core/Inc/vesc_interface.h](Core/Inc/vesc_interface.h)<br>[Core/Src/vesc_interface.c](Core/Src/vesc_interface.c) | - `VESC_Interface_InitCANFilter()`: inisialisasi filter 32-bit FIFO0 CAN1.<br>- `VESC_Interface_ProcessRx()`: polling & parsing frame telemetri VESC.<br>- `VESC_Interface_SetCurrent()`: transmisi perintah arus target.<br>- Membaca register diagnostik error `ESR`, `TEC`, `REC`, dan `BOFF`. |
| **Telemetri** | [Core/Inc/telemetry.h](Core/Inc/telemetry.h)<br>[Core/Src/telemetry.c](Core/Src/telemetry.c) | - Struktur data terpusat `System_Monitor_t` untuk Live Expressions.<br>- `Telemetry_Update()`: agregasi data seluruh modul.<br>- `Telemetry_TransmitUART()`: format integer-breakdown dan pengiriman ke UART1. |

---

## ⏱️ Penyesuaian Frekuensi Sampling (Dinamis)

Firmware mendukung pengubahan frekuensi sampling tanpa perlu mengubah logika loop:
1. **Melalui Kode:** Ubah nilai variabel di [main.c](Core/Src/main.c):
   ```c
   float sample_rate_hz = 100.0f; // Contoh: diubah ke 100 Hz (delay 10 ms)
   ```
2. **Melalui Live Debugging (STM32CubeIDE):**
   - Buka tab **Live Expressions**.
   - Masukkan variabel `sample_rate_hz`.
   - Ubah nilainya saat firmware sedang berjalan (misal: 20, 50, atau 100). Task RTOS akan langsung menyesuaikan waktu `osDelay` dan periode $\Delta t$.

---

## 🚀 Cara Import & Build di STM32CubeIDE

1. Buka **STM32CubeIDE**.
2. Pilih menu **File -> Import... -> General -> Existing Projects into Workspace**.
3. Arahkan *Root Directory* ke folder repositori ini (`skripsi-hill-up-control-main`).
4. Pastikan opsi *Copy projects into workspace* **TIDAK** dicentang agar tetap sinkron.
5. Klik **Finish**.
6. Tekan tombol **Build (Palu)** atau `Ctrl + B` untuk mengompilasi.
7. Hubungkan debugger ST-Link ke board STM32F446RE Nucleo dan tekan **Run / Debug** (`F11`).

---

## 📖 Panduan Lanjutan & Roadmap untuk Pengembang / AI

Dokumentasi teknis lengkap mengenai diagram alur (*sequence diagram*), riwayat perbaikan bug kritis, serta daftar pekerjaan selanjutnya (**Roadmap Prioritas 1 s.d. 4: Closed-Loop Hill Assist, 4 Sensor IR EXTI, Interrupt CAN, dan Auto-Tare**) tersedia di:
👉 **[ARCHITECTURE_AND_ROADMAP.md](ARCHITECTURE_AND_ROADMAP.md)**
