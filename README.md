# Firmware Sistem Hill-Up Control (Hill-Start Assist) - STM32F446RE

Repositori ini berisi firmware embedded untuk sistem kendali **Hill-Up Control (Hill-Start Assist)** berbasis mikrokontroler **STM32F446RE** (ARM Cortex-M4) pada kendaraan uji sasis/powertrain 10 kg.

---

## 📌 Fitur Utama Firmware

1. **Estimasi Sudut Kemiringan (Pitch Estimation):**
   - Akuisisi sensor IMU **MPU6050** via antarmuka I2C1 (Fast Mode 400 kHz).
   - Filter Komplementer (bobot α = 0.98).
   - 2-State Kalman Filter (estimasi sudut θ dan tracking gyro bias drift).
   - Sampling rate deterministik 50 Hz (periode 20 ms).

2. **Komunikasi CAN Bus Powertrain (VESC CAN Protocol):**
   - Transmisi dan penerimaan data telemetri aktuator BLDC/VESC via CAN1.
   - Perintah kendali torsi/arus (current control) atau duty cycle untuk mencegah backward roll pada tanjakan.

3. **Output Telemetri Serial UART:**
   - Transmisi data serial 115200 bps ke PC / Python Telemetry Dashboard.
   - Format integer breakdown (`%d.%02d`) untuk kompatibilitas GCC Newlib-nano.

---

## 🛠️ Spesifikasi Perangkat Lunak & Kebutuhan

- **IDE:** STM32CubeIDE (v1.14.0 atau lebih baru)
- **Toolchain:** GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`)
- **Framework:** STM32Cube HAL (Hardware Abstraction Layer)
- **Target MCU:** STM32F446RET6 (84 MHz / 180 MHz Clock)

---

## 📂 Struktur Direktori Proyek

```
SKRIPSI/
├── Core/
│   ├── Inc/
│   │   ├── main.h                 # Definisi pin, macro, & konstanta
│   │   └── vesc_can_protocol.h    # Header protokol CAN VESC
│   └── Src/
│       ├── main.c                 # Inisialisasi, loop 50 Hz, Kalman & CAN logic
│       ├── stm32f4xx_hal_msp.c    # Low-level HAL MSP hardware init
│       └── stm32f4xx_it.c         # ISR handler interrupt (CAN, I2C, SysTick)
├── Drivers/                       # STM32 HAL Driver & CMSIS
├── SKRIPSI.ioc                    # Konfigurasi Pinout & Clock STM32CubeMX
├── STM32F446RETX_FLASH.ld         # Linker script Flash memory
├── .gitignore                     # Aturan ignore direktori build (Debug/Release)
└── README.md                      # Dokumentasi firmware
```

---

## 🚀 Cara Import & Build di STM32CubeIDE

1. Buka **STM32CubeIDE**.
2. Pilih menu **File -> Import... -> General -> Existing Projects into Workspace**.
3. Arahkan *Root Directory* ke folder repositori ini (`SKRIPSI`).
4. Pastikan opsi *Copy projects into workspace* **TIDAK** dicentang agar tetap sinkron dengan repositori Git.
5. Klik **Finish**.
6. Klik tombol **Build (Palu)** atau tekan `Ctrl + B` untuk mengompilasi.
7. Hubungkan ST-Link ke board STM32F446RE Nucleo/Discovery dan klik **Run / Debug** (`F11`).




Ini mau test sensor RPM di blackpill, nanti kubuat "SensorIR_RPM.h", boleh taro di .SKRIPSI/Core/Inc/SensorIR_RPM.h
