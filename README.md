# robotized-gearbox-controller-for-STM32
Adaptive Robotized Manual Transmission Controller for STM32
This commit adds the complete firmware for the Adaptive Robotized Manual Transmission Controller (STM32F103C8T6).
## 🌟 Project Overview

This open-source controller transforms a standard manual transmission into a fully automated robotized gearbox (AMT). Using two stepper motors for gear selection and a DC actuator for clutch control, the system reads vehicle data via CAN bus and performs automatic or manual gear changes with professional-grade safety features.

### Key Features
- 🎯 Automatic CAN Bus Profiling** – Detects Ford, VW, Toyota, or BMW in 4 seconds without reprogramming
- ⚡ Adaptive Shift Algorithm** – Rev-matching emulation via adaptive clutch hold time
- 🛡️ Dual Redundant Sensors** – AS5600 magnetic encoder + analog potentiometer with automatic failover
-  Intelligent Limp Mode** – Safe operation during sensor/CAN failures with smartphone alerts
- 📱 Bluetooth Control** – HM-10 module for smartphone commands
- 💾 Persistent Storage** – Flash + SD card backup for calibration data
- 🔋 Power Saving** – Automatic sleep mode after 5s inactivity
- 🅿️ Auto-Parking** – Intelligent parking with clutch disengagement

📄 gearbox_controller.c (v1.3 - Optimized Production Build)
   • Compact, optimized version for flashing to the MCU
   • All known issues resolved (USART3 remapped to PC10/PC11, no pin conflicts)
   • Analog tachometer implemented (TIM4, PB6)
   • Stack-safe buffers (UART_MSG_BUF_SIZE = 64)
   • Flash address range check added
   • ~38 KB Flash usage (59% of 64 KB)

📄 gearbox_contr_coment.c (v2.0 - Ultimate Documented Edition)
   • Fully commented version for competition submission and educational use
   • Every function and critical line explained in English
   • Ideal for Hackaday Prize, MDPI Electronics paper, and code review
   • Same functionality as v1.3, just with extensive documentation

 Key Features Implemented:
   • Automatic CAN bus profiling (Ford, VW, Toyota, BMW)
   • Adaptive clutch control with rev-matching emulation
   • Dual redundant clutch sensors (AS5600 + potentiometer)
   • Intelligent Limp Mode with Bluetooth alerts
   • 4 drive modes: Comfort, Normal, Sport, Winter
   • Auto-parking mode with clutch disengagement
   • AWD control module (2WD/AUTO/LOCK/LOW)
   • Current monitoring, watchdog, error logging
   • Power saving sleep mode

📋 Hardware: STM32F103C8T6, TB6600 × 2, NEMA17 × 2, L298N, AS5600, SN65HVD230, HM-10
💰 Total BOM cost: ~$130-180 USD
📄 License: GPL-3.0
