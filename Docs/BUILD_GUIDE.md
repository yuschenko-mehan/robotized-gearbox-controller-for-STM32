# 🛠️ Complete Build Guide

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Hardware Assembly](#hardware-assembly)
3. [Software Setup](#software-setup)
4. [Wiring Diagram](#wiring-diagram)
5. [Calibration](#calibration)
6. [Testing](#testing)
7. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Tools Required
- Soldering iron and solder
- Wire strippers/cutters
- Multimeter
- Screwdrivers (Phillips and flathead)
- 3D printer (optional, for mounts)
- Drill and bits (for enclosure)

### Components
See [BOM] for complete list.
## Hardware Assembly
### Step 1: Power Distribution
12V 5A Supply
├──→ TB6600 #1 (X motor)
├──→ TB6600 #2 (Y motor)
├──→ L298N (Clutch motor)
├──→ Relay (PA8 control)
└──→ LM2596 → 3.3V (Logic)
├──→ STM32
├──→ Sensors
└──→ Modules

**Important**: Use thick wires (16 AWG) for 12V lines, thin wires (22 AWG) for logic.

### Step 2: Stepper Motors

**TB6600 #1 (X-axis – Shift Row):**
TB6600 STM32
PUL+ ──220Ω──→ PA0 (STEP)
PUL- ─────────→ GND
DIR+ ──220Ω──→ PA1 (DIR)
DIR- ─────────→ GND
ENA+ ──220Ω──→ PA2 (ENABLE)
ENA- ─────────→ GND
A+, A- ────────→ NEMA17 coils (pair 1)
B+, B- ────────→ NEMA17 coils (pair 2)

**TB6600 #2 (Y-axis – Gear Selection):**
TB6600 STM32
PUL+ ──220Ω──→ PA3 (STEP)
PUL- ─────────→ GND
DIR+ ──220Ω──→ PA4 (DIR)
DIR- ─────────→ GND
ENA+ ──220Ω──→ PA5 (ENABLE)
ENA- ─────────→ GND

**Current Setting**: Set TB6600 dip switches to 2.0A for NEMA17.

### Step 3: Clutch Actuator (L298N H-Bridge)

**Current Setting**: Set TB6600 dip switches to 2.0A for NEMA17.

### Step 3: Clutch Actuator (L298N H-Bridge)
L298N STM32
IN1 ─────────→ PA7 (DIR1)
IN2 ─────────→ PB0 (DIR2)
ENA ─────────→ PA6 (PWM, TIM2_CH1)
OUT1, OUT2 ────→ DC Motor (Clutch)
+12V ─────────→ 12V Supply
GND ─────────→ Common GND

### Step 4: Sensors

**AS5600 Magnetic Encoder:**

AS5600 STM32
VCC ─────────→ 3.3V
GND ─────────→ GND
SCL ─────────→ PB6 (I2C)
SDA ─────────→ PB7 (I2C)


**Pull-up Resistors**: Add 4.7kΩ resistors on SCL and SDA to 3.3V.

**Limit Switches (Active LOW, Internal Pull-up):**

Switch STM32
Signal ────────→ PB1 (Clutch endstop)
Signal ────────→ PB2 (X left limit)
Signal ────────→ PB3 (X right limit)
Signal ────────→ PB10 (Y front limit)
Signal ────────→ PB11 (Y back limit)
GND ─────────→ GND

**Current Sensors (ACS712 30A):**
ACS712 #1 STM32
OUT ─────────→ PC0 (ADC10) – X motor
VCC ─────────→ 3.3V
GND ─────────→ GND
IP+ / IP- ─────→ In series with motor power line
ACS712 #2 STM32
OUT ─────────→ PC1 (ADC11) – Y motor
ACS712 #3 STM32
OUT ─────────→ PC2 (ADC12) – Clutch motor

**Backup Potentiometer:**

Potentiometer STM32
VCC ─────────→ 3.3V
GND ─────────→ GND
Wiper ─────────→ PC4 (ADC14)


### Step 5: Communication Modules

**CAN Bus (SN65HVD230):**

SN65HVD230 STM32 Vehicle
VCC ─────────→ 3.3V
GND ─────────→ GND
TX ─────────→ PA12 (CAN_TX)
RX ─────────→ PA11 (CAN_RX)
D ─────────→ CAN_H (via 120Ω termination)
R ─────────→ CAN_L (via 120Ω termination)

**Important**: Add 120Ω termination resistor between CAN_H and CAN_L at each end of the bus.

**Bluetooth (HM-10):**
HM-10 STM32
VCC ─────────→ 3.3V
GND ─────────→ GND
TX ─────────→ PC11 (USART3_RX)
RX ──1kΩ────→ PC10 (USART3_TX)

**Voltage Divider**: HM-10 TX is 3.3V (OK), but RX is 3.3V max. Use 1kΩ series resistor.

**USB-UART Console (CP2102/CH340):**

USB-UART STM32
TX ─────────→ PA10 (USART1_RX)
RX ─────────→ PA9 (USART1_TX)
GND ─────────→ GND


### Step 6: Safety & Control

**Emergency Stop Button:**

Button STM32
Signal ────────→ PB8 (EXTI8, Pull-up)
GND ─────────→ GND

**Relay (12V 40A, Emergency Power Off):**

Relay STM32
Control ───────→ PA8 (Output)
NO ─────────→ 12V Supply
COM ─────────→ Motors/Drivers

**LED Indicator (PC13 – Onboard):**

LED (Active LOW)
Cathode ───────→ GND
Anode ───────→ PC13 (via 220Ω)

---

## Software Setup

### 1. Install STM32CubeIDE

1. Download from [st.com](https://www.st.com/en/development-tools/stm32cubeide.html)
2. Install with default settings
3. Install ST-Link V2 drivers (included with CubeIDE)

### 2. Create Project

File → New → STM32 Project
Search: STM32F103C8
Select: STM32F103C8Tx
Name: Robotized_Gearbox
Click: Finish
Accept: "Initialize all peripherals with default Mode? → No"


### 3. Configure CubeMX

Open `Robotized_Gearbox.ioc` and configure:

**System Core:**
- SYS → Debug: **Serial Wire**
- RCC → HSE: **Crystal/Ceramic Resonator**
- Clock Configuration: **HCLK = 72 MHz**

**Peripherals:**
- I2C1: **100 kHz** (AS5600)
- TIM2: **Prescaler=719, Period=999** (PWM 100 Hz)
- TIM4: **Input Capture** (Analog tachometer, PB6)
- ADC1: **4 channels** (PC0, PC1, PC2, PC4)
- CAN1: **500 kbps, Normal Mode**
- USART1: **115200 baud** (PA9, PA10)
- USART3: **9600 baud** (Remap to PC10, PC11)
- SPI2: **Master, Prescaler=256** (PB13-PB15)

**Generate Code** (Ctrl+S)
ST-Link → Blue Pill
3.3V → 3.3
GND → GND
SWDIO → A13
SWCLK → A14
### 4. Flash Firmware

1. Copy `gearbox_controller.c` content to `Core/Src/main.c`
2. Build project: **Project → Build All** (Ctrl+B)
3. Connect ST-Link:
4. Debug → Start (F11)

---

## Wiring Diagram

See for visual diagram.

[Wiring Diagram](Docs/WIRING_DIAGRAM.png)
*Complete pinout and connections for all components*
**Golden Pin Map:**

| Pin | Function | Mode | Notes |
|-----|----------|------|-------|
| PA0 | STEP_X | Output, High Speed | TB6600 #1 |
| PA1 | DIR_X | Output, High Speed | TB6600 #1 |
| PA2 | EN_X | Output, Low Speed | Active LOW |
| PA3 | STEP_Y | Output, High Speed | TB6600 #2 |
| PA4 | DIR_Y | Output, High Speed | TB6600 #2 |
| PA5 | EN_Y | Output, Low Speed | Active LOW |
| PA6 | TIM2_CH1 | PWM | Clutch speed |
| PA7 | DIR1 | Output | H-bridge |
| PA8 | Relay | Output | Emergency power |
| PA9 | USART1_TX | Alternate | Console TX |
| PA10 | USART1_RX | Alternate | Console RX |
| PA11 | CAN1_RX | Alternate | CAN bus |
| PA12 | CAN1_TX | Alternate | CAN bus |
| PB0 | DIR2 | Output | H-bridge |
| PB1 | Clutch Endstop | EXTI, Pull-up | Falling edge |
| PB2 | X Left Limit | EXTI, Pull-up | Falling edge |
| PB3 | X Right Limit | EXTI, Pull-up | Falling edge |
| PB6 | TIM4_CH1 | Input Capture | Analog tach |
| PB8 | Emergency Stop | EXTI, Pull-up | Highest priority |
| PB10 | Y Front Limit | EXTI, Pull-up | Falling edge |
| PB11 | Y Back Limit | EXTI, Pull-up | Falling edge |
| PC0 | ADC10 | Analog | Current X |
| PC1 | ADC11 | Analog | Current Y |
| PC2 | ADC12 | Analog | Current Clutch |
| PC4 | ADC14 | Analog | Backup pot |
| PC10 | USART3_TX | Alternate | Bluetooth TX |
| PC11 | USART3_RX | Alternate | Bluetooth RX |
| PC13 | LED | Output | Active LOW |

---

## Calibration

### 1. Power-On Test

1. **Disconnect motors** initially
2. Apply 12V power
3. Check voltages:
- STM32 3.3V pin: **3.3V ±0.1V**
- TB6600 VCC: **12V**
- L298N +12V: **12V**
4. LED PC13 should blink 3 times (boot sequence)

### 2. UART Console Test

1. Open terminal: **115200 8N1** (PuTTY, screen, minicom)
2. You should see:

=== ROBOTIZED GEARBOX v2.0 ===
CAN profile loaded.
Calibration loaded.

4. Type: `help` – should show command list

### 3. Full Calibration
calibrate
[System will:
Find clutch endstop
Calibrate AS5600 angles
Find X axis limits
Find Y axis limits
Measure backlash
Save to Flash]

### 4. Gear Position Learning

learn_gears
Engine OFF, handbrake ON. Press Enter...
[Move lever to Neutral]
Move lever into 'Neutral (N)'.
Press Enter when lever is in position...
Recorded: X=0 Y=0
Save? (y/n): y
[OK] Saved.
[Repeat for gears 1-5 and Reverse]

---

## Testing

### 1. Sensor Tests

sensor_status
Active: AS5600
AS5600: 2048
Backup raw: 2050
Mapped: 2045

**Test each limit switch:**

status
[Press each limit switch]
[Verify flags change in status output]

### 2. Current Sensor Calibration

With all motors disabled:

calibrate_current
Offsets: X=2048, Y=2047, Clutch=2049

### 3. Manual Shifting Test
gear 1
[Verify motor moves to gear 1]
gear N
[Verify motor returns to neutral]


### 4. Bluetooth Test

1. Pair phone with HM-10 (default PIN: 1234 or 0000)
2. Send: `S` (status)
3. Should receive: `Gear:N RPM:800 Spd:0 Throt:0% Mode:MANUAL Limp:OFF`
---
## Troubleshooting

| Problem | Solution |
|---------|----------|
| No UART output | Check PA9/PA10 connections, verify 115200 baud |
| CAN scan fails | Verify 120Ω termination, check CAN_H/L polarity |
| Steppers don't move | Check EN pins (should be LOW), verify TB6600 power |
| AS5600 not found | Check I2C pull-ups (4.7kΩ on SCL/SDA) |
| Current reading wrong | Re-run calibration with motors disabled |
| Bluetooth not pairing | Check HM-10 power (3.3V), verify PC10/PC11 connections |
| Calibration fails | Check limit switches are wired correctly (NO, pull-up) |

---

## Final Checklist

- [ ] All components soldered/wired correctly
- [ ] Power supply stable (12V & 3.3V)
- [ ] Firmware uploaded successfully
- [ ] UART console responsive
- [ ] All sensors detected
- [ ] Calibration completed
- [ ] Gear positions learned
- [ ] CAN bus communication working
- [ ] Bluetooth pairing successful
- [ ] Emergency stop functional
- [ ] Mechanical movement smooth
- [ ] Safety limits working

**Estimated Total Build Time: 4-6 hours**

---
## Safety Warnings
⚠️ **HIGH CURRENT**: 12V 5A supply can cause burns or fire. Use proper fuses.
⚠️ **MOVING PARTS**: Stepper motors and clutch actuator can cause injury. Keep fingers clear.
⚠️ **VEHICLE SAFETY**: This system controls your transmission. Test thoroughly before road use. Always use handbrake during calibration.
⚠️ **EMERGENCY STOP**: Always have the emergency stop button accessible. Test it before every drive.
---

## 🎮 Control Interface
![Control Flow](Docs/CONTROL_FLOW.png)
*User commands and system responses*

### UART Console Commands (115200 baud)
help                    - Show all commands
status                  - Current system status
calibrate               - Run full calibration
learn_gears             - Teach gear positions
mode auto/manual        - Toggle drive mode
gear N/1-5/R            - Shift to gear
can_scan                - Rescan CAN bus
can_status              - Show CAN profile
sensor_status           - Check sensor health
save                    - Save calibration to Flash
reset                   - Soft reset MCU

## Support

For questions or issues:

**Happy building! 🚗⚙️**
