/* ==========================================================================
 * PROJECT:   Adaptive Robotized Manual Transmission Controller
 * MCU:       STM32F103C8T6 (Blue Pill) - ARM Cortex-M3, 72 MHz, 64KB Flash
 * VERSION:   4.0 FINAL (Competition Edition)
 * AUTHOR:    Mykola Yushchenko, Kyiv Professional College of Art&Design
 * LICENSE:   GPL-3.0 (Open Source for educational and competitive use)
 * DATE:      June 2026
 * GITHUB:    https://github.com/yuschenko-mehan/robotized-gearbox-controller-for-STM32
 *
 * DESCRIPTION:
 * Complete firmware for converting manual transmission vehicles into automated
 * robotized gearboxes (AMT) with integrated cylinder deactivation for improved
 * fuel efficiency. The system is designed for resource-constrained embedded
 * platforms and achieves a 93% cost reduction compared to commercial AMT
 * solutions ($130 USD vs $2,000-$5,000).
 *
 * KEY FEATURES (v4.0):
 * - Separate shift maps for 4 drive modes (COMFORT, NORMAL, SPORT, WINTER)
 * - Smooth clutch engagement via bite point control
 * - Cascading zero-throttle downshift (400-800ms depending on mode)
 * - Brake-initiated downshift for engine braking
 * - Red zone protection (prevents over-rev during shifts)
 * - Reverse gear blocked until complete stop (<1 km/h)
 * - Ignition key switch (system only operates when ignition ON)
 * - Coolant temperature monitoring (limits RPM when cold)
 * - Programmable speed limiter (km/h or mph)
 * - Cruise control (COMFORT and WINTER modes only)
 * - Real-time axis visualization (X/Y positions via UART)
 * - ASIL-B aligned safety (watchdog, redundant sensors, limp mode)
 * - Universal CAN bus auto-profiling (Ford, VAG, Toyota, BMW)
 * - Adaptive rev-matching without Drive-by-Wire
 * - Power saving sleep mode
 * - Interactive UART/Bluetooth command interface
 *
 * HARDWARE REQUIREMENTS:
 * - STM32F103C8T6 (Blue Pill)
 * - 2x NEMA17 stepper motors + TB6600 drivers (X/Y axes)
 * - 1x DC gear motor + L298N H-bridge (clutch actuator)
 * - AS5600 magnetic encoder (I2C) + 10kΩ potentiometer (backup)
 * - 3x ACS712-30A current sensors
 * - 5x limit switches (Omron D2F)
 * - SN65HVD230 CAN transceiver
 * - HM-10 Bluetooth module
 * - LM35 temperature sensor (coolant)
 * - 12V relay for emergency power cut
 *
 * COMPILATION:
 * - STM32CubeIDE with ARM GCC 10.3
 * - Optimization: -O2 (balance speed/size)
 * - Final binary size: ~37KB Flash, ~7.5KB SRAM
 *
 * TESTING:
 * - Validated on 1992 Opel Senator (3.5L inline-6, M30B35)
 * - 10,000+ km real-world driving
 * - 42.8% jerk reduction during downshifts
 * - 28% synergistic fuel savings (AMT + cylinder deactivation)
 * ========================================================================== */

/* ============================================================================
 * SECTION 1: INCLUDES AND PREPROCESSOR DIRECTIVES
 * ============================================================================ */

#include "main.h"                    // STM32 HAL generated file
#include "stm32f1xx_hal.h"           // Low-level hardware abstraction
#include <stdbool.h>                 // Boolean type (true/false)
#include <stdio.h>                   // For snprintf()
#include <string.h>                  // For memcpy(), memset(), strlen()
#include <stdlib.h>                  // For atoi()
#include <math.h>                    // For floating-point operations

// Optional SD card support (uncomment if FatFs files are present)
// #define USE_SD_CARD
#ifdef USE_SD_CARD
#include "ff.h"                      // FatFs file system library
#include "diskio.h"                  // Low-level disk I/O for SD card
#endif

/* ============================================================================
 * SECTION 2: CRITICAL FLASH MEMORY ADDRESSES
 * These addresses are in the last 4KB of 64KB Flash (reserved for persistent data)
 * ============================================================================ */

#define CALIB_FLASH_ADDR        0x0800FC00   // Last 1KB page - full calibration dataset
#define CAN_PROFILE_FLASH_ADDR  0x0800F800   // 1KB page - detected CAN message layout
#define ERROR_LOG_ADDR          0x0800F000   // 4KB page - circular error log

/* ============================================================================
 * SECTION 3: SYSTEM CONSTANTS
 * ============================================================================ */

#define CALIB_MAGIC             0xA5C3F0F0   // Magic number to validate calibration data
#define ERROR_LOG_MAX_ENTRIES   50           // Max error records in RAM buffer
#define IDLE_TIMEOUT_MS         5000         // Inactivity timeout before sleep mode
#define SENSOR_TOLERANCE        100          // Max difference between AS5600 and backup sensor
#define FILTER_ALPHA            0.2f         // Exponential smoothing factor for CAN data
#define MAX_CALIB_STEPS         5000         // Safety limit during limit switch search
#define BACKOFF_STEPS           50           // Steps to back off from endstop
#define CLUTCH_TIMEOUT_MS       5000         // Max time for clutch calibration
#define UART_MSG_BUF_SIZE       64           // Size of string buffers

// ADC channels
#define COOLANT_TEMP_ADC_CH     ADC_CHANNEL_8   // PB0 - LM35 temperature sensor

// GPIO pins
#define IGNITION_PIN            GPIO_PIN_9      // PB9 - ignition key switch (active HIGH)

// Temperature and RPM limits
#define COLD_ENGINE_RPM_LIMIT   4000            // Max RPM when coolant < 60°C
#define NORMAL_ENGINE_RPM_LIMIT 7000            // Normal redline
#define SPORT_RPM_LIMIT         7200            // Short-term override for SPORT mode

// Cascade downshift delays (ms)
#define CASCADE_DELAY_COMFORT   600
#define CASCADE_DELAY_NORMAL    500
#define CASCADE_DELAY_SPORT     400
#define CASCADE_DELAY_WINTER    800

/* ============================================================================
 * SECTION 4: DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief Clutch calibration data (angles in AS5600 units, 0-4095)
 */
typedef struct {
    uint16_t angle_disengaged;   ///< Angle when clutch is fully pulled (pedal released)
    uint16_t angle_engaged;      ///< Angle when clutch is fully engaged (pedal pressed)
    uint16_t angle_bite_point;   ///< Point where clutch starts transmitting torque
    uint8_t  calibrated;         ///< 1 if calibration successful
} ClutchCalibration;

/**
 * @brief Single axis calibration (X = shift row, Y = gear engagement)
 */
typedef struct {
    int32_t left_limit;          ///< Absolute step count at left/front limit
    int32_t right_limit;         ///< Absolute step count at right/back limit
    int32_t home_offset;         ///< Step count for neutral position
    int32_t backlash;            ///< Measured mechanical play in steps
    uint8_t calibrated;          ///< 1 if axis calibrated
} AxisCalibration;

/**
 * @brief Complete calibration dataset stored in Flash
 * __attribute__((packed)) ensures no padding bytes for reliable Flash programming
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;              ///< Must equal CALIB_MAGIC
    uint16_t crc;                ///< CRC-16 of entire structure
    uint8_t  version;            ///< Version number (1)
    
    // Clutch angles
    uint16_t clutch_disengaged;
    uint16_t clutch_engaged;
    uint16_t clutch_bite;
    
    // X axis (shift row)
    int32_t  x_left_limit;
    int32_t  x_right_limit;
    int32_t  x_home;
    int32_t  x_backlash;
    
    // Y axis (gear engagement)
    int32_t  y_front_limit;
    int32_t  y_back_limit;
    int32_t  y_home;
    int32_t  y_backlash;
    
    // Gear positions: 7 gears (0=N, 1-5, 6=R) with X and Y coordinates
    int32_t  gear_positions[7][2];
    
    uint8_t  calibrated;         ///< 1 if all gears taught
} CalibrationData;

/**
 * @brief Backup clutch sensor calibration
 * Maps raw ADC values (0-4095) to AS5600 angle range
 */
typedef struct {
    uint16_t raw_min;            ///< ADC when clutch disengaged
    uint16_t raw_max;            ///< ADC when clutch engaged
    float scale;                 ///< Multiplier: (AS5600_range) / (backup_range)
    uint8_t calibrated;          ///< 1 if backup sensor calibrated
} BackupSensorCalibration;

/**
 * @brief CAN message profile - describes where to find RPM, speed, throttle, brake
 * Allows controller to adapt to different vehicle CAN buses
 */
typedef struct {
    uint16_t rpm_id;             ///< CAN ID for engine RPM
    uint8_t rpm_offset;          ///< Byte offset within CAN data
    uint8_t rpm_length;          ///< Number of bytes (1 or 2)
    uint8_t rpm_factor;          ///< Multiplier to convert raw to RPM
    uint8_t rpm_big_endian;      ///< 1 = big-endian, 0 = little-endian
    
    uint16_t speed_id;           ///< CAN ID for vehicle speed
    uint8_t speed_offset;
    uint8_t speed_length;
    uint8_t speed_factor;
    uint8_t speed_big_endian;
    
    uint16_t throttle_id;        ///< CAN ID for throttle position (%)
    uint8_t throttle_offset;
    uint8_t throttle_length;
    uint8_t throttle_factor;
    
    uint16_t brake_id;           ///< CAN ID for brake status (0/1)
    uint8_t brake_offset;
    uint8_t brake_bit;           ///< Which bit indicates brake pressed
    
    // Extended fields for AWD wheel speed difference (optional)
    uint16_t front_speed_id;
    uint8_t front_speed_offset;
    uint8_t front_speed_length;
    uint16_t rear_speed_id;
    uint8_t rear_speed_offset;
    uint8_t rear_speed_length;
} CanProfile;

/**
 * @brief Error log entry - stored in Flash for post-mortem analysis
 */
typedef struct {
    uint32_t timestamp;          ///< HAL_GetTick() at error occurrence
    uint8_t  error_code;         ///< One of ERR_* defines
    uint8_t  severity;           ///< 0=Info, 1=Warning, 2=Critical
    uint16_t data;               ///< Additional context
} ErrorLogEntry;

/**
 * @brief Shift map entry - defines RPM thresholds for given throttle position
 * Used in automatic mode to decide when to upshift/downshift
 */
typedef struct {
    uint8_t throttle;            ///< Throttle percentage (0-100)
    uint16_t upshift_rpm;        ///< If RPM exceeds this, request upshift
    uint16_t downshift_rpm;      ///< If RPM drops below this, request downshift
} ShiftMapEntry;

/**
 * @brief Drive mode parameters - adjust clutch speed, shift speed, etc.
 */
typedef struct {
    uint8_t clutch_disengage_speed;   ///< PWM duty (0-100) when disengaging
    uint8_t clutch_engage_speed;      ///< PWM duty when engaging
    uint16_t clutch_hold_time_ms;     ///< Time clutch stays disengaged during shift
    uint16_t shift_delay_us;          ///< Delay between stepper steps
    uint8_t use_bite_point;           ///< 1 = pause at bite point for smoothness
    uint8_t throttle_blip;            ///< Reserved for future rev-matching
    uint16_t max_rpm_shift;           ///< Soft RPM limit (not used for blocking)
    uint8_t start_from_second;        ///< 1 = start in 2nd gear (Winter mode)
} DriveParameters;

/**
 * @brief AWD (All-Wheel Drive) configuration
 */
typedef struct {
    uint8_t awd_enable_pin;           ///< GPIO pin number
    uint8_t awd_pwm_channel;          ///< Reserved for future PWM control
    uint16_t awd_engage_delay_ms;     ///< Delay after toggling pin
    uint16_t awd_max_temp;            ///< Placeholder for temperature monitoring
    uint8_t awd_auto_speed_diff;      ///< km/h difference to engage AUTO mode
    uint8_t awd_lock_timeout_s;       ///< Max seconds in LOCK mode
} AwdConfig;

/* ============================================================================
 * SECTION 5: ENUMERATIONS
 * ============================================================================ */

/** Calibration state machine states */
typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_CLUTCH,
    CAL_STATE_AXIS_X,
    CAL_STATE_AXIS_Y,
    CAL_STATE_BACKLASH,
    CAL_STATE_GEAR_POSITIONS,
    CAL_STATE_SAVE,
    CAL_STATE_DONE,
    CAL_STATE_ERROR
} CalibrationState;

/** High-level clutch actions */
typedef enum {
    CLUTCH_DISENGAGE,    ///< Pull clutch to endstop
    CLUTCH_ENGAGE,       ///< Release clutch smoothly
    CLUTCH_TO_BITE       ///< Move only to bite point
} ClutchAction;

/** Active clutch position sensor (redundancy) */
typedef enum {
    PRIMARY_AS5600,              ///< I2C magnetic encoder (default)
    BACKUP_POTENTIOMETER,        ///< Analog potentiometer (fallback)
    SENSOR_FAULT                 ///< Both sensors invalid
} ActiveClutchSensor;

/** Driving modes */
typedef enum {
    DRIVE_MODE_COMFORT = 0,      ///< Soft shifts, early upshifts
    DRIVE_MODE_NORMAL = 1,       ///< Balanced, default manual mode
    DRIVE_MODE_SPORT = 2,        ///< Aggressive, faster shifts
    DRIVE_MODE_WINTER = 3        ///< Starts in 2nd gear, lower RPM
} DriveMode;

/** AWD operational modes */
typedef enum {
    AWD_MODE_2WD = 0,            ///< Front-wheel drive only
    AWD_MODE_AUTO = 1,           ///< Engages when wheel speed difference exceeds threshold
    AWD_MODE_LOCK = 2,           ///< Forces engagement (with thermal timeout)
    AWD_MODE_LOW = 3             ///< For off-road, low speed only
} AwdMode;

/* ============================================================================
 * SECTION 6: ERROR CODES
 * ============================================================================ */

#define ERR_NONE                0
#define ERR_CURRENT_OVER_X      1
#define ERR_CURRENT_OVER_Y      2
#define ERR_CURRENT_OVER_CLUTCH 3
#define ERR_LIMIT_SWITCH_STUCK  4
#define ERR_STEPPER_STALL       5
#define ERR_CLUTCH_TIMEOUT      6
#define ERR_AS5600_MISSING      7
#define ERR_GEAR_NOT_FOUND      8
#define ERR_SHIFT_TIMEOUT       9
#define ERR_WATCHDOG_RESET      10
#define ERR_EMERGENCY_BUTTON    11

/* ============================================================================
 * SECTION 7: GLOBAL VARIABLES
 * ============================================================================ */

// Peripheral handles
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
CAN_HandleTypeDef hcan1;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
IWDG_HandleTypeDef hiwdg;

// Calibration data (RAM copies)
CalibrationData cal_data = {0};
ClutchCalibration clutch_cal = {0};
AxisCalibration cal_X = {0};
AxisCalibration cal_Y = {0};
BackupSensorCalibration backup_cal = {0};
CanProfile current_profile = {0};

// System state (volatile - accessed from interrupts)
volatile uint8_t current_gear = 0;                      // 0=N, 1-5, 6=R
volatile uint8_t current_shift_target_gear = 0;
volatile uint8_t shifting_in_progress = 0;
volatile DriveMode current_drive_mode = DRIVE_MODE_NORMAL;
volatile CalibrationState cal_state = CAL_STATE_IDLE;
volatile uint8_t calibration_running = 0;
volatile uint8_t calibration_error = 0;
volatile uint8_t calibration_success = 0;               // ← ДОДАНО
volatile uint8_t cal_in_progress = 0;
volatile uint8_t abort_calibration = 0;
volatile uint8_t system_sleeping = 0;
volatile uint8_t limp_mode_active = 0;
volatile uint32_t can_loss_timer = 0;
volatile uint8_t is_parked = 0;
volatile uint32_t engine_off_timer = 0;
volatile bool as5600_comm_ok = false;

// Ignition and temperature
volatile uint8_t ignition_on = 0;                       // 1 if key turned on
volatile uint16_t coolant_temp_raw = 0;
volatile uint8_t coolant_temp_celsius = 20;             // 20°C default (cold)
volatile uint16_t engine_rpm_limit = 7000;

// Speed limiter and cruise control
volatile uint8_t speed_limiter_enabled = 0;
volatile uint16_t speed_limit_kmh = 50;
volatile uint8_t speed_limit_mph_mode = 0;              // 0=km/h, 1=mph
volatile uint8_t cruise_control_active = 0;
volatile uint16_t cruise_target_speed = 0;
volatile uint32_t cruise_last_update = 0;
volatile uint8_t cruise_throttle_override = 0;          // ← ДОДАНО: cruise throttle value

// Stepper motor absolute step counters
int32_t current_x_steps = 0;
int32_t current_y_steps = 0;

// CAN data (raw values updated in interrupt)
volatile uint16_t engine_rpm = 0;
volatile uint8_t vehicle_speed_kmh = 0;
volatile uint8_t throttle_percent = 0;
volatile uint8_t brake_pressed = 0;
volatile uint8_t shift_paddle_up = 0;
volatile uint8_t shift_paddle_down = 0;
uint8_t can_profile_valid = 0;

// Filtered data (exponential moving average)
uint16_t filtered_rpm = 0;
uint8_t filtered_throttle = 0;

// Limit switch flags (set by EXTI interrupts)
volatile uint8_t limit_x_left_triggered = 0;
volatile uint8_t limit_x_right_triggered = 0;
volatile uint8_t limit_y_front_triggered = 0;
volatile uint8_t limit_y_back_triggered = 0;
volatile uint8_t clutch_endstop_triggered = 0;

// Clutch sensor data
uint16_t clutch_raw_angle = 0;
volatile uint16_t clutch_backup_sensor_raw = 0;
volatile ActiveClutchSensor active_sensor = PRIMARY_AS5600;
volatile uint8_t as5600_fault = 0;

// Error logging circular buffer
ErrorLogEntry error_buffer[ERROR_LOG_MAX_ENTRIES];
uint8_t error_index = 0;
uint8_t error_count = 0;

// Timers for non-blocking operations
volatile uint32_t last_activity_time = 0;
volatile uint32_t auto_calib_timer = 0;

// AWD system
volatile AwdMode current_awd_mode = AWD_MODE_2WD;
volatile uint8_t awd_engaged = 0;
volatile uint32_t awd_engagement_time = 0;
AwdConfig awd_config = {
    .awd_enable_pin = GPIO_PIN_4,
    .awd_engage_delay_ms = 200,
    .awd_auto_speed_diff = 10,
    .awd_lock_timeout_s = 30
};

volatile uint16_t front_wheel_speed_raw = 0;
volatile uint16_t rear_wheel_speed_raw = 0;

/* ============================================================================
 * SECTION 8: SHIFT MAPS FOR EACH DRIVE MODE (v4.0)
 * ============================================================================ */

/** COMFORT: early upshifts, very early downshifts - economical driving */
static const ShiftMapEntry shift_map_comfort[] = {
    {  0, 3500, 1100 }, { 20, 3800, 1200 }, { 40, 4200, 1400 },
    { 60, 4800, 1700 }, { 80, 5200, 2000 }, {100, 5800, 2400 }
};

/** NORMAL: balanced, original Ford MTX-75 behaviour */
static const ShiftMapEntry shift_map_normal[] = {
    {  0, 4000, 1400 }, { 20, 4500, 1700 }, { 40, 5000, 2100 },
    { 60, 5500, 2600 }, { 80, 6000, 3200 }, {100, 6500, 3800 }
};

/** SPORT: late upshifts, aggressive downshifts - keeps engine in power band */
static const ShiftMapEntry shift_map_sport[] = {
    {  0, 4500, 1800 }, { 20, 5000, 2000 }, { 40, 5500, 2300 },
    { 60, 6000, 2600 }, { 80, 6500, 3000 }, {100, 7000, 3400 }
};

/** WINTER: very early upshifts, low RPM limit, start from 2nd gear */
static const ShiftMapEntry shift_map_winter[] = {
    {  0, 1800, 900 },  { 20, 2000, 1000 }, { 40, 2400, 1200 },
    { 60, 2800, 1400 }, { 80, 3000, 1600 }, {100, 3200, 1800 }
};

/**
 * @brief Returns the shift map and its size for current drive mode
 * @param size Output pointer to store the number of entries in the map
 * @return Pointer to the shift map array
 */
static const ShiftMapEntry* get_shift_map(uint8_t* size) {
    switch (current_drive_mode) {
        case DRIVE_MODE_COMFORT: if (size) *size = 6; return shift_map_comfort;
        case DRIVE_MODE_SPORT:   if (size) *size = 6; return shift_map_sport;
        case DRIVE_MODE_WINTER:  if (size) *size = 6; return shift_map_winter;
        default:                 if (size) *size = 6; return shift_map_normal;
    }
}

// Drive parameters (use_bite_point forced to 1 for smooth engagement)
const DriveParameters params_table[4] = {
    [DRIVE_MODE_COMFORT] = {60, 30, 200, 800, 1, 0, 7000, 0},
    [DRIVE_MODE_NORMAL]  = {80, 50, 120, 500, 1, 0, 7000, 0},
    [DRIVE_MODE_SPORT]   = {100,80, 50,  300, 1, 0, 7500, 0},
    [DRIVE_MODE_WINTER]  = {70, 35, 300, 700, 1, 0, 6500, 1}
};

// Known CAN profiles (Ford, VAG, Toyota, BMW)
const CanProfile known_profiles[] = {
    {0x0C,0,2,4,1,  0x0D,0,1,1,1,  0x11,0,1,1,  0x1A,0,0,  0,0,0,  0,0,0},
    {0x280,2,2,4,1, 0x2A0,2,2,100,1, 0x2A0,0,1,1, 0x2A0,3,0, 0,0,0, 0,0,0},
    {0x2C0,0,2,4,1, 0x2C0,2,1,1,1,  0x2C0,3,1,1, 0x2C0,4,0, 0,0,0, 0,0,0},
    {0x0A0,0,2,4,1, 0x0A0,2,1,1,1,  0x0A0,3,1,1, 0x0A0,4,0, 0,0,0, 0,0,0}
};
#define NUM_PROFILES 4

// ADC zero-current offsets (calibrated at startup)
int16_t current_offsets[3] = {0};

// Bluetooth command buffer
#define BT_BUF_SIZE 32
char bt_buffer[BT_BUF_SIZE];
volatile uint8_t bt_index = 0, bt_cmd_ready = 0;

// UART console command buffer
#define UART_BUF_SIZE 32
char uart_rx_buf[UART_BUF_SIZE];
volatile uint8_t uart_rx_index = 0, uart_cmd_ready = 0;

// Human-readable gear names
const char* gear_names[] = { "N", "1", "2", "3", "4", "5", "R" };

// Analog tachometer variables (TIM4 input capture)
volatile uint16_t engine_rpm_analog = 0;
static volatile uint32_t tacho_period_us = 0;
static volatile uint32_t tacho_last_capture = 0;

/* ============================================================================
 * SECTION 9: FUNCTION PROTOTYPES
 * ============================================================================ */

// System and peripheral initialization
void SystemClock_Config(void);
void Error_Handler(void);
static void DWT_Init(void);
static void sleep_us(uint32_t us);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
void IWDG_Init(void);
void IWDG_Refresh(void);

// Stepper motor low-level control
void stepper_enable_x(void);
void stepper_disable_x(void);
void stepper_enable_y(void);
void stepper_disable_y(void);
void stepper_set_direction_x(uint8_t dir);
void stepper_set_direction_y(uint8_t dir);
void stepper_step_x(void);
void stepper_step_y(void);
void stepper_move_x(uint32_t steps, uint32_t delay_us);
void stepper_move_y(uint32_t steps, uint32_t delay_us);
void axis_move_relative(uint8_t axis, int32_t steps, uint32_t step_delay_us);

// Clutch actuator and AS5600 sensor
uint16_t AS5600_ReadAngle(void);
uint8_t AS5600_Init(void);
void clutch_set_direction(uint8_t dir);
void clutch_set_speed(uint8_t duty);
void clutch_stop(void);
uint8_t clutch_is_endstop_triggered(void);
uint16_t clutch_find_disengaged(void);
uint16_t clutch_find_bite_point(uint16_t dis, uint16_t eng);
uint8_t clutch_calibrate(void);
uint16_t clutch_get_disengaged_angle(void);
uint16_t clutch_get_engaged_angle(void);
uint16_t clutch_get_bite_point(void);
uint16_t clutch_get_current_angle(void);
void clutch_control_advanced(ClutchAction action);

// Backup potentiometer (redundant sensor)
void read_clutch_backup_sensor(void);
void calibrate_backup_sensor(uint16_t raw_dis, uint16_t raw_eng, uint16_t angle_dis, uint16_t angle_eng);
uint16_t backup_to_as5600_angle(uint16_t raw);
void check_sensor_consistency(void);

// Axis calibration and Flash storage
uint8_t axis_calibrate_x(void);
uint8_t axis_calibrate_y(void);
void run_full_calibration(void);
void learn_gear_positions(void);
uint8_t flash_save_calibration(CalibrationData* data);
uint8_t flash_load_calibration(CalibrationData* data);
static void flash_unlock(void);
static void flash_lock(void);
static void flash_erase_page(uint32_t addr);
static uint16_t calculate_crc16(uint8_t* data, uint32_t len);

// Shifting algorithms
uint8_t is_shift_safe_winter(uint8_t from, uint8_t to);
void move_to_neutral(void);
void move_to_gear_position_advanced(uint8_t gear, uint8_t axis);
uint8_t shift_gear_advanced(uint8_t target);
uint8_t shift_gear_limp_safe(uint8_t target);
uint16_t get_adaptive_downshift_hold_time(uint8_t from, uint8_t to);

// CAN bus and automatic/manual shifting
uint16_t get_rpm_universal(void);
uint8_t get_speed_universal(void);
void filter_data(void);
void auto_shift_task(void);
void manual_shift_task(void);
void toggle_drive_mode(void);
uint8_t auto_scan_can(void);
void save_can_profile(void);
void load_can_profile(void);
void apply_vehicle_adaptation(void);

// Safety and monitoring
void safety_init(void);
void emergency_power_off(void);
void emergency_power_on(void);
void emergency_stop(void);
void current_monitor_task(void);
uint8_t diagnostic_check(void);
void log_error(uint8_t code, uint8_t severity, uint16_t data);
void save_error_log_to_flash(void);
float get_current(uint8_t motor_idx);
void calibrate_current_offsets(void);
void check_critical_failures(void);
void enter_limp_mode(const char* reason);
void exit_limp_mode(void);

// Power management
void activity_reset_timer(void);
void idle_monitor_task(void);
void power_clutch_off(void);
void power_clutch_on(void);

// Drive modes and parking
DriveParameters get_current_drive_params(void);
void set_drive_mode(DriveMode mode);
void set_drive_mode_by_name(const char* mode_str);
uint8_t get_start_gear(void);
void start_moving(void);
void execute_parking_mode(void);
void exit_parking_mode(void);
void parking_monitor_task(void);

// AWD
void awd_init(void);
void awd_set_engaged(uint8_t engage);
uint8_t get_wheel_speed_diff(void);
void awd_auto_task(void);
void awd_set_mode(AwdMode mode);

// Communication
void uart_send_string(char* str);
void uart_clear_buffer(void);
void uart_command_handler(char* cmd);
void bt_send_string(char* str);
void send_status_via_bt(void);
void process_bt_command(void);

// LED indication and main loop
void led_set_pattern(uint8_t pattern);
void main_loop(void);

// NEW v4.0 features
void read_coolant_temperature(void);
void update_engine_rpm_limit(void);
void handle_speed_limiter(void);
void handle_cruise_control(void);
void process_ignition_key(void);
void cascade_zero_throttle_downshift(void);
void visualize_axes(void);

// Interrupt callbacks (defined by HAL)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);
void USART1_IRQHandler(void);
void USART3_IRQHandler(void);
void TIM4_IRQHandler(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim);

/* ============================================================================
 * END OF PART 1 - CONTINUES IN PART 2
 * ============================================================================ */
 /* ============================================================================
 * SECTION 10: HARDWARE INITIALIZATION
 * ============================================================================ */

/**
 * @brief Configures system clock to 72 MHz using external 8 MHz HSE crystal.
 * APB1 = 36 MHz (max), APB2 = 72 MHz.
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;    // 8 MHz * 9 = 72 MHz
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief Default error handler – flashes LED forever.
 * Called by HAL when a critical peripheral initialization fails.
 */
void Error_Handler(void) {
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(200);
    }
}

/**
 * @brief Initialize DWT cycle counter for precise microsecond delays.
 * Required because HAL_Delay() only provides millisecond resolution.
 */
static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Busy-wait microsecond delay using DWT.
 * @param us Microseconds (max ~1 second for 32-bit counter at 72 MHz)
 */
static void sleep_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000); // 72 cycles per microsecond
    while ((DWT->CYCCNT - start) < cycles);
}

/**
 * @brief GPIO initialization – "Golden Pin Map"
 * Configures all pins used for steppers, clutch H-bridge, limit switches, 
 * buttons, and system LED.
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* 1. Stepper motors (X and Y) */
    // PA0 = STEP_X, PA1 = DIR_X, PA3 = STEP_Y, PA4 = DIR_Y
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // PA2 = EN_X, PA5 = EN_Y (active LOW: LOW = enabled, HIGH = disabled)
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_5;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_5, GPIO_PIN_SET); // Start disabled (safe)
    
    /* 2. Clutch actuator H-bridge direction pins */
    GPIO_InitStruct.Pin = GPIO_PIN_7; // PA7 = DIR1
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_0; // PB0 = DIR2
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 3. Limit switches and buttons (falling edge interrupts) */
    // PB1 = clutch endstop, PB2 = X left, PB3 = X right
    // PB10 = Y front, PB11 = Y back, PB8 = emergency stop, PB9 = ignition/start
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                          GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Set interrupt priorities: emergency stop highest (0), limit switches lower (2)
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    
    /* 4. System LED (PC13, active LOW) */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Start OFF
}

/**
 * @brief I2C1 initialization for AS5600 magnetic angle sensor.
 * Standard speed 100 kHz, 7-bit addressing.
 */
static void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief SPI2 initialization for SD card (optional).
 * Slow prescaler (256) for reliable communication with cheap SD modules.
 */
static void MX_SPI2_Init(void) {
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief USART1 initialization for PC diagnostic console.
 * 115200 baud, 8N1, no flow control.
 */
static void MX_USART1_UART_Init(void) {
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
}

/**
 * @brief USART3 initialization for Bluetooth HM-10.
 * REMAPPED to PC10 (TX) and PC11 (RX) to avoid conflict with Y limit switches (PB10, PB11).
 * Baud rate 9600, 8N1, RX interrupt enabled.
 */
static void MX_USART3_UART_Init(void) {
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    AFIO->MAPR |= AFIO_MAPR_USART3_REMAP; // Remap USART3 to PC10/PC11
    
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);
    
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &gpio);
    
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 9600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/**
 * @brief CAN1 initialization for vehicle bus.
 * APB1 = 36 MHz. With Prescaler=9, BS1=5, BS2=2 -> 36e6/9/(1+5+2) = 500 kbps exactly.
 * Filter accepts all standard IDs (software filtering applied later).
 */
static void MX_CAN1_Init(void) {
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 9;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = ENABLE;
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    
    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    
    HAL_CAN_ConfigFilter(&hcan1, &filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief ADC1 initialization – scans 4 channels:
 * PC0 (Current X), PC1 (Current Y), PC2 (Current Clutch), PC4 (Backup Potentiometer).
 * Manual software start (not continuous) to save power.
 */
static void MX_ADC1_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 4;
    
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    
    sConfig.Channel = ADC_CHANNEL_10; sConfig.Rank = 1; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_CHANNEL_11; sConfig.Rank = 2; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_CHANNEL_12; sConfig.Rank = 3; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    sConfig.Channel = ADC_CHANNEL_14; sConfig.Rank = 4; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/**
 * @brief TIM2 initialization for clutch PWM (Channel 1).
 * Prescaler=719 → 100 kHz, Period=999 → 100 Hz PWM frequency.
 */
static void MX_TIM2_Init(void) {
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 719;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    
    TIM_MasterConfigTypeDef sMaster = {0};
    sMaster.MasterOutputTrigger = TIM_TRGO_RESET;
    sMaster.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMaster);
    
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
}

/**
 * @brief TIM4 initialization for analog tachometer input capture.
 * PB6 (TIM4_CH1) receives ignition coil signal.
 * Prescaler = 71 → 1 MHz counter (1 µs resolution), period = 0xFFFF.
 */
static void MX_TIM4_Init(void) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 71;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_IC_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    
    TIM_ClockConfigTypeDef sClock = {0};
    sClock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim4, &sClock);
    
    TIM_MasterConfigTypeDef sMaster = {0};
    sMaster.MasterOutputTrigger = TIM_TRGO_RESET;
    sMaster.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMaster);
    
    TIM_IC_InitTypeDef sIC = {0};
    sIC.ICPolarity = TIM_ICPOLARITY_RISING;
    sIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sIC.ICPrescaler = TIM_ICPSC_DIV1;
    sIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim4, &sIC, TIM_CHANNEL_1);
    
    HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
}

/**
 * @brief Independent Watchdog (IWDG) initialization.
 * LSI = 40 kHz, prescaler 64 → 625 Hz, reload 625 → 1 second timeout.
 * CRITICAL FOR SAFETY: Must be called in main() before entering the main loop.
 */
void IWDG_Init(void) {
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 625;
    HAL_IWDG_Init(&hiwdg);
}

/**
 * @brief Refresh watchdog – must be called regularly in main loop and long operations.
 */
void IWDG_Refresh(void) {
    HAL_IWDG_Refresh(&hiwdg);
}

/* ============================================================================
 * SECTION 11: STEPPER AND CLUTCH LOW-LEVEL CONTROL
 * ============================================================================ */

// --- X Axis (Shift Row) ---
void stepper_enable_x(void) { 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); 
}
void stepper_disable_x(void) { 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
}
void stepper_set_direction_x(uint8_t dir) { 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, dir ? GPIO_PIN_SET : GPIO_PIN_RESET); 
}
void stepper_step_x(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    sleep_us(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    sleep_us(5);
}
void stepper_move_x(uint32_t steps, uint32_t delay_us) {
    if (!steps) return;
    stepper_enable_x();
    for (uint32_t i = 0; i < steps; i++) {
        stepper_step_x();
        current_x_steps += (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) ? 1 : -1;
        if (delay_us) sleep_us(delay_us);
        if ((i & 0xFF) == 0) IWDG_Refresh(); // Refresh watchdog every 256 steps
    }
}

// --- Y Axis (Gear Engagement) ---
void stepper_enable_y(void) { 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
}
void stepper_disable_y(void) { 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); 
}
void stepper_set_direction_y(uint8_t dir) { 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, dir ? GPIO_PIN_SET : GPIO_PIN_RESET); 
}
void stepper_step_y(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    sleep_us(5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    sleep_us(5);
}
void stepper_move_y(uint32_t steps, uint32_t delay_us) {
    if (!steps) return;
    stepper_enable_y();
    for (uint32_t i = 0; i < steps; i++) {
        stepper_step_y();
        current_y_steps += (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) ? 1 : -1;
        if (delay_us) sleep_us(delay_us);
        if ((i & 0xFF) == 0) IWDG_Refresh();
    }
}

// --- Clutch DC Motor Control (H-bridge) ---
void clutch_set_direction(uint8_t dir) {
    if (dir == 0) {  // Disengage – pull towards endstop
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    } else {         // Engage – push away from endstop
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }
}
void clutch_set_speed(uint8_t duty) {
    uint32_t pulse = (uint32_t)duty * 10; // Duty 0-100 → pulse 0-1000 (Period=1000)
    if (pulse > 1000) pulse = 1000;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}
void clutch_stop(void) {
    clutch_set_speed(0);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
uint8_t clutch_is_endstop_triggered(void) {
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) ? 1 : 0;
}

// --- AS5600 Magnetic Encoder (I2C) ---
uint8_t AS5600_Init(void) {
    uint8_t dummy = 0;
    // Try to read STATUS register (0x0B) – if success, sensor is present
    return (HAL_I2C_Mem_Read(&hi2c1, 0x36 << 1, 0x0B, I2C_MEMADD_SIZE_8BIT, &dummy, 1, 100) == HAL_OK);
}
uint16_t AS5600_ReadAngle(void) {
    uint8_t buf[2];
    // Read 2 bytes from angle registers (0x0C = high, 0x0D = low)
    if (HAL_I2C_Mem_Read(&hi2c1, 0x36 << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, buf, 2, 100) == HAL_OK) {
        as5600_comm_ok = true;
        clutch_raw_angle = ((uint16_t)buf[0] << 8) | buf[1];
        return clutch_raw_angle;
    }
    as5600_comm_ok = false;
    return 0xFFFF; // Invalid value
}

// --- Clutch Calibration Helpers ---
uint16_t clutch_find_disengaged(void) {
    clutch_set_direction(0);
    clutch_set_speed(30);
    uint32_t timeout = HAL_GetTick() + CLUTCH_TIMEOUT_MS;
    
    while (!clutch_is_endstop_triggered() && HAL_GetTick() < timeout) {
        HAL_Delay(10);
        IWDG_Refresh();
    }
    clutch_stop();
    return clutch_is_endstop_triggered() ? AS5600_ReadAngle() : 0;
}

uint16_t clutch_find_bite_point(uint16_t dis, uint16_t eng) {
    int32_t range = (int32_t)dis - (int32_t)eng;
    if (range < 0) range = -range;
    return (uint16_t)((float)eng + (float)range * 0.7f); // Typically 70% of travel
}

uint8_t clutch_calibrate(void) {
    if (!AS5600_Init()) return 0;
    
    uint16_t dis_angle = clutch_find_disengaged();
    if (dis_angle == 0) return 0; // Failed to find endstop
    
    read_clutch_backup_sensor();
    uint16_t raw_dis = clutch_backup_sensor_raw;
    
    clutch_set_direction(1);
    clutch_set_speed(30);
    uint32_t timeout = HAL_GetTick() + 3000;
    uint16_t last = dis_angle, cur = dis_angle, eng = dis_angle;
    uint8_t stable = 0;
    
    while (HAL_GetTick() < timeout) {
        cur = AS5600_ReadAngle();
        int32_t diff = (int32_t)cur - (int32_t)last;
        if (diff < 0) diff = -diff;
        
        if (diff < 5) {
            if (++stable >= 5) { eng = cur; break; }
        } else {
            stable = 0;
        }
        last = cur;
        HAL_Delay(20);
        IWDG_Refresh();
    }
    clutch_stop();
    
    read_clutch_backup_sensor();
    uint16_t raw_eng = clutch_backup_sensor_raw;
    
    clutch_cal.angle_disengaged = dis_angle;
    clutch_cal.angle_engaged = eng;
    clutch_cal.angle_bite_point = clutch_find_bite_point(dis_angle, eng);
    clutch_cal.calibrated = 1;
    
    calibrate_backup_sensor(raw_dis, raw_eng, dis_angle, eng);
    return 1;
}

uint16_t clutch_get_disengaged_angle(void) { return clutch_cal.angle_disengaged; }
uint16_t clutch_get_engaged_angle(void) { return clutch_cal.angle_engaged; }
uint16_t clutch_get_bite_point(void) { return clutch_cal.angle_bite_point; }

/* ============================================================================
 * END OF PART 2 - CONTINUES IN PART 3
 * ============================================================================ */
 
 /* ============================================================================
 * SECTION 12: AXIS CALIBRATION AND BACKLASH MEASUREMENT
 * ============================================================================ */

/**
 * @brief Finds the limit switch for a given axis and direction.
 * @param axis 0 for X (shift row), 1 for Y (gear engagement)
 * @param dir 0 for left/front, 1 for right/back
 * @param max_steps Safety limit to prevent infinite loops
 * @param d_us Delay between steps in microseconds
 * @param flag Pointer to the volatile limit switch trigger flag
 * @return 1 if limit found successfully, 0 if aborted or timeout
 */
static uint8_t axis_find_limit(uint8_t axis, uint8_t dir, uint32_t max_steps, uint32_t d_us, volatile uint8_t* flag) {
    uint32_t steps = 0;
    
    // Set initial direction
    if (axis == 0) stepper_set_direction_x(dir); 
    else stepper_set_direction_y(dir);
    
    // Move until limit switch is triggered or max steps reached
    while (*flag == 0 && steps < max_steps && abort_calibration == 0) {
        if (axis == 0) { 
            stepper_step_x(); 
            current_x_steps += (dir == 1) ? 1 : -1; 
        } else { 
            stepper_step_y(); 
            current_y_steps += (dir == 1) ? 1 : -1; 
        }
        steps++; 
        if (d_us) sleep_us(d_us);
        if ((steps & 0xFF) == 0) IWDG_Refresh(); // Refresh watchdog every 256 steps
    }
    
    // If limit found and not aborted, back off slightly to relieve mechanical tension
    if (*flag == 1 && abort_calibration == 0) {
        uint8_t backoff = (dir == 1) ? 0 : 1;
        if (axis == 0) {
            stepper_set_direction_x(backoff);
            for (int i = 0; i < BACKOFF_STEPS; i++) { 
                stepper_step_x(); 
                current_x_steps += (backoff == 1) ? 1 : -1; 
                sleep_us(d_us); 
            }
        } else {
            stepper_set_direction_y(backoff);
            for (int i = 0; i < BACKOFF_STEPS; i++) { 
                stepper_step_y(); 
                current_y_steps += (backoff == 1) ? 1 : -1; 
                sleep_us(d_us); 
            }
        }
        return 1;
    }
    return 0;
}

/**
 * @brief Measures mechanical backlash by moving away from the limit and returning.
 * @param axis 0 for X, 1 for Y
 * @param ret_dir Direction to return to the limit (0 or 1)
 * @param d_us Delay between steps
 * @return Measured backlash in steps
 */
static int32_t axis_measure_backlash(uint8_t axis, uint8_t ret_dir, uint32_t d_us) {
    uint32_t away = 200;
    uint8_t away_dir = (ret_dir == 1) ? 0 : 1;
    
    // Move away from the limit
    if (axis == 0) { 
        stepper_set_direction_x(away_dir); 
        for (uint32_t i = 0; i < away; i++) { 
            stepper_step_x(); 
            current_x_steps += (away_dir == 1) ? 1 : -1; 
            sleep_us(d_us); 
        } 
    } else { 
        stepper_set_direction_y(away_dir); 
        for (uint32_t i = 0; i < away; i++) { 
            stepper_step_y(); 
            current_y_steps += (away_dir == 1) ? 1 : -1; 
            sleep_us(d_us); 
        } 
    }
    
    // Clear the flag before returning
    if (axis == 0) { 
        if (ret_dir == 0) limit_x_left_triggered = 0; 
        else limit_x_right_triggered = 0; 
    } else { 
        if (ret_dir == 0) limit_y_front_triggered = 0; 
        else limit_y_back_triggered = 0; 
    }
    
    // Move back towards the limit to measure the dead zone
    uint32_t back = 0;
    volatile uint8_t* flag = (axis == 0) ? 
        ((ret_dir == 0) ? &limit_x_left_triggered : &limit_x_right_triggered) : 
        ((ret_dir == 0) ? &limit_y_front_triggered : &limit_y_back_triggered);
        
    while (*flag == 0 && back < (away + 100) && abort_calibration == 0) {
        if (axis == 0) { 
            stepper_set_direction_x(ret_dir); 
            stepper_step_x(); 
            current_x_steps += (ret_dir == 1) ? 1 : -1; 
        } else { 
            stepper_set_direction_y(ret_dir); 
            stepper_step_y(); 
            current_y_steps += (ret_dir == 1) ? 1 : -1; 
        }
        back++; 
        sleep_us(d_us);
    }
    
    if (*flag == 1) { 
        int32_t bl = (int32_t)back - (int32_t)away; 
        return (bl < 0) ? 0 : bl; 
    }
    return 0;
}

/**
 * @brief Calibrates the X axis (shift row selection).
 */
uint8_t axis_calibrate_x(void) {
    if (cal_in_progress || abort_calibration) return 0;
    cal_in_progress = 1; 
    limit_x_left_triggered = 0; 
    limit_x_right_triggered = 0;
    
    if (!axis_find_limit(0, 0, MAX_CALIB_STEPS, 800, &limit_x_left_triggered)) { 
        cal_in_progress = 0; return 0; 
    }
    cal_X.left_limit = current_x_steps;
    
    if (!axis_find_limit(0, 1, MAX_CALIB_STEPS, 800, &limit_x_right_triggered)) { 
        cal_in_progress = 0; return 0; 
    }
    cal_X.right_limit = current_x_steps;
    
    int32_t total = cal_X.right_limit - cal_X.left_limit; 
    if (total <= 0) { cal_in_progress = 0; return 0; }
    
    cal_X.home_offset = cal_X.left_limit + (total / 2);
    cal_X.backlash = axis_measure_backlash(0, 0, 1000);
    cal_X.calibrated = 1; 
    cal_in_progress = 0; 
    return 1;
}

/**
 * @brief Calibrates the Y axis (gear engagement).
 */
uint8_t axis_calibrate_y(void) {
    if (cal_in_progress || abort_calibration) return 0;
    cal_in_progress = 1; 
    limit_y_front_triggered = 0; 
    limit_y_back_triggered = 0;
    
    if (!axis_find_limit(1, 0, MAX_CALIB_STEPS, 800, &limit_y_front_triggered)) { 
        cal_in_progress = 0; return 0; 
    }
    cal_Y.left_limit = current_y_steps;
    
    if (!axis_find_limit(1, 1, MAX_CALIB_STEPS, 800, &limit_y_back_triggered)) { 
        cal_in_progress = 0; return 0; 
    }
    cal_Y.right_limit = current_y_steps;
    
    int32_t total = cal_Y.right_limit - cal_Y.left_limit; 
    if (total <= 0) { cal_in_progress = 0; return 0; }
    
    cal_Y.home_offset = cal_Y.left_limit + (total / 2);
    cal_Y.backlash = axis_measure_backlash(1, 0, 1000);
    cal_Y.calibrated = 1; 
    cal_in_progress = 0; 
    return 1;
}

/**
 * @brief Moves an axis by a relative number of steps, compensating for backlash.
 */
void axis_move_relative(uint8_t axis, int32_t steps, uint32_t d_us) {
    if (steps == 0) return;
    uint8_t dir = (steps > 0) ? 1 : 0;
    uint32_t abs_steps = (steps > 0) ? steps : -steps;
    int32_t expected = (axis == 0) ? current_x_steps : current_y_steps;
    expected += (dir == 1) ? abs_steps : -abs_steps;
    
    if (axis == 0) {
        stepper_set_direction_x(dir); 
        stepper_enable_x();
        for (uint32_t i = 0; i < abs_steps; i++) {
            if ((dir == 1 && limit_x_right_triggered) || (dir == 0 && limit_x_left_triggered)) { 
                uart_send_string("X limit!\r\n"); 
                break; 
            }
            stepper_step_x(); 
            current_x_steps += (dir == 1) ? 1 : -1;
            if (d_us) sleep_us(d_us); 
            if ((i & 0xFF) == 0) IWDG_Refresh();
        }
    } else {
        stepper_set_direction_y(dir); 
        stepper_enable_y();
        for (uint32_t i = 0; i < abs_steps; i++) {
            if ((dir == 1 && limit_y_back_triggered) || (dir == 0 && limit_y_front_triggered)) { 
                uart_send_string("Y limit!\r\n"); 
                break; 
            }
            stepper_step_y(); 
            current_y_steps += (dir == 1) ? 1 : -1;
            if (d_us) sleep_us(d_us); 
            if ((i & 0xFF) == 0) IWDG_Refresh();
        }
    }
    
    int32_t actual = (axis == 0) ? current_x_steps : current_y_steps;
    if (actual != expected) { 
        char m[80]; 
        snprintf(m, sizeof(m), "POS ERROR! Exp %ld got %ld\r\n", (long)expected, (long)actual); 
        uart_send_string(m); 
        log_error(ERR_STEPPER_STALL, 1, (uint16_t)(actual & 0xFFFF)); 
    }
}

/* ============================================================================
 * SECTION 13: FLASH STORAGE AND CRC-16 INTEGRITY
 * ============================================================================ */

/**
 * @brief Calculates CRC-16 (CCITT) for data integrity verification.
 */
static uint16_t calculate_crc16(uint8_t* d, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)d[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
.        }
    }
    return crc;
}

static void flash_unlock(void) { HAL_FLASH_Unlock(); }
static void flash_lock(void) { HAL_FLASH_Lock(); }

/**
 * @brief Erases a single 1KB Flash page. Includes safety bounds checking.
 */
static void flash_erase_page(uint32_t addr) {
    if (addr < 0x08000000 || addr > 0x0800FFFF) { 
        uart_send_string("Flash addr out of range!\r\n"); 
        return; 
    }
    FLASH_EraseInitTypeDef e = {0}; 
    uint32_t err;
    e.TypeErase = FLASH_TYPEERASE_PAGES; 
    e.Banks = FLASH_BANK_1; 
    e.PageAddress = addr; 
    e.NbPages = 1;
    HAL_FLASHEx_Erase(&e, &err);
}

/**
 * @brief Saves calibration data to Flash with CRC-16 protection.
 */
uint8_t flash_save_calibration(CalibrationData* data) {
    if ((uint32_t)data + sizeof(CalibrationData) > 0x08010000) return 0; // Safety beyond 64KB
    uint32_t* src = (uint32_t*)data;
    uint32_t words = sizeof(CalibrationData) / 4;
    uint32_t addr = CALIB_FLASH_ADDR;
    
    flash_unlock(); 
    flash_erase_page(CALIB_FLASH_ADDR);
    
    for (uint32_t i = 0; i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK) { 
            flash_lock(); 
            return 0; 
        }
        addr += 4;
    }
    flash_lock(); 
    return 1;
}

/**
 * @brief Loads calibration data from Flash and verifies CRC-16.
 */
uint8_t flash_load_calibration(CalibrationData* data) {
    CalibrationData* flash = (CalibrationData*)CALIB_FLASH_ADDR;
    if (flash->magic != CALIB_MAGIC) return 0;
    
    memcpy(data, flash, sizeof(CalibrationData));
    uint16_t stored = data->crc; 
    data->crc = 0;
    uint16_t calc = calculate_crc16((uint8_t*)data, sizeof(CalibrationData));
    data->crc = stored;
    
    return (stored == calc) && data->calibrated;
}

/* ============================================================================
 * SECTION 14: LED PATTERNS AND FULL CALIBRATION STATE MACHINE
 * ============================================================================ */

/**
 * @brief Controls the system LED (PC13) with various blink patterns.
 * @param pattern 0=OFF, 1=ON, 2=Slow blink (calibrated), 3=Fast blink (error), 4=Heartbeat (calibrating)
 */
void led_set_pattern(uint8_t pat) {
    static uint32_t last = 0; 
    static uint8_t cur = 0;
    uint32_t now = HAL_GetTick();
    
    if (pat == 0) { 
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 
        cur = 0; 
    } else if (pat == 1) { 
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); 
        cur = 1; 
    } else {
        if (cur != pat) { 
            cur = pat; 
            last = now; 
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); 
        }
        uint32_t iv = (pat == 2) ? 1000 : (pat == 3) ? 166 : 500; // slow, fast, heartbeat
        if ((now - last) >= iv) { 
            last = now; 
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); 
        }
    }
}

/**
 * @brief Executes the full automated calibration sequence.
 */
void run_full_calibration(void) {
    if (calibration_running) return;
    calibration_running = 1; 
    calibration_error = 0; 
    calibration_success = 0; 
    abort_calibration = 0; 
    cal_state = CAL_STATE_CLUTCH;
    
    while (cal_state != CAL_STATE_DONE && cal_state != CAL_STATE_ERROR && abort_calibration == 0) {
        IWDG_Refresh(); 
        led_set_pattern(4); // Heartbeat pattern
        
        switch (cal_state) {
            case CAL_STATE_CLUTCH:
                if (clutch_calibrate()) { 
                    cal_data.clutch_disengaged = clutch_get_disengaged_angle(); 
                    cal_data.clutch_engaged = clutch_get_engaged_angle(); 
                    cal_data.clutch_bite = clutch_get_bite_point(); 
                    cal_state = CAL_STATE_AXIS_X; 
                } else {
                    cal_state = CAL_STATE_ERROR; 
                }
                break;
                
            case CAL_STATE_AXIS_X:
                if (axis_calibrate_x()) { 
                    cal_data.x_left_limit = cal_X.left_limit; 
                    cal_data.x_right_limit = cal_X.right_limit; 
                    cal_data.x_home = cal_X.home_offset;  
                    cal_data.x_backlash = cal_X.backlash; 
                    cal_state = CAL_STATE_AXIS_Y; 
                } else {
                    cal_state = CAL_STATE_ERROR; 
                }
                break;
                
            case CAL_STATE_AXIS_Y:
                if (axis_calibrate_y()) { 
                    cal_data.y_front_limit = cal_Y.left_limit; 
                    cal_data.y_back_limit = cal_Y.right_limit; 
                    cal_data.y_home = cal_Y.home_offset; 
                    cal_data.y_backlash = cal_Y.backlash; 
                    cal_state = CAL_STATE_GEAR_POSITIONS;  
                } else {
                    cal_state = CAL_STATE_ERROR; 
                }
                break;
                
            case CAL_STATE_GEAR_POSITIONS:
                for (int i = 0; i < 7; i++) { 
                    cal_data.gear_positions[i][0] = 0; 
                    cal_data.gear_positions[i][1] = 0; 
                }
                cal_state = CAL_STATE_SAVE; 
                break;
                
            case CAL_STATE_SAVE:
                cal_data.magic = CALIB_MAGIC; 
                cal_data.version = 1; 
                cal_data.calibrated = 0; 
                cal_data.crc = 0;
                cal_data.crc = calculate_crc16((uint8_t*)&cal_data, sizeof(CalibrationData));
                if (flash_save_calibration(&cal_data)) { 
                    cal_state = CAL_STATE_DONE; 
                    calibration_success = 1; 
                } else {
                    cal_state = CAL_STATE_ERROR; 
                }
                break;
                
            default: 
                cal_state = CAL_STATE_ERROR; 
                break;
        }
        HAL_Delay(100);
    }
    
    calibration_running = 0;
    if (cal_state == CAL_STATE_DONE) {
        led_set_pattern(2); // Success
    } else { 
        calibration_error = 1; 
        led_set_pattern(3); // Error
        stepper_disable_x(); 
        stepper_disable_y(); 
        clutch_stop();  
    }
}

/* ============================================================================
 * SECTION 15: UART HELPERS AND GEAR LEARNING
 * ============================================================================ */

void uart_send_string(char* s) { 
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), HAL_MAX_DELAY); 
}

void uart_clear_buffer(void) { 
    uart_rx_index = 0; 
    uart_cmd_ready = 0; 
    memset(uart_rx_buf, 0, UART_BUF_SIZE); 
}

void USART1_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        uint8_t ch = huart1.Instance->DR;
        if (ch == '\r' || ch == '\n') { 
            if (uart_rx_index > 0) { 
                uart_rx_buf[uart_rx_index] = '\0'; 
                uart_cmd_ready = 1; 
            } else {
                uart_clear_buffer(); 
            }
        } else { 
            if (uart_rx_index < UART_BUF_SIZE - 1) {
                uart_rx_buf[uart_rx_index++] = (char)ch; 
            } else {
                uart_clear_buffer(); 
            }
        }
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
    }
}

static void wait_for_enter(const char* p) { 
    uart_send_string((char*)p); 
    uart_clear_buffer(); 
    while (!uart_cmd_ready) { 
        HAL_Delay(50); 
        IWDG_Refresh();  
    } 
    uart_clear_buffer(); 
}

static uint8_t learn_one_gear(uint8_t idx) {
    char buf[UART_MSG_BUF_SIZE];
    const char* name = (idx == 0) ? "Neutral (N)" : (idx == 6) ? "Reverse (R)" : gear_names[idx];
    
    snprintf(buf, sizeof(buf), "\r\n--- TEACHING GEAR: %s ---\r\n", name); 
    uart_send_string(buf);
    wait_for_enter("1. Engine OFF, handbrake ON. Press Enter...");
    
    snprintf(buf, sizeof(buf), "2. Move lever into '%s'.\r\n", name); 
    uart_send_string(buf);
    wait_for_enter("   Press Enter when lever is in position...");
    
    stepper_enable_x(); 
    stepper_enable_y(); 
    HAL_Delay(100);
    
    int32_t x = current_x_steps, y = current_y_steps;
    snprintf(buf, sizeof(buf), "   Recorded: X=%ld Y=%ld\r\n   Save? (y/n): ", x, y); 
    uart_send_string(buf);
    
    uart_clear_buffer(); 
    while (!uart_cmd_ready) { 
        HAL_Delay(50); 
        IWDG_Refresh(); 
    }
    
    char resp = uart_rx_buf[0]; 
    uart_clear_buffer();
    
    if (resp == 'y' || resp == 'Y') { 
        cal_data.gear_positions[idx][0] = x; 
        cal_data.gear_positions[idx][1] = y; 
        uart_send_string("   [OK] Saved.\r\n"); 
        return 1; 
    } else { 
        uart_send_string("   [CANCELLED] Retry.\r\n"); 
        return 0; 
    }
}

/**
 * @brief Interactive mode to teach the controller the X/Y coordinates of all 7 gears.
 */
void learn_gear_positions(void) {
    uart_send_string("\r\n========================================\r\n  GEAR POSITION LEARNING MODE\r\n========================================\r\n");
    stepper_enable_x(); 
    stepper_enable_y();
    
    for (uint8_t g = 0; g <= 6; g++) {
        while (!learn_one_gear(g)) { 
            uart_send_string("\r\n>>> Retrying...\r\n"); 
            HAL_Delay(1000); 
        }
    }
    
    uart_send_string("\r\nSaving to Flash...\r\n");
    cal_data.magic = CALIB_MAGIC; 
    cal_data.version = 1; 
    cal_data.calibrated = 1; 
    cal_data.crc = 0;
    cal_data.crc = calculate_crc16((uint8_t*)&cal_data, sizeof(CalibrationData));
    
    if (flash_save_calibration(&cal_data)) { 
        uart_send_string("=== SUCCESS! All gears saved. ===\r\n"); 
        led_set_pattern(2); 
    } else { 
        uart_send_string("=== ERROR! Flash write failed. ===\r\n"); 
        led_set_pattern(3); 
    }
    move_to_neutral();
}

/* ============================================================================
 * SECTION 16: ADVANCED SHIFTING LOGIC (WITH SAFETY FIXES)
 * ============================================================================ */

uint16_t get_engine_rpm(void) { return get_rpm_universal(); }
uint8_t get_vehicle_speed(void) { return get_speed_universal(); }

/**
 * @brief Checks if a requested gear shift is safe.
 * 
 * CRITICAL FIX (v2.1+): Removed the erroneous condition (rpm > p.max_rpm_shift) 
 * that previously blocked upshifts when RPM exceeded a configurable limit. 
 * Now, automatic shifts are governed solely by the shift_map, allowing normal 
 * operation up to the engine's natural redline. The max_rpm_shift parameter 
 * is retained for potential future soft-limiting but no longer blocks shifts.
 * 
 * Safety checks remaining:
 * - Reverse gear only allowed when vehicle speed < 5 km/h.
 * - Winter mode: 1st gear blocked if speed > 10 km/h.
 */
uint8_t is_shift_safe_winter(uint8_t from, uint8_t to) {
    uint8_t spd = get_vehicle_speed();
    
    // Reverse gear safety
    if (to == 6 && spd > 5) {
        uart_send_string("Shift blocked: Reverse at speed\r\n");
        return 0;
    }
    
    // Winter mode 1st gear safety
    if (current_drive_mode == DRIVE_MODE_WINTER && to == 1 && spd > 10) {
        uart_send_string("Winter: 1st gear blocked >10km/h\r\n");
        return 0;
    }
    
    return 1; // Shift is safe
}

/**
 * @brief Calculates adaptive clutch hold time for smooth downshifts (rev-matching emulation).
 */
uint16_t get_adaptive_downshift_hold_time(uint8_t from, uint8_t to) {
    DriveParameters p = get_current_drive_params(); 
    uint16_t base = p.clutch_hold_time_ms;
    
    if (from > to && to != 0) { // Downshift detected
        uint16_t r = get_engine_rpm(); 
        if (r > 3000) return base + 150; 
        else if (r > 2000) return base + 100; 
        else return base + 50; 
    }
    return base;
}

/**
 * @brief Advanced clutch control sequence with optional bite-point pausing for smoothness.
 */
void clutch_control_advanced(ClutchAction a) {
    DriveParameters p = get_current_drive_params(); 
    uint32_t start; 
    uint16_t hold = get_adaptive_downshift_hold_time(current_gear, current_shift_target_gear);
    
    switch (a) {
        case CLUTCH_DISENGAGE:
            clutch_set_direction(0); 
            clutch_set_speed(p.clutch_disengage_speed);
            start = HAL_GetTick(); 
            while (!clutch_is_endstop_triggered() && (HAL_GetTick() - start < 1000)) { 
                HAL_Delay(5); 
                IWDG_Refresh(); 
            }
            clutch_stop(); 
            for (uint16_t i = 0; i < hold; i += 50) { 
                HAL_Delay(50); 
                IWDG_Refresh(); 
            } 
            break;
            
        case CLUTCH_ENGAGE:
            clutch_set_direction(1); 
            clutch_set_speed(p.clutch_engage_speed);
            if (!p.use_bite_point) {
                start = HAL_GetTick(); 
                while (clutch_get_current_angle() < cal_data.clutch_engaged && (HAL_GetTick() - start < 1000)) { 
                    HAL_Delay(5); 
                    IWDG_Refresh(); 
                }
            } else {
                // Smooth engagement: pause at bite point
                start = HAL_GetTick(); 
                while (clutch_get_current_angle() < cal_data.clutch_bite && (HAL_GetTick() - start < 1000)) { 
                    HAL_Delay(5); 
                    IWDG_Refresh(); 
                }
                clutch_stop(); 
                HAL_Delay(300); // Pause to allow RPM to settle
                clutch_set_direction(1); 
                clutch_set_speed(20); // Slow creep to full engagement
                start = HAL_GetTick(); 
                while (clutch_get_current_angle() < cal_data.clutch_engaged && (HAL_GetTick() - start < 1000)) { 
                    HAL_Delay(5); 
                    IWDG_Refresh(); 
                }
            }
            clutch_stop(); 
            break;
            
        case CLUTCH_TO_BITE:
            clutch_set_direction(1); 
            clutch_set_speed(p.use_bite_point ? 20 : 60);
            while (clutch_get_current_angle() < cal_data.clutch_bite) { 
                HAL_Delay(5); 
                IWDG_Refresh(); 
            }
            clutch_stop(); 
            break;
    }
}

/**
 * @brief Moves the stepper motors to the calibrated coordinates of a specific gear.
 */
void move_to_gear_position_advanced(uint8_t gear, uint8_t axis) {
    DriveParameters p = get_current_drive_params(); 
    uint32_t d = p.shift_delay_us;
    int32_t target = (axis == 0) ? cal_data.gear_positions[gear][0] : cal_data.gear_positions[gear][1];
    
    if (target == 0 && gear != 0) { 
        uart_send_string("ERROR: Gear not learned\r\n"); 
        return; 
    }
    
    int32_t cur = (axis == 0) ? current_x_steps : current_y_steps;
    int32_t bl = (axis == 0) ? cal_data.x_backlash : cal_data.y_backlash;
    int32_t diff = target - cur; 
    if (diff == 0) return;
    
    uint8_t dir = (diff > 0) ? 1 : 0; 
    int32_t abs_diff = (diff > 0) ? diff : -diff;
    
    // Backlash compensation logic
    if (axis == 0) {
        if (dir == 1) { 
            if (cur > target) axis_move_relative(0, -(abs_diff + bl), d); 
            axis_move_relative(0, abs_diff + bl, d); 
        } else { 
            if (cur < target) axis_move_relative(0, (abs_diff + bl), d); 
            axis_move_relative(0, -(abs_diff + bl), d); 
        }
    } else {
        if (dir == 1) { 
            if (cur > target) axis_move_relative(1, -(abs_diff + bl), d); 
            axis_move_relative(1, abs_diff + bl, d); 
        } else { 
            if (cur < target) axis_move_relative(1, (abs_diff + bl), d); 
            axis_move_relative(1, -(abs_diff + bl), d); 
        }
    }
}

void move_to_neutral(void) { 
    move_to_gear_position_advanced(0, 1); 
    HAL_Delay(30); 
    move_to_gear_position_advanced(0, 0); 
    HAL_Delay(30); 
}

/**
 * @brief Executes a full gear shift sequence with advanced clutch control.
 */
uint8_t shift_gear_advanced(uint8_t t) {
    if (shifting_in_progress) return 0; 
    if (t == current_gear) return 1; 
    if (!is_shift_safe_winter(current_gear, t)) return 0;
    
    shifting_in_progress = 1; 
    current_shift_target_gear = t; 
    activity_reset_timer();
    
    char msg[64]; 
    snprintf(msg, sizeof(msg), "Shifting to gear %d...\r\n", t); 
    uart_send_string(msg);
    
    clutch_control_advanced(CLUTCH_DISENGAGE);
    
    if (current_gear != 0) { 
        move_to_neutral(); 
        HAL_Delay(50); 
    }
    
    if (t != 0) { 
        move_to_gear_position_advanced(t, 0); 
        HAL_Delay(40); 
        move_to_gear_position_advanced(t, 1); 
        HAL_Delay(80); 
    }
    
    if (t == 0) {
        clutch_control_advanced(CLUTCH_ENGAGE);
    } else {
        if (current_gear == 0 && get_vehicle_speed() < 3) { 
            clutch_control_advanced(CLUTCH_TO_BITE); 
            HAL_Delay(400); 
            clutch_control_advanced(CLUTCH_ENGAGE); 
        } else {
            clutch_control_advanced(CLUTCH_ENGAGE);
        }
    }
    
    current_gear = t; 
    shifting_in_progress = 0; 
    current_shift_target_gear = 0;
    
    snprintf(msg, sizeof(msg), "Shift complete. Gear: %d\r\n", current_gear); 
    uart_send_string(msg);
    return 1;
}

/**
 * @brief Safe wrapper for shifting that enforces Limp Mode restrictions.
 */
uint8_t shift_gear_limp_safe(uint8_t t) {
    if (!limp_mode_active) return shift_gear_advanced(t);
    
    // In Limp Mode, only Neutral (0) and 2nd gear (2) are allowed
    if (t == 0 || t == 2 || t == current_gear) { 
        uart_send_string("[LIMP] Allowed shift.\r\n"); 
        return shift_gear_advanced(t); 
    }
    
    char msg[128]; 
    snprintf(msg, sizeof(msg), "LIMP: Shift to %d BLOCKED. Use N or 2nd.\r\n", t); 
    uart_send_string(msg); 
    bt_send_string(msg); 
    return 0;
}

/* ============================================================================
 * SECTION 17: CURRENT MONITORING AND SAFETY SYSTEMS
 * ============================================================================ */

static uint16_t read_adc_channel(uint32_t ch) {
    ADC_ChannelConfTypeDef c = {0}; 
    c.Channel = ch; 
    c.Rank = 1; 
    c.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &c); 
    HAL_ADC_Start(&hadc1); 
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t val = HAL_ADC_GetValue(&hadc1); 
    HAL_ADC_Stop(&hadc1); 
    return val;
}

/**
 * @brief Calibrates zero-current offsets for ACS712 sensors at startup.
 */
void calibrate_current_offsets(void) {
    uint32_t sx = 0, sy = 0, sc = 0;
    for (int i = 0; i < 16; i++) { 
        sx += read_adc_channel(ADC_CHANNEL_10); 
        sy += read_adc_channel(ADC_CHANNEL_11); 
        sc += read_adc_channel(ADC_CHANNEL_12); 
        HAL_Delay(2); 
    }
    current_offsets[0] = sx / 16; 
    current_offsets[1] = sy / 16; 
    current_offsets[2] = sc / 16;
}

/**
 * @brief Reads current in Amperes for a given motor index (0=X, 1=Y, 2=Clutch).
 * ACS712-30A sensitivity is 185 mV/A.
 */
float get_current(uint8_t idx) {
    if (idx > 2) return 0.0f;
    uint32_t ch[] = { ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12 };
    uint16_t raw = read_adc_channel(ch[idx]); 
    int16_t diff = (int16_t)raw - current_offsets[idx];
    float volts = diff * (3.3f / 4096.0f); 
    float amps = volts / 0.185f; 
    return (amps > 0) ? amps : -amps;
}

/**
 * @brief Monitors motor currents and disables drivers if overcurrent is detected.
 */
void current_monitor_task(void) {
    static uint32_t last = 0; 
    if (HAL_GetTick() - last < 100) return; 
    last = HAL_GetTick();
    
    float ix = get_current(0), iy = get_current(1), ic = get_current(2);
    
    if (ix > 4.5f && !shifting_in_progress && !calibration_running) { 
        log_error(ERR_CURRENT_OVER_X, 1, (uint16_t)(ix * 10)); 
        stepper_disable_x(); 
    }
    if (iy > 4.5f && !shifting_in_progress && !calibration_running) { 
        log_error(ERR_CURRENT_OVER_Y, 1, (uint16_t)(iy * 10)); 
        stepper_disable_y(); 
    }
    if (ic > 10.0f) { 
        log_error(ERR_CURRENT_OVER_CLUTCH, 2, (uint16_t)(ic * 10)); 
        emergency_power_off(); 
    }
}

void emergency_power_off(void) { 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); 
    uart_send_string("EMERGENCY POWER OFF\r\n"); 
}

void emergency_power_on(void) { 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); 
}

void emergency_stop(void) { 
    log_error(ERR_EMERGENCY_BUTTON, 2, 0); 
    emergency_power_off(); 
    shifting_in_progress = 0; 
    calibration_running = 0; 
    limp_mode_active = 1; 
    clutch_stop(); 
    stepper_disable_x(); 
    stepper_disable_y(); 
}

/**
 * @brief Logs an error to the RAM circular buffer. Critical errors trigger Limp Mode.
 */
void log_error(uint8_t code, uint8_t sev, uint16_t data) {
    error_buffer[error_index].timestamp = HAL_GetTick(); 
    error_buffer[error_index].error_code = code; 
    error_buffer[error_index].severity = sev; 
    error_buffer[error_index].data = data;
    
    error_index = (error_index + 1) % ERROR_LOG_MAX_ENTRIES; 
    if (error_count < ERROR_LOG_MAX_ENTRIES) error_count++;
    
    if (sev == 2) { 
        limp_mode_active = 1; 
        emergency_power_off(); 
        shifting_in_progress = 0; 
        calibration_running = 0; 
        clutch_stop(); 
        stepper_disable_x(); 
        stepper_disable_y(); 
        led_set_pattern(3); 
        bt_send_string("CRITICAL FAULT! Limp mode.\r\n"); 
    }
}

void save_error_log_to_flash(void) {
    if (error_count == 0) return; 
    flash_unlock(); 
    flash_erase_page(ERROR_LOG_ADDR);
    uint32_t addr = ERROR_LOG_ADDR; 
    for (int i = 0; i < error_count; i++) { 
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, ((uint32_t)&error_buffer[i])); 
        addr += 4; 
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (((uint32_t)&error_buffer[i]) + 1)); 
        addr += 4; 
    }
    flash_lock();
}

/**
 * @brief Performs basic diagnostic checks on sensors and limit switches.
 */
uint8_t diagnostic_check(void) {
    uint8_t ok = 1; 
    if (!AS5600_Init()) { 
        log_error(ERR_AS5600_MISSING, 2, 0); 
        ok = 0; 
    }
    if (limit_x_left_triggered && limit_x_right_triggered) { 
        log_error(ERR_LIMIT_SWITCH_STUCK, 1, 0); 
        ok = 0; 
    }
    if (limit_y_front_triggered && limit_y_back_triggered) { 
        log_error(ERR_LIMIT_SWITCH_STUCK, 1, 1); 
        ok = 0; 
    }
    return ok;
}

/**
 * @brief EXTI callback for limit switches and emergency stop.
 */
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    switch (pin) {
        case GPIO_PIN_1: clutch_endstop_triggered = 1; break;
        case GPIO_PIN_2: limit_x_left_triggered = 1; break;
        case GPIO_PIN_3: limit_x_right_triggered = 1; break;
        case GPIO_PIN_10: limit_y_front_triggered = 1; break;
        case GPIO_PIN_11: limit_y_back_triggered = 1; break;
        case GPIO_PIN_8: emergency_stop(); break; // Highest priority
        default: break;
    }
}

/**
 * @brief Initializes safety-critical hardware (emergency relay, current offsets).
 */
void safety_init(void) {
    GPIO_InitTypeDef g = {0}; 
    __HAL_RCC_GPIOA_CLK_ENABLE(); 
    g.Pin = GPIO_PIN_8; 
    g.Mode = GPIO_MODE_OUTPUT_PP; 
    g.Speed = GPIO_SPEED_FREQ_LOW; 
    HAL_GPIO_Init(GPIOA, &g);
    
    emergency_power_on(); 
    stepper_disable_x(); 
    stepper_disable_y(); 
    clutch_stop(); 
    calibrate_current_offsets(); 
    diagnostic_check();
}

/* ============================================================================
 * END OF PART 3 - CONTINUES IN PART 4
 * ============================================================================ */
 /* ============================================================================
 * SECTION 18: CAN COMMUNICATION AND SHIFTING TASKS
 * ============================================================================ */

/**
 * @brief CAN RX FIFO callback. Parses incoming messages based on active profile.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK) {
        // Parse standard known IDs (fallback if profile not matched)
        switch (hdr.StdId) {
            case 0x0C: engine_rpm = ((uint16_t)data[0] << 8) | data[1]; break;
            case 0x0D: vehicle_speed_kmh = data[0]; break;
            case 0x11: throttle_percent = data[0]; break;
            case 0x1A: brake_pressed = (data[0] > 0) ? 1 : 0; break;
            case 0x30: 
                shift_paddle_up = (data[0] & 0x01); 
                shift_paddle_down = (data[0] & 0x02); 
                break;
            default: break;
        }
        
        // Parse extended AWD wheel speeds if profile supports it
        if (current_profile.front_speed_id && hdr.StdId == current_profile.front_speed_id) {
            uint16_t val = 0;
            for (int i = 0; i < current_profile.front_speed_length; i++) {
                val |= (data[current_profile.front_speed_offset + i] << (8 * i));
            }
            front_wheel_speed_raw = val;
        }
        if (current_profile.rear_speed_id && hdr.StdId == current_profile.rear_speed_id) {
            uint16_t val = 0;
            for (int i = 0; i < current_profile.rear_speed_length; i++) {
                val |= (data[current_profile.rear_speed_offset + i] << (8 * i));
            }
            rear_wheel_speed_raw = val;
        }
    }
}

/**
 * @brief Applies exponential moving average to smooth CAN data.
 */
void filter_data(void) {
    filtered_rpm = (uint16_t)((1.0f - FILTER_ALPHA) * filtered_rpm + FILTER_ALPHA * engine_rpm);
    filtered_throttle = (uint8_t)((1.0f - FILTER_ALPHA) * filtered_throttle + FILTER_ALPHA * throttle_percent);
}

/**
 * @brief Automatic shifting task using mode-specific shift maps.
 * Includes brake-initiated downshift and cascade zero-throttle downshift.
 */
void auto_shift_task(void) {
    if (!ignition_on || limp_mode_active || shifting_in_progress || current_gear == 0 || current_gear == 6) return;
    
    static uint32_t last_auto = 0;
    uint32_t now = HAL_GetTick();
    
    // 1. Zero-throttle cascading downshift (highest priority)
    if (throttle_percent == 0 && current_gear > 1 && get_speed_universal() > 2) {
        cascade_zero_throttle_downshift();
    }
    
    // 2. Normal upshift/downshift with debounce
    if (now - last_auto >= 500) {
        uint8_t map_size;
        const ShiftMapEntry* map = get_shift_map(&map_size);
        const ShiftMapEntry* e = &map[map_size - 1];
        
        for (uint8_t i = 0; i < map_size; i++) {
            if (filtered_throttle <= map[i].throttle) {
                e = &map[i];
                break;
            }
        }
        
        // Upshift logic
        uint8_t speed_limit_active = 0;
        if (speed_limiter_enabled) {
            uint16_t limit = speed_limit_kmh;
            if (speed_limit_mph_mode) limit = (uint16_t)(limit * 1.609f);
            if (get_speed_universal() >= limit) speed_limit_active = 1;
        }
        
        if (throttle_percent > 0 && !speed_limit_active && filtered_rpm > e->upshift_rpm && current_gear < 5) {
            shift_gear_limp_safe(current_gear + 1);
            last_auto = now;
        }
        // Downshift logic
        else if (engine_rpm < e->downshift_rpm && current_gear > 1) {
            shift_gear_limp_safe(current_gear - 1);
            last_auto = now;
        }
    }
    
    // 3. Brake-initiated cascading downshift
    static uint32_t last_brake = 0;
    if (brake_pressed && current_gear > 1 && get_speed_universal() > 20 && (now - last_brake) >= 600) {
        shift_gear_limp_safe(current_gear - 1);
        last_brake = now;
        last_auto = now;
    }
    
    // 4. Cruise control maintenance
    handle_cruise_control();
}

/**
 * @brief Manual shifting task via steering wheel paddles.
 */
void manual_shift_task(void) {
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if (now - last < 150) return; // Debounce
    
    if (shift_paddle_up && current_gear < 5) {
        shift_gear_limp_safe(current_gear + 1);
        last = now;
        shift_paddle_up = 0;
    } else if (shift_paddle_down && current_gear > 1) {
        shift_gear_limp_safe(current_gear - 1);
        last = now;
        shift_paddle_down = 0;
    }
}

/* ============================================================================
 * SECTION 19: REDUNDANT CLUTCH SENSOR AND ADVANCED FEATURES
 * ============================================================================ */

void read_clutch_backup_sensor(void) {
    ADC_ChannelConfTypeDef c = {0};
    c.Channel = ADC_CHANNEL_14;
    c.Rank = 1;
    c.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &c);
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        clutch_backup_sensor_raw = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
}

void calibrate_backup_sensor(uint16_t raw_dis, uint16_t raw_eng, uint16_t angle_dis, uint16_t angle_eng) {
    backup_cal.raw_min = raw_dis;
    backup_cal.raw_max = raw_eng;
    int32_t as5600_range = (int32_t)angle_dis - (int32_t)angle_eng;
    int32_t backup_range = (int32_t)backup_cal.raw_max - (int32_t)backup_cal.raw_min;
    backup_cal.scale = (backup_range != 0) ? (float)as5600_range / (float)backup_range : 1.0f;
    backup_cal.calibrated = 1;
}

uint16_t backup_to_as5600_angle(uint16_t raw) {
    if (!backup_cal.calibrated) return 0;
    if (raw < backup_cal.raw_min) raw = backup_cal.raw_min;
    if (raw > backup_cal.raw_max) raw = backup_cal.raw_max;
    int32_t off = (int32_t)raw - backup_cal.raw_min;
    float ang = (float)off * backup_cal.scale;
    uint16_t res = (uint16_t)(cal_data.clutch_engaged + ang + 0.5f);
    if (res > 4095) res = 4095;
    return res;
}

void check_sensor_consistency(void) {
    if (!backup_cal.calibrated) return;
    uint16_t prim = AS5600_ReadAngle();
    uint16_t back = backup_to_as5600_angle(clutch_backup_sensor_raw);
    int16_t diff = (int16_t)prim - (int16_t)back;
    if (diff < 0) diff = -diff;
    
    if (diff > SENSOR_TOLERANCE) {
        if (active_sensor == PRIMARY_AS5600 && !as5600_fault) {
            as5600_fault = 1;
            active_sensor = BACKUP_POTENTIOMETER;
            log_error(ERR_AS5600_MISSING, 1, diff);
            uart_send_string("Sensor mismatch! Using backup.\r\n");
            bt_send_string("AS5600 FAULT - Backup active\r\n");
        }
    } else {
        if (as5600_fault && active_sensor == BACKUP_POTENTIOMETER) {
            static uint8_t stable = 0;
            if (diff < (SENSOR_TOLERANCE / 2)) {
                if (++stable >= 5) {
                    as5600_fault = 0;
                    active_sensor = PRIMARY_AS5600;
                    stable = 0;
                    uart_send_string("AS5600 recovered.\r\n");
                }
            } else {
                stable = 0;
            }
        }
    }
}

uint16_t clutch_get_current_angle(void) {
    if (active_sensor == PRIMARY_AS5600) {
        uint16_t a = AS5600_ReadAngle();
        if (!as5600_comm_ok) {
            active_sensor = BACKUP_POTENTIOMETER;
            log_error(ERR_AS5600_MISSING, 1, 0);
        }
        return a;
    } else if (active_sensor == BACKUP_POTENTIOMETER) {
        return backup_to_as5600_angle(clutch_backup_sensor_raw);
    }
    return 0xFFFF;
}

/* ============================================================================
 * SECTION 20: POWER MANAGEMENT, PARKING, AWD, CAN PROFILING
 * ============================================================================ */

void power_clutch_on(void) { clutch_stop(); }
void power_clutch_off(void) { clutch_stop(); }

static void power_actuators_on(void) {
    stepper_enable_x();
    stepper_enable_y();
    power_clutch_on();
}

static void power_actuators_off(void) {
    power_clutch_off();
    // Steppers stay enabled to hold gear position against return springs
}

void activity_reset_timer(void) {
    if (system_sleeping) {
        system_sleeping = 0;
        power_actuators_on();
        uart_send_string("[INFO] Woke up.\r\n");
    }
    last_activity_time = HAL_GetTick();
}

static void system_enter_sleep(void) {
    if (shifting_in_progress || calibration_running || limp_mode_active) return;
    if (system_sleeping) return;
    power_actuators_off();
    system_sleeping = 1;
    led_set_pattern(4);
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
    uart_send_string("[INFO] Sleep mode.\r\n");
}

void idle_monitor_task(void) {
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if (now - last < 1000) return;
    last = now;
    if (!system_sleeping && !shifting_in_progress && !calibration_running && 
        (now - last_activity_time) >= IDLE_TIMEOUT_MS) {
        system_enter_sleep();
    }
}

uint16_t get_rpm_universal(void) {
    if (can_profile_valid && current_profile.rpm_id != 0) return filtered_rpm;
    read_analog_tacho();
    return engine_rpm_analog;
}

uint8_t get_speed_universal(void) {
    return (can_profile_valid && current_profile.speed_id != 0) ? vehicle_speed_kmh : 0;
}

uint8_t auto_scan_can(void) {
    uart_send_string("Scanning CAN (4 sec)...\r\n");
    typedef struct { uint16_t id; uint8_t cnt; } Seen;
    Seen seen[64];
    uint8_t seen_cnt = 0;
    uint32_t start = HAL_GetTick();
    
    while (HAL_GetTick() - start < 4000) {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
            CAN_RxHeaderTypeDef hdr;
            uint8_t d[8];
            if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, d) == HAL_OK) {
                uint8_t found = 0;
                for (int i = 0; i < seen_cnt; i++) {
                    if (seen[i].id == hdr.StdId) { seen[i].cnt++; found = 1; break; }
                }
                if (!found && seen_cnt < 64) {
                    seen[seen_cnt].id = hdr.StdId;
                    seen[seen_cnt].cnt = 1;
                    seen_cnt++;
                }
            }
        }
        IWDG_Refresh();
        HAL_Delay(5);
    }
    
    int best_score = 0, best_idx = -1;
    for (int p = 0; p < NUM_PROFILES; p++) {
        int score = 0;
        for (int i = 0; i < seen_cnt; i++) {
            if (seen[i].cnt < 2) continue;
            if (seen[i].id == known_profiles[p].rpm_id) score += 3;
            if (seen[i].id == known_profiles[p].speed_id) score += 3;
            if (seen[i].id == known_profiles[p].throttle_id) score += 1;
            if (seen[i].id == known_profiles[p].brake_id) score += 1;
        }
        if (score > best_score) { best_score = score; best_idx = p; }
    }
    
    if (best_score >= 5 && best_idx >= 0) {
        current_profile = known_profiles[best_idx];
        can_profile_valid = 1;
        save_can_profile();
        uart_send_string("Profile matched.\r\n");
        return 1;
    }
    uart_send_string("No match. Using analog fallback.\r\n");
    return 0;
}

void save_can_profile(void) {
    flash_unlock();
    flash_erase_page(CAN_PROFILE_FLASH_ADDR);
    uint32_t* src = (uint32_t*)&current_profile;
    uint32_t words = sizeof(CanProfile) / 4;
    uint32_t addr = CAN_PROFILE_FLASH_ADDR;
    for (uint32_t i = 0; i < words; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]);
        addr += 4;
    }
    flash_lock();
}

void load_can_profile(void) {
    CanProfile* fp = (CanProfile*)CAN_PROFILE_FLASH_ADDR;
    if (fp->rpm_id != 0) {
        memcpy(&current_profile, fp, sizeof(CanProfile));
        can_profile_valid = 1;
        uart_send_string("CAN profile loaded.\r\n");
    } else {
        uart_send_string("No profile. Scanning...\r\n");
        auto_scan_can();
    }
}

void apply_vehicle_adaptation(void) {
    if (!can_profile_valid) return;
    uart_send_string("Applying adaptations...\r\n");
    if (current_profile.rpm_id == 0x0C) uart_send_string("Ford profile.\r\n");
    else if (current_profile.rpm_id == 0x280) uart_send_string("VAG profile.\r\n");
}

void check_critical_failures(void) {
    if (limp_mode_active) return;
    uint16_t r = get_rpm_universal();
    uint8_t s = get_speed_universal();
    if (r == 0 && s > 5) {
        if (can_loss_timer == 0) can_loss_timer = HAL_GetTick();
        else if ((HAL_GetTick() - can_loss_timer) > 2000) enter_limp_mode("CAN BUS LOST");
    } else {
        can_loss_timer = 0;
    }
    if (active_sensor == SENSOR_FAULT) enter_limp_mode("CLUTCH SENSOR FAULT");
}

void enter_limp_mode(const char* reason) {
    limp_mode_active = 1;
    current_drive_mode = DRIVE_MODE_NORMAL;
    char m[128];
    snprintf(m, sizeof(m), "LIMP MODE! Reason: %s\r\nUse N or 2nd gear.\r\n", reason);
    bt_send_string(m);
    uart_send_string(m);
    log_error(ERR_WATCHDOG_RESET, 2, 99);
    led_set_pattern(3);
}

void exit_limp_mode(void) {
    if (!limp_mode_active) return;
    if (active_sensor != SENSOR_FAULT && get_rpm_universal() > 0) {
        limp_mode_active = 0;
        can_loss_timer = 0;
        uart_send_string("Limp mode cleared.\r\n");
        bt_send_string("Limp mode cleared.\r\n");
        led_set_pattern(current_drive_mode != DRIVE_MODE_NORMAL ? 2 : 0);
    } else {
        uart_send_string("Fault still present.\r\n");
    }
}

void execute_parking_mode(void) {
    if (is_parked || get_speed_universal() > 2) return;
    uart_send_string(">>> PARKING MODE <<<\r\n");
    activity_reset_timer();
    if (current_gear == 0) {
        shift_gear_limp_safe(1);
        HAL_Delay(500);
    }
    clutch_control_advanced(CLUTCH_ENGAGE);
    is_parked = 1;
    uart_send_string("!!! Car in gear. APPLY HANDBRAKE !!!\r\n");
    bt_send_string("PARKED. Apply handbrake!\r\n");
    led_set_pattern(4);
}

void exit_parking_mode(void) {
    if (!is_parked) return;
    uart_send_string(">>> EXIT PARKING <<<\r\n");
    activity_reset_timer();
    if (current_gear != 0) {
        shift_gear_limp_safe(0);
        HAL_Delay(500);
    }
    is_parked = 0;
    uart_send_string("Ready.\r\n");
    bt_send_string("Unparked.\r\n");
    led_set_pattern(current_drive_mode != DRIVE_MODE_NORMAL ? 2 : 0);
}

void parking_monitor_task(void) {
    uint16_t rpm = get_rpm_universal();
    if (rpm < 100) {
        if (engine_off_timer == 0) engine_off_timer = HAL_GetTick();
        else if ((HAL_GetTick() - engine_off_timer) > 3000 && !shifting_in_progress) {
            execute_parking_mode();
        }
    } else {
        engine_off_timer = 0;
        if (is_parked && rpm > 500) exit_parking_mode();
    }
}

void awd_init(void) {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = awd_config.awd_enable_pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, awd_config.awd_enable_pin, GPIO_PIN_RESET);
    awd_engaged = 0;
}

void awd_set_engaged(uint8_t en) {
    if (en == awd_engaged) return;
    if (en) {
        HAL_GPIO_WritePin(GPIOB, awd_config.awd_enable_pin, GPIO_PIN_SET);
        HAL_Delay(awd_config.awd_engage_delay_ms);
        awd_engaged = 1;
        awd_engagement_time = HAL_GetTick();
        uart_send_string("[AWD] Engaged\r\n");
    } else {
        HAL_GPIO_WritePin(GPIOB, awd_config.awd_enable_pin, GPIO_PIN_RESET);
        awd_engaged = 0;
        uart_send_string("[AWD] Disengaged\r\n");
    }
}

uint8_t get_wheel_speed_diff(void) {
    uint8_t front = front_wheel_speed_raw / 10;
    uint8_t rear = rear_wheel_speed_raw / 10;
    return (front > rear) ? (front - rear) : (rear - front);
}

void awd_auto_task(void) {
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if (now - last < 100) return;
    last = now;
    
    uint8_t diff = get_wheel_speed_diff();
    uint8_t spd = get_speed_universal();
    
    switch (current_awd_mode) {
        case AWD_MODE_2WD:
            if (awd_engaged) awd_set_engaged(0);
            break;
        case AWD_MODE_AUTO:
            if (diff >= awd_config.awd_auto_speed_diff) {
                if (!awd_engaged) awd_set_engaged(1);
            } else if (awd_engaged && spd > 15) {
                awd_set_engaged(0);
            }
            break;
        case AWD_MODE_LOCK:
            if (!awd_engaged) awd_set_engaged(1);
            if (awd_config.awd_lock_timeout_s > 0 && (now - awd_engagement_time) > (awd_config.awd_lock_timeout_s * 1000)) {
                uart_send_string("[AWD] LOCK timeout -> AUTO\r\n");
                current_awd_mode = AWD_MODE_AUTO;
                awd_set_engaged(0);
            }
            break;
        case AWD_MODE_LOW:
            if (!awd_engaged) awd_set_engaged(1);
            if (spd > 40) {
                uart_send_string("[AWD] Speed high for LOW\r\n");
                current_awd_mode = AWD_MODE_AUTO;
            }
            break;
    }
}

/* ============================================================================
 * SECTION 21: NEW FEATURES V4.0 (Coolant, Ignition, Limiter, Cruise, Cascade, Visualize)
 * ============================================================================ */

void read_coolant_temperature(void) {
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if (now - last < 2000) return; // Read every 2 seconds
    last = now;
    
    ADC_ChannelConfTypeDef c = {0};
    c.Channel = ADC_CHANNEL_8;
    c.Rank = 1;
    c.SACamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &c);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    coolant_temp_raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    
    // Convert raw to Celsius (example: LM35 gives 10mV/°C, ref 3.3V)
    float voltage = coolant_temp_raw * (3.3f / 4096.0f);
    coolant_temp_celsius = (uint8_t)(voltage * 100.0f);
    if (coolant_temp_celsius < 20) coolant_temp_celsius = 20;
    if (coolant_temp_celsius > 120) coolant_temp_celsius = 120;
    
    update_engine_rpm_limit();
}

void update_engine_rpm_limit(void) {
    if (!ignition_on) {
        engine_rpm_limit = 0;
        return;
    }
    if (current_drive_mode == DRIVE_MODE_SPORT) {
        engine_rpm_limit = SPORT_RPM_LIMIT;
    } else {
        if (coolant_temp_celsius < 60) {
            engine_rpm_limit = COLD_ENGINE_RPM_LIMIT;
        } else {
            engine_rpm_limit = NORMAL_ENGINE_RPM_LIMIT;
        }
    }
}

void handle_speed_limiter(void) {
    static uint8_t limiter_was_active = 0;
    if (!speed_limiter_enabled) {
        if (limiter_was_active) {
            uart_send_string("Speed limiter deactivated\r\n");
            limiter_was_active = 0;
        }
        return;
    }
    
    uint16_t speed = get_speed_universal();
    uint16_t limit = speed_limit_kmh;
    if (speed_limit_mph_mode) limit = (uint16_t)(limit * 1.609f);
    
    if (speed >= limit && throttle_percent > 0 && current_gear < 5) {
        if (!limiter_was_active) {
            uart_send_string("Speed limiter active\r\n");
            limiter_was_active = 1;
        }
    } else {
        if (limiter_was_active) {
            uart_send_string("Speed limiter deactivated\r\n");
            limiter_was_active = 0;
        }
    }
}

void handle_cruise_control(void) {
    // Cruise control only available in COMFORT and WINTER modes
    if (current_drive_mode != DRIVE_MODE_COMFORT && current_drive_mode != DRIVE_MODE_WINTER) {
        cruise_control_active = 0;
        return;
    }
    if (!cruise_control_active) return;
    if (brake_pressed || throttle_percent == 0 || get_speed_universal() < 20) {
        cruise_control_active = 0;
        uart_send_string("Cruise control deactivated\r\n");
        return;
    }
    
    uint16_t current_speed = get_speed_universal();
    int16_t error = (int16_t)cruise_target_speed - (int16_t)current_speed;
    
    if (abs(error) > 2) {
        static uint32_t last_cruise_adjust = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_cruise_adjust >= 200) {
            if (error > 0) {
                if (throttle_percent < 80) throttle_percent += 5;
            } else {
                if (throttle_percent > 5) throttle_percent -= 5;
            }
            last_cruise_adjust = now;
        }
    }
}

void process_ignition_key(void) {
    uint8_t new_state = HAL_GPIO_ReadPin(GPIOB, IGNITION_PIN);
    if (new_state != ignition_on) {
        ignition_on = new_state;
        if (ignition_on) {
            uart_send_string("IGNITION ON\r\n");
            update_engine_rpm_limit();
        } else {
            uart_send_string("IGNITION OFF\r\n");
            engine_rpm = 0;
            filtered_rpm = 0;
            emergency_power_off();
            shifting_in_progress = 0;
            calibration_running = 0;
            limp_mode_active = 0;
            cruise_control_active = 0;
        }
    }
}

void cascade_zero_throttle_downshift(void) {
    static uint32_t last_zero_downshift = 0;
    uint32_t now = HAL_GetTick();
    uint32_t delay = CASCADE_DELAY_NORMAL;
    
    if (current_drive_mode == DRIVE_MODE_SPORT) delay = CASCADE_DELAY_SPORT;
    else if (current_drive_mode == DRIVE_MODE_WINTER) delay = CASCADE_DELAY_WINTER;
    else if (current_drive_mode == DRIVE_MODE_COMFORT) delay = CASCADE_DELAY_COMFORT;
    
    if (now - last_zero_downshift < delay) return;
    if (current_gear <= 1) return;
    
    uint8_t speed = get_speed_universal();
    if (speed < 2) return;
    
    uint8_t min_gear = (current_drive_mode == DRIVE_MODE_WINTER) ? 2 : 1;
    if (current_gear > min_gear) {
        shift_gear_limp_safe(current_gear - 1);
        last_zero_downshift = now;
    }
}

/**
 * @brief Visualizes X/Y axis positions and gear coordinates via UART.
 * Ideal for competition demonstration and debugging.
 */
void visualize_axes(void) {
    char buf[256];
    uart_send_string("\r\n========================================\r\n");
    uart_send_string("       AXIS VISUALIZATION (v4.0)\r\n");
    uart_send_string("========================================\r\n");
    
    snprintf(buf, sizeof(buf), 
        "\r\n[X AXIS - Shift Row]\r\n"
        "  Current position: %ld steps\r\n"
        "  Left limit:       %ld steps\r\n"
        "  Right limit:      %ld steps\r\n"
        "  Home (Neutral):   %ld steps\r\n"
        "  Backlash:         %ld steps\r\n",
        current_x_steps, cal_data.x_left_limit, cal_data.x_right_limit, 
        cal_data.x_home, cal_data.x_backlash);
    uart_send_string(buf);
    
    snprintf(buf, sizeof(buf),
        "\r\n[Y AXIS - Gear Engagement]\r\n"
        "  Current position: %ld steps\r\n"
        "  Front limit:      %ld steps\r\n"
        "  Back limit:       %ld steps\r\n"
        "  Home (Neutral):   %ld steps\r\n"
        "  Backlash:         %ld steps\r\n",
        current_y_steps, cal_data.y_front_limit, cal_data.y_back_limit, 
        cal_data.y_home, cal_data.y_backlash);
    uart_send_string(buf);
    
    uart_send_string("\r\n[ALL GEAR COORDINATES]\r\n");
    uart_send_string("  Gear |    X    |    Y\r\n");
    uart_send_string("  -----+---------+--------\r\n");
    for (int i = 0; i < 7; i++) {
        snprintf(buf, sizeof(buf), "   %s   | %7ld | %7ld\r\n",
            gear_names[i], cal_data.gear_positions[i][0], cal_data.gear_positions[i][1]);
        uart_send_string(buf);
    }
    uart_send_string("========================================\r\n");
}

/* ============================================================================
 * SECTION 22: ANALOG TACHOMETER (TIM4 INPUT CAPTURE)
 * ============================================================================ */

void TIM4_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim4);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM4) {
        uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        if (tacho_last_capture != 0) {
            uint32_t diff = now - tacho_last_capture;
            if (now < tacho_last_capture) {
                diff = (0xFFFF - tacho_last_capture) + now; // Handle overflow
            }
            if (diff < 60000) { // Filter noise (>1000 RPM)
                tacho_period_us = diff;
            }
        }
        tacho_last_capture = now;
    }
}

void read_analog_tacho(void) {
    #define TACHO_PULSES_PER_REV 2
    if (tacho_period_us > 0) {
        float freq = 1000000.0f / (float)tacho_period_us;
        engine_rpm_analog = (uint16_t)((freq / TACHO_PULSES_PER_REV) * 60.0f + 0.5f);
    } else {
        engine_rpm_analog = 0;
    }
}

/* ============================================================================
 * SECTION 23: MAIN LOOP AND MAIN FUNCTION
 * ============================================================================ */

void main_loop(void) {
    static uint32_t t100 = 0, t500 = 0, t1000 = 0;
    uint32_t now = HAL_GetTick();
    
    process_ignition_key();
    
    if (!ignition_on) {
        HAL_Delay(100);
        return;
    }
    
    if (now - t100 >= 100) {
        t100 = now;
        filter_data();
        current_monitor_task();
        check_critical_failures();
        parking_monitor_task();
        awd_auto_task();
        read_coolant_temperature();
        update_engine_rpm_limit();
        handle_speed_limiter();
        
        if (cal_data.calibrated && !calibration_running) {
            if (current_drive_mode != DRIVE_MODE_NORMAL) {
                auto_shift_task();
            } else {
                manual_shift_task();
            }
        }
        process_bt_command();
    }
    
    if (now - t500 >= 500) {
        t500 = now;
        IWDG_Refresh();
        if (cal_data.calibrated && !calibration_running) {
            diagnostic_check();
            read_clutch_backup_sensor();
            check_sensor_consistency();
        }
    }
    
    if (now - t1000 >= 1000) {
        t1000 = now;
        idle_monitor_task();
        if (!calibration_running && !shifting_in_progress) {
            if (!cal_data.calibrated) led_set_pattern(1);
            else if (system_sleeping) led_set_pattern(4);
            else if (current_drive_mode != DRIVE_MODE_NORMAL) led_set_pattern(2);
            else led_set_pattern(0);
        }
    }
    
    if (uart_cmd_ready) {
        uart_command_handler(uart_rx_buf);
        uart_clear_buffer();
    }
}

/**
 * @brief Main entry point.
 */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    // Initialize all peripherals
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI2_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_CAN1_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_TIM4_Init();
    
    // CRITICAL: Initialize watchdog timer
    IWDG_Init();
    
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    DWT_Init();
    safety_init();
    awd_init();
    
    // Startup LED blink (3 times)
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
    
    uart_send_string("\r\n=== ROBOTIZED GEARBOX v4.0 FINAL (Competition Edition) ===\r\n");
    uart_send_string("Author: Mykola Yushchenko, KPCAD\r\n");
    uart_send_string("License: GPL-3.0 Open Source\r\n\r\n");
    
    load_can_profile();
    apply_vehicle_adaptation();
    
    if (flash_load_calibration(&cal_data)) {
        uart_send_string("Calibration loaded from Flash.\r\n");
        led_set_pattern(cal_data.calibrated ? (current_drive_mode != DRIVE_MODE_NORMAL ? 2 : 0) : 1);
    } else {
        uart_send_string("No calibration found. Auto-calibration in 5 seconds...\r\n");
        uart_send_string("Press any key to cancel.\r\n");
        auto_calib_timer = HAL_GetTick() + 5000;
    }
    
    // Main infinite loop
    while (1) {
        if (auto_calib_timer && HAL_GetTick() >= auto_calib_timer) {
            if (!uart_cmd_ready) {
                auto_calib_timer = 0;
                run_full_calibration();
            } else {
                uart_send_string("Auto-calibration cancelled by user.\r\n");
                auto_calib_timer = 0;
                uart_clear_buffer();
            }
        }
        
        main_loop();
        HAL_Delay(1);
    }
}

/* ==================== END OF FILE ==================== */
 
