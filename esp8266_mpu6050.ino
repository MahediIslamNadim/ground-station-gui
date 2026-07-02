/*
 * ESP8266 Flight Controller Firmware
 * Hardware: ESP8266 (NodeMCU 1.0) + MPU6050 + BMP280
 * 
 * Features: Madgwick fusion, PID control, PPM RC input,
 *           Quad-X mixer, flight modes, MAVLink telemetry,
 *           WiFi AP, EEPROM params, safety systems, binary logging
 */

#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define PIN_SDA       4     // D2 GPIO4
#define PIN_SCL       5     // D1 GPIO5
#define PIN_MOTOR1    0     // D3 GPIO0
#define PIN_MOTOR2    14    // D5 GPIO14
#define PIN_MOTOR3    12    // D6 GPIO12
#define PIN_MOTOR4    13    // D7 GPIO13
#define PIN_BATTERY   A0    // Battery voltage ADC
#define PIN_PPM       16    // D0 GPIO16 - PPM input
#define PIN_LED       2     // D4 GPIO2 - Status LED

// ============================================================================
// I2C ADDRESSES
// ============================================================================
#define MPU6050_ADDR_LOW   0x68
#define MPU6050_ADDR_HIGH  0x69
#define BMP280_ADDR        0x76

// ============================================================================
// EEPROM LAYOUT
// ============================================================================
#define EEPROM_SIZE         512
#define EEPROM_MAGIC_ADDR   96
#define EEPROM_MAGIC_VALUE  0xABCD
#define EEPROM_DATA_ADDR    100

// ============================================================================
// CONSTANTS
// ============================================================================
#define LOOP_FREQ_HZ        500
#define LOOP_PERIOD_US      (1000000 / LOOP_FREQ_HZ)
#define TELEMETRY_FREQ_HZ   50
#define LED_FAST_BLINK_MS   100
#define LED_SLOW_BLINK_MS   500
#define RC_TIMEOUT_MS       500
#define ARM_THROTTLE_MIN    1100
#define ARM_THROTTLE_MAX    1200
#define ARM_YAW_RIGHT       1800
#define ARM_YAW_LEFT        1200
#define ARM_TIME_MS         3000
#define DISARM_TIME_MS      3000
#define MOTOR_MINPWM        1000
#define MOTOR_MAXPWM        2000
#define MOTOR_IDLEPWM       1100
#define BATTERY_VOLTAGE_MIN 3.3
#define BATTERY_VOLTAGE_MAX 4.2
#define CRASH_ACCEL_G       4.0
#define RC_CHANNELS         8

// ============================================================================
// FLIGHT MODES
// ============================================================================
#define MODE_STABILIZE  0
#define MODE_ACRO       1
#define MODE_ALTHOLD    2
#define MODE_RTL        3
#define MODE_LAND       4

// ============================================================================
// MAVLink MESSAGE IDs
// ============================================================================
#define MAVLINK_MSG_ID_HEARTBEAT      0
#define MAVLINK_MSG_ID_SYS_STATUS     1
#define MAVLINK_MSG_ID_BATTERY_STATUS 14
#define MAVLINK_MSG_ID_RADIO_STATUS   101
#define MAVLINK_MSG_ID_GPS_RAW_INT    24
#define MAVLINK_MSG_ID_ATTITUDE       30
#define MAVLINK_MSG_ID_RAW_IMU        27
#define MAVLINK_MSG_ID_SERVO_OUTPUT_RAW 36
#define MAVLINK_MSG_ID_RC_CHANNELS    65
#define MAVLINK_MSG_ID_PARAM_VALUE    22

// ============================================================================
// STRUCTURES
// ============================================================================

struct Quaternion {
    float w, x, y, z;
};

struct Vector3f {
    float x, y, z;
};

struct PIDController {
    float kp, ki, kd;
    float i_limit;
    float d_filter;
    float output_limit;
    float integrator;
    float prev_error;
    float d_filtered;
    float output;
};

struct RCData {
    uint16_t channels[RC_CHANNELS];
    uint16_t raw[RC_CHANNELS];
    uint16_t rc_min[RC_CHANNELS];
    uint16_t rc_max[RC_CHANNELS];
    uint16_t rc_mid[RC_CHANNELS];
    float expo[RC_CHANNELS];
    float deadband[RC_CHANNELS];
    bool valid;
    unsigned long last_update;
};

struct MotorOutput {
    uint16_t m1, m2, m3, m4;
    bool armed;
    bool test_mode;
    uint16_t test_values[4];
};

struct FlightData {
    Quaternion attitude;
    Vector3f accel;
    Vector3f gyro;
    Vector3f mag;
    float roll, pitch, yaw;
    float roll_rate, pitch_rate, yaw_rate;
    float baro_alt;
    float baro_temp;
    float battery_voltage;
    float battery_current;
    uint8_t battery_percent;
    bool gps_valid;
    int32_t gps_lat, gps_lon, gps_alt;
    uint8_t flight_mode;
    bool armed;
    bool failsafe;
    unsigned long timestamp;
};

struct EEPROMData {
    uint16_t magic;
    PIDController pid_rate_roll;
    PIDController pid_rate_pitch;
    PIDController pid_rate_yaw;
    PIDController pid_angle_roll;
    PIDController pid_angle_pitch;
    Vector3f accel_offset;
    Vector3f accel_scale;
    Vector3f gyro_offset;
    Vector3f mag_offset;
    Vector3f mag_scale;
    uint16_t rc_min[RC_CHANNELS];
    uint16_t rc_max[RC_CHANNELS];
    uint16_t rc_mid[RC_CHANNELS];
    float expo[RC_CHANNELS];
    uint16_t failsafe_values[RC_CHANNELS];
    float throttle_expo;
    uint8_t mode_channel;
    float max_altitude;
    float max_distance;
    float bat_voltage_min;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

ESP8266WebServer server(80);

// Sensor data
FlightData fd;
EEPROMData eeprom_data;

// PID controllers
PIDController pid_rate_roll, pid_rate_pitch, pid_rate_yaw;
PIDController pid_angle_roll, pid_angle_pitch;

// RC
RCData rc;
volatile unsigned long ppm_last_pulse = 0;
volatile uint8_t ppm_channel_index = 0;
volatile uint16_t ppm_buffer[RC_CHANNELS];
volatile bool ppm_frame_complete = false;

// Motors
MotorOutput motors;

// Timing
unsigned long last_loop_time = 0;
unsigned long last_telemetry_time = 0;
unsigned long last_led_time = 0;
unsigned long arm_start_time = 0;
unsigned long disarm_start_time = 0;
unsigned long loop_count = 0;

// LED state
uint8_t led_state = 0;
bool led_on = false;

// Calibration
bool calibration_mode = false;
float accel_cal_sum[3] = {0, 0, 0};
float gyro_cal_sum[3] = {0, 0, 0};
float mag_cal_sum[3] = {0, 0, 0};
uint32_t cal_sample_count = 0;

// MPU6050
uint8_t mpu_addr = 0;
bool mpu_found = false;

// BMP280
bool bmp_found = false;
float bmp_pressure;
float bmp_temperature;
float bmp_altitude;
float bmp_sea_level_pressure = 101325.0;

// WiFi
const char* ap_ssid = "DroneCal-AP";
const char* ap_pass = "12345678";
bool wifi_connected = false;

// Serial command buffer
char serial_cmd_buf[128];
uint8_t serial_cmd_len = 0;

// Log enable
bool log_enable = false;

// Safety
bool crash_detected = false;
float home_lat = 0, home_lon = 0;
bool home_set = false;

// ============================================================================
// MADGWICK SENSOR FUSION
// ============================================================================

float madgwick_beta = 0.1;

void quaternionInit(Quaternion *q) {
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void quaternionNormalize(Quaternion *q) {
    float norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    }
}

void quaternionToEuler(Quaternion *q, float *roll, float *pitch, float *yaw) {
    *roll = atan2(2.0f * (q->w * q->x + q->y * q->z),
                  1.0f - 2.0f * (q->x * q->x + q->y * q->y));
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabs(sinp) >= 1.0f) {
        *pitch = copysign(M_PI / 2.0f, sinp);
    } else {
        *pitch = asin(sinp);
    }
    *yaw = atan2(2.0f * (q->w * q->z + q->x * q->y),
                 1.0f - 2.0f * (q->y * q->y + q->z * q->z));
}

void madgwickUpdate(Quaternion *q, Vector3f *accel, Vector3f *gyro, Vector3f *mag, float dt) {
    float ax = accel->x, ay = accel->y, az = accel->z;
    float gx = gyro->x, gy = gyro->y, gz = gyro->z;
    float mx = mag->x, my = mag->y, mz = mag->z;
    float q0 = q->w, q1 = q->x, q2 = q->y, q3 = q->z;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        // Normalise step magnitude
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= madgwick_beta * s0;
        qDot2 -= madgwick_beta * s1;
        qDot3 -= madgwick_beta * s2;
        qDot4 -= madgwick_beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q->w = q0;
    q->x = q1;
    q->y = q2;
    q->z = q3;
}

// Fast inverse square root
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// ============================================================================
// PID CONTROLLER
// ============================================================================

void pidInit(PIDController *pid, float kp, float ki, float kd,
             float i_limit, float d_filter, float output_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_limit = i_limit;
    pid->d_filter = d_filter;
    pid->output_limit = output_limit;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_filtered = 0.0f;
    pid->output = 0.0f;
}

float pidUpdate(PIDController *pid, float error, float dt) {
    if (dt <= 0.0f) return pid->output;

    // Proportional
    float p_term = pid->kp * error;

    // Integral with anti-windup
    pid->integrator += error * dt;
    if (pid->integrator > pid->i_limit) pid->integrator = pid->i_limit;
    if (pid->integrator < -pid->i_limit) pid->integrator = -pid->i_limit;
    float i_term = pid->ki * pid->integrator;

    // Derivative with low-pass filter
    float d_raw = (error - pid->prev_error) / dt;
    pid->d_filtered = pid->d_filtered * pid->d_filter + d_raw * (1.0f - pid->d_filter);
    float d_term = pid->kd * pid->d_filtered;

    pid->prev_error = error;

    // Sum and clamp
    pid->output = p_term + i_term + d_term;
    if (pid->output > pid->output_limit) pid->output = pid->output_limit;
    if (pid->output < -pid->output_limit) pid->output = -pid->output_limit;

    return pid->output;
}

void pidReset(PIDController *pid) {
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->d_filtered = 0.0f;
    pid->output = 0.0f;
}

// ============================================================================
// I2C HELPERS
// ============================================================================

void i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t i2cReadByte(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.read();
}

void i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t *buf) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, count);
    for (uint8_t i = 0; i < count && Wire.available(); i++) {
        buf[i] = Wire.read();
    }
}

// ============================================================================
// MPU6050
// ============================================================================

bool mpu6050Init() {
    // Try low address first
    Wire.beginTransmission(MPU6050_ADDR_LOW);
    if (Wire.endTransmission() == 0) {
        mpu_addr = MPU6050_ADDR_LOW;
        mpu_found = true;
    } else {
        Wire.beginTransmission(MPU6050_ADDR_HIGH);
        if (Wire.endTransmission() == 0) {
            mpu_addr = MPU6050_ADDR_HIGH;
            mpu_found = true;
        } else {
            mpu_found = false;
            return false;
        }
    }

    // Reset
    i2cWriteByte(mpu_addr, 0x6B, 0x80);
    delay(100);

    // Wake up, clock from gyro x
    i2cWriteByte(mpu_addr, 0x6B, 0x01);
    delay(10);

    // DLPF config: 44Hz bandwidth
    i2cWriteByte(mpu_addr, 0x1A, 0x03);

    // Gyro config: ±500°/s
    i2cWriteByte(mpu_addr, 0x1B, 0x08);

    // Accel config: ±4G
    i2cWriteByte(mpu_addr, 0x1C, 0x08);

    // Enable I2C bypass for BMP280
    i2cWriteByte(mpu_addr, 0x37, 0x02);

    return true;
}

void mpu6050ReadAccel(Vector3f *accel) {
    uint8_t buf[6];
    i2cReadBytes(mpu_addr, 0x3B, 6, buf);
    int16_t raw_x = (buf[0] << 8) | buf[1];
    int16_t raw_y = (buf[2] << 8) | buf[3];
    int16_t raw_z = (buf[4] << 8) | buf[5];

    // Scale: ±4G = 8192 LSB/g
    accel->x = (float)raw_x / 8192.0f;
    accel->y = (float)raw_y / 8192.0f;
    accel->z = (float)raw_z / 8192.0f;

    // Apply calibration
    accel->x = accel->x * eeprom_data.accel_scale.x + eeprom_data.accel_offset.x;
    accel->y = accel->y * eeprom_data.accel_scale.y + eeprom_data.accel_offset.y;
    accel->z = accel->z * eeprom_data.accel_scale.z + eeprom_data.accel_offset.z;
}

void mpu6050ReadGyro(Vector3f *gyro) {
    uint8_t buf[6];
    i2cReadBytes(mpu_addr, 0x43, 6, buf);
    int16_t raw_x = (buf[0] << 8) | buf[1];
    int16_t raw_y = (buf[2] << 8) | buf[3];
    int16_t raw_z = (buf[4] << 8) | buf[5];

    // Scale: ±500°/s = 65.5 LSB/(°/s) → convert to rad/s
    gyro->x = (float)raw_x / 65.5f * DEG_TO_RAD;
    gyro->y = (float)raw_y / 65.5f * DEG_TO_RAD;
    gyro->z = (float)raw_z / 65.5f * DEG_TO_RAD;

    // Apply calibration
    gyro->x -= eeprom_data.gyro_offset.x;
    gyro->y -= eeprom_data.gyro_offset.y;
    gyro->z -= eeprom_data.gyro_offset.z;
}

// ============================================================================
// BMP280
// ============================================================================

uint32_t bmp280_compensate_T(int32_t adc_T);
uint32_t bmp280_compensate_P(int32_t adc_P);

// BMP280 compensation parameters (stored from calibration)
uint16_t bmp280_dig_T1;
int16_t  bmp280_dig_T2, bmp280_dig_T3;
uint16_t bmp280_dig_P1;
int16_t  bmp280_dig_P2, bmp280_dig_P3, bmp280_dig_P4, bmp280_dig_P5;
int16_t  bmp280_dig_P6, bmp280_dig_P7, bmp280_dig_P8, bmp280_dig_P9;
int32_t  bmp280_t_fine;

bool bmp280Init() {
    Wire.beginTransmission(BMP280_ADDR);
    if (Wire.endTransmission() != 0) {
        bmp_found = false;
        return false;
    }

    // Read chip ID
    uint8_t id = i2cReadByte(BMP280_ADDR, 0xD0);
    if (id != 0x58 && id != 0x60) {
        bmp_found = false;
        return false;
    }

    // Read calibration data
    uint8_t cal[26];
    i2cReadBytes(BMP280_ADDR, 0x88, 26, cal);
    bmp280_dig_T1 = (cal[1] << 8) | cal[0];
    bmp280_dig_T2 = (cal[3] << 8) | cal[2];
    bmp280_dig_T3 = (cal[5] << 8) | cal[4];
    bmp280_dig_P1 = (cal[7] << 8) | cal[6];
    bmp280_dig_P2 = (cal[9] << 8) | cal[8];
    bmp280_dig_P3 = (cal[11] << 8) | cal[10];
    bmp280_dig_P4 = (cal[13] << 8) | cal[12];
    bmp280_dig_P5 = (cal[15] << 8) | cal[14];
    bmp280_dig_P6 = (cal[17] << 8) | cal[16];
    bmp280_dig_P7 = (cal[19] << 8) | cal[18];
    bmp280_dig_P8 = (cal[21] << 8) | cal[20];
    bmp280_dig_P9 = (cal[23] << 8) | cal[22];

    // Config: osrs_t=x1, osrs_p=x1, mode=normal, standby=500ms
    i2cWriteByte(BMP280_ADDR, 0xF4, 0x27);
    // Config register: standby=500ms, filter=x16
    i2cWriteByte(BMP280_ADDR, 0xF5, 0x90);

    bmp_found = true;
    return true;
}

uint32_t bmp280_compensate_T(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_dig_T1 << 1))) * ((int32_t)bmp280_dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_dig_T1))) >> 12) * ((int32_t)bmp280_dig_T3)) >> 14;
    bmp280_t_fine = var1 + var2;
    T = (bmp280_t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t bmp280_compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_dig_P3) >> 8) + ((var1 * (int64_t)bmp280_dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_dig_P7) << 4);
    return (uint32_t)p;
}

void bmp280Read() {
    if (!bmp_found) return;

    uint8_t data[6];
    i2cReadBytes(BMP280_ADDR, 0xF7, 6, data);

    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);

    bmp280_compensate_T(adc_T);
    uint32_t raw_p = bmp280_compensate_P(adc_P);

    bmp_temperature = (float)bmp280_t_fine / 256.0f;
    bmp_pressure = (float)raw_p / 256.0f;
    bmp_altitude = 44330.0f * (1.0f - pow(bmp_pressure / bmp_sea_level_pressure, 0.1903f));
}

// ============================================================================
// RC INPUT (PPM)
// ============================================================================

void IRAM_ATTR ppmISR() {
    unsigned long now = micros();
    unsigned long elapsed = now - ppm_last_pulse;
    ppm_last_pulse = now;

    if (elapsed > 3000) {
        // Sync pulse - frame start
        if (ppm_channel_index > 0 && ppm_channel_index <= RC_CHANNELS) {
            for (uint8_t i = 0; i < ppm_channel_index && i < RC_CHANNELS; i++) {
                rc.raw[i] = ppm_buffer[i];
            }
            if (ppm_channel_index >= RC_CHANNELS) {
                ppm_frame_complete = true;
            }
        }
        ppm_channel_index = 0;
    } else if (elapsed > 500 && elapsed < 3000) {
        // Channel pulse
        if (ppm_channel_index < RC_CHANNELS) {
            ppm_buffer[ppm_channel_index] = elapsed;
            ppm_channel_index++;
        }
    }
}

void rcInit() {
    pinMode(PIN_PPM, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_PPM), ppmISR, RISING);

    // Init default values
    for (uint8_t i = 0; i < RC_CHANNELS; i++) {
        rc.channels[i] = 1500;
        rc.raw[i] = 1500;
        rc.rc_min[i] = 1000;
        rc.rc_max[i] = 2000;
        rc.rc_mid[i] = 1500;
        rc.expo[i] = 0.0f;
        rc.deadband[i] = 30.0f;
    }
    rc.valid = false;
    rc.last_update = 0;

    // Default failsafe
    eeprom_data.failsafe_values[0] = 1500;  // roll
    eeprom_data.failsafe_values[1] = 1500;  // pitch
    eeprom_data.failsafe_values[2] = 1000;  // throttle
    eeprom_data.failsafe_values[3] = 1500;  // yaw
    eeprom_data.failsafe_values[4] = 1000;  // mode
    eeprom_data.failsafe_values[5] = 1500;  // aux1
    eeprom_data.failsafe_values[6] = 1500;  // aux2
    eeprom_data.failsafe_values[7] = 1500;  // aux3

    eeprom_data.mode_channel = 4;  // ch5 index 4
}

float applyExpo(float input, float expo) {
    // input: -1.0 to 1.0, expo: 0.0 to 1.0
    if (expo == 0.0f) return input;
    float abs_input = fabs(input);
    float sign = (input >= 0.0f) ? 1.0f : -1.0f;
    return sign * (abs_input * (1.0f - expo) + abs_input * abs_input * abs_input * expo);
}

float applyDeadband(float input, float deadband) {
    if (fabs(input) < deadband) return 0.0f;
    if (input > 0.0f) return (input - deadband) / (1.0f - deadband);
    return (input + deadband) / (1.0f - deadband);
}

void rcUpdate() {
    if (ppm_frame_complete) {
        ppm_frame_complete = false;
        rc.last_update = millis();
        rc.valid = true;
    }

    // Check failsafe
    if (rc.valid && (millis() - rc.last_update > RC_TIMEOUT_MS)) {
        rc.valid = false;
        fd.failsafe = true;
        // Apply failsafe values
        for (uint8_t i = 0; i < RC_CHANNELS; i++) {
            rc.channels[i] = eeprom_data.failsafe_values[i];
        }
    } else if (rc.valid) {
        fd.failsafe = false;
    }

    // Map raw to channels with endpoints
    for (uint8_t i = 0; i < RC_CHANNELS; i++) {
        uint16_t raw = rc.raw[i];
        uint16_t min_val = eeprom_data.rc_min[i];
        uint16_t max_val = eeprom_data.rc_max[i];
        uint16_t mid_val = eeprom_data.rc_mid[i];

        // Constrain to valid range
        if (raw < min_val) raw = min_val;
        if (raw > max_val) raw = max_val;

        // Map to 1000-2000
        float normalized = (float)(raw - min_val) / (float)(max_val - min_val);
        float output = 1000.0f + normalized * 1000.0f;

        // Apply expo to axes (roll, pitch, yaw)
        if (i == 0 || i == 1 || i == 3) {
            float centered = (output - 1500.0f) / 500.0f;
            centered = applyExpo(centered, eeprom_data.expo[i]);
            centered = applyDeadband(centered, 0.05f);
            output = 1500.0f + centered * 500.0f;
        }

        rc.channels[i] = (uint16_t)constrain(output, 1000.0f, 2000.0f);
    }
}

// ============================================================================
// MOTOR MIXER
// ============================================================================

void motorInit() {
    pinMode(PIN_MOTOR1, OUTPUT);
    pinMode(PIN_MOTOR2, OUTPUT);
    pinMode(PIN_MOTOR3, OUTPUT);
    pinMode(PIN_MOTOR4, OUTPUT);

    analogWriteFreq(400);  // 400Hz PWM
    analogWriteRange(1023); // 10-bit resolution → map 1000-2000 to 0-1023

    motors.m1 = MOTOR_MINPWM;
    motors.m2 = MOTOR_MINPWM;
    motors.m3 = MOTOR_MINPWM;
    motors.m4 = MOTOR_MINPWM;
    motors.armed = false;
    motors.test_mode = false;
    motorWriteAll(MOTOR_MINPWM, MOTOR_MINPWM, MOTOR_MINPWM, MOTOR_MINPWM);
}

uint16_t pwmToAnalog(uint16_t pwm) {
    // Map 1000-2000 to 0-1023
    return map(constrain(pwm, 1000, 2000), 1000, 2000, 0, 1023);
}

void motorWriteAll(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    if (!motors.armed && !motors.test_mode) {
        m1 = m2 = m3 = m4 = MOTOR_MINPWM;
    }

    analogWrite(PIN_MOTOR1, pwmToAnalog(m1));
    analogWrite(PIN_MOTOR2, pwmToAnalog(m2));
    analogWrite(PIN_MOTOR3, pwmToAnalog(m3));
    analogWrite(PIN_MOTOR4, pwmToAnalog(m4));

    motors.m1 = m1;
    motors.m2 = m2;
    motors.m3 = m3;
    motors.m4 = m4;
}

void motorMix(float throttle, float roll, float pitch, float yaw) {
    if (motors.test_mode) {
        motorWriteAll(motors.test_values[0], motors.test_values[1],
                      motors.test_values[2], motors.test_values[3]);
        return;
    }

    // Apply throttle curve
    float t = throttle;
    if (eeprom_data.throttle_expo > 0.0f) {
        t = t * (1.0f - eeprom_data.throttle_expo) + t * t * t * eeprom_data.throttle_expo;
    }

    // Quad X mixer
    // M1 = back-right (CW)
    // M2 = front-right (CCW)
    // M3 = back-left (CCW)
    // M4 = front-left (CW)
    float m1 = t - roll + pitch - yaw;
    float m2 = t + roll + pitch + yaw;
    float m3 = t - roll - pitch + yaw;
    float m4 = t + roll - pitch - yaw;

    // Constrain
    m1 = constrain(m1, (float)MOTOR_MINPWM, (float)MOTOR_MAXPWM);
    m2 = constrain(m2, (float)MOTOR_MINPWM, (float)MOTOR_MAXPWM);
    m3 = constrain(m3, (float)MOTOR_MINPWM, (float)MOTOR_MAXPWM);
    m4 = constrain(m4, (float)MOTOR_MINPWM, (float)MOTOR_MAXPWM);

    // Idle throttle when armed
    if (motors.armed && t < MOTOR_IDLEPWM) {
        m1 = m2 = m3 = m4 = MOTOR_IDLEPWM;
    }

    motorWriteAll((uint16_t)m1, (uint16_t)m2, (uint16_t)m3, (uint16_t)m4);
}

// ============================================================================
// FLIGHT MODES
// ============================================================================

void flightModeUpdate() {
    uint16_t mode_pwm = rc.channels[eeprom_data.mode_channel];

    if (mode_pwm < 1250) {
        fd.flight_mode = MODE_STABILIZE;
    } else if (mode_pwm < 1750) {
        fd.flight_mode = MODE_ACRO;
    } else if (mode_pwm < 2000) {
        fd.flight_mode = MODE_ALTHOLD;
    } else {
        fd.flight_mode = MODE_RTL;
    }
}

void stabilizeControl() {
    // Angle PID → rate target
    float roll_target = map((float)rc.channels[0], 1000.0f, 2000.0f, -30.0f, 30.0f);
    float pitch_target = map((float)rc.channels[1], 1000.0f, 2000.0f, -30.0f, 30.0f);
    float throttle = (float)rc.channels[2];

    // Angle PID
    float roll_angle_error = roll_target - fd.roll * RAD_TO_DEG;
    float pitch_angle_error = pitch_target - fd.pitch * RAD_TO_DEG;

    float roll_rate_target = pidUpdate(&pid_angle_roll, roll_angle_error, 1.0f / LOOP_FREQ_HZ);
    float pitch_rate_target = pidUpdate(&pid_angle_pitch, pitch_angle_error, 1.0f / LOOP_FREQ_HZ);

    // Rate PID
    float roll_rate_error = roll_rate_target - fd.gyro.x * RAD_TO_DEG;
    float pitch_rate_error = pitch_rate_target - fd.gyro.y * RAD_TO_DEG;
    float yaw_rate_error = map((float)rc.channels[3], 1000.0f, 2000.0f, -200.0f, 200.0f) - fd.gyro.z * RAD_TO_DEG;

    float roll_output = pidUpdate(&pid_rate_roll, roll_rate_error, 1.0f / LOOP_FREQ_HZ);
    float pitch_output = pidUpdate(&pid_rate_pitch, pitch_rate_error, 1.0f / LOOP_FREQ_HZ);
    float yaw_output = pidUpdate(&pid_rate_yaw, yaw_rate_error, 1.0f / LOOP_FREQ_HZ);

    motorMix(throttle, roll_output, pitch_output, yaw_output);
}

void acroControl() {
    float throttle = (float)rc.channels[2];
    float roll_rate_target = map((float)rc.channels[0], 1000.0f, 2000.0f, -400.0f, 400.0f);
    float pitch_rate_target = map((float)rc.channels[1], 1000.0f, 2000.0f, -400.0f, 400.0f);
    float yaw_rate_target = map((float)rc.channels[3], 1000.0f, 2000.0f, -200.0f, 200.0f);

    // Rate PID
    float roll_rate_error = roll_rate_target - fd.gyro.x * RAD_TO_DEG;
    float pitch_rate_error = pitch_rate_target - fd.gyro.y * RAD_TO_DEG;
    float yaw_rate_error = yaw_rate_target - fd.gyro.z * RAD_TO_DEG;

    float roll_output = pidUpdate(&pid_rate_roll, roll_rate_error, 1.0f / LOOP_FREQ_HZ);
    float pitch_output = pidUpdate(&pid_rate_pitch, pitch_rate_error, 1.0f / LOOP_FREQ_HZ);
    float yaw_output = pidUpdate(&pid_rate_yaw, yaw_rate_error, 1.0f / LOOP_FREQ_HZ);

    motorMix(throttle, roll_output, pitch_output, yaw_output);
}

float althold_target = 0.0f;
float alt_pid_integrator = 0.0f;
float alt_pid_prev_error = 0.0f;
float alt_kp = 2.0f;
float alt_ki = 0.5f;
float alt_kd = 0.1f;

void altholdControl() {
    // Use barometer for altitude
    float current_alt = fd.baro_alt;
    float throttle_stick = (float)rc.channels[1];  // pitch stick for altitude

    // Target altitude from stick (1500 = maintain current, above = climb, below = descend)
    if (rc.channels[2] > 1100) {
        // Throttle active - manual altitude control with angle mode
        stabilizeControl();
        return;
    }

    // Set target on stick center
    althold_target = current_alt;

    // Altitude PID
    float alt_error = althold_target - current_alt;
    alt_pid_integrator += alt_error * (1.0f / LOOP_FREQ_HZ);
    alt_pid_integrator = constrain(alt_pid_integrator, -500.0f, 500.0f);
    float alt_d = (alt_error - alt_pid_prev_error) * LOOP_FREQ_HZ;
    alt_pid_prev_error = alt_error;

    float throttle_output = 1500.0f + alt_kp * alt_error + alt_ki * alt_pid_integrator + alt_kd * alt_d;
    throttle_output = constrain(throttle_output, 1100.0f, 2000.0f);

    // Still stabilize angles
    float roll_target = map((float)rc.channels[0], 1000.0f, 2000.0f, -30.0f, 30.0f);
    float pitch_target = map((float)rc.channels[1], 1000.0f, 2000.0f, -30.0f, 30.0f);

    float roll_angle_error = roll_target - fd.roll * RAD_TO_DEG;
    float pitch_angle_error = pitch_target - fd.pitch * RAD_TO_DEG;

    float roll_rate_target = pidUpdate(&pid_angle_roll, roll_angle_error, 1.0f / LOOP_FREQ_HZ);
    float pitch_rate_target = pidUpdate(&pid_angle_pitch, pitch_angle_error, 1.0f / LOOP_FREQ_HZ);

    float roll_rate_error = roll_rate_target - fd.gyro.x * RAD_TO_DEG;
    float pitch_rate_error = pitch_rate_target - fd.gyro.y * RAD_TO_DEG;
    float yaw_rate_error = map((float)rc.channels[3], 1000.0f, 2000.0f, -200.0f, 200.0f) - fd.gyro.z * RAD_TO_DEG;

    float roll_output = pidUpdate(&pid_rate_roll, roll_rate_error, 1.0f / LOOP_FREQ_HZ);
    float pitch_output = pidUpdate(&pid_rate_pitch, pitch_rate_error, 1.0f / LOOP_FREQ_HZ);
    float yaw_output = pidUpdate(&pid_rate_yaw, yaw_rate_error, 1.0f / LOOP_FREQ_HZ);

    motorMix(throttle_output, roll_output, pitch_output, yaw_output);
}

void rtlControl() {
    // Simple RTL: just land slowly
    if (rc.channels[2] > 1100) {
        // Override with manual control
        stabilizeControl();
        return;
    }

    // Slowly descend
    float throttle = 1200.0f;

    float roll_output = pidUpdate(&pid_rate_roll, -fd.gyro.x * RAD_TO_DEG, 1.0f / LOOP_FREQ_HZ);
    float pitch_output = pidUpdate(&pid_rate_pitch, -fd.gyro.y * RAD_TO_DEG, 1.0f / LOOP_FREQ_HZ);
    float yaw_output = pidUpdate(&pid_rate_yaw, -fd.gyro.z * RAD_TO_DEG, 1.0f / LOOP_FREQ_HZ);

    motorMix(throttle, roll_output, pitch_output, yaw_output);
}

// ============================================================================
// SAFETY SYSTEMS
// ============================================================================

void safetyUpdate() {
    // Check arming/disarming
    uint16_t throttle = rc.channels[2];
    uint16_t yaw = rc.channels[3];

    if (!motors.armed) {
        // Arm: throttle low + yaw right for 3s
        if (throttle < ARM_THROTTLE_MAX && yaw > ARM_YAW_RIGHT) {
            if (arm_start_time == 0) {
                arm_start_time = millis();
            } else if (millis() - arm_start_time > ARM_TIME_MS) {
                motors.armed = true;
                fd.armed = true;
                arm_start_time = 0;
                pidReset(&pid_rate_roll);
                pidReset(&pid_rate_pitch);
                pidReset(&pid_rate_yaw);
                pidReset(&pid_angle_roll);
                pidReset(&pid_angle_pitch);
            }
        } else {
            arm_start_time = 0;
        }
    } else {
        // Disarm: throttle low + yaw left for 3s
        if (throttle < ARM_THROTTLE_MAX && yaw < ARM_YAW_LEFT) {
            if (disarm_start_time == 0) {
                disarm_start_time = millis();
            } else if (millis() - disarm_start_time > DISARM_TIME_MS) {
                motors.armed = false;
                fd.armed = false;
                disarm_start_time = 0;
            }
        } else {
            disarm_start_time = 0;
        }
    }

    // Battery failsafe
    if (fd.battery_voltage < eeprom_data.bat_voltage_min && fd.battery_voltage > 1.0f) {
        fd.failsafe = true;
    }

    // Crash detection
    float total_accel = sqrt(fd.accel.x * fd.accel.x + fd.accel.y * fd.accel.y + fd.accel.z * fd.accel.z);
    if (total_accel > CRASH_ACCEL_G && motors.armed) {
        crash_detected = true;
        motors.armed = false;
        fd.armed = false;
        motorWriteAll(MOTOR_MINPWM, MOTOR_MINPWM, MOTOR_MINPWM, MOTOR_MINPWM);
    }

    // Geofence
    if (eeprom_data.max_altitude > 0.0f && fd.baro_alt > eeprom_data.max_altitude) {
        // Force descend
        fd.failsafe = true;
    }
}

// ============================================================================
// BATTERY MONITORING
// ============================================================================

void batteryUpdate() {
    // Read ADC with averaging
    static uint32_t bat_sum = 0;
    static uint8_t bat_count = 0;

    bat_sum += analogRead(PIN_BATTERY);
    bat_count++;

    if (bat_count >= 16) {
        float avg_adc = (float)bat_sum / (float)bat_count;
        // 4:1 divider, reference 3.3V, 10-bit ADC
        fd.battery_voltage = (avg_adc / 1023.0f) * 3.3f * 4.0f;

        // Calculate percentage
        fd.battery_percent = (uint8_t)map(constrain((long)(fd.battery_voltage * 100),
                                            (long)(BATTERY_VOLTAGE_MIN * 100),
                                            (long)(BATTERY_VOLTAGE_MAX * 100)),
                                    (long)(BATTERY_VOLTAGE_MIN * 100),
                                    (long)(BATTERY_VOLTAGE_MAX * 100), 0, 100);

        bat_sum = 0;
        bat_count = 0;
    }
}

// ============================================================================
// EEPROM PARAMETER SYSTEM
// ============================================================================

void eepromLoadDefaults() {
    eeprom_data.magic = EEPROM_MAGIC_VALUE;

    // PID defaults
    pidInit(&eeprom_data.pid_rate_roll, 1.0f, 0.0f, 0.01f, 50.0f, 0.8f, 500.0f);
    pidInit(&eeprom_data.pid_rate_pitch, 1.0f, 0.0f, 0.01f, 50.0f, 0.8f, 500.0f);
    pidInit(&eeprom_data.pid_rate_yaw, 1.5f, 0.0f, 0.0f, 50.0f, 0.8f, 500.0f);
    pidInit(&eeprom_data.pid_angle_roll, 2.0f, 0.0f, 0.0f, 20.0f, 0.8f, 200.0f);
    pidInit(&eeprom_data.pid_angle_pitch, 2.0f, 0.0f, 0.0f, 20.0f, 0.8f, 200.0f);

    // Calibration defaults
    eeprom_data.accel_offset = {0, 0, 0};
    eeprom_data.accel_scale = {1, 1, 1};
    eeprom_data.gyro_offset = {0, 0, 0};
    eeprom_data.mag_offset = {0, 0, 0};
    eeprom_data.mag_scale = {1, 1, 1};

    // RC defaults
    for (uint8_t i = 0; i < RC_CHANNELS; i++) {
        eeprom_data.rc_min[i] = 1000;
        eeprom_data.rc_max[i] = 2000;
        eeprom_data.rc_mid[i] = 1500;
        eeprom_data.expo[i] = 0.0f;
    }

    // Safety
    eeprom_data.max_altitude = 50.0f;
    eeprom_data.max_distance = 0.0f;  // disabled
    eeprom_data.bat_voltage_min = 3.3f;
    eeprom_data.throttle_expo = 0.3f;
    eeprom_data.mode_channel = 4;
}

void eepromSave() {
    EEPROM.put(EEPROM_DATA_ADDR, eeprom_data);
    EEPROM.commit();
}

void eepromLoad() {
    EEPROM.get(EEPROM_DATA_ADDR, eeprom_data);
    if (eeprom_data.magic != EEPROM_MAGIC_VALUE) {
        Serial.println("[EEPROM] Invalid magic, loading defaults");
        eepromLoadDefaults();
        eepromSave();
    } else {
        Serial.println("[EEPROM] Loaded valid data");
    }

    // Apply to runtime PID controllers
    pid_rate_roll = eeprom_data.pid_rate_roll;
    pid_rate_pitch = eeprom_data.pid_rate_pitch;
    pid_rate_yaw = eeprom_data.pid_rate_yaw;
    pid_angle_roll = eeprom_data.pid_angle_roll;
    pid_angle_pitch = eeprom_data.pid_angle_pitch;
}

// ============================================================================
// SENSOR READING & FUSION
// ============================================================================

void i2cBusRecover() {
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
    for (int i = 0; i < 9; i++) {
        pinMode(PIN_SCL, OUTPUT);
        digitalWrite(PIN_SCL, LOW);
        delayMicroseconds(5);
        pinMode(PIN_SCL, INPUT_PULLUP);
        delayMicroseconds(5);
    }
    pinMode(PIN_SDA, OUTPUT);
    digitalWrite(PIN_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_SDA, HIGH);
    delayMicroseconds(5);
    pinMode(PIN_SDA, INPUT_PULLUP);
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
}

void readSensors() {
    static uint8_t zero_count = 0;
    static unsigned long last_retry = 0;

    if (!mpu_found) {
        if (millis() - last_retry > 2000) {
            last_retry = millis();
            i2cBusRecover();
            if (mpu6050Init()) {
                Serial.println("[MPU6050] Reinitialized after I2C recovery");
            }
        }
        return;
    }

    mpu6050ReadAccel(&fd.accel);
    mpu6050ReadGyro(&fd.gyro);

    float mag = sqrt(fd.accel.x * fd.accel.x + fd.accel.y * fd.accel.y + fd.accel.z * fd.accel.z);
    if (mag < 0.01f) {
        zero_count++;
        if (zero_count > 10) {
            zero_count = 0;
            mpu_found = false;
            return;
        }
    } else {
        zero_count = 0;
    }

    // Read mag (use accel registers as placeholder - MPU6050 has no mag)
    // For real mag, add HMC5883L or QMC5883L
    fd.mag.x = 0;
    fd.mag.y = 0;
    fd.mag.z = 1.0f;  // Assume level

    // Read barometer
    bmp280Read();
    fd.baro_alt = bmp_altitude;
    fd.baro_temp = bmp_temperature;

    // Battery
    batteryUpdate();
}

void sensorFusion() {
    static unsigned long last_fusion_time = 0;
    unsigned long now = micros();
    float dt = (now - last_fusion_time) / 1000000.0f;
    last_fusion_time = now;

    if (dt <= 0.0f || dt > 0.1f) dt = 1.0f / LOOP_FREQ_HZ;

    madgwickUpdate(&fd.attitude, &fd.accel, &fd.gyro, &fd.mag, dt);
    quaternionToEuler(&fd.attitude, &fd.roll, &fd.pitch, &fd.yaw);

    fd.roll_rate = fd.gyro.x;
    fd.pitch_rate = fd.gyro.y;
    fd.yaw_rate = fd.gyro.z;
}

// ============================================================================
// STATUS LED
// ============================================================================

void ledUpdate() {
    unsigned long now = millis();
    uint16_t blink_interval = LED_FAST_BLINK_MS;

    if (!motors.armed && !rc.valid) {
        // Fast blink: not armed, no RC
        blink_interval = LED_FAST_BLINK_MS;
    } else if (motors.armed && !motors.test_mode) {
        // Solid ON when motors running
        if (motors.m1 > MOTOR_IDLEPWM || motors.m2 > MOTOR_IDLEPWM ||
            motors.m3 > MOTOR_IDLEPWM || motors.m4 > MOTOR_IDLEPWM) {
            digitalWrite(PIN_LED, HIGH);
            return;
        }
        // Slow blink when armed standby
        blink_interval = LED_SLOW_BLINK_MS;
    } else if (fd.failsafe) {
        // Double blink for failsafe
        static uint8_t failsafe_blink = 0;
        static unsigned long last_failsafe_blink = 0;
        if (now - last_failsafe_blink > 200) {
            last_failsafe_blink = now;
            failsafe_blink++;
            if (failsafe_blink >= 4) failsafe_blink = 0;
        }
        digitalWrite(PIN_LED, (failsafe_blink == 0 || failsafe_blink == 1) ? HIGH : LOW);
        return;
    } else {
        // Fast blink: not armed
        blink_interval = LED_FAST_BLINK_MS;
    }

    if (now - last_led_time > blink_interval) {
        last_led_time = now;
        led_on = !led_on;
        digitalWrite(PIN_LED, led_on ? HIGH : LOW);
    }
}

// ============================================================================
// MAVLink-LIKE TELEMETRY
// ============================================================================

void mavSendHeader(uint8_t msg_id, uint8_t length) {
    Serial.write(0xFE);  // Start marker
    Serial.write(length);
    Serial.write(0x00);  // Sequence
    Serial.write(0xFF);  // System ID
    Serial.write(0xBE);  // Component ID
    Serial.write(msg_id);
}

uint8_t mavChecksum(uint8_t *buf, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum ^= buf[i];
    return sum;
}

void sendHeartbeat() {
    uint8_t buf[9];
    buf[0] = 0;   // type: generic
    buf[1] = 0;   // autopilot: generic
    buf[2] = 0;   // base_mode
    buf[3] = 0; buf[4] = 0; buf[5] = 0; buf[6] = 0;  // custom_mode
    buf[7] = motors.armed ? 4 : 0;  // system_status
    buf[8] = 3;   // mavlink_version

    mavSendHeader(MAVLINK_MSG_ID_HEARTBEAT, 9);
    Serial.write(buf, 9);
    Serial.write(mavChecksum(buf, 9));
}

void sendAttitude() {
    uint8_t buf[28];
    uint32_t boot_ms = millis();
    memcpy(buf + 0, &boot_ms, 4);
    float roll = fd.roll;
    float pitch = fd.pitch;
    float yaw = fd.yaw;
    float rollspeed = fd.gyro.x;
    float pitchspeed = fd.gyro.y;
    float yawspeed = fd.gyro.z;
    memcpy(buf + 4, &roll, 4);
    memcpy(buf + 8, &pitch, 4);
    memcpy(buf + 12, &yaw, 4);
    memcpy(buf + 16, &rollspeed, 4);
    memcpy(buf + 20, &pitchspeed, 4);
    memcpy(buf + 24, &yawspeed, 4);

    mavSendHeader(MAVLINK_MSG_ID_ATTITUDE, 28);
    Serial.write(buf, 28);
    Serial.write(mavChecksum(buf, 28));
}

void sendRawIMU() {
    uint8_t buf[26];
    uint64_t usec = (uint64_t)micros();
    memcpy(buf + 0, &usec, 8);
    int16_t xacc = (int16_t)(fd.accel.x * 1000);
    int16_t yacc = (int16_t)(fd.accel.y * 1000);
    int16_t zacc = (int16_t)(fd.accel.z * 1000);
    int16_t xgyro = (int16_t)(fd.gyro.x * 1000);
    int16_t ygyro = (int16_t)(fd.gyro.y * 1000);
    int16_t zgyro = (int16_t)(fd.gyro.z * 1000);
    int16_t xmag = (int16_t)(fd.mag.x * 1000);
    int16_t ymag = (int16_t)(fd.mag.y * 1000);
    int16_t zmag = (int16_t)(fd.mag.z * 1000);
    memcpy(buf + 8, &xacc, 2);
    memcpy(buf + 10, &yacc, 2);
    memcpy(buf + 12, &zacc, 2);
    memcpy(buf + 14, &xgyro, 2);
    memcpy(buf + 16, &ygyro, 2);
    memcpy(buf + 18, &zgyro, 2);
    memcpy(buf + 20, &xmag, 2);
    memcpy(buf + 22, &ymag, 2);
    memcpy(buf + 24, &zmag, 2);

    mavSendHeader(MAVLINK_MSG_ID_RAW_IMU, 26);
    Serial.write(buf, 26);
    Serial.write(mavChecksum(buf, 26));
}

void sendBattery() {
    uint8_t buf[10];
    int16_t voltage = (int16_t)(fd.battery_voltage * 1000);
    int16_t current = (int16_t)(fd.battery_current * 100);
    buf[0] = 0; buf[1] = 0;  // battery_id
    memcpy(buf + 2, &voltage, 2);
    memcpy(buf + 4, &current, 2);
    buf[6] = fd.battery_percent;
    buf[7] = 0;  // battery_remaining (0 = unknown)
    buf[8] = 0; buf[9] = 0;  // battery_function, type

    mavSendHeader(MAVLINK_MSG_ID_BATTERY_STATUS, 10);
    Serial.write(buf, 10);
    Serial.write(mavChecksum(buf, 10));
}

void sendRCChannels() {
    uint8_t buf[33];
    uint32_t boot_ms = millis();
    memcpy(buf + 0, &boot_ms, 4);
    for (uint8_t i = 0; i < 8; i++) {
        uint16_t ch = rc.channels[i];
        memcpy(buf + 4 + i * 2, &ch, 2);
    }
    buf[20] = 0;  // rssi

    mavSendHeader(MAVLINK_MSG_ID_RC_CHANNELS, 21);
    Serial.write(buf, 21);
    Serial.write(mavChecksum(buf, 21));
}

void sendServoOutput() {
    uint8_t buf[16];
    buf[0] = 0;  // port
    uint16_t servo_out[8] = {motors.m1, motors.m2, motors.m3, motors.m4, 0, 0, 0, 0};
    for (uint8_t i = 0; i < 8; i++) {
        memcpy(buf + 1 + i * 2, &servo_out[i], 2);
    }

    mavSendHeader(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 16);
    Serial.write(buf, 16);
    Serial.write(mavChecksum(buf, 16));
}

void sendSysStatus() {
    uint8_t buf[31];
    int16_t voltage = (int16_t)(fd.battery_voltage * 1000);
    int16_t current = (int16_t)(fd.battery_current * 100);
    memcpy(buf + 0, &voltage, 2);
    memcpy(buf + 2, &current, 2);
    buf[4] = fd.battery_percent;
    buf[5] = 0;  // drop_rate_comm
    buf[6] = 0; buf[7] = 0;  // errors_comm
    buf[8] = 0; buf[9] = 0;  // errors_count1
    buf[10] = 0; buf[11] = 0;  // errors_count2
    buf[12] = 0; buf[13] = 0;  // errors_count3
    buf[14] = 0; buf[15] = 0;  // errors_count4
    uint16_t load = (uint16_t)(loop_count / 100);
    memcpy(buf + 16, &load, 2);
    buf[18] = motors.armed ? 4 : 0;  // system_status

    mavSendHeader(MAVLINK_MSG_ID_SYS_STATUS, 18);
    Serial.write(buf, 18);
    Serial.write(mavChecksum(buf, 18));
}

void sendGPSRaw() {
    uint8_t buf[30];
    uint64_t usec = (uint64_t)micros();
    memcpy(buf + 0, &usec, 8);
    int32_t lat = home_set ? (int32_t)(home_lat * 1e7) : 0;
    int32_t lon = home_set ? (int32_t)(home_lon * 1e7) : 0;
    int32_t alt = (int32_t)(fd.baro_alt * 1000);
    memcpy(buf + 8, &lat, 4);
    memcpy(buf + 12, &lon, 4);
    memcpy(buf + 16, &alt, 4);
    buf[20] = 0;  // fix_type
    buf[21] = 0;  // satellites_visible
    buf[22] = 0;  // eph
    buf[23] = 0;  // epv
    buf[24] = 0; buf[25] = 0;  // cog
    buf[26] = 0;  // satellites_visible
    buf[27] = 0;  // alt_ellipsoid

    mavSendHeader(MAVLINK_MSG_ID_GPS_RAW_INT, 28);
    Serial.write(buf, 28);
    Serial.write(mavChecksum(buf, 28));
}

// ============================================================================
// SERIAL COMMAND HANDLER
// ============================================================================

void handleSerialCommand(char* cmd) {
    // Convert to uppercase for comparison
    char upper[128];
    uint8_t len = strlen(cmd);
    for (uint8_t i = 0; i < len; i++) upper[i] = toupper(cmd[i]);
    upper[len] = 0;

    if (strncmp(upper, "HELP", 4) == 0) {
        Serial.println("=== ESP8266 Flight Controller ===");
        Serial.println("Commands:");
        Serial.println("  HELP        - Show this help");
        Serial.println("  STATUS      - Show system status");
        Serial.println("  I2C_SCAN    - Scan I2C bus");
        Serial.println("  RAW         - Show raw sensor data");
        Serial.println("  CALIBRATE   - Start accelerometer calibration");
        Serial.println("  ARM         - Arm motors");
        Serial.println("  DISARM      - Disarm motors");
        Serial.println("  MOTOR x x x x - Set motor values (1000-2000)");
        Serial.println("  SET_PID rate_roll kp ki kd - Set PID");
        Serial.println("  GET_PID     - Show all PID values");
        Serial.println("  RESET       - Reset to defaults");
        Serial.println("  LOG 0/1     - Enable/disable binary log");
        Serial.println("  SAVE        - Save params to EEPROM");
        Serial.println("  LOAD        - Load params from EEPROM");
        Serial.println("  WP x y      - Set waypoint");
        Serial.println("  MODE x      - Set flight mode (0-3)");
        Serial.println("  BETA x      - Set Madgwick beta");
        Serial.println("  LEARN       - Learn RC endpoints");
    }
    else if (strncmp(upper, "STATUS", 6) == 0) {
        Serial.println("=== System Status ===");
        Serial.printf("Armed: %s\n", motors.armed ? "YES" : "NO");
        Serial.printf("Flight Mode: %d\n", fd.flight_mode);
        Serial.printf("RC Valid: %s\n", rc.valid ? "YES" : "NO");
        Serial.printf("Failsafe: %s\n", fd.failsafe ? "YES" : "NO");
        Serial.printf("Battery: %.2fV (%d%%)\n", fd.battery_voltage, fd.battery_percent);
        Serial.printf("Roll: %.2f° Pitch: %.2f° Yaw: %.2f°\n",
                       fd.roll * RAD_TO_DEG, fd.pitch * RAD_TO_DEG, fd.yaw * RAD_TO_DEG);
        Serial.printf("Accel: %.3f %.3f %.3f G\n", fd.accel.x, fd.accel.y, fd.accel.z);
        Serial.printf("Gyro: %.3f %.3f %.3f °/s\n",
                       fd.gyro.x * RAD_TO_DEG, fd.gyro.y * RAD_TO_DEG, fd.gyro.z * RAD_TO_DEG);
        Serial.printf("Baro Alt: %.2f m\n", fd.baro_alt);
        Serial.printf("Motors: %d %d %d %d\n", motors.m1, motors.m2, motors.m3, motors.m4);
        Serial.printf("RC Channels: %d %d %d %d %d %d %d %d\n",
                       rc.channels[0], rc.channels[1], rc.channels[2], rc.channels[3],
                       rc.channels[4], rc.channels[5], rc.channels[6], rc.channels[7]);
        Serial.printf("MPU6050: %s (0x%02X)\n", mpu_found ? "OK" : "NOT FOUND", mpu_addr);
        Serial.printf("BMP280: %s\n", bmp_found ? "OK" : "NOT FOUND");
        Serial.printf("WiFi: %s\n", wifi_connected ? "CONNECTED" : "AP MODE");
    }
    else if (strncmp(upper, "I2C_SCAN", 8) == 0) {
        Serial.println("=== I2C Scan ===");
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                Serial.printf("  Found device at 0x%02X\n", addr);
            }
        }
        Serial.println("Scan complete");
    }
    else if (strncmp(upper, "RAW", 3) == 0) {
        Serial.println("=== Raw Sensor Data ===");
        Serial.printf("Accel (G): %.4f %.4f %.4f\n", fd.accel.x, fd.accel.y, fd.accel.z);
        Serial.printf("Gyro (rad/s): %.6f %.6f %.6f\n", fd.gyro.x, fd.gyro.y, fd.gyro.z);
        Serial.printf("Mag: %.4f %.4f %.4f\n", fd.mag.x, fd.mag.y, fd.mag.z);
        Serial.printf("Baro: %.2f hPa, %.2f°C, %.2f m\n", bmp_pressure, bmp_temperature, bmp_altitude);
        Serial.printf("Battery ADC: %d\n", analogRead(PIN_BATTERY));
    }
    else if (strncmp(upper, "CALIBRATE", 9) == 0) {
        Serial.println("=== Accelerometer Calibration ===");
        Serial.println("Place on level surface. Collecting samples...");
        calibration_mode = true;
        cal_sample_count = 0;
        accel_cal_sum[0] = accel_cal_sum[1] = accel_cal_sum[2] = 0;
        gyro_cal_sum[0] = gyro_cal_sum[1] = gyro_cal_sum[2] = 0;

        for (uint32_t i = 0; i < 1000; i++) {
            Vector3f a, g;
            mpu6050ReadAccel(&a);
            mpu6050ReadGyro(&g);
            accel_cal_sum[0] += a.x;
            accel_cal_sum[1] += a.y;
            accel_cal_sum[2] += a.z;
            gyro_cal_sum[0] += g.x + eeprom_data.gyro_offset.x;
            gyro_cal_sum[1] += g.y + eeprom_data.gyro_offset.y;
            gyro_cal_sum[2] += g.z + eeprom_data.gyro_offset.z;
            cal_sample_count++;
            delay(2);
        }

        eeprom_data.accel_offset.x = -(accel_cal_sum[0] / cal_sample_count);
        eeprom_data.accel_offset.y = -(accel_cal_sum[1] / cal_sample_count);
        eeprom_data.accel_offset.z = -(accel_cal_sum[2] / cal_sample_count - 1.0f);
        eeprom_data.accel_scale = {1, 1, 1};

        eeprom_data.gyro_offset.x = gyro_cal_sum[0] / cal_sample_count;
        eeprom_data.gyro_offset.y = gyro_cal_sum[1] / cal_sample_count;
        eeprom_data.gyro_offset.z = gyro_cal_sum[2] / cal_sample_count;

        Serial.printf("Accel offsets: %.4f %.4f %.4f\n",
                       eeprom_data.accel_offset.x, eeprom_data.accel_offset.y, eeprom_data.accel_offset.z);
        Serial.printf("Gyro offsets: %.6f %.6f %.6f\n",
                       eeprom_data.gyro_offset.x, eeprom_data.gyro_offset.y, eeprom_data.gyro_offset.z);
        Serial.println("Calibration complete. Use SAVE to store.");
        calibration_mode = false;
    }
    else if (strncmp(upper, "ARM", 3) == 0) {
        if (rc.channels[2] < ARM_THROTTLE_MAX) {
            motors.armed = true;
            fd.armed = true;
            Serial.println("Motors ARMED");
        } else {
            Serial.println("Cannot arm: throttle not low");
        }
    }
    else if (strncmp(upper, "DISARM", 6) == 0) {
        motors.armed = false;
        fd.armed = false;
        motorWriteAll(MOTOR_MINPWM, MOTOR_MINPWM, MOTOR_MINPWM, MOTOR_MINPWM);
        Serial.println("Motors DISARMED");
    }
    else if (strncmp(upper, "MOTOR", 5) == 0) {
        int m1, m2, m3, m4;
        if (sscanf(cmd + 6, "%d %d %d %d", &m1, &m2, &m3, &m4) == 4) {
            motors.test_mode = true;
            motors.armed = true;
            motors.test_values[0] = constrain(m1, 1000, 2000);
            motors.test_values[1] = constrain(m2, 1000, 2000);
            motors.test_values[2] = constrain(m3, 1000, 2000);
            motors.test_values[3] = constrain(m4, 1000, 2000);
            motorWriteAll(motors.test_values[0], motors.test_values[1],
                          motors.test_values[2], motors.test_values[3]);
            Serial.printf("Motors set: %d %d %d %d\n", m1, m2, m3, m4);
        } else {
            motors.test_mode = false;
            motorWriteAll(MOTOR_MINPWM, MOTOR_MINPWM, MOTOR_MINPWM, MOTOR_MINPWM);
            Serial.println("Motor test mode OFF");
        }
    }
    else if (strncmp(upper, "SET_PID", 7) == 0) {
        char target[32];
        float kp, ki, kd;
        if (sscanf(cmd + 8, "%s %f %f %f", target, &kp, &ki, &kd) == 3) {
            PIDController *pid = NULL;
            if (strcmp(target, "rate_roll") == 0) pid = &pid_rate_roll;
            else if (strcmp(target, "rate_pitch") == 0) pid = &pid_rate_pitch;
            else if (strcmp(target, "rate_yaw") == 0) pid = &pid_rate_yaw;
            else if (strcmp(target, "angle_roll") == 0) pid = &pid_angle_roll;
            else if (strcmp(target, "angle_pitch") == 0) pid = &pid_angle_pitch;

            if (pid) {
                pid->kp = kp;
                pid->ki = ki;
                pid->kd = kd;
                Serial.printf("PID %s set: kp=%.3f ki=%.3f kd=%.3f\n", target, kp, ki, kd);
            } else {
                Serial.println("Unknown PID target. Use: rate_roll, rate_pitch, rate_yaw, angle_roll, angle_pitch");
            }
        } else {
            Serial.println("Usage: SET_PID <target> <kp> <ki> <kd>");
        }
    }
    else if (strncmp(upper, "GET_PID", 7) == 0) {
        Serial.println("=== PID Controllers ===");
        Serial.printf("Rate Roll:  kp=%.4f ki=%.4f kd=%.4f\n", pid_rate_roll.kp, pid_rate_roll.ki, pid_rate_roll.kd);
        Serial.printf("Rate Pitch: kp=%.4f ki=%.4f kd=%.4f\n", pid_rate_pitch.kp, pid_rate_pitch.ki, pid_rate_pitch.kd);
        Serial.printf("Rate Yaw:   kp=%.4f ki=%.4f kd=%.4f\n", pid_rate_yaw.kp, pid_rate_yaw.ki, pid_rate_yaw.kd);
        Serial.printf("Angle Roll: kp=%.4f ki=%.4f kd=%.4f\n", pid_angle_roll.kp, pid_angle_roll.ki, pid_angle_roll.kd);
        Serial.printf("Angle Pitch:kp=%.4f ki=%.4f kd=%.4f\n", pid_angle_pitch.kp, pid_angle_pitch.ki, pid_angle_pitch.kd);
    }
    else if (strncmp(upper, "RESET", 5) == 0) {
        eepromLoadDefaults();
        eepromSave();
        Serial.println("Reset to defaults and saved");
    }
    else if (strncmp(upper, "LOG", 3) == 0) {
        int enable;
        if (sscanf(cmd + 4, "%d", &enable) == 1) {
            log_enable = (enable != 0);
            Serial.printf("Binary log %s\n", log_enable ? "ENABLED" : "DISABLED");
        }
    }
    else if (strncmp(upper, "SAVE", 4) == 0) {
        // Copy runtime PID to eeprom_data
        eeprom_data.pid_rate_roll = pid_rate_roll;
        eeprom_data.pid_rate_pitch = pid_rate_pitch;
        eeprom_data.pid_rate_yaw = pid_rate_yaw;
        eeprom_data.pid_angle_roll = pid_angle_roll;
        eeprom_data.pid_angle_pitch = pid_angle_pitch;
        eepromSave();
        Serial.println("Parameters saved to EEPROM");
    }
    else if (strncmp(upper, "LOAD", 4) == 0) {
        eepromLoad();
        Serial.println("Parameters loaded from EEPROM");
    }
    else if (strncmp(upper, "LEARN", 5) == 0) {
        Serial.println("=== RC Endpoint Learning ===");
        Serial.println("Move each channel to min/max positions...");
        uint16_t rc_min_local[RC_CHANNELS];
        uint16_t rc_max_local[RC_CHANNELS];
        for (uint8_t i = 0; i < RC_CHANNELS; i++) {
            rc_min_local[i] = 65535;
            rc_max_local[i] = 0;
        }

        unsigned long start = millis();
        while (millis() - start < 10000) {  // 10 seconds
            rcUpdate();
            for (uint8_t i = 0; i < RC_CHANNELS; i++) {
                if (rc.raw[i] < rc_min_local[i]) rc_min_local[i] = rc.raw[i];
                if (rc.raw[i] > rc_max_local[i]) rc_max_local[i] = rc.raw[i];
            }
            delay(10);
        }

        for (uint8_t i = 0; i < RC_CHANNELS; i++) {
            eeprom_data.rc_min[i] = rc_min_local[i];
            eeprom_data.rc_max[i] = rc_max_local[i];
            Serial.printf("CH%d: min=%d max=%d\n", i + 1, rc_min_local[i], rc_max_local[i]);
        }
        Serial.println("Learning complete. Use SAVE to store.");
    }
    else if (strncmp(upper, "BETA", 4) == 0) {
        float beta;
        if (sscanf(cmd + 5, "%f", &beta) == 1) {
            madgwick_beta = beta;
            Serial.printf("Madgwick beta set to %.4f\n", beta);
        }
    }
    else if (strlen(cmd) > 0) {
        Serial.printf("Unknown command: %s (type HELP for list)\n", cmd);
    }
}

void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serial_cmd_len > 0) {
                serial_cmd_buf[serial_cmd_len] = 0;
                handleSerialCommand(serial_cmd_buf);
                serial_cmd_len = 0;
            }
        } else if (serial_cmd_len < sizeof(serial_cmd_buf) - 1) {
            serial_cmd_buf[serial_cmd_len++] = c;
        }
    }
}

// ============================================================================
// BINARY LOG RECORDING
// ============================================================================

void binaryLogWrite() {
    if (!log_enable) return;

    // Header
    Serial.write(0xA3);
    Serial.write(0x95);

    // Timestamp (4 bytes)
    uint32_t ts = millis();
    Serial.write((uint8_t*)&ts, 4);

    // IMU data: accel (6) + gyro (6) + mag (6) = 18 bytes
    int16_t ax = (int16_t)(fd.accel.x * 1000);
    int16_t ay = (int16_t)(fd.accel.y * 1000);
    int16_t az = (int16_t)(fd.accel.z * 1000);
    int16_t gx = (int16_t)(fd.gyro.x * 1000);
    int16_t gy = (int16_t)(fd.gyro.y * 1000);
    int16_t gz = (int16_t)(fd.gyro.z * 1000);
    int16_t mx = (int16_t)(fd.mag.x * 1000);
    int16_t my = (int16_t)(fd.mag.y * 1000);
    int16_t mz = (int16_t)(fd.mag.z * 1000);
    Serial.write((uint8_t*)&ax, 2);
    Serial.write((uint8_t*)&ay, 2);
    Serial.write((uint8_t*)&az, 2);
    Serial.write((uint8_t*)&gx, 2);
    Serial.write((uint8_t*)&gy, 2);
    Serial.write((uint8_t*)&gz, 2);
    Serial.write((uint8_t*)&mx, 2);
    Serial.write((uint8_t*)&my, 2);
    Serial.write((uint8_t*)&mz, 2);

    // Attitude: roll, pitch, yaw (12 bytes)
    float r = fd.roll;
    float p = fd.pitch;
    float y = fd.yaw;
    Serial.write((uint8_t*)&r, 4);
    Serial.write((uint8_t*)&p, 4);
    Serial.write((uint8_t*)&y, 4);

    // RC channels: 8 * 2 = 16 bytes
    for (uint8_t i = 0; i < 8; i++) {
        uint16_t ch = rc.channels[i];
        Serial.write((uint8_t*)&ch, 2);
    }

    // Motor outputs: 4 * 2 = 8 bytes
    uint16_t m[4] = {motors.m1, motors.m2, motors.m3, motors.m4};
    for (uint8_t i = 0; i < 4; i++) {
        Serial.write((uint8_t*)&m[i], 2);
    }

    // Battery: voltage(2) + current(2) = 4 bytes
    int16_t bv = (int16_t)(fd.battery_voltage * 100);
    int16_t bc = (int16_t)(fd.battery_current * 100);
    Serial.write((uint8_t*)&bv, 2);
    Serial.write((uint8_t*)&bc, 2);

    // Checksum
    uint8_t checksum = 0xA3 ^ 0x95;
    uint8_t* ts_bytes = (uint8_t*)&ts;
    for (uint8_t i = 0; i < 4; i++) checksum ^= ts_bytes[i];
    Serial.write(checksum);
}

// ============================================================================
// WIFI AP + WEB INTERFACE
// ============================================================================

const char WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>ESP8266 FC</title>
<style>
body{font-family:monospace;background:#1a1a2e;color:#0f0;margin:20px}
h1{color:#0ff}
.card{background:#16213e;border:1px solid #0f3;padding:10px;margin:5px;display:inline-block;min-width:200px}
.val{font-size:1.2em;color:#0ff}
input[type=range]{width:200px}
button{background:#0f3;color:#000;border:none;padding:5px 15px;cursor:pointer;margin:2px}
button:hover{background:#0ff}
#log{background:#000;padding:5px;max-height:200px;overflow-y:auto}
</style>
</head>
<body>
<h1>ESP8266 Flight Controller</h1>
<div id="status">
<div class="card"><b>Armed:</b> <span id="armed" class="val">NO</span></div>
<div class="card"><b>Mode:</b> <span id="mode" class="val">0</span></div>
<div class="card"><b>Battery:</b> <span id="batt" class="val">0.0V</span></div>
<div class="card"><b>Failsafe:</b> <span id="fs" class="val">OFF</span></div>
</div>
<div id="sensors">
<div class="card"><b>Attitude</b><br>Roll: <span id="roll" class="val">0</span>°<br>Pitch: <span id="pitch" class="val">0</span>°<br>Yaw: <span id="yaw" class="val">0</span>°</div>
<div class="card"><b>Accel (G)</b><br>X: <span id="ax" class="val">0</span><br>Y: <span id="ay" class="val">0</span><br>Z: <span id="az" class="val">0</span></div>
<div class="card"><b>Gyro (°/s)</b><br>X: <span id="gx" class="val">0</span><br>Y: <span id="gy" class="val">0</span><br>Z: <span id="gz" class="val">0</span></div>
<div class="card"><b>Baro</b><br>Alt: <span id="alt" class="val">0</span>m<br>Temp: <span id="temp" class="val">0</span>°C</div>
</div>
<h2>RC Channels</h2>
<div id="rc">
<div class="card">CH1: <span id="ch1" class="val">1500</span></div>
<div class="card">CH2: <span id="ch2" class="val">1500</span></div>
<div class="card">CH3: <span id="ch3" class="val">1000</span></div>
<div class="card">CH4: <span id="ch4" class="val">1500</span></div>
<div class="card">CH5: <span id="ch5" class="val">1000</span></div>
<div class="card">CH6: <span id="ch6" class="val">1500</span></div>
<div class="card">CH7: <span id="ch7" class="val">1500</span></div>
<div class="card">CH8: <span id="ch8" class="val">1500</span></div>
</div>
<h2>Motors</h2>
<div>
<div class="card">M1: <span id="m1" class="val">1000</span></div>
<div class="card">M2: <span id="m2" class="val">1000</span></div>
<div class="card">M3: <span id="m3" class="val">1000</span></div>
<div class="card">M4: <span id="m4" class="val">1000</span></div>
</div>
<h2>Commands</h2>
<button onclick="cmd('ARM')">ARM</button>
<button onclick="cmd('DISARM')">DISARM</button>
<button onclick="cmd('STATUS')">STATUS</button>
<button onclick="cmd('CALIBRATE')">CALIBRATE</button>
<button onclick="cmd('SAVE')">SAVE</button>
<button onclick="cmd('LOAD')">LOAD</button>
<button onclick="cmd('RESET')">RESET</button>
<h2>PID Tuning</h2>
<div class="card">
Rate Roll: kp<input type="range" min="0" max="5" step="0.01" id="rr_kp" onchange="setpid('rate_roll',this.id)">
 ki<input type="range" min="0" max="2" step="0.01" id="rr_ki" onchange="setpid('rate_roll',this.id)">
 kd<input type="range" min="0" max="0.5" step="0.001" id="rr_kd" onchange="setpid('rate_roll',this.id)">
</div>
<h2>Log</h2>
<div id="log"></div>
<script>
function $(id){return document.getElementById(id)}
function cmd(c){fetch('/cmd?c='+c).then(r=>r.text()).then(t=>{addLog('CMD: '+c);addLog(t)})}
function setpid(target,el){var v=$(el).value;fetch('/set?pid='+target+'&val='+v).then(r=>r.text()).then(t=>addLog(t))}
function addLog(t){var l=$('log');l.innerHTML+=t+'<br>';l.scrollTop=l.scrollHeight}
function update(){
fetch('/data').then(r=>r.json()).then(d=>{
$('armed').textContent=d.armed?'YES':'NO';
$('mode').textContent=['STABILIZE','ACRO','ALT_HOLD','RTL'][d.mode]||d.mode;
$('batt').textContent=d.voltage.toFixed(2)+'V ('+d.batt_pct+'%)';
$('fs').textContent=d.failsafe?'ON':'OFF';
$('roll').textContent=(d.roll*57.3).toFixed(1);
$('pitch').textContent=(d.pitch*57.3).toFixed(1);
$('yaw').textContent=(d.yaw*57.3).toFixed(1);
$('ax').textContent=d.ax.toFixed(3);
$('ay').textContent=d.ay.toFixed(3);
$('az').textContent=d.az.toFixed(3);
$('gx').textContent=(d.gx*57.3).toFixed(1);
$('gy').textContent=(d.gy*57.3).toFixed(1);
$('gz').textContent=(d.gz*57.3).toFixed(1);
$('alt').textContent=d.alt.toFixed(1);
$('temp').textContent=d.temp.toFixed(1);
for(var i=0;i<8;i++)$('ch'+(i+1)).textContent=d.rc[i];
$('m1').textContent=d.m1;$('m2').textContent=d.m2;$('m3').textContent=d.m3;$('m4').textContent=d.m4;
}).catch(e=>addLog('Error: '+e))}
setInterval(update,200);
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
    server.send_P(200, "text/html", WEB_PAGE);
}

void handleData() {
    String json = "{";
    json += "\"armed\":" + String(fd.armed ? "true" : "false") + ",";
    json += "\"mode\":" + String(fd.flight_mode) + ",";
    json += "\"failsafe\":" + String(fd.failsafe ? "true" : "false") + ",";
    json += "\"voltage\":" + String(fd.battery_voltage, 2) + ",";
    json += "\"batt_pct\":" + String(fd.battery_percent) + ",";
    json += "\"roll\":" + String(fd.roll, 6) + ",";
    json += "\"pitch\":" + String(fd.pitch, 6) + ",";
    json += "\"yaw\":" + String(fd.yaw, 6) + ",";
    json += "\"ax\":" + String(fd.accel.x, 4) + ",";
    json += "\"ay\":" + String(fd.accel.y, 4) + ",";
    json += "\"az\":" + String(fd.accel.z, 4) + ",";
    json += "\"gx\":" + String(fd.gyro.x, 6) + ",";
    json += "\"gy\":" + String(fd.gyro.y, 6) + ",";
    json += "\"gz\":" + String(fd.gyro.z, 6) + ",";
    json += "\"alt\":" + String(fd.baro_alt, 2) + ",";
    json += "\"temp\":" + String(fd.baro_temp, 1) + ",";
    json += "\"m1\":" + String(motors.m1) + ",";
    json += "\"m2\":" + String(motors.m2) + ",";
    json += "\"m3\":" + String(motors.m3) + ",";
    json += "\"m4\":" + String(motors.m4) + ",";
    json += "\"rc\":[";
    for (uint8_t i = 0; i < 8; i++) {
        json += String(rc.channels[i]);
        if (i < 7) json += ",";
    }
    json += "]";
    json += "}";
    server.send(200, "application/json", json);
}

void handleCmd() {
    if (server.hasArg("c")) {
        String cmd = server.arg("c");
        handleSerialCommand((char*)cmd.c_str());
        server.send(200, "text/plain", "OK: " + cmd);
    } else {
        server.send(400, "text/plain", "Missing parameter c");
    }
}

void handleParams() {
    String json = "{";
    json += "\"rate_roll\":{\"kp\":" + String(pid_rate_roll.kp, 4) + ",\"ki\":" + String(pid_rate_roll.ki, 4) + ",\"kd\":" + String(pid_rate_roll.kd, 4) + "},";
    json += "\"rate_pitch\":{\"kp\":" + String(pid_rate_pitch.kp, 4) + ",\"ki\":" + String(pid_rate_pitch.ki, 4) + ",\"kd\":" + String(pid_rate_pitch.kd, 4) + "},";
    json += "\"rate_yaw\":{\"kp\":" + String(pid_rate_yaw.kp, 4) + ",\"ki\":" + String(pid_rate_yaw.ki, 4) + ",\"kd\":" + String(pid_rate_yaw.kd, 4) + "},";
    json += "\"angle_roll\":{\"kp\":" + String(pid_angle_roll.kp, 4) + ",\"ki\":" + String(pid_angle_roll.ki, 4) + ",\"kd\":" + String(pid_angle_roll.kd, 4) + "},";
    json += "\"angle_pitch\":{\"kp\":" + String(pid_angle_pitch.kp, 4) + ",\"ki\":" + String(pid_angle_pitch.ki, 4) + ",\"kd\":" + String(pid_angle_pitch.kd, 4) + "}";
    json += "}";
    server.send(200, "application/json", json);
}

void handleSet() {
    if (server.hasArg("pid") && server.hasArg("val")) {
        String pid_name = server.arg("pid");
        float val = server.arg("val").toFloat();

        PIDController *pid = NULL;
        if (pid_name == "rate_roll") pid = &pid_rate_roll;
        else if (pid_name == "rate_pitch") pid = &pid_rate_pitch;
        else if (pid_name == "rate_yaw") pid = &pid_rate_yaw;
        else if (pid_name == "angle_roll") pid = &pid_angle_roll;
        else if (pid_name == "angle_pitch") pid = &pid_angle_pitch;

        if (pid) {
            // Simple: set kp
            pid->kp = val;
            server.send(200, "text/plain", "OK: " + pid_name + " kp=" + String(val, 4));
        } else {
            server.send(400, "text/plain", "Unknown PID: " + pid_name);
        }
    } else {
        server.send(400, "text/plain", "Missing pid or val");
    }
}

void wifiInit() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_pass);
    Serial.printf("WiFi AP started: %s\n", ap_ssid);
    Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());

    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/cmd", handleCmd);
    server.on("/params", handleParams);
    server.on("/set", handleSet);
    server.begin();
    Serial.println("Web server started");
    wifi_connected = true;
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== ESP8266 Flight Controller ===");
    Serial.println("Initializing...");

    // LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // I2C
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    Serial.println("[I2C] Initialized");

    // EEPROM
    EEPROM.begin(EEPROM_SIZE);
    Serial.println("[EEPROM] Initialized");

    // Load parameters
    eepromLoad();

    // Init MPU6050
    if (mpu6050Init()) {
        Serial.printf("[MPU6050] Found at 0x%02X\n", mpu_addr);
    } else {
        Serial.println("[MPU6050] NOT FOUND!");
    }

    // Init BMP280
    if (bmp280Init()) {
        Serial.println("[BMP280] Found");
    } else {
        Serial.println("[BMP280] NOT FOUND (optional)");
    }

    // Init motors
    motorInit();
    Serial.println("[Motors] Initialized");

    // Init RC
    rcInit();
    Serial.println("[RC] PPM initialized on D0");

    // Init WiFi
    wifiInit();

    // Init quaternion
    quaternionInit(&fd.attitude);

    // Init PIDs from loaded params
    pid_rate_roll = eeprom_data.pid_rate_roll;
    pid_rate_pitch = eeprom_data.pid_rate_pitch;
    pid_rate_yaw = eeprom_data.pid_rate_yaw;
    pid_angle_roll = eeprom_data.pid_angle_roll;
    pid_angle_pitch = eeprom_data.pid_angle_pitch;

    Serial.println("[System] Ready!");
    Serial.println("Type HELP for commands");

    last_loop_time = micros();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    unsigned long now = micros();

    // Fixed frequency loop
    if (now - last_loop_time < LOOP_PERIOD_US) return;
    float dt = (now - last_loop_time) / 1000000.0f;
    last_loop_time = now;
    loop_count++;

    // Handle web requests
    server.handleClient();

    // Handle serial commands
    handleSerial();

    // Read sensors
    readSensors();

    // Sensor fusion
    sensorFusion();

    // Update RC
    rcUpdate();

    // Flight mode
    flightModeUpdate();

    // Safety
    safetyUpdate();

    // Control
    if (motors.armed && !fd.failsafe && !motors.test_mode && !crash_detected) {
        switch (fd.flight_mode) {
            case MODE_STABILIZE: stabilizeControl(); break;
            case MODE_ACRO:      acroControl();      break;
            case MODE_ALTHOLD:   altholdControl();    break;
            case MODE_RTL:       rtlControl();        break;
            default:             stabilizeControl();  break;
        }
    } else if (!motors.armed || fd.failsafe || crash_detected) {
        // Motors off
        motorWriteAll(MOTOR_MINPWM, MOTOR_MINPWM, MOTOR_MINPWM, MOTOR_MINPWM);
        pidReset(&pid_rate_roll);
        pidReset(&pid_rate_pitch);
        pidReset(&pid_rate_yaw);
        pidReset(&pid_angle_roll);
        pidReset(&pid_angle_pitch);
    }

    // LED
    ledUpdate();

    // Telemetry at 50Hz
    if (now - last_telemetry_time >= (1000000 / TELEMETRY_FREQ_HZ)) {
        last_telemetry_time = now;
        sendHeartbeat();
        sendAttitude();
        sendRawIMU();
        sendBattery();
        sendRCChannels();
        sendServoOutput();
        sendSysStatus();
        sendGPSRaw();
    }

    // Binary log
    binaryLogWrite();

    // Auto-save home position
    if (motors.armed && !home_set) {
        home_set = true;
        home_lat = 0;
        home_lon = 0;
    }

    // Reset crash detection after 1 second
    static unsigned long crash_time = 0;
    if (crash_detected && crash_time == 0) {
        crash_time = millis();
    }
    if (crash_detected && millis() - crash_time > 1000) {
        crash_detected = false;
        crash_time = 0;
    }
}
