#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "dshot.pio.h"
#include "alt.pio.h"
#include <ctype.h>
#include <string.h>

//defy dla uart
#define UART_ID uart0
#define BAUD_RATE 9600      // musi zgadzać się z RPi Zero
#define UART_RX_PIN 1       // RX Pico podłączony do TX RPi
#define UART_TX_PIN 0 
#define MPU_ADDR 0x68
#define BUF_SIZE 64
#define TRIG_PIN 7
#define ECHO_PIN 8


char rx_buffer[BUF_SIZE];
int rx_pos = 0;
bool new_data = false;

// gdzie sie wykonuje
PIO pio = pio0;
uint sm1 = 0;
uint sm2 = 1;
uint sm3 = 2;
uint sm4 = 3;

//wartości dla stanu statycznego
uint16_t Preval1;
uint16_t Preval2;
uint16_t Preval3;
uint16_t Preval4;

// wartosci od 0 do 2047
volatile uint16_t val1, val2, val3, val4;
// Ramki
volatile uint32_t eng1, eng2, eng3, eng4 = 0;

volatile float target_alt = 30;
volatile float base_roll, base_pitch = 0;
int hover_throttle = 900;
volatile int base_motor = 200;
volatile int max_base_motor = 1150;
volatile int min_base_motor = 100;
volatile int min_motor_speed = 0;
volatile int max_motor_speed = 1200;
volatile float roll_zero = 0.0f;
volatile float pitch_zero = 0.0f;


uint16_t calculate_frame(uint16_t frame, bool telemetry) {
      uint16_t a = frame << 1;
      uint16_t crc = (a ^ (a >> 4) ^ (a >> 8)) & 0x0F;
      a = (a << 4) | crc;
      return a;
    }

// IMU + DMA + przerwanie
//presety 
int xp = 0;
int yp = 0;
int zp = 0;
// nowe dane z czujnika
volatile uint8_t accel_buf[6];

void imu_dma_init(){
  int chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, DREQ_PIO1_RX0);

    dma_channel_configure(
        chan,          // Channel to be configured
        &c,            // The configuration we just created
        NULL,           // The initial write address
        NULL,           // The initial read address
        0xFFFFFFFF, // Number of transfers; in this case each is 1 byte.
        true           // Start immediately.
    );
}

// MPU6500
#define I2C_PORT i2c0

void init_i2c() {
    // Start I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    sleep_ms(200);
    
    uint8_t wake_cmd[2] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, 0x68, wake_cmd, 2, false);
    sleep_ms(100);

    uint8_t dlpf_gyro[2] = {0x1A, 0x02};
    i2c_write_blocking(I2C_PORT, 0x68, dlpf_gyro, 2, false);

    uint8_t dlpf_accel[2] = {0x1D, 0x02};
    i2c_write_blocking(I2C_PORT, 0x68, dlpf_accel, 2, false);

    sleep_ms(50);
}


typedef struct { float x, y, z; } vec3;
typedef struct { uint16_t a, b, c, d; } engines;

static inline vec3 convert_accel(int16_t ax, int16_t ay, int16_t az)
{
    // 16384 LSB / 1g → g = 9.80665
    const float scale = 9.80665f / 16384.0f;

    return (vec3){
        ax * scale,
        ay * scale,
        az * scale
    };
}

static inline vec3 convert_gyro(int16_t gx, int16_t gy, int16_t gz)
{
    const float deg2rad = 0.01745329252f; // π/180
    const float scale = 0.0001331;

    return (vec3){
        gx * scale,
        gy * scale,
        gz * scale
    };
}


// Funkcja sterowania PD + mixer
// Układ silników:
//        front
//         ^
//         |
//       B   D
//         X
//       A   C
vec3 gyro_offset = {0};
vec3 accel_offset = {0};

void calibrate_imu_offsets() {
    const int samples = 500;
    vec3 gyro_sum = {0};
    vec3 accel_sum = {0};

    uint8_t buf[14];

    printf("Kalibracja IMU... NIE RUSZAJ DRONA\n");
    sleep_ms(2000);

    for (int i = 0; i < samples; i++) {
        uint8_t reg = 0x3B;
        i2c_write_blocking(I2C_PORT, 0x68, &reg, 1, true);
        int ret = i2c_read_blocking(I2C_PORT, 0x68, buf, 14, false);
        if (ret != 14) continue;

        int16_t ax = (buf[0] << 8) | buf[1];
        int16_t ay = (buf[2] << 8) | buf[3];
        int16_t az = (buf[4] << 8) | buf[5];

        int16_t gx = (buf[8] << 8) | buf[9];
        int16_t gy = (buf[10] << 8) | buf[11];
        int16_t gz = (buf[12] << 8) | buf[13];

        vec3 accel = convert_accel(ax, ay, az);
        vec3 gyro  = convert_gyro(gx, gy, gz);

        gyro_sum.x  += gyro.x;
        gyro_sum.y  += gyro.y;
        gyro_sum.z  += gyro.z;

        accel_sum.x += accel.x;
        accel_sum.y += accel.y;
        // Z pomijamy – grawitacja

        sleep_ms(2);
    }

    gyro_offset.x  = gyro_sum.x  / samples;
    gyro_offset.y  = gyro_sum.y  / samples;
    gyro_offset.z  = gyro_sum.z  / samples;

    accel_offset.x = accel_sum.x / samples;
    accel_offset.y = accel_sum.y / samples;
    accel_offset.z = 0.0f;

    printf("Kalibracja zakonczona\n");
}


typedef struct {
    float roll;
    float pitch;
    float bgx;
    float bgy;

    float P[4][4];
} ekf_rp_t;

ekf_rp_t ekf;

void ekf_rp_init(ekf_rp_t *e) {
    e->roll = 0.0f;
    e->pitch = 0.0f;
    e->bgx = 0.0f;
    e->bgy = 0.0f;

    memset(e->P, 0, sizeof(e->P));
    e->P[0][0] = 0.1f;
    e->P[1][1] = 0.1f;
    e->P[2][2] = 0.01f;
    e->P[3][3] = 0.01f;
}


void ekf_rp_predict(
    ekf_rp_t *e,
    float gx, float gy,
    float dt
) {
    // state prediction
    e->roll  += (gx - e->bgx) * dt;
    e->pitch += (gy - e->bgy) * dt;

    // process noise
    const float q_angle = 0.002f;
    const float q_bias  = 0.00001f;

    e->P[0][0] += q_angle * dt;
    e->P[1][1] += q_angle * dt;
    e->P[2][2] += q_bias * dt;
    e->P[3][3] += q_bias * dt;
}


void ekf_rp_update_acc(
    ekf_rp_t *e,
    float ax, float ay, float az
) {
    // --- normalizacja ---
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.1f) return; // zabezpieczenie numeryczne

    ax /= norm;
    ay /= norm;
    az /= norm;

    // --- miara wiarygodności acc ---
    float acc_err = fabsf(norm - 1.0f);

    // miękka waga: 1 = idealnie, 0 = kompletnie niewiarygodny
    float w = 1.0f - acc_err * 2.5f;
    if (w < 0.0f) w = 0.0f;

    // dynamiczne R (im gorszy acc, tym większe R)
    const float R_base = 0.04f;
    float R = R_base / (w*w + 0.02f);

    // --- pomiar ---
    float roll_meas  = atan2f(ay, az);
    float pitch_meas = atan2f(-ax, sqrtf(ay*ay + az*az));

    // ================= ROLL =================
    float y = roll_meas - e->roll;

    // ograniczenie innowacji (anty-impuls)
    if (y >  0.35f) y =  0.35f;
    if (y < -0.35f) y = -0.35f;

    float S = e->P[0][0] + R;
    float K = e->P[0][0] / S;

    e->roll += K * y;
    e->bgx  += 0.005f * K * y;

    e->P[0][0] *= (1.0f - K);

    // ================= PITCH =================
    y = pitch_meas - e->pitch;

    if (y >  0.35f) y =  0.35f;
    if (y < -0.35f) y = -0.35f;

    S = e->P[1][1] + R;
    K = e->P[1][1] / S;

    e->pitch += K * y;
    e->bgy   += 0.005f * K * y;

    e->P[1][1] *= (1.0f - K);
}



// static inline void ekf_get_angles(ekf_t *ekf,float *roll,float *pitch) {
//     *roll  = atan2f(2*(ekf->q0*ekf->q1 + ekf->q2*ekf->q3),
//                      1 - 2*(ekf->q1*ekf->q1 + ekf->q2*ekf->q2));
//     *pitch = -asinf(2*(ekf->q0*ekf->q2 - ekf->q3*ekf->q1));
// }

// float roll_zero = 0.0f;
// float pitch_zero = 0.0f;

// void calibrate_level_ekf(void)
// {
//     const int N = 300;
//     float r = 0, p = 0;

//     for (int i = 0; i < N; i++) {
//         float roll, pitch;
//         ekf_get_angles(&ekf, &roll, &pitch);
//         r += roll;
//         p += pitch;
//         sleep_ms(2);
//     }

//     roll_zero  = r / N;
//     pitch_zero = p / N;
// }


// ================= PD =================
typedef struct {
    float kp;
    float ki;
    float kd;

    float integrator;
    float prev_error;
    float prev_output;

    float out_limit;
    float i_limit;
    float last;
} pid_t;



// ANGLE loop
float kp_angle = 2.5f;        // rad/s per rad
float max_rate = 2.5f;        // rad/s

// RATE loop
pid_t pid_roll = {
    .kp = 8.0f,
    .ki = 0.0f,
    .kd = 0.03f,
    .integrator = 0,
    .prev_error = 0,
    .out_limit = 30.0f,
    .i_limit = 5.0f
};

pid_t pid_pitch = {
    .kp = 8.0f,
    .ki = 0.0f,
    .kd = 0.03f,
    .integrator = 0,
    .prev_error = 0,
    .out_limit = 30.0f,
    .i_limit = 5.0f
};


float roll_trim = 0.0f;
float pitch_trim = 0.0f;
float trim_gain = 0.03f;
const float trim_angle_thresh = 0.02f; 

const float angle_deadzone = 0.005f;   // ~0.3 stopnia
const float rate_deadzone  = 0.002f;
const float rate_limit  = 10.0f;

static inline float pid_rate_update(
    pid_t *pid,
    float rate_target,
    float rate_meas,
    float dt,
    float *gyro_filtered
){
    // --- filtracja sygnału z gyro ---
    *gyro_filtered += 0.15f * (rate_meas - *gyro_filtered); // LPF łagodniejszy
    //if(fabs(*gyro_filtered) < rate_deadzone) *gyro_filtered = 0.0f;

    // --- błąd ---
    float error = rate_target - *gyro_filtered;

    // --- P-term ---
    float P_term = pid->kp * error;

    // --- D-term z błędu (nie surowy gyro) ---
    float D_term = -pid->kd * (*gyro_filtered);

    // --- sumowanie PID ---
    float out = P_term + D_term;

    // --- rate limit (bezpieczne ograniczenie zmiany) ---
    float delta = out - pid->prev_output;
    if(delta > rate_limit) delta = rate_limit;
    if(delta < -rate_limit) delta = -rate_limit;
    out = pid->prev_output + delta;

    // --- ograniczenia absolutne ---
    if(out > pid->out_limit) out = pid->out_limit;
    if(out < -pid->out_limit) out = -pid->out_limit;

    pid->prev_error = error;
    pid->prev_output = out;

    return out;
}



static inline float angle_to_rate(float angle)
{
    if(fabs(angle) < angle_deadzone) return 0.0f;

    float rate = kp_angle * angle; // liniowo
    if (rate >  max_rate) rate =  max_rate;
    if (rate < -max_rate) rate = -max_rate;
    return rate;
}

static float frac_a = 0.0f;
static float frac_b = 0.0f;
static float frac_c = 0.0f;
static float frac_d = 0.0f;
float corr_limit = 25.0f;

static inline void update_trim(float roll, float pitch, float roll_rate_target, float pitch_rate_target, float dt) {
    // PROGI do aktywacji trimu
    const float rate_thresh  = 0.4f;  // rad/s, większy próg
    const float angle_thresh = 0.1f;  // rad, większy próg (~5-6 deg)

    // Roll trim
    if(fabs(roll_rate_target) < rate_thresh && fabs(roll) < angle_thresh) {
        roll_trim += trim_gain * roll * dt;

        // limity trim
        if(roll_trim > 2.0f * 0.01745f) roll_trim = 2.0f * 0.01745f;   
        if(roll_trim < -2.0f * 0.01745f) roll_trim = -2.0f * 0.01745f;
    }

    // Pitch trim
    if(fabs(pitch_rate_target) < rate_thresh && fabs(pitch) < angle_thresh) {
        pitch_trim += trim_gain * pitch * dt;

        // limity trim
        if(pitch_trim > 2.0f * 0.01745f) pitch_trim = 2.0f * 0.01745f;
        if(pitch_trim < -2.0f * 0.01745f) pitch_trim = -2.0f * 0.01745f;
    }
}

// ===== FEEDFORWARD =====
float ff_gain = 0.35f;      // 0.2 – 0.6 (zacznij od 0.35)
float ff_tau  = 0.025f;     // stała czasowa śmigła [s] (20–40 ms)

// ===== RATE PREDICTION =====
float rate_pred_gain = 1.0f; // zwykle 1.0

static inline float predict_rate(
    float rate_meas,    // gyro
    float rate_cmd,     // rate_target
    float dt
) {
    // Model 1-rzędu:  ω_dot = (ω_cmd - ω) / tau
    float rate_dot = (rate_cmd - rate_meas) / ff_tau;
    return rate_meas + rate_dot * dt;
}

static inline float rate_feedforward(
    float rate_target,
    float prev_rate_target,
    float dt
) {
    return ff_gain * (rate_target - prev_rate_target) / dt;
}

static float prev_roll_rate_target  = 0.0f;
static float prev_pitch_rate_target = 0.0f;
float gyro_filter_x = 0;
float gyro_filter_y = 0;

engines calculate_speed_pd(float roll, float pitch, vec3 gyro, float dt)
{

    // ===== TRIM NA KĄCIE =====
    roll  += roll_trim;
    pitch += pitch_trim;

    // ===== ANGLE → RATE =====
    float roll_rate_target  = angle_to_rate(-roll);
    float pitch_rate_target = angle_to_rate(-pitch);

    // ===== RZECZYWISTE RATE (znaki!) =====
    float roll_rate_meas  = gyro.x;
    float pitch_rate_meas = gyro.y;

    // ===== PREDYKCJA RATE =====
    // float roll_rate_pred =
    //     predict_rate(roll_rate_meas, roll_rate_target, dt);

    // float pitch_rate_pred =
    //     predict_rate(pitch_rate_meas, pitch_rate_target, dt);

    // ===== RATE PID (NA PREDYKCJI) =====
    
    float rc = pid_rate_update(
        &pid_roll,
        roll_rate_target,
        roll_rate_meas,
        dt,
        &gyro_filter_x
    );

    float pc = pid_rate_update(
        &pid_pitch,
        pitch_rate_target,
        pitch_rate_meas,
        dt,
        &gyro_filter_y
    );

    // ===== FEEDFORWARD =====
    // rc += rate_feedforward(
    //     roll_rate_target,
    //     prev_roll_rate_target,
    //     dt
    // );

    // pc += rate_feedforward(
    //     pitch_rate_target,
    //     prev_pitch_rate_target,
    //     dt
    // );

    prev_roll_rate_target  = roll_rate_target;
    prev_pitch_rate_target = pitch_rate_target;

    // ===== LIMITY KOREKCJI =====
    if(rc >  corr_limit) rc =  corr_limit;
    if(rc < -corr_limit) rc = -corr_limit;
    if(pc >  corr_limit) pc =  corr_limit;
    if(pc < -corr_limit) pc = -corr_limit;

    // ===== MIXER X + FRAKCJE =====
    float fa = base_motor + pc + rc + frac_a;
    float fb = base_motor - pc + rc + frac_b;
    float fc = base_motor + pc - rc + frac_c;
    float fd = base_motor - pc - rc + frac_d;

    engines e;
    e.a = (uint16_t)fa;
    e.b = (uint16_t)fb;
    e.c = (uint16_t)fc;
    e.d = (uint16_t)fd;

    frac_a = fa - e.a;
    frac_b = fb - e.b;
    frac_c = fc - e.c;
    frac_d = fd - e.d;

    // ===== LIMITY SILNIKÓW =====
    if(e.a < min_motor_speed) e.a = min_motor_speed;
    if(e.b < min_motor_speed) e.b = min_motor_speed;
    if(e.c < min_motor_speed) e.c = min_motor_speed;
    if(e.d < min_motor_speed) e.d = min_motor_speed;

    if(e.a > max_motor_speed) e.a = max_motor_speed;
    if(e.b > max_motor_speed) e.b = max_motor_speed;
    if(e.c > max_motor_speed) e.c = max_motor_speed;
    if(e.d > max_motor_speed) e.d = max_motor_speed;

    // ===== SLOW TRIM =====
    update_trim(
        roll,
        pitch,
        roll_rate_target,
        pitch_rate_target,
        dt
    );

    return e;
}


// ================= MAIN LOOP =================
void steer_by_orientation() {
    uint8_t buf[14];

    sleep_ms(3000);  // start delay
    ekf_rp_init(&ekf);
    calibrate_imu_offsets(); // stabilny offset

    // ===== BEZPIECZNY START SILNIKÓW =====
    // val1 = val2 = val3 = val4 = hover_throttle;
    // eng1 = calculate_frame(val1,0);
    // eng2 = calculate_frame(val2,0);
    // eng3 = calculate_frame(val3,0);
    // eng4 = calculate_frame(val4,0);
    // sleep_ms(500); // 1 sekunda stabilizacji

    static uint64_t last_time = 0;

    while(1){
        uint64_t now = time_us_64();

        
        float dt = (now - last_time) * 1e-6f;
        last_time = now;

        // ograniczenia dt
        if(dt < 0.001f) dt = 0.001f;
        if(dt > 0.01f) dt = 0.01f;

        // ===== ODCZYT MPU =====
        uint8_t reg=0x3B;
        i2c_write_blocking(I2C_PORT, MPU_ADDR, &reg,1,true);
        if(i2c_read_blocking(I2C_PORT, MPU_ADDR, buf,14,false)!=14) continue;

        vec3 acc = convert_accel(
            (buf[0]<<8)|buf[1],
            (buf[2]<<8)|buf[3],
            (buf[4]<<8)|buf[5]);

        vec3 gyro = convert_gyro(
            (buf[8]<<8)|buf[9],
            (buf[10]<<8)|buf[11],
            (buf[12]<<8)|buf[13]);

        // ===== OFFSET =====
        gyro.x  -= gyro_offset.x;
        gyro.y  -= gyro_offset.y;
        gyro.z  -= gyro_offset.z;

        acc.x -= accel_offset.x;
        acc.y -= accel_offset.y;
        acc.z -= accel_offset.z;

        // ===== EKF =====
        ekf_rp_predict(&ekf, gyro.x, gyro.y, dt);
        ekf_rp_update_acc(&ekf, acc.x / 9.80665f, acc.y / 9.80665f, acc.z / 9.80665f);

        float roll  = ekf.roll;
        float pitch = ekf.pitch;

        // ===== KALKULACJA SILNIKÓW =====
        engines out = calculate_speed_pd(roll - base_roll, pitch - base_pitch, gyro, dt);

        val1=out.a; val2=out.b; val3=out.c; val4=out.d;
        eng1=calculate_frame(val1,0);
        eng2=calculate_frame(val2,0);
        eng3=calculate_frame(val3,0);
        eng4=calculate_frame(val4,0);

        // ===== DEBUG =====
        printf("roll: %3.3f, pitch: %3.3f | a:%d b:%d c:%d d:%d dt:%.4f\n",
               roll, pitch, val1, val2, val3, val4, dt);
    }
}


//init dma for motors
void init_dma_eng(uint dreq, volatile void * write_addr, const volatile void * read_addr){
  int chan = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, dreq);

    dma_channel_configure(
        chan,          // Channel to be configured
        &c,            // The configuration we just created
        write_addr,           // The initial write address
        read_addr,           // The initial read address
        0xFFFFFFFF, // Number of transfers; in this case each is 1 byte.
        true           // Start immediately.
    );
}

volatile char uart_last_char = 0;

void on_uart_irq() {
    while (uart_is_readable(UART_ID)) {
        uart_last_char = uart_getc(UART_ID);
    }
}

void handle_uart_char(char c) {
    // if(c != 0){
    // printf("z uarta: %c\n", c);
    // }
    switch (c) {
        case 'w':
            base_pitch = -0.09f;
            sleep_ms(1250);
            base_pitch = 0;
            uart_last_char = 0;
            break;

        case 's':
            base_pitch = +0.09f;
            sleep_ms(1250);
            base_pitch = 0;
            uart_last_char = 0;
            break;

        case 'a':
            base_roll = -0.09f;
            sleep_ms(1250);
            base_roll = 0;
            uart_last_char = 0;
            break;

        case 'd':
            base_roll = +0.09f;
            sleep_ms(1250);
            base_roll = 0;
            uart_last_char = 0;
            break;

        case 'r':
            target_alt = 30;
            base_roll = base_pitch = 0.0f;
            uart_last_char = 0;
            //base_motor = min_base_motor;   // KLUCZOWE
            break;

        case 'l':
            target_alt = 0;
            base_roll = base_pitch = 0.0f;
            uart_last_char = 0;
            break;

        case 'x':
            base_roll = base_pitch = 0.0f;
            uart_last_char = 0;
            break;
        
        default:
            break;
    }
}



void sr04_init(){
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

bool altitude_measure_cm(float *alt_cm)
{
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    uint64_t t0 = time_us_64();
    while (!gpio_get(ECHO_PIN)) {
        if (time_us_64() - t0 > 30000) return false;
    }

    uint64_t start = time_us_64();
    while (gpio_get(ECHO_PIN)) {
        if (time_us_64() - start > 30000) return false;
    }

    uint64_t pulse = time_us_64() - start;
    *alt_cm = pulse * 0.0343f * 0.5f;

    return true;
}

bool altitude_filter(float raw_cm, float *alt_filt)
{
    static float h1 = 0, h2 = 0, h3 = 0;

    if (raw_cm < 5.0f || raw_cm > 300.0f)
        return false;

    h3 = h2;
    h2 = h1;
    h1 = raw_cm;

    // mediana z 3
    float a = h1, b = h2, c = h3;
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }

    *alt_filt = b;
    return true;
}

typedef struct {
    float kp, ki, kd;
    float integrator;
    float prev_error;
    float i_limit;
    float out_limit;
} alt_pid_t;

alt_pid_t alt_pid = {
    .kp = 2.8f,
    .ki = 0.25f,
    .kd = 0.15f,
    .i_limit = 10.0f,
    .out_limit = 50.0f
};

float altitude_pid_update(
    alt_pid_t *pid,
    float target_cm,
    float current_cm,
    float dt
){
    float err = target_cm - current_cm;

    pid->integrator += pid->ki * err * dt;
    if (pid->integrator > pid->i_limit)  pid->integrator = pid->i_limit;
    if (pid->integrator < -pid->i_limit) pid->integrator = -pid->i_limit;

    float d = (err - pid->prev_error) / dt;
    pid->prev_error = err;

    float out = pid->kp * err + pid->integrator + pid->kd * d;

    if (out > pid->out_limit)  out = pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;

    return out;
}

void steer_by_altitude_step(float target_alt_cm)
{
    static absolute_time_t next_time = {0};
    const float dt = 0.05f;   // 50 ms

    if (is_nil_time(next_time)) {
        next_time = delayed_by_ms(get_absolute_time(), 50);
        return;
    }

    if (!time_reached(next_time))
        return;

    next_time = delayed_by_ms(next_time, 50);

    float raw_alt;
    if (!altitude_measure_cm(&raw_alt))
        return;

    float alt_filt;
    if (!altitude_filter(raw_alt, &alt_filt))
        return;

    float corr = altitude_pid_update(
        &alt_pid,
        target_alt_cm,
        alt_filt,
        dt
    );

    base_motor = hover_throttle + corr;

    if (base_motor > max_base_motor) base_motor = max_base_motor;
    if (base_motor < min_base_motor) base_motor = min_base_motor;
}



// void steer_by_altitude() {
//     static absolute_time_t next_time = 0;

//     const float dt = 0.05f;   // 50 ms

//     // Inicjalizacja pierwszego wywołania
//     if (next_time == 0) {
//         next_time = delayed_by_ms(get_absolute_time(), 49);
//     }

//     // Jeśli jeszcze nie czas — wyjdź
//     if (!time_reached(next_time)) {
//         return;
//     }

//     // Ustaw kolejny termin
//     next_time = delayed_by_ms(next_time, 50);

//     // ======= Pomiar SR04 =======
//     gpio_put(TRIG_PIN, 1);
//     sleep_us(10);
//     gpio_put(TRIG_PIN, 0);

//     uint64_t t0 = time_us_64();
//     while (!gpio_get(ECHO_PIN)) {
//         if (time_us_64() - t0 > 30000) goto measurement_failed;
//     }

//     uint64_t start = time_us_64();

//     while (gpio_get(ECHO_PIN)) {
//         if (time_us_64() - start > 30000) goto measurement_failed;
//     }

//     uint64_t pulse = time_us_64() - start;
//     float distance_cm = (pulse * 0.0343f) * 0.5f;

//     // ======= PID wysokości =======
//     float err = target_alt - distance_cm;
//     float d   = (err - params.last) / dt;
//     params.last = err;

//     float alt_corr = params.kp * err + params.kd * d;

//     if (alt_corr > 10)  alt_corr = 10;
//     if (alt_corr < -10) alt_corr = -10;

//      base_motor += alt_corr;

//     if (base_motor > max_base_motor) base_motor = max_base_motor;
//     if (base_motor < min_base_motor) base_motor = min_base_motor;

//     return;

// measurement_failed:
//     params.last = 0;
// }




int main()

{
    stdio_init_all();
    init_i2c();
    sr04_init();
    calibrate_imu_offsets();
    //calibrate_level();

    uint offset = pio_add_program(pio, &dshot_program);
    dshot_program_init(pio, sm1, offset, 21, true);
    dshot_program_init(pio, sm2, offset, 20, true);
    dshot_program_init(pio, sm3, offset, 19, true);
    dshot_program_init(pio, sm4, offset, 18, true);
    
    init_dma_eng(DREQ_PIO0_TX0, &pio0->txf[sm1], &eng1);
    init_dma_eng(DREQ_PIO0_TX1, &pio0->txf[sm2], &eng2);
    init_dma_eng(DREQ_PIO0_TX2, &pio0->txf[sm3], &eng3);
    init_dma_eng(DREQ_PIO0_TX3, &pio0->txf[sm4], &eng4);

    // uart_init(UART_ID, BAUD_RATE);
    // gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    // gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // // Włącz IRQ tylko dla odbioru
    // uart_set_irq_enables(UART_ID, true, false);
    // irq_set_exclusive_handler(UART0_IRQ, on_uart_irq);
    // irq_set_enabled(UART0_IRQ, true);


    
    multicore_launch_core1(steer_by_orientation);

    sleep_ms(2000);

    while(true){
        handle_uart_char(uart_last_char);
        steer_by_altitude_step(target_alt);    

    }
    return 0;
}
