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
int hover_throttle = 880;
volatile float base_motor = 200;
volatile int max_base_motor = 1150;
volatile int min_base_motor = 100;
volatile int min_motor_speed = 0;
volatile int max_motor_speed = 1200;
volatile float roll_zero = 0.0f;
volatile float pitch_zero = 0.0f;

volatile float target_base_motor = 200.0f;     // Ustawiane przez Rdzeń 0 (wysokość)
volatile float current_internal_base = 0.0f; // Rośnie powoli (Soft Start)
const float ramp_speed = 250.0f;


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
    float roll, pitch;      // Estymowane kąty (rad)
    float bgx, bgy;        // Estymowany dryf żyroskopu (rad/s)
    float P[4][4];         // Macierz kowariancji błędu
    float Q_angle;         // Szum procesu dla kąta
    float Q_bias;          // Szum procesu dla dryfu
    float R_accel;         // Szum pomiarowy akcelerometru
} ekf_t;

ekf_t ekf;

void ekf_init(ekf_t *e) {
    e->roll = e->pitch = 0.0f;
    e->bgx = e->bgy = 0.0f;
    
    // Inicjalizacja macierzy P (pewność początkowa)
    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) e->P[i][j] = 0.0f;
        e->P[i][i] = 0.1f;
    }

    // Parametry strojenia - dostosuj w zależności od wibracji drona
    e->Q_angle = 0.001f;   // Jak bardzo ufamy modelowi matematycznemu
    e->Q_bias  = 0.00003f; // Jak szybko filtr ma adaptować się do dryfu żyra
    e->R_accel = 0.05f;    // Bazowy szum akcelerometru
}


void ekf_predict(ekf_t *e, float gx, float gy, float dt) {
    // 1. Stan - odejmujemy dryf
    float rate_x = gx - e->bgx;
    float rate_y = gy - e->bgy;

    e->roll  += rate_x * dt;
    e->pitch += rate_y * dt;

    // 2. Bezpieczna aktualizacja macierzy P
    // Dodajemy szum procesu Q tylko do przekątnej - to gwarantuje stabilność
    e->P[0][0] += e->Q_angle * dt;
    e->P[1][1] += e->Q_angle * dt;
    e->P[2][2] += e->Q_bias * dt;
    e->P[3][3] += e->Q_bias * dt;

    // Zabezpieczenie przed "rozjechaniem" się macierzy (Cap)
    for(int i=0; i<4; i++) {
        if(e->P[i][i] > 100.0f) e->P[i][i] = 100.0f;
    }
}

void ekf_update_accel(ekf_t *e, float ax, float ay, float az) {
    // 1. Sprawdzenie poprawności danych
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.82f || norm > 1.18f) return; // Węższe okno = większa precyzja

    // 2. Obliczanie kątów (surowe dane - brak opóźnienia)
    float r_meas = atan2f(ay, az);
    float p_meas = atan2f(-ax, sqrtf(ay*ay + az*az));

    // 3. Wyliczenie błędu (Innowacji)
    float y_r = r_meas - e->roll;
    float y_p = p_meas - e->pitch;

    // --- TŁUMIENIE (Smoothing) ---
    // Zamiast nagłych skoków, stosujemy funkcję, która wygładza małe drgania
    // Jeśli błąd jest mniejszy niż 0.01 rad, osłabiamy go, by uniknąć "jittera" silników
    if (fabsf(y_r) < 0.01f) y_r *= 0.5f; 
    if (fabsf(y_p) < 0.01f) y_p *= 0.5f;

    // --- AKTUALIZACJA ROLL ---
    float S_r = e->P[0][0] + e->R_accel;
    if (S_r > 0.0001f) {
        float K0 = e->P[0][0] / S_r;
        float K1 = e->P[2][0] / S_r;

        e->roll += K0 * y_r;
        // Bias aktualizujemy bardzo ostrożnie, by nie wywołać dryfu
        e->bgx  += K1 * y_r * 0.5f; 

        // Stabilizacja macierzy P
        float P00_temp = e->P[0][0];
        e->P[0][0] -= K0 * P00_temp;
        e->P[2][0] -= K1 * P00_temp;
        e->P[0][2] = e->P[2][0];
        e->P[2][2] -= K1 * e->P[0][2];
    }

    // --- AKTUALIZACJA PITCH ---
    float S_p = e->P[1][1] + e->R_accel;
    if (S_p > 0.0001f) {
        float K0 = e->P[1][1] / S_p;
        float K1 = e->P[3][1] / S_p;

        e->pitch += K0 * y_p;
        e->bgy   += K1 * y_p * 0.5f;

        float P11_temp = e->P[1][1];
        e->P[1][1] -= K0 * P11_temp;
        e->P[3][1] -= K1 * P11_temp;
        e->P[1][3] = e->P[3][1];
        e->P[3][3] -= K1 * e->P[1][3];
    }

    // Ograniczenie kowariancji (zapobiega numerycznemu "pływaniu")
    for(int i=0; i<4; i++) {
        if(e->P[i][i] < 0.000001f) e->P[i][i] = 0.000001f;
    }
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
float kp_angle = 4.0f;        // rad/s per rad
float max_rate = 3.5f;        // rad/s

// RATE loop
pid_t pid_roll = {
    .kp = 28.0f,
    .ki = 4.0f,
    .kd = 0.3f,
    .integrator = 0,
    .prev_error = 0,
    .out_limit = 180.0f,
    .i_limit = 350.0f
};

pid_t pid_pitch = {
    .kp = 28.0f,
    .ki = 4.0f,
    .kd = 0.3f,
    .integrator = 0,
    .prev_error = 0,
    .out_limit = 180.0f,
    .i_limit = 350.0f
};


float roll_trim = 0.0f;
float pitch_trim = 0.06f;
float trim_gain = 0.2f;
const float trim_angle_thresh = 0.02f; 

const float angle_deadzone = 0.005f;   // ~0.3 stopnia
const float rate_deadzone  = 0.002f;
const float rate_limit  = 180.0f;
float for_debug = 0;
static inline float pid_rate_update(
    pid_t *pid,
    float rate_target,
    float rate_meas,
    float dt,
    float *gyro_state // To musi być unikalna zmienna dla każdej osi (np. pid->f_gyro)
){
    // 1. LPF na żyroskop (Alpha 0.30 - szybsza reakcja)
    float prev_gyro = *gyro_state;
    *gyro_state += 0.20f * (rate_meas - *gyro_state);

    // 2. Błąd
    float error = rate_target - *gyro_state;

    // 3. P-term i I-term (bez zmian)
    float P_term = pid->kp * error;
    pid->integrator += error * dt;
    if(pid->integrator > pid->i_limit) pid->integrator = pid->i_limit;
    if(pid->integrator < -pid->i_limit) pid->integrator = -pid->i_limit;
    float I_term = pid->ki * pid->integrator;

    // 4. WYGŁADZONY D-term
    // Liczymy surowe D
    float raw_D = -pid->kd * (*gyro_state - prev_gyro) / dt;
    
    // Dodajemy filtr LPF tylko dla D (tzw. D-term filter)
    // 'pid->last' użyjemy jako stan filtra dla D
    pid->last += 0.20f * (raw_D - pid->last); 
    float D_term = pid->last;

    // 6. Sumowanie
    float out = P_term + I_term + D_term;

    // 7. Rate limit (Slew rate - zapobiega uderzeniom prądowym w ESC)
    float delta = out - pid->prev_output;
    if(delta > rate_limit) delta = rate_limit;
    if(delta < -rate_limit) delta = -rate_limit;
    out = pid->prev_output + delta;

    // 8. Ograniczenia absolutne
    if(out > pid->out_limit) out = pid->out_limit;
    if(out < -pid->out_limit) out = -pid->out_limit;

    pid->prev_output = out;
    pid->prev_error = error;
    return out;
}



static inline float angle_to_rate(float error_angle) {
    float abs_err = fabsf(error_angle);
    // if (abs_err < 0.002f) return 0.0f; // TYMCZASOWO WYŁĄCZ
    float rate = kp_angle * abs_err; 
    if (rate > max_rate) rate = max_rate;
    return (error_angle > 0) ? rate : -rate;
}

static float frac_a = 0.0f;
static float frac_b = 0.0f;
static float frac_c = 0.0f;
static float frac_d = 0.0f;
float corr_limit = 150.0f;

static inline void update_trim(float roll, float pitch, float roll_rate_target, float pitch_rate_target, float dt) {
    // PROGI do aktywacji trimu
    const float rate_thresh  = 0.2f;  // rad/s, większy próg
    const float angle_thresh = 0.05f;  // rad, większy próg (~5-6 deg)

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
float ff_gain = 1.0f;      // 0.2 – 0.6 

float rate_feedforward(
    float rate_target,
    float prev_rate_target
) {
    float ff = ff_gain * (rate_target - prev_rate_target);
    if (ff > 5.0f) ff = 5.0f;
    if (ff < -5.0f) ff = -5.0f;
    return ff;
}

static float prev_roll_rate_target  = 0.0f;
static float prev_pitch_rate_target = 0.0f;
float gyro_filter_x = 0;
float gyro_filter_y = 0;

engines calculate_speed_pd(float roll, float pitch, vec3 gyro, float dt)
{
    // 1. SOFT START / RAMPING (Płynne dążenie do gazu z pętli wysokości)
    // Zmienna base_motor goni target_base_motor, żeby uniknąć szarpnięć
    if ((float)base_motor < target_base_motor) {
        base_motor += 300.0f * dt; // Rampa w górę
        if ((float)base_motor > target_base_motor) base_motor = target_base_motor;
    } else if ((float)base_motor > target_base_motor) {
        base_motor -= 500.0f * dt; // Rampa w dół
        if ((float)base_motor < target_base_motor) base_motor = target_base_motor;
    }

    // 2. TPA (Osłabianie korekcji PID przy wysokim gazie, by uniknąć oscylacji)
    float tpa = 1.0f;
    if (base_motor > 800) {
        tpa = 1.0f - (float)(base_motor - 800) * 0.0008f;
        if (tpa < 0.7f) tpa = 0.7f;
    }

    // ===== TRIM NA KĄCIE =====
    roll  += roll_trim;
    pitch += pitch_trim;

    // ===== ANGLE → RATE =====
    float roll_rate_target  = angle_to_rate(-roll);
    float pitch_rate_target = angle_to_rate(-pitch);

    // ===== RZECZYWISTE RATE =====
    float roll_rate_meas =gyro.x;
    float pitch_rate_meas = gyro.y;

    // ===== RATE PID (Z uwzględnieniem TPA) =====
    float rc = pid_rate_update(&pid_roll, roll_rate_target, roll_rate_meas, dt, &gyro_filter_x);
    float pc = pid_rate_update(&pid_pitch, pitch_rate_target, pitch_rate_meas, dt, &gyro_filter_y);
    rc += rate_feedforward(roll_rate_target, prev_roll_rate_target);
    pc += rate_feedforward(pitch_rate_target, prev_pitch_rate_target);
    // Aplikacja TPA na korekcje
    rc *= tpa;
    pc *= tpa;

    prev_roll_rate_target  = roll_rate_target;
    prev_pitch_rate_target = pitch_rate_target;

    // ===== LIMITY KOREKCJI =====
    if(rc >  corr_limit) rc =  corr_limit;
    if(rc < -corr_limit) rc = -corr_limit;
    if(pc >  corr_limit) pc =  corr_limit;
    if(pc < -corr_limit) pc = -corr_limit;

    // ===== MIXER X + FRAKCJE (Używamy płynnego base_motor zamiast target) =====
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

    // Całkowite wyłączenie jeśli gaz jest minimalny (Safe Disarm)
    if(base_motor < 20) {
        e.a = e.b = e.c = e.d = 0;
    }

    // ===== SLOW TRIM =====
    update_trim(roll, pitch, roll_rate_target, pitch_rate_target, dt);

    return e;
}


// ================= MAIN LOOP =================
void steer_by_orientation() {
    uint8_t buf[14];

    sleep_ms(1000);  // start delay
    ekf_init(&ekf);
    calibrate_imu_offsets(); // stabilny offset

    // ===== BEZPIECZNY START SILNIKÓW =====
    // val1 = val2 = val3 = val4 = hover_throttle;
    // eng1 = calculate_frame(val1,0);
    // eng2 = calculate_frame(val2,0);
    // eng3 = calculate_frame(val3,0);
    // eng4 = calculate_frame(val4,0);
    // sleep_ms(500); // 1 sekunda stabilizacji

    static uint64_t last_time = 0;
    static float ax_f = 0.0f, ay_f = 0.0f, az_f = 1.0f; 
    const float alpha = 0.5f;

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

        //Pod lpf(ewentualnie)
        // ax_f = (ax_f * (1.0f - alpha)) + (acc.x * alpha);
        // ay_f = (ay_f * (1.0f - alpha)) + (acc.y * alpha);
        // az_f = (az_f * (1.0f - alpha)) + (acc.z * alpha);

        // ===== EKF =====
        ekf_predict(&ekf, gyro.x, gyro.y, dt);
        ekf_update_accel(&ekf, acc.x / 9.80665f, acc.y / 9.80665f, acc.z / 9.80665f);

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
        // printf("roll: %3.3f, pitch: %3.3f | a:%d b:%d c:%d d:%d dt:%.4f\n",
        //        roll, pitch, val1, val2, val3, val4, dt);
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

bool altitude_measure_cm(float *alt_cm) {
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    uint64_t t0 = time_us_64();
    while (!gpio_get(ECHO_PIN)) {
        if (time_us_64() - t0 > 25000) return false; // Timeout 25ms
    }

    uint64_t start = time_us_64();
    while (gpio_get(ECHO_PIN)) {
        if (time_us_64() - start > 25000) return false;
    }

    uint64_t pulse = time_us_64() - start;
    float dist = pulse * 0.01715f; // Uproszczone pulse * 0.0343 / 2

    if (dist < 2.0f || dist > 300.0f) return false;
    *alt_cm = dist;
    return true;
}

// 2. Filtr medianowy z 5 próbek (bardzo odporny na "piki" sonaru)
bool altitude_filter(float raw_cm, float *alt_filt) {
    static float buffer[5] = {0, 0, 0, 0, 0};
    static int idx = 0;

    buffer[idx] = raw_cm;
    idx = (idx + 1) % 5;

    // Kopia do sortowania
    float sorted[5];
    for(int i=0; i<5; i++) sorted[i] = buffer[i];

    // Proste sortowanie bąbelkowe dla 5 elementów
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4 - i; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    // Jeśli środek jest zerem (brak danych na starcie), odrzuć
    if (sorted[2] < 1.0f) return false;

    *alt_filt = sorted[2]; // Mediana
    return true;
}

// 3. Struktura PID z adaptacją do napięcia baterii
typedef struct {
    float kp, ki, kd;
    float integrator;
    float prev_error;
    float i_limit;
    float out_limit;
} alt_pid_t;

// Zwiększone KI dla adaptacji do słabnącej baterii
alt_pid_t alt_pid = {
    .kp = 3.2f,
    .ki = 1.1f,        // Silna całka: kompensuje spadek V w czasie rzeczywistym
    .kd = 1.4f,
    .integrator = 0.0f,
    .prev_error = 0.0f,
    .i_limit = 200.0f,  // Duży limit, aby mógł "dopompować" sporo gazu przy 11V
    .out_limit = 250.0f
};

// 4. Update PID wysokości
float altitude_pid_update(alt_pid_t *pid, float target_cm, float current_cm, float dt) {
    float err = target_cm - current_cm;

    // Martwa strefa - dron nie "pływa" góra-dół przy małych szumach
    if (fabsf(err) < 1.5f) err = 0;

    // Całka zbiera "brakującą moc" rozładowanej baterii
    pid->integrator += pid->ki * err * dt;
    if (pid->integrator > pid->i_limit)  pid->integrator = pid->i_limit;
    if (pid->integrator < -pid->i_limit) pid->integrator = -pid->i_limit;

    // Pochodna (D) tłumi gwałtowne ruchy
    float d = (err - pid->prev_error) / dt;
    pid->prev_error = err;

    float out = (pid->kp * err) + pid->integrator + (pid->kd * d);

    if (out > pid->out_limit)  out = pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;

    return out;
}

// 5. Główny krok pętli (Rdzeń 0)
void steer_by_altitude_step(float target_alt_cm) {
    static absolute_time_t next_time = {0};
    const float dt = 0.05f; // 50 ms

    if (is_nil_time(next_time)) {
        next_time = delayed_by_ms(get_absolute_time(), 50);
        return;
    }
    if (!time_reached(next_time)) return;
    next_time = delayed_by_ms(next_time, 50);

    float raw_alt;
    if (!altitude_measure_cm(&raw_alt)) return;

    float alt_filt;
    if (!altitude_filter(raw_alt, &alt_filt)) return;

    // Jeśli target jest bliski 0, wyłączamy silniki całkowicie (Safe Disarm)
    if (target_alt_cm < 5.0f) {
        target_base_motor = 0;
        alt_pid.integrator = 0; // Resetuj naukę baterii po wylądowaniu
        return;
    }

    float corr = altitude_pid_update(&alt_pid, target_alt_cm, alt_filt, dt);

    // Ustawiamy TARGET dla drugiego rdzenia (Soft Start obsłuży resztę)
    float new_base = (float)hover_throttle + corr;

    // Bezpieczne granice dla base_motor
    if (new_base > 950.0f) new_base = 950.0f;
    if (new_base < 150.0f) new_base = 150.0f;

    target_base_motor = new_base;
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
    sleep_ms(1500);
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
        //printf("iterm: %3.3f\n", for_debug);   

    }
    return 0;
}
