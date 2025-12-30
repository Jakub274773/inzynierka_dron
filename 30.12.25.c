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
volatile int base_motor = 100;
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
    float q0,q1,q2,q3;   // kwaternion
    float bx, by, bz;    // bias żyroskopu
    float gyro_noise;
    float acc_noise;
} ekf_t;

ekf_t ekf;

void ekf_init(ekf_t *ekf) {
    ekf->q0 = 1.0f; ekf->q1 = ekf->q2 = ekf->q3 = 0.0f;
    ekf->bx = gyro_offset.x;
    ekf->by = gyro_offset.y;
    ekf->bz = gyro_offset.z;
    ekf->gyro_noise = 0.002f;
    ekf->acc_noise  = 0.05f;
}

static inline void quat_normalize(float *q0,float *q1,float *q2,float *q3) {
    float norm = sqrtf((*q0)*(*q0)+(*q1)*(*q1)+(*q2)*(*q2)+(*q3)*(*q3));
    *q0/=norm; *q1/=norm; *q2/=norm; *q3/=norm;
}

void ekf_predict(ekf_t *ekf, float gx, float gy, float gz, float dt) {
    float wx = gx - ekf->bx;
    float wy = gy - ekf->by;
    float wz = gz - ekf->bz;

    float q0=ekf->q0, q1=ekf->q1, q2=ekf->q2, q3=ekf->q3;

    float dq0 = 0.5f*(-q1*wx - q2*wy - q3*wz);
    float dq1 = 0.5f*( q0*wx + q2*wz - q3*wy);
    float dq2 = 0.5f*( q0*wy - q1*wz + q3*wx);
    float dq3 = 0.5f*( q0*wz + q1*wy - q2*wx);

    ekf->q0 += dq0*dt;
    ekf->q1 += dq1*dt;
    ekf->q2 += dq2*dt;
    ekf->q3 += dq3*dt;

    quat_normalize(&ekf->q0,&ekf->q1,&ekf->q2,&ekf->q3);
}

void ekf_update_acc(ekf_t *ekf, float ax, float ay, float az) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm < 0.01f) return;

    ax /= norm; ay /= norm; az /= norm;

    float gx = 2*(ekf->q1*ekf->q3 - ekf->q0*ekf->q2);
    float gy = 2*(ekf->q0*ekf->q1 + ekf->q2*ekf->q3);
    float gz = ekf->q0*ekf->q0 - ekf->q1*ekf->q1 - ekf->q2*ekf->q2 + ekf->q3*ekf->q3;

    float ex = (ay*gz - az*gy);
    float ey = (az*gx - ax*gz);
    float ez = (ax*gy - ay*gx);

    float Kp = 0.5f;

    ekf->q0 += -0.5f*(ekf->q1*ex + ekf->q2*ey + ekf->q3*ez) * Kp;
    ekf->q1 +=  0.5f*(ekf->q0*ex + ekf->q2*ez - ekf->q3*ey) * Kp;
    ekf->q2 +=  0.5f*(ekf->q0*ey - ekf->q1*ez + ekf->q3*ex) * Kp;
    ekf->q3 +=  0.5f*(ekf->q0*ez + ekf->q1*ey - ekf->q2*ex) * Kp;

    quat_normalize(&ekf->q0,&ekf->q1,&ekf->q2,&ekf->q3);
}

static inline void ekf_get_angles(ekf_t *ekf,float *roll,float *pitch) {
    *roll  = atan2f(2*(ekf->q0*ekf->q1 + ekf->q2*ekf->q3),
                     1 - 2*(ekf->q1*ekf->q1 + ekf->q2*ekf->q2));
    *pitch = -asinf(2*(ekf->q0*ekf->q2 - ekf->q3*ekf->q1));
}

// float roll_zero = 0.0f;
// float pitch_zero = 0.0f;

void calibrate_level_ekf(void)
{
    const int N = 300;
    float r = 0, p = 0;

    for (int i = 0; i < N; i++) {
        float roll, pitch;
        ekf_get_angles(&ekf, &roll, &pitch);
        r += roll;
        p += pitch;
        sleep_ms(2);
    }

    roll_zero  = r / N;
    pitch_zero = p / N;
}


// ================= PD =================
typedef struct {
    float kp;
    float ki;
    float kd;

    float integrator;
    float prev_error;

    float out_limit;
    float i_limit;
    float last;
} pid_t;



// ANGLE loop
float kp_angle = 3.0f;        // rad/s per rad
float max_rate = 4.0f;        // ~350 deg/s

// RATE loop
pid_t pid_roll = {
    .kp = 6.0f,
    .ki = 0.20f,
    .kd = 0.002f,
    .integrator = 0,
    .prev_error = 0,
    .out_limit = 300.0f,
    .i_limit = 100.0f
};

pid_t pid_pitch = {
    .kp = 6.0f,
    .ki = 0.20f,
    .kd = 0.002f,
    .integrator = 0,
    .prev_error = 0,
    .out_limit = 300.0f,
    .i_limit = 100.0f
};


static inline float pid_rate_update(
    pid_t *pid,
    float rate_target,   // rad/s
    float rate_meas,     // rad/s (gyro!)
    float dt
) {
    float error = rate_target - rate_meas;

    // P
    float p = pid->kp * error;

    // I – integruje także dla małych błędów, aby kompensować asymetrię
    pid->integrator += pid->ki * error * dt;
    if (pid->integrator > pid->i_limit)  pid->integrator = pid->i_limit;
    if (pid->integrator < -pid->i_limit) pid->integrator = -pid->i_limit;

    // D (z błędu RATE)
    float d = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float out = p + pid->integrator + d;

    if (out > pid->out_limit)  out = pid->out_limit;
    if (out < -pid->out_limit) out = -pid->out_limit;

    return out;
}

#define MIN_RATE_CORRECTION 0.02f   // rad/s, dostosuj eksperymentalnie

static inline float angle_to_rate(
    float angle_error,   // rad
    float kp_angle,
    float max_rate
) {
    // nieliniowe wzmocnienie: małe błędy mają większy wpływ
    float sign = (angle_error >= 0.0f) ? 1.0f : -1.0f;
    float rate = kp_angle * angle_error * fabs(angle_error);  // kwadrat błędu

    // minimalna korekcja, by przezwyciężyć martwą strefę
    if (fabs(rate) < MIN_RATE_CORRECTION) {
        rate = sign * MIN_RATE_CORRECTION;
    }

    if (rate > max_rate)  rate = max_rate;
    if (rate < -max_rate) rate = -max_rate;

    return rate;
}


// void calibrate_level() {
//     const int N = 400;
//     float r = 0.0f, p = 0.0f;

//     for (int i = 0; i < N; i++) {
//         float roll, pitch;
//         mahony_get_angles(&mahony, &roll, &pitch);
//         r += roll;
//         p += pitch;
//         sleep_ms(2);
//     }

//     roll_zero  = r / N;
//     pitch_zero = p / N;
// }
// ================= MIXER =================
static float frac_a = 0.0f;
static float frac_b = 0.0f;
static float frac_c = 0.0f;
static float frac_d = 0.0f;
// volatile float roll_corr = 0;
// volatile float pitch_corr = 0;
float corr_limit = 25;

engines calculate_speed_pd(
    float roll,
    float pitch,
    vec3 gyro,
    float dt
) {
    // === ANGLE → RATE ===
    float roll_rate_target =
        angle_to_rate(-roll, kp_angle, max_rate);

    float pitch_rate_target =
        angle_to_rate(-pitch, kp_angle, max_rate);

    // === RATE PID ===
    float rc = pid_rate_update(
        &pid_roll,
        roll_rate_target,
        -gyro.x,
        dt
    );

    float pc = pid_rate_update(
        &pid_pitch,
        pitch_rate_target,
        -gyro.y,
        dt
    );

    printf("angle: %.3f  rate_t: %.3f  gyro: %.3f  out: %.2f\n",
       roll,
       roll_rate_target,
       gyro.x,
       rc);


    // === MIXER X ===
    float fa = base_motor - pc + rc + frac_a;
    float fb = base_motor + pc + rc + frac_b;
    float fc = base_motor - pc - rc + frac_c;
    float fd = base_motor + pc - rc + frac_d;

    engines e;

    e.a = (uint16_t)fa;
    e.b = (uint16_t)fb;
    e.c = (uint16_t)fc;
    e.d = (uint16_t)fd;

    frac_a = fa - e.a;
    frac_b = fb - e.b;
    frac_c = fc - e.c;
    frac_d = fd - e.d;

    // === LIMITY ===
    if(e.a<min_motor_speed)e.a=min_motor_speed; if(e.b<min_motor_speed)e.b=min_motor_speed;
    if(e.c<min_motor_speed)e.c=min_motor_speed; if(e.d<min_motor_speed)e.d=min_motor_speed;
    if(e.a>max_motor_speed)e.a=max_motor_speed; if(e.b>max_motor_speed)e.b=max_motor_speed;
    if(e.c>max_motor_speed)e.c=max_motor_speed; if(e.d>max_motor_speed)e.d=max_motor_speed;

    return e;
}


// ================= MAIN LOOP =================
void steer_by_orientation() {
    uint8_t buf[14];
    sleep_ms(3000);

    ekf_init(&ekf);
    calibrate_level_ekf();

    while(1){
        uint64_t start_time = time_us_64();  // początek pomiaru

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

        gyro.x  -= gyro_offset.x;
        gyro.y  -= gyro_offset.y;
        gyro.z  -= gyro_offset.z;

        acc.x -= accel_offset.x;
        acc.y -= accel_offset.y;
        acc.z -= accel_offset.z;

        float dt = 0.002f;

        ekf_predict(&ekf, gyro.x, gyro.y, gyro.z, dt);
        ekf_update_acc(&ekf, acc.x/9.80665f, acc.y/9.80665f, acc.z/9.80665f);

        float roll, pitch;
        ekf_get_angles(&ekf, &roll, &pitch);

        printf("roll: %3.3f, pitch: %3.3f\n", roll, pitch);

        engines base = {val1,val2,val3,val4};
        engines out = calculate_speed_pd(roll - base_roll, pitch - base_pitch, gyro, dt);

        val1=out.a; val2=out.b; val3=out.c; val4=out.d;
        eng1=calculate_frame(val1,0);
        eng2=calculate_frame(val2,0);
        eng3=calculate_frame(val3,0);
        eng4=calculate_frame(val4,0);

        printf("a: %d, B: %d, C: %d, D: %d\n", val1, val2, val3, val4);

        uint64_t end_time = time_us_64();  // koniec pomiaru
        printf("Iteration time: %llu us\n", end_time - start_time);
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

static pid_t params = {0.20f, 0, 0.15f};   // PID wysokości

void steer_by_altitude() {
    static absolute_time_t next_time = 0;

    const float dt = 0.05f;   // 50 ms

    // Inicjalizacja pierwszego wywołania
    if (next_time == 0) {
        next_time = delayed_by_ms(get_absolute_time(), 49);
    }

    // Jeśli jeszcze nie czas — wyjdź
    if (!time_reached(next_time)) {
        return;
    }

    // Ustaw kolejny termin
    next_time = delayed_by_ms(next_time, 50);

    // ======= Pomiar SR04 =======
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    uint64_t t0 = time_us_64();
    while (!gpio_get(ECHO_PIN)) {
        if (time_us_64() - t0 > 30000) goto measurement_failed;
    }

    uint64_t start = time_us_64();

    while (gpio_get(ECHO_PIN)) {
        if (time_us_64() - start > 30000) goto measurement_failed;
    }

    uint64_t pulse = time_us_64() - start;
    float distance_cm = (pulse * 0.0343f) * 0.5f;

    // ======= PID wysokości =======
    float err = target_alt - distance_cm;
    float d   = (err - params.last) / dt;
    params.last = err;

    float alt_corr = params.kp * err + params.kd * d;

    if (alt_corr > 10)  alt_corr = 10;
    if (alt_corr < -10) alt_corr = -10;

     base_motor += alt_corr;

    if (base_motor > max_base_motor) base_motor = max_base_motor;
    if (base_motor < min_base_motor) base_motor = min_base_motor;

    return;

measurement_failed:
    params.last = 0;
}




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
        steer_by_altitude();    

    }
    return 0;
}
