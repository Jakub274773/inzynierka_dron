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

//defy dla uart
#define UART_ID uart0
#define BAUD_RATE 9600      // musi zgadzać się z RPi Zero
#define UART_RX_PIN 1       // RX Pico podłączony do TX RPi
#define UART_TX_PIN 0 
#define MPU_ADDR 0x68
#define BUF_SIZE 64
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

// typedef struct {
//     float kp;
//     float kd;
//     float last_error;
// } pd_t;

// static inline float pd_update(pd_t *pd, float error, float dt) {
//     float derivative = (error - pd->last_error) / dt;
//     pd->last_error = error;
//     return pd->kp * error + pd->kd * derivative;
// }

// -----------------------------
// Globalne regulatory PD
// -----------------------------
// pd_t pd_roll  = { .kp = 0.1f, .kd = 0.4f, .last_error = 0.0f };
// pd_t pd_pitch = { .kp = 0.1f, .kd = 0.4f, .last_error = 0.0f };

// -----------------------------
// Funkcja sterowania PD + mixer
// Układ silników:
//        front
//         ^
//         |
//       B   D
//         X
//       A   C
// -----------------------------

typedef struct {
    float q0,q1,q2,q3;
    float integralFBx, integralFBy, integralFBz;
    float twoKp, twoKi;
} mahony_t;

mahony_t mahony = {
    .q0=1,.q1=0,.q2=0,.q3=0,
    .twoKp=2.0f,
    .twoKi=0.0f
};

static inline void mahony_update(mahony_t *m,
    float gx,float gy,float gz,
    float ax,float ay,float az,
    float dt)
{
    float recipNorm, halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa,qb,qc;

    float alen2 = ax*ax + ay*ay + az*az;
    if (alen2 > 0.81f && alen2 < 1.21f) {
        recipNorm = 1.0f / sqrtf(alen2);
        ax*=recipNorm; ay*=recipNorm; az*=recipNorm;

        halfvx = m->q1*m->q3 - m->q0*m->q2;
        halfvy = m->q0*m->q1 + m->q2*m->q3;
        halfvz = m->q0*m->q0 - 0.5f + m->q3*m->q3;

        halfex = (ay*halfvz - az*halfvy);
        halfey = (az*halfvx - ax*halfvz);
        halfez = (ax*halfvy - ay*halfvx);

        gx += m->twoKp * halfex;
        gy += m->twoKp * halfey;
        gz += m->twoKp * halfez;
    }

    gx*=0.5f*dt; gy*=0.5f*dt; gz*=0.5f*dt;
    qa=m->q0; qb=m->q1; qc=m->q2;

    m->q0 += (-qb*gx - qc*gy - m->q3*gz);
    m->q1 += (qa*gx + qc*gz - m->q3*gy);
    m->q2 += (qa*gy - qb*gz + m->q3*gx);
    m->q3 += (qa*gz + qb*gy - qc*gx);

    recipNorm = 1.0f/sqrtf(m->q0*m->q0+m->q1*m->q1+m->q2*m->q2+m->q3*m->q3);
    m->q0*=recipNorm; m->q1*=recipNorm;
    m->q2*=recipNorm; m->q3*=recipNorm;
}

static inline void mahony_get_angles(mahony_t *m,float *roll,float *pitch) {
    *roll = atan2f(2*(m->q0*m->q1+m->q2*m->q3),
                   1-2*(m->q1*m->q1+m->q2*m->q2));
    *pitch = asinf(2*(m->q0*m->q2-m->q3*m->q1));
}

// ================= PD =================
typedef struct {
    float kp,kd,last;
} pd_t;

pd_t pd_roll  = {0.8f, 0.45f,0};
pd_t pd_pitch = {0.8f, 0.45f,0};

static inline float pd_update(pd_t *pd,float e,float dt){
    float d = (e - pd->last)/dt;
    printf("derivative: %3.3f\n", d);
    pd->last = e;
    return pd->kp*e + pd->kd*d;
}

// ================= MIXER =================
static float frac_a = 0.0f;
static float frac_b = 0.0f;
static float frac_c = 0.0f;
static float frac_d = 0.0f;

engines calculate_speed_pd(engines base,float roll,float pitch,float dt){
    float rc = pd_update(&pd_roll, roll, dt);
    float pc = pd_update(&pd_pitch, pitch, dt);

    if(rc>3.5)rc=3.5; if(rc<-3.5)rc=-3.5;
    if(pc>3.5)pc=3.5; if(pc<-3.5)pc=-3.5;

    printf("RC: %3.3f, PC: %3.3f\n", rc, pc);
    float fa = base.a - rc - pc + frac_a;
    float fb = base.b - rc + pc + frac_b;
    float fc = base.c + rc - pc + frac_c;
    float fd = base.d + rc + pc + frac_d;

    engines e;

    e.a = (uint16_t)fa;
    e.b = (uint16_t)fb;
    e.c = (uint16_t)fc;
    e.d = (uint16_t)fd;

    frac_a = fa - e.a;
    frac_b = fb - e.b;
    frac_c = fc - e.c;
    frac_d = fd - e.d;

    if(e.a<100)e.a=100; if(e.b<100)e.b=100;
    if(e.c<100)e.c=100; if(e.d<100)e.d=100;
    if(e.a>500)e.a=500; if(e.b>500)e.b=500;
    if(e.c>500)e.c=500; if(e.d>500)e.d=500;
    printf("A: %d, B: %d, C: %d, D: %d\n", e.a, e.b, e.c, e.d);
    return e;
}

// ================= MAIN LOOP =================
void steer_by_orientation() {
    uint8_t buf[14];
    sleep_ms(3000);

    while(1){
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

        float dt = 0.002f;

        mahony_update(&mahony,
            gyro.x,gyro.y,gyro.z,
            acc.x/9.80665f,
            acc.y/9.80665f,
            acc.z/9.80665f,
            dt);

        float roll,pitch;
        mahony_get_angles(&mahony,&roll,&pitch);
        
        printf("ROLL[X]: %7.2f deg | PITCH[Y]: %7.2f deg\n",
        roll * 57.29578f,
        pitch * 57.29578f);

        engines base = {val1,val2,val3,val4};
        engines out = calculate_speed_pd(base, roll, pitch, dt);

        val1=out.a; val2=out.b; val3=out.c; val4=out.d;
        eng1=calculate_frame(val1,0);
        eng2=calculate_frame(val2,0);
        eng3=calculate_frame(val3,0);
        eng4=calculate_frame(val4,0);
    }
}




// Wysokosc + DMA + przerwanie
// void wys_dma_init(){}
// int height = 0;
// int height_preset = 1500;
// void steer_by_altitude(){

//   if(height > (height_preset + 50)){
//     //zmniejsz predkosc o 2
//   }
//   else if (height < height_preset - 50){
//     //zwieksz predkosc o 2
//   }
// }

// przerwanie UART RX

void on_uart_irq() {
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);

        // Przechwytywanie linii kończącej '\n'
        if (c == '\n' || c == '\r') {
            rx_buffer[rx_pos] = '\0';  // zakończ string
            val1 = val2 = val3 = val4 = (uint16_t)strtol(rx_buffer, NULL, 10);
            //("Odebrano: %d\n");
            printf("wartos %d", val1);
            printf("wartos %d", eng1);
            rx_pos = 0; 
        } else {
            if (rx_pos < BUF_SIZE - 1) {
                rx_buffer[rx_pos++] = c;
            }
        }
    }
}


// silniki dshot

void start_cycle(){
  int Xcur;
  int Ycur;
  int Zcur;

  val1 += 50;
  val2 += 50;
  val3 += 50;
  val4 += 50;

  // while(true){
  //   // pobierz dane z imu oraz czujnika odległosci
  //   // jeśli odległość od podłogi zaczyna się zwiększać to ustaw te wartości predkosći jako base i zakończ
  //   if(Xcur > 1.05*x){
  //     Preval1 = val1 - 20;
  //     Preval2 = val2 - 20;
  //     Preval3 = val3 - 20;
  //     Preval4 = val4 - 20;
  //     break;
  //   }
  //   else{
  //     val1 += 20;
  //     val2 += 20;
  //     val3 += 20;
  //     val4 += 20;
  //   }
  //   sleep_ms(500);
  // }
}


// void program(){

//   while(true){
// eng1 = calculate_frame(val1, 0);
// eng2 = calculate_frame(val2, 0);
// eng3 = calculate_frame(val3, 0);
// eng4 = calculate_frame(val4, 0);
//   }
// //i2c_write_blocking(i2c_default, 0x68, &reg, 1, false);
// }

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

int main()

{
    init_i2c();
  // Inicjalizacja pinow
    stdio_init_all();
    uint offset = pio_add_program(pio, &dshot_program);
    dshot_program_init(pio, sm1, offset, 21, true);
    dshot_program_init(pio, sm2, offset, 20, true);
    dshot_program_init(pio, sm3, offset, 19, true);
    dshot_program_init(pio, sm4, offset, 18, true);
    
    

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Włącz IRQ tylko dla odbioru
    uart_set_irq_enables(UART_ID, true, false);
    irq_set_exclusive_handler(UART0_IRQ, on_uart_irq);
    irq_set_enabled(UART0_IRQ, true);

    init_dma_eng(DREQ_PIO0_TX0, &pio0->txf[sm1], &eng1);
    init_dma_eng(DREQ_PIO0_TX1, &pio0->txf[sm2], &eng2);
    init_dma_eng(DREQ_PIO0_TX2, &pio0->txf[sm3], &eng3);
    init_dma_eng(DREQ_PIO0_TX3, &pio0->txf[sm4], &eng4);
    
    multicore_launch_core1(steer_by_orientation);


    while(true){
      tight_loop_contents();
    }
    return 0;
}
