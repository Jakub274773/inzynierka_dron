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

void init_i2c(){
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    sleep_ms(200);
}

typedef struct { float x, y, z; } vec3;
typedef struct { uint16_t a, b, c, d; } engines;

/* ================= PI CONTROLLER ================= */

typedef struct {
    float kp;
    float ki;
    float integral;
    float integral_limit;
} pi_t;

static inline float pi_update(pi_t *pi, float error, float dt)
{
    pi->integral += error * dt;

    if (pi->integral > pi->integral_limit)
        pi->integral = pi->integral_limit;
    else if (pi->integral < -pi->integral_limit)
        pi->integral = -pi->integral_limit;

    return pi->kp * error + pi->ki * pi->integral;
}

/* --- regulatory --- */
pi_t pi_roll  = { .kp = 0.6f, .ki = 0.3f, .integral = 0, .integral_limit = 30 };
pi_t pi_pitch = { .kp = 0.6f, .ki = 0.3f, .integral = 0, .integral_limit = 30 };

/* ================= UTILS ================= */

static inline uint16_t slew_u16(uint16_t target, uint16_t current, uint16_t max_step)
{
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}

static inline vec3 convert_accel(int16_t ax, int16_t ay, int16_t az)
{
    const float scale = 9.80665f / 16384.0f;
    return (vec3){ ax * scale, ay * scale, az * scale };
}

static inline vec3 convert_gyro(int16_t gx, int16_t gy, int16_t gz)
{
    const float scale = 0.0001331f;
    return (vec3){ gx * scale, gy * scale, gz * scale };
}

/* ================= PI MIXER ================= */

engines calculate_speed_pi(engines base, float roll_err, float pitch_err, float dt)
{
    float roll_corr  = pi_update(&pi_roll,  roll_err,  dt);
    float pitch_corr = pi_update(&pi_pitch, pitch_err, dt);

    engines e;

    e.a = base.a + roll_corr - pitch_corr;
    e.b = base.b - roll_corr - pitch_corr;
    e.c = base.c + roll_corr + pitch_corr;
    e.d = base.d - roll_corr + pitch_corr;

    return e;
}

/* ================= MAIN LOOP ================= */

void steer_by_orientation()
{
    int8_t wake_cmd[2] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, 0x68, wake_cmd, 2, false);
    sleep_ms(100);

    uint8_t buf[14];

    float currentx = 0, currenty = 0;
    float lastx = 0, lasty = 0;

    uint64_t last_t = time_us_64();
    engines eng;

    sleep_ms(3000);

    while (1)
    {
        uint64_t now = time_us_64();
        float dt = (now - last_t) / 1e6f;
        last_t = now;

        uint8_t reg = 0x3B;
        i2c_write_blocking(I2C_PORT, 0x68, &reg, 1, true);
        if (i2c_read_blocking(I2C_PORT, 0x68, buf, 14, false) != 14)
            continue;

        int16_t ax = (buf[0] << 8) | buf[1];
        int16_t ay = (buf[2] << 8) | buf[3];
        int16_t az = (buf[4] << 8) | buf[5];
        int16_t gx = (buf[8] << 8) | buf[9];
        int16_t gy = (buf[10] << 8) | buf[11];

        vec3 accel = convert_accel(ax, ay, az);
        vec3 gyro  = convert_gyro(gx, gy, 0);

        accel.z -= 0.65f;
        accel.x -= gyro.y * gyro.y * 0.03f;
        accel.y -= gyro.x * gyro.x * 0.01f;

        /* proste wygładzenie */
        lastx = currentx;
        lasty = currenty;
        currentx += 0.1f * (accel.x - currentx);
        currenty += 0.1f * (accel.y - currenty);

        eng.a = val1;
        eng.b = val2;
        eng.c = val3;
        eng.d = val4;

        engines target = calculate_speed_pi(eng, currentx, currenty, dt);

        val1 = slew_u16(target.a, val1, 2);
        val2 = slew_u16(target.b, val2, 2);
        val3 = slew_u16(target.c, val3, 2);
        val4 = slew_u16(target.d, val4, 2);

        if (val1 < 100) val1 = 100; if (val1 > 300) val1 = 300;
        if (val2 < 100) val2 = 100; if (val2 > 300) val2 = 300;
        if (val3 < 100) val3 = 100; if (val3 > 300) val3 = 300;
        if (val4 < 100) val4 = 100; if (val4 > 300) val4 = 300;

        eng1 = calculate_frame(val1, 0);
        eng2 = calculate_frame(val2, 0);
        eng3 = calculate_frame(val3, 0);
        eng4 = calculate_frame(val4, 0);

        printf("e1 %d e2 %d e3 %d e4 %d\n", val1, val2, val3, val4);
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


void program(){

  while(true){
eng1 = calculate_frame(val1, 0);
eng2 = calculate_frame(val2, 0);
eng3 = calculate_frame(val3, 0);
eng4 = calculate_frame(val4, 0);
  }
//i2c_write_blocking(i2c_default, 0x68, &reg, 1, false);
}

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
    multicore_launch_core1(steer_by_orientation);
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
    

    while(true){
      tight_loop_contents();
    }
    return 0;
}
