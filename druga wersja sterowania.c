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

void init_i2c() {
    // Start I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    sleep_ms(200);

    // Wake MPU6500
    uint8_t wake_cmd[2] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, 0x68, wake_cmd, 2, false);
    sleep_ms(100);

    // Set Gyro DLPF = 41 Hz   (CONFIG register 0x1A)
    uint8_t dlpf_gyro[2] = {0x1A, 0x03};
    i2c_write_blocking(I2C_PORT, 0x68, dlpf_gyro, 2, false);

    // Set Accel DLPF = 41 Hz  (ACCEL_CONFIG2 register 0x1D)
    uint8_t dlpf_accel[2] = {0x1D, 0x03};
    i2c_write_blocking(I2C_PORT, 0x68, dlpf_accel, 2, false);

    // Optional: you can sleep a bit for safety
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

typedef struct {
    float kp;
    float kd;
    float last_error;
} pd_t;

static inline float pd_update(pd_t *pd, float error, float dt) {
    float derivative = (error - pd->last_error) / dt;
    pd->last_error = error;
    return pd->kp * error + pd->kd * derivative;
}

// -----------------------------
// Globalne regulatory PD
// -----------------------------
pd_t pd_roll  = { .kp = 2.0f, .kd = 0.3f, .last_error = 0.0f };
pd_t pd_pitch = { .kp = 2.0f, .kd = 0.3f, .last_error = 0.0f };

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
engines calculate_speed_pd(
    engines base,
    float currentx,
    float lastx,
    float currenty,
    float lasty,
    float dt
) {
    float roll_error  = currentx;
    float pitch_error = currenty;

    float roll_corr  = pd_update(&pd_roll,  roll_error,  dt);
    float pitch_corr = pd_update(&pd_pitch, pitch_error, dt);

    if(roll_corr > 4) roll_corr = 4;
    if(roll_corr < -4) roll_corr = -4;
    if(pitch_corr > 4) pitch_corr = 4;
    if(pitch_corr < -4) pitch_corr = -4;

    printf("x %6.3f,  y %6.3f", roll_corr, pitch_corr);

    engines eng;

    // Mixer dopasowany do układu B D / A C
    eng.a = (int)roundf(base.a + roll_corr - pitch_corr);   // tył-lewo
    eng.b = (int)roundf(base.b - roll_corr - pitch_corr);   // przód-lewo
    eng.c = (int)roundf(base.c + roll_corr + pitch_corr);   // tył-prawo
    eng.d = (int)roundf(base.d - roll_corr + pitch_corr);   // przód-prawo

    // eng.a = 100;
    // eng.b = 100;
    // eng.c = 100;
    // eng.d = 100;

    if (eng.a > 300) eng.a = 300;
    if (eng.b > 300) eng.b = 300;
    if (eng.c > 300) eng.c = 300;
    if (eng.d > 300) eng.d = 300;

    if (eng.a < 100) eng.a = 100;
    if (eng.b < 100) eng.b = 100;
    if (eng.c < 100) eng.c = 100;
    if (eng.d < 100) eng.d = 100;

    return eng;
}

// -----------------------------
// Pętla sterowania orientacją
// -----------------------------
void steer_by_orientation() {

    uint8_t buf[14];

    uint64_t last_t = time_us_64();
    float lastx = 0;
    float lasty = 0;
    float currentx = 0;
    float currenty = 0;

    int counter = 0;
    engines eng;

    float buf_x[5];
    float buf_y[5];
    int idx = 0;
    int idy = 0;

    sleep_ms(3000);

    while (1) {
        // Odczyt IMU
        uint8_t reg = 0x3B;
        i2c_write_blocking(I2C_PORT, 0x68, &reg, 1, true);
        int ret = i2c_read_blocking(I2C_PORT, 0x68, buf, 14, false);
        if (ret != 14) continue;

        int16_t ax = (buf[0] << 8) | buf[1];
        int16_t ay = (buf[2] << 8) | buf[3];
        int16_t az = (buf[4] << 8) | buf[5];
        vec3 accel = convert_accel(ax, ay, az);
        //accel.z -= 0.650f;

        int16_t gx = (buf[8] << 8) | buf[9];
        int16_t gy = (buf[10] << 8) | buf[11];
        int16_t gz = (buf[12] << 8) | buf[13];
        vec3 gyro = convert_gyro(gx, gy, gz);

        accel.x -= gyro.y * gyro.y * 0.03f;
        accel.y -= gyro.x * gyro.x * 0.01f;

        lastx = currentx;
        lasty = currenty;
        currentx = lastx + 0.15f * (accel.x - lastx);
        currenty = lasty + 0.15f * (accel.y - lasty);

        // buf_x[idx] = currentx;
        // idx = (idx+1) % 5;

        // float sum = 0;
        // for (int i=0; i<5; i++) sum += buf_x[i];
        // currentx = sum / 5;

        // buf_y[idy] = currenty;
        // idx = (idy+1) % 5;

        // sum = 0;
        // for (int i=0; i<5; i++) sum += buf_y[i];
        // currenty = sum / 5;

        
        // Wartości bazowe silników
        eng.a = val1;
        eng.b = val2;
        eng.c = val3;
        eng.d = val4;

        // Sterowanie PD
        float dt = 0.002f; // odpowiada pętli ~500 Hz
        eng = calculate_speed_pd(eng, currentx, lastx, currenty, lasty, dt);

        // Zapis do zmiennych globalnych / DMA
        val1 = eng.a;
        val2 = eng.b;
        val3 = eng.c;
        val4 = eng.d;

        eng1 = calculate_frame(val1, 0);
        eng2 = calculate_frame(val2, 0);
        eng3 = calculate_frame(val3, 0);
        eng4 = calculate_frame(val4, 0);

        // Debug
        // printf("ACC: %6.3f %6.3f %6.3f   |  GYRO(rad/s): %6.3f %6.3f %6.3f\n",
        //     accel.x, accel.y, accel.z,
        //     gyro.x, gyro.y, gyro.z);
        printf("engine1 %d, engine2 %d, engine3 %d, engine4 %d\n",
            val1, val2, val3, val4);

        sleep_ms(1);
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
