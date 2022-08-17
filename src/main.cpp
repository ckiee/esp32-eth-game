#include "Arduino.h"
#include "driver/gpio.h"
#include "esp32-hal-adc.h"
#include "esp32-hal-cpu.h"
#include "esp32-hal-dac.h"
#include "esp32-hal-gpio.h"
#include "esp_cpu.h"
#include "esp_int_wdt.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"
#include "hal/cpu_types.h"
#include "hal/i2s_types.h"
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "driver/i2s.h"
#include <cstdint>
#include <cstdlib>

// STATUS:
// I tried using i2s for this, but it complains that
// 80MHz is too fast and IDF can't figure out
// how to calculate the clock.
//
// Next up is using one of the cores to write CPU
// instructions for the other one. At 240MHz we have
// 24 instructions for each 10MHz 10BASE-T clock, which
// should hopefully be enough. Further down in this file
// are various attempts at fitting it in, the best coming
// in at 22 cycles. So we even have 2 to spare occasionally!
//

const char rx_pin = 25;
const char tx_pin = 26;

void dbgTask(void *unused);
void setup() {
    pinMode(rx_pin, INPUT);
    pinMode(tx_pin, OUTPUT);
    Serial.begin(115200);
    setCpuFrequencyMhz(240);
    Serial.print("cpu freq: ");
    Serial.println(getCpuFrequencyMhz());
    const auto i2s_num = I2S_NUM_0;
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate = (uint32_t)(10e6*8),
        .bits_per_sample = I2S_BITS_PER_SAMPLE_8BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // Interrupt level 1, default 0
        .dma_buf_count = 8,
        .dma_buf_len = 300,
        // .use_apll = true,
        .tx_desc_auto_clear = false,
    };
    static const i2s_pin_config_t pin_config = {
        .bck_io_num = 4,
        .ws_io_num = 5,
        .data_out_num = tx_pin,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    i2s_driver_install(i2s_num, &i2s_config, 0, NULL);   //install and start i2s driver
    i2s_set_pin(i2s_num, &pin_config);

    int tx_size = 114;
    uint8_t tx_data[] = {
        // Ethernet
        0x55,0x55,0x55,0x55 , 0x55,0x55,0x55,0x55,
        0x55,0x55,0x55,0x55 , 0x55,0x55,0x55,0xd5, // 802.3 preamble
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff,        // dest MAC
        0x00, 0x55, 0x00, 0x55, 0x00, 0x55,        // src  MAC
        0x08, 0x00, // IPv4
        // wireshark dump, IP
        0x45,0x00,0x00,0x54,0x8e,0x0f,0x40,0x00,0x40,0x01,0x2a,0x88,0xc0,0xa8,0x00,0x11,
        0xc0,0xa8,0x00,0xe0,
        // fixed ICMP ping
        0x08,0x00,0xd7,0x4d,0x00,0x0b,0x00,0x01,0xbc,0x4f,0xfd,0x62,0x00,0x00,0x00,0x00,
        0x9c,0x20,0x0c,0x00,0x00,0x00,0x00,0x00,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,
        0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,
        0x28,0x29,0x2a,0x2b,0x2c,0x2d,0x2e,0x2f,0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,
    };

    int i2s_size = tx_size*8;
    uint8_t *i2s_data = (uint8_t*)malloc(i2s_size);
    for (int i =0 ; i<i2s_size;i++) {
        uint8_t bit = (tx_data[i/8]&(1<<(i%8)))!=0;
        *(i2s_data + i) = bit ? 0b00001111 : 0b11110000;
    }

    size_t i2s_bytes_written;
    xTaskCreatePinnedToCore(dbgTask, "dbgTask", 10000, NULL, 1, NULL, 1);
    esp_task_wdt_delete(NULL); // make the dog go away!
    while(1){
        i2s_write(i2s_num, i2s_data, i2s_size,&i2s_bytes_written,100);
        // Serial.print("wrote ");
        // Serial.println(i2s_bytes_written);
    }

}

void dbgTask(void *unused) {
    while(1) {
        Serial.println(analogReadMilliVolts(rx_pin));
    }
}

void loop() {
}

// void _cycle_specific_attempt() {
//     // METHOD                            CYCLES @240MHz
//     // digitalWrite(26, 1);            // 79.57
//     // dacWrite(26,1);                 // 4346.69
//     // gpio_set_level(GPIO_NUM_26, 1); // 68.38
//     // [the following]                 // ~22

//     uint32_t *OUT_REG_LSB32 = (uint32_t*)GPIO_OUT_REG;
//     #define NO_OPTIMIZE(var) __asm__ volatile("" : "+g" (var) : :);
//     #define SET_HIGH(pin) *OUT_REG_LSB32|=1<<tx_pin;   NO_OPTIMIZE(*OUT_REG_LSB32);
//     #define SET_LOW(pin) *OUT_REG_LSB32&=~(1<<tx_pin); NO_OPTIMIZE(*OUT_REG_LSB32);
//     portDISABLE_INTERRUPTS();
//     esp_task_wdt_delete(NULL); // not interested in RTOS
//     SET_HIGH(tx_pin);
//     while (1) {
//         // *OUT_REG_LSB32<<=1;
//         __asm__ volatile("slli %0, %0, $1" : "=r"(OUT_REG_LSB32) :"0"(OUT_REG_LSB32) );
//         for (int i = 0; i < 10000000; i++)
//                 __asm__ volatile("" : "+g" (i) : :);
//         // digitalWrite(tx_pin, 0);
//         // *OUT_REG_LSB32>>=1;
//         __asm__ volatile("srli %0, %0, $1" : "=r"(OUT_REG_LSB32) :"0"(OUT_REG_LSB32) );
//         for (int i = 0; i < 10000000; i++)
//                 __asm__ volatile("" : "+g" (i) : :);
//     }
// }

// void _measure_cycles() {
//     portDISABLE_INTERRUPTS();
//         auto start = esp_cpu_get_ccount();
//         auto samples = 100000;
//         uint32_t *out_reg = (uint32_t*)GPIO_OUT_REG;
//         for (int i = 0 ; i < samples; i ++) {
//             __asm__ volatile("" : "+g" (i) : :);
//             SET_LOW(tx_pin);
//         }

//         auto end = esp_cpu_get_ccount();
//     portENABLE_INTERRUPTS();


//     float took = (float)(end) - (float)(start);
//     float per = (took / (float)samples)-5;
//     Serial.print("took ");
//     Serial.print(per);
//     Serial.println(" ticks");
// }
