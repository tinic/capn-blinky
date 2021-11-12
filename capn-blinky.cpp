
#include <stdint.h>
#include <memory.h>

#include "fpm/fixed.hpp"
#include "fpm/math.hpp"

#include "stm32l0xx_hal.h"

extern "C" SPI_HandleTypeDef hspi1;
extern "C" DMA_HandleTypeDef hdma_spi1_tx;

class Leds {
public:
    static constexpr size_t ledsN = 8;
    static constexpr size_t spiPaddingBytes = 64;

    static Leds &instance();

    void transfer();


private:

    static uint16_t led_buffer[ledsN * 3];
    static uint8_t spi_buffer[ledsN * sizeof(uint16_t) * 3 * 4 + spiPaddingBytes];

    void init();
    bool initialized = false;
};

uint8_t Leds::spi_buffer[ledsN * sizeof(uint16_t) * 3 * 4 + spiPaddingBytes];
uint16_t Leds::led_buffer[ledsN * 3];

Leds &Leds::instance() {
    static Leds leds;
    if (!leds.initialized) {
        leds.initialized = true;
        leds.init();
    }
    return leds;
}

void Leds::init() {
    memset(led_buffer, 0, sizeof(led_buffer));
}

void Leds::transfer() {
    uint32_t *ptr = reinterpret_cast<uint32_t *>(&spi_buffer[0]);
    for (size_t c = 0; c < spiPaddingBytes/(sizeof(uint32_t)*2); c++ ) {
        *ptr++ = 0;
    }

    for (size_t c = 0; c < ledsN; c++) {
        auto convert_to_one_wire_spi = [] (uint32_t *p, uint16_t v) {
            auto convert_half_to_spi = [] (uint8_t x) {
                return 0x88888888 | (((x >>  4) | (x <<  6) | (x << 16) | (x << 26)) & 0x04040404)|
                                    (((x >>  1) | (x <<  9) | (x << 19) | (x << 29)) & 0x40404040);
            };
            *p++ = convert_half_to_spi((v>>8)&0xFF);
            *p++ = convert_half_to_spi((v>>0)&0xFF);
            return p;
        };
        ptr = convert_to_one_wire_spi(ptr, led_buffer[c]);
    }

    for (size_t c = 0; c < spiPaddingBytes/(sizeof(uint32_t)*2); c++ ) {
        *ptr++ = 0;
    }

    HAL_SPI_DMAStop(&hspi1);

    HAL_SPI_Transmit_DMA(&hspi1, spi_buffer, sizeof(spi_buffer));
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *) {
    Leds::instance().transfer();
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t) {
}
