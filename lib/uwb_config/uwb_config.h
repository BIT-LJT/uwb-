#ifndef UWB_CONFIG_H
#define UWB_CONFIG_H
#include <Arduino.h>

typedef struct {
    float x;
    float y;
    float z;
} Position;

static constexpr uint8_t MAX_ANCHOR_NUM = 8;

#ifdef CONFIG_IDF_TARGET_ESP32C3
    static constexpr uint8_t mode_select_pin     = 12;
    static constexpr uint8_t index_select_pins[] = {13, 5, 4};
    static constexpr uint8_t SPI_SCK  = 2;
    static constexpr uint8_t SPI_MISO = 10;
    static constexpr uint8_t SPI_MOSI = 3;
    static constexpr uint8_t PIN_RST  = 8;
    static constexpr uint8_t PIN_IRQ  = 6;
    static constexpr uint8_t PIN_SS   = 7;
#endif
#ifdef CONFIG_IDF_TARGET_ESP32S3
    static constexpr uint8_t mode_select_pin     = 12;
    static constexpr uint8_t index_select_pins[] = {13, 5, 4};
    static constexpr uint8_t SPI_SCK  = 2;
    static constexpr uint8_t SPI_MISO = 10;
    static constexpr uint8_t SPI_MOSI = 3;
    static constexpr uint8_t PIN_RST  = 8;
    static constexpr uint8_t PIN_IRQ  = 6;
    static constexpr uint8_t PIN_SS   = 7;
#endif


#endif
