#pragma once
#include "driver/gpio.h"

// Freenove FNK0104S (ESP32-S3R8, 4.0" ST7796 IPS + FT6336U). Pins from Freenove's
// TFT_eSPI setup FNK0104S_4.0_320x480_ST7796.h and its Tutorial_With_Touch sketches.
namespace board {

constexpr gpio_num_t kLcdMosi = GPIO_NUM_11;
constexpr gpio_num_t kLcdSclk = GPIO_NUM_12;
constexpr gpio_num_t kLcdCs = GPIO_NUM_10;
constexpr gpio_num_t kLcdDc = GPIO_NUM_46;
constexpr gpio_num_t kLcdBacklight = GPIO_NUM_45;
constexpr int kLcdSpiHz = 80 * 1000 * 1000;

constexpr gpio_num_t kTouchSda = GPIO_NUM_16;
constexpr gpio_num_t kTouchScl = GPIO_NUM_15;
constexpr gpio_num_t kTouchRst = GPIO_NUM_18;
constexpr gpio_num_t kTouchInt = GPIO_NUM_17;

constexpr gpio_num_t kBootKey = GPIO_NUM_0;

// Panel is 320x480 portrait; the UI runs landscape.
constexpr int kPanelW = 320;
constexpr int kPanelH = 480;
constexpr int kWidth = 480;
constexpr int kHeight = 320;

}  // namespace board
