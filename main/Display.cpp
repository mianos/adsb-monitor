#include "Display.h"

#include <atomic>

#include "Board.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7796.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace display {
namespace {

constexpr char TAG[] = "display";
constexpr int kBufLines = 40;
constexpr ledc_channel_t kBlChannel = LEDC_CHANNEL_0;

esp_lcd_panel_handle_t s_panel = nullptr;
esp_lcd_touch_handle_t s_touch = nullptr;
lv_display_t* s_disp = nullptr;
SemaphoreHandle_t s_mutex = nullptr;
std::atomic<uint32_t> s_slowestUs{0};

bool onColorDone(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t*, void* ctx) {
  lv_display_flush_ready(static_cast<lv_display_t*>(ctx));
  return false;
}

void flush(lv_display_t* disp, const lv_area_t* area, uint8_t* px) {
  // LVGL renders little-endian RGB565; the panel takes it big-endian over SPI.
  lv_draw_sw_rgb565_swap(px, lv_area_get_size(area));
  esp_lcd_panel_draw_bitmap(s_panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, px);
}

void readTouch(lv_indev_t*, lv_indev_data_t* data) {
  esp_lcd_touch_point_data_t pt[1];
  uint8_t count = 0;
  if (esp_lcd_touch_read_data(s_touch) == ESP_OK &&
      esp_lcd_touch_get_data(s_touch, pt, &count, 1) == ESP_OK && count > 0) {
    // FT6336U reports in the panel's portrait frame; Freenove's landscape sketches
    // map it as x = raw y, y = 320 - raw x.
    data->point.x = pt[0].y;
    data->point.y = board::kPanelW - 1 - pt[0].x;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

uint32_t tickMs() { return esp_timer_get_time() / 1000; }

void lvglTask(void*) {
  while (true) {
    lock();
    const int64_t start = esp_timer_get_time();
    const uint32_t next = lv_timer_handler();
    const uint32_t took = esp_timer_get_time() - start;
    if (took > s_slowestUs) s_slowestUs = took;
    unlock();
    vTaskDelay(pdMS_TO_TICKS(next < 5 ? 5 : (next > 30 ? 30 : next)));
  }
}

void initBacklight() {
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = LEDC_TIMER_10_BIT;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = 5000;
  timer.clk_cfg = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&timer));
  ledc_channel_config_t ch = {};
  ch.gpio_num = board::kLcdBacklight;
  ch.speed_mode = LEDC_LOW_SPEED_MODE;
  ch.channel = kBlChannel;
  ch.timer_sel = LEDC_TIMER_0;
  ch.duty = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

void initPanel() {
  spi_bus_config_t bus = {};
  bus.mosi_io_num = board::kLcdMosi;
  bus.miso_io_num = -1;
  bus.sclk_io_num = board::kLcdSclk;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = board::kWidth * kBufLines * sizeof(uint16_t);
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

  esp_lcd_panel_io_spi_config_t io_cfg = {};
  io_cfg.cs_gpio_num = board::kLcdCs;
  io_cfg.dc_gpio_num = board::kLcdDc;
  io_cfg.spi_mode = 0;
  io_cfg.pclk_hz = board::kLcdSpiHz;
  io_cfg.trans_queue_depth = 10;
  io_cfg.lcd_cmd_bits = 8;
  io_cfg.lcd_param_bits = 8;
  esp_lcd_panel_io_handle_t io = nullptr;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(SPI2_HOST), &io_cfg, &io));

  esp_lcd_panel_dev_config_t panel_cfg = {};
  panel_cfg.reset_gpio_num = GPIO_NUM_NC;
  // The panel wires its sub-pixels blue-first; as RGB, orange came out blue.
  panel_cfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;
  panel_cfg.bits_per_pixel = 16;
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io, &panel_cfg, &s_panel));
  ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
  ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
  // Landscape, matching TFT_eSPI rotation 1 on this panel (MADCTL MV only).
  ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, true));
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, false, false));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

  s_disp = lv_display_create(board::kWidth, board::kHeight);
  const size_t bytes = board::kWidth * kBufLines * sizeof(uint16_t);
  void* buf1 = heap_caps_malloc(bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  void* buf2 = heap_caps_malloc(bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  assert(buf1 && buf2);
  lv_display_set_color_format(s_disp, LV_COLOR_FORMAT_RGB565);
  lv_display_set_buffers(s_disp, buf1, buf2, bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(s_disp, flush);

  const esp_lcd_panel_io_callbacks_t cbs = {.on_color_trans_done = onColorDone};
  ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io, &cbs, s_disp));
}

void initTouch() {
  i2c_master_bus_config_t bus = {};
  bus.i2c_port = I2C_NUM_0;
  bus.sda_io_num = board::kTouchSda;
  bus.scl_io_num = board::kTouchScl;
  bus.clk_source = I2C_CLK_SRC_DEFAULT;
  bus.glitch_ignore_cnt = 7;
  bus.flags.enable_internal_pullup = true;
  i2c_master_bus_handle_t i2c = nullptr;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus, &i2c));

  esp_lcd_panel_io_i2c_config_t io_cfg = {};
  io_cfg.dev_addr = ESP_LCD_TOUCH_IO_I2C_FT5x06_ADDRESS;
  io_cfg.scl_speed_hz = 400000;
  io_cfg.control_phase_bytes = 1;
  io_cfg.lcd_cmd_bits = 8;
  io_cfg.flags.disable_control_phase = 1;
  esp_lcd_panel_io_handle_t io = nullptr;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c, &io_cfg, &io));

  esp_lcd_touch_config_t cfg = {};
  cfg.x_max = board::kPanelW;
  cfg.y_max = board::kPanelH;
  cfg.rst_gpio_num = board::kTouchRst;
  cfg.int_gpio_num = GPIO_NUM_NC;  // polled; INT is board::kTouchInt if ever needed
  // A missing or unresponsive touch controller shouldn't stop the radar.
  esp_err_t err = esp_lcd_touch_new_i2c_ft5x06(io, &cfg, &s_touch);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "touch init failed: %s", esp_err_to_name(err));
    s_touch = nullptr;
    return;
  }
  lv_indev_t* indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_display(indev, s_disp);
  lv_indev_set_read_cb(indev, readTouch);
}

}  // namespace

void init() {
  s_mutex = xSemaphoreCreateRecursiveMutex();
  initBacklight();
  lv_init();
  lv_tick_set_cb(tickMs);
  initPanel();
  initTouch();
  // Pinned off core 0, which runs Wi-Fi and the TLS handshakes.
  xTaskCreatePinnedToCore(lvglTask, "lvgl", 8192, nullptr, 4, nullptr, 1);
  ESP_LOGI(TAG, "ST7796 %dx%d up, touch %s", board::kWidth, board::kHeight, s_touch ? "ok" : "missing");
}

void setBrightness(int percent) {
  if (percent < 1) percent = 1;
  if (percent > 100) percent = 100;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, kBlChannel, 1023 * percent / 100);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, kBlChannel);
}

bool lock(uint32_t timeoutMs) {
  const TickType_t ticks = timeoutMs == UINT32_MAX ? portMAX_DELAY : pdMS_TO_TICKS(timeoutMs);
  return xSemaphoreTakeRecursive(s_mutex, ticks) == pdTRUE;
}

void unlock() { xSemaphoreGiveRecursive(s_mutex); }

uint32_t takeSlowestMs() { return (s_slowestUs.exchange(0) + 500) / 1000; }

namespace {
struct SnapshotJob {
  lv_draw_buf_t* buf;
  SemaphoreHandle_t done;
  bool ok = false;
};
}  // namespace

bool snapshot(lv_draw_buf_t* buf) {
  SnapshotJob job{buf, xSemaphoreCreateBinary()};
  if (!job.done) return false;
  {
    Guard guard;
    lv_async_call(
        [](void* arg) {
          auto* j = static_cast<SnapshotJob*>(arg);
          j->ok = lv_snapshot_take_to_draw_buf(lv_screen_active(), LV_COLOR_FORMAT_RGB565, j->buf) == LV_RESULT_OK;
          xSemaphoreGive(j->done);
        },
        &job);
  }
  xSemaphoreTake(job.done, portMAX_DELAY);  // the LVGL task always runs the call
  vSemaphoreDelete(job.done);
  return job.ok;
}

}  // namespace display
