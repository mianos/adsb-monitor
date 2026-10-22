#pragma once
#include <cstdint>

#include "lvgl.h"

// ST7796 panel, FT6336U touch and the LVGL glue (tick, flush, pointer input, and
// the task that runs lv_timer_handler). Other tasks must hold lock() while they
// touch LVGL objects.
namespace display {

void init();
void setBrightness(int percent);  // 1-100
bool lock(uint32_t timeoutMs = UINT32_MAX);
void unlock();
// Renders the active screen into buf (RGB565, screen-sized) on the LVGL task,
// whose stack is deep enough for software rendering. Don't hold lock().
bool snapshot(lv_draw_buf_t* buf);

class Guard {
 public:
  Guard() { lock(); }
  ~Guard() { unlock(); }
  Guard(const Guard&) = delete;
  Guard& operator=(const Guard&) = delete;
};

}  // namespace display
