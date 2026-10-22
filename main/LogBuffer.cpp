#include "LogBuffer.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

namespace {

constexpr size_t kSize = 8 * 1024;
char s_ring[kSize];
size_t s_head = 0;  // next write position
bool s_wrapped = false;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
vprintf_like_t s_uart = nullptr;

// RTC slow memory keeps its contents across a software or panic reset.
constexpr uint32_t kRtcMagic = 0x4C4F4731;  // "LOG1"
constexpr size_t kRtcSize = 3 * 1024;
struct RtcTail {
    uint32_t magic;
    uint32_t head;
    uint32_t wrapped;
    char ring[kRtcSize];
};
RTC_NOINIT_ATTR RtcTail s_rtc;
std::string s_previous;

void append(char* ring, size_t size, size_t& head, bool& wrapped, const char* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        ring[head] = data[i];
        if (++head == size) {
            head = 0;
            wrapped = true;
        }
    }
}

int teeVprintf(const char* fmt, va_list args) {
    char line[256];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof line, fmt, copy);
    va_end(copy);
    if (n > 0) {
        size_t len = std::min<size_t>(n, sizeof line - 1);
        portENTER_CRITICAL(&s_mux);
        append(s_ring, kSize, s_head, s_wrapped, line, len);
        size_t rtcHead = s_rtc.head;
        bool rtcWrapped = s_rtc.wrapped;
        append(s_rtc.ring, kRtcSize, rtcHead, rtcWrapped, line, len);
        s_rtc.head = rtcHead;
        s_rtc.wrapped = rtcWrapped;
        portEXIT_CRITICAL(&s_mux);
    }
    return s_uart(fmt, args);
}

}  // namespace

void LogBuffer::install() {
    // Power-on leaves RTC memory random; only trust it behind the magic.
    if (s_rtc.magic == kRtcMagic && s_rtc.head < kRtcSize) {
        if (s_rtc.wrapped) s_previous.append(s_rtc.ring + s_rtc.head, kRtcSize - s_rtc.head);
        s_previous.append(s_rtc.ring, s_rtc.head);
    }
    s_rtc.magic = kRtcMagic;
    s_rtc.head = 0;
    s_rtc.wrapped = 0;
    s_uart = esp_log_set_vprintf(teeVprintf);
}

std::string LogBuffer::previousBoot() {
    return s_previous;
}

std::string LogBuffer::snapshot() {
    std::string out;
    out.reserve(kSize);
    portENTER_CRITICAL(&s_mux);
    if (s_wrapped) out.append(s_ring + s_head, kSize - s_head);
    out.append(s_ring, s_head);
    portEXIT_CRITICAL(&s_mux);
    return out;
}
