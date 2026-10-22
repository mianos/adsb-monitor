#pragma once
#include <string>

// Tees ESP_LOG output into RAM so GET /log works without serial. The tail is also
// kept in RTC memory that survives a reset, so the log leading up to a crash can
// be read after the reboot (GET /log?prev).
namespace LogBuffer {
void install();
std::string snapshot();
std::string previousBoot();
}  // namespace LogBuffer
