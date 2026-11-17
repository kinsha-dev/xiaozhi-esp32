#ifndef DIAG_LOG_H
#define DIAG_LOG_H

#include <string>

// Persistent ring buffer of connectivity events (NVS-backed, survives reboot).
// Uploaded to the server as a "boardlog" text message after the next hello.
namespace DiagLog {

// Call once after nvs_flash_init(). Records the reset reason, registers Wi-Fi/IP
// event handlers and dumps the stored log to serial.
void Init();

// printf-style event; prefixes uptime, wall clock (if SNTP is set), heap and RSSI.
void Event(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Builds the {"type":"boardlog",...} JSON. Empty string when nothing is pending.
std::string BuildUploadJson();

// Drop entries that were included in the last BuildUploadJson().
void MarkUploaded();

// Print every stored entry to serial.
void Dump();

}  // namespace DiagLog

#endif
