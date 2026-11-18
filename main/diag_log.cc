#include "diag_log.h"

#include <cJSON.h>
#include <esp_event.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <nvs.h>
#include <sys/time.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>

#define TAG "DiagLog"

namespace {

constexpr int kMaxEntries = 16;
constexpr int kEntryLen = 100;
constexpr const char* kNs = "diag";

struct Ring {
    uint32_t boot_count;
    uint32_t head;      // next write index (monotonic)
    uint32_t uploaded;  // number of entries (by monotonic index) already uploaded
    char entries[kMaxEntries][kEntryLen];
};

Ring g_ring;
SemaphoreHandle_t g_mutex = nullptr;
bool g_inited = false;
uint32_t g_pending_upload_end = 0;
int g_last_reason = -1;
int64_t g_last_disc_us = 0;
int g_suppressed = 0;

const char* ResetReasonName(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON: return "poweron";
        case ESP_RST_EXT: return "ext";
        case ESP_RST_SW: return "sw";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "int_wdt";
        case ESP_RST_TASK_WDT: return "task_wdt";
        case ESP_RST_WDT: return "wdt";
        case ESP_RST_DEEPSLEEP: return "deepsleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "sdio";
        default: return "unknown";
    }
}

// NVS is only 16KB and shared, so store one small string per entry (not one big blob).
void SaveMeta(nvs_handle_t h) {
    nvs_set_u32(h, "head", g_ring.head);
    nvs_set_u32(h, "boot", g_ring.boot_count);
    nvs_set_u32(h, "upl", g_ring.uploaded);
}

void SaveEntry(uint32_t idx) {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return;
    char key[8];
    snprintf(key, sizeof(key), "e%u", (unsigned)(idx % kMaxEntries));
    esp_err_t e = nvs_set_str(h, key, g_ring.entries[idx % kMaxEntries]);
    SaveMeta(h);
    if (e == ESP_OK) e = nvs_commit(h);
    if (e != ESP_OK) ESP_LOGE(TAG, "NVS save failed: 0x%x", (unsigned)e);
    nvs_close(h);
}

void SaveMetaOnly() {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return;
    SaveMeta(h);
    nvs_commit(h);
    nvs_close(h);
}

void Load() {
    memset(&g_ring, 0, sizeof(g_ring));
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READONLY, &h) != ESP_OK) return;
    nvs_get_u32(h, "head", &g_ring.head);
    nvs_get_u32(h, "boot", &g_ring.boot_count);
    nvs_get_u32(h, "upl", &g_ring.uploaded);
    for (int i = 0; i < kMaxEntries; i++) {
        char key[8];
        snprintf(key, sizeof(key), "e%d", i);
        size_t len = kEntryLen;
        nvs_get_str(h, key, g_ring.entries[i], &len);
    }
    nvs_close(h);
}

void OnWifiEvent(void*, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        auto* e = static_cast<wifi_event_sta_disconnected_t*>(data);
        int64_t now = esp_timer_get_time();
        // Reconnect attempts fire this repeatedly; coalesce identical reasons.
        if (e->reason == g_last_reason && now - g_last_disc_us < 60 * 1000000LL) {
            g_suppressed++;
            return;
        }
        if (g_suppressed > 0) {
            DiagLog::Event("wifi: %d repeated disconnects suppressed", g_suppressed);
            g_suppressed = 0;
        }
        g_last_reason = e->reason;
        g_last_disc_us = now;
        DiagLog::Event("wifi: DISCONNECTED reason=%d rssi=%d ssid=%.*s", e->reason, e->rssi,
                       e->ssid_len, (const char*)e->ssid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        auto* e = static_cast<wifi_event_sta_connected_t*>(data);
        DiagLog::Event("wifi: connected ch=%d ssid=%.*s", e->channel, e->ssid_len,
                       (const char*)e->ssid);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto* e = static_cast<ip_event_got_ip_t*>(data);
        g_last_reason = -1;
        g_suppressed = 0;
        DiagLog::Event("ip: got " IPSTR " gw " IPSTR, IP2STR(&e->ip_info.ip),
                       IP2STR(&e->ip_info.gw));
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        DiagLog::Event("ip: LOST ip");
    }
}

}  // namespace

namespace DiagLog {

void Init() {
    if (g_inited) return;
    g_mutex = xSemaphoreCreateMutex();
    {   // drop the old single-blob format; it filled the 16KB NVS partition
        nvs_handle_t h;
        if (nvs_open(kNs, NVS_READWRITE, &h) == ESP_OK) {
            if (nvs_erase_key(h, "ring") == ESP_OK) nvs_commit(h);
            nvs_close(h);
        }
    }
    Load();
    g_ring.boot_count++;
    g_inited = true;

    auto reason = esp_reset_reason();
    Event("boot #%lu reset=%s", (unsigned long)g_ring.boot_count, ResetReasonName(reason));
    Dump();

    // Handlers run on the default event loop, which the Wi-Fi stack creates/uses.
    esp_event_loop_create_default();  // returns ESP_ERR_INVALID_STATE if it already exists
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, OnWifiEvent, nullptr);
    esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, OnWifiEvent, nullptr);
}

void Event(const char* fmt, ...) {
    if (!g_inited) return;
    char msg[48];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    wifi_ap_record_t ap_info;
    int rssi = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) ? ap_info.rssi : 0;
    time_t t = time(nullptr);
    char line[kEntryLen];
    snprintf(line, sizeof(line), "t=%lld up=%llds h=%u r=%d %s",
             t > 1600000000 ? (long long)t : 0LL, (long long)(esp_timer_get_time() / 1000000),
             (unsigned)esp_get_free_heap_size(), rssi, msg);
    ESP_LOGW(TAG, "%s", line);

    xSemaphoreTake(g_mutex, portMAX_DELAY);
    strlcpy(g_ring.entries[g_ring.head % kMaxEntries], line, kEntryLen);
    uint32_t idx = g_ring.head++;
    SaveEntry(idx);
    xSemaphoreGive(g_mutex);
}

std::string BuildUploadJson() {
    if (!g_inited) return "";
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    uint32_t start = g_ring.uploaded;
    if (g_ring.head > kMaxEntries && start < g_ring.head - kMaxEntries) start = g_ring.head - kMaxEntries;
    if (start >= g_ring.head) {
        xSemaphoreGive(g_mutex);
        return "";
    }
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "boardlog");
    cJSON_AddNumberToObject(root, "boot", g_ring.boot_count);
    cJSON_AddNumberToObject(root, "uptime_s", (double)(esp_timer_get_time() / 1000000));
    cJSON_AddStringToObject(root, "reset", ResetReasonName(esp_reset_reason()));
    cJSON* arr = cJSON_AddArrayToObject(root, "events");
    for (uint32_t i = start; i < g_ring.head; i++) {
        cJSON_AddItemToArray(arr, cJSON_CreateString(g_ring.entries[i % kMaxEntries]));
    }
    g_pending_upload_end = g_ring.head;
    xSemaphoreGive(g_mutex);
    char* s = cJSON_PrintUnformatted(root);
    std::string out(s);
    cJSON_free(s);
    cJSON_Delete(root);
    return out;
}

void MarkUploaded() {
    if (!g_inited) return;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_ring.uploaded = g_pending_upload_end;
    SaveMetaOnly();
    xSemaphoreGive(g_mutex);
}

void Dump() {
    if (!g_inited) return;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    uint32_t start = g_ring.head > kMaxEntries ? g_ring.head - kMaxEntries : 0;
    printf("==== DIAG LOG (boot #%lu, %lu entries) ====\n", (unsigned long)g_ring.boot_count,
           (unsigned long)(g_ring.head - start));
    for (uint32_t i = start; i < g_ring.head; i++) {
        printf("[diag %lu] %s\n", (unsigned long)i, g_ring.entries[i % kMaxEntries]);
    }
    printf("==== END DIAG LOG ====\n");
    xSemaphoreGive(g_mutex);
}

}  // namespace DiagLog
