#include "ble_protocol.h"

#include "application.h"
#include "assets/lang_config.h"
#include "board.h"
#include "settings.h"
#include "system_info.h"

#include <cJSON.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <wifi_manager.h>

#include <algorithm>
#include <cstring>

#if CONFIG_BT_NIMBLE_ENABLED

#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/ble_uuid.h"
#include "host/ble_hs_mbuf.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

extern "C" void ble_store_config_init(void);

#endif

static const char* TAG = "BLE";

#if CONFIG_BT_NIMBLE_ENABLED

static const ble_uuid128_t kSvcUuid =
    BLE_UUID128_INIT(0x10, 0x2a, 0x3b, 0x4c, 0x5d, 0x6e, 0x1a, 0x9f, 0x4a, 0x4b, 0x1e, 0x3c, 0x01, 0x00,
                     0x2a, 0x7d);
static const ble_uuid128_t kTxUuid =
    BLE_UUID128_INIT(0x10, 0x2a, 0x3b, 0x4c, 0x5d, 0x6e, 0x1a, 0x9f, 0x4a, 0x4b, 0x1e, 0x3c, 0x02, 0x00,
                     0x2a, 0x7d);
static const ble_uuid128_t kRxUuid =
    BLE_UUID128_INIT(0x10, 0x2a, 0x3b, 0x4c, 0x5d, 0x6e, 0x1a, 0x9f, 0x4a, 0x4b, 0x1e, 0x3c, 0x03, 0x00,
                     0x2a, 0x7d);
static const ble_uuid128_t kHeartUuid =
    BLE_UUID128_INIT(0x10, 0x2a, 0x3b, 0x4c, 0x5d, 0x6e, 0x1a, 0x9f, 0x4a, 0x4b, 0x1e, 0x3c, 0x04, 0x00,
                     0x2a, 0x7d);

static uint16_t s_tx_val = 0;
static uint16_t s_heart_val = 0;
static BleProtocol* s_proto = nullptr;

static int OnAccess(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt* ctxt,
                    void* arg) {
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR || s_proto == nullptr) {
        return 0;
    }
    uint8_t buf[512];
    uint16_t len = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    s_proto->OnWrite(buf, len);
    return 0;
}

static const struct ble_gatt_chr_def kChrs[] = {
    {.uuid = &kTxUuid.u,
     .access_cb = OnAccess,
     .arg = nullptr,
     .descriptors = nullptr,
     .flags = BLE_GATT_CHR_F_NOTIFY,
     .min_key_size = 0,
     .val_handle = &s_tx_val,
     .cpfd = nullptr},
    {.uuid = &kRxUuid.u,
     .access_cb = OnAccess,
     .arg = nullptr,
     .descriptors = nullptr,
     .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
     .min_key_size = 0,
     .val_handle = nullptr,
     .cpfd = nullptr},
    {.uuid = &kHeartUuid.u,
     .access_cb = OnAccess,
     .arg = nullptr,
     .descriptors = nullptr,
     .flags = BLE_GATT_CHR_F_NOTIFY,
     .min_key_size = 0,
     .val_handle = &s_heart_val,
     .cpfd = nullptr},
    {},
};

static const struct ble_gatt_svc_def kSvcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &kSvcUuid.u,
     .includes = nullptr,
     .characteristics = kChrs},
    {},
};

static int GapEvent(struct ble_gap_event* event, void* arg) {
    (void)arg;
    if (s_proto != nullptr) {
        s_proto->OnGapEvent(event);
    }
    return 0;
}

static void OnReset(int reason) { ESP_LOGW(TAG, "nimble reset reason=%d", reason); }

static void OnSync() {
    if (s_proto != nullptr) {
        s_proto->OnStackSync();
    }
}

static void HostTask(void* param) {
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
    if (s_proto != nullptr) {
        s_proto->OnHostStopped();
    }
}

#endif

static bool TokenMatch(const std::string& a, const std::string& b) {
    const size_t n = std::max(a.size(), b.size());
    unsigned diff = static_cast<unsigned>(a.size() ^ b.size());
    for (size_t i = 0; i < n; ++i) {
        const unsigned ca = i < a.size() ? static_cast<unsigned char>(a[i]) : 0;
        const unsigned cb = i < b.size() ? static_cast<unsigned char>(b[i]) : 0;
        diff |= ca ^ cb;
    }
    return diff == 0;
}

BleProtocol::BleProtocol() {
    hello_event_ = xEventGroupCreate();
    ack_queue_ = xQueueCreate(8, sizeof(uint8_t));
    sync_sem_ = xSemaphoreCreateBinary();
    host_done_ = xSemaphoreCreateBinary();
}

BleProtocol::~BleProtocol() {
    CloseAudioChannel(false);
    StopStack();
    if (hello_event_ != nullptr) {
        vEventGroupDelete(hello_event_);
    }
    if (ack_queue_ != nullptr) {
        vQueueDelete(ack_queue_);
    }
    if (sync_sem_ != nullptr) {
        vSemaphoreDelete(sync_sem_);
    }
    if (host_done_ != nullptr) {
        vSemaphoreDelete(host_done_);
    }
}

bool BleProtocol::Start() {
#if !CONFIG_BT_NIMBLE_ENABLED
    ESP_LOGE(TAG, "NimBLE is not enabled in this build");
    return false;
#else
    if (stack_up_) {
        return true;
    }
    Settings settings("websocket", false);
    token_ = settings.GetString("token");
    int version = settings.GetInt("version");
    if (version != 0) {
        version_ = version;
    }
    stopping_.store(false);
    s_proto = this;
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        s_proto = nullptr;
        return false;
    }
    ble_svc_gap_init();
    ble_svc_gap_device_name_set("Vibi");
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(kSvcs);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(kSvcs);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "gatt init failed: %d", rc);
        nimble_port_deinit();
        s_proto = nullptr;
        return false;
    }
    ble_hs_cfg.reset_cb = OnReset;
    ble_hs_cfg.sync_cb = OnSync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
    nimble_port_freertos_init(HostTask);
    if (xSemaphoreTake(sync_sem_, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "nimble sync timed out");
        StopStack();
        return false;
    }
    const esp_timer_create_args_t args = {
        .callback = &BleProtocol::TickThunk,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ble_tick",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&args, &tick_timer_);
    esp_timer_start_periodic(tick_timer_, 1000000);
    const esp_timer_create_args_t ack_args = {
        .callback = &BleProtocol::AckThunk,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ble_ack",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&ack_args, &ack_timer_);
    stack_up_ = true;
    SystemInfo::PrintHeapStats();
    ESP_LOGI(TAG, "BLE fallback advertising");
    return true;
#endif
}

void BleProtocol::StopStack() {
#if CONFIG_BT_NIMBLE_ENABLED
    stopping_.store(true);
    if (tick_timer_ != nullptr) {
        esp_timer_stop(tick_timer_);
        esp_timer_delete(tick_timer_);
        tick_timer_ = nullptr;
    }
    if (ack_timer_ != nullptr) {
        esp_timer_stop(ack_timer_);
        esp_timer_delete(ack_timer_);
        ack_timer_ = nullptr;
    }
    if (!stack_up_ && s_proto != this) {
        return;
    }
    if (conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
        conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
    }
    ble_gap_adv_stop();
    if (s_proto == this) {
        int rc = nimble_port_stop();
        if (rc == 0) {
            xSemaphoreTake(host_done_, pdMS_TO_TICKS(5000));
        }
        nimble_port_deinit();
        s_proto = nullptr;
    }
    stack_up_ = false;
    authed_ = false;
#else
    stack_up_ = false;
#endif
}

bool BleProtocol::IsAudioChannelOpened() const {
    return channel_open_ && authed_ && conn_handle_ != 0xffff && !error_occurred_ && !IsTimeout();
}

bool BleProtocol::IsTimeout() const {
    const int64_t last = last_rx_us_.load();
    if (last == 0) {
        return false;
    }
    return (esp_timer_get_time() - last) > 120LL * 1000000LL;
}

bool BleProtocol::Notify(uint16_t attr, const uint8_t* data, size_t len) {
#if !CONFIG_BT_NIMBLE_ENABLED
    (void)attr;
    (void)data;
    (void)len;
    return false;
#else
    if (conn_handle_ == BLE_HS_CONN_HANDLE_NONE || attr == 0 || len > 512) {
        return false;
    }
    struct os_mbuf* om = ble_hs_mbuf_from_flat(data, static_cast<uint16_t>(len));
    if (om == nullptr) {
        return false;
    }
    int rc = ble_gatts_notify_custom(conn_handle_, attr, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "notify failed: %d", rc);
        return false;
    }
    return true;
#endif
}

bool BleProtocol::SendTyped(uint8_t type, const uint8_t* data, size_t len, bool reliable) {
    if (conn_handle_ == 0xffff) {
        return false;
    }
    const uint8_t seq = seq_.fetch_add(1);
    const int tries = reliable ? 3 : 1;
    for (int attempt = 0; attempt < tries; ++attempt) {
        std::vector<std::vector<uint8_t>> frags;
        if (!BleFragment(seq, type, data, len, mtu_payload_.load(), &frags)) {
            return false;
        }
        bool sent = true;
        for (const auto& frag : frags) {
            if (!Notify(tx_handle_, frag.data(), frag.size())) {
                sent = false;
                break;
            }
        }
        if (!reliable) {
            return sent;
        }
        if (!sent) {
            continue;
        }
        const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
        while (static_cast<int32_t>(deadline - xTaskGetTickCount()) > 0) {
            uint8_t got = 0;
            if (xQueueReceive(ack_queue_, &got, pdMS_TO_TICKS(50)) == pdTRUE && got == seq) {
                return true;
            }
        }
    }
    return false;
}

bool BleProtocol::SendText(const std::string& text) {
    return SendTyped(kBleTypeJson, reinterpret_cast<const uint8_t*>(text.data()), text.size(), true);
}

bool BleProtocol::SendAudio(std::unique_ptr<AudioStreamPacket> packet) {
    if (!channel_open_ || packet == nullptr) {
        return false;
    }
    // One dropped frame must not flush the rest of the send queue.
    SendTyped(kBleTypeAudio, packet->payload.data(), packet->payload.size(), false);
    return true;
}

void BleProtocol::QueueAck(uint8_t acked_seq) {
    pending_ack_.store(acked_seq);
    ack_pending_.store(true);
    if (ack_timer_ != nullptr) {
        esp_timer_stop(ack_timer_);
        esp_timer_start_once(ack_timer_, 1000);
    }
}

void BleProtocol::FireAck() {
    if (!ack_pending_.exchange(false)) {
        return;
    }
    SendAck(pending_ack_.load());
}

void BleProtocol::SendAck(uint8_t acked_seq) {
    const uint8_t seq = seq_.fetch_add(1);
    std::vector<std::vector<uint8_t>> frags;
    if (!BleFragment(seq, kBleTypeAck, &acked_seq, 1, mtu_payload_.load(), &frags)) {
        return;
    }
    for (const auto& frag : frags) {
        Notify(tx_handle_, frag.data(), frag.size());
    }
}

void BleProtocol::OnWrite(const uint8_t* data, size_t len) {
    BleMessage msg;
    bool duplicate = false;
    if (!reassembler_.Push(data, len, &msg, &duplicate)) {
        return;
    }
    last_rx_us_.store(esp_timer_get_time());
    if (msg.type == kBleTypeAck) {
        if (!msg.payload.empty()) {
            const uint8_t seq = msg.payload[0];
            xQueueSend(ack_queue_, &seq, 0);
        }
        return;
    }
    if (msg.type == kBleTypeJson) {
        const std::string text(reinterpret_cast<const char*>(msg.payload.data()), msg.payload.size());
        if (!authed_) {
            cJSON* root = cJSON_ParseWithLength(text.data(), text.size());
            auto type = root != nullptr ? cJSON_GetObjectItem(root, "type") : nullptr;
            auto token = root != nullptr ? cJSON_GetObjectItem(root, "token") : nullptr;
            const bool ok = cJSON_IsString(type) && std::strcmp(type->valuestring, "auth") == 0 &&
                            cJSON_IsString(token) && TokenMatch(token->valuestring, token_);
            if (root != nullptr) {
                cJSON_Delete(root);
            }
            if (!ok) {
                ESP_LOGW(TAG, "BLE auth rejected");
#if CONFIG_BT_NIMBLE_ENABLED
                ble_gap_terminate(conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
#endif
                return;
            }
            authed_ = true;
            QueueAck(reassembler_.last_completed_seq());
            ESP_LOGI(TAG, "BLE central accepted");
            return;
        }
        QueueAck(reassembler_.last_completed_seq());
        if (duplicate) {
            return;
        }
        cJSON* root = cJSON_ParseWithLength(text.data(), text.size());
        auto type = root != nullptr ? cJSON_GetObjectItem(root, "type") : nullptr;
        const bool hello = cJSON_IsString(type) && std::strcmp(type->valuestring, "hello") == 0;
        if (hello) {
            ParseServerHello(root);
            if (root != nullptr) {
                cJSON_Delete(root);
            }
            return;
        }
        if (root != nullptr) {
            cJSON_Delete(root);
        }
        Application::GetInstance().Schedule([this, text]() {
            if (stopping_.load()) {
                return;
            }
            HandleJson(text);
        });
        return;
    }
    if (msg.type == kBleTypeAudio && channel_open_ && !duplicate && on_incoming_audio_ != nullptr) {
        auto packet = std::make_unique<AudioStreamPacket>();
        packet->sample_rate = server_sample_rate_;
        packet->frame_duration = server_frame_duration_;
        packet->payload = std::move(msg.payload);
        on_incoming_audio_(std::move(packet));
    }
}

void BleProtocol::HandleJson(const std::string& text) {
    cJSON* root = cJSON_ParseWithLength(text.data(), text.size());
    if (root == nullptr) {
        return;
    }
    auto type = cJSON_GetObjectItem(root, "type");
    if (cJSON_IsString(type) && std::strcmp(type->valuestring, "hello") == 0) {
        ParseServerHello(root);
    } else if (on_incoming_json_ != nullptr) {
        on_incoming_json_(root);
    }
    cJSON_Delete(root);
}

void BleProtocol::ParseServerHello(const cJSON* root) {
    if (root == nullptr) {
        return;
    }
    auto transport = cJSON_GetObjectItem(root, "transport");
    if (!cJSON_IsString(transport) || std::strcmp(transport->valuestring, "websocket") != 0) {
        ESP_LOGE(TAG, "server hello missing websocket transport");
        return;
    }
    auto session_id = cJSON_GetObjectItem(root, "session_id");
    if (cJSON_IsString(session_id)) {
        session_id_ = session_id->valuestring;
    }
    auto audio_params = cJSON_GetObjectItem(root, "audio_params");
    if (cJSON_IsObject(audio_params)) {
        auto sample_rate = cJSON_GetObjectItem(audio_params, "sample_rate");
        if (cJSON_IsNumber(sample_rate)) {
            server_sample_rate_ = sample_rate->valueint;
        }
        auto frame_duration = cJSON_GetObjectItem(audio_params, "frame_duration");
        if (cJSON_IsNumber(frame_duration)) {
            server_frame_duration_ = frame_duration->valueint;
        }
    }
    xEventGroupSetBits(hello_event_, kHelloBit);
}

std::string BleProtocol::HelloMessage() {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "hello");
    cJSON_AddNumberToObject(root, "version", version_);
    cJSON* features = cJSON_CreateObject();
#if CONFIG_USE_SERVER_AEC
    cJSON_AddBoolToObject(features, "aec", true);
#endif
    cJSON_AddBoolToObject(features, "mcp", true);
    cJSON_AddItemToObject(root, "features", features);
    AddTextFontCapabilities(root);
    cJSON_AddStringToObject(root, "transport", "websocket");
    cJSON* audio_params = cJSON_CreateObject();
    cJSON_AddStringToObject(audio_params, "format", "opus");
    cJSON_AddNumberToObject(audio_params, "sample_rate", 16000);
    cJSON_AddNumberToObject(audio_params, "channels", 1);
    cJSON_AddNumberToObject(audio_params, "frame_duration", OPUS_FRAME_DURATION_MS);
    cJSON_AddItemToObject(root, "audio_params", audio_params);
    char* json = cJSON_PrintUnformatted(root);
    std::string message = json != nullptr ? json : "";
    cJSON_free(json);
    cJSON_Delete(root);
    return message;
}

std::string BleProtocol::OpenMessage() {
    std::string auth = token_;
    if (!auth.empty() && auth.find(' ') == std::string::npos) {
        auth = "Bearer " + auth;
    }
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "type", "ble_open");
    cJSON_AddStringToObject(root, "device_id", SystemInfo::GetMacAddress().c_str());
    cJSON_AddStringToObject(root, "client_id", Board::GetInstance().GetUuid().c_str());
    cJSON_AddStringToObject(root, "authorization", auth.c_str());
    cJSON_AddStringToObject(root, "protocol_version", std::to_string(version_).c_str());
    char* json = cJSON_PrintUnformatted(root);
    std::string message = json != nullptr ? json : "";
    cJSON_free(json);
    cJSON_Delete(root);
    return message;
}

void BleProtocol::PauseStation() {
    if (WifiManager::GetInstance().IsConfigMode()) {
        return;
    }
    WifiManager::GetInstance().StopStation();
    station_paused_ = true;
}

void BleProtocol::ResumeStation() {
    if (!station_paused_) {
        return;
    }
    station_paused_ = false;
    auto& wifi = WifiManager::GetInstance();
    if (!wifi.IsConfigMode() && !wifi.IsConnected()) {
        wifi.StartStation();
    }
}

bool BleProtocol::OpenAudioChannel() {
    if (!authed_ || conn_handle_ == 0xffff) {
        SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        return false;
    }
    error_occurred_ = false;
    last_rx_us_.store(esp_timer_get_time());
    xEventGroupClearBits(hello_event_, kHelloBit);
    PauseStation();
    if (!SendText(OpenMessage()) || !SendText(HelloMessage())) {
        ResumeStation();
        SetError(Lang::Strings::SERVER_NOT_CONNECTED);
        return false;
    }
    const EventBits_t bits = xEventGroupWaitBits(hello_event_, kHelloBit, pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
    if ((bits & kHelloBit) == 0) {
        ResumeStation();
        SetError(Lang::Strings::SERVER_TIMEOUT);
        return false;
    }
    channel_open_ = true;
    if (on_audio_channel_opened_ != nullptr) {
        on_audio_channel_opened_();
    }
    return true;
}

void BleProtocol::CloseAudioChannel(bool send_goodbye) {
    (void)send_goodbye;
    if (!channel_open_) {
        ResumeStation();
        return;
    }
    channel_open_ = false;
    SendText("{\"type\":\"ble_close\"}");
    ResumeStation();
    if (on_audio_channel_closed_ != nullptr) {
        on_audio_channel_closed_();
    }
}

void BleProtocol::OnStackSync() {
#if CONFIG_BT_NIMBLE_ENABLED
    ble_att_set_preferred_mtu(256);
    tx_handle_ = s_tx_val;
    heart_handle_ = s_heart_val;
    uint8_t addr_type = 0;
    if (ble_hs_util_ensure_addr(0) == 0 && ble_hs_id_infer_auto(0, &addr_type) == 0) {
        own_addr_type_ = addr_type;
    }
    Advertise();
    xSemaphoreGive(sync_sem_);
#endif
}

void BleProtocol::OnHostStopped() {
    if (host_done_ != nullptr) {
        xSemaphoreGive(host_done_);
    }
}

void BleProtocol::Advertise() {
#if CONFIG_BT_NIMBLE_ENABLED
    if (stopping_.load() || conn_handle_ != BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    struct ble_hs_adv_fields fields = {};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    static const char kName[] = "Vibi";
    fields.name = reinterpret_cast<const uint8_t*>(kName);
    fields.name_len = 4;
    fields.name_is_complete = 1;
    fields.uuids128 = const_cast<ble_uuid128_t*>(&kSvcUuid);
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    ble_gap_adv_stop();
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv fields failed: %d", rc);
        return;
    }
    struct ble_gap_adv_params params = {};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    params.itvl_min = BLE_GAP_ADV_ITVL_MS(300);
    params.itvl_max = BLE_GAP_ADV_ITVL_MS(320);
    rc = ble_gap_adv_start(own_addr_type_, nullptr, BLE_HS_FOREVER, &params, GapEvent, nullptr);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv start failed: %d", rc);
    }
#endif
}

void BleProtocol::OnGapEvent(void* raw) {
#if CONFIG_BT_NIMBLE_ENABLED
    auto* event = static_cast<struct ble_gap_event*>(raw);
    if (event->type == BLE_GAP_EVENT_CONNECT) {
        if (event->connect.status != 0) {
            Advertise();
            return;
        }
        conn_handle_ = event->connect.conn_handle;
        authed_ = false;
        connected_at_us_ = esp_timer_get_time();
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(conn_handle_, &desc) == 0) {
            ESP_LOGI(TAG, "connected interval=%u (%u ms) latency=%u timeout=%u", desc.conn_itvl,
                     (desc.conn_itvl * 5) / 4, desc.conn_latency, desc.supervision_timeout);
        }
        struct ble_gap_upd_params want = {};
        want.itvl_min = 12;
        want.itvl_max = 12;
        want.latency = 0;
        want.supervision_timeout = 500;
        int rc = ble_gap_update_params(conn_handle_, &want);
        if (rc != 0) {
            ESP_LOGW(TAG, "conn param update failed: %d", rc);
        }
        return;
    }
    if (event->type == BLE_GAP_EVENT_DISCONNECT) {
        ESP_LOGI(TAG, "disconnected reason=%d", event->disconnect.reason);
        conn_handle_ = BLE_HS_CONN_HANDLE_NONE;
        authed_ = false;
        const bool was_open = channel_open_;
        channel_open_ = false;
        if (was_open) {
            ResumeStation();
            if (on_audio_channel_closed_ != nullptr) {
                on_audio_channel_closed_();
            }
        }
        if (!stopping_.load()) {
            Advertise();
        }
        return;
    }
    if (event->type == BLE_GAP_EVENT_CONN_UPDATE) {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
            ESP_LOGI(TAG, "conn update interval=%u (%u ms) status=%d", desc.conn_itvl,
                     (desc.conn_itvl * 5) / 4, event->conn_update.status);
        }
        return;
    }
    if (event->type == BLE_GAP_EVENT_MTU) {
        const uint16_t mtu = event->mtu.value;
        mtu_payload_.store(mtu > 3 ? static_cast<uint16_t>(mtu - 3) : 20);
        ESP_LOGI(TAG, "mtu=%u payload=%u", mtu, mtu_payload_.load());
        return;
    }
    if (event->type == BLE_GAP_EVENT_ADV_COMPLETE && !stopping_.load()) {
        Advertise();
    }
#else
    (void)raw;
#endif
}

void BleProtocol::OnTick() {
#if CONFIG_BT_NIMBLE_ENABLED
    if (stopping_.load() || conn_handle_ == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    const int64_t now = esp_timer_get_time();
    if (!authed_) {
        if (connected_at_us_ != 0 && now - connected_at_us_ > 5LL * 1000000LL) {
            ESP_LOGW(TAG, "BLE auth timed out");
            ble_gap_terminate(conn_handle_, BLE_ERR_REM_USER_CONN_TERM);
        }
        return;
    }
    if (now - last_heart_us_ >= 30LL * 1000000LL) {
        last_heart_us_ = now;
        uint8_t beat[4] = {
            static_cast<uint8_t>((now >> 24) & 0xff),
            static_cast<uint8_t>((now >> 16) & 0xff),
            static_cast<uint8_t>((now >> 8) & 0xff),
            static_cast<uint8_t>(now & 0xff),
        };
        Notify(heart_handle_, beat, sizeof(beat));
    }
#endif
}

void BleProtocol::TickThunk(void* arg) { static_cast<BleProtocol*>(arg)->OnTick(); }

void BleProtocol::AckThunk(void* arg) { static_cast<BleProtocol*>(arg)->FireAck(); }
