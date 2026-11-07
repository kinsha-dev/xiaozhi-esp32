#ifndef BLE_PROTOCOL_H_
#define BLE_PROTOCOL_H_

#include "protocol.h"

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_timer.h>

#include "ble_frag.h"

class BleProtocol : public Protocol {
public:
    BleProtocol();
    ~BleProtocol() override;

    bool Start() override;
    bool SendAudio(std::unique_ptr<AudioStreamPacket> packet) override;
    bool OpenAudioChannel() override;
    void CloseAudioChannel(bool send_goodbye = true) override;
    bool IsAudioChannelOpened() const override;
    bool IsTimeout() const override;

    void OnStackSync();
    void OnGapEvent(void* event);
    void OnWrite(const uint8_t* data, size_t len);
    void OnHostStopped();

private:
    static constexpr int kHelloBit = 1 << 0;
    static constexpr int64_t kFallbackHelloWaitUs = 10 * 1000000LL;

    EventGroupHandle_t hello_event_ = nullptr;
    QueueHandle_t ack_queue_ = nullptr;
    SemaphoreHandle_t sync_sem_ = nullptr;
    SemaphoreHandle_t host_done_ = nullptr;
    esp_timer_handle_t tick_timer_ = nullptr;
    esp_timer_handle_t ack_timer_ = nullptr;
    std::atomic<uint8_t> pending_ack_{0};
    std::atomic<bool> ack_pending_{false};

    std::atomic<uint8_t> seq_{0};
    std::atomic<int64_t> last_rx_us_{0};
    std::atomic<bool> stopping_{false};
    bool stack_up_ = false;
    bool authed_ = false;
    bool channel_open_ = false;
    bool station_paused_ = false;
    uint16_t conn_handle_ = 0xffff;
    uint16_t tx_handle_ = 0;
    uint16_t heart_handle_ = 0;
    uint8_t own_addr_type_ = 0;
    std::atomic<uint16_t> mtu_payload_{20};
    int64_t connected_at_us_ = 0;
    int64_t last_heart_us_ = 0;
    int version_ = 1;
    std::string token_;
    BleReassembler reassembler_;

    bool SendText(const std::string& text) override;
    bool SendTyped(uint8_t type, const uint8_t* data, size_t len, bool reliable);
    bool Notify(uint16_t attr, const uint8_t* data, size_t len);
    void SendAck(uint8_t acked_seq);
    void QueueAck(uint8_t acked_seq);
    void FireAck();
    void HandleJson(const std::string& text);
    void ParseServerHello(const cJSON* root);
    std::string HelloMessage();
    std::string OpenMessage();
    void Advertise();
    void PauseStation();
    void ResumeStation();
    void StopStack();
    void OnTick();
    static void TickThunk(void* arg);
    static void AckThunk(void* arg);
};

#endif
