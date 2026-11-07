#ifndef BLE_FRAG_H_
#define BLE_FRAG_H_

#include <stddef.h>
#include <stdint.h>

#include <vector>

// One ATT payload. A lost fragment drops that message only.
//   seq, index, count, crc8, slice
// crc8 is poly 0x07 init 0 over seq, index, count, and the slice.
// The body is BinaryProtocol3: type, reserved, big-endian payload_size, payload.
// type 0 Opus, 1 JSON, 2 ack (payload is the one-byte seq being acked).

enum BleMsgType {
    kBleTypeAudio = 0,
    kBleTypeJson = 1,
    kBleTypeAck = 2,
};

struct BleMessage {
    uint8_t type = 0;
    std::vector<uint8_t> payload;
};

uint8_t BleCrc8(const uint8_t* data, size_t len);
uint8_t BleCrc8Update(uint8_t crc, const uint8_t* data, size_t len);

// mtu_payload is the ATT value size (MTU - 3). Returns false if it cannot fit.
bool BleFragment(uint8_t seq, uint8_t type, const uint8_t* payload, size_t len,
                 size_t mtu_payload, std::vector<std::vector<uint8_t>>* out);

class BleReassembler {
public:
    // True when a message completed. duplicate is a retransmission of the last seq.
    bool Push(const uint8_t* data, size_t len, BleMessage* out, bool* duplicate);
    uint8_t last_completed_seq() const { return static_cast<uint8_t>(last_completed_); }

private:
    int seq_ = -1;
    uint8_t count_ = 0;
    uint8_t got_ = 0;
    int last_completed_ = -1;
    std::vector<std::vector<uint8_t>> parts_;
    std::vector<uint8_t> have_;
};

#endif
