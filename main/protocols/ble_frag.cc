#include "ble_frag.h"

#include <algorithm>

uint8_t BleCrc8Update(uint8_t crc, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

uint8_t BleCrc8(const uint8_t* data, size_t len) { return BleCrc8Update(0, data, len); }

bool BleFragment(uint8_t seq, uint8_t type, const uint8_t* payload, size_t len,
                 size_t mtu_payload, std::vector<std::vector<uint8_t>>* out) {
    out->clear();
    if (mtu_payload < 5) {
        return false;
    }
    std::vector<uint8_t> body;
    body.reserve(4 + len);
    body.push_back(type);
    body.push_back(0);
    body.push_back(static_cast<uint8_t>((len >> 8) & 0xff));
    body.push_back(static_cast<uint8_t>(len & 0xff));
    body.insert(body.end(), payload, payload + len);

    const size_t slice_len = mtu_payload - 4;
    const size_t count = (body.size() + slice_len - 1) / slice_len;
    if (count == 0 || count > 255) {
        return false;
    }
    out->reserve(count);
    for (size_t index = 0; index < count; ++index) {
        const size_t begin = index * slice_len;
        const size_t n = std::min(slice_len, body.size() - begin);
        std::vector<uint8_t> frag;
        frag.resize(4 + n);
        frag[0] = seq;
        frag[1] = static_cast<uint8_t>(index);
        frag[2] = static_cast<uint8_t>(count);
        frag[3] = 0;
        std::copy(body.begin() + begin, body.begin() + begin + n, frag.begin() + 4);
        frag[3] = BleCrc8Update(BleCrc8(frag.data(), 3), frag.data() + 4, n);
        out->push_back(std::move(frag));
    }
    return true;
}

bool BleReassembler::Push(const uint8_t* data, size_t len, BleMessage* out, bool* duplicate) {
    *duplicate = false;
    if (len < 5) {
        return false;
    }
    const uint8_t seq = data[0];
    const uint8_t index = data[1];
    const uint8_t count = data[2];
    const uint8_t crc = data[3];
    const uint8_t* slice = data + 4;
    const size_t slice_len = len - 4;
    if (count == 0 || index >= count) {
        return false;
    }
    const uint8_t expect = BleCrc8Update(BleCrc8(data, 3), slice, slice_len);
    if (expect != crc) {
        return false;
    }
    if (seq_ != seq) {
        seq_ = seq;
        count_ = count;
        got_ = 0;
        parts_.assign(count, {});
        have_.assign(count, 0);
    }
    if (count != count_ || index >= parts_.size()) {
        return false;
    }
    if (!have_[index]) {
        have_[index] = 1;
        got_++;
    }
    parts_[index].assign(slice, slice + slice_len);
    if (got_ != count_) {
        return false;
    }
    std::vector<uint8_t> body;
    for (uint8_t i = 0; i < count_; ++i) {
        body.insert(body.end(), parts_[i].begin(), parts_[i].end());
    }
    got_ = 0;
    have_.assign(count_, 0);
    parts_.assign(count_, {});
    if (body.size() < 4) {
        return false;
    }
    const size_t size = (static_cast<size_t>(body[2]) << 8) | body[3];
    if (body.size() != 4 + size) {
        return false;
    }
    out->type = body[0];
    out->payload.assign(body.begin() + 4, body.end());
    if (last_completed_ == seq) {
        *duplicate = true;
    } else {
        last_completed_ = seq;
    }
    return true;
}
