#include "profile_store.h"
#include "vision_pipeline.h"
#include "../hal/hal_storage.h"
#include "../hal/hal_system.h"
#include <cstring>

namespace RobofestDrone {

namespace {

constexpr uint8_t kMagic[4] = {'R', 'V', 'P', '1'};
constexpr uint16_t kHeaderSize = 12;
constexpr uint8_t kMaxSerializedProfiles = Config::VISION_PROFILE_MAX;

// Per-profile fixed payload size (see header layout docs).
constexpr uint16_t kProfileRecordSize =
    1 + 1 + 6 + 7 + 16 + 2 + 20 + 2;

void put_u16(uint8_t*& p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p += 2;
}

uint16_t get_u16(const uint8_t*& p) {
    const uint16_t v = static_cast<uint16_t>(p[0] | (p[1] << 8));
    p += 2;
    return v;
}

void put_f32(uint8_t*& p, float v) {
    // Both host and ESP32-S3 are little-endian IEEE-754.
    uint8_t tmp[4];
    std::memcpy(tmp, &v, 4);
    p[0] = tmp[0];
    p[1] = tmp[1];
    p[2] = tmp[2];
    p[3] = tmp[3];
    p += 4;
}

float get_f32(const uint8_t*& p) {
    uint8_t tmp[4] = {p[0], p[1], p[2], p[3]};
    p += 4;
    float v;
    std::memcpy(&v, tmp, 4);
    return v;
}

void write_u16(uint8_t* dst, uint16_t v) {
    dst[0] = static_cast<uint8_t>(v & 0xFF);
    dst[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void put_band(uint8_t*& p, uint8_t hmin, uint8_t hmax, uint8_t smin,
              uint8_t smax, uint8_t vmin, uint8_t vmax) {
    *p++ = hmin;
    *p++ = hmax;
    *p++ = smin;
    *p++ = smax;
    *p++ = vmin;
    *p++ = vmax;
}

} // namespace

// ============================================================================
// CRC-32 (bitwise, no table -> tiny flash footprint)
// ============================================================================

uint32_t profile_store_crc32(const uint8_t* data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    if (data == nullptr) return 0;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

// ============================================================================
// SERIALIZE / DESERIALIZE
// ============================================================================

uint16_t profile_store_serialize(
    const VisionPipeline& pipeline,
    uint8_t* out,
    uint16_t cap) {

    const uint8_t count = pipeline.getProfileCount();
    const uint16_t payload_len = static_cast<uint16_t>(1 + count * kProfileRecordSize);
    const uint16_t total = static_cast<uint16_t>(kHeaderSize + payload_len);
    if (out == nullptr || cap < total || count > kMaxSerializedProfiles) {
        return total; // caller decides: cap==0 means query size
    }

    uint8_t* p = out + kHeaderSize;

    *p++ = count;
    for (uint8_t i = 0; i < count; ++i) {
        const VisionMarkerProfile* prof = pipeline.getProfileByIndex(i);
        if (prof == nullptr) return 0;

        *p++ = static_cast<uint8_t>(static_cast<uint8_t>(prof->profile_type) & 0xFF);
        // Runtime enable flags are intentionally NOT persisted.
        *p++ = 1;

        put_band(p, prof->h_min, prof->h_max, prof->s_min, prof->s_max,
                 prof->v_min, prof->v_max);

        // Alt band: flag byte followed by the full HSV band.
        *p++ = prof->has_alt_band ? 1 : 0;
        put_band(p, prof->alt_h_min, prof->alt_h_max, prof->alt_s_min,
                 prof->alt_s_max, prof->alt_v_min, prof->alt_v_max);

        put_f32(p, prof->min_area_px);
        put_f32(p, prof->max_area_px);
        put_f32(p, prof->circularity_min);
        put_f32(p, prof->confidence_bias);

        put_u16(p, prof->expected_marker_area_px);

        put_f32(p, prof->aspect_min);
        put_f32(p, prof->aspect_max);
        put_f32(p, prof->extent_min);
        put_f32(p, prof->extent_max);
        put_f32(p, prof->solidity_min);

        *p++ = prof->corners_min;
        *p++ = prof->corners_max;
    }

    // Header.
    uint8_t* h = out;
    h[0] = kMagic[0];
    h[1] = kMagic[1];
    h[2] = kMagic[2];
    h[3] = kMagic[3];
    write_u16(h + 4, payload_len);
    h[6] = 0;
    h[7] = 0;
    const uint32_t crc = profile_store_crc32(out + kHeaderSize, payload_len);
    h[8] = static_cast<uint8_t>(crc & 0xFF);
    h[9] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    h[10] = static_cast<uint8_t>((crc >> 16) & 0xFF);
    h[11] = static_cast<uint8_t>((crc >> 24) & 0xFF);

    return total;
}

bool profile_store_deserialize(
    VisionPipeline& pipeline,
    const uint8_t* data,
    uint16_t len) {

    if (data == nullptr || len <= kHeaderSize) return false;
    if (data[0] != kMagic[0] || data[1] != kMagic[1] ||
        data[2] != kMagic[2] || data[3] != kMagic[3]) {
        return false;
    }

    const uint8_t* p = data + kHeaderSize;
    const uint8_t count = *p++;
    if (count == 0 || count > kMaxSerializedProfiles ||
        len < static_cast<uint16_t>(kHeaderSize + 1 + count * kProfileRecordSize)) {
        return false;
    }

    // Validate everything first: a bad record aborts the whole load so the
    // pipeline never ends up half-calibrated.
    struct StagedRecord {
        Types::VisionMarkerType type;
        uint8_t h_min, h_max, s_min, s_max, v_min, v_max;
        bool has_alt;
        uint8_t alt_h_min, alt_h_max, alt_s_min, alt_s_max, alt_v_min, alt_v_max;
        float min_area, max_area, circ_min, bias;
        uint16_t expected_area;
        float aspect_min, aspect_max, extent_min, extent_max, solidity_min;
        uint8_t corners_min, corners_max;
    };
    StagedRecord staged[kMaxSerializedProfiles];

    for (uint8_t i = 0; i < count; ++i) {
        StagedRecord& r = staged[i];

        const uint8_t type_id = *p++;
        p++; // persisted enabled flag ignored by design
        if (type_id == 0 || type_id > 10) return false;
        r.type = static_cast<Types::VisionMarkerType>(type_id);

        r.h_min = *p++;
        r.h_max = *p++;
        r.s_min = *p++;
        r.s_max = *p++;
        r.v_min = *p++;
        r.v_max = *p++;

        r.has_alt = (*p++ != 0);
        r.alt_h_min = *p++;
        r.alt_h_max = *p++;
        r.alt_s_min = *p++;
        r.alt_s_max = *p++;
        r.alt_v_min = *p++;
        r.alt_v_max = *p++;

        r.min_area = get_f32(p);
        r.max_area = get_f32(p);
        r.circ_min = get_f32(p);
        r.bias = get_f32(p);

        r.expected_area = get_u16(p);

        r.aspect_min = get_f32(p);
        r.aspect_max = get_f32(p);
        r.extent_min = get_f32(p);
        r.extent_max = get_f32(p);
        r.solidity_min = get_f32(p);

        r.corners_min = *p++;
        r.corners_max = *p++;

        // Structural sanity gates.
        if (!(r.min_area >= 0.0f && r.max_area >= r.min_area)) return false;
        if (!(r.circ_min >= 0.0f && r.circ_min <= 1.0f)) return false;
        if (!(r.expected_area > 0u)) return false;
        if (!(r.aspect_min >= 1.0f && r.aspect_max >= r.aspect_min)) return false;
        if (r.s_min > r.s_max || r.v_min > r.v_max) return false;
    }

    // All records valid - commit atomically.
    for (uint8_t i = 0; i < count; ++i) {
        VisionMarkerProfile* prof = pipeline.getProfileByType(staged[i].type);
        if (prof == nullptr) continue; // table changed since save; skip row

        prof->h_min = staged[i].h_min;
        prof->h_max = staged[i].h_max;
        prof->s_min = staged[i].s_min;
        prof->s_max = staged[i].s_max;
        prof->v_min = staged[i].v_min;
        prof->v_max = staged[i].v_max;

        prof->has_alt_band = staged[i].has_alt;
        prof->alt_h_min = staged[i].alt_h_min;
        prof->alt_h_max = staged[i].alt_h_max;
        prof->alt_s_min = staged[i].alt_s_min;
        prof->alt_s_max = staged[i].alt_s_max;
        prof->alt_v_min = staged[i].alt_v_min;
        prof->alt_v_max = staged[i].alt_v_max;

        prof->min_area_px = staged[i].min_area;
        prof->max_area_px = staged[i].max_area;
        prof->circularity_min = staged[i].circ_min;
        prof->confidence_bias = staged[i].bias;
        prof->expected_marker_area_px = staged[i].expected_area;

        prof->aspect_min = staged[i].aspect_min;
        prof->aspect_max = staged[i].aspect_max;
        prof->extent_min = staged[i].extent_min;
        prof->extent_max = staged[i].extent_max;
        prof->solidity_min = staged[i].solidity_min;
        prof->corners_min = staged[i].corners_min;
        prof->corners_max = staged[i].corners_max;
    }
    return true;
}

// ============================================================================
// STORAGE ENVELOPE
// ============================================================================

ProfileStoreStatus profile_store_save(VisionPipeline& pipeline) {
    uint8_t buf[Hal::HAL_STORAGE_BLOB_MAX];
    const uint16_t n = profile_store_serialize(pipeline, buf, sizeof(buf));
    if (n == 0 || n > sizeof(buf)) return ProfileStoreStatus::STORE_IO_ERROR;
    if (!Hal::hal_storage_write_blob("vision_profiles", buf, n)) {
        return ProfileStoreStatus::STORE_IO_ERROR;
    }
    Hal::hal_log("[PROFILE_STORE] Saved calibrated vision profiles.");
    return ProfileStoreStatus::STORE_OK;
}

ProfileStoreStatus profile_store_load(VisionPipeline& pipeline) {
    uint8_t buf[Hal::HAL_STORAGE_BLOB_MAX];
    const int n = Hal::hal_storage_read_blob("vision_profiles", buf, sizeof(buf));
    if (n <= 0) return ProfileStoreStatus::STORE_MISSING;

    // Envelope check before applying anything.
    if (static_cast<uint16_t>(n) < kHeaderSize ||
        buf[0] != kMagic[0] || buf[1] != kMagic[1] ||
        buf[2] != kMagic[2] || buf[3] != kMagic[3]) {
        Hal::hal_log("[PROFILE_STORE] Stored record corrupt (magic). Using defaults.");
        return ProfileStoreStatus::STORE_CORRUPT;
    }

    const uint16_t payload_len =
        static_cast<uint16_t>(buf[4] | (buf[5] << 8));
    if (payload_len != static_cast<uint16_t>(n - kHeaderSize)) {
        Hal::hal_log("[PROFILE_STORE] Stored record corrupt (length). Using defaults.");
        return ProfileStoreStatus::STORE_CORRUPT;
    }

    uint32_t stored_crc = static_cast<uint32_t>(buf[8]) |
                          (static_cast<uint32_t>(buf[9]) << 8) |
                          (static_cast<uint32_t>(buf[10]) << 16) |
                          (static_cast<uint32_t>(buf[11]) << 24);
    if (profile_store_crc32(buf + kHeaderSize, payload_len) != stored_crc) {
        Hal::hal_log("[PROFILE_STORE] Stored record corrupt (CRC). Using defaults.");
        return ProfileStoreStatus::STORE_CORRUPT;
    }

    if (!profile_store_deserialize(pipeline, buf, static_cast<uint16_t>(n))) {
        Hal::hal_log("[PROFILE_STORE] Stored record invalid (structure). Using defaults.");
        return ProfileStoreStatus::STORE_CORRUPT;
    }

    Hal::hal_log("[PROFILE_STORE] Loaded calibrated vision profiles.");
    return ProfileStoreStatus::STORE_OK;
}

void profile_store_erase() {
    Hal::hal_storage_delete_blob("vision_profiles");
}

} // namespace RobofestDrone
