#include "test_framework.h"
#include "../src/human_detector.h"
#include "../src/gesture_engine.h"
#include "../src/code_reader.h"
#include "../config/thresholds.h"
#include <cstring>
#include <vector>

// ============================================================================
// REQ-DER-102/103/110 (items 2, 3, 10): perception backends.
// All pure logic exercised on synthetic inputs; deterministic.
// ============================================================================

using namespace RobofestDrone;

// ----------------------------------------------------------------------------
// Human motion detector (item 2)
// ----------------------------------------------------------------------------

TEST(human_detector, frame_diff_thresholds) {
    const uint8_t prev[8] = {100, 100, 100, 100, 0, 0, 0, 0};
    const uint8_t cur[8]  = {130, 90, 100, 255, 10, 0, 0, 0};
    uint8_t mask[8] = {};
    human_detector_frame_diff(cur, prev, 8, 18, mask);

    ASSERT_EQ(mask[0], 1); // |130-100| = 30 > 18
    ASSERT_EQ(mask[1], 0); // |90-100| = 10 <= 18
    ASSERT_EQ(mask[2], 0);
    ASSERT_EQ(mask[3], 1);  // |255-100| = 155
    ASSERT_EQ(mask[4], 0);  // |10-0| = 10 <= 18
    ASSERT_EQ(mask[5], 0);
    ASSERT_EQ(mask[6], 0);
    ASSERT_EQ(mask[7], 0);
}

TEST(human_detector, largest_component_finds_blob_and_metrics) {
    // 16x16 mask with a 3x4 blob at (4..6, 3..6) plus single-pixel noise.
    std::vector<uint8_t> mask(256, 0);
    for (uint16_t y = 3; y <= 6; ++y) {
        for (uint16_t x = 4; x <= 6; ++x) {
            mask[y * 16 + x] = 1;
        }
    }
    mask[10 * 16 + 13] = 1; // noise speck

    float cx, cy;
    uint16_t bw, bh;
    const uint32_t n =
        human_detector_largest_component(mask.data(), 16, 16, 6, cx, cy, bw, bh);

    ASSERT_EQ(n, 12u);
    ASSERT_FLOAT_EQ(cx, 5.0f);
    ASSERT_FLOAT_EQ(cy, 4.5f);
    ASSERT_EQ(bw, 3u);
    ASSERT_EQ(bh, 4u);
}

TEST(human_detector, moving_person_detected_static_scene_rejected) {
    constexpr uint16_t W = 64, H = 48;
    HumanMotionDetector det;
    ASSERT_TRUE(det.init(W, H));

    Types::HumanDetectionSample out;

    // Seed history with an empty ground frame.
    std::vector<uint8_t> base(W * H, 40);
    ASSERT_FALSE(det.process(base.data(), out, 100));

    // Walk-by modeled as the person's clothing flickering between two
    // brightness levels in place: every consecutive pair produces one
    // person-sized connected diff blob (no aspect-breaking slivers).
    bool emitted = false;
    for (int f = 1; f <= 4 && !emitted; ++f) {
        std::vector<uint8_t> cur(W * H, 40);
        const uint8_t level = (f % 2 == 1) ? 150 : 110;
        for (int y = 14; y < 30; ++y) {
            for (int x = 20; x < 32; ++x) {
                cur[y * W + x] = level;
            }
        }
        emitted = det.process(cur.data(), out, 100 + f * 66);
    }

    ASSERT_TRUE(emitted);
    ASSERT_TRUE(out.valid);
    ASSERT_TRUE(out.confidence >= 0.4f);
    ASSERT_TRUE(out.pixel_width > 0.0f);
    ASSERT_NEAR(out.pixel_x, 25.5f, 0.6f); // centroid of cols 20..31

    // Static continuation: identical frame yields no further emissions.
    Types::HumanDetectionSample again;
    std::vector<uint8_t> frozen(W * H, 110);
    frozen[14 * W + 20] = 110;
    ASSERT_FALSE(det.process(frozen.data(), again,
                             100 + 5 * 66 + Config::HUMAN_MOTION_REFRACTORY_MS));
}

TEST(human_detector, global_flash_rejected_by_area_gate) {
    constexpr uint16_t W = 64, H = 48;
    HumanMotionDetector det;
    ASSERT_TRUE(det.init(W, H));

    std::vector<uint8_t> a(W * H, 40), b(W * H, 220); // whole-frame flash
    Types::HumanDetectionSample s;
    det.process(a.data(), s, 10);
    ASSERT_FALSE(det.process(b.data(), s, 76));  // area gate kills it
    ASSERT_FALSE(det.process(b.data(), s, 142)); // confirm counter reset
}

// ----------------------------------------------------------------------------
// Gesture engine (item 3)
// ----------------------------------------------------------------------------

namespace {
HandObservation obs_at(bool present, float cx, float cy, uint32_t ms) {
    HandObservation o;
    o.present = present;
    o.cx = cx;
    o.cy = cy;
    o.timestamp_ms = ms;
    return o;
}
} // namespace

TEST(gesture_engine, stationary_hold_emits_start_once) {
    GestureEngine g;
    Types::CommandSample s;

    ASSERT_FALSE(g.update(obs_at(true, 0.5f, 0.5f, 0), s));
    ASSERT_FALSE(g.update(obs_at(true, 0.505f, 0.498f, 300), s));
    ASSERT_TRUE(g.update(obs_at(true, 0.502f, 0.5f, 950), s)); // hold reached
    ASSERT_EQ(s.command, Types::CommandType::START);
    ASSERT_EQ(s.source, Types::CommandSource::COMMAND_SOURCE_GESTURE);

    // Continued presence must NOT retrigger (cooldown).
    ASSERT_FALSE(g.update(obs_at(true, 0.5f, 0.5f, 1200), s));
}

TEST(gesture_engine, lateral_sweep_classifies_direction_once) {
    GestureEngine g;
    Types::CommandSample s;

    g.update(obs_at(true, 0.30f, 0.5f, 0), s);
    ASSERT_TRUE(g.update(obs_at(true, 0.65f, 0.52f, 250), s));
    ASSERT_EQ(s.command, Types::CommandType::SCAN_RIGHT);

    GestureEngine gl;
    gl.update(obs_at(true, 0.70f, 0.5f, 0), s);
    ASSERT_TRUE(gl.update(obs_at(true, 0.32f, 0.5f, 260), s));
    ASSERT_EQ(s.command, Types::CommandType::SCAN_LEFT);
}

TEST(gesture_engine, downward_push_triggers_forward) {
    GestureEngine g;
    Types::CommandSample s;
    g.update(obs_at(true, 0.5f, 0.25f, 0), s);
    ASSERT_TRUE(g.update(obs_at(true, 0.51f, 0.60f, 240), s));
    ASSERT_EQ(s.command, Types::CommandType::FORWARD);
}

TEST(gesture_engine, abrupt_exit_after_hold_emits_stop) {
    GestureEngine g;
    Types::CommandSample s;
    g.update(obs_at(true, 0.5f, 0.5f, 0), s);
    g.update(obs_at(true, 0.5f, 0.5f, 800), s);   // hold matures internally
    ASSERT_TRUE(g.update(obs_at(false, 0.5f, 0.5f, 1000), s));
    ASSERT_EQ(s.command, Types::CommandType::STOP_ABORT);
}

TEST(gesture_engine, cooldown_blocks_immediate_second_gesture) {
    GestureEngine g;
    Types::CommandSample s;
    g.update(obs_at(true, 0.30f, 0.5f, 0), s);
    ASSERT_TRUE(g.update(obs_at(true, 0.65f, 0.5f, 200), s)); // SCAN_RIGHT

    // New sweep attempt inside lockout: engine still in COOLDOWN.
    // Hand leaves and re-enters quickly.
    g.update(obs_at(false, 0.3f, 0.5f, 400), s);
    g.update(obs_at(true, 0.30f, 0.5f, 600), s);
    ASSERT_FALSE(g.update(obs_at(true, 0.70f, 0.5f, 900), s));

    // After lockout expiry a fresh gesture fires again.
    g.update(obs_at(false, 0.7f, 0.5f, Config::COMMAND_LOCKOUT_MS + 1000), s);
    g.update(obs_at(true, 0.30f, 0.5f, Config::COMMAND_LOCKOUT_MS + 1200), s);
    ASSERT_TRUE(g.update(obs_at(true, 0.68f, 0.5f,
                                Config::COMMAND_LOCKOUT_MS + 1500), s));
    ASSERT_EQ(s.command, Types::CommandType::SCAN_RIGHT);
}

TEST(gesture_skin, centroid_of_synthetic_skin_region) {
    constexpr uint16_t W = 64, H = 48;
    std::vector<uint8_t> rgb(W * H * 3, 30);
    // Skin-tone block (r>g>b): rows 10..21, cols 24..39.
    for (uint16_t y = 10; y <= 21; ++y) {
        for (uint16_t x = 24; x <= 39; ++x) {
            rgb[(y * W + x) * 3] = 180;
            rgb[(y * W + x) * 3 + 1] = 130;
            rgb[(y * W + x) * 3 + 2] = 90;
        }
    }
    float cx, cy, frac;
    ASSERT_TRUE(gesture_skin_blob_centroid(rgb.data(), W, H, cx, cy, frac));
    ASSERT_NEAR(cx, (24.0f + 39.0f) / 2 / W, 0.01f);
    ASSERT_NEAR(cy, (10.0f + 21.0f) / 2 / H, 0.01f);
    ASSERT_TRUE(frac > 0.02f && frac < 0.45f);

    // No skin anywhere -> absence reported.
    std::vector<uint8_t> none(W * H * 3, 20);
    ASSERT_FALSE(gesture_skin_blob_centroid(none.data(), W, H, cx, cy, frac));
}

// ----------------------------------------------------------------------------
// Code reader (item 10)
// ----------------------------------------------------------------------------

// Render a glyph string into a binary buffer using the same 5x7 geometry the
// OCR expects (scale px per cell pixel, blank columns between glyphs).
namespace {

constexpr uint8_t kFontW = 5, kFontH = 7;
struct FontRow { char ch; uint8_t col[5]; };

// Mirror of kGlyphs needed only for test-side rendering.
constexpr FontRow kTestFont[] = {
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
};

const FontRow* find_font(char c) {
    for (const auto& f : kTestFont) {
        if (f.ch == c) return &f;
    }
    return nullptr;
}

uint16_t render_text_strip(
    const char* text,
    std::vector<uint8_t>& bin,
    uint16_t& w, uint16_t& h,
    uint8_t scale) {

    size_t len = std::strlen(text);
    h = kFontH * scale;
    w = static_cast<uint16_t>(len * (kFontW * scale + 3 * scale));
    bin.assign(static_cast<size_t>(w) * h, 0);

    uint16_t pen = 0;
    for (size_t i = 0; i < len; ++i) {
        const FontRow* g = find_font(text[i]);
        if (g != nullptr) {
            for (int cx = 0; cx < kFontW; ++cx) {
                for (int ry = 0; ry < kFontH; ++ry) {
                    if ((g->col[cx] >> ry) & 1) {
                        for (uint8_t sy = 0; sy < scale; ++sy) {
                            for (uint8_t sx = 0; sx < scale; ++sx) {
                                bin[static_cast<size_t>(ry * scale + sy) * w +
                                    pen + cx * scale + sx] = 1;
                            }
                        }
                    }
                }
            }
        }
        pen += kFontW * scale + 3 * scale; // blank gap >= blank_gap threshold
    }
    return static_cast<uint16_t>(len);
}

// Code39 encoder mirroring the canonical table inside the reader.
bool encode_code39(const char* text, std::vector<uint16_t>& runs,
                   uint8_t narrow_px) {
    const size_t len = std::strlen(text);
    runs.clear();
    for (size_t i = 0; i < len; ++i) {
        uint16_t pat;
        switch (text[i]) {
            case '*': pat = 0b010010100; break;
            case 'R': pat = 0b100000110; break;
            case 'O': pat = 0b100010010; break;
            case 'B': pat = 0b001001001; break;
            case 'F': pat = 0b001011000; break;
            case 'E': pat = 0b100011000; break;
            case 'S': pat = 0b001000110; break;
            case 'T': pat = 0b000010110; break;
            case 'A': pat = 0b100001001; break;
            case '1': pat = 0b100100001; break;
            case 'C': pat = 0b101001000; break;
            default: return false;
        }
        for (int b = 8; b >= 0; --b) {
            runs.push_back((pat >> b) & 1 ? narrow_px * 3 : narrow_px);
        }
        if (i + 1 < len) runs.push_back(narrow_px); // inter-char space
    }
    return true;
}

bool encode_itf(const char* digits, std::vector<uint16_t>& runs,
                uint8_t narrow_px) {
    const size_t n = std::strlen(digits);
    if (n % 2 != 0 || n < 2 || n > 16) return false;

    constexpr const char* TBL[10] = {
        "NNWWN", "WNNNW", "NWNNW", "WWNNN", "NNWNW",
        "WNWNN", "NWWNN", "NNNWW", "WNNWN", "NWNWN"};

    runs.clear();
    runs.push_back(narrow_px); runs.push_back(narrow_px);  // START b,s
    runs.push_back(narrow_px); runs.push_back(narrow_px);  //        b,s
    for (size_t i = 0; i < n; i += 2) {
        const char* d1 = TBL[digits[i] - '0'];
        const char* d2 = TBL[digits[i + 1] - '0'];
        for (int k = 0; k < 5; ++k) {
            runs.push_back(d1[k] == 'W' ? narrow_px * 3 : narrow_px);
            runs.push_back(d2[k] == 'W' ? narrow_px * 3 : narrow_px);
        }
    }
    runs.push_back(narrow_px * 3); // STOP wide bar
    runs.push_back(narrow_px);     //       narrow space
    runs.push_back(narrow_px);     //       narrow bar
    return true;
}

} // namespace

TEST(code_reader, binarize_dark_ink_detection) {
    constexpr uint16_t W = 20, H = 10;
    std::vector<uint8_t> gray(W * H, 200);
    // Dark square in center.
    for (uint16_t y = 3; y < 7; ++y) {
        for (uint16_t x = 6; x < 14; ++x) {
            gray[y * W + x] = 40;
        }
    }
    std::vector<uint8_t> bin(W * H, 0);
    ASSERT_TRUE(code_reader_binarize(gray.data(), W, H, bin.data(), 10));

    unsigned ink = 0;
    for (auto v : bin) ink += v;
    ASSERT_TRUE(ink >= 30); // dark square detected as ink
    ASSERT_EQ(bin[0], 0);   // background stays paper
}

TEST(code_reader, glyph_text_roundtrip) {
    std::vector<uint8_t> bin;
    uint16_t w = 0, h = 0;
    render_text_strip("ROBOT-9", bin, w, h, 3);

    CodeReaderResult res;
    ASSERT_TRUE(code_reader_read_text(bin.data(), w, h, &res));
    ASSERT_EQ(res.format, CodeReaderFormat::GLYPH_TEXT);
    ASSERT_TRUE(std::strcmp(res.text, "ROBOT-9") == 0);
    ASSERT_TRUE(res.confidence > 0.85f);
}

TEST(code_reader, glyph_garbage_rejected) {
    // Random salt-and-pepper noise must not produce confident text.
    constexpr uint16_t W = 80, H = 21;
    std::vector<uint8_t> bin(W * H, 0);
    uint32_t seed = 12345;
    for (auto& v : bin) {
        seed = seed * 1103515245u + 12345u;
        v = ((seed >> 16) % 100) < 35 ? 1 : 0;
    }
    CodeReaderResult res;
    if (code_reader_read_text(bin.data(), W, H, &res)) {
        // If something decoded, confidence must be low or mostly '?'.
        ASSERT_TRUE(res.confidence < 0.95f ||
                    std::strchr(res.text, '?') != nullptr);
    }
}

TEST(code_reader, code39_roundtrip_full_alphabet_subset) {
    std::vector<uint16_t> runs;
    ASSERT_TRUE(encode_code39("*ROBOT*", runs, 2));
    CodeReaderResult res;
    ASSERT_TRUE(code_reader_scan_code39(runs.data(),
                                        static_cast<uint16_t>(runs.size()), &res));
    ASSERT_EQ(res.format, CodeReaderFormat::CODE39);
    ASSERT_TRUE(std::strcmp(res.text, "ROBOT") == 0);

    ASSERT_TRUE(encode_code39("*A1B*", runs, 3));
    ASSERT_TRUE(code_reader_scan_code39(runs.data(),
                                        static_cast<uint16_t>(runs.size()), &res));
    ASSERT_TRUE(std::strcmp(res.text, "A1B") == 0);
}

TEST(code_reader, code39_missing_start_stop_rejected) {
    std::vector<uint16_t> runs;
    ASSERT_TRUE(encode_code39("*ABC", runs, 2)); // truncated symbol
    CodeReaderResult res;
    ASSERT_FALSE(code_reader_scan_code39(runs.data(),
                                         static_cast<uint16_t>(runs.size()), &res));
}

TEST(code_reader, interleaved_2of5_roundtrip_and_checksum_shape) {
    std::vector<uint16_t> runs;
    ASSERT_TRUE(encode_itf("12345670", runs, 2));
    CodeReaderResult res;
    ASSERT_TRUE(code_reader_scan_interleaved_2of5(
        runs.data(), static_cast<uint16_t>(runs.size()), &res));
    ASSERT_EQ(res.format, CodeReaderFormat::INTERLEAVED_2OF5);
    ASSERT_TRUE(std::strcmp(res.text, "12345670") == 0);

    // Odd digit counts are invalid symbols.
    std::vector<uint16_t> bad;
    ASSERT_TRUE(encode_itf("123", bad, 2) == false);
}

TEST(code_reader, qr_adapter_graceful_without_symbol) {
    constexpr uint16_t W = 60, H = 60;
    std::vector<uint8_t> gray(W * H, 128);
    CodeReaderResult res;
    // No QR symbol present: adapter returns false cleanly (vendored quirc
    // exercises its decoder; non-vendored builds short-circuit).
    ASSERT_FALSE(code_reader_read_qr(gray.data(), W, H, &res));
    ASSERT_EQ(res.format, CodeReaderFormat::NONE);
}
