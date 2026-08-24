#include "code_reader.h"
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <vector>

#if defined(__has_include)
#if __has_include("quirc.h")
#define ROBOFEST_HAS_QUIRC 1
#include "quirc.h"
#endif
#endif

namespace RobofestDrone {

// ============================================================================
// BINARIZATION (adaptive local mean, separable box blur O(N))
// ============================================================================

bool code_reader_binarize(
    const uint8_t* gray, uint16_t w, uint16_t h,
    uint8_t* out_bin, int16_t bias) {

    if (gray == nullptr || out_bin == nullptr || w == 0 || h == 0) return false;

    // Integral image for O(1) box means (radius 4 window).
    const uint32_t W = w;
    static std::vector<uint32_t> integ;
    integ.assign(static_cast<size_t>(W + 1) * (h + 1), 0);

    for (uint16_t y = 0; y < h; ++y) {
        uint32_t row_sum = 0;
        for (uint16_t x = 0; x < w; ++x) {
            row_sum += gray[static_cast<size_t>(y) * w + x];
            integ[static_cast<size_t>(y + 1) * (W + 1) + (x + 1)] =
                integ[static_cast<size_t>(y) * (W + 1) + (x + 1)] + row_sum;
        }
    }

    constexpr int R = 4;
    for (uint16_t y = 0; y < h; ++y) {
        const int y0 = (y > R) ? (y - R - 1) : -1;
        const int y1 = (y + R < h) ? (y + R) : (h - 1);
        for (uint16_t x = 0; x < w; ++x) {
            const int x0 = (x > R) ? (x - R - 1) : -1;
            const int x1 = (x + R < w) ? (x + R) : (w - 1);
            const uint32_t area =
                static_cast<uint32_t>(x1 - x0) * (y1 - y0);
            const uint32_t sum =
                integ[static_cast<size_t>(y1 + 1) * (W + 1) + (x1 + 1)] -
                integ[static_cast<size_t>(y0 + 1) * (W + 1) + (x1 + 1)] -
                integ[static_cast<size_t>(y1 + 1) * (W + 1) + (x0 + 1)] +
                integ[static_cast<size_t>(y0 + 1) * (W + 1) + (x0 + 1)];
            const int32_t mean = static_cast<int32_t>(sum / (area ? area : 1));
            const uint8_t v = gray[static_cast<size_t>(y) * w + x];
            out_bin[static_cast<size_t>(y) * w + x] =
                (static_cast<int32_t>(v) < mean - bias) ? 1 : 0;
        }
    }
    return true;
}

// ============================================================================
// RUN EXTRACTION
// ============================================================================

uint16_t code_reader_row_runs(
    const uint8_t* bin_row, uint16_t w,
    uint16_t* out_runs, uint16_t cap) {

    if (bin_row == nullptr || out_runs == nullptr || cap == 0) return 0;

    // Trim quiet-zone light padding from both ends.
    uint16_t start = 0, end = w;
    while (start < end && bin_row[start] == 0) start++;
    while (end > start && bin_row[end - 1] == 0) end--;
    if (start >= end) return 0;

    uint16_t n = 0;
    uint8_t cur = 1; // trimmed row starts on ink
    uint16_t len = 0;
    for (uint16_t i = start; i < end; ++i) {
        if (bin_row[i] == cur) {
            len++;
        } else {
            if (n >= cap) return 0;
            out_runs[n++] = len;
            cur = bin_row[i];
            len = 1;
        }
    }
    if (n >= cap) return 0;
    out_runs[n++] = len;
    return n;
}

// ============================================================================
// CODE 39
// ============================================================================

namespace {

struct Code39Entry {
    char ch;
    uint16_t pattern; // 9 bits, bit8 = first bar/space element, 1 = wide
};

// Canonical Code39 width patterns (bars/spaces interleaved, 3 wide of 9).
constexpr Code39Entry kCode39[] = {
    {'0', 0b000110100}, {'1', 0b100100001}, {'2', 0b001100001},
    {'3', 0b101100000}, {'4', 0b000110001}, {'5', 0b100110000},
    {'6', 0b001110000}, {'7', 0b000100101}, {'8', 0b100100100},
    {'9', 0b001100100}, {'A', 0b100001001}, {'B', 0b001001001},
    {'C', 0b101001000}, {'D', 0b000011001}, {'E', 0b100011000},
    {'F', 0b001011000}, {'G', 0b000001101}, {'H', 0b100001100},
    {'I', 0b001001100}, {'J', 0b000011100}, {'K', 0b100000011},
    {'L', 0b001000011}, {'M', 0b101000010}, {'N', 0b000010011},
    {'O', 0b100010010}, {'P', 0b001010010}, {'Q', 0b000000111},
    {'R', 0b100000110}, {'S', 0b001000110}, {'T', 0b000010110},
    {'U', 0b110000001}, {'V', 0b011000001}, {'W', 0b111000000},
    {'X', 0b010010001}, {'Y', 0b110010000}, {'Z', 0b011010000},
    {'-', 0b010000101}, {'.', 0b110000100}, {' ', 0b011000100},
    {'$', 0b010101000}, {'/', 0b010100010}, {'+', 0b010001010},
    {'%', 0b010000010}, {'*', 0b010010100},
};
constexpr uint16_t kCode39Count =
    sizeof(kCode39) / sizeof(kCode39[0]);

char code39_lookup(uint16_t pattern) {
    for (uint16_t i = 0; i < kCode39Count; ++i) {
        if (kCode39[i].pattern == pattern) return kCode39[i].ch;
    }
    return '\0';
}

uint16_t code39_encode(char c) {
    for (uint16_t i = 0; i < kCode39Count; ++i) {
        if (kCode39[i].ch == c) return kCode39[i].pattern;
    }
    return 0xFFFF;
}

} // namespace

bool code_reader_scan_code39(
    const uint16_t* runs, uint16_t run_count,
    CodeReaderResult* out_result) {

    if (out_result == nullptr) return false;
    out_result->format = CodeReaderFormat::NONE;
    out_result->text[0] = '\0';
    out_result->confidence = 0.0f;
    if (runs == nullptr || run_count < 19) return false; // '*' sep '*'

    // Width classification: threshold between observed narrow/wide modes.
    uint16_t min_run = 0xFFFF, max_run = 0;
    for (uint16_t i = 0; i < run_count; ++i) {
        if (runs[i] < min_run) min_run = runs[i];
        if (runs[i] > max_run) max_run = runs[i];
    }
    const uint16_t thr = static_cast<uint16_t>((min_run + max_run) / 2);
    if (max_run == min_run) return false;

    auto to_bits = [&](const uint16_t* r) -> uint16_t {
        uint16_t pat = 0;
        for (int b = 0; b < 9; ++b) {
            pat <<= 1;
            pat |= (r[b] > thr) ? 1u : 0u;
        }
        return pat;
    };

    // Symbol: '*'(9) [sep(1) char(9)]* where the final stop '*' has no
    // trailing separator.
    uint16_t idx = 0;
    char text[CODE_READER_MAX_TEXT + 1];
    uint16_t tlen = 0;

    // Start character.
    if (code39_lookup(to_bits(runs)) != '*') return false;
    idx = 9;

    while (true) {
        const uint16_t remaining = run_count - idx;
        if (remaining == 9) {
            // Final stop candidate without separator.
            if (tlen == 0) return false;
            if (code39_lookup(to_bits(&runs[idx])) != '*') return false;
            text[tlen] = '\0';
            out_result->format = CodeReaderFormat::CODE39;
            out_result->confidence = 0.95f;
            std::memcpy(out_result->text, text, static_cast<size_t>(tlen) + 1);
            return true;
        }
        if (remaining < 10) return false;

        // Separator must be narrow, then nine element runs follow.
        if (runs[idx] > thr) return false;
        const char c = code39_lookup(to_bits(&runs[idx + 1]));
        if (c == '*') {
            if (tlen == 0) return false;
            text[tlen] = '\0';
            out_result->format = CodeReaderFormat::CODE39;
            out_result->confidence = 0.95f;
            std::memcpy(out_result->text, text, static_cast<size_t>(tlen) + 1);
            return true;
        }
        if (c == '\0' || tlen >= CODE_READER_MAX_TEXT) return false;
        text[tlen++] = c;
        idx += 10;
    }
}

// ============================================================================
// INTERLEAVED 2 OF 5
// ============================================================================

namespace {

// Digit element widths (5 elements, 2 wide), 'W'/'N'.
constexpr const char* kItfDigits[10] = {
    "NNWWN", // 0
    "WNNNW", // 1
    "NWNNW", // 2
    "WWNNN", // 3
    "NNWNW", // 4
    "WNWNN", // 5
    "NWWNN", // 6
    "NNNWW", // 7
    "WNNWN", // 8
    "NWNWN", // 9
};

inline int itf_decode_digit(const char* widths) {
    for (int d = 0; d < 10; ++d) {
        if (std::memcmp(kItfDigits[d], widths, 5) == 0) return d;
    }
    return -1;
}

} // namespace

bool code_reader_scan_interleaved_2of5(
    const uint16_t* runs, uint16_t run_count,
    CodeReaderResult* out_result) {

    if (out_result == nullptr) return false;
    out_result->format = CodeReaderFormat::NONE;
    out_result->text[0] = '\0';
    out_result->confidence = 0.0f;
    if (runs == nullptr || run_count < 6 + 10 - 1) return false;

    uint16_t min_run = 0xFFFF, max_run = 0;
    for (uint16_t i = 0; i < run_count; ++i) {
        if (runs[i] < min_run) min_run = runs[i];
        if (runs[i] > max_run) max_run = runs[i];
    }
    const uint16_t thr = static_cast<uint16_t>((min_run + max_run) / 2);
    auto w_of = [&](uint16_t r) -> char { return (r > thr) ? 'W' : 'N'; };

    // START: four narrow elements (bar,space,bar,space).
    if (!(w_of(runs[0]) == 'N' && w_of(runs[1]) == 'N' &&
          w_of(runs[2]) == 'N' && w_of(runs[3]) == 'N')) {
        return false;
    }

    // STOP: wide bar, narrow space, narrow bar at the very end.
    const uint16_t body_end = run_count - 3;
    const uint16_t body_runs = body_end - 4;
    if (body_runs % 10 != 0 || body_runs == 0) return false;

    char text[CODE_READER_MAX_TEXT + 1];
    uint16_t tlen = 0;

    for (uint16_t p = 0; p < body_runs; p += 10) {
        char bars[6] = {};
        char spcs[6] = {};
        for (int k = 0; k < 5; ++k) {
            bars[k] = w_of(runs[4 + p + 2 * k]);      // bar runs
            spcs[k] = w_of(runs[4 + p + 2 * k + 1]);  // space runs
        }
        const int d1 = itf_decode_digit(bars);
        const int d2 = itf_decode_digit(spcs);
        if (d1 < 0 || d2 < 0) return false;
        if (tlen + 2 > CODE_READER_MAX_TEXT) return false;
        text[tlen++] = static_cast<char>('0' + d1);
        text[tlen++] = static_cast<char>('0' + d2);
    }

    // STOP validation.
    if (!(w_of(runs[body_end]) == 'W' &&
          w_of(runs[body_end + 1]) == 'N' &&
          w_of(runs[body_end + 2]) == 'N')) {
        return false;
    }

    text[tlen] = '\0';
    out_result->format = CodeReaderFormat::INTERLEAVED_2OF5;
    out_result->confidence = 0.95f;
    std::memcpy(out_result->text, text, static_cast<size_t>(tlen) + 1);
    return true;
}

// ============================================================================
// GLYPH TEXT OCR (5x7 template atlas)
// ============================================================================

namespace {

// Column-encoded classic 5x7 glyph bitmaps (bit0 = top row).
constexpr uint8_t kFontWidth = 5;
constexpr uint8_t kFontHeight = 7;

struct GlyphDef {
    char ch;
    uint8_t col[5];
};

constexpr GlyphDef kGlyphs[] = {
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x3A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3F, 0x01}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x7F, 0x20, 0x18, 0x20, 0x7F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x03, 0x04, 0x78, 0x04, 0x03}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'/', {0x20, 0x10, 0x08, 0x04, 0x02}},
    {'+', {0x08, 0x08, 0x3E, 0x08, 0x08}},
};
constexpr uint16_t kGlyphCount = sizeof(kGlyphs) / sizeof(kGlyphs[0]);

float glyph_sad_score(const uint8_t cell[5][7], const GlyphDef& g) {
    int sad = 0;
    for (int cx = 0; cx < 5; ++cx) {
        for (int ry = 0; ry < 7; ++ry) {
            const int ref = (g.col[cx] >> ry) & 1;
            sad += (cell[cx][ry] != 0) ? (1 - ref) : ref;
        }
    }
    return 1.0f - static_cast<float>(sad) / 35.0f;
}

char best_glyph(const uint8_t cell[5][7], float& out_score) {
    float best = -1.0f;
    char bc = '\0';
    for (uint16_t i = 0; i < kGlyphCount; ++i) {
        const float s = glyph_sad_score(cell, kGlyphs[i]);
        if (s > best) {
            best = s;
            bc = kGlyphs[i].ch;
        }
    }
    out_score = best;
    return bc;
}

} // namespace

bool code_reader_read_text(
    const uint8_t* bin, uint16_t w, uint16_t h,
    CodeReaderResult* out_result) {

    if (out_result == nullptr) return false;
    out_result->format = CodeReaderFormat::NONE;
    out_result->text[0] = '\0';
    out_result->confidence = 0.0f;
    if (bin == nullptr || w < kFontWidth || h < kFontHeight) return false;

    // Vertical ink projection -> glyph segmentation on blank gaps.
    static uint16_t proj[1024];
    if (w > 1024) return false;
    for (uint16_t x = 0; x < w; ++x) {
        uint16_t cnt = 0;
        for (uint16_t y = 0; y < h; ++y) {
            cnt += bin[static_cast<size_t>(y) * w + x] ? 1 : 0;
        }
        proj[x] = cnt;
    }

    const uint16_t blank_gap = (h / 10) + 1;
    char text[CODE_READER_MAX_TEXT + 1];
    uint16_t tlen = 0;
    float score_acc = 0.0f;
    uint16_t matched = 0;

    uint16_t x = 0;
    while (x < w && tlen < CODE_READER_MAX_TEXT) {
        // Skip blanks.
        while (x < w && proj[x] == 0) x++;
        if (x >= w) break;

        // Find end of this glyph band.
        uint16_t gx0 = x;
        uint16_t gap = 0;
        uint16_t gx1 = x;
        while (gx1 < w) {
            if (proj[gx1] == 0) {
                gap++;
                if (gap >= blank_gap) break;
            } else {
                gap = 0;
            }
            gx1++;
        }
        const uint16_t band_w_raw = (gx1 - gap >= gx0) ? (gx1 - gap - gx0) : 0;
        if (band_w_raw == 0) break;

        // Vertical extent of actual ink inside the band.
        uint16_t gy0 = h, gy1 = 0;
        for (uint16_t yy = 0; yy < h; ++yy) {
            for (uint16_t xx = gx0; xx < gx0 + band_w_raw; ++xx) {
                if (bin[static_cast<size_t>(yy) * w + xx]) {
                    if (yy < gy0) gy0 = yy;
                    if (yy > gy1) gy1 = yy;
                }
            }
        }
        if (gy1 < gy0) {
            x = gx1 - gap;
            continue;
        }

        // Aspect-lock the sampling window to 5:7 around the ink center so
        // glyphs whose outer rows are blank still map onto the full grid.
        uint16_t band_w = band_w_raw;
        uint16_t want_h = static_cast<uint16_t>(
            static_cast<uint32_t>(band_w) * 7 / 5);
        if (want_h > h) {
            // Width-constrained fallback.
            want_h = h;
            band_w = static_cast<uint16_t>(static_cast<uint32_t>(h) * 5 / 7);
            const int mid = (gx0 + gx0 + band_w_raw - 1) / 2;
            gx0 = static_cast<uint16_t>((mid - band_w / 2 > 0) ? (mid - band_w / 2) : 0);
        }
        const int cy_center = (static_cast<int>(gy0) + static_cast<int>(gy1)) / 2;
        int sy_top = cy_center - static_cast<int>(want_h) / 2;
        if (sy_top < 0) sy_top = 0;
        if (sy_top + want_h > h) sy_top = static_cast<int>(h - want_h);
        const uint16_t band_h = want_h;

        // Box-sample the band into the 5x7 reference grid.
        uint8_t cell[5][7] = {};
        for (int cy = 0; cy < 7; ++cy) {
            for (int cx = 0; cx < 5; ++cx) {
                const uint16_t sx0 = gx0 + (band_w * cx) / 5;
                const uint16_t sx1 = gx0 + (band_w * (cx + 1)) / 5;
                const uint16_t sy0 = static_cast<uint16_t>(sy_top + (band_h * cy) / 7);
                const uint16_t sy1 = static_cast<uint16_t>(sy_top + (band_h * (cy + 1)) / 7);
                uint32_t ink = 0, tot = 0;
                for (uint16_t yy = sy0; yy < sy1 && yy < h; ++yy) {
                    for (uint16_t xx = sx0; xx < sx1 && xx < w; ++xx) {
                        tot++;
                        ink += bin[static_cast<size_t>(yy) * w + xx] ? 1 : 0;
                    }
                }
                cell[cx][cy] = (tot > 0 && ink * 2 >= tot) ? 1 : 0;
            }
        }

        float sc = 0.0f;
        const char c = best_glyph(cell, sc);
        if (sc < 0.70f) {
            text[tlen++] = '?';
        } else {
            text[tlen++] = c;
            score_acc += sc;
            matched++;
        }

        x = gx1 - gap;
    }

    if (tlen == 0 || matched == 0 ||
        matched * 2 < tlen) { // more than half garbage => reject
        return false;
    }

    text[tlen] = '\0';
    out_result->format = CodeReaderFormat::GLYPH_TEXT;
    out_result->confidence = score_acc / matched;
    std::memcpy(out_result->text, text, static_cast<size_t>(tlen) + 1);
    return true;
}

// ============================================================================
// QR ADAPTER (quirc)
// ============================================================================

bool code_reader_read_qr(
    const uint8_t* gray, uint16_t w, uint16_t h,
    CodeReaderResult* out_result) {

    if (out_result == nullptr) return false;
    out_result->format = CodeReaderFormat::NONE;
    out_result->text[0] = '\0';
    out_result->confidence = 0.0f;

#if defined(ROBOFEST_HAS_QUIRC)
    if (gray == nullptr || w == 0 || h == 0) return false;

    quirc_code qcode;
    quirc_data qdata;
    quirc* qr = quirc_new();
    if (qr == nullptr) return false;
    bool ok = false;
    do {
        if (quirc_resize(qr, w, h) < 0) break;
        uint8_t* img = quirc_begin(qr, nullptr, nullptr);
        if (img == nullptr) break;
        std::memcpy(img, gray, static_cast<size_t>(w) * h);
        quirc_end(qr);

        const int n = quirc_count(qr);
        for (int i = 0; i < n; ++i) {
            quirc_extract(qr, i, &qcode);
            if (quirc_decode(&qcode, &qdata) < 0) {
                continue;
            }
            uint16_t copy = static_cast<uint16_t>(qdata.payload_len);
            if (copy > CODE_READER_MAX_TEXT) copy = CODE_READER_MAX_TEXT;
            std::memcpy(out_result->text, qdata.payload, copy);
            out_result->text[copy] = '\0';
            out_result->format = CodeReaderFormat::QR_CODE;
            out_result->confidence = 1.0f;
            ok = true;
            break;
        }
    } while (false);
    quirc_destroy(qr);
    return ok;
#else
    // quirc not vendored: report unsupported so callers degrade gracefully.
    (void)gray;
    (void)w;
    (void)h;
    return false;
#endif
}

} // namespace RobofestDrone
