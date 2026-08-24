#pragma once

#include <stdint.h>

namespace RobofestDrone {

// ============================================================================
// CODE READER (REQ-DER-110, item 10)
// ----------------------------------------------------------------------------
// Onboard recognition of machine-readable payloads inside marker ROIs:
//   1. Glyph text   - digits / letters / symbols via a fixed 5x7 template
//                     atlas matched with normalized SAD after box-scaling.
//                     Covers standardized competition marker fonts.
//   2. Code 39      - full ASCII subset barcode (bars/spaces width decode).
//   3. Interleaved
//      2 of 5       - numeric-pair industrial barcode.
//   4. QR codes     - through the quirc library when vendored into lib/quirc
//                     (scripts/vendor_quirc.py); gracefully reports
//                     UNSUPPORTED otherwise so builds never break.
//
// All decoders take binarized input; code_reader_binarize provides an
// adaptive mean-threshold converter from gray. Payload validation rejects
// garbage reads against per-format charset rules before results are
// reported upward.
// ============================================================================

constexpr uint8_t CODE_READER_MAX_TEXT = 32;

enum class CodeReaderFormat : uint8_t {
    NONE = 0,
    GLYPH_TEXT,
    CODE39,
    INTERLEAVED_2OF5,
    QR_CODE
};

struct CodeReaderResult {
    CodeReaderFormat format = CodeReaderFormat::NONE;
    char text[CODE_READER_MAX_TEXT + 1] = {};
    float confidence = 0.0f;
};

// ----------------------------------------------------------------------------
// Binarization: local-mean adaptive threshold on an 8-bit gray ROI.
// out_bin values: 1 = dark (ink), 0 = light (paper). Returns false on nulls.
// ----------------------------------------------------------------------------
bool code_reader_binarize(
    const uint8_t* gray, uint16_t w, uint16_t h,
    uint8_t* out_bin, int16_t bias);

// ----------------------------------------------------------------------------
// Glyph OCR over a binary strip. Splits glyphs by blank columns, scales each
// to the 5x7 atlas grid, and matches by SAD. Writes up to max_chars into
// out_result->text. Confidence = best average match quality 0..1.
// ----------------------------------------------------------------------------
bool code_reader_read_text(
    const uint8_t* bin, uint16_t w, uint16_t h,
    CodeReaderResult* out_result);

// ----------------------------------------------------------------------------
// Barcode scanning along ONE decoded scanline of run widths (alternating
// bar/space, starting with a bar). `runs` holds pixel lengths; the first run
// is a bar. Returns true with payload in out_result when a valid symbol is
// recognized end-to-end including start/stop and checksum where defined.
// ----------------------------------------------------------------------------
bool code_reader_scan_code39(
    const uint16_t* runs, uint16_t run_count,
    CodeReaderResult* out_result);

bool code_reader_scan_interleaved_2of5(
    const uint16_t* runs, uint16_t run_count,
    CodeReaderResult* out_result);

// ----------------------------------------------------------------------------
// Run-length extraction: converts one binarized row into alternating runs.
// Returns the number of runs written (<= cap), starting polarity = ink.
// ----------------------------------------------------------------------------
uint16_t code_reader_row_runs(
    const uint8_t* bin_row, uint16_t w,
    uint16_t* out_runs, uint16_t cap);

// ----------------------------------------------------------------------------
// QR adapter. Returns false with format NONE when quirc is not vendored or
// no symbol decodes; on success fills text (NUL-terminated, truncated at
// CODE_READER_MAX_TEXT) with confidence 1.0.
// ----------------------------------------------------------------------------
bool code_reader_read_qr(
    const uint8_t* gray, uint16_t w, uint16_t h,
    CodeReaderResult* out_result);

} // namespace RobofestDrone
