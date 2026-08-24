#pragma once

#include <stdint.h>
#include "../src/types.h"

namespace RobofestDrone {
namespace Hal {

// ============================================================================
// VISUAL GUIDANCE MARKER OUTPUT HAL (REQ-DER-104, item 4)
// ----------------------------------------------------------------------------
// Real hardware path: WS2812B addressable LED strip driven by the ESP32-S3
// RMT peripheral on MARKER_LED_DATA_PIN. A dedicated render task refreshes
// the strip at 20 Hz so control-thread calls never block on wire timing.
// State hand-off between threads is guarded by a short critical section.
// ============================================================================

// Strip geometry (mirrored in Config; kept here for HAL-level defaults).
constexpr uint8_t MARKER_LED_DATA_PIN = 43;  // XIAO ESP32S3 D6
constexpr uint8_t MARKER_LED_COUNT = 16;

// Initializes the marker output hardware and starts the render loop.
bool hal_marker_init();

// Returns true if marker output hardware driver is healthy.
bool hal_marker_is_healthy();

// Runtime marker: true when compiled for host tests / no LED hardware.
bool hal_marker_is_stub();

// Sets the active visual guidance pattern on physical marker hardware.
void hal_marker_set_pattern(Types::MarkerPattern pattern);

// Sets the brightness percentage (0 - 100%) for marker LED arrays.
void hal_marker_set_brightness(uint8_t brightness_percent);

// Enables or disables the physical marker output.
void hal_marker_enable(bool enabled);

// Returns the current pattern set on the hardware.
Types::MarkerPattern hal_marker_get_pattern();

// Guidance-vector steering input (item 4): lateral error of the human/path
// in degrees (-90..+90). Positive steers the arrow window toward higher
// LED indices. Applied by directional patterns (FORWARD/LEFT/RIGHT/REJOIN).
void hal_marker_set_guidance_deg(float heading_error_deg);

// Pure mapping: heading error in degrees -> LED window center index.
// -90 deg -> LED 0, 0 -> strip center, +90 deg -> last LED (clamped outside).
// Host-testable.
uint8_t hal_marker_guidance_index(float heading_error_deg);

// ----------------------------------------------------------------------------
// PURE RENDER MATH (host-testable): maps abstract state -> one LED's color.
//   pattern        active guidance pattern
//   led_index      0..MARKER_LED_COUNT-1 (0 = nearest start zone side)
//   phase          animation phase 0..255 (wraps with blink period)
//   guidance_idx   window center index derived from heading error
// Writes 8-bit RGB into out_rgb[3].
// ----------------------------------------------------------------------------
void marker_render_led(
    Types::MarkerPattern pattern,
    uint8_t led_index,
    uint8_t phase,
    uint8_t guidance_idx,
    uint8_t out_rgb[3]);

} // namespace Hal
} // namespace RobofestDrone
