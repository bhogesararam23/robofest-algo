#include "vision_pipeline.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include "../hal/hal_system.h"

namespace RobofestDrone {

namespace {
    static VisionPipeline s_global_vision_pipeline;

    constexpr float M_PI_F = 3.14159265358979323846f;
    constexpr float DEG_TO_RAD = M_PI_F / 180.0f;

    // Static buffers to prevent any heap allocation on the embedded vision update path
    static uint8_t s_binary_mask[Config::VISION_PROCESS_WIDTH * Config::VISION_PROCESS_HEIGHT];
    static uint8_t s_morph_temp[Config::VISION_PROCESS_WIDTH * Config::VISION_PROCESS_HEIGHT];
    static uint16_t s_bfs_x[Config::VISION_PROCESS_WIDTH * Config::VISION_PROCESS_HEIGHT];
    static uint16_t s_bfs_y[Config::VISION_PROCESS_WIDTH * Config::VISION_PROCESS_HEIGHT];
    static VisionBlob s_extracted_blobs[Config::VISION_MAX_BLOBS];
}

// ============================================================================
// CONSTRUCTOR & INITIALIZATION
// ============================================================================

VisionPipeline::VisionPipeline() {
    reset();
}

void VisionPipeline::init() {
    reset();
    Hal::hal_camera_init();
    camera_healthy_ = Hal::hal_camera_is_healthy();

    // Lock camera exposure and white-balance to prevent auto-adjustment from
    // invalidating calibrated HSV values mid-flight.
    Hal::hal_camera_set_auto_exposure(Config::CAMERA_AUTO_EXPOSURE_ENABLED);
    if (Config::CAMERA_MANUAL_EXPOSURE_VALUE != 0) {
        Hal::hal_camera_set_exposure(Config::CAMERA_MANUAL_EXPOSURE_VALUE);
    }
    Hal::hal_camera_set_auto_whitebalance(Config::CAMERA_AUTO_WHITEBALANCE_ENABLED);

    if (Hal::hal_camera_is_stub()) {
        setTelemetryEvent(TE_VISION_MASK_EMPTY); // Distinct event: using stub camera, no real data
        hal_log("[VISION] Using stub camera - no real image data being processed.");
    }
}

void VisionPipeline::reset() {
    initProfiles();
    active_profile_type_ = Types::VisionMarkerType::ON_GROUND_MINE;

    candidate_count_ = 0;
    for (uint8_t i = 0; i < Config::VISION_MAX_CANDIDATES; ++i) {
        candidates_[i] = Types::VisionCandidate();
    }

    for (uint8_t i = 0; i < Config::VISION_MAX_PERSISTENCE_TRACKS; ++i) {
        tracks_[i] = VisionPersistenceTrack();
    }
    next_track_id_ = 1;

    h_fov_deg_ = Config::H_FOV_DEG;
    v_fov_deg_ = Config::V_FOV_DEG;
    image_center_x_ = Config::IMAGE_CENTER_X;
    image_center_y_ = Config::IMAGE_CENTER_Y;

    attitude_compensation_enabled_ = Config::VISION_ATTITUDE_COMPENSATION_ENABLED;
    downscale_enabled_ = Config::VISION_DOWNSCALE_ENABLED;

    camera_healthy_ = false;
    last_frame_time_ms_ = 0;
    last_process_time_ms_ = 0;
    processing_duration_us_ = 0;
    frame_count_ = 0;
    frame_timeout_count_ = 0;
    dropped_frame_count_ = 0;

    last_telemetry_event_id_ = TE_VISION_INITIALIZED;
    telemetry_event_valid_ = true;
}

void VisionPipeline::initProfiles() {
    // 1. On-Ground Mine Profile (Standard High-Contrast Marker)
    profile_on_ground_.profile_type = Types::VisionMarkerType::ON_GROUND_MINE;
    profile_on_ground_.enabled = true;
    profile_on_ground_.h_min = Config::ON_GROUND_MINE_HSV_LOW.h;
    profile_on_ground_.h_max = Config::ON_GROUND_MINE_HSV_HIGH.h;
    profile_on_ground_.s_min = Config::ON_GROUND_MINE_HSV_LOW.s;
    profile_on_ground_.s_max = Config::ON_GROUND_MINE_HSV_HIGH.s;
    profile_on_ground_.v_min = Config::ON_GROUND_MINE_HSV_LOW.v;
    profile_on_ground_.v_max = Config::ON_GROUND_MINE_HSV_HIGH.v;
    profile_on_ground_.min_area_px = Config::BLOB_AREA_MIN_PX;
    profile_on_ground_.max_area_px = Config::BLOB_AREA_MAX_PX;
    profile_on_ground_.circularity_min = Config::CIRCULARITY_MIN;
    profile_on_ground_.confidence_bias = 0.0f;
    profile_on_ground_.expected_marker_area_px = static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX);

    // 2. Buried Surface Marker Profile (Tunable Calibration Category)
    profile_buried_.profile_type = Types::VisionMarkerType::BURIED_SURFACE_MARKER;
    profile_buried_.enabled = true;
    profile_buried_.h_min = Config::BURIED_SURFACE_MARKER_HSV_LOW.h;
    profile_buried_.h_max = Config::BURIED_SURFACE_MARKER_HSV_HIGH.h;
    profile_buried_.s_min = Config::BURIED_SURFACE_MARKER_HSV_LOW.s;
    profile_buried_.s_max = Config::BURIED_SURFACE_MARKER_HSV_HIGH.s;
    profile_buried_.v_min = Config::BURIED_SURFACE_MARKER_HSV_LOW.v;
    profile_buried_.v_max = Config::BURIED_SURFACE_MARKER_HSV_HIGH.v;
    profile_buried_.min_area_px = Config::BLOB_AREA_MIN_PX;
    profile_buried_.max_area_px = Config::BLOB_AREA_MAX_PX;
    profile_buried_.circularity_min = Config::CIRCULARITY_MIN;
    profile_buried_.confidence_bias = 5.0f;
    profile_buried_.expected_marker_area_px = static_cast<uint16_t>(Config::MARKER_AREA_NOMINAL_PX);
}


// ============================================================================
// PROFILE & CALIBRATION SETTERS
// ============================================================================

void VisionPipeline::setActiveProfile(Types::VisionMarkerType profile) {
    active_profile_type_ = profile;
}

void VisionPipeline::setCameraCalibration(
    float h_fov_deg,
    float v_fov_deg,
    float image_center_x,
    float image_center_y
) {
    if (h_fov_deg > 10.0f && v_fov_deg > 10.0f) {
        h_fov_deg_ = h_fov_deg;
        v_fov_deg_ = v_fov_deg;
        image_center_x_ = image_center_x;
        image_center_y_ = image_center_y;
    }
}

void VisionPipeline::setTelemetryEvent(uint16_t event_id) {
    last_telemetry_event_id_ = event_id;
    telemetry_event_valid_ = true;
}

bool VisionPipeline::isFrameFresh(uint32_t now_ms) const {
    return (now_ms - last_frame_time_ms_) <= Config::CAMERA_STALL_TIMEOUT_MS;
}

Types::VisionCandidate VisionPipeline::getCandidate(uint8_t index) const {
    if (!camera_healthy_) {
        return Types::VisionCandidate();
    }
    if (index < candidate_count_) {
        return candidates_[index];
    }
    return Types::VisionCandidate();
}


// ============================================================================
// COLOR SEGMENTATION (HSV BINARY MASK)
// ============================================================================

void VisionPipeline::segmentHsvMask(const Hal::CameraFrame& frame, const VisionMarkerProfile& profile) {
    std::memset(s_binary_mask, 0, sizeof(s_binary_mask));

    if (frame.data == nullptr || frame.width == 0 || frame.height == 0) {
        setTelemetryEvent(TE_VISION_MASK_EMPTY);
        return;
    }

    // Check pixel format once upfront - bail with distinct telemetry if unsupported,
    // rather than discovering it one pixel at a time and producing an empty mask
    // indistinguishable from "no markers visible."
    if (frame.format != Hal::PixelFormat::PIXEL_FORMAT_RGB565 &&
        frame.format != Hal::PixelFormat::PIXEL_FORMAT_RGB888) {
        setTelemetryEvent(TE_VISION_UNSUPPORTED_PIXEL_FORMAT);
        return;
    }

    uint16_t step_x = frame.width / Config::VISION_PROCESS_WIDTH;
    uint16_t step_y = frame.height / Config::VISION_PROCESS_HEIGHT;
    if (step_x == 0) step_x = 1;
    if (step_y == 0) step_y = 1;

    for (uint16_t py = 0; py < Config::VISION_PROCESS_HEIGHT; ++py) {
        uint16_t src_y = py * step_y;
        if (src_y >= frame.height) break;

        for (uint16_t px = 0; px < Config::VISION_PROCESS_WIDTH; ++px) {
            uint16_t src_x = px * step_x;
            if (src_x >= frame.width) break;

            uint8_t r = 0, g = 0, b = 0;

            if (frame.format == Hal::PixelFormat::PIXEL_FORMAT_RGB565) {
                uint32_t pixel_idx = (src_y * frame.width + src_x) * 2;
                uint16_t raw_pixel =
                    Config::RGB565_LE_BYTE_ORDER
                        ? (frame.data[pixel_idx] | (frame.data[pixel_idx + 1] << 8))
                        : (frame.data[pixel_idx + 1] | (frame.data[pixel_idx] << 8));
                r = ((raw_pixel >> 11) & 0x1F) << 3;
                g = ((raw_pixel >> 5) & 0x3F) << 2;
                b = (raw_pixel & 0x1F) << 3;
            } else if (frame.format == Hal::PixelFormat::PIXEL_FORMAT_RGB888) {
                uint32_t pixel_idx = (src_y * frame.width + src_x) * 3;
                r = frame.data[pixel_idx];
                g = frame.data[pixel_idx + 1];
                b = frame.data[pixel_idx + 2];
            } else {
                continue;
            }

            // Optional 3x3 box blur: average the pixel with its neighbors to smooth
            // single-pixel sensor noise before HSV conversion. Only applies when
            // blur is enabled AND the pixel is not at the frame boundary.
            if (Config::VISION_BLUR_ENABLED &&
                src_x > 0 && src_y > 0 &&
                src_x < (frame.width - 1) && src_y < (frame.height - 1)) {
                uint32_t sum_r = r, sum_g = g, sum_b = b;
                uint8_t count = 1;
                // Sample 4 cardinal neighbors (cheaper than full 3x3, good enough for noise)
                static const int16_t ndx[4] = {-1, 1, 0, 0};
                static const int16_t ndy[4] = {0, 0, -1, 1};
                for (int n = 0; n < 4; ++n) {
                    uint16_t nx = static_cast<uint16_t>(src_x + ndx[n]);
                    uint16_t ny = static_cast<uint16_t>(src_y + ndy[n]);
                    if (frame.format == Hal::PixelFormat::PIXEL_FORMAT_RGB565) {
                        uint32_t ni = (ny * frame.width + nx) * 2;
                        uint16_t np = Config::RGB565_LE_BYTE_ORDER
                            ? (frame.data[ni] | (frame.data[ni + 1] << 8))
                            : (frame.data[ni + 1] | (frame.data[ni] << 8));
                        sum_r += ((np >> 11) & 0x1F) << 3;
                        sum_g += ((np >> 5) & 0x3F) << 2;
                        sum_b += (np & 0x1F) << 3;
                    } else {
                        uint32_t ni = (ny * frame.width + nx) * 3;
                        sum_r += frame.data[ni];
                        sum_g += frame.data[ni + 1];
                        sum_b += frame.data[ni + 2];
                    }
                    count++;
                }
                r = static_cast<uint8_t>(sum_r / count);
                g = static_cast<uint8_t>(sum_g / count);
                b = static_cast<uint8_t>(sum_b / count);
            }

            // Fast inline RGB to HSV conversion
            uint8_t c_max = std::max(r, std::max(g, b));
            uint8_t c_min = std::min(r, std::min(g, b));
            uint8_t delta = c_max - c_min;

            uint8_t v = c_max;
            uint8_t s = (c_max == 0) ? 0 : static_cast<uint8_t>((255UL * delta) / c_max);
            uint8_t h = 0;

            if (delta > 0) {
                int16_t h_calc = 0;
                if (c_max == r) {
                    h_calc = 30 * (static_cast<int16_t>(g) - static_cast<int16_t>(b)) / delta;
                } else if (c_max == g) {
                    h_calc = 60 + 30 * (static_cast<int16_t>(b) - static_cast<int16_t>(r)) / delta;
                } else {
                    h_calc = 120 + 30 * (static_cast<int16_t>(r) - static_cast<int16_t>(g)) / delta;
                }
                if (h_calc < 0) h_calc += 180;
                h = static_cast<uint8_t>(h_calc);
            }

            // Check HSV inclusion
            bool h_ok = (profile.h_min <= profile.h_max) ?
                        (h >= profile.h_min && h <= profile.h_max) :
                        (h >= profile.h_min || h <= profile.h_max);

            bool s_ok = (s >= profile.s_min && s <= profile.s_max);
            bool v_ok = (v >= profile.v_min && v <= profile.v_max);

            if (h_ok && s_ok && v_ok) {
                s_binary_mask[py * Config::VISION_PROCESS_WIDTH + px] = 1;
            }
        }
    }
}


// ============================================================================
// MORPHOLOGICAL NOISE CLEANUP (ERODE + DILATE ON BINARY MASK)
// ============================================================================

void VisionPipeline::applyMorphologyCleanup() {
    if (!Config::VISION_MORPHOLOGY_ENABLED) {
        return;
    }

    const uint16_t W = Config::VISION_PROCESS_WIDTH;
    const uint16_t H = Config::VISION_PROCESS_HEIGHT;
    const uint8_t R = Config::VISION_MORPHOLOGY_RADIUS;

    // Pass 1: Erosion — pixel stays ON only if ALL neighbors within radius are ON.
    // This removes isolated noise specks smaller than the kernel.
    std::memcpy(s_morph_temp, s_binary_mask, W * H);
    for (uint16_t y = 0; y < H; ++y) {
        for (uint16_t x = 0; x < W; ++x) {
            if (s_morph_temp[y * W + x] == 0) continue;
            bool all_set = true;
            for (int16_t dy = -R; dy <= R && all_set; ++dy) {
                for (int16_t dx = -R; dx <= R && all_set; ++dx) {
                    int16_t nx = static_cast<int16_t>(x) + dx;
                    int16_t ny = static_cast<int16_t>(y) + dy;
                    if (nx < 0 || nx >= W || ny < 0 || ny >= H) {
                        all_set = false;
                    } else if (s_morph_temp[ny * W + nx] == 0) {
                        all_set = false;
                    }
                }
            }
            s_binary_mask[y * W + x] = all_set ? 1 : 0;
        }
    }

    // Pass 2: Dilation — pixel turns ON if ANY neighbor within radius is ON.
    // This restores the object shape that erosion may have slightly shrunk,
    // and fills small internal holes so contourArea isn't underestimated.
    std::memcpy(s_morph_temp, s_binary_mask, W * H);
    for (uint16_t y = 0; y < H; ++y) {
        for (uint16_t x = 0; x < W; ++x) {
            if (s_morph_temp[y * W + x] != 0) continue;
            bool any_set = false;
            for (int16_t dy = -R; dy <= R && !any_set; ++dy) {
                for (int16_t dx = -R; dx <= R && !any_set; ++dx) {
                    int16_t nx = static_cast<int16_t>(x) + dx;
                    int16_t ny = static_cast<int16_t>(y) + dy;
                    if (nx >= 0 && nx < W && ny >= 0 && ny < H) {
                        if (s_morph_temp[ny * W + nx] != 0) {
                            any_set = true;
                        }
                    }
                }
            }
            if (any_set) {
                s_binary_mask[y * W + x] = 1;
            }
        }
    }
}


// ============================================================================
// CONNECTED COMPONENT EXTRACTION (4-CONNECTIVITY BLOB ANALYSIS)
// ============================================================================

uint8_t VisionPipeline::extractBlobs(const VisionMarkerProfile& profile) {
    uint8_t blob_count = 0;

    for (uint16_t y = 0; y < Config::VISION_PROCESS_HEIGHT; ++y) {
        for (uint16_t x = 0; x < Config::VISION_PROCESS_WIDTH; ++x) {
            uint32_t idx = y * Config::VISION_PROCESS_WIDTH + x;

            if (s_binary_mask[idx] == 1) {
                if (blob_count >= Config::VISION_MAX_BLOBS) {
                    return blob_count;
                }

                // BFS Flood Fill for connected component
                uint16_t q_head = 0;
                uint16_t q_tail = 0;
                s_bfs_x[q_tail] = x;
                s_bfs_y[q_tail] = y;
                q_tail++;
                s_binary_mask[idx] = 2; // Visited

                uint32_t area = 0;
                uint32_t sum_x = 0;
                uint32_t sum_y = 0;
                uint16_t min_x = x, max_x = x;
                uint16_t min_y = y, max_y = y;
                uint32_t perimeter = 0;

while (q_head < q_tail && q_tail < Config::VISION_PROCESS_WIDTH * Config::VISION_PROCESS_HEIGHT) {
                    uint16_t cx = s_bfs_x[q_head];
                    uint16_t cy = s_bfs_y[q_head];
                    q_head++;

                    area++;
                    sum_x += cx;
                    sum_y += cy;

                    if (cx < min_x) min_x = cx;
                    if (cx > max_x) max_x = cx;
                    if (cy < min_y) min_y = cy;
                    if (cy > max_y) max_y = cy;

                    // Check 4-connectivity neighbors
                    static const int dx[4] = {0, 1, 0, -1};
                    static const int dy[4] = {-1, 0, 1, 0};
                    bool is_border_pixel = false;

                    for (int d = 0; d < 4; ++d) {
                        int nx = static_cast<int>(cx) + dx[d];
                        int ny = static_cast<int>(cy) + dy[d];

                        if (nx < 0 || nx >= Config::VISION_PROCESS_WIDTH ||
                            ny < 0 || ny >= Config::VISION_PROCESS_HEIGHT) {
                            is_border_pixel = true;
                        } else {
                            uint32_t n_idx = static_cast<uint32_t>(ny * Config::VISION_PROCESS_WIDTH + nx);
                            if (s_binary_mask[n_idx] == 0) {
                                is_border_pixel = true;
                            } else if (s_binary_mask[n_idx] == 1 && q_tail < Config::VISION_PROCESS_WIDTH * Config::VISION_PROCESS_HEIGHT) {
                                s_binary_mask[n_idx] = 2; // Mark queued
                                s_bfs_x[q_tail] = static_cast<uint16_t>(nx);
                                s_bfs_y[q_tail] = static_cast<uint16_t>(ny);
                                q_tail++;
                            }
                        }
                    }

                    if (is_border_pixel) {
                        perimeter++;
                    }
                }

                // Check if BFS overflowed the scan buffer
                if (q_tail >= Config::VISION_PROCESS_WIDTH * Config::VISION_PROCESS_HEIGHT) {
                    setTelemetryEvent(TE_VISION_BLOB_EXCEEDED_SCAN_BUFFER);
                    continue; // Discard this entire blob - don't measure/score partial region
                }

                // Rescale metrics to full camera frame coordinates
                float scale_x = static_cast<float>(Config::IMAGE_WIDTH) / static_cast<float>(Config::VISION_PROCESS_WIDTH);
                float scale_y = static_cast<float>(Config::IMAGE_HEIGHT) / static_cast<float>(Config::VISION_PROCESS_HEIGHT);
                float full_area = static_cast<float>(area) * (scale_x * scale_y);
                float full_perimeter = static_cast<float>(perimeter) * std::sqrt((scale_x * scale_x + scale_y * scale_y) * 0.5f);


                if (full_perimeter < 1.0f) full_perimeter = 1.0f;

                // Circularity metric: 4 * PI * Area / (Perimeter^2)
                float circularity = (4.0f * M_PI_F * full_area) / (full_perimeter * full_perimeter);
                if (circularity > 1.0f) circularity = 1.0f;
                if (circularity < 0.0f) circularity = 0.0f;

                // Glare rejection: distinct tighter check that runs before general area check.
                // Catches extremely large areas from lens flare/specular reflection (> 5000 px)
                // that would otherwise pass the normal area filter. Glare threshold is set above
                // the normal marker area max (2500 px) so it only triggers for abnormally large regions.
                if (Config::GLARE_REJECT_ENABLED && full_area > Config::GLARE_AREA_MAX_PX) {
                    setTelemetryEvent(TE_VISION_BLOB_REJECTED_GLARE);
                    continue;
                }

                if (full_area < profile.min_area_px || full_area > profile.max_area_px) {
                    setTelemetryEvent(TE_VISION_BLOB_REJECTED_AREA);
                    continue;
                }

                if (circularity < profile.circularity_min) {
                    setTelemetryEvent(TE_VISION_BLOB_REJECTED_CIRCULARITY);
                    continue;
                }

                float full_min_x = static_cast<float>(min_x) * scale_x;
                float full_max_x = static_cast<float>(max_x) * scale_x;
                float full_min_y = static_cast<float>(min_y) * scale_y;
                float full_max_y = static_cast<float>(max_y) * scale_y;

                if (full_min_x <= Config::EDGE_REJECT_MARGIN_PX ||
                    full_max_x >= (Config::IMAGE_WIDTH - Config::EDGE_REJECT_MARGIN_PX) ||
                    full_min_y <= Config::EDGE_REJECT_MARGIN_PX ||
                    full_max_y >= (Config::IMAGE_HEIGHT - Config::EDGE_REJECT_MARGIN_PX)) {
                    setTelemetryEvent(TE_VISION_BLOB_REJECTED_EDGE);
                    continue;
                }

                // Populate valid blob descriptor
                s_extracted_blobs[blob_count].centroid_x = (static_cast<float>(sum_x) / static_cast<float>(area)) * scale_x;
                s_extracted_blobs[blob_count].centroid_y = (static_cast<float>(sum_y) / static_cast<float>(area)) * scale_y;
                s_extracted_blobs[blob_count].area = full_area;
                s_extracted_blobs[blob_count].perimeter = full_perimeter;
                s_extracted_blobs[blob_count].circularity = circularity;
                s_extracted_blobs[blob_count].x_min = static_cast<uint16_t>(full_min_x);
                s_extracted_blobs[blob_count].x_max = static_cast<uint16_t>(full_max_x);
                s_extracted_blobs[blob_count].y_min = static_cast<uint16_t>(full_min_y);
                s_extracted_blobs[blob_count].y_max = static_cast<uint16_t>(full_max_y);
                s_extracted_blobs[blob_count].valid = true;
                blob_count++;
            }
        }
    }

    return blob_count;
}


// ============================================================================
// WORLD PROJECTION (GROUND PLANE RAY INTERSECTION)
// ============================================================================

void VisionPipeline::projectToWorld(
    float pixel_x,
    float pixel_y,
    float altitude_m,
    const Types::Pose2D& drone_pose,
    const Types::AttitudeSample& attitude,
    float& out_world_x,
    float& out_world_y,
    bool& out_valid
) {
    out_valid = true;  // assume valid unless we detect an issue

    if (altitude_m < Config::MIN_PROJECTION_ALTITUDE_M) {
        setTelemetryEvent(TE_VISION_ALTITUDE_TOO_LOW);
        out_valid = false;
        out_world_x = drone_pose.field_x;
        out_world_y = drone_pose.field_y;
        return;
    }

    // Normalized camera coordinate space
    float u = (pixel_x - image_center_x_) / (Config::IMAGE_WIDTH * 0.5f);
    float v = (pixel_y - image_center_y_) / (Config::IMAGE_HEIGHT * 0.5f);

    float tan_h_half = std::tan((h_fov_deg_ * 0.5f) * DEG_TO_RAD);
    float tan_v_half = std::tan((v_fov_deg_ * 0.5f) * DEG_TO_RAD);

    float ray_body_x = u * tan_h_half;
    float ray_body_y = v * tan_v_half;
    float ray_body_z = 1.0f; // Downward along optical axis

    if (attitude_compensation_enabled_ && attitude.valid) {
        if (std::abs(attitude.roll_deg) > 30.0f || std::abs(attitude.pitch_deg) > 30.0f) {
            setTelemetryEvent(TE_VISION_ATTITUDE_INVALID);
            out_valid = false;
            out_world_x = drone_pose.field_x;
            out_world_y = drone_pose.field_y;
            return;
        }

        if (std::abs(attitude.roll_deg) > 15.0f || std::abs(attitude.pitch_deg) > 15.0f) {
            setTelemetryEvent(TE_VISION_PROJECTION_DEGRADED);
        }

        // Apply Roll (phi), Pitch (theta), and Heading Yaw (psi) rotation
        float phi   = attitude.roll_deg * DEG_TO_RAD;
        float theta = attitude.pitch_deg * DEG_TO_RAD;
        float psi   = drone_pose.yaw_deg * DEG_TO_RAD;

        float cos_phi = std::cos(phi);
        float sin_phi = std::sin(phi);
        float cos_theta = std::cos(theta);
        float sin_theta = std::sin(theta);
        float cos_psi = std::cos(psi);
        float sin_psi = std::sin(psi);

        // Rotation matrix: R = R_z(psi) * R_y(theta) * R_x(phi)
        float r_z = -sin_theta * ray_body_x + sin_phi * cos_theta * ray_body_y + cos_phi * cos_theta * ray_body_z;
        if (r_z <= 0.05f) r_z = 0.05f; // Protect against grazing ground ray

        float r_x = (cos_psi * cos_theta) * ray_body_x +
                    (cos_psi * sin_theta * sin_phi - sin_psi * cos_phi) * ray_body_y +
                    (cos_psi * sin_theta * cos_phi + sin_psi * sin_phi) * ray_body_z;

        float r_y = (sin_psi * cos_theta) * ray_body_x +
                    (sin_psi * sin_theta * sin_phi + cos_psi * cos_phi) * ray_body_y +
                    (sin_psi * sin_theta * cos_phi - cos_psi * sin_phi) * ray_body_z;

        float scale = altitude_m / r_z;
        out_world_x = drone_pose.field_x + (r_x * scale);
        out_world_y = drone_pose.field_y + (r_y * scale);
    } else {
        // Fallback flat projection
        float yaw_rad = drone_pose.yaw_deg * DEG_TO_RAD;
        float cos_yaw = std::cos(yaw_rad);
        float sin_yaw = std::sin(yaw_rad);

        float offset_x = altitude_m * ray_body_x;
        float offset_y = altitude_m * ray_body_y;

        out_world_x = drone_pose.field_x + (offset_x * cos_yaw - offset_y * sin_yaw);
        out_world_y = drone_pose.field_y + (offset_x * sin_yaw + offset_y * cos_yaw);
    }
}


// ============================================================================
// TEMPORAL PERSISTENCE & CANDIDATE REPORTING
// ============================================================================

void VisionPipeline::pruneStaleTracks(uint32_t now_ms) {
    // Prune expired tracks on a wall-clock basis, regardless of camera state.
    for (uint8_t t = 0; t < Config::VISION_MAX_PERSISTENCE_TRACKS; ++t) {
        if (tracks_[t].active) {
            if ((now_ms - tracks_[t].last_seen_ms) > Config::PERSISTENCE_TIMEOUT_MS) {
                tracks_[t].active = false;
                setTelemetryEvent(TE_VISION_TRACK_EXPIRED);
            }
        }
    }
}

void VisionPipeline::updatePersistenceTracks(
    const VisionBlob* blobs,
    uint8_t blob_count,
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
) {
    const VisionMarkerProfile& active_profile =
        (active_profile_type_ == Types::VisionMarkerType::ON_GROUND_MINE) ? profile_on_ground_ : profile_buried_;

    // 1. Fuse new blob detections into persistence tracks
    for (uint8_t b = 0; b < blob_count; ++b) {
        if (!blobs[b].valid) continue;

        float world_x = 0.0f;
        float world_y = 0.0f;
        bool proj_valid = false;
        projectToWorld(blobs[b].centroid_x, blobs[b].centroid_y, fused_altitude_m, drone_pose, attitude, world_x, world_y, proj_valid);

        // If projection was invalid due to excessive tilt, skip fusing this blob
        // into any track entirely for this frame — decouple "we logged the fault"
        // from "we still used the bad data."
        if (!proj_valid) {
            continue;
        }

        // Confidence calculation formula
        float normalized_area_score = std::min(40.0f, 40.0f * (blobs[b].area / static_cast<float>(active_profile.expected_marker_area_px)));
        float confidence = std::min(60.0f, blobs[b].circularity * 60.0f) + normalized_area_score + active_profile.confidence_bias;
        if (confidence > 100.0f) confidence = 100.0f;
        if (confidence < 0.0f) confidence = 0.0f;

        if (confidence < Config::CONFIDENCE_REPORT_MIN) {
            continue;
        }

        // Check distance match with existing persistence tracks.
            // Skip tracks already claimed this frame (frame_id == frame_count_)
            // so no track is double-claimed by multiple blobs in the same frame.
            int match_idx = -1;
            float min_dist_sq = Config::PERSISTENCE_RADIUS_M * Config::PERSISTENCE_RADIUS_M;

            for (uint8_t t = 0; t < Config::VISION_MAX_PERSISTENCE_TRACKS; ++t) {
                if (tracks_[t].active && tracks_[t].marker_type == active_profile_type_ &&
                    tracks_[t].frame_id != frame_count_) {
                    float dx = tracks_[t].world_x - world_x;
                    float dy = tracks_[t].world_y - world_y;
                    float dist_sq = dx * dx + dy * dy;

                    if (dist_sq <= min_dist_sq) {
                        min_dist_sq = dist_sq;
                        match_idx = t;
                    }
                }
            }

        if (match_idx >= 0) {
            // Fuse existing track
            tracks_[match_idx].world_x = 0.7f * tracks_[match_idx].world_x + 0.3f * world_x;
            tracks_[match_idx].world_y = 0.7f * tracks_[match_idx].world_y + 0.3f * world_y;
            tracks_[match_idx].pixel_x = blobs[b].centroid_x;
            tracks_[match_idx].pixel_y = blobs[b].centroid_y;
            tracks_[match_idx].average_confidence = 0.6f * tracks_[match_idx].average_confidence + 0.4f * confidence;
            tracks_[match_idx].circularity = 0.6f * tracks_[match_idx].circularity + 0.4f * blobs[b].circularity;
            tracks_[match_idx].area = 0.6f * tracks_[match_idx].area + 0.4f * blobs[b].area;
            tracks_[match_idx].normalized_area_score = normalized_area_score;
            tracks_[match_idx].persistence_count++;
            tracks_[match_idx].last_seen_ms = now_ms;
            tracks_[match_idx].frame_id = frame_count_;
            setTelemetryEvent(TE_VISION_TRACK_FUSED);
        } else {
            // Allocate new track
            for (uint8_t t = 0; t < Config::VISION_MAX_PERSISTENCE_TRACKS; ++t) {
                if (!tracks_[t].active) {
                    tracks_[t].track_id = next_track_id_++;
                    tracks_[t].world_x = world_x;
                    tracks_[t].world_y = world_y;
                    tracks_[t].pixel_x = blobs[b].centroid_x;
                    tracks_[t].pixel_y = blobs[b].centroid_y;
                    tracks_[t].average_confidence = confidence;
                    tracks_[t].circularity = blobs[b].circularity;
                    tracks_[t].area = blobs[b].area;
                    tracks_[t].normalized_area_score = normalized_area_score;
                    tracks_[t].persistence_count = 1;
                    tracks_[t].last_seen_ms = now_ms;
                    tracks_[t].marker_type = active_profile_type_;
                    tracks_[t].frame_id = frame_count_;
                    tracks_[t].active = true;
                    setTelemetryEvent(TE_VISION_TRACK_CREATED);
                    break;
                }
            }
        }
    }

    // 3. Populate stable candidates exceeding persistence threshold
    candidate_count_ = 0;
    for (uint8_t t = 0; t < Config::VISION_MAX_PERSISTENCE_TRACKS; ++t) {
        if (tracks_[t].active && tracks_[t].persistence_count >= Config::PERSISTENCE_COUNT_MIN) {
            if (candidate_count_ < Config::VISION_MAX_CANDIDATES) {
                candidates_[candidate_count_].pixel_x = tracks_[t].pixel_x;
                candidates_[candidate_count_].pixel_y = tracks_[t].pixel_y;
                candidates_[candidate_count_].world_x = tracks_[t].world_x;
                candidates_[candidate_count_].world_y = tracks_[t].world_y;
                candidates_[candidate_count_].confidence = tracks_[t].average_confidence;
                candidates_[candidate_count_].circularity = tracks_[t].circularity;
                candidates_[candidate_count_].area = tracks_[t].area;
                candidates_[candidate_count_].normalized_area_score = tracks_[t].normalized_area_score;
                candidates_[candidate_count_].persistence_count = tracks_[t].persistence_count;
                candidates_[candidate_count_].marker_type = tracks_[t].marker_type;
                candidates_[candidate_count_].frame_id = tracks_[t].frame_id;
                candidates_[candidate_count_].timestamp_ms = tracks_[t].last_seen_ms;
                candidate_count_++;
            }
        }
    }

    if (candidate_count_ > 0) {
        setTelemetryEvent(TE_VISION_CANDIDATE_REPORTED);
    }
}


// ============================================================================
// MAIN PIPELINE UPDATE DISPATCHER
// ============================================================================

void VisionPipeline::update(
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
) {
    uint32_t start_us = Hal::hal_micros();
    last_process_time_ms_ = now_ms;

    // Prune stale tracks on a wall-clock basis, even when no frame is retrieved.
    pruneStaleTracks(now_ms);

    Hal::CameraFrame frame;
    if (!Hal::hal_camera_get_frame(frame)) {
        frame_timeout_count_++;
        if ((now_ms - last_frame_time_ms_) > Config::CAMERA_STALL_TIMEOUT_MS) {
            camera_healthy_ = false;
            setTelemetryEvent(TE_VISION_FRAME_TIMEOUT);
        }
        // Safety net: refuse to return candidates when camera is unhealthy
        // so callers can't act on frozen, stale data.
        return;
    }

    last_frame_time_ms_ = now_ms;
    camera_healthy_ = true;
    frame_count_++;

    const VisionMarkerProfile& active_profile =
        (active_profile_type_ == Types::VisionMarkerType::ON_GROUND_MINE) ? profile_on_ground_ : profile_buried_;

    // Execute vision pipeline stages
    segmentHsvMask(frame, active_profile);
    applyMorphologyCleanup();
    uint8_t blob_count = extractBlobs(active_profile);
    updatePersistenceTracks(s_extracted_blobs, blob_count, drone_pose, fused_altitude_m, attitude, now_ms);

    uint32_t end_us = Hal::hal_micros();
    processing_duration_us_ = end_us - start_us;

    if (processing_duration_us_ > 20000UL) { // Over 20ms
        setTelemetryEvent(TE_VISION_PROCESSING_SLOW);
    }
}


// ============================================================================
// MODULE FACADE FUNCTIONS
// ============================================================================

void vision_pipeline_init() {
    s_global_vision_pipeline.init();
}

void vision_pipeline_update(
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    uint32_t now_ms
) {
    Types::AttitudeSample attitude;
    attitude.valid = false;
    s_global_vision_pipeline.update(drone_pose, fused_altitude_m, attitude, now_ms);
}

void vision_pipeline_update_full(
    const Types::Pose2D& drone_pose,
    float fused_altitude_m,
    const Types::AttitudeSample& attitude,
    uint32_t now_ms
) {
    s_global_vision_pipeline.update(drone_pose, fused_altitude_m, attitude, now_ms);
}

uint8_t vision_pipeline_get_candidates(Types::VisionCandidate* out_candidates, uint8_t max_count) {
    uint8_t count = s_global_vision_pipeline.getCandidateCount();
    if (count > max_count) count = max_count;
    for (uint8_t i = 0; i < count; ++i) {
        out_candidates[i] = s_global_vision_pipeline.getCandidate(i);
    }
    return count;
}

VisionPipeline& vision_pipeline_get_instance() {
    return s_global_vision_pipeline;
}

} // namespace RobofestDrone
