"""
Simulation Configuration for Robofest Gujarat 6.0 Minefield Swarm Drone.
Calibrated for the Final Aerial Hardware Components:
- Companion MC: Seeed Studio XIAO ESP32-S3 Sense (Dual-Core @ 240MHz, 8MB PSRAM)
- Flight Controller: Matek H743-SLIM V3 (STM32H743 @ 480MHz, Dual IMU: MPU6000 + ICM-42605)
- ESC: Foxeer Reaper F4 65A 4-in-1 (100A burst, DShot600)
- Motors: Darwin 1504 2300KV
- Propellers: Gemfan Hurricane 4024
- Camera: OmniVision OV5640 5MP DVP
- Ground Distance: Holybro ST VL53L1X LiDAR (up to 4.0m range)
- Front Obstacle Avoidance: LDRobot LD06 2D LiDAR (12m range, 4500Hz)
- Battery: 4S1P LiPo (14.8V 2200mAh) / Li-ion (14.8V 2800mAh)
"""

# =============================================================================
# FIELD & ARENA GEOMETRY
# =============================================================================
FIELD_WIDTH_M = 15.0            # Total arena width along X-axis (0.0 to 15.0 m)
FIELD_LENGTH_M = 60.0           # Total arena length along Y-axis (0.0 to 60.0 m)

FIELD_X_MIN = 0.0
FIELD_X_MAX = 15.0
FIELD_Y_MIN = 0.0
FIELD_Y_MAX = 60.0

START_ZONE_Y_MIN = 0.0
START_ZONE_Y_MAX = 1.0          # Y: [0.0, 1.0] safe deployment zone

MINEFIELD_Y_MIN = 1.0
MINEFIELD_Y_MAX = 59.0          # Y: [1.0, 59.0] hazardous minefield area

EXIT_ZONE_Y_MIN = 59.0
EXIT_ZONE_Y_MAX = 60.0          # Y: [59.0, 60.0] safe extraction zone

# =============================================================================
# MINEFIELD PARAMETERS
# =============================================================================
MINE_COUNT_ESTIMATE = 40        # Expected default number of active mines
ON_GROUND_MINE_RATIO = 0.75     # ~75% visible surface mines
BURIED_MINE_RATIO = 0.25        # ~25% buried mines with surface markers
MINE_CLEARANCE_RADIUS_M = 1.0   # Mandatory 1.0 meter radial clearance
MIN_MINE_SPACING_M = 0.5        # Minimum physical distance between mine centers

# =============================================================================
# DRONE & FLEET SPECIFICATIONS (HARDWARE CALIBRATED)
# =============================================================================
DEFAULT_DRONE_COUNT = 3         # Minimum official swarm size (3 to 4 drones)
MAX_DRONE_COUNT = 4
DRONE_ALL_UP_WEIGHT_G = 420.0   # Total mass with battery, FC, ESC, and LiDARs
MISSION_ALTITUDE_M = 2.0        # Nominal flight altitude AGL
SEARCH_FORWARD_SPEED_MPS = 0.45 # Nominal lane search scan speed (Darwin 1504 2300KV)
CRUISE_SPEED_MPS = 0.85         # Repositioning & transit cruise speed
DRONE_COLLISION_RADIUS_M = 0.25 # Physical drone airframe radius (4-inch wheelbase)
UNSAFE_PROXIMITY_M = 1.0        # Minimum safe inter-drone horizontal separation

# =============================================================================
# SENSOR & NOISE MODELS (HOLYBRO VL53L1X + LDROBOT LD06)
# =============================================================================
TOF_MAX_RANGE_M = 4.0           # Holybro ST VL53L1X maximum ranging distance
TOF_NOISE_STD_M = 0.02          # Holybro VL53L1X accuracy (mm-level precision)
LIDAR_OBSTACLE_RANGE_M = 12.0   # LDRobot LD06 2D LiDAR maximum range
LIDAR_SCAN_RATE_HZ = 10.0       # LDRobot LD06 scan rate (4500Hz sample rate)
FLOW_NOISE_STD_MPS = 0.04       # Optical flow velocity noise std dev
FLOW_BIAS_DRIFT_MPS = 0.008     # Optical flow bias drift rate (dampened by Dual IMU)
DRIFT_RANDOM_WALK_STD_M = 0.015 # Random walk position drift std dev per second
LOW_TEXTURE_PROBABILITY = 0.015 # Probability of optical flow surface texture dropout
FLOW_DROPOUT_DURATION_S = 0.4   # Duration of optical flow dropouts

# =============================================================================
# VISION PIPELINE MODEL (OMNIVISION OV5640 5MP DVP)
# =============================================================================
CAMERA_H_FOV_DEG = 62.0         # OV5640 horizontal field of view
CAMERA_V_FOV_DEG = 48.0         # OV5640 vertical field of view
ON_GROUND_TRUE_POSITIVE_RATE = 0.92 # High sensitivity on OV5640 color sensor
BURIED_TRUE_POSITIVE_RATE = 0.75    # Surface marker visibility
FALSE_POSITIVE_RATE_PER_FRAME = 0.015 # Reduced false positives with OV5640 sharpness
CONFIDENCE_NOISE_STD = 4.0      # Confidence score Gaussian noise std dev

# =============================================================================
# SWARM RADIO & COMMUNICATION
# =============================================================================
RADIO_PACKET_LOSS_PROBABILITY = 0.02 # Packet drop probability
RADIO_LATENCY_MS = 20           # Inter-drone packet transmission latency
HEARTBEAT_PERIOD_S = 0.25       # 4 Hz heartbeat broadcast rate
PEER_LOST_TIMEOUT_S = 2.0       # Time before declaring disconnected peer offline

# =============================================================================
# HUMAN MODEL & INTERACTION
# =============================================================================
HUMAN_DETECTION_RANGE_M = 4.5   # Range for detecting person-at-risk
HUMAN_TRUE_POSITIVE_RATE = 0.96 # Person recognition reliability
HUMAN_CONFIDENCE_NOISE_STD = 0.04
HUMAN_MIN_SPEED_MPS = 0.3
HUMAN_NOMINAL_SPEED_MPS = 0.8
HUMAN_MAX_SPEED_MPS = 1.5

# =============================================================================
# BATTERY & POWER MANAGEMENT (4S1P 14.8V 2200-2800mAh)
# =============================================================================
BATTERY_CELL_COUNT = 4
BATTERY_FULL_V = 16.8           # 4.20V per cell
BATTERY_NOMINAL_V = 14.8        # 3.70V per cell
BATTERY_LOW_V = 14.4            # 3.60V per cell (warning return threshold)
BATTERY_CRITICAL_V = 13.6       # 3.40V per cell (mandatory emergency touchdown)
ESC_CONTINUOUS_CURRENT_A = 65.0 # Foxeer Reaper 65A continuous rating
ESC_BURST_CURRENT_A = 100.0     # Foxeer Reaper 100A burst rating
MISSION_TIME_LIMIT_S = 600.0    # 10 minutes maximum allowable duration
LANDING_SPEED_MPS = 0.20        # Safe descent touchdown rate

# =============================================================================
# SIMULATION ENGINE
# =============================================================================
SIM_DT_S = 0.05                 # Simulation time step (20 Hz)

# =============================================================================
# SCORING WEIGHTS & PENALTIES (TUNABLE ESTIMATES)
# =============================================================================
TAKEOFF_AND_ACTIVATION_WEIGHT = 10.0
COMMAND_RECOGNITION_WEIGHT = 10.0
SWARM_FORMATION_WEIGHT = 10.0
MINE_DETECTION_MAPPING_WEIGHT = 25.0
SAFE_PATH_CREATION_MARKING_WEIGHT = 20.0
SAFE_HUMAN_CROSSING_WEIGHT = 20.0
TIME_BONUS_WEIGHT = 5.0

COLLISION_OR_UNSAFE_PROXIMITY_PENALTY = -10.0
CRASH_OR_SURFACE_CONTACT_PENALTY = -20.0
CLEARANCE_VIOLATION_PENALTY = -15.0
