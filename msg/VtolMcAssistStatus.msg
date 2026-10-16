uint8 PHASE_HOVER = 0
uint8 PHASE_ACCEL = 1
uint8 PHASE_ASSIST = 2
uint8 PHASE_RETURN = 3
uint8 PHASE_DESCENT = 4
uint8 PHASE_INHIBITED = 5

uint32 FLAG_ARMED = 1
uint32 FLAG_AIRSPEED_VALID = 2
uint32 FLAG_AIRSPEED_LIMIT = 4
uint32 FLAG_SETPOINT_STALE = 8
uint32 FLAG_ATTITUDE_STALE = 16
uint32 FLAG_LANDED = 32
uint32 FLAG_PUSHER_FAILURE = 64
uint32 FLAG_STABLE = 128
uint32 FLAG_VELOCITY_VALID = 256
uint32 FLAG_AIRSPEED_MISSING = 512
uint32 FLAG_PUSHER_DISABLED = 1024
uint32 FLAG_CONTROL_INACTIVE = 2048

uint64 timestamp
uint8 phase
uint32 flags
float32 requested_pitch
float32 commanded_pitch
float32 pusher_target
float32 pusher_throttle
float32 pusher_rate
float32 airspeed
float32 ground_speed
float32 velocity_error
float32 vertical_velocity
float32 stable_time
