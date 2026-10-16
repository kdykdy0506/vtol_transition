uint8 PHASE_TRANSITION_TO_FW = 1
uint8 PHASE_TRANSITION_TO_MC = 2
uint8 PHASE_MC = 3
uint8 PHASE_FW = 4

uint32 CONDITION_MIN_TRANSITION_TIME = 1
uint32 CONDITION_AIRSPEED_VALID = 2
uint32 CONDITION_TRANSITION_AIRSPEED = 4
uint32 CONDITION_OPENLOOP_TIME = 8
uint32 CONDITION_ROLL_ERROR = 16
uint32 CONDITION_PITCH_ERROR = 32
uint32 CONDITION_YAW_ERROR = 64
uint32 CONDITION_ANGULAR_RATE = 128
uint32 CONDITION_VERTICAL_VELOCITY = 256
uint32 CONDITION_STABLE_TIME = 512
uint32 CONDITION_READY_TO_COMPLETE = 1024
uint32 CONDITION_FW_INHIBITED = 2048

uint64 timestamp                       # time since system start (microseconds)

uint8 phase                            # transition phase, see PHASE_* constants
float32 elapsed_time                   # time since current transition started [s]

float32 airspeed                       # calibrated airspeed used by VTOL transition logic [m/s], NaN if unavailable
float32 target_airspeed                # transition airspeed threshold [m/s]
float32 pusher_throttle                # standard VTOL pusher throttle command [0, 1], NaN if not applicable
float32 mc_weight                      # multicopter control weight [0, 1]

float32 roll                           # vehicle roll [rad]
float32 pitch                          # vehicle pitch [rad]
float32 yaw                            # vehicle yaw [rad]
float32 roll_error                     # vehicle attitude setpoint roll error [rad]
float32 pitch_error                    # vehicle attitude setpoint pitch error [rad]
float32 yaw_error                      # vehicle attitude setpoint yaw error [rad]

float32 roll_rate                      # body roll rate [rad/s]
float32 pitch_rate                     # body pitch rate [rad/s]
float32 yaw_rate                       # body yaw rate [rad/s]
float32 vertical_velocity              # NED down velocity [m/s]

float32 stable_time                    # continuous time that all enabled front-transition completion conditions are met [s]
float32 decel_sp                       # backtransition deceleration setpoint [m/s^2]
float32 decel_actual                   # measured backtransition deceleration in horizontal velocity direction [m/s^2]

uint32 condition_flags                 # bitmask of CONDITION_* values
