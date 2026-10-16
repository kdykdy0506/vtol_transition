# PX4 1.17 VTOL transition 제어 변경안

이 저장소의 `msg/`와 `src/`에는 PX4-Autopilot `release/1.17` 소스의 같은 경로에 적용할 변경 파일이 들어 있다. 작업 기준 커밋은 `f65ddce`다. 실제 항공기에 적용하기 전 SITL, bench test, tethered flight test가 필요하다. 이 코드는 비행 인증이나 실기체 안전성을 보증하지 않는다.

## 기체 구성과 적용 범위

[팀의 연말 비행 시뮬레이션](https://github.com/devkyber/PSI-vtol-team/blob/year-end-flight/year_end_flight/README.md)은 조종면이 없는 기체의 lift rotor 4개와 pusher 1개를 **비행 내내 MC mode**에서 운용한다. pusher는 MC 위치 제어기의 전진 가속을 돕고, 날개는 속도가 올라가면서 양력을 만든다. 기준 모델(7 kg, 순항 10 m/s, 무풍)은 고도 20 m에서 이륙·가속·순항·감속·착륙하는 시뮬레이션 시나리오를 완료했다. 무작위 조건 40건도 모두 착륙까지 진행했지만, 그중 2건은 airspeed 19 m/s에서 제어 여유 기준을 벗어났다. 실제 기체의 CG, 공력 모멘트, 모터 추력 곡선, 후류 등은 아직 검증되지 않았다. 이 결과는 수정 펌웨어의 시험 결과가 아니다.

PX4 Standard VTOL의 FW mode에서는 lift rotor가 정지하고 FW controller를 사용한다([PX4 v1.17 actuator 설정](https://docs.px4.io/v1.17/en/config/actuators)). 이 기체에는 조종면이 없으므로 FW mode로 전환해서는 안 된다. `VT_PSI_MC_ONLY=1`은 FW transition 요청을 거부하고 MC 제어를 유지하기 위한 보호 설정이다. 팀의 MC-only 비행에서 속도를 올리고 내리는 과정은 **PX4 mode transition이 아니라 mission speed setpoint ramp**다. 따라서 이 변경안의 Standard VTOL용 pusher S-curve, MC weight fade, stable-time 완료 판정, back-transition `FlightTaskTransition`은 MC-only 비행에서 실행되지 않는다. 팀 기체의 pusher 응답은 mission speed ramp와 `CA_R4_SLEW`로 별도 제어·검증해야 한다.

이 문서에서 MC는 multicopter, FW는 fixed-wing을 뜻한다. **forward transition**은 MC→FW mode 전환이고, **back-transition**은 FW→MC mode 전환이다. MC-only 비행의 speed ramp는 어느 쪽 mode transition도 아니다.

## 변경 파일

모든 경로는 PX4-Autopilot 소스 루트 기준이다. 이 저장소에는 아래 파일의 수정된 **전체 내용**을 싣는다.

| 경로 | 수정한 부분과 함수 |
|---|---|
| `msg/CMakeLists.txt` | `VtolTransitionDecel.msg`, `VtolTransitionStatus.msg`를 uORB 생성 목록에 등록 |
| `msg/VtolTransitionDecel.msg` | back-transition `FlightTaskTransition`의 감속 setpoint와 측정값을 전달하는 새 topic |
| `msg/VtolTransitionStatus.msg` | mode/transition 단계, 속도, pusher 추력, MC weight, 자세 오차, angular rate, 안정 시간, 감속, 조건 bit를 기록하는 새 ULog topic |
| `src/modules/flight_mode_manager/tasks/Transition/FlightTaskTransition.cpp` | 생성자에서 `VT_B_DEC_JMAX` 읽기, `activate()` 초기화, `computeBackTranstionPitchSetpoint()`의 감속 setpoint 변화 제한 및 실제 사용 값 발행 |
| `src/modules/flight_mode_manager/tasks/Transition/FlightTaskTransition.hpp` | `_param_vt_b_dec_jmax`, `_decel_sp_bt_limited`, `_decel_pub` 추가 |
| `src/modules/logger/logged_topics.cpp` | `vtol_transition_status`를 기본 ULog 목록에 20 ms 간격으로 등록 |
| `src/modules/vtol_att_control/standard.cpp` | `quintic_s_curve()`, `update_vtol_state()`, `update_transition_state()`의 pusher 및 MC weight S-curve, `update_mc_state()`의 기존 MC pusher assist 유지 |
| `src/modules/vtol_att_control/standard.h` | `_pusher_transition_start_throttle`, `_pusher_transition_initialized`, `getPusherThrottle()` 추가 |
| `src/modules/vtol_att_control/vtol_att_control_main.cpp` | `action_request_poll()`과 `vehicle_cmd_poll()`의 MC-only FW 요청 차단, `Run()`의 상태 구독, `publish_transition_status()`의 transition 중/완료 후/MC pusher assist 기록 |
| `src/modules/vtol_att_control/vtol_att_control_main.h` | 새 uORB subscription/publication/cache, `VT_PSI_MC_ONLY` 보호 조건, 로그 종료 시각 변수와 함수 선언 |
| `src/modules/vtol_att_control/vtol_att_control_params.c` | 아래 신규 파라미터 9개 정의 |
| `src/modules/vtol_att_control/vtol_type.cpp` | `isFrontTransitionCompletedBase()`의 stable-time 판정, `resetTransitionStates()`, `updateFrontTransitionStability()`, `fillTransitionStatus()` |
| `src/modules/vtol_att_control/vtol_type.h` | body angular-rate 입력, stable-time 상태 변수, parameter binding 및 status logging 함수 선언 |

## 제어 동작

### PX4 Standard VTOL transition 경로

- `Standard::update_transition_state()`에서 forward transition의 pusher throttle setpoint를 `s(x)=6x⁵−15x⁴+10x³`으로 보간한다. 시작 throttle은 transition 시작 시 저장한 값이다. `VT_PSHER_SLEW`가 최대 변화율의 기준이다. 이는 **모터 명령의 S-curve**일 뿐 기체 가속도나 실제 jerk까지 보장하지 않는다.
- airspeed가 유효하면 `VT_ARSP_BLEND`부터 `VT_ARSP_TRANS`까지 MC weight 보간에 같은 S-curve를 쓴다. airspeed가 없으면 기존 시간 기반 구간에 사용한다. 이 MC weight fade를 조종면 없는 기체에 적용해서는 안 된다.
- `VtolType::isFrontTransitionCompletedBase()`는 기존 최소 transition 시간과 airspeed 조건(airspeed sensor가 없으면 open-loop 시간)을 확인한 뒤, 설정된 roll/pitch/yaw setpoint 오차·body angular rate·수직 속도가 **연속으로 기준을 만족한 시간**을 검사한다. 하나라도 벗어나면 stable-time timer를 0으로 되돌린다. `VT_F_TR_ST_TM=0`이면 기존 PX4 완료 판정을 유지한다.
- stable-time 조건을 충족하지 못한 채 transition timeout에 도달하면 기존 `isFrontTransitionTimeout()`/quadchute 경로가 MC mode로 복귀시킨다. 기존 고도 손실·자세 한계 등의 quadchute 조건도 유지된다. 안정 조건에서 한 번 벗어났다는 이유만으로 즉시 abort하는 것은 아니며 timer를 다시 시작한다.
- `FlightTaskTransition::computeBackTranstionPitchSetpoint()`는 back-transition 감속 setpoint가 위치·속도 변화로 급변할 때 그 변화폭을 `VT_B_DEC_JMAX × dt` 이내로 제한한다. `0`이면 기존 동작이다. 실제 제어에 사용한 setpoint와 측정 감속을 `vtol_transition_decel`로 보내므로 로그의 값은 별도로 계산한 추정치가 아니다.

### 조종면 없는 PSI 기체의 MC-only 경로

- `VT_PSI_MC_ONLY=1`이면 `is_fixed_wing_requested()`가 false를 반환한다. `VEHICLE_CMD_DO_VTOL_TRANSITION`의 FW 요청은 `DENIED` ACK로 거부하고 내부 `action_request`의 FW 요청도 무시한다. 설정할 때 이미 FW mode라면 다음 제어 주기부터 MC 복귀 경로로 이동한다. offboard mission에서도 FW transition 명령을 보내지 않아야 한다.
- MC 위치·속도·자세·angular-rate 제어가 lift rotor에 계속 작동하고, 기존 `VT_FWD_THRUST_EN`/`VT_FWD_THRUST_SC`가 pusher를 제어한다. 무장 상태에서 pusher가 작동하거나 ground speed가 0.5 m/s를 넘는 동안, 그리고 종료 후 `VT_TR_LOG_DUR` 동안 status topic을 발행한다. `elapsed_time`은 이 MC pusher-assist 구간의 시작부터 측정하고, `decel_actual`은 측정한 수평 감속이다. FW transition의 target airspeed와 back-transition의 감속 setpoint는 이 경로에서 적용되지 않아 NaN으로 기록한다.
- 이 경로에는 FW transition 완료 판정이나 `FlightTaskTransition`이 없다. 팀 시뮬레이션과 같은 speed ramp는 companion mission node에서 구현해야 하며, 감속 시 jerk도 mission setpoint에서 제한해야 한다. `VT_B_DEC_JMAX`는 MC-only 감속에 적용되지 않는다.

## 새 파라미터

기본값이 `0`인 비활성화 파라미터는 기존 PX4 동작을 유지한다. roll/pitch/yaw 오차와 angular-rate threshold는 stable-time 판정을 켰을 때 사용한다.

| 이름 | 기본값 | 단위 | 기능 |
|---|---:|---|---|
| `VT_PSI_MC_ONLY` | 0 | bool | 1이면 FW transition 요청 차단, MC-only 보호 |
| `VT_F_TR_ST_TM` | 0 | s | forward transition의 stable-time 기준 |
| `VT_F_TR_ROLL` | 10 | deg | roll setpoint 오차 한도, 0이면 해당 축 조건 생략 |
| `VT_F_TR_PITCH` | 10 | deg | pitch setpoint 오차 한도, 0이면 해당 축 조건 생략 |
| `VT_F_TR_YAW` | 20 | deg | yaw setpoint 오차 한도, 0이면 해당 축 조건 생략 |
| `VT_F_TR_RATE` | 45 | deg/s | 세 축 body angular rate의 최대값 한도, 0이면 생략 |
| `VT_F_TR_VZ` | 2 | m/s | NED 수직 속도 절댓값 한도, 0이면 생략 |
| `VT_B_DEC_JMAX` | 0 | m/s³ | Standard VTOL back-transition 감속 setpoint의 변화율 한도 |
| `VT_TR_LOG_DUR` | 3 | s | transition 또는 MC pusher assist 종료 후 추가 status logging 시간 |

기존 관련 파라미터: `VT_PSHER_SLEW`(Standard VTOL forward transition의 pusher 변화율), `VT_F_TRANS_THR`(forward transition의 pusher throttle 목표), `VT_ARSP_BLEND`/`VT_ARSP_TRANS`(airspeed 기반 blending), `VT_FWD_THRUST_EN`/`VT_FWD_THRUST_SC`(MC pusher assist), `CA_R4_SLEW`(팀 기체의 pusher actuator slew rate).

## uORB topics, fields, 내부 상태

- `vtol_transition_status`: `phase`(1 forward transition, 2 back-transition, 3 MC, 4 FW), `elapsed_time`, `airspeed`, `target_airspeed`, `pusher_throttle`, `mc_weight`, `roll/pitch/yaw`, `roll_error/pitch_error/yaw_error`, `roll_rate/pitch_rate/yaw_rate`, `vertical_velocity`, `stable_time`, `decel_sp`, `decel_actual`, `condition_flags`, `timestamp`. 자세는 rad, angular rate는 rad/s, 속도는 m/s, 감속은 m/s²다. 측정값이 없으면 NaN으로 기록한다.
- `vtol_transition_decel`: `timestamp`, `decel_sp`, `decel_actual`. back-transition `FlightTaskTransition`의 실제 제어 setpoint를 `vtol_att_control`에 전달한다. 수신값이 0.5 s보다 오래되면 status topic에 NaN을 기록한다.
- `condition_flags`: `CONDITION_MIN_TRANSITION_TIME`, `AIRSPEED_VALID`, `TRANSITION_AIRSPEED`, `OPENLOOP_TIME`, `ROLL_ERROR`, `PITCH_ERROR`, `YAW_ERROR`, `ANGULAR_RATE`, `VERTICAL_VELOCITY`, `STABLE_TIME`, `READY_TO_COMPLETE`, `FW_INHIBITED` bit. `FW_INHIBITED`는 MC-only 보호 설정이 켜졌음을 뜻한다.
- 주요 추가 내부 변수: `_front_transition_stable_start`, `_front_transition_stable_time`, `_front_transition_condition_flags`, `_vehicle_angular_velocity`, `_pusher_transition_start_throttle`, `_pusher_transition_initialized`, `_decel_sp_bt_limited`, `_vtol_transition_decel`, `_transition_status_log_until`, `_last_transition_status_mode`, `_mc_assist_start_timestamp`. 모두 위 표의 해당 클래스에 선언된다.

## 기체 설정 및 확인 순서

1. PX4 1.17 Standard VTOL airframe에서 lift rotor 4개와 pusher 1개를 올바르게 할당하고 control surface servo 수를 0으로 설정한다. 팀 시뮬레이션의 시작값은 `CA_AIRFRAME=2`, `VT_TYPE=2`, `CA_ROTOR_COUNT=5`, `CA_SV_CS_COUNT=0`, `VT_FWD_THRUST_EN=4`, `VT_PITCH_MIN=0`, `VT_PSI_MC_ONLY=1`이다. 실제 CG와 pusher 추력 방향을 먼저 확인한다.
2. `VT_FWD_THRUST_SC`는 **실측한** 최대 pusher 추력으로 계산한다. 팀 시뮬레이션의 2.75는 가정값이다. 날개가 과도한 양력을 내지 않도록 mission airspeed를 제한한다. 팀 시뮬레이션에서는 날개가 중량의 40% 이상을 지지한 조건 중 일부가 제어 여유 기준을 벗어났다.
3. mission에서 VTOL FW transition 명령과 FW 경로를 제거하고, MC velocity setpoint를 가속도 한도 안에서 점진적으로 올리고 내린다. RC transition switch와 자동 mission/RTL 경로도 확인한다. 보호 설정은 FW 요청을 차단하지만 상위 mission의 실패 처리까지 대신하지 않는다.
4. SITL에서 `listener vtol_transition_status`와 ULog의 같은 topic을 확인하고 FW 명령 거부 ACK 및 MC 제어 지속을 확인한다. Standard VTOL 기능은 조종면이 있는 별도 모델에서 S-curve, stable-time timer reset, timeout quadchute, back-transition jerk 한도를 검증한다.
5. 실기체에서는 모터 추력 곡선·CG·전류·온도·비상 차단을 bench test로 확인한 뒤 tethered hover, 저속 pusher test, 단계적 속도 증가 순서로 시험한다.

## 검증 상태

`git diff --check`, PX4 SITL의 `modules__vtol_att_control` 및 `modules__flight_mode_manager` 빌드, `logged_topics.cpp` syntax check를 통과했다. 전체 logger target 빌드는 이 소스 복사본의 생성 파일 `src/modules/uxrce_dds_client/dds_topics.h`가 없어 중단됐다. 수정 펌웨어로 전체 SITL flight test나 실기체 비행 시험은 수행하지 않았다. 팀의 공개 시뮬레이션은 기존 PX4 v1.17.0 MC-only 구성의 결과이며, **이 수정 펌웨어의 검증 결과가 아니다**.
