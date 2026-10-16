# PX4 1.17 VTOL 천이 제어 검토안

이 저장소의 `msg/`와 `src/`는 PX4-Autopilot `release/1.17` 소스 루트에 덮어 적용하는 변경 파일이다. 작업 기준 커밋은 `f65ddce`다. 실제 항공기에 넣기 전 SITL, 벤치, 구속 비행으로 검증해야 한다. 이 코드는 비행 인증이나 실기체 안전성을 보증하지 않는다.

## 기체 개념과 적용 범위

[팀의 연말 비행 시뮬레이션](https://github.com/devkyber/PSI-vtol-team/blob/year-end-flight/year_end_flight/README.md)은 조종면이 없는 4개 리프트 로터와 1개 푸셔를 **항상 MC 모드**에서 운용한다. 푸셔는 MC 위치 제어기가 요구하는 전진 가속을 돕고, 날개 양력은 수동으로 생긴다. 시뮬레이션의 기준 기체(7 kg, 10 m/s)는 비행 프로파일을 통과했지만, 실제 기체의 CG, 공력 모멘트, 모터 추력 곡선, 후류 등은 미검증이다. 랜덤 스윕 40건 중 2건은 높은 대기속도에서 제어 여유 기준을 넘었다.

PX4 기본 Standard VTOL의 고정익 모드는 리프트 로터의 제어를 끄고 고정익 제어기에 의존한다. 조종면이 없는 본 기체에는 이 모드로 전환하면 안 된다. `VT_PSI_MC_ONLY=1`로 고정익 천이 요청을 거부하고 MC 제어를 유지한다. MC 전용 비행에서 “천이”는 고정익 모드 전환이 아니라 미션 속도 명령을 부드럽게 올리고 내리는 과정이다. 따라서 아래의 **기본 VTOL 천이 경로**용 S 곡선, MC weight 감소, 안정성 완료 판정, 역천이 FlightTask는 본 기체의 MC 전용 비행에서는 호출되지 않는다. 팀 기체의 푸셔 부드러움은 미션 속도 ramp와 `CA_R4_SLEW`를 사용한다. 이 구분은 비행 안전에 필수적이다.

## 변경 파일

모든 경로는 PX4-Autopilot 소스 루트 기준이다. 이 저장소에는 아래 파일의 수정된 **전체 내용**을 싣는다.

| 경로 | 수정한 부분과 함수 |
|---|---|
| `msg/CMakeLists.txt` | `VtolTransitionDecel.msg`, `VtolTransitionStatus.msg`를 uORB 생성 목록에 등록 |
| `msg/VtolTransitionDecel.msg` | 역천이 FlightTask의 실제 감속 목표와 측정치를 전달하는 새 토픽 |
| `msg/VtolTransitionStatus.msg` | 단계, 속도, 추력, MC 가중치, 자세/오차/각속도, 안정 시간, 감속, 조건 비트의 새 ULog 토픽 |
| `src/modules/flight_mode_manager/tasks/Transition/FlightTaskTransition.cpp` | 생성자에서 `VT_B_DEC_JMAX` 읽기, `activate()` 초기화, `computeBackTranstionPitchSetpoint()`의 목표 감속 변화 제한 및 실제 사용 값 발행 |
| `src/modules/flight_mode_manager/tasks/Transition/FlightTaskTransition.hpp` | `_param_vt_b_dec_jmax`, `_decel_sp_bt_limited`, `_decel_pub` 추가 |
| `src/modules/logger/logged_topics.cpp` | `vtol_transition_status`를 기본 ULog 목록에 20 ms 간격으로 등록 |
| `src/modules/vtol_att_control/standard.cpp` | `quintic_s_curve()`, `update_vtol_state()`, `update_transition_state()`의 푸셔 및 MC weight S 곡선, `update_mc_state()`의 기존 MC 푸셔 보조 유지 |
| `src/modules/vtol_att_control/standard.h` | `_pusher_transition_start_throttle`, `_pusher_transition_initialized`, `getPusherThrottle()` 추가 |
| `src/modules/vtol_att_control/vtol_att_control_main.cpp` | `action_request_poll()`과 `vehicle_cmd_poll()`의 MC 전용 FW 요청 차단, `Run()`의 상태 구독, `publish_transition_status()`의 천이 중/완료 후/MC 보조 비행 기록 |
| `src/modules/vtol_att_control/vtol_att_control_main.h` | 새 uORB 구독/발행/캐시, `VT_PSI_MC_ONLY` 보호 조건, 로그 종료 시각 변수와 함수 선언 |
| `src/modules/vtol_att_control/vtol_att_control_params.c` | 아래 신규 파라미터 9개 정의 |
| `src/modules/vtol_att_control/vtol_type.cpp` | `isFrontTransitionCompletedBase()`의 연속 안정 시간 판정, `resetTransitionStates()`, `updateFrontTransitionStability()`, `fillTransitionStatus()` |
| `src/modules/vtol_att_control/vtol_type.h` | 각속도 입력, 안정성 상태 변수, 파라미터 바인딩 및 상태 기록 함수 선언 |

## 제어 동작

### 기본 Standard VTOL 천이 경로

- `Standard::update_transition_state()`에서 전천이 푸셔 목표를 `s(x)=6x⁵−15x⁴+10x³`으로 보간한다. 시작 추력은 천이 시작 시 저장한 값이다. `VT_PSHER_SLEW`가 최대 변화율의 기준이 된다. 이는 **모터 명령의 S 곡선**이며 기체 가속도나 실제 jerk의 S 곡선을 보장하지 않는다.
- 대기속도가 유효하면 기존 `VT_ARSP_BLEND`부터 `VT_ARSP_TRANS`까지 MC 가중치의 보간에 같은 S 곡선을 쓴다. 대기속도가 없으면 기존 시간 기반 구간에 사용한다. 이 감소는 조종면 없는 기체에 적용하면 안 된다.
- `VtolType::isFrontTransitionCompletedBase()`는 기존 최소 천이 시간과 대기속도(또는 센서 없는 경우 open-loop 시간) 조건 뒤에, 설정된 roll/pitch/yaw 오차·몸체 각속도·수직 속도의 **연속 충족 시간**을 검사한다. 조건을 벗어나면 안정 타이머를 0으로 되돌린다. `VT_F_TR_ST_TM=0`이면 PX4 기존 완료 판정을 유지한다.
- 연속 안정 판정이 불가능한 채 천이 시간이 지나면 PX4의 기존 `isFrontTransitionTimeout()`/quadchute 경로가 MC로 복귀시킨다. 고도 손실, 자세 한계 등 기존 quadchute 조건도 그대로 작동한다. 개별 안정 조건 이탈 순간에 즉시 abort하지는 않으며 타이머를 재시작한다.
- `FlightTaskTransition::computeBackTranstionPitchSetpoint()`는 기존 감속 목표가 위치/속도 변화로 급변할 때 `VT_B_DEC_JMAX × dt` 이내로 제한한다. `0`이면 기존 동작이다. 위치 제어기에서 실제 사용한 목표와 측정 감속을 `vtol_transition_decel`로 보내므로 로그의 감속 목표는 별도로 재계산한 추정치가 아니다.

### 조종면 없는 PSI 기체의 MC 전용 경로

- `VT_PSI_MC_ONLY=1`이면 `is_fixed_wing_requested()`가 false를 반환한다. `VEHICLE_CMD_DO_VTOL_TRANSITION`의 FW 요청은 `DENIED` ACK로 거부하고 내부 `action_request`의 FW 요청도 무시한다. 설정 도중 이미 FW였다면 다음 제어 주기부터 MC 복귀 경로로 이동한다. 오프보드 미션에서도 천이 명령을 보내지 않아야 한다.
- MC 위치·속도·자세·각속도 제어가 리프트 로터에 계속 작동하고, 기존 `VT_FWD_THRUST_EN`/`VT_FWD_THRUST_SC`가 푸셔를 제어한다. 로그는 무장 중 푸셔가 작동하거나 대지속도가 0.5 m/s를 넘는 동안과, 그 뒤 `VT_TR_LOG_DUR` 동안 발행한다. `elapsed_time`은 이 MC 보조 구간의 시작부터, `decel_actual`은 측정한 수평 감속이다. FW 천이 목표속도와 역천이 목표감속은 이 경로에서 적용되지 않으므로 NaN이다.
- 이 경로에는 FW 완료 판정이나 `FlightTaskTransition`이 없다. 팀 시뮬레이션과 같은 속도 ramp는 companion 미션 노드에서 구현해야 하며, 역가속 jerk도 미션 setpoint에서 제한해야 한다. `VT_B_DEC_JMAX`는 MC 전용 역감속에는 적용되지 않는다.

## 새 파라미터

`0`이 비활성인 파라미터는 기존 PX4 동작을 유지한다. 각도와 각속도 임계값은 안정 지속시간을 활성화했을 때 사용한다.

| 이름 | 기본값 | 단위 | 기능 |
|---|---:|---|---|
| `VT_PSI_MC_ONLY` | 0 | bool | 1이면 FW 천이 요청 차단, MC 전용 보호 |
| `VT_F_TR_ST_TM` | 0 | s | 전천이 안정 조건 연속 유지 시간 |
| `VT_F_TR_ROLL` | 10 | deg | roll 설정값 오차 한도, 0이면 해당 축 조건 생략 |
| `VT_F_TR_PITCH` | 10 | deg | pitch 설정값 오차 한도, 0이면 해당 축 조건 생략 |
| `VT_F_TR_YAW` | 20 | deg | yaw 설정값 오차 한도, 0이면 해당 축 조건 생략 |
| `VT_F_TR_RATE` | 45 | deg/s | 세 축 중 최대 몸체 각속도 한도, 0이면 생략 |
| `VT_F_TR_VZ` | 2 | m/s | 절댓값 NED 수직 속도 한도, 0이면 생략 |
| `VT_B_DEC_JMAX` | 0 | m/s³ | 기본 역천이 목표 감속의 변화율 한도 |
| `VT_TR_LOG_DUR` | 3 | s | 천이 또는 MC 보조 비행 종료 후 추가 상태 기록 시간 |

기존 관련 파라미터: `VT_PSHER_SLEW`(기본 VTOL 전천이 푸셔 변화율), `VT_F_TRANS_THR`(전천이 푸셔 목표), `VT_ARSP_BLEND`/`VT_ARSP_TRANS`(대기속도 기반 혼합), `VT_FWD_THRUST_EN`/`VT_FWD_THRUST_SC`(MC 푸셔 보조), `CA_R4_SLEW`(팀 기체의 푸셔 액추에이터 slew).

## 토픽, 필드, 내부 상태

- `vtol_transition_status`: `phase`(1 전천이, 2 역천이, 3 MC, 4 FW), `elapsed_time`, `airspeed`, `target_airspeed`, `pusher_throttle`, `mc_weight`, `roll/pitch/yaw`, `roll_error/pitch_error/yaw_error`, `roll_rate/pitch_rate/yaw_rate`, `vertical_velocity`, `stable_time`, `decel_sp`, `decel_actual`, `condition_flags`, `timestamp`. 자세는 rad, 각속도는 rad/s, 속도는 m/s, 감속은 m/s²다. 없는 측정치는 NaN이다.
- `vtol_transition_decel`: `timestamp`, `decel_sp`, `decel_actual`. 역천이 FlightTask가 만든 실제 제어 목표를 `vtol_att_control`에 전달한다. 수신값이 0.5 s보다 오래되면 상태 토픽에는 NaN을 쓴다.
- `condition_flags`: `CONDITION_MIN_TRANSITION_TIME`, `AIRSPEED_VALID`, `TRANSITION_AIRSPEED`, `OPENLOOP_TIME`, `ROLL_ERROR`, `PITCH_ERROR`, `YAW_ERROR`, `ANGULAR_RATE`, `VERTICAL_VELOCITY`, `STABLE_TIME`, `READY_TO_COMPLETE`, `FW_INHIBITED` 비트. `FW_INHIBITED`는 MC 전용 보호 설정이 켜졌음을 뜻한다.
- 주요 추가 내부 변수: `_front_transition_stable_start`, `_front_transition_stable_time`, `_front_transition_condition_flags`, `_vehicle_angular_velocity`, `_pusher_transition_start_throttle`, `_pusher_transition_initialized`, `_decel_sp_bt_limited`, `_vtol_transition_decel`, `_transition_status_log_until`, `_last_transition_status_mode`, `_mc_assist_start_timestamp`. 모두 위 표의 해당 클래스에 선언된다.

## 기체 설정 및 확인 순서

1. 기존 PX4 1.17 Standard VTOL airframe에서 리프트 로터 4개와 푸셔 1개를 올바르게 할당하고, 서보 수를 0으로 설정한다. 팀 시뮬레이션의 시작값은 `CA_AIRFRAME=2`, `VT_TYPE=2`, `CA_ROTOR_COUNT=5`, `CA_SV_CS_COUNT=0`, `VT_FWD_THRUST_EN=4`, `VT_PITCH_MIN=0`, `VT_PSI_MC_ONLY=1`이다. 실제 CG와 푸셔 추력 방향을 먼저 확인한다.
2. `VT_FWD_THRUST_SC`는 **측정한** 최대 푸셔 추력으로 계산한다. 팀 시뮬레이션의 2.75는 가정값이다. 미션 속도는 대기속도에서 날개 양력이 과도해지지 않도록 제한한다. 팀 시뮬레이션에서 날개가 중량의 40% 이상을 지지한 경우 일부 시나리오가 실패했다.
3. 미션에서 VTOL FW 천이 명령과 고정익 경로를 제거하고 MC 속도 setpoint를 가속 한계 안에서 올리고 내린다. RC 천이 스위치와 자동 미션/RTL 경로도 확인한다. 보호 설정은 요청을 차단하지만, 상위 미션의 실패 처리까지 대신하지 않는다.
4. SITL에서 `listener vtol_transition_status`와 ULog의 동일 토픽을 확인하고, FW 명령 거부 ACK 및 MC 제어 지속을 확인한다. 기본 VTOL 기능 검증은 조종면을 가진 별도 모델에서 S 곡선, 안정 타이머 리셋, timeout quadchute, 역천이 jerk 한도를 검증한다.
5. 실기체에서는 모터 추력 곡선·CG·전류·온도·비상 차단을 벤치에서 확인한 뒤 구속 호버, 저속 푸셔, 단계적 속도 증가 순서로 시험한다.

## 검증 상태

`git diff --check`, PX4 SITL의 `modules__vtol_att_control` 및 `modules__flight_mode_manager` 빌드, `logged_topics.cpp` 구문 검사를 통과했다. 전체 로거 타깃 빌드는 이 소스 복사본의 생성 파일 `src/modules/uxrce_dds_client/dds_topics.h`가 없어 중단됐다. 전체 SITL 실행 및 실제 비행 시뮬레이션은 수행하지 않았다. 팀의 공개 시뮬레이션 결과는 기존 PX4 v1.17.0 MC 전용 구성의 결과이며, **이 수정 펌웨어의 시험 결과가 아니다**.
