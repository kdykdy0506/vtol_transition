# PSI VTOL: MC-only lift + pusher 제어 변경안

이 저장소는 PX4-Autopilot `release/1.17`의 커밋 `f65ddce`를 기준으로 작성한 **검증 전 펌웨어 변경안**이다. `msg/`와 `src/`의 파일은 PX4 소스 루트에서 같은 상대 경로에 적용한다. 이 저장소만 내려받아서는 빌드하거나 비행할 수 없다. 실제 기체에 적용하기 전 SITL 통합 시험, actuator bench test, tethered hover test가 필요하다.

## 팀이 말하는 transition

[팀의 연말 비행 시뮬레이션](https://github.com/devkyber/PSI-vtol-team/tree/year-end-flight/year_end_flight)은 조종면이 없는 lift rotor 4개와 단방향 pusher 1개를 사용한다. **비행 전 구간에서 PX4 mode는 MC로 유지한다.** 팀의 transition은 PX4의 MC→FW mode transition이 아니라 다음 임무 단계다.

1. **MC lift:** lift rotor가 자세와 고도를 제어하고 pusher는 꺼져 있다.
2. **MC + pusher:** MC velocity controller가 전진 가속을 위해 요구한 nose-down pitch를 기존 PX4 `pusher_assist()`가 pusher thrust로 바꾼다. MC attitude/rate controller와 lift rotor는 roll·yaw·고도 제어를 계속한다. 방향 전환도 MC 제어가 담당한다.
3. **MC return/descent:** mission velocity setpoint를 낮추면 pusher demand가 감소한다. pitch-up 제동이 요구되면 pusher command를 즉시 0으로 내린다. pusher는 역추력을 낼 수 없으므로 필요한 제동은 MC의 **pitch-up**으로 수행한다. 이후 MC로 하강·착륙한다.

```text
mission velocity setpoint → MC position/velocity controller → pitch demand
    forward pitch demand < 0 → level attitude setpoint + pusher throttle
    braking pitch demand > 0 → MC pitch-up + pusher 감소
    roll / yaw / vertical thrust → MC controller와 lift rotor 계속 사용
```

PX4 Standard VTOL의 FW mode는 lift rotor를 정지시킨다([PX4 v1.17 actuator 설명](https://docs.px4.io/v1.17/en/config/actuators)). 조종면이 없는 이 기체에는 FW mode가 안전한 비행 상태가 아니므로 `VT_PSI_MC_ONLY=1`에서 FW transition 요청을 거부한다. 기존 제안에 있던 **FW 전환 전용 MC weight fade, FW 완료 조건, FW back-transition FlightTask**는 이 기체에서 사용하지 않으므로 이 변경안에서 제거했다. MC control weight는 1로 유지한다.

팀 시뮬레이션의 기준 모델(7 kg, 순항 10 m/s, 무풍)은 고도 20 m의 이륙·가속·순항·감속·착륙 시나리오를 완료했다. 무작위 조건 40건도 모두 착륙했지만, 그중 2건은 airspeed 19 m/s에서 날개 양력이 중량의 59%가 되어 제어 여유 기준을 벗어났다. 이는 **기존 PX4 설정의 시뮬레이션 결과**이며 이 변경 펌웨어의 시험 결과가 아니다. 실제 CG, 공력 모멘트, motor thrust curve, propwash는 미검증이다.

## MC-only 제어 알고리즘

`VtolType::pusher_assist()`는 PX4의 기존 pitch-to-pusher 변환을 사용한다. MC-only에서는 **velocity control과 climb-rate control이 모두 활성화된 때만** 이 변환을 적용하고, forward pitch 하한을 0 rad로 고정한다. MC velocity controller의 pitch demand가 음수이면 heading frame에서 다음 throttle 목표를 만든 뒤 attitude setpoint를 수평에 가깝게 다시 구성한다. 이 범위 밖의 manual attitude/rate control까지 수평 자세로 강제하지는 않는다.

```text
pusher_target = clamp(-sin(requested_pitch) × VT_FWD_THRUST_SC, 0, 0.9)
```

이 식은 **normalized motor command** 계산이다. 실제 추력이나 기체 가속도를 직접 측정·제어하는 식이 아니다. `VT_FWD_THRUST_SC`는 pusher의 실측 최대 추력과 기체 중량으로 보정해야 한다. pitch demand가 양수인 제동 구간에서는 이 변환이 pitch-up을 막지 않는다.

`Standard::update_mc_state()`는 MC mode에서만 이 목표를 받아 `VT_PSI_P_SLEW`(throttle 변화율)와 `VT_PSI_P_JMAX`(그 변화율의 변화 한도)로 pusher command를 제한한다. 정상적인 목표 추종 중에는 **rate/jerk-limited command**지만 안전 차단·pitch-up 제동·목표 반전 시에는 rate를 즉시 재설정할 수 있다. 따라서 고정된 두 throttle 값 사이의 정확한 quintic S-curve 또는 실제 항공기 jerk 보장은 아니다. `CA_R4_SLEW`는 actuator 경로의 별도 제한이다.

**mission speed ramp는 펌웨어가 대신 만들지 않는다.** 팀 시뮬레이션처럼 companion mission node가 velocity setpoint를 가속도 한도 안에서 점진적으로 올리고 내려야 한다. 이 저장소에는 해당 mission node가 없으므로 통합되지 않았다. setpoint를 갑자기 바꿔도 pusher command 제한은 작동하지만, mission-level 속도/감속 제한을 보장하지는 않는다.

| 팀의 단계 | companion mission node의 책임 | 이 펌웨어의 동작 |
|---|---|---|
| MC lift | 이륙 위치·고도 setpoint, 전진 속도 0 | MC lift rotor 제어, pusher 0 |
| MC + pusher 가속/turn | 가속도 한도 내 velocity setpoint와 진행 방향 yaw 명령 | 음수 pitch demand를 pusher target으로 변환, MC 자세 제어 지속 |
| MC + pusher 순항 | airspeed·경로·yaw 오차 감시와 제한 | pusher target 추종, stable flag 기록 |
| MC return | velocity setpoint를 점진적으로 0으로 감소 | pusher 감소·pitch-up 제동 허용, MC 제어 지속 |
| MC descent | 속도·pusher가 낮아진 것을 확인한 뒤 LAND 명령 | MC로 하강하며 안전 조건에 따라 pusher 차단 |

companion은 `PHASE_INHIBITED`, airspeed 누락/초과, 지속적인 velocity error를 감시해 임무를 중단하거나 안전한 MC 경로로 복귀해야 한다. status topic만으로 자동 임무 중단이 실행되지는 않는다. pusher thrust는 기수 방향이므로 yaw가 진행 방향을 향하도록 별도 제어·검증해야 한다.

## 안전 제한과 상태 판정

- `VT_PSI_MC_ONLY=1`은 내부 `action_request`와 MAVLink `VEHICLE_CMD_DO_VTOL_TRANSITION`의 FW 요청을 차단한다. 외부 명령에는 `DENIED` ACK를 보낸다. FW 요청 경로 전체에 대한 임무 통합 감사까지 대신하지는 않는다.
- disarmed, landed/ground contact, stale MC attitude setpoint 또는 attitude estimate(200 ms), PX4의 `fixed_wing_system_failure`, pusher 설정 비활성, velocity/climb-rate control 비활성에서는 pusher command를 0으로 한다. stale setpoint가 새로 발행되지 않아도 모듈이 실행되면 `fill_actuator_outputs()`에서 pusher를 끈다.
- `VT_PSI_ARSP_MAX`가 0보다 크면 유효한 calibrated airspeed가 없거나 한도 이상일 때 pusher를 끈다. 기본값 12 m/s는 팀의 10 m/s 시뮬레이션 순항 속도보다 높게 둔 **검토용 시작값**이며 실제 기체의 안전 한도가 아니다. `0`은 이 보호를 해제하므로 bench/SITL 외에서는 사용하지 않는다.
- 안전 조건이 발동하면 pusher command는 rate limit을 기다리지 않고 0이 된다. 그러나 downstream actuator 설정과 motor 관성 때문에 **실제 추력이 즉시 0이 되는 것은 아니다**. MC lift/attitude 제어는 계속 유지된다.
- `VT_PSI_ST_TM`, `VT_PSI_VERR`, `VT_PSI_PITCH`, `VT_PSI_VZ_MAX`는 속도 오차, 실제 pitch, 수직 속도, pusher tracking이 연속으로 기준을 만족하는지 기록한다. 한 조건이라도 깨지면 stable timer를 리셋한다. 이는 **상태 기록용**이며 자동 mode switch·착륙·abort 명령이 아니다.
- pusher motor/ESC 고장을 직접 측정하는 feedback은 없다. `fixed_wing_system_failure` bit가 설정되면 pusher를 끄지만, 이것만으로 실제 pusher 고장 감지가 보장되지는 않는다.

## 변경 파일 및 함수

모든 경로는 PX4-Autopilot 소스 루트 기준이다. 이 저장소에는 다음 파일의 수정된 **전체 내용**을 싣는다.

| 경로 | 변경 내용 |
|---|---|
| `msg/CMakeLists.txt` | `VtolMcAssistStatus.msg`를 uORB message 생성 목록에 등록 |
| `msg/VtolMcAssistStatus.msg` | MC-only pusher assist 상태, pitch demand/command, throttle target/output/rate, airspeed, velocity error, stable time, phase/flags 정의 |
| `src/modules/logger/logged_topics.cpp` | `vtol_mc_assist_status`를 ULog 목록에 20 ms 간격으로 등록 |
| `src/modules/vtol_att_control/vtol_att_control_main.h` | `is_psi_mc_only()`, `is_fixed_wing_requested()` guard, `_param_vt_psi_mc_only` 추가 |
| `src/modules/vtol_att_control/vtol_att_control_main.cpp` | `action_request_poll()`, `vehicle_cmd_poll()`에서 FW transition 요청 차단/ACK |
| `src/modules/vtol_att_control/vtol_type.cpp` | `pusher_assist()`에서 MC-only forward pitch 하한 0, 기존 pitch→pusher 변환 유지, pusher가 억제되어도 제어 모드가 유효하면 nose-down setpoint를 수평으로 수정 |
| `src/modules/vtol_att_control/standard.h` | pusher limiter 상태 변수, uORB publication, MC-only parameter binding, `update_psi_mc_assist()` 선언 |
| `src/modules/vtol_att_control/standard.cpp` | `update_mc_state()`의 MC-only pusher 변환/제한, `update_psi_mc_assist()`의 safety gating·stable timer·phase·logging, `fill_actuator_outputs()`의 stale-input pusher cutoff |
| `src/modules/vtol_att_control/vtol_att_control_params.c` | 아래 PSI 전용 parameter 8개 정의 |

기존 FW transition 경로의 제어 수식이나 MC weight는 변경하지 않는다. 이 패치에서 `VT_PSI_MC_ONLY` 기본값은 0이므로 기존 PX4의 일반 VTOL 동작은 유지된다.

## 새 parameter

| 이름 | 기본값 | 단위 | 목적 |
|---|---:|---|---|
| `VT_PSI_MC_ONLY` | 0 | bool | 1이면 FW 요청 거부, MC-only 제어 활성 |
| `VT_PSI_P_SLEW` | 0.8 | 1/s | pusher throttle 변화율 한도 |
| `VT_PSI_P_JMAX` | 1.6 | 1/s² | pusher throttle 변화율의 변화 한도 |
| `VT_PSI_ARSP_MAX` | 12 | m/s | calibrated airspeed 한도; 0은 보호 해제 |
| `VT_PSI_ST_TM` | 1 | s | stable 상태 연속 유지 시간 |
| `VT_PSI_VERR` | 1 | m/s | 수평 velocity tracking error 한도 |
| `VT_PSI_PITCH` | 10 | deg | stable 상태의 실제 pitch 한도 |
| `VT_PSI_VZ_MAX` | 2 | m/s | stable 상태의 절대 수직 속도 한도 |

기존 필수 설정: `CA_AIRFRAME=2`, `VT_TYPE=2`, `CA_ROTOR_COUNT=5`, `CA_SV_CS_COUNT=0`, `VT_FWD_THRUST_EN=4`, `VT_FWD_THRUST_SC`(실측 기반), `VT_PSI_MC_ONLY=1`. `CA_R4_SLEW`, `THR_MDL_FAC`, `MPC_TILTMAX_AIR`, `MPC_XY_VEL_MAX`도 실제 기체에 맞춰 확인한다. `VT_FWD_THRUST_EN`의 고도/착륙 제한 모드를 선택하면 해당 조건에서도 pusher를 억제할 수 있다.

## uORB/ULog topic과 내부 상태

`vtol_mc_assist_status`는 `timestamp`, `phase`, `flags`, `requested_pitch`, `commanded_pitch`, `pusher_target`, `pusher_throttle`, `pusher_rate`, `airspeed`, `ground_speed`, `velocity_error`, `vertical_velocity`, `stable_time`을 기록한다. pitch는 rad, 속도는 m/s, normalized throttle rate는 1/s다. 측정값이 없으면 NaN이다.

`phase`는 `PHASE_HOVER`, `PHASE_ACCEL`, `PHASE_ASSIST`, `PHASE_RETURN`, `PHASE_DESCENT`, `PHASE_INHIBITED`다. **관측한 MC+pusher 상태**일 뿐 PX4의 실제 mode나 companion mission stage를 바꾸지 않는다. `flags`는 armed, airspeed 유효/한도/누락, setpoint·attitude stale, landed, PX4 failure bit, stable, velocity 유효, pusher 설정 비활성, velocity/climb-rate control 비활성을 나타낸다.

추가 내부 변수: `_psi_last_pusher_update`(limiter 시각), `_psi_pusher_rate`(현재 throttle rate), `_psi_stable_start`(stable timer), `_psi_assist_seen`(return 판정), `_psi_status`(발행 메시지), `_psi_status_pub`(uORB publication). 이들은 `Standard`에 선언된다.

## 적용·검증 순서

1. 별도 PX4 `release/1.17` checkout에 이 저장소의 `msg/`와 `src/`를 같은 경로로 적용하고 빌드한다. 기존 firmware를 덮어쓰거나 `main` branch에 바로 merge하지 않는다.
2. SITL에서 MC-only parameter, lift rotor 4개와 pusher 1개의 actuator mapping을 확인한다. `listener vtol_mc_assist_status`와 ULog에서 pitch demand가 음수일 때 commanded pitch가 0 근처이고 pusher target이 증가하는지 확인한다. MC torque/thrust 출력을 유지해야 한다.
3. FW transition 명령(외부·내부), disarm/land, stale setpoint, airspeed 누락/초과, pusher failure bit를 각각 주입하고 pusher cutoff와 MC 유지 여부를 확인한다. mission node의 가속/감속 speed ramp와 turn setpoint를 포함한 통합 SITL이 필요하다.
4. 감속에서 pusher가 0으로 줄고 MC pitch-up이 허용되는지, `PHASE_RETURN`과 stable flag가 의도대로 기록되는지 확인한다. pusher가 pitch-up 제동에 반대 추력을 내지 않는지도 검사한다.
5. 실기체의 CG, motor/ESC thrust curve, pusher 추력선과 yaw authority를 측정한다. 이후 actuator bench test → tethered hover → 저속 pusher → 단계적 속도 증가 순서로만 검증한다.

## 현재 검증 상태와 한계

`git diff --check`, 변경된 C++ 4개 파일의 syntax check, PX4 SITL의 `modules__vtol_att_control` 빌드를 통과했다. 새 uORB message와 parameter metadata도 이 빌드에서 생성됐다. 전체 SITL mission 통합 시험과 실기체 시험은 아직 수행하지 않았다.

이 문서의 안전 조건은 설계 의도이지 비행 안전 보증이 아니다. PX4의 offboard setpoint smoothing, airspeed sensor 신뢰도, pusher 고장 감지, 방향전환 중 side-slip, 날개 양력에 따른 lift rotor 여유, battery/ESC 열 한계는 이 패치만으로 해결되지 않는다. 특히 **mission speed ramp는 companion 쪽에 별도 구현해야 한다.** 새 코드의 비행 동작은 미검증이며 실기체에 바로 적용하면 안 된다.
