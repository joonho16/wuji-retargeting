# Native Gen3 + MANUS / WH110

기존 `scripts/gen3_manus_teleop.sh full-real` 실행법을 그대로 사용합니다.
팔은 native Twist로 제어하며, MANUS 브리지와 손 리타게팅 경로를 유지합니다.
통합 실행에서는 A/B 누름 입력으로 팔과 손 출력을 함께 제어합니다.

```text
VIVE UDP → PoseStamped → 기존 상대 목표 Mapper → base 속도 오차
 → 현재 TOOL 축 변환 + angular rad/s→deg/s → /twist_controller/commands
 → PickNik Twist controller → Kortex SINGLE_LEVEL_SERVOING

MANUS Core → 기존 UDP bridge → 기존 retargeting → A/B 출력 worker → 왼쪽 WH110

조작 창: A hold → 팔·손 teleop / B hold → 저장된 ready + 손 펴기 / release → 정지 / Q → 전체 종료
```

실기 프로파일에서 MoveIt Servo는 실행하지 않습니다. Compose service 이름
`gen3-servo-real`은 기존 서비스 선택 명령과의 호환성을 위해 유지했지만 내부는
`gen3_native_stack`입니다. `wrapper-real`은 `gen3_native_teleop`입니다.
실험 폴더 `../gen3_teleop`에 의존하지 않습니다. fake 프로파일은 기존 Servo 테스트입니다.

## 실행

```bash
cd /home/joonho-koh/Project/wujihand/wuji-retargeting
GEN3_ROBOT_IP=192.168.50.10 \
MANUS_CORE_IP=<기존-Windows-MANUS-Core-IP> \
HAND=left CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES \
scripts/gen3_manus_teleop.sh full-real
```

기존 `.env.gen3-manus` 또는 `.env`에 설정해 두었다면 `scripts/gen3_manus_teleop.sh full-real`만
실행합니다. `full-real --print`도 기존처럼 네 창의 명령을 출력합니다.
`gen3` 창은 드라이버·VIVE 수신기·native mapper, `deadman`은 A/B 조작 창 실행,
`manus`는 장갑 브리지, `hand`는 손 제어입니다.

Linux 데스크톱에서 실행하고 **Gen3 + WH110 control** 창을 클릭한 뒤 사용합니다.

| 입력 | Gen3 | 왼쪽 WH110 |
|---|---|---|
| A를 누르는 동안 | VIVE 텔레옵 | MANUS 텔레옵 |
| B를 누르는 동안 | 저장된 `ready` 액션 실행 | 손가락을 편 자세로 이동 |
| 키 해제 / 포커스 상실 / A+B 동시 누름 | 정지 | 현재 관절 위치 유지 |
| Q | 정지 후 텔레옵 종료 | 정지 후 손 제어 종료 |

Enter 토글은 통합 실행에서 사용하지 않습니다. 모드 변경이나 입력 끊김 뒤에는 두 키를 모두
놓았다가 다시 누릅니다. Esc 또는 조작 창 닫기도 팔·손 출력을 정지합니다.
전체 종료는 조작 창에 포커스를 둔 상태에서 **Q**를 누릅니다. 팔·손 정지 입력을 먼저 기록하고
이 실행에 해당하는 손 제어, Gen3 스택, MANUS 브리지와 조작 창을 종료합니다.
Q 기능이 없는 기존 실행을 끌 때는 **키 해제 → `hand` Ctrl+C → `gen3` Ctrl+C →
`manus`, `deadman` Ctrl+C** 순서로 종료한 뒤 다시 실행합니다.
정상 종료 시 WH110 관절을 disable합니다. tmux detach는 프로세스를 종료하지 않습니다.

조작 창은 X11의 실제 키 상태를 50 Hz로 확인하므로 터미널 키 반복 입력에 의존하지 않습니다.
데스크톱의 `DISPLAY`와 `XAUTHORITY`를 기존 런처가 전달합니다. 창이 없는 SSH 터미널에서
실행하면 `DISPLAY`가 필요하다는 오류를 표시합니다.

같은 UDP 9000 수신기나 로봇 드라이버를 실험 폴더에서 중복 실행하지 마세요.
코드 변경 후 `full-real`은 기존처럼 Docker 이미지를 빌드합니다. 구형 이미지로 실행하면
새 native 노드가 watchdog/종료 패치 marker를 검사하고 시작을 거부합니다.

## 설정과 단위

실기 calibration 경로는 기존 `.local/gen3_vive_calib/real_gen3_vive_calibration.yaml`입니다.
이 작업 공간의 검증된 좌표축 정렬은 유지하고 아래 사용자 설정을 반영했습니다.

```yaml
linear_scale: 1.0
angular_scale: 1.0
max_linear_mps: 0.5
max_angular_radps: 1.7453292519943295  # 100 deg/s
```

A를 새로 누를 때 잡은 기준 위치 대비 트래커 병진에 `linear_scale`을 곱해 목표 위치로 사용합니다. 속도는 목표 오차에
K=1/s를 적용하며, 두 cap은 각각 3차원 벡터의 크기를 제한합니다.
calibration의 낮은 상한은 그대로 적용합니다. 새 설치의 미검토 template은 낮은 기본값을 유지합니다.

기존 `GEN3_VIVE_CALIBRATION`, `GEN3_VIVE_CALIBRATION_DIR`, deadman/status topic,
`GEN3_VIVE_LOW_PASS_ALPHA` 환경 변수는 유지됩니다. 과거 `.env`의 Twist topic이
`/servo_node/delta_twist_cmds`이면 실기 native 노드가 `/twist_controller/commands`로 전환합니다.
native `geometry_msgs/Twist`는 설치된 Kortex 0.2.3/PickNik 0.0.3의 계약에 따라
**TOOL 좌표계, linear m/s, angular deg/s**입니다. 일반 ROS SI Twist와 다릅니다.

진단 토픽은 `/gen3_vive/status`, `/gen3_vive/debug/base_twist`(base, SI),
`/gen3_vive/debug/native_twist`(TOOL, m/s·deg/s)입니다.

## 손등 제어점 오프셋

실기 calibration의 `T_tool_tracker_mount`를 native 제어 계산에 적용합니다.
`translation`은 **로봇의 실제 TCP(`end_effector_link`)에서 손등 대응점까지의 벡터**이며,
**TCP 자체의 X/Y/Z 축 기준, 단위 m**입니다. 사람이 착용한 트래커의 위치를 이 로봇 제어점에
대응시킵니다. 사람 손목까지의 거리나 로봇 base 축에서 잰 값을 그대로 넣지 않습니다.

```yaml
T_tool_tracker_mount:
  translation: [0.0, -0.070, 0.120]  # 측정값: X=0, Y=-70, Z=+120 mm
  rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

현재 측정값은 X=0, Y=-70, Z=+120mm이며, 0 오프셋은 기존 TCP 제어입니다.
좌표축·오프셋·속도 설정의 복원용 사본은
`example/config/gen3_vive_native_handback.example.yaml`에 있습니다. 실행용 `.local` 파일은
Git에서 제외되므로 복원 시 이 예제를 참고합니다.
`rotation_xyzw`는 TCP 기준 제어점 프레임의 방향입니다. 위치만 옮길 때는 identity를 사용합니다.
현재 상대 회전 매핑에서 이 고정 회전은 회전 오차 계산에서 상쇄되므로, VIVE 명령 축의 정렬은
계속 `T_planning_source.rotation_xyzw`가 담당합니다.

현재 제어점 자세는 `T_base_tool × T_tool_tracker_mount`로 계산하고 A를 누를 때 그 자세를
기준으로 잡습니다. 제어점 속도를 계산한 뒤 `v_TCP = v_control − ω × r_base`로 실제 TCP의
속도로 변환합니다. 회전 때문에 필요한 손목 이동도 포함됩니다. 오프셋에는 병진 스케일을
곱하지 않습니다. `--motion rotation`도 제어점 중심 회전을 위한 TCP 병진은 생성합니다.

보정 후 TCP 병진 속도가 상한을 넘으면 병진·회전 속도를 같은 비율로 줄여 회전 중심을
유지합니다. 키 해제·입력 만료는 두 속도를 모두 0으로 만듭니다. `workspace_limits`를 사용하면
중립 대비 제어점 이동 범위를 제한합니다. B의 저장 액션은 로봇에 저장된 경로를 사용합니다.

`--tool-frame`은 실제 native 명령의 TCP 프레임을 유지합니다. 오프셋 프레임으로 바꾸면
명령 좌표계까지 달라집니다. 상태 JSON은 `tcp`, `control_point`, `twist_origin_frame`을 구분하고,
기존 두 debug Twist 토픽은 모두 실제 TCP 원점에서의 명령을 표시합니다.

코드 변경은 기존 `full-real` 재실행의 빌드로 적용됩니다. 이후 YAML 측정값 변경은 mapper
재시작으로 읽으며, 실행 중 자동으로 기준점을 바꾸지는 않습니다.
오프셋 전용 안전 검사 6개와 기존 native 검사 15개를 로봇 연결 없이 통과했습니다.
실제 손등 대응점 측정값을 넣은 상태의 동작 확인은 아직 수행하지 않았습니다.

## B: ready 액션과 손 펴기

`GEN3_HOME_ACTION=ready`가 기본값입니다. 로봇에서 이름을 조회해 저장된 액션을 그대로
실행하며 이름의 대소문자는 구분하지 않습니다. 2026-09-17 실제 로봇의 `ready` 액션
(identifier 10009)을 읽기 전용으로 확인했습니다. 찾지 못하거나 중복이면 실행하지 않습니다.
액션의 경로와 속도는 로봇에 저장된 설정을 사용하며, 텔레옵의 TCP 속도 상한과는 별개입니다.

B를 누르면 Twist controller를 먼저 비활성화하고, 완료 응답 후 액션을 한 번 실행합니다.
B를 놓으면 같은 드라이버 세션에서 Kortex `Stop`을 호출하고 정지 응답을 확인한 뒤
Twist controller를 다시 활성화합니다. 끝까지 누르면 ready에서 대기하며 액션을 반복 실행하지 않습니다.
중간에 놓았다가 다시 B를 누르면 현재 위치에서 저장된 ready를 다시 실행합니다.
액션 API가 실패하면 키를 모두 놓아야 재시도할 수 있습니다.

액션은 Twist 제어에 사용하던 드라이버의 SDK 세션으로 실행합니다. 별도 세션을 만들면서
발생하던 `SESSION_NOT_IN_CONTROL` 오류와 세션 생성 시간 초과 경로를 제거했습니다.
드라이버는 B 입력의 만료를 직접 검사하며, Twist와 액션을 동시에 실행하지 않습니다.
조작 창이 기록한 원래 시각을 전달하므로 통신 지연이 100ms 입력 유효 시간을 늘리지 않습니다.
정지 응답이 없으면 Twist를 재활성화하지 않으며, Stop 호출이 실패하면 TCP 연결을 끊어
백그라운드 세션 유지가 액션을 계속 살려두지 않도록 합니다.

손 펴기 목표는 `example/config/wh110_open_pose.yaml`의 `left` 5×4 관절 배열(rad)입니다.
WH110 모델에서 네 손가락을 손바닥 길이 방향으로 펴고 엄지는 바깥쪽으로 편 자세를 계산했습니다.
실제 기기의 관절 한계를 읽어 목표가 범위 밖이면 거부합니다. 실물 손의 자세는 아직 검증하지 않았습니다.
B를 누르는 동안 관절별 최대 0.6rad/s로 이동하며 MANUS 프레임이 없어도 동작합니다.

통합 실행의 손 출력은 별도 스레드에서 키 상태를 확인합니다. A를 놓거나 입력이 만료되면
장치에서 읽은 현재 관절 위치로 목표를 바꿔 유지합니다. A 입력의 5Hz smoothing은 소프트웨어에서
처리하므로 해제 시 이전 필터 목표를 계속 추종하지 않습니다. A로 손 텔레옵을 시작할 때마다
첫 유효 MANUS 목표가 도착한 시점의 실제 손 자세에서 **0.5초 동안** 현재 장갑 목표로 부드럽게
연결합니다. 전환 비율은 시작과 끝에서 변화율이 0인 곡선을 사용하며, 전환 중에도 장갑 목표를
계속 갱신합니다. 중간에 A를 놓으면 즉시 현재 자세 유지로 전환하고, 다시 누르거나 입력 끊김에서
복구되면 현재 자세에서 새 0.5초 전환을 시작합니다. B 손 펴기는 기존 0.6rad/s 제한을 사용합니다.
새 A 입력마다 리타게팅의 이전 필터 상태도 비웁니다.
`GEN3_OPERATOR_STATE`가 없는 단독 `hand` 실행은 기존 동작을 유지합니다.

## 정지 처리

- VIVE/TF/deadman freshness: 100ms. 상위 노드가 0 명령을 보내고 neutral 재캡처를 요구합니다.
- A/B 입력은 `.local/gen3_operator/intent.json`을 양쪽 컨테이너에 공유합니다.
  파일을 갱신하지 못하면 100ms 후 입력이 만료되며, 양쪽 키를 해제해야 재개합니다.
- Twist controller: 마지막 명령 수신 후 100ms가 지나면 6개 출력을 모두 0으로 만듭니다.
  ROS 시간과 무관한 steady clock을 쓰므로 Python SIGKILL·DDS 단절에도 controller가 살아 있으면
  작동합니다. 활성화·timeout·NaN 이후에는 새로운 0 명령을 받아야 비영점 명령을 수락합니다.
  재개하려면 A를 놓았다가 다시 눌러 neutral을 다시 잡습니다.
- PC/드라이버 전체 종료 또는 로봇 네트워크 단절: SDK 세션의 connection inactivity timeout을
  최대 500ms로 제한합니다. 로봇 펌웨어가 세션에서 시작된 움직임을 정지시키는 기능입니다.
  물리적인 감속 완료 시간까지 100/500ms라는 의미는 아닙니다.
- controller 비활성화와 정상 드라이버 종료에서 Kortex Stop을 호출합니다. 통신 예외가 나더라도
  남은 세션·전송 계층 정리를 진행합니다. cyclic 오류는 기존처럼 latch되고 재시작이 필요합니다.

`TwistCommand.duration`은 공식 API 문서에서 미구현으로 명시되어 있으므로 watchdog에 사용하지
않습니다. [SDK 세션 timeout 정의](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_python/doc/markdown/messages/Session/CreateSessionInfo.md).

종료 `-11` 원인은 protobuf 부모가 소유한 `mutable_twist()` / `add_motor_cmd()` 결과를
드라이버가 다시 `delete`한 것입니다. 이중 해제를 제거했습니다. 내부 Robotiq 통신을 쓰지 않을
때 그리퍼 피드백 배열에 접근하는 경로도 제거했습니다. Wuji 손은 계속 별도 제어합니다.

native 경로는 MoveIt 충돌·특이점 검사와 Servo 관절별 가속도 limiter를 사용하지 않습니다.
로봇 내부 제어 제한은 적용됩니다. 키 입력 timeout은 명령 취소 기준이며 물리적인 감속 완료 시간을
보장하는 수치는 아닙니다.

## 로봇 없는 검증

아래 helper는 `--network none`에서만 실행합니다. 실제 명령 토픽으로 합성 입력을 발행하므로
host network로 실행하면 거부합니다.

```bash
docker run --rm --network none wuji-retargeting:gen3-vive-ros2-humble \
  python3 tests/helpers/gen3_twist_watchdog_runtime.py
docker run --rm --network none wuji-retargeting:gen3-vive-ros2-humble \
  python3 tests/helpers/gen3_native_ros_runtime.py
docker run --rm --network none wuji-retargeting:gen3-vive-ros2-humble \
  python3 tests/helpers/gen3_kortex_cyclic_runtime.py
docker run --rm --init --network none wuji-retargeting:gen3-vive-ros2-humble \
  xvfb-run -a python3 tests/helpers/gen3_operator_gui_runtime.py
```

2026-09-17 검증: Gen3 unit 113개, WH110 hold/model/mock-worker unit 9개 통과.
손 텔레옵의 0.5초 전환, 첫 입력 지연, 전환 중 장갑 목표 변경, 키 해제·재시작도 포함합니다.
가상 X11에서 실제 key-down/up·포커스 상실·프로세스 일시 정지·종료 검증 통과.
당시 ROS native 노드 + 합성 VIVE + 가짜 controller manager로 A 입력, B 전환,
별도 액션 세션의 연결 실패 후 키 해제·복구를 검증했습니다. 이번 수정에서는
별도 액션 세션을 제거했으며, 실물 이동 테스트는 사용자가 직접 수행합니다.

2026-09-18 수정 검증은 최소 정지 관련 항목으로 제한했습니다. Python 14개
(B 전환·정지 확인 6개, 입력 만료·재입력 3개, Q 종료 5개)와 C++ 입력 만료·중복 요청
검사를 통과했습니다. 변경한 Kortex 드라이버는 로봇 네트워크 없이 컴파일·링크했습니다.
전체 이미지 재빌드와 실물 A/B/Q 동작 확인은 기존 `full-real` 명령으로 다시 실행하면서 진행합니다.

## 영점 진단용 읽기 전용 기록

```bash
GEN3_ROBOT_IP=192.168.50.10 docker compose -f docker-compose.gen3-vive.yml \
  --profile tools run --rm -T shell python3 -m example.arm_teleop.gen3_native_snapshot
```

현재 각도·fault·저장된 Home/Zero joint action·axis offset·tool 설정을 JSON으로 출력합니다.
같은 물리적 기준에서 재부팅 전후 값을 비교할 때 사용합니다. 명령 실행이나 보정값 쓰기는
포함하지 않습니다. JSON에서 생략된 protobuf 숫자 필드는 기본값 0입니다.

2026-09-17 관측: 4번 관절 SDK 피드백 약 359.494°(−0.506°), fault 없음.
저장된 Home의 4번 관절 목표는 230°이고, Zero의 목표는 0°였습니다.
사용자가 육안으로 추정한 약 3°의 기계적 어긋남은 SDK 조회만으로 입증하거나 교정할 수 없습니다.
현재 스냅샷은 `.local/gen3_native_validation/`에 보관합니다.
