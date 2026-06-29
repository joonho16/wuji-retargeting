# Gen3 + MANUS teleop HOWTO

이 폴더는 이제 `wuji-retargeting` 원본을 기반으로 한 **Gen3 + MANUS + WH110 teleop용 로컬 프로젝트**입니다. Python 패키지 이름(`wuji_retargeting`)은 호환성을 위해 그대로 두고, 실행은 아래 단일 런처로 통일합니다.

## 0. 최초 1회 준비

```bash
# submodule 포함 clone 후
cd <repo>
git submodule update --init --recursive

# Gen3 실기 캘리브레이션 작업 파일 생성(ignored private file)
scripts/gen3_manus_teleop.sh init-calib
# 생성된 .local/gen3_vive_calib/real_gen3_vive_calibration.yaml 편집 후
# real_robot_calibration_reviewed: true 로 바꾸기 전까지 실기 실행은 막힘
```

MANUS SDK는 Git에 넣지 않습니다. `manus-sdk/README.md`를 보고 로컬에만 `manus-sdk/ManusSDK_v3.1.1/`를 준비하세요. SDK→UDP 브리지는 `tools/manus_bridge/`의 tracked 소스에서 런타임에 빌드됩니다.

## 1. 상태 확인

```bash
scripts/gen3_manus_teleop.sh status
scripts/gen3_manus_teleop.sh compose-config
scripts/gen3_manus_teleop.sh compose-gen3-config
```

## 2. 안전한 단계별 실행

```bash
# Gen3 VIVE fake stack / dry run
scripts/gen3_manus_teleop.sh gen3 full-fake

# MANUS UDP 한 프레임 확인(손/로봇 움직임 없음)
HAND=left MANUS_PORT=15020 scripts/gen3_manus_teleop.sh manus-check

# MANUS tuning GUI(손/로봇 움직임 없음)
HAND=left MANUS_PORT=15020 scripts/gen3_manus_teleop.sh tuning

# MANUS -> MuJoCo simulation(실물 손 움직임 없음)
HAND=left MANUS_PORT=15020 scripts/gen3_manus_teleop.sh sim
```

## 3. 기존 4터미널 실기 플로우를 한 스크립트로 실행

먼저 실제 실행될 4개 명령을 출력해서 확인하세요.

```bash
GEN3_ROBOT_IP=192.168.50.10 \
MANUS_CORE_IP=192.168.50.132 \
HAND=left \
MANUS_PORT=15020 \
CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES \
GEN3_VIVE_CALIBRATION_DIR=.local/gen3_vive_calib \
GEN3_VIVE_CALIBRATION=/calib/real_gen3_vive_calibration.yaml \
scripts/gen3_manus_teleop.sh full-real --print
```

실행 준비가 끝났고 tmux가 설치되어 있으면:

```bash
GEN3_ROBOT_IP=192.168.50.10 \
MANUS_CORE_IP=192.168.50.132 \
HAND=left \
MANUS_PORT=15020 \
CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES \
GEN3_VIVE_CALIBRATION_DIR=.local/gen3_vive_calib \
GEN3_VIVE_CALIBRATION=/calib/real_gen3_vive_calibration.yaml \
scripts/gen3_manus_teleop.sh full-real
```

이 명령은 tmux 세션 `gen3-manus-teleop`에 다음 네 창을 만듭니다.

1. `gen3`: `scripts/gen3_vive_compose_rm.sh full-real`
2. `deadman`: `scripts/gen3_vive_enter_deadman.sh`
3. `manus`: `docker compose --profile manus-sdk run --rm manus-sdk-udp-bridge-unicast`
4. `hand`: `docker compose --profile real-wh110 --profile manus run --rm --service-ports real-wh110-manus`

안전 래치:

- WH110 실물 손은 `CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES` 없이는 시작하지 않습니다.
- Gen3 실기는 `GEN3_ROBOT_IP`와 reviewed calibration 없이는 시작하지 않습니다.
- `.local/gen3_vive_calib/real_gen3_vive_calibration.yaml`은 로컬 private/ignored 파일입니다.
- `~/.wuji`와 MANUS SDK는 로컬 private asset입니다.

## 4. 개별 명령

```bash
scripts/gen3_manus_teleop.sh help
scripts/gen3_manus_teleop.sh gen3 receiver
scripts/gen3_manus_teleop.sh deadman
MANUS_CORE_IP=192.168.50.132 HAND=left scripts/gen3_manus_teleop.sh manus-bridge
HAND=left MANUS_PORT=15020 CONFIRM_REAL_WH110=I_UNDERSTAND_WH110_MOVES scripts/gen3_manus_teleop.sh hand
```

복원/개인 GitHub publish 절차는 `docs/restore-and-publish.md`를 보세요. publish/push는 사용자가 직접 수행합니다.
