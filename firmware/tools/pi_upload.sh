#!/usr/bin/env bash
# 라즈베리파이에서 ESP32 펌웨어 컴파일 + 업로드 (UART /dev/ttyAMA0, EN·BOOT 를 파이 GPIO 로 제어)
#
#   배선: Pi GPIO23 → ESP32 EN,  Pi GPIO24 → ESP32 BOOT(GPIO0),  Pi UART(ttyAMA0) ↔ ESP32 UART0
#   ttyAMA0 에는 DTR/RTS 가 없어서 esptool 이 스스로 업로드 모드로 못 바꾼다 → 이 스크립트가 GPIO 로 해 준다.
#
#   firmware/tools/pi_upload.sh                 # firmware/firmware 를 컴파일해서 올림
#   firmware/tools/pi_upload.sh --reset         # 리셋만 (펌웨어 다시 시작)
#   SKETCH=firmware/tools/MotorCheck firmware/tools/pi_upload.sh   # 다른 스케치
#
#   환경 변수: PORT(/dev/ttyAMA0) FQBN(esp32:esp32:esp32) EN_GPIO(23) BOOT_GPIO(24) UPLOAD_SPEED(460800)
#   ★ GPIO 가 ESP32 핀에 바로 연결돼 있다고 가정 (LOW = 누름). 트랜지스터로 뒤집혀 있으면 dl/dh 를 바꿀 것.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SKETCH="${SKETCH:-$ROOT/firmware/firmware}"
PORT="${PORT:-/dev/ttyAMA0}"
FQBN="${FQBN:-esp32:esp32:esp32}"
EN="${EN_GPIO:-23}"
BOOT="${BOOT_GPIO:-24}"
SPEED="${UPLOAD_SPEED:-460800}"
FULL_FQBN="$FQBN:UploadSpeed=$SPEED"     # 컴파일과 업로드에 같은 FQBN 을 써야 빌드 결과를 찾는다
LIBS=()
[ -d "$HOME/Arduino/libraries" ] && LIBS=(--libraries "$HOME/Arduino/libraries")   # Arduino IDE 로 깐 라이브러리

command -v pinctrl >/dev/null || { echo "pinctrl 이 없음 (Raspberry Pi OS 의 raspi-utils). sudo apt install raspi-utils"; exit 1; }
command -v arduino-cli >/dev/null || { echo "arduino-cli 가 없음. 설치: curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh"; exit 1; }

pin() { local g="$1"; shift; pinctrl set "$g" "$@"; }   # pin 24 op dl = 출력 LOW (누름), pin 24 ip pu = 입력 + 풀업 (놓음)
release() { pin "$BOOT" ip pu; pin "$EN" ip pu; }
reset_run() {                              # BOOT 놓은 채 리셋 → 펌웨어 실행
  pin "$BOOT" ip pu
  pin "$EN" op dl; sleep 0.15; pin "$EN" ip pu
}
download_mode() {                          # BOOT 누른 채 리셋 → ROM 업로드 모드에서 기다림
  pin "$BOOT" op dl
  pin "$EN" op dl; sleep 0.15; pin "$EN" ip pu
  sleep 0.3
  pin "$BOOT" ip pu                       # ROM 이 이미 업로드 모드로 들어갔으므로 놓아도 됨
}
trap release EXIT

if [ "${1:-}" = "--reset" ]; then
  reset_run
  echo "ESP32 리셋함"
  exit 0
fi

if pgrep -f robot_main.py >/dev/null; then
  echo "robot_main.py 가 $PORT 를 쓰는 중 → 끔"
  pkill -f robot_main.py || true
  sleep 1
fi

echo "[1/3] 컴파일: $SKETCH ($FQBN)"
arduino-cli compile --fqbn "$FULL_FQBN" "${LIBS[@]}" "$SKETCH"

echo "[2/3] 업로드 모드로 (BOOT=GPIO$BOOT, EN=GPIO$EN)"
download_mode

echo "[3/3] 업로드: $PORT @ $SPEED"
if ! arduino-cli upload -p "$PORT" --fqbn "$FULL_FQBN" "$SKETCH"; then
  echo "업로드 실패. 느린 속도로 다시: UPLOAD_SPEED=115200 $0"
  exit 1
fi

reset_run
echo "완료: 펌웨어를 실행했음. 확인: python3 -m serial.tools.miniterm $PORT 115200 (Ctrl+] 로 종료)"
