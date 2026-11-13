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

# sudo 로 전체를 돌리면 root 의 arduino-cli 설정(보드 패키지·라이브러리 없음)을 써 버린다 → 사용자로 돌리고 pinctrl 만 sudo
if [ "$(id -u)" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
  echo "sudo 없이 실행할 것:  $0   (GPIO 를 바꿀 때만 스크립트가 sudo 를 쓴다)"
  exit 1
fi

command -v pinctrl >/dev/null || { echo "pinctrl 이 없음 (Raspberry Pi OS 의 raspi-utils). sudo apt install raspi-utils"; exit 1; }
PINCTRL=(pinctrl)
if [ "$(id -u)" -ne 0 ]; then
  PINCTRL=(sudo pinctrl)               # 이 파이는 pinctrl 에 root 가 필요하다 ("Must be root")
  # 비밀번호를 물으면 여기서 한 번만. NOPASSWD 계정은 묻지 않고 넘어간다
  # (sudo -v 는 비밀번호가 필요한 규칙이 하나라도 있으면 NOPASSWD 여도 묻는다 → 먼저 -n 으로 확인)
  sudo -n true 2>/dev/null || sudo -v
fi

# arduino-cli: PATH → ~/.local/bin → ~/bin → Arduino IDE 2 안에 들어 있는 것
ARDUINO_CLI="$(command -v arduino-cli || true)"
if [ -z "$ARDUINO_CLI" ]; then
  for c in "$HOME/.local/bin/arduino-cli" "$HOME/bin/arduino-cli" \
           /opt/arduino-ide*/resources/app/lib/backend/resources/arduino-cli \
           /usr/share/arduino-ide*/resources/app/lib/backend/resources/arduino-cli \
           "$HOME"/arduino-ide*/resources/app/lib/backend/resources/arduino-cli \
           "$HOME"/Applications/arduino-ide*/resources/app/lib/backend/resources/arduino-cli; do
    if [ -x "$c" ]; then ARDUINO_CLI="$c"; break; fi
  done
fi
if [ -z "$ARDUINO_CLI" ]; then
  echo "arduino-cli 를 못 찾음. 설치 (한 번만):"
  echo "  mkdir -p ~/.local/bin && curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh"
  exit 1
fi
echo "arduino-cli: $ARDUINO_CLI"
if ! "$ARDUINO_CLI" core list 2>/dev/null | grep -q '^esp32:esp32'; then
  echo "ESP32 보드 패키지(esp32:esp32)가 없음. 설치 (한 번만, 몇 분 걸림):"
  echo "  $ARDUINO_CLI core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json"
  exit 1
fi

pin() { local g="$1"; shift; "${PINCTRL[@]}" set "$g" "$@"; }   # pin 24 op dl = 출력 LOW (누름), pin 24 ip pu = 입력 + 풀업 (놓음)
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
"$ARDUINO_CLI" compile --fqbn "$FULL_FQBN" "${LIBS[@]}" "$SKETCH"

echo "[2/3] 업로드 모드로 (BOOT=GPIO$BOOT, EN=GPIO$EN)"
download_mode

echo "[3/3] 업로드: $PORT @ $SPEED"
if ! "$ARDUINO_CLI" upload -p "$PORT" --fqbn "$FULL_FQBN" "$SKETCH"; then
  echo "업로드 실패. 느린 속도로 다시: UPLOAD_SPEED=115200 $0"
  exit 1
fi

reset_run
echo "완료: 펌웨어를 실행했음. 확인: python3 -m serial.tools.miniterm $PORT 115200 (Ctrl+] 로 종료)"
