# legacy — 강화학습 이전의 규칙 기반 코드 (2026-10 중순)

라즈베리파이 없이 PC가 블루투스(또는 USB)로 ESP32에 직접 `<다리,관절,각도>` 명령을 보내던 시절의 코드입니다.
정해진 순서로 다리를 옮겨 계단을 오르려 했던 시도이며, 이 방식의 한계 때문에 강화학습([`../learning`](../learning))으로 넘어갔습니다.
**더 이상 유지보수하지 않습니다.**

| 파일 | 내용 |
|---|---|
| `bluetooth_pc/hexapod_sdk.py` | PC용 통신 SDK (`Link`, `Hexapod`) — 아래 스크립트가 모두 사용 |
| `bluetooth_pc/hexapod_body.py` | 다리 순기구학·역기구학, 몸체 이동 |
| `bluetooth_pc/hexapod_joy_smooth.py` | 조이스틱 연속 보행 (`--sim`으로 로봇 없이 점검 가능) |
| `bluetooth_pc/hexapod_stair_seq.py` | 정해진 순서로 계단 오르기 (`--pause`로 단계마다 멈춤) |
| `bluetooth_pc/hexapod_pose_gui.py` | 18개 관절 슬라이더 GUI, 자세 저장/불러오기 |

- 다리 치수·관절 한계는 당시 값입니다. 현재 치수는 [`../learning`](../learning)의 URDF를 따릅니다.
- `hexapod_pose_gui.py` 주석의 `hexapod_pose_fw.ino`는 공개하지 않은 초기 펌웨어입니다. 같은 명령 형식을 쓰는 [`../firmware/firmware`](../firmware/firmware)로 동작합니다.
