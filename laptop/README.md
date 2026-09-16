# GRISE Laptop Node

역할은 **외부 USB 카메라 송신 + 모니터링**뿐입니다. 판단/경로계획/모터제어는 하지 않습니다.

## 실행

```bash
python -m venv .venv
# Windows: .venv\\Scripts\\activate
pip install -r requirements.txt
python main.py
```

Raspberry Pi는 `http://<노트북IP>:8080/stream.mjpg`를 읽습니다.
Pi의 상태 JSON은 UDP 9001로 이 노트북에 들어옵니다.

초기 카메라 값은 기존 grise 코드와 동일하게 1280x720, 30 FPS, CAMERA_INDEX=0 입니다.

## 설정과 상태

`laptop/config.py`의 기본값을 사용하며 아래 환경변수로 변경할 수 있습니다.

| 환경변수 | 기본값 |
|---|---|
| `GRISE_CAMERA_INDEX` | `0` |
| `GRISE_FRAME_WIDTH`, `GRISE_FRAME_HEIGHT` | `1280`, `720` |
| `GRISE_CAMERA_FPS`, `GRISE_JPEG_QUALITY` | `30`, `80` |
| `GRISE_CAMERA_RETRY_S`, `GRISE_CAMERA_FRAME_STALE_S` | `1.0`, `0.5` 초 |
| `GRISE_MJPEG_HOST`, `GRISE_MJPEG_PORT` | `0.0.0.0`, `8080` |
| `GRISE_MONITOR_BIND_HOST`, `GRISE_MONITOR_UDP_PORT` | `0.0.0.0`, `9001` |
| `GRISE_MONITOR_STALE_S` | `1.5` 초 |

Pi가 이 노트북의 IP로 접속하므로 Laptop에 Pi IP 설정은 필요하지 않습니다. 카메라 프레임이 없으면 `/health`는 `{"ok":false}`, `/snapshot.jpg`와 `/stream.mjpg`는 HTTP 503을 반환합니다. 카메라 재연결 후 기존 URL에서 다시 수신할 수 있습니다. 스트림을 받던 중 카메라가 끊기면 연결을 닫으므로 Pi가 재접속해야 합니다.

화면은 현재 노트북 카메라 상태와 Pi의 최신 상태를 표시합니다. Pi UDP가 `GRISE_MONITOR_STALE_S` 동안 오지 않으면 Pi를 OFFLINE으로 표시하고 이전 정보를 숨깁니다. Pi 패킷에 없는 threat direction 등은 `UNKNOWN`/`N/A`로 표시합니다. `q`, Esc, 창 닫기 또는 Ctrl+C로 종료합니다.

로컬 테스트: 작업공간 루트에서 `python -B -m unittest discover -s tests -v`. 가짜 카메라·루프백 UDP 검증이며 실제 USB 카메라, Wi-Fi, Raspberry Pi 검증은 별도입니다.
