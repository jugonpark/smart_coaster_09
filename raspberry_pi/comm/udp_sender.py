"""
전송 계층 - {vx, vy, w, status}를 JSON으로 직렬화해 UDP로 ESP32에 전송.

왜 UDP인가
  - 제어 명령은 30Hz로 계속 새로 나온다. 하나 잃어도 33ms 뒤 최신 값이 온다.
  - TCP는 재전송/혼잡제어 때문에 오래된 명령이 뒤늦게 도착할 수 있어 오히려 위험하다.
  - 파일이 아니라 소켓으로 흘려보내므로 디스크 I/O 지연도 없다.

패킷 형식 (ESP32의 ArduinoJson이 그대로 파싱)
    {
      "seq": 1234,          // 순번. 수신측이 순서 뒤바뀜/유실을 감지하는 데 쓴다.
      "t":   1234.567,      // 송신 시각 (float, epoch)
      "vx":  12.3,          // 로봇 좌표계 전진 속도 [cm/s]
      "vy":  -4.5,          // 로봇 좌표계 좌측 속도 [cm/s]
      "w":   0.35,          // 각속도 [rad/s], 반시계 양수
      "status": "RUN"       // RUN | SLOW | STOP
    }

중요: 소켓은 논블로킹으로 둔다.
      네트워크가 막혔을 때 send가 블로킹되면 영상 루프 전체가 멈춘다.
"""

from __future__ import annotations

import json
import socket
import time
from dataclasses import asdict, dataclass

import config


@dataclass
class CommandPacket:
    seq: int
    t: float
    vx: float
    vy: float
    w: float
    status: str
    motion_id: int | None = None
    target_distance_cm: float | None = None

    def to_json(self) -> str:
        # 소수점 3자리로 잘라 패킷 크기를 줄인다 (ESP32 파싱 부담도 감소)
        d = asdict(self)
        d = {k: v for k, v in d.items() if v is not None}
        for k in ("t", "vx", "vy", "w"):
            d[k] = round(d[k], 3)
        return json.dumps(d, separators=(",", ":"))


class UdpSender:
    def __init__(self, ip: str | None = None, port: int | None = None) -> None:
        self.addr = (ip or config.ESP32_IP, port or config.ESP32_PORT)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)

        self._seq = 0
        self._min_interval = 1.0 / config.UDP_SEND_HZ
        self._last_send_t = 0.0

        self.sent_count = 0
        self.error_count = 0

        print(f"[UDP] 대상 {self.addr[0]}:{self.addr[1]}, {config.UDP_SEND_HZ}Hz")

    def send(self, vx: float, vy: float, w: float, status: str,
             force: bool = False, *, motion_id: int | None = None,
             target_distance_cm: float | None = None) -> CommandPacket | None:
        """
        전송 주기를 지키며 명령을 보낸다.
        force=True면 주기를 무시하고 즉시 보낸다 (정지 명령 등 긴급 상황용).
        실제로 보냈으면 패킷을, 주기 때문에 건너뛰었으면 None을 반환한다.
        """
        now = time.time()
        if not force and (now - self._last_send_t) < self._min_interval:
            return None

        self._seq += 1
        pkt = CommandPacket(
            seq=self._seq, t=now,
            vx=float(vx), vy=float(vy), w=float(w),
            status=status,
            motion_id=motion_id, target_distance_cm=target_distance_cm,
        )

        try:
            self._sock.sendto(pkt.to_json().encode("utf-8"), self.addr)
            self.sent_count += 1
        except (BlockingIOError, OSError) as e:
            # 논블로킹 소켓이 꽉 찼거나 라우팅이 없을 때. 다음 주기에 다시 시도한다.
            self.error_count += 1
            if self.error_count % 60 == 1:
                print(f"[UDP] 송신 실패 ({self.error_count}회): {e}")

        self._last_send_t = now
        return pkt

    def send_stop(self) -> None:
        """즉시 정지 명령. 종료 시와 예외 발생 시 여러 번 보낸다 (유실 대비)."""
        for _ in range(5):
            self.send(0.0, 0.0, 0.0, "STOP", force=True)
            time.sleep(0.01)

    def close(self) -> None:
        self.send_stop()
        self._sock.close()
        print(f"[UDP] 종료. 전송 {self.sent_count}건, 실패 {self.error_count}건")
