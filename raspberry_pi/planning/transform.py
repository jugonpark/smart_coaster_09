"""
경로계산 계층 - 월드 좌표계 -> 로봇 좌표계 회전변환 + heading 제어.

월드 속도 (vx_w, vy_w)를 로봇이 이해하는 몸체 좌표계로 바꾼다.
로봇의 heading θ는 월드 +X축 기준 반시계 양수 (ArUco에서 나온 값).

    | vx_r |   |  cosθ   sinθ | | vx_w |
    |      | = |              | |      |
    | vy_r |   | -sinθ   cosθ | | vy_w |

즉 로봇 좌표계는 월드를 -θ만큼 회전시킨 것.
  vx_r > 0 : 로봇 정면 방향으로 전진
  vy_r > 0 : 로봇 기준 왼쪽으로 횡이동 (오른손 좌표계)

각속도 w는 "진행 방향으로 정면을 맞추는" P 제어로 만든다.
옴니휠이라 굳이 방향을 맞추지 않아도 이동은 가능하지만,
정면을 진행 방향에 맞추면 로봇 전면 센서/카메라 활용에 유리하고 움직임이 자연스럽다.
"""

from __future__ import annotations

import math

import config


def wrap_pi(angle: float) -> float:
    """각도를 (-pi, pi]로 정규화."""
    return math.atan2(math.sin(angle), math.cos(angle))


def world_to_robot(vx_w: float, vy_w: float, heading_rad: float) -> tuple[float, float]:
    """월드 속도벡터를 로봇 몸체 좌표계로 회전변환."""
    c = math.cos(heading_rad)
    s = math.sin(heading_rad)
    vx_r = c * vx_w + s * vy_w
    vy_r = -s * vx_w + c * vy_w
    return vx_r, vy_r


def heading_command(vx_w: float, vy_w: float, heading_rad: float) -> float:
    """
    진행 방향과 현재 heading의 오차로부터 각속도 명령 [rad/s]를 만든다.
    속도가 거의 0이면 회전시키지 않는다 (제자리에서 빙빙 도는 것 방지).
    """
    speed = math.hypot(vx_w, vy_w)
    if speed < 1.0:  # cm/s
        return 0.0

    desired = math.atan2(vy_w, vx_w)
    err = wrap_pi(desired - heading_rad)

    if abs(err) < config.HEADING_DEADBAND_RAD:
        return 0.0

    w = config.HEADING_KP * err
    return max(-config.MAX_ANGULAR_SPEED_RAD_S,
               min(config.MAX_ANGULAR_SPEED_RAD_S, w))
