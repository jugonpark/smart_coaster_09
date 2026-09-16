"""
인식 계층 - YOLO 물체 검출 (Roboflow에서 학습한 모델 사용).

Roboflow에서 학습한 모델을 쓰는 방법은 두 가지이고, config.YOLO_BACKEND로 고른다.

  1) "ultralytics" (권장)
     Roboflow 프로젝트 > Deploy > "Download Dataset/Weights"에서 YOLOv8/v11 .pt를 받아
     config.YOLO_WEIGHTS 경로에 두면 된다. 로컬 GPU/CPU 추론이라 지연이 가장 낮다.

  2) "roboflow"
     roboflow inference SDK로 model_id를 직접 호출한다.
     호스팅 API를 쓰면 네트워크 왕복 때문에 실시간 제어에는 지연이 크다.
     로컬 inference 서버를 띄운 경우에만 권장.

  3) "stub"
     모델이 아직 없을 때 파이프라인 배선만 검증하는 용도. 항상 빈 리스트를 반환한다.

어떤 backend를 쓰든 이 모듈 밖으로 나가는 결과는 Detection 리스트로 동일하다.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass

import config


@dataclass
class Detection:
    """검출된 물체 하나. 픽셀 좌표계 기준."""

    label: str
    confidence: float
    cx_px: float
    cy_px: float
    w_px: float
    h_px: float

    # 월드 좌표 (cm). ObjectDetector.detect()에서 채워준다.
    x_cm: float = 0.0
    y_cm: float = 0.0
    # 척력 계산용 반경. 바운딩박스 긴 변의 절반을 cm로 환산한 값.
    radius_cm: float = 0.0
    timestamp: float | None = None

    @property
    def role(self) -> str:
        """이 검출이 파이프라인에서 갖는 역할: 'cup' | 'obstacle' | 'other'."""
        if self.label == config.CLASS_CUP:
            return "cup"
        if self.label in config.CLASS_OBSTACLE_NAMES:
            return "obstacle"
        return "other"


class ObjectDetector:
    """Roboflow 학습 모델 래퍼. backend를 감춰서 상위 계층은 신경 쓰지 않게 한다."""

    def __init__(self) -> None:
        self.backend = config.YOLO_BACKEND
        self._model = None
        self._frame_counter = 0
        self._cached: list[Detection] = []
        self._inferred_once = False
        self.available = self.backend != "stub"

        if self.backend == "ultralytics":
            self._init_ultralytics()
        elif self.backend == "roboflow":
            self._init_roboflow()
        elif self.backend == "stub":
            print("[YOLO] stub 모드 - 물체 검출이 비활성화되어 있습니다.")
        else:
            raise ValueError(f"알 수 없는 YOLO_BACKEND: {self.backend}")

    # ---------- backend 초기화 ----------
    def _init_ultralytics(self) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise RuntimeError(
                "ultralytics가 설치되어 있지 않습니다.  pip install ultralytics"
            ) from e

        if not os.path.isfile(config.YOLO_WEIGHTS):
            raise RuntimeError(
                f"YOLO 가중치를 찾을 수 없습니다: {config.YOLO_WEIGHTS}\n"
                "Roboflow > Deploy 에서 YOLOv8/v11 .pt 를 내려받아 해당 경로에 두거나,\n"
                "config.YOLO_BACKEND = 'stub' 으로 두고 나머지 계층을 먼저 테스트하세요."
            )

        self._model = YOLO(config.YOLO_WEIGHTS)
        print(f"[YOLO] ultralytics 로드 완료: {config.YOLO_WEIGHTS}")
        print(f"[YOLO] 클래스: {self._model.names}")

    def _init_roboflow(self) -> None:
        try:
            from inference import get_model
        except ImportError as e:
            raise RuntimeError(
                "inference 패키지가 없습니다.  pip install inference"
            ) from e

        api_key = os.environ.get("ROBOFLOW_API_KEY") or config.ROBOFLOW_API_KEY
        if not api_key:
            raise RuntimeError(
                "Roboflow API 키가 없습니다. 환경변수 ROBOFLOW_API_KEY를 설정하세요."
            )

        self._model = get_model(model_id=config.ROBOFLOW_MODEL_ID, api_key=api_key)
        print(f"[YOLO] roboflow 모델 로드 완료: {config.ROBOFLOW_MODEL_ID}")

    # ---------- 추론 ----------
    def detect(self, frame, world, scan=None, now=None) -> list[Detection]:
        """
        frame(BGR)에서 물체를 검출하고 월드 좌표까지 채워서 반환한다.
        config.YOLO_EVERY_N_FRAMES 마다 한 번만 실제 추론하고, 사이 프레임은 캐시를 쓴다.

        scan/now는 MarkerObjectDetector와 시그니처를 맞추기 위한 것으로 여기선 쓰지 않는다.
        (호출부가 두 검출기를 구분 없이 쓸 수 있게 하려는 목적)
        """
        self._frame_counter += 1
        should_infer = (self._frame_counter % config.YOLO_EVERY_N_FRAMES) == 0

        if should_infer or not self._inferred_once:
            try:
                if self.backend == "ultralytics":
                    self._cached = self._detect_ultralytics(frame)
                elif self.backend == "roboflow":
                    self._cached = self._detect_roboflow(frame)
                else:
                    self._cached = []
                self._inferred_once = True
                self.available = self.backend != "stub"
                detected_at = time.monotonic()
                for detection in self._cached:
                    detection.timestamp = detected_at
            except Exception:
                self._cached = []
                self.available = False
                raise

        # 캐시를 쓰더라도 월드 좌표는 현재 스케일로 다시 계산한다.
        for d in self._cached:
            d.x_cm, d.y_cm = world.to_world(d.cx_px, d.cy_px)
            d.radius_cm = world.px_len_to_cm(max(d.w_px, d.h_px) * 0.5)

        return self._cached

    def _detect_ultralytics(self, frame) -> list[Detection]:
        results = self._model.predict(
            frame,
            conf=config.YOLO_CONF_THRESHOLD,
            imgsz=config.YOLO_IMG_SIZE,
            verbose=False,
        )
        out: list[Detection] = []
        names = self._model.names
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cls_id = int(box.cls[0])
                out.append(
                    Detection(
                        label=names.get(cls_id, str(cls_id)),
                        confidence=float(box.conf[0]),
                        cx_px=(x1 + x2) * 0.5,
                        cy_px=(y1 + y2) * 0.5,
                        w_px=abs(x2 - x1),
                        h_px=abs(y2 - y1),
                    )
                )
        return out

    def _detect_roboflow(self, frame) -> list[Detection]:
        results = self._model.infer(frame, confidence=config.YOLO_CONF_THRESHOLD)
        out: list[Detection] = []
        # inference SDK는 리스트를 반환한다.
        preds = results[0].predictions if isinstance(results, list) else results.predictions
        for p in preds:
            out.append(
                Detection(
                    label=p.class_name,
                    confidence=float(p.confidence),
                    cx_px=float(p.x),
                    cy_px=float(p.y),
                    w_px=float(p.width),
                    h_px=float(p.height),
                )
            )
        return out

    # ---------- 편의 함수 ----------
    @staticmethod
    def pick_cup(detections: list[Detection]) -> Detection | None:
        """목표 컵 하나를 고른다. 여러 개면 confidence가 가장 높은 것."""
        cups = [d for d in detections if d.role == "cup"]
        if not cups:
            return None
        return max(cups, key=lambda d: d.confidence)

    @staticmethod
    def pick_obstacles(detections: list[Detection]) -> list[Detection]:
        return [d for d in detections if d.role == "obstacle"]
