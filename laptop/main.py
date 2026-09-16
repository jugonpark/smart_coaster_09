from __future__ import annotations

import time
import cv2
import numpy as np

import config
from camera_streamer import CameraStreamer
from monitor import MonitorReceiver, draw_status


def main() -> None:
    camera = None
    monitor = None
    try:
        camera = CameraStreamer()
        monitor = MonitorReceiver()
        camera.start()
        monitor.start()
        print(f"[LAPTOP] MJPEG: http://<LAPTOP_IP>:{config.MJPEG_PORT}/stream.mjpg")
        print(f"[LAPTOP] monitor UDP port: {config.MONITOR_UDP_PORT}")
        print("[LAPTOP] q/ESC, window close, or Ctrl+C to quit")
        while True:
            frame = camera.latest_frame()
            status, online = monitor.snapshot()
            if config.SHOW_PREVIEW:
                if frame is None:
                    frame = np.zeros((config.FRAME_HEIGHT, config.FRAME_WIDTH, 3), dtype=np.uint8)
                draw_status(frame, status, online, camera.camera_state)
                cv2.imshow(config.WINDOW_NAME, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                try:
                    if cv2.getWindowProperty(config.WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                        break
                except cv2.error:
                    break
            else:
                time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        if monitor is not None:
            monitor.close()
        if camera is not None:
            camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
