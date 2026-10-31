"""Explicit synthetic interface demonstration, never a live inference result."""
import argparse
from pathlib import Path
import time

import cv2
import numpy as np

from decision_service import DecisionService, load_config


def make_frame(color, side_kind):
    frame = np.full((256, 448, 3), 90, np.uint8)
    cv2.rectangle(frame, (200, 12), (240, 68), (5,5,5), -1)
    cv2.circle(frame, (220, 34), 10, (0,0,255) if color == "red" else (0,255,0), -1)
    detections = np.array([[200, 12, 240, 68, .95, 0]], np.float32)
    road = np.ones((256,448), np.uint8)
    lane = np.zeros_like(road)
    lane[125:245, 125:129] = 1 if side_kind[0] == "solid" else 2
    lane[125:245, 319:323] = 1 if side_kind[1] == "solid" else 2
    return frame, detections, (road, lane)


def main():
    parser = argparse.ArgumentParser(description="Synthetic HTTP/rule test; not a camera or RKNN demo")
    parser.add_argument("--port", type=int, default=18088)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--color", choices=("red", "green"), default="red")
    parser.add_argument("--seconds", type=float, default=0)
    args = parser.parse_args()
    service = DecisionService(load_config(Path(__file__).with_name("decision_config.json")), evidence_mode="synthetic_test")
    service.start(args.host, args.port)
    epoch = service.begin_source("synthetic_test")
    frame, dets, masks = make_frame(args.color, ("solid", "dashed"))
    started, seq = time.monotonic(), 0
    print("SYNTHETIC_TEST_ONLY http://%s:%s/" % (args.host,args.port), flush=True)
    try:
        while not args.seconds or time.monotonic()-started < args.seconds:
            seq += 1
            packet = service.frame(frame, seq, epoch)
            service.submit_detection(packet, dets, ["traffic light"])
            service.submit_lane(packet, masks)
            time.sleep(.14)
    except KeyboardInterrupt:
        pass
    finally:
        service.close()


if __name__ == "__main__":
    main()
