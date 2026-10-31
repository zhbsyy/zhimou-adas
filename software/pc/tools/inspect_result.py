"""Inspect a downloaded video/summary; no inference or network required."""
from pathlib import Path
import argparse
import json
import cv2


def main():
    p = argparse.ArgumentParser(description='Check metadata of a downloaded offline result.mp4')
    p.add_argument('video', type=Path)
    args = p.parse_args()
    cap = cv2.VideoCapture(str(args.video))
    try:
        if not cap.isOpened():
            raise SystemExit('Cannot open video: '+str(args.video))
        fps = cap.get(cv2.CAP_PROP_FPS)
        frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        print(json.dumps({'file':str(args.video), 'size':[int(cap.get(3)),int(cap.get(4))],
            'fps':fps, 'frames':frames, 'duration_s':frames/fps if fps else None}, ensure_ascii=False, indent=2))
    finally:
        cap.release()
    summary = args.video.with_suffix('.summary.json')
    if summary.is_file():
        print(summary.read_text(encoding='utf-8'))


if __name__ == '__main__':
    main()
