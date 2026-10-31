"""Only the existing model pre/postprocessing; no live stream/device imports."""
from pcie_post import preprocess_fast, decode_auto, to_original, draw
from r15_front_fast import LiveLaneModel, draw_lane_mask

def rknn_input(bgr, size=640):
    return preprocess_fast(bgr, size=size)
