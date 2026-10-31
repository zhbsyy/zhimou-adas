"""Reuse the frozen CARLA pipeline; replace only its inactive AEB link/policy."""
import numpy as np
import pcie_stream_r15_async as pipeline
from carla_aeb_link import CarlaAebLink
from aeb_status import make_handler


def main():
    original = pipeline.base.decode_auto
    def decode(*args, **kwargs):
        dets = original(*args, **kwargs)
        if len(dets):
            dets = dets[(dets[:, 5] != 10) | (dets[:, 4] >= .30)]
        return dets
    pipeline.base.decode_auto = decode
    pipeline.base.OibLink = CarlaAebLink
    pipeline.base.HAVE_OIB = True
    pipeline.base.STATS['class10_min_confidence'] = .30
    CarlaAebLink.stats_sink = pipeline.base.STATS
    pipeline.base.Handler = make_handler(pipeline.base.Handler,pipeline.base.STATS)
    pipeline.main()


if __name__ == '__main__':
    main()
