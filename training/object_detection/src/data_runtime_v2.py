"""Same transforms/data, with epoch/index keyed augmentation for exact boundary recovery."""
from common import *
import random
import numpy as np
import torch
from torch.utils.data import DataLoader
from data_runtime import DetectionData, collate, worker_seed, cuda_batch


class EpochData(DetectionData):
    def __init__(self, records, epoch):
        super().__init__(records, augment=True)
        self.epoch = epoch

    def __getitem__(self, index):
        # Independent of worker prefetch/scheduling. Existing HSV/flip unchanged.
        old_np, old_py = np.random.get_state(), random.getstate()
        keyed = (SEED + self.epoch * 1000003 + index * 9176) % 2**32
        np.random.seed(keyed); random.seed(keyed)
        try:
            return super().__getitem__(index)
        finally:
            np.random.set_state(old_np); random.setstate(old_py)


def epoch_order(count, epoch):
    return torch.randperm(count, generator=torch.Generator().manual_seed(SEED + epoch)).tolist()


def epoch_loader(records, epoch, start_microbatch=0, workers=2):
    order = epoch_order(len(records), epoch)
    gen = torch.Generator().manual_seed(SEED + epoch)
    result = DataLoader(EpochData(records, epoch), batch_size=16,
        sampler=order[start_microbatch * 16:], num_workers=workers,
        collate_fn=collate, pin_memory=True, worker_init_fn=worker_seed,
        generator=gen, **({'prefetch_factor': 2} if workers else {}))
    return result, order
