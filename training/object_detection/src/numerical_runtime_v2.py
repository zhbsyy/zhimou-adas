"""Finite checks, fixed virtual-batch normalization, and recoverable valid states."""
from common import *
import random
import numpy as np
import torch

NOMINAL_BATCH = 32
MEMORY_LIMIT = int(6.5 * 1024**3)


def context(precision):
    return torch.autocast('cuda', dtype=torch.bfloat16 if precision == 'bf16' else torch.float16,
                          enabled=precision != 'fp32')


def finite_tensor(name, tensor):
    if not torch.isfinite(tensor).all():
        raise FloatingPointError('First nonfinite tensor: ' + name)


def check_gradients(model, label='gradient'):
    for name, p in model.named_parameters():
        if p.grad is not None:
            finite_tensor(label + ':' + name, p.grad)


def check_parameters(model, optimizer=None):
    for name, p in model.named_parameters():
        finite_tensor('parameter:' + name, p)
        if optimizer is not None:
            for key, value in optimizer.state.get(p, {}).items():
                if torch.is_tensor(value):
                    finite_tensor('optimizer:' + name + ':' + key, value)


def rng_state():
    return {'python': random.getstate(), 'numpy': np.random.get_state(),
            'torch': torch.get_rng_state(), 'cuda': torch.cuda.get_rng_state_all()}


def restore_rng(state):
    random.setstate(state['python']); np.random.set_state(state['numpy'])
    torch.set_rng_state(state['torch']); torch.cuda.set_rng_state_all(state['cuda'])


def save_valid_state(path, model, ema, optimizer, scaler, position, selection):
    check_parameters(model, optimizer); check_parameters(ema)
    payload = {'valid': True, 'model': model.state_dict(), 'ema': ema.state_dict(),
        'optimizer': optimizer.state_dict(), 'scaler': scaler.state_dict(),
        'scheduler': {'type': 'stateless_set_lr_v1', **position},
        'rng': rng_state(), 'position': position, 'selection': selection,
        'loader': {'seed': SEED, 'epoch_seed': SEED + position['epoch'],
                   'next_microbatch': position['next_microbatch'],
                   'augmentation': 'seed+epoch*1000003+record_index*9176 modulo2**32'},
        'gradients': 'zero; checkpoint at completed optimizer boundary',
        'source_sha256': INIT_SHA, 'manifest_sha256': sha(ASSET_ROOT/'configs/manifest.json')}
    temp = Path(str(path) + '.tmp'); torch.save(payload, temp); temp.replace(path)
