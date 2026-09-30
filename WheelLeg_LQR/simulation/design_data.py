"""Read offline parameters without importing the LQR solver."""
import json
from dataclasses import asdict
from pathlib import Path
from config.robot_params import PARAMS

ROOT = Path(__file__).resolve().parents[1]


def load_design():
    path = ROOT / 'config' / 'lqr_design.json'
    if not path.exists():
        raise RuntimeError('Run python -m simulation.design_lqr first.')
    data = json.loads(path.read_text(encoding='utf-8'))
    if data['params'] != asdict(PARAMS):
        raise RuntimeError('Physical parameters changed: redesign gains and rebuild DLL.')
    return data
