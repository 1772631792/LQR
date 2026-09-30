"""Illustrative symmetric five-bar dimensions, metres; not a team's CAD replica."""
from dataclasses import dataclass


@dataclass(frozen=True)
class LegParams:
    hip_spacing: float = 0.12
    upper_length: float = 0.13
    lower_length: float = 0.20
    nominal_height: float = 0.20
    track_width: float = 0.36
    body_length: float = 0.22
    body_width: float = 0.26
    body_height: float = 0.10


LEGS = LegParams()
