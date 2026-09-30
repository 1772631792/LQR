"""Geometry for the independent two-link serial-leg UI renderer."""
import numpy as np

from config.leg_params import LEGS
from config.robot_params import PARAMS
from simulation.mujoco_model import serial_angles


def geometry(position, pitch, height, upper=.135, lower=.24):
    """Return hip/knee/wheel points without depending on five-bar geometry."""
    hip_angle, knee_angle = serial_angles(height, upper, lower)
    local = np.array([
        [0., 0.],
        [upper*np.sin(hip_angle), -upper*np.cos(hip_angle)],
        [upper*np.sin(hip_angle)+lower*np.sin(hip_angle+knee_angle),
         -upper*np.cos(hip_angle)-lower*np.cos(hip_angle+knee_angle)],
    ])
    rot = np.array([[np.cos(pitch), np.sin(pitch)], [-np.sin(pitch), np.cos(pitch)]])
    axle = np.array([position, PARAMS.wheel_radius])

    def world(points):
        return (np.asarray(points)-local[2])@rot.T+axle

    half = LEGS.body_length/2
    body = world([[-half, 0], [half, 0], [half, LEGS.body_height], [-half, LEGS.body_height]])
    com = world([0, PARAMS.com_height-LEGS.nominal_height])
    return world(local), body, com, np.array([hip_angle, knee_angle])
