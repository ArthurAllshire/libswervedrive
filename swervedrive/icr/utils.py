import numpy as np


def constrain_angle(angle):
    """ Wrap `angle` to +- pi"""
    return np.arctan2(np.sin(angle), np.cos(angle))
