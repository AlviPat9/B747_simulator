"""
===================
    Utilities
===================

    Module that includes basic utilities accessible to all models

"""
import numpy as np
from numpy import cos, sin, tan, pi, cross
from numpy.linalg import norm


def deg2rad(x):
    """
    Conversion from º to radians
    :param x: angle in º
    :type x: float
    :return: angle in rad
    :rtype: float
    """
    return x * pi / 180


def calculate_angles(ang):
    """
    Calculate trigonometric relations of an angle
    
    :param ang: angle in º
    :type ang: float
    :return trigonometric relations of the angle
    :rtype list
    """
    ang = deg2rad(ang)
    
    return [cos(ang), sin(ang), tan(ang)]


def surface_calculation(r1, r2, r3, r4):
    """
    Function to calculate the surface delimited by 4 points

    :param r1: top left corner of the surface
    :type r1: np.array
    :param r2: top right corner of the surface
    :type r2: np.array
    :param r3: bottom left corner of the surface
    :type r3: np.array
    :param r4: bottom right corner of the surface
    :type r4: np.array
    :return: area
    :rtype float
    """

    surface = np.linalg.norm(np.cross(r3 - r1, r2 - r1)) / 2 + np.linalg.norm(np.cross(r1 - r2, r4 - r2))

    return surface
