"""
==================================
Enumerator class for aircraft data
==================================
"""

from enum import IntEnum, auto


class CONTROL_SURFACES(IntEnum):
    """
    Enumerator class for control surfaces
    """
    FLAP = auto()
    AILERON = auto()
    RUDDER = auto()
    ELEVATOR = auto()
    WINGLET = auto()


class AERODYNAMIC_SURFACE(IntEnum):
    """
    Enumerator class for aerodynamic surfaces
    """
    WING = auto()
    WING_FLAP = auto()
    WING_AILERON = auto()
    VTP = auto()
    VTP_RUDDER = auto()
    HTP = auto()
    HTP_ELEVATOR = auto()
    FUSELAGE = auto()


class WINGLET_TYPE(IntEnum):
    """
    Enumerator class for winglet type
    """

    BLENDED_WINGLET = auto()
    

class AIRFOIL_TYPE(IntEnum):
    """
    Enumerator class for airfoil type
    """
    
    NACA_5_DIGIT_STD = auto()
    NACA_5_DIGIT_RFX = auto()


class ROTATION_AXIS(IntEnum):
    """
    Enumerator class for rotation axis
    """
    X = auto()
    Y = auto()
    Z = auto()
    EYE = auto()

    
class MASS_CREATOR(IntEnum):
    """
    Enumerator class for mass input file. If data is provided from various resources
    """
    MEAN = auto()
    DATA = auto()


class AIRCRAFT_PARTS(IntEnum):
    """
    Enumerator class for AIRCRAFT PARTS
    """
    WING = auto()
    HTP = auto()
    VTP = auto()
    ENGINE = auto()
    MAIN_LG = auto()
    AUX_LG = auto()
    FUEL = auto()    
    FUSELAGE = auto()


__all__ = ['CONTROL_SURFACES', 'AERODYNAMIC_SURFACE', 'WINGLET_TYPE', 'AIRFOIL_TYPE', 'ROTATION_AXIS', 'MASS_CREATOR', 'AIRCRAFT_PARTS']
