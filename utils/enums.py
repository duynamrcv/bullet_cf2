from enum import Enum

class DroneModel(Enum):
    """Drone models enumeration class."""

    CF2X = "cf2x"   # Bitcraze Craziflie 2.0 in the X configuration
    CF2P = "cf2p"   # Bitcraze Craziflie 2.0 in the + configuration

class CFProperties:
    # System configuration
    RADIUS = 0.04   # m

    # System limitation
    MAX_VEL = 1.5   # m/s
    MAX_ACC = 2.0   # m/s2

    # Sensing linmitation
    MAX_COM = 10.   # m
    MAX_SEN = 5.    # m
    MAX_NEIGHBOR = 3

class MPCParameters:
    HORIZONTAL_LENGTH = 20
    W_sep = 1.0
    W_dir = 2.0
    W_nav = 2.0
    W_u = 4e-1