from enum import Enum
class DroneModel(Enum):
    """Drone models enumeration class."""

    CF2X = "cf2x"   # Bitcraze Craziflie 2.0 in the X configuration
    CF2P = "cf2p"   # Bitcraze Craziflie 2.0 in the + configuration

class CFProperties:
    # System configuration
    RADIUS = 0.04 # m

    # System limitation
    MAX_VEL = 1.5 # m/s
    MAX_ACC = 2.0 # m/s2