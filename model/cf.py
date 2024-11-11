import os
import numpy as np
from utils.enums import DroneModel, CFProperties

class Drone:
    """
    Base class for drone model
    """

    ################################################################################

    def __init__(self,
                 position=np.zeros(3),
                 velocity=np.zeros(3),
                 drone_model=DroneModel.CF2X
                 ):
        """Common drone classes __init__ method.

        Parameters
        ----------
        position : numpy.array
            The position of drone
        velocity : numpy.array
            The velocity of drone
        drone_model: DroneModel, default = CF2X
            The model type of drone (CF2X, CF2P)
        """
        # Drone state and control
        self.position = position
        self.velocity = velocity
        self.control = np.zeros(3)

        # Drone properties
        self.drone_model = drone_model
        self.radius = CFProperties.RADIUS
        self.max_vel = CFProperties.MAX_VEL
        self.max_acc = CFProperties.MAX_ACC

    def update_drone_state(self, control, dt):
        # Boundary control signal
        norm_control = np.linalg.norm(control)
        if(norm_control > self.max_acc):
            control = control/norm_control*self.max_acc

        velocity = self.velocity + control*dt
        norm_velocity = np.linalg.norm(velocity)
        if(norm_velocity > self.max_vel):
            velocity = velocity/norm_velocity*self.max_vel
        
        position = self.position + velocity*dt

        # Update state and control
        self.position = position
        self.velocity = velocity
        self.control = control