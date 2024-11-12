import os
import numpy as np
import pybullet as p
from utils.enums import DroneModel, CFProperties

class CFDrone:
    """
    Base class for drone model
    """

    ################################################################################

    def __init__(self, index,
                 position=np.zeros(3),
                 drone_model=DroneModel.CF2X,
                 physical_client=None,
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
        self.index = index
        self.client = physical_client

        # Create drone
        orientation = p.getQuaternionFromEuler([0,0,0])
        self.drone_id = p.loadURDF("assets/{}.urdf".format(drone_model.value),
                                   position,
                                   orientation,
                                   physicsClientId=self.client)

        # Drone properties
        self.radius = CFProperties.RADIUS
        self.max_vel = CFProperties.MAX_VEL
        self.max_acc = CFProperties.MAX_ACC

        self.max_com = CFProperties.MAX_COM
        self.max_sen = CFProperties.MAX_SEN

    def update_state(self, control, dt):
        position, velocity = self.get_position_and_velocity()

        # Boundary control signal
        norm_control = np.linalg.norm(control)
        if(norm_control > self.max_acc):
            control = control/norm_control*self.max_acc

        # Boundary velocity signal
        velocity += control*dt
        norm_velocity = np.linalg.norm(velocity)
        if(norm_velocity > self.max_vel):
            velocity = velocity/norm_velocity*self.max_vel
        
        position += velocity*dt
        orientation = p.getQuaternionFromEuler([0,0,0])

        p.resetBasePositionAndOrientation(self.drone_id,
                                          position,
                                          orientation, 
                                          physicsClientId=self.client)
        p.resetBaseVelocity(self.drone_id,
                            velocity,
                            np.zeros(3),
                            physicsClientId=self.client
                            )

    def get_position_and_velocity(self):
        position, _ = p.getBasePositionAndOrientation(self.drone_id, physicsClientId=self.client)
        velocity, _ = p.getBaseVelocity(self.drone_id, physicsClientId=self.client)
        return position, velocity