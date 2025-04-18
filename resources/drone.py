import pybullet as p
import os
import numpy as np

class Drone:
    def __init__(self, client, pose=np.array([0.,0.,1.0])):
        self.client = client
        self.max_thrust = 1.0
        self.max_velocity = 1.0
        self.n_action = 3
        self.n_states = 6
        self.pos = pose
        self.vel = np.zeros(3)
        self.drone = p.loadURDF(fileName="assets/cf2x.urdf",
                                basePosition=pose,
                                physicsClientId=client)

    def get_ids(self):
        return self.drone, self.client

    def apply_action(self, action):
        # print(action)
        # action is 3 dimension
        thrust_x, thrust_y, thrust_z = action

        # Clip thrust and torque
        thrust_x = np.clip(thrust_x, -self.max_thrust, self.max_thrust)
        thrust_y = np.clip(thrust_y, -self.max_thrust, self.max_thrust)
        thrust_z = np.clip(thrust_z, -self.max_thrust, self.max_thrust)

        p.applyExternalForce(self.drone, 4,
                             forceObj=[thrust_x, thrust_y, thrust_z],
                             posObj=[0, 0, 0],
                             flags=p.LINK_FRAME)


    def get_observation(self, goal):
        """ Get observation of the drone
        """
        # Get the position and orientation in the simulation
        pos, ang = p.getBasePositionAndOrientation(self.drone, self.client)
        # ang = p.getEulerFromQuaternion(ang)

        # Get the velocity of the car
        linear_velocity, angular_velocity = p.getBaseVelocity(self.drone, self.client)

        # Update Drone data
        self.pos = pos
        self.vel = linear_velocity
        
        # Concatenate position, linear velocity and goal
        observation = np.concatenate([self.pos, self.vel, goal])
        info = np.concatenate([self.pos, self.vel])
        return observation, info