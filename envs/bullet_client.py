import pybullet as p
import time
import numpy as np
import pybullet_data
import math
from utils.enums import DroneModel, CFProperties
from model.cf import CFDrone

from envs.create_environment import Environment

class ClientParameters:
    GRAPHIC = p.GUI # GUI, DIRECT

class Client:
    def __init__ (self, env:Environment):
        self.client = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        self.env = env
        self.num_drones = env.NUM_DRONE
        self.create_scenarios()

    def create_scenarios(self):
        self.create_environment()
        self.create_drones()

    def create_environment(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,0)
        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client)
        
        if self.env.OBSTACLES is not None:
            self.obstacles = []
            for obstacle in self.env.OBSTACLES:
                height = 1
                shape = p.createCollisionShape(shapeType=p.GEOM_CYLINDER,
                                            radius=obstacle[2],
                                            height=height,
                                            physicsClientId=self.client)

                cylinder = p.createMultiBody(baseMass=1,
                                            baseCollisionShapeIndex=shape,
                                            baseVisualShapeIndex=-1,
                                            basePosition=[obstacle[0], obstacle[1], height / 2])
                self.obstacles.append(cylinder)

    def create_drones(self):
        self.drones = [CFDrone(index=i,
                               position=self.env.INIT_POSITIONS[i,:],
                               physical_client=self.client) for i in range(self.env.NUM_DRONE)]
        
    def step(self, controls):
        p.stepSimulation()
        for i in range(self.env.NUM_DRONE):
            self.drones[i].update_state(controls[i,:], self.env.TIMESTEP)
        time.sleep(self.env.TIMESTEP)