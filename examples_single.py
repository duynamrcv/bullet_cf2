import os, sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import gym
import numpy as np
import math
import time
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt

from methods.pid import PIDController
from resources.drone import Drone

class Environment(gym.Env):
    metadata = {'render.modes': ['humans']}

    def __init__(self, display=p.GUI):   # p.GUI, p.DIRECT
        self.client = p.connect(display)
        self.drone = Drone(self.client)
        self.goal = None
        self.obstacles = None
        
        self.rendered_img= None

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf", physicsClientId=self.client)

        self.action_space = gym.spaces.box.Box(
            low=np.array([-self.drone.max_thrust,
                          -self.drone.max_thrust,
                          -self.drone.max_thrust], dtype=np.float32),
            high=np.array([self.drone.max_thrust,
                           self.drone.max_thrust,
                           self.drone.max_thrust], dtype=np.float32)
        )
        self.observation_space = gym.spaces.box.Box(
            low=np.array([-np.inf, -np.inf, -np.inf,
                          -np.inf, -np.inf, -np.inf,
                          -np.inf, -np.inf, -np.inf], dtype=np.float32),
            high=np.array([np.inf, np.inf, np.inf,
                           np.inf, np.inf, np.inf,
                           np.inf, np.inf, np.inf], dtype=np.float32)
        )

        p.setTimeStep(1/20.0, self.client)  # 20hz

        self.reset()

    def step(self, action):
        self.drone.apply_action(action)
        p.stepSimulation()

        observation, info = self.drone.get_observation(self.goal)
        reward, done, reach_target = self.calculate_reward(observation, action)

        # Reset camera view
        drone_pos = info[:3]
        p.resetDebugVisualizerCamera(cameraDistance = 0.5, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=drone_pos)
        return observation, reward, info, done, reach_target

    def reset(self):
        p.resetSimulation(self.client)
        
        # Reload the plane and drone
        self.drone = Drone(self.client)
        self.reset_goal_position()
        observation, info = self.drone.get_observation(self.goal)
        return observation, info

    def render(self, mode='human'):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((100, 100, 4)))

        # Base information
        drone_id, client_id = self.drone.get_ids()
        proj_matrix = p.computeProjectionMatrixFOV(fov=80, aspect=1, nearVal=0.01, farVal=100)
        pos, ori = p.getBasePositionAndOrientation(drone_id, client_id)

        # Rotate camera direction
        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, [1, 0, 0])
        up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        view_matrix = p.computeViewMatrix((pos[0], pos[1], pos[2]+0.05), pos + camera_vec, up_vec)

        # Display image
        frame = p.getCameraImage(100, 100, view_matrix, proj_matrix)[2]
        frame = np.reshape(frame, (100, 100, 4))
        self.rendered_img.set_data(frame)
        plt.draw()

    def seed(self, seed=None):
        np.random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def close(self):
        p.disconnect(self.client)

    def calculate_reward(self, observation, action):
        # TODO: Redefine the reward value
        pos = observation[:3]
        vel = observation[3:6]
        goal = observation[6:]

        dis_to_goal = np.linalg.norm(goal - pos)
        done = False
        reach_target = False

        # Reward factor
        R_max = 100
        target_rf = 0.1
        action_rf = 0.001

        # Done if reaching goal
        if dis_to_goal < 0.05:
            done = True
            reach_target = True
            print("reach the goal!")
            reward = R_max
        else:
            reward_target = target_rf*dis_to_goal**2
            reward_action = action_rf*np.linalg.norm(action)**2

            reward = - reward_target - reward_action
        return reward, done, reach_target
    
    def reset_goal_position(self, random=False):
        self.goal = np.array([3, 3, 3])

        # Set visual element of the goal
        p.loadURDF(fileName="assets/goal.urdf", basePosition=self.goal, physicsClientId=self.client)
        return self.goal
    
if __name__ == "__main__":
    env = Environment()
    agent = PIDController(env, kp=0.5, kd=0.01, ki=0.15)

    iter_max = 300
    iter = 0
    _, info = env.reset()
    while iter < iter_max:
        action = agent.choose_action(info, env.goal)
        observation_, reward, info, done, reach_target = env.step(action)

        print("Reward: ", reward)
        print("Action: ", action)
        print("State: ", info)
        print("-----------")
        env.render()
        iter += 1

        # if reach_target:
        #     break

    env.close()