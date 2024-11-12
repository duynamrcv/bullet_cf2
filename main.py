from envs.bullet_client import Client
from envs.create_environment import Environment
import numpy as np

if __name__ == "__main__":
    env = Environment()
    client = Client(env)

    while True:
        controls = np.zeros((env.NUM_DRONE, 3))
        client.step(controls)