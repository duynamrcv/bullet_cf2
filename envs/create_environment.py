import numpy as np

class Environment:
    TIMESTEP = 0.1
    EPSILON = 0.1

    INIT_POSITIONS = np.array([[ 0.0, 0.0, 0.2],
                            [-0.5, 0.5, 0.2],
                            [ 0.0, 0.5, 0.2],
                            [ 0.0,-0.5, 0.2],
                            [-0.5,-0.5, 0.2]])
    NUM_DRONE = INIT_POSITIONS.shape[0]
    X_GOAL = 10.

    # Obstacle x, y, r
    OBSTACLES = np.array([[4.0,-1.0, 0.2],
                        [6.5, 0.5, 0.2],
                        [8.0,-1.0, 0.2],
                        [8.0, 2.0, 0.2],
                        [4.0, 2.0, 0.2],
                        [6.0, 3.0, 0.2],
                        [2.0, 0.0, 0.2],
                        [9.0, 1.0, 0.2]])