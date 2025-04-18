import pybullet as p

# Example trajectory (list of 3D points)
trajectory = [
    [0, 0, 1],
    [1, 0, 1.5],
    [2, 1, 2],
    [3, 2, 1.5]
]

# Draw lines between each consecutive waypoint
for i in range(len(trajectory) - 1):
    p.addUserDebugLine(trajectory[i], trajectory[i + 1], [1, 0, 0], lineWidth=2.0, lifeTime=0)
