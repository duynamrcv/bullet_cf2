import pybullet as p
import time
import pybullet_data
import math
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("assets/cf2x.urdf",startPos, startOrientation)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
while True:
    p.stepSimulation()
    p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
    time.sleep(1.)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print("Current position and orientation", cubePos,cubeOrn)
p.disconnect()
