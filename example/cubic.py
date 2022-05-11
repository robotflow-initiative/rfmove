import sys

import numpy
import time
import pybullet as p
import pybullet_data
sys.path.append("../install/lib")






print("== Initalize Pybullet ==")
physicsClient=p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId=p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("../resources")

startPos=[0,0,0]
startOrientation=p.getQuaternionFromEuler([0,0,0])
boxId=p.loadURDF("franka.urdf",startPos,startOrientation,useFixedBase=1)
initalizePybulletRobotState=list([0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279])






while(1):
    p.stepSimulation()
    time.sleep(1./1000.)
