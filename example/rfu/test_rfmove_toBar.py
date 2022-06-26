import sys

import numpy
import time
import math
import moveit_noros as moveit
import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from moveit_noros import rfWaypoint,PlannerSpline

print("== Initalize Pybullet ==")
physicsClient=p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId=p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("/home/ziye01/rfmove/resources/r300_description/urdf")

# 设定位置起点
startPos=[0,0,0]
#从欧拉角设定四元素
startOrientation=p.getQuaternionFromEuler([0,0,0])
boxId=p.loadURDF("tobar.urdf",startPos,startOrientation,useFixedBase=1)



for i in range(250000000000000000000):
    p.stepSimulation()
    time.sleep(1./1000.)