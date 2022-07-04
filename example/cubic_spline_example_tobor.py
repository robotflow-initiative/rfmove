import sys

import numpy
import time
import math
sys.path.append("../install/lib")
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
p.setAdditionalSearchPath("../resources/tobor/tobor_description")

# 设定位置起点
startPos=[0,0,0]
#从欧拉角设定四元素
startOrientation=p.getQuaternionFromEuler([0,0,0])
boxId=p.loadURDF("tobor.urdf",startPos,startOrientation,useFixedBase=1)

# pybullet 添加长方体

def createBoxForpyBullet(halfExtents,basePositon,baseOrientation):
    box_collision_id =p.createCollisionShape(shapeType = p.GEOM_BOX,
                                        halfExtents = halfExtents,
                                        collisionFramePosition = [0,0,0])
    box_visual_id = p.createVisualShape(shapeType = p.GEOM_BOX,
                                        halfExtents = halfExtents,
                                        visualFramePosition = [0,0,0],
                                        rgbaColor = [1, 0.2, 0, 0.9],
                                        specularColor = [0, 0, 0])
    box_body_id = p.createMultiBody(baseMass = 0, # 0 means this object is fixed
                                        baseCollisionShapeIndex = box_collision_id,
                                        baseVisualShapeIndex = box_visual_id,
                                        basePosition = basePositon,  #都是在几何中心
                                        baseOrientation =baseOrientation,
                                        baseInertialFramePosition = [0,0,0])

    return box_body_id

createBoxForpyBullet([0.005, 0.25, 0.2],[ 1.00461376,-0.08487324 ,1.07463121],[ 0.113482, 0.697941, -0.0988456, 0.700164])

def createSpherepyBullet(radius,basePosition,baseOrientation):
    # pybullet 添加圆形
    sphere_collision_id =p.createCollisionShape(shapeType = p.GEOM_SPHERE,
                                        radius  = radius,
                                        collisionFramePosition = [0,0,0])
    sphere_visual_id = p.createVisualShape(shapeType = p.GEOM_SPHERE,
                                        radius = 0.15153881907463074,
                                        visualFramePosition = [0,0,0],
                                        rgbaColor = [1, 0.2, 0, 0.9],
                                        specularColor = [0, 0, 0])
    sphere_body_id = p.createMultiBody(baseMass = 0, # 0 means this object is fixed
                                        baseCollisionShapeIndex = sphere_collision_id,
                                        baseVisualShapeIndex = sphere_visual_id,
                                        basePosition = basePosition,  #都是在几何中心
                                        baseOrientation = baseOrientation, 
                                        baseInertialFramePosition = [0,0,0])
    return sphere_body_id

def createClindyerpyBullet(radius,height,basePosition,baseOrientation):
    clindyer_collision_id =p.createCollisionShape(shapeType = p.GEOM_CYLINDER,
                                        radius  = radius,
                                        collisionFramePosition = [0,0,0],
                                        height  = height)

    clindyer_visual_id = p.createVisualShape(shapeType = p.GEOM_CYLINDER,
                                            radius = 0.22323070466518402,
                                            length=0.8929228186607361,
                                            visualFramePosition = [0,0,0],
                                            rgbaColor = [1, 0.2, 0, 0.9],
                                            specularColor = [0, 0, 0])

    clindyer_body_id = p.createMultiBody(baseMass = 0, # 0 means this object is fixed
                                        baseCollisionShapeIndex = clindyer_collision_id,
                                        baseVisualShapeIndex = clindyer_visual_id,
                                        basePosition = basePosition,  #都是在几何中心
                                        baseOrientation = baseOrientation, 
                                        baseInertialFramePosition = [0,0,0])
    return clindyer_body_id

createClindyerpyBullet(0.05,0.4,[1.,1.,0.30000001],[ -0.337559, -0.435233, 0.18028, 0.814939])

'''
for i in range(250000000000000000000):
    p.stepSimulation()
    time.sleep(1./100.)
'''



