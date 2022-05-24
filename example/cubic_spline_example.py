import sys

import numpy
import time

import pybullet as p
import pybullet_data
sys.path.append("../install/lib")
import moveit_noros as moveit
import numpy as np
import matplotlib.pyplot as plt
from moveit_noros import rfWaypoint,PlannerSpline

joint_name_list=["panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"]

waypoint1=rfWaypoint([0.33,0,0.58],[0,3.14,-0.5])
waypoint2=rfWaypoint([0.33,0.2,0.58],[0,2.14,-0.5])
waypoint3=rfWaypoint([0.33,0.1,0.68],[0,3.14,0.5])

print("== Initalize Planner moveit model ==")
plannerspline=PlannerSpline("panda_arm")
plannerspline.init("../resources/franka_convert.urdf",
                   "../resources/panda_arm_hand.srdf",
                   "../resources/kinematics.yaml",
                   "../resources/ompl_planning.yaml",
                   "../resources/joint_limits.yaml")


print("== Initalize Pybullet ==")
physicsClient=p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId=p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("../resources")

# 设定位置起点
startPos=[0,0,0]
#从欧拉角设定四元素
startOrientation=p.getQuaternionFromEuler([0,0,0])
boxId=p.loadURDF("franka_convert.urdf",startPos,startOrientation,useFixedBase=1)

home=[0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]
initalizePybulletRobotState=list(home)
p.setJointMotorControlArray(bodyIndex=boxId,
                            jointIndices=range(len(joint_name_list)),
                            targetPositions=initalizePybulletRobotState,
                            controlMode=p.POSITION_CONTROL)

'''
#每次运行前，必须使用当前机器人的状态重新init一下机器人
plannerspline.InitRobotState(np.array(home),"panda_arm")
plannerspline.CreateSingleWaypointPath(waypoint1,"panda_arm","panda_link0","panda_link7",1,1)
timeslist=plannerspline.getTimeList()
'''
#每次运行前，必须使用当前机器人的状态重新init一下机器人
plannerspline.InitRobotState(np.array([0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]),"panda_arm")
plannerspline.CreateSplineParameterization([waypoint1,waypoint2,waypoint3],"panda_arm","panda_link0","panda_link7")
timeslist=plannerspline.getTimeList()


plannerspline.sample_by_interval(0.001)

ompltimelist=plannerspline.get_sample_by_interval_times()

posvecacl_1=plannerspline.get_ompl_sample("panda_joint1")
omplposlist_1=posvecacl_1.position
omplveclist_1=posvecacl_1.velocity
omplacllist_1=posvecacl_1.acceleration

posvecacl_2=plannerspline.get_ompl_sample("panda_joint2")
omplposlist_2=posvecacl_2.position
omplveclist_2=posvecacl_2.velocity
omplacllist_2=posvecacl_2.acceleration

posvecacl_3=plannerspline.get_ompl_sample("panda_joint3")
omplposlist_3=posvecacl_3.position
omplveclist_3=posvecacl_3.velocity
omplacllist_3=posvecacl_3.acceleration

posvecacl_4=plannerspline.get_ompl_sample("panda_joint4")
omplposlist_4=posvecacl_4.position
omplveclist_4=posvecacl_4.velocity
omplacllist_4=posvecacl_4.acceleration

posvecacl_5=plannerspline.get_ompl_sample("panda_joint5")
omplposlist_5=posvecacl_5.position
omplveclist_5=posvecacl_5.velocity
omplacllist_5=posvecacl_5.acceleration

posvecacl_6=plannerspline.get_ompl_sample("panda_joint6")
omplposlist_6=posvecacl_6.position
omplveclist_6=posvecacl_6.velocity
omplacllist_6=posvecacl_6.acceleration

posvecacl_7=plannerspline.get_ompl_sample("panda_joint7")
omplposlist_7=posvecacl_7.position
omplveclist_7=posvecacl_7.velocity
omplacllist_7=posvecacl_7.acceleration


waypoints=[]
joint1pos=plannerspline.get_ompl_sample(joint_name_list[0]).position
joint2pos=plannerspline.get_ompl_sample(joint_name_list[1]).position
joint3pos=plannerspline.get_ompl_sample(joint_name_list[2]).position
joint4pos=plannerspline.get_ompl_sample(joint_name_list[3]).position
joint5pos=plannerspline.get_ompl_sample(joint_name_list[4]).position
joint6pos=plannerspline.get_ompl_sample(joint_name_list[5]).position
joint7pos=plannerspline.get_ompl_sample(joint_name_list[6]).position
for i in range(len(ompltimelist)):
    waypoints.append([joint1pos[i],joint2pos[i],joint3pos[i],joint4pos[i],joint5pos[i],joint6pos[i],joint7pos[i]])


for i in range(len(ompltimelist)):
    p.stepSimulation()
    p.setJointMotorControlArray(bodyIndex=boxId,
                                jointIndices=range(len(joint_name_list)),
                                targetPositions=waypoints[i],
                                controlMode=p.POSITION_CONTROL)
    time.sleep(1./1000.)

vlist_1=[omplposlist_1,omplposlist_2,omplposlist_3,omplposlist_4,omplposlist_5,omplposlist_6,omplposlist_7]
vlist_2=[omplveclist_1,omplveclist_2,omplveclist_3,omplveclist_4,omplveclist_5,omplveclist_6,omplveclist_7]
vlist_3=[omplacllist_1,omplacllist_2,omplacllist_3,omplacllist_4,omplacllist_5,omplacllist_6,omplacllist_7]

def showplt(label='position',vlist=None,timeslist=None):
    fig = plt.figure(num = 1, figsize=(32,26), dpi =80)
    p1 = fig.add_subplot(4, 2, 1)
    p2 = fig.add_subplot(4, 2, 2)
    p3 = fig.add_subplot(4, 2, 3)
    p4 = fig.add_subplot(4, 2, 4)
    p5 = fig.add_subplot(4, 2, 5)
    p6 = fig.add_subplot(4, 2, 6)   
    p7 = fig.add_subplot(4, 2, 7)


    plist=[p1,p2,p3,p4,p5,p6,p7]
    for i in range(len(plist)):
        plist[i].set_title("Trajectory "+label+str(i))
        plist[i].set_xlabel("time")
        plist[i].set_ylabel("position")
        plist[i].plot(timeslist, vlist[i], linestyle='-', marker='o', markersize = 3)
    plt.show()

showplt(label="position",vlist=vlist_1,timeslist=ompltimelist)
showplt(label="vec",vlist=vlist_2,timeslist=ompltimelist)
showplt(label="acl",vlist=vlist_3,timeslist=ompltimelist)


