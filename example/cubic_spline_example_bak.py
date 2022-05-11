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
waypoint2=rfWaypoint([0.33,0.1,0.58],[0,3.14,-0.5])
waypoint3=rfWaypoint([0.33,0.1,0.68],[0,3.14,0])

print("== Initalize Planner moveit model ==")
plannerspline=PlannerSpline("panda_arm")
plannerspline.loadRobotModel("../resources/panda_arm_hand_urdf_convert.urdf","../resources/panda_arm_hand.srdf")
plannerspline.loadKinematicModel("../resources/kinematics.yaml")
plannerspline.loadPlannerConfig("../resources/ompl_planning.yaml")
plannerspline.loadJointLimit("../resources/joint_limits.yaml")

print("== Initalize Pybullet ==")
physicsClient=p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId=p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("../resources")

startPos=[0,0,0]
startOrientation=p.getQuaternionFromEuler([0,0,0])
boxId=p.loadURDF("panda_arm_hand_urdf_convert.urdf",startPos,startOrientation,useFixedBase=1)
initalizePybulletRobotState=list([0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279])
p.setJointMotorControlArray(bodyIndex=boxId,
                            jointIndices=range(len(joint_name_list)),
                            targetPositions=initalizePybulletRobotState,
                            controlMode=p.POSITION_CONTROL)


plannerspline.InitRobotState(np.array([0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]),"panda_arm")
plannerspline.CreateSplineParameterization([waypoint1,waypoint2,waypoint3],"panda_arm",0.5,1,1)
timeslist=plannerspline.getTimeList()

joint_name="panda_joint2"
valuelist=plannerspline.getJointPosValue(joint_name)
veclist=plannerspline.getJointVecValue(joint_name)
acllist=plannerspline.getJointAclValue(joint_name)



#sampletimelist2
#sampleveclist2
#sampleacllist2

result=plannerspline.computeSpline(0.001,"panda_arm")
if result:
    print("error")
sampletimelist=plannerspline.getSampletimestamp()
sampleposlist=plannerspline.getwaypointPositionList(joint_name)
sampleveclist=plannerspline.getwaypointVelocityList(joint_name)
sampleacllist=plannerspline.getwaypointAccelerationList(joint_name)
#print(timeslist)
#print(valuelist)
#print(sampletimelist)
#print(samplevaluelist)
fig = plt.figure(num = 1, figsize=(16, 9), dpi = 120)

# ompl 
plannerspline.sample_by_interval(0.001)
ompltimelist=plannerspline.get_sample_by_interval_times()
posvecacl=plannerspline.get_ompl_sample(joint_name)
omplposlist=posvecacl.position
omplveclist=posvecacl.velocity
omplacllist=posvecacl.acceleration


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
#print(ompltimelist)
#print(omplposlist)
#print(omplveclist)
#print(omplacllist)

#for i in range(len(ompltimelist)):
while(1):
    p.stepSimulation()
    p.setJointMotorControlArray(bodyIndex=boxId,
                                jointIndices=range(len(joint_name_list)),
                                targetPositions=waypoints[i],
                                controlMode=p.POSITION_CONTROL)
    time.sleep(1./1000.)

origin_position = fig.add_subplot(3, 3, 1)
origin_velocity = fig.add_subplot(3, 3, 4)
origin_acceleration = fig.add_subplot(3, 3, 7)
sample_position = fig.add_subplot(3, 3, 2)
sample_velocity = fig.add_subplot(3, 3, 5)
sample_acceleration = fig.add_subplot(3, 3, 8)
omplsample_position = fig.add_subplot(3, 3, 3)
omplsample_velocity = fig.add_subplot(3, 3, 6)
omplsample_acceleration = fig.add_subplot(3, 3, 9)


origin_position.set_title("Origin Trajectory Position")
origin_position.set_xlabel("time")
origin_position.set_ylabel("position")
origin_velocity.set_title("Origin Trajectory Velocity (1000Hz)")
origin_velocity.set_xlabel("time")
origin_velocity.set_ylabel("velocity")
origin_acceleration.set_title("Origin Trajectory Acceleration (1000Hz)")
origin_acceleration.set_xlabel("time")
origin_acceleration.set_ylabel("Acceleration")


sample_position.set_title("Sampled Trajectory Position (1000Hz)")
sample_position.set_xlabel("time")
sample_position.set_ylabel("position")
sample_velocity.set_title("Sampled Trajectory Velocity (1000Hz)")
sample_velocity.set_xlabel("time")
sample_velocity.set_ylabel("velocity")
sample_acceleration.set_title("Sampled Trajectory Acceleration (1000Hz)")
sample_acceleration.set_xlabel("time")
sample_acceleration.set_ylabel("Acceleration")

omplsample_position.set_title("ompl Sampled Trajectory Position (1000Hz)")
omplsample_position.set_xlabel("time")
omplsample_position.set_ylabel("position")
omplsample_velocity.set_title("ompl Sampled Trajectory Velocity (1000Hz)")
omplsample_velocity.set_xlabel("time")
omplsample_velocity.set_ylabel("velocity")
omplsample_acceleration.set_title("ompl Sampled Trajectory Acceleration (1000Hz)")
omplsample_acceleration.set_xlabel("time")
omplsample_acceleration.set_ylabel("Acceleration")

plot1=origin_position.plot(timeslist, valuelist, linestyle='-', marker='o', markersize = 3)
plot2=origin_velocity.plot(timeslist, veclist, linestyle='-', marker='o', markersize = 3)
plot3=origin_acceleration.plot(timeslist, acllist, linestyle='-', marker='o', markersize = 3)

plot4=sample_position.plot(sampletimelist,sampleposlist, linestyle='-')
plot5=sample_velocity.plot(sampletimelist,sampleveclist, linestyle='-')
plot6=sample_acceleration.plot(sampletimelist,sampleacllist, linestyle='-')

plot7=omplsample_position.plot(ompltimelist,omplposlist, linestyle='-')
plot8=omplsample_velocity.plot(ompltimelist,omplveclist, linestyle='-')
plot9=omplsample_acceleration.plot(ompltimelist,omplacllist, linestyle='-')

plt.show()
