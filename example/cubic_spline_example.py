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

joint_name_list=["panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"]

waypoint1=rfWaypoint([0.34,0,0.28],[0,math.pi,math.pi/2])
waypoint2=rfWaypoint([0.33,0.2,0.38],[0,math.pi,math.pi/2])
#waypoint3=rfWaypoint([0.20,-0.2,0.54],[0,math.pi,math.pi/2])

print("== Initalize Planner moveit model ==")
plannerspline=PlannerSpline("panda_arm")
plannerspline.init("../resources/franka/urdf/franka_convert.urdf",
                   "../resources/franka/urdf/panda.srdf",
                   "../resources/franka/config/kinematics.yaml",
                   "../resources/franka/config/ompl_planning.yaml",
                   "../resources/franka/config/joint_limits.yaml")

# 长方体添加
def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

'''
r=[math.pi/4,math.pi/4,math.pi/2]
t=[0.0,0.3,0.2]
moveit_box=moveit.Box(0.06,0.06,0.4)
box_pose=moveit.EigenAffine3d()
box_pose.translation=t  #都是在几何中心
box_pose.quaternion=euler_to_quaternion(r[0],r[1],r[2])

plannerspline.AddCollectionObject("box_1",moveit_box,box_pose)
'''
# 移除长方体
# plannerspline.RemoveObject("box_1")

# 添加圆柱形
'''
moveit_Cylinder=moveit.Cylinder(0.1,0.4)  #r,l
Cylinder_pose=moveit.EigenAffine3d()
Cylinder_pose.translation=[0.0,0.3,0.2]  #都是在几何中心
Cylinder_pose.quaternion=[0,0,0,1]

plannerspline.AddCollectionObject("Cylinder_1",moveit_Cylinder,Cylinder_pose)

# 移除圆柱形
plannerspline.RemoveObject("Cylinder_1")
'''

# 添加圆形
'''
moveit_Sphere=moveit.Sphere(0.2)  # r
Sphere_pose=moveit.EigenAffine3d()
Sphere_pose.translation=[0.0,0.4,0.2]  #都是在几何中心
Sphere_pose.quaternion=[0,0,0,1]

plannerspline.AddCollectionObject("Sphere_1",moveit_Sphere,Sphere_pose)

'''
# 移除圆形
# plannerspline.RemoveObject("Sphere_1")

# 添加圆锥形
'''
moveit_Cone=moveit.Cone(0.1,0.4) # r,l
Cone_pose=moveit.EigenAffine3d()
Cone_pose.translation=[0.0,0.4,0.2]  #都是在几何中心
Cone_pose.quaternion=[0,0,0,1]
plannerspline.AddCollectionObject("Cone_1",moveit_Cone,Cone_pose)
# 移除圆形
# plannerspline.RemoveObject("Cone_1")
'''

# 添加平面
'''
moveit_Plane=moveit.Plane(1,0,0,-0.3) # r,l
Plane_pose=moveit.EigenAffine3d()
Plane_pose.translation=[0.0,0.0,0]  #都是在几何中心
Plane_pose.quaternion=[0,0,0,1]
plannerspline.AddCollectionObject("Cone_1",moveit_Plane,Plane_pose)
# 移除圆形
# plannerspline.RemoveObject("Plane_1")
'''

# 移除所有点
# plannerspline.clearObjects()

print("== Initalize Pybullet ==")
physicsClient=p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId=p.loadURDF("plane.urdf")
p.setAdditionalSearchPath("../resources/franka/urdf/")

# 设定位置起点
startPos=[0,0,0]
#从欧拉角设定四元素
startOrientation=p.getQuaternionFromEuler([0,0,0])
boxId=p.loadURDF("franka_convert.urdf",startPos,startOrientation,useFixedBase=1)

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

createBoxForpyBullet([0.03, 0.03, 0.2],[0,0.3,0.2],[ 0, 0, 0, 1])

createBoxForpyBullet([0.5, 0.03, 0.3],[0.6,0.0,0.9],[0.786566,0.0794593,-0.103553,0.603553])

createBoxForpyBullet([0.15, 0.03, 0.2],[0.4,0.0,0.4],[ 0.707107, 0, 0, 0.707107])

createBoxForpyBullet([0.03, 0.03, 0.3],[-0.2,0.5,0.4],[ 0.683013, 0.183013, -0.183013, 0.683013])
'''
# pybullet 添加圆形

sphere_collision_id =p.createCollisionShape(shapeType = p.GEOM_SPHERE,
                                       radius  = 0.15153881907463074,
                                       collisionFramePosition = [0,0,0])
sphere_visual_id = p.createVisualShape(shapeType = p.GEOM_SPHERE,
                                       radius = 0.15153881907463074,
                                       visualFramePosition = [0,0,0],
                                       rgbaColor = [1, 0.2, 0, 0.9],
                                       specularColor = [0, 0, 0])
sphere_body_id = p.createMultiBody(baseMass = 0, # 0 means this object is fixed
                                       baseCollisionShapeIndex = sphere_collision_id,
                                       baseVisualShapeIndex = sphere_visual_id,
                                       basePosition = [0.38326538,1.13855827,0.22807938],  #都是在几何中心
                                       baseOrientation = [0,0,0,1], 
                                       baseInertialFramePosition = [0,0,0])

'''

# pybullet 添加圆柱形
'''
clindyer_collision_id =p.createCollisionShape(shapeType = p.GEOM_CYLINDER,
                                       radius  = 0.22323070466518402,
                                       collisionFramePosition = [0,0,0],
                                       height  = 0.8929228186607361)

clindyer_visual_id = p.createVisualShape(shapeType = p.GEOM_CYLINDER,
                                         radius = 0.22323070466518402,
                                         length=0.8929228186607361,
                                         visualFramePosition = [0,0,0],
                                         rgbaColor = [1, 0.2, 0, 0.9],
                                         specularColor = [0, 0, 0])

clindyer_body_id = p.createMultiBody(baseMass = 0, # 0 means this object is fixed
                                     baseCollisionShapeIndex = clindyer_collision_id,
                                     baseVisualShapeIndex = clindyer_visual_id,
                                     basePosition =  [ 2.16998529, -0.59760112,  0.69395792],  #都是在几何中心
                                     baseOrientation = [ 0.279206, -0.350397, -0.110265, 0.887191], 
                                     baseInertialFramePosition = [0,0,0])
'''

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
plannerspline.CreateSplineParameterization([waypoint1,waypoint2],"panda_arm","panda_link0","panda_link7",0.1,0.1,0.1)
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


for i in range(len(ompltimelist)+250000000000000000000):

    if i<len(ompltimelist):
        p.stepSimulation()
        p.setJointMotorControlArray(bodyIndex=boxId,
                                    jointIndices=range(len(joint_name_list)),
                                    targetPositions=waypoints[i],
                                    controlMode=p.POSITION_CONTROL)
    else:
        p.stepSimulation()
        p.setJointMotorControlArray(bodyIndex=boxId,
                                    jointIndices=range(len(joint_name_list)),
                                    targetPositions=waypoints[len(ompltimelist)-1],
                                    controlMode=p.POSITION_CONTROL)
        print("======================bulletPoselin8=========================")
        print(p.getLinkState(boxId,6,)[4])
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


