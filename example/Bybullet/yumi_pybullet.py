import sys
from tkinter import BOTH
import numpy as np
import time
import math

# rfmove_dir 根目录
rfmove_dir="/home/ziye01/rfmove"
sys.path.append(rfmove_dir+"/install/lib")

import pybullet as p
import pybullet_data

import moveit_noros as moveit
import matplotlib.pyplot as plt
from moveit_noros import rfWaypoint,PlannerSpline

from enum import Enum

class YumiRobotInfo():
    def __init__(self):
        # 各种规划组的名字
        self.left_arm_name_list=["yumi_joint_1_l","yumi_joint_2_l",
                                 "yumi_joint_7_l","yumi_joint_3_l",
                                 "yumi_joint_4_l","yumi_joint_5_l","yumi_joint_6_l"]
        self.left_arm_links=["yumi_body","yumi_link_7_l"]
        
        self.right_arm_name_list=["yumi_joint_1_r","yumi_joint_2_r",
                                  "yumi_joint_7_r","yumi_joint_3_r",
                                  "yumi_joint_4_r","yumi_joint_5_r","yumi_joint_6_r"]
        self.right_arm_links=["yumi_body","yumi_link_7_r"]

        self.both_arm_name_list=["yumi_joint_1_l","yumi_joint_2_l","yumi_joint_7_l","yumi_joint_3_l",
                                 "yumi_joint_4_l","yumi_joint_5_l","yumi_joint_6_l",
                                 "yumi_joint_1_r","yumi_joint_2_r","yumi_joint_7_r","yumi_joint_3_r",
                                 "yumi_joint_4_r","yumi_joint_5_r","yumi_joint_6_r"]
        self.grasp_name_list=["gripper_l_joint","gripper_r_joint"]
        
        # home_position
        self.home=[0,-2.2689, 2.3562,0.5236,0,0.6981,0,
                   0,-2.2689,-2.3562,0.5236,0,0.6981,0,]
        # bullet robot position
        self.startPos=[0,0,0]
        self.eulerPos=[0,0,0]

        # 各种配置文件
        self.urdf="/home/ziye01/rfmove/resources/yumi_description/urdf/yumi.urdf"
        self.srdf="/home/ziye01/rfmove/resources/yumi_description/config/yumi.srdf"
        self.kinematics="/home/ziye01/rfmove/resources/yumi_description/config/kinematics.yaml"
        self.joint_limit="/home/ziye01/rfmove/resources/yumi_description/config/joint_limits.yaml"
        self.ompl="/home/ziye01/rfmove/resources/yumi_description/config/ompl_planning.yaml"

        # 几个规划组
        self.left_arm_group="left_arm"
        self.right_arm_group="right_arm"
        self.both_arms_group="both_arms"

    def get_left_arm_name_list(self):
        return self.left_arm_name_list

    def get_right_arm_name_list(self):
        return self.right_arm_name_list

    def get_both_arm_name_list(self):
        return self.both_arm_name_list
    
    def get_grasp_name_list(self):
        return self.grasp_name_list
    
    def get_home(self):
        return self.home 
    


class YumiBulletRobot: 
    def __init__(self,yumirobotinof:YumiRobotInfo):
        # 模型文件
        self.urdf=yumirobotinof.urdf
        # home位置
        self.home_pos=yumirobotinof.get_home()
        # bullet name list
        self.bullet_name_list=["world_joint",               #0
                               "yumi_base_link_to_body",    #1
                               "yumi_joint_1_r",            #2
                               "yumi_joint_2_r",            #3
                               "yumi_joint_7_r",            #4
                               "yumi_joint_3_r",            #5
                               "yumi_joint_4_r",            #6
                               "yumi_joint_5_r",            #7
                               "yumi_joint_6_r",            #8
                               "yumi_link_7_r_joint",       #9
                               "gripper_r_joint",           #10
                               "gripper_r_joint_m",         #11
                               "yumi_joint_1_l",            #12
                               "yumi_joint_2_l",            #13
                               "yumi_joint_7_l",            #14
                               "yumi_joint_3_l",            #15
                               "yumi_joint_4_l",            #16
                               "yumi_joint_5_l",            #17
                               "yumi_joint_6_l",            #18
                               "yumi_link_7_l_joint",       #19
                               "gripper_l_joint",           #20
                               "gripper_l_joint_m",         #21
                               ]
        # 初始化位置
        print("== Initalize Pybullet ==")
        self.physicsClient=p.connect(p.GUI)
        p.setGravity(0,0,-10)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.planeId=p.loadURDF("plane.urdf")

        # 设定位置起点
        startPos=yumirobotinof.startPos
        startOrientation=p.getQuaternionFromEuler(yumirobotinof.eulerPos)
        self.boxId=p.loadURDF(self.urdf,startPos,startOrientation,useFixedBase=1)
        
        # 初始化bullet位置
        self.init()
        pass 
    
    def init(self):
        initalizeRobotState=list(self.home_pos)
        p.stepSimulation()
        initalizePybulletRobotState=self.get_robot_state()
        initalizePybulletRobotState[2:9]=np.array(initalizeRobotState)[7:]
        initalizePybulletRobotState[12:19]=np.array(initalizeRobotState)[0:7]
       
        p.setJointMotorControlArray(bodyIndex=self.boxId,
                                    jointIndices=range(22),
                                    targetPositions=initalizePybulletRobotState,
                                    controlMode=p.POSITION_CONTROL)
        for i in range(1000):
            p.stepSimulation()
        
        
       

    # 或者机器人当前状态
    def get_robot_state(self):
        p.stepSimulation()
        jointstates=p.getJointStates(bodyUniqueId =self.boxId,jointIndices=range(p.getNumJoints(self.boxId)))
        jointvalues=[]
        for i in range(len(jointstates)):
            jointvalues.append(jointstates[i][0])
        return jointvalues

    # 机器人运行
    def run(self,jointvalueslist,timeout=100000):
        for i in range(len(jointvalueslist)+timeout):
            if i<len(jointvalueslist):
                p.stepSimulation()
                p.setJointMotorControlArray(bodyIndex=self.boxId,
                                            jointIndices=range(22),
                                            targetPositions=jointvalueslist[i],
                                            controlMode=p.POSITION_CONTROL)
                
            else:
                p.stepSimulation()
                p.setJointMotorControlArray(bodyIndex=self.boxId,
                                            jointIndices=range(22),
                                            targetPositions=jointvalueslist[len(jointvalueslist)-1],
                                            controlMode=p.POSITION_CONTROL)
                
            # print(i)
            time.sleep(1./1000.)
       


class YumiPlanner:   

    def __init__(self,yumirobotinfo:YumiRobotInfo,yumi_Bullet_Robot:YumiBulletRobot):
        self.LEFT="LEFT_ARM"
        self.RIGHT="RIGHT_ARM"
        self.BOTH="BOTH_ARMS"

        print("== Initalize Planner moveit model ==")
        self.Yumi_Robot_Info=yumirobotinfo
        self.yumi_Bullet_Robot=yumi_Bullet_Robot

        # 左手规划器
        self.plannerspline=PlannerSpline(self.Yumi_Robot_Info.left_arm_group)
        self.plannerspline.init(self.Yumi_Robot_Info.urdf,
                                    self.Yumi_Robot_Info.srdf,
                                    self.Yumi_Robot_Info.kinematics,
                                    self.Yumi_Robot_Info.ompl,
                                    self.Yumi_Robot_Info.joint_limit)
        self.waypoints=[]

        # 初始化机器人规划组状态状态
        self.home_pos=self.yumi_Bullet_Robot.get_robot_state()
        self.init(self.home_pos)
        pass

    def init(self,initpos):
        numpy_pos=np.array(initpos)
        leftinitpos=numpy_pos[12:19]
        rightinitpos=numpy_pos[2:9]
        
        # 初始化三个规划组状态
        self.plannerspline.InitRobotState(leftinitpos,self.Yumi_Robot_Info.left_arm_group)
        self.plannerspline.InitRobotState(rightinitpos,self.Yumi_Robot_Info.right_arm_group)
        
        # 我们暂时不做双手规划
        # self.bothplannerspline.InitRobotState(bothpos)
        pass
    
    def plan(self,type,targetpose):
        initpos=self.yumi_Bullet_Robot.get_robot_state()
        self.init(initpos)
        self.waypoints.clear()
        if type==self.LEFT:
            self.plannerspline.CreateSplineParameterization(targetpose,self.Yumi_Robot_Info.left_arm_group,
                                                                 self.Yumi_Robot_Info.left_arm_links[0],
                                                                 self.Yumi_Robot_Info.left_arm_links[1],
                                                                 1,1,1)
            self.plannerspline.sample_by_interval(0.001)
            ompltimelist=self.plannerspline.get_sample_by_interval_times()
            joint1pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.left_arm_name_list[0]).position
            joint2pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.left_arm_name_list[1]).position
            joint3pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.left_arm_name_list[2]).position
            joint4pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.left_arm_name_list[3]).position
            joint5pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.left_arm_name_list[4]).position
            joint6pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.left_arm_name_list[5]).position
            joint7pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.left_arm_name_list[6]).position
            for i in range(len(ompltimelist)):
                initpos[12]=joint1pos[i]
                initpos[13]=joint2pos[i]
                initpos[14]=joint3pos[i]
                initpos[15]=joint4pos[i]
                initpos[16]=joint5pos[i]
                initpos[17]=joint6pos[i]
                initpos[18]=joint7pos[i]
                targetpos=initpos.copy()
                self.waypoints.append(targetpos)
        else:
            self.plannerspline.CreateSplineParameterization(targetpose,self.Yumi_Robot_Info.right_arm_group,
                                                                 self.Yumi_Robot_Info.right_arm_links[0],
                                                                 self.Yumi_Robot_Info.right_arm_links[1],
                                                                 1,1,1)
            self.plannerspline.sample_by_interval(0.001)
            ompltimelist=self.plannerspline.get_sample_by_interval_times()
            joint1pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.right_arm_name_list[0]).position
            joint2pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.right_arm_name_list[1]).position
            joint3pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.right_arm_name_list[2]).position
            joint4pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.right_arm_name_list[3]).position
            joint5pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.right_arm_name_list[4]).position
            joint6pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.right_arm_name_list[5]).position
            joint7pos=self.plannerspline.get_ompl_sample(self.Yumi_Robot_Info.right_arm_name_list[6]).position
            for i in range(len(ompltimelist)):
                initpos[2]=joint1pos[i]
                initpos[3]=joint2pos[i]
                initpos[4]=joint3pos[i]
                initpos[5]=joint4pos[i]
                initpos[6]=joint5pos[i]
                initpos[7]=joint6pos[i]
                initpos[8]=joint7pos[i]
                targetpos=initpos.copy()
                self.waypoints.append(targetpos)

    def move(self,timeout=1000000):
        self.yumi_Bullet_Robot.run(self.waypoints,timeout=timeout)
        pass


if __name__=="__main__":
    waypoint1=rfWaypoint([0.6,0.1,0.5],[math.pi,0,math.pi])
    waypoint2=rfWaypoint([0.4,0.3,0.4],[math.pi,0,math.pi])
    waypoint5=rfWaypoint([0.4,0.1,0.4],[math.pi,0,math.pi])
    yumi_info=YumiRobotInfo()
    yumi_bullet=YumiBulletRobot(yumi_info)
    yumiplanner=YumiPlanner(yumirobotinfo=yumi_info,yumi_Bullet_Robot=yumi_bullet)
    yumiplanner.plan(yumiplanner.LEFT,[waypoint1,waypoint2,waypoint5])
    yumiplanner.move(timeout=1)

    waypoint3=rfWaypoint([0.6,-0.1,0.5],[math.pi,0,math.pi])
    waypoint4=rfWaypoint([0.4,-0.3,0.4],[math.pi,0,math.pi])
    waypoint6=rfWaypoint([0.4,-0.1,0.4],[math.pi,0,math.pi])
    yumiplanner.plan(yumiplanner.RIGHT,[waypoint3,waypoint4,waypoint6])
    yumiplanner.move()

