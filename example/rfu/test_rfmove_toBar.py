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
import threading
from pyrfuniverse.envs import RFUniverseBaseEnv

class RFMoveEnv(RFUniverseBaseEnv):

    def __init__(self):
        self.id = 9874615
        #self.id=59039706
        self.left_arm = self.id * 10 + 0
        self.right_arm = self.id * 10 + 1
        self.left_gripper = self.left_arm * 10 + 0
        self.right_gripper = self.right_arm * 10 + 0
        self.interval = 20
        super().__init__(
            executable_file='/home/lees/linux_bak/RFUniverse_toBor/Player.x86_64',
            scene_file='/home/lees/linux_bak/RFUniverse_toBor/Player_Data/StreamingAssets/SceneData/RFMoveToborTest.json',
            
            #scene_file='/home/ziye01/RFUniverse_toBor/Player_Data/StreamingAssets/SceneData/test_tobor.json',
            assets=['GameObject_Box', 'GameObject_Sphere', "GameObject_Cylinder", "GameObject_Capsule"],
            articulation_channel=True,
            game_object_channel=True
        )

    def SetLeftArmPositionContinue(self, time_joint_positions: list):
        self.articulation_channel.set_action(
            'SetJointPositionContinue',
            id=self.left_arm,
            interval=self.interval,
            time_joint_positions=time_joint_positions
        )
        self._step()

    def SetRightArmPositionContinue(self, time_joint_positions: list):
        self.articulation_channel.set_action(
            'SetJointPositionContinue',
            id=self.right_arm,
            interval=self.interval,
            time_joint_positions=time_joint_positions
        )
        self._step()
        
    def GetJointPosition(self):
        kwargs = {}
        kwargs['interval'] = self.interval
        kwargs['left_grasp_point'] = self.articulation_channel.data[self.left_gripper]['grasp_point_position']
        kwargs['left_grasp_euler'] = self.articulation_channel.data[self.left_gripper]['grasp_point_rotation']
        kwargs['left_grasp_quaternion'] = self.articulation_channel.data[self.left_gripper]['grasp_point_rotation_quaternion']
        kwargs['right_grasp_point'] = self.articulation_channel.data[self.right_gripper]['grasp_point_position']
        kwargs['right_grasp_euler'] = self.articulation_channel.data[self.right_gripper]['grasp_point_rotation']
        kwargs['right_grasp_quaternion'] = self.articulation_channel.data[self.right_gripper]['grasp_point_rotation_quaternion']
        kwargs['left_joint_position'] = self.articulation_channel.data[self.left_arm]['joint_positions']
        kwargs['right_joint_position'] = self.articulation_channel.data[self.right_arm]['joint_positions']
        return kwargs

class Rf_Move_Rfuniverse():
    # 初始化 moveit 参数
    # 初始化 delta 参数，link7与末端之间的位姿差
    def __init__(self):
        self.env = RFMoveEnv()

        print("== Initalze Home and DeltaH ==")
        self.left_arm=["left_arm_joint_1","left_arm_joint_2","left_arm_joint_3","left_arm_joint_4","left_arm_joint_5","left_arm_joint_6","left_arm_joint_7"]
        self.right_arm=["right_arm_joint_1","right_arm_joint_2","right_arm_joint_3","right_arm_joint_4","right_arm_joint_5","right_arm_joint_6","right_arm_joint_7"]
        
        rfmove_dir="/home/lees/linux_bak/rfmove"
        print("== Initalize Planner moveit model ==")
        self.plannerspline=PlannerSpline("left_arm_group")
        self.plannerspline.init(rfmove_dir+"/resources/tobor/tobor_description/tobor.urdf",
                                rfmove_dir+"/resources/tobor/tobor_config/config/r300.srdf",
                                rfmove_dir+"/resources/tobor/tobor_config/config/kinematics.yaml",
                                rfmove_dir+"/resources/tobor/tobor_config/config/ompl_planning.yaml",
                                rfmove_dir+"/resources/tobor/tobor_config/config/joint_limits.yaml")
        
        self.ompltimelist_left=[]
        self.ompltimelist_right=[]
        # 移动列表
        self.time_left_joint_positions = []
        self.time_right_joint_positions = []
       
    def setrfuniverPosition(self,poslist):  
        waypoints=[]
        for i in range(len(poslist)):
            H=np.zeros((4,4))
            H[0:3,0:3]=self.eulerAngle2rotationMat(poslist[i][3:])
            H[0:3,3]=np.array(poslist[i][0:3])
            H[3,3]=1
            # yaw ,pitch ,roll
            H_t=H[0:3,3]
            H_r=self.rotationMatrixToEulerAngles(H[0:3,0:3])
            waypoint=rfWaypoint([H_t[0],H_t[1],H_t[2]],
                                [H_r[0],H_r[1],H_r[2]])
            waypoints.append(waypoint)
        return waypoints
    

    # 欧拉角转四元数
    # eular:x,y,z
    def eularToQua(self,roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]


    def planner_left(self,home=[],poslist=[[0.63,0.2,1.40,0,0,0]],):
        if(len(home)==7):
            self.Home=home
            self.plannerspline.InitRobotState(np.array(self.Home),"left_arm_group")
        else:
            home_rd=self.env.GetJointPosition()["left_joint_position"]
            self.Home=[home_rd[0]*math.pi/180,
                       home_rd[1]*math.pi/180,
                       home_rd[2]*math.pi/180,
                       home_rd[3]*math.pi/180,
                       home_rd[4]*math.pi/180,
                       home_rd[5]*math.pi/180,
                       home_rd[6]*math.pi/180]
            print("== Current Joint positions  ==")
            print(self.Home)
            self.plannerspline.InitRobotState(np.array(self.Home),"left_arm_group")

        self.plannerspline.CreateSplineParameterization(self.setrfuniverPosition(poslist=poslist),"left_arm_group","base_link","left_arm_link7",1,1,1)
        
        self.plannerspline.sample_by_interval(0.01)
        self.ompltimelist_left.clear()
        self.ompltimelist_left=self.plannerspline.get_sample_by_interval_times()

        joint1pos=self.plannerspline.get_ompl_sample(self.left_arm[0]).position
        joint2pos=self.plannerspline.get_ompl_sample(self.left_arm[1]).position
        joint3pos=self.plannerspline.get_ompl_sample(self.left_arm[2]).position
        joint4pos=self.plannerspline.get_ompl_sample(self.left_arm[3]).position
        joint5pos=self.plannerspline.get_ompl_sample(self.left_arm[4]).position
        joint6pos=self.plannerspline.get_ompl_sample(self.left_arm[5]).position
        joint7pos=self.plannerspline.get_ompl_sample(self.left_arm[6]).position

        self.time_left_joint_positions.clear()

        for i in range(len(self.ompltimelist_left)):
            self.time_left_joint_positions.append([joint1pos[i]*180/math.pi,
                                                   joint2pos[i]*180/math.pi,
                                                   joint3pos[i]*180/math.pi,
                                                   joint4pos[i]*180/math.pi,
                                                   joint5pos[i]*180/math.pi,
                                                   joint6pos[i]*180/math.pi,
                                                   joint7pos[i]*180/math.pi])
        

    def detectUnityObject(self):
         #冗余量 防止刮
        thread=0.01 #52

        self.env.asset_channel.GetRFMoveColliders()
        self.env._step()
        colliders = self.env.asset_channel.data['colliders']

        # 输出unity环境中的碰撞对象
        for one in range(len(colliders)):
            if one==0:
                continue
            for i in colliders[one]['collider']:
                if i['type'] == 'box':
                    # scale=[shape_obj.scale[1],shape_obj.scale[2],shape_obj.scale[0]]
                    moveit_box=moveit.Box(i['size'][2]+thread,
                                          i['size'][0]+thread,
                                          i['size'][1]+thread)
                    
                   
                    # [-u_t[1], u_t[2], u_t[0]],
                    # print(i['position'])
                    box_pose=moveit.EigenAffine3d()
                    box_pose.translation=[i['position'][2],
                                         -i['position'][0],
                                          i['position'][1]]  #长方体在几何中心
                      
                    print("box=========================")
                    print(i["size"])
                   
                    # [u_r[1], -u_r[2], -u_r[0]],
                    # 我们已经不用欧拉角了
                    '''
                    box_pose.quaternion=self.eularToQua(-i['rotation'][2],
                                                         i['rotation'][0],
                                                        -i['rotation'][1]) # x,y,z

                    '''
                    # 轴角法，角度是求逆向函数的
                    box_pose.quaternion=[-i['rotation'][2],i['rotation'][0],-i['rotation'][1],i['rotation'][3]]
                    print(box_pose.translation)
                    print(box_pose.quaternion)
                    self.plannerspline.AddCollectionObject("unity_rfmove_"+str(i),moveit_box,box_pose)
                    print("box end==================")
                elif i['type'] == 'sphere':
                    moveit_Sphere=moveit.Sphere(i['radius']+thread)  #r
                    
                    Shpere_pose=moveit.EigenAffine3d()
                    
                    print("sphere ===================")
                    print(i['radius'])
                    Shpere_pose.translation=[i['position'][2],
                                            -i['position'][0],
                                             i['position'][1]]  #长方体在几何中心
                    # 球形本来就没有角度一说，我们也不用欧拉角
                    Shpere_pose.quaternion=self.eularToQua(0,0,0)
                    print(Shpere_pose.translation)
                    print("sphere end ===================")
                    self.plannerspline.AddCollectionObject("unity_rfmove_"+str(i), moveit_Sphere,Shpere_pose)

                elif i['type'] == 'capsule':    
                    moveit_Cylinder=moveit.Cylinder(i['radius']+thread,
                                                    i['height']+thread)
        
                    # 在moveit这一侧其实是圆柱体
                    Cylinder_pose=moveit.EigenAffine3d()
                    print("clindy=========================")

                    # [-u_t[1], u_t[2], u_t[0]],
                    # print(i['position'])
                    Cylinder_pose.translation=[i['position'][2],
                                              -i['position'][0],
                                               i['position'][1]]  #都是在几何中心
                   
                    Cylinder_pose.quaternion=[-i['rotation'][2],i['rotation'][0],-i['rotation'][1],i['rotation'][3]]
          

                    self.plannerspline.AddCollectionObject("unity_rfmove_"+str(i),moveit_Cylinder,Cylinder_pose)


    def planner_right(self,home=[],poslist=[[0.63,-0.2,1.40,0,0,0]],):
        if(len(home)==7):
            self.Home=home
            self.plannerspline.InitRobotState(np.array(self.Home),"right_arm_group")
        else:
            home_rd=self.env.GetJointPosition()["right_joint_position"]
            self.Home=[home_rd[0]*math.pi/180,
                       home_rd[1]*math.pi/180,
                       home_rd[2]*math.pi/180,
                       home_rd[3]*math.pi/180,
                       home_rd[4]*math.pi/180,
                       home_rd[5]*math.pi/180,
                       home_rd[6]*math.pi/180]
            print("== Current Joint positions  ==")
            print(home_rd)
            self.plannerspline.InitRobotState(np.array(self.Home),"right_arm_group")

        self.plannerspline.CreateSplineParameterization(self.setrfuniverPosition(poslist=poslist),"right_arm_group","base_link","right_arm_link7",1,1,1)
        
        self.plannerspline.sample_by_interval(0.01)
        self.ompltimelist_right.clear()
        self.ompltimelist_right=self.plannerspline.get_sample_by_interval_times()

        joint1pos=self.plannerspline.get_ompl_sample(self.right_arm[0]).position
        joint2pos=self.plannerspline.get_ompl_sample(self.right_arm[1]).position
        joint3pos=self.plannerspline.get_ompl_sample(self.right_arm[2]).position
        joint4pos=self.plannerspline.get_ompl_sample(self.right_arm[3]).position
        joint5pos=self.plannerspline.get_ompl_sample(self.right_arm[4]).position
        joint6pos=self.plannerspline.get_ompl_sample(self.right_arm[5]).position
        joint7pos=self.plannerspline.get_ompl_sample(self.right_arm[6]).position

        self.time_right_joint_positions.clear()

        for i in range(len(self.ompltimelist_right)):
            self.time_right_joint_positions.append([joint1pos[i]*180/math.pi,
                                                    joint2pos[i]*180/math.pi,
                                                    joint3pos[i]*180/math.pi,
                                                    joint4pos[i]*180/math.pi,
                                                    joint5pos[i]*180/math.pi,
                                                    joint6pos[i]*180/math.pi,
                                                    joint7pos[i]*180/math.pi])

        print(self.time_right_joint_positions[0])
        
     # Unity 下的旋转矩阵转欧拉角
    def unityRotationMatrixToEulerAngles(self,R):
        assert self.isRotationMatrix(R)

        sz=math.sqrt(R[1,1]*R[1,1]+R[2,1]*R[2,1])

        singular=sz<1e-6
        
        if not singular:
            x=math.atan2(R[2,1],R[1,1])
            y=math.atan2(R[0,2],R[0,0])
            z=math.atan2(-R[0,1],sz)
        else:
            x=0
            y=math.atan2(-R[2,0],R[2,2])
            z=math.atan2(-R[0,1],sz)

        return np.array([x, y, z])

    # 欧拉角转Mat
    # theta:x,y,z
    # 欧拉角转旋转矩阵的时候是z,y,x方向的
    def eulerAngle2rotationMat(self,theta):
        R_x=np.array([[1,          0,                   0],
                    [0,          math.cos(theta[0]),  -math.sin(theta[0])],
                    [0,          math.sin(theta[0]),   math.cos(theta[0])]])
        
        R_y=np.array([[math.cos(theta[1]) ,   0,    math.sin(theta[1])],
                    [0                  ,   1,                     0],
                    [-math.sin(theta[1]),   0,    math.cos(theta[1])]])
        
        R_z=np.array([[math.cos(theta[2]),  -math.sin(theta[2])   ,0],
                      [math.sin(theta[2]),  math.cos(theta[2])    ,0],
                      [0                 ,  0                    ,1]])

        R=np.dot(R_z,np.dot(R_y,R_x))
        return R

    # 旋转矩阵转欧拉角
    def rotationMatrixToEulerAngles(self,R) :
    
       # assert(self.isRotationMatrix(R))
        
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        
        singular = sy < 1e-6
    
        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
    
        return np.array([x, y, z])
    
if __name__=="__main__":
    move=Rf_Move_Rfuniverse()
    # 检查环境中可碰撞对象，为了进行规避障碍物的路径规划
    move.detectUnityObject()
    
    # step1:
    # 左臂运动，waypoint
    plist=[[0.63,0.4,1.0,math.pi/2,0,0],
           [0.0,0.7,1.5,0,0,math.pi/4],
           [0.4,0.7,0.7,math.pi/2,0,math.pi/2],
           [0.6,0.0,0.9,math.pi/2,0,0],
           [0.6,0.4,0.9,math.pi/2,0,0],]
           
    move.planner_left(poslist=plist)
    
    # 设定左臂的运动路径
    move.env.SetLeftArmPositionContinue(move.time_left_joint_positions)
    # 开始启动仿真器，开始运行
    for i in range(len(move.ompltimelist_left)+10):
        move.env._step()

    # 同上，这里是右臂的运行
    plist=[[0.63,-0.3,1.0,-math.pi/2,0,0],
           [0.0,-0.7,1.5,0,0,-math.pi/4],
           [0.4,-0.7,0.7,-math.pi/2,0,-math.pi/2],
           [0.6,0.0,0.9,-math.pi/2,0,0],
           [0.6,-0.4,0.9,-math.pi/2,0,0],]
    move.planner_right(poslist=plist)
    move.env.SetRightArmPositionContinue(move.time_right_joint_positions)
    for i in range(len(move.ompltimelist_right)+10):
        move.env._step()

    # 左臂和右臂同时运行1
    p_r=[[0.5,-0.6,0.9,-math.pi/2,0,0]]
    p_l=[[0.5,0.6,0.9,math.pi/2,0,0]]
    move.planner_left(poslist=p_l)
    move.planner_right(poslist=p_r)
    move.env.SetLeftArmPositionContinue(move.time_left_joint_positions)
    move.env.SetRightArmPositionContinue(move.time_right_joint_positions)
    get_step_num=lambda x,y:max(x,y)
    step_num=get_step_num(len(move.ompltimelist_left),len(move.ompltimelist_right))
    for i in range(step_num+10):
        move.env._step()
    
    # 左臂和右臂同时运行2
    p_r=[[0.9,-0.2,0.8,-math.pi/2,0,0]]
    p_l=[[0.9,0.2,0.8,math.pi/2,0,0]]
    move.planner_left(poslist=p_l)
    move.planner_right(poslist=p_r)
    move.env.SetLeftArmPositionContinue(move.time_left_joint_positions)
    move.env.SetRightArmPositionContinue(move.time_right_joint_positions)
    get_step_num=lambda x,y:max(x,y)
    step_num=get_step_num(len(move.ompltimelist_left),len(move.ompltimelist_right))
    for i in range(step_num+10):
        move.env._step()
    
    
    # 右臂运行
    p_r=[[0.9,-0.2,1.2,-math.pi/2,0,0]]
    move.planner_right(poslist=p_r)
    move.env.SetRightArmPositionContinue(move.time_right_joint_positions)
    for i in range(len(move.ompltimelist_right)+10):
        move.env._step()
    
    # 左臂运行
    p_l=[[0.9,0.2,1.2,math.pi/2,0,0]]
    move.planner_left(poslist=p_l)
    move.env.SetLeftArmPositionContinue(move.time_left_joint_positions)
    for i in range(len(move.ompltimelist_left)+10):
        move.env._step()

    # 右臂再运行
    p_r=[[0.6,-0.2,1.5,-math.pi/2,0,0]]
    move.planner_right(poslist=p_r)
    move.env.SetRightArmPositionContinue(move.time_right_joint_positions)
    for i in range(len(move.ompltimelist_right)+10):
        move.env._step()

    # 左臂再运行
    p_l=[[0.6,0.2,1.5,math.pi/2,0,0]]
    move.planner_left(poslist=p_l)
    move.env.SetLeftArmPositionContinue(move.time_left_joint_positions)
    for i in range(len(move.ompltimelist_left)+100):
        move.env._step()
    print(move.env.GetJointPosition()["left_grasp_point"])
    print(move.env.GetJointPosition()["right_grasp_point"])