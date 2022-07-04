from gettext import translation
from multiprocessing import pool
from ntpath import join
import sys
from turtle import pos, position
import numpy as np
import time
import math
# rfmove_dir 根目录
rfmove_dir="/home/lees/linux_bak/rfmove"
sys.path.append(rfmove_dir+"/install/lib")


import moveit_noros as moveit
import matplotlib.pyplot as plt
from moveit_noros import rfWaypoint,PlannerSpline
import threading
from pyrfuniverse.envs import RFUniverseBaseEnv

class RFMoveEnv(RFUniverseBaseEnv):

    def __init__(self):
        self.id = 69413587
        self.gripper_id = self.id * 10
        self.interval = 20
        super().__init__(
            executable_file='/home/lees/linux_bak/RFUniverse_toBor/Player.x86_64',
            scene_file='/home/lees/linux_bak/RFUniverse_toBor/Player_Data/StreamingAssets/SceneData/RFMoveTest2.json',
            assets=['GameObject_Box','GameObject_Sphere',"GameObject_Capsule"],
            articulation_channel=True,
            game_object_channel=True
        )
        
    def SetJointPositionContinue(self, time_joint_positions: list) -> None:
        self.articulation_channel.set_action(
            'SetJointPositionContinue',
            id=self.id,
            interval=self.interval,
            time_joint_positions=time_joint_positions
        )
        self._step()

    def GetJointPosition(self) -> dict:
        kwargs = {}
        kwargs['interval'] = self.interval
        grasp_point = self.articulation_channel.data[self.gripper_id]['positions'][3]
        grasp_euler = self.articulation_channel.data[self.gripper_id]['rotations'][3]
        kwargs['grasp_point'] = [grasp_point[2], -grasp_point[0], grasp_point[1]]
        kwargs['grasp_eluer'] = [grasp_euler[2], -grasp_euler[0], grasp_euler[1]]
        #kwargs['grasp_eluer'] = [-grasp_euler[2], grasp_euler[0], -grasp_euler[1]]
        kwargs['joint_position'] = self.articulation_channel.data[self.id]['joint_positions']
        return kwargs

class ShapeObject():
    def __init__(self,shape="Box",id=1,scale=[0.06,0.06,0.4],pose=[0,0.3,0,0,0,0],color=[0,0,0,1]):
        self.shape=shape
        self.id=id
        self.scale=scale
        self.pose=pose
        self.color=color

class Rf_Move_Rfuniverse():
    # 初始化 moveit 参数
    # 初始化 delta 参数，link7与末端之间的位姿差
    def __init__(self,Home=[0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]):
        print("== Initializer RFUniverse Env ==")
        self.env = RFMoveEnv()
        
    
        print("== Initalze Home and DeltaH ==")
        self.joint_name_list=["panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"]
        self.Home=Home
        
        self.delta=np.array([[-0.707,  0.707,    0,  0],
                             [-0.707, -0.707,    0,  0],
                             [ 0    ,      0,  1.0, -0.21],
                             [ 0    ,      0,    0,  1]])
        
        
        '''
        self.delta=np.array([[ 7.04764836e-01,  7.09438110e-01,  2.02346842e-03,  2.39871493e-03],
                             [-7.09391101e-01,  7.04746230e-01,  9.84978817e-03,  1.40684186e-04],
                             [-8.41384684e-03, -5.50635385e-03,  9.99949442e-01, -2.11556604e-01],
                             [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        '''

        print("== Initalize Planner moveit model ==")
        self.plannerspline=PlannerSpline("panda_arm")
        self.plannerspline.init(rfmove_dir+"/resources/franka/urdf/franka_convert.urdf",
                                rfmove_dir+"/resources/franka/urdf/panda.srdf",
                                rfmove_dir+"/resources/franka/config/kinematics.yaml",
                                rfmove_dir+"/resources/franka/config/ompl_planning.yaml",
                                rfmove_dir+"/resources/franka/config/joint_limits.yaml")
        #跳出标志符号
        self.stopflag=False
        #代表当前行为已经完成用的计数器
        self.count=0
        #数据锁
        self.lock=threading.Lock()
        # 移动列表
        self.time_joint_positions = []
        
        #t=threading.Thread(target=self.unvierse_run, name='aa')
        #t.start()

    def unvierse_run(self):
        while(1):
            self.lock.acquire()
            if(len(self.time_joint_positions)!=0):
                self.count+=1
            self.lock.release()
            if(self.stopflag):
                break
            self.env._step()
        
    # 重新生成link7的误差矩阵
    def setDelta(self,DeltaH):
        self.delta=DeltaH
        print("================delta=====================")
        print(self.delta)

    # 升级与link7之间的误差矩阵
    # 方向如下 updateH(其实是正向的逆)->self.delta->link7
    def updateDelta(self,updateH=np.array([[-1, 0,  0   ,0],
                                           [ 0, 1,  0   ,0],
                                           [ 0, 0,  -1  ,0],
                                           [ 0, 0,  0   ,1]])):
        self.delta=updateH.dot(self.delta)

    # 完成规划
    # arg1： home，初始位置，如果没有输入/输入错误，home位置为当前位置
    # arg2： poslit waypoint列表
    def planner(self,home=[0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279],
                     poslist=[[0.33,0,0.38,0,math.pi,-math.pi/4],
                              [0.33,0,0.58,0,math.pi/2,-math.pi/4],
                              [0.33,0,0.58,0,math.pi,-math.pi/4],]):
        if(len(home)==7):
            self.Home=home
            self.plannerspline.InitRobotState(np.array(self.Home),"panda_arm")
        else:
            home_rd=self.env.GetJointPosition()["joint_position"]
            self.Home=[home_rd[0]*math.pi/180,
                       home_rd[1]*math.pi/180,
                       home_rd[2]*math.pi/180,
                       home_rd[3]*math.pi/180,
                       home_rd[4]*math.pi/180,
                       home_rd[5]*math.pi/180,
                       home_rd[6]*math.pi/180]
            print("== Current Joint positions  ==")
            print(self.Home)
            self.plannerspline.InitRobotState(np.array(self.Home),"panda_arm")

        print(poslist)
        self.plannerspline.CreateSplineParameterization(self.setrfuniverPosition(poslist=poslist),"panda_arm","panda_link0","panda_link7")
        self.timeslist=self.plannerspline.getTimeList()

        self.plannerspline.sample_by_interval(0.01)

        self.ompltimelist=self.plannerspline.get_sample_by_interval_times()


        joint1pos=self.plannerspline.get_ompl_sample(self.joint_name_list[0]).position
        joint2pos=self.plannerspline.get_ompl_sample(self.joint_name_list[1]).position
        joint3pos=self.plannerspline.get_ompl_sample(self.joint_name_list[2]).position
        joint4pos=self.plannerspline.get_ompl_sample(self.joint_name_list[3]).position
        joint5pos=self.plannerspline.get_ompl_sample(self.joint_name_list[4]).position
        joint6pos=self.plannerspline.get_ompl_sample(self.joint_name_list[5]).position
        joint7pos=self.plannerspline.get_ompl_sample(self.joint_name_list[6]).position


        self.time_joint_positions = []
        self.time_joint_positions.clear()
        

        for i in range(len(self.ompltimelist)):
            self.time_joint_positions.append([joint1pos[i]*180/math.pi,
                                              joint2pos[i]*180/math.pi,
                                              joint3pos[i]*180/math.pi,
                                              joint4pos[i]*180/math.pi,
                                              joint5pos[i]*180/math.pi,
                                              joint6pos[i]*180/math.pi,
                                              joint7pos[i]*180/math.pi])

    def detectUnityObject(self):
         #冗余量 防止刮
        thread=0.02

        self.env.asset_channel.GetRFMoveColliders()
        self.env._step()
        colliders = self.env.asset_channel.data['colliders']

        # 输出unity环境中的碰撞对象
        for one in colliders:
            for i in one['collider']:
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
                    
                    
                    # [u_r[1], -u_r[2], -u_r[0]],
                    # 我们已经不用欧拉角了
                    '''
                    box_pose.quaternion=self.eularToQua(-i['rotation'][2],
                                                         i['rotation'][0],
                                                        -i['rotation'][1]) # x,y,z

                    '''
                    # 轴角法，角度是求逆向函数的
                    box_pose.quaternion=[-i['rotation'][2],i['rotation'][0],-i['rotation'][1],i['rotation'][3]]
                       
                    self.plannerspline.AddCollectionObject("unity_rfmove_"+str(i),moveit_box,box_pose)

                elif i['type'] == 'sphere':
                    moveit_Sphere=moveit.Sphere(i['radius']+thread)  #r
                    
                    Shpere_pose=moveit.EigenAffine3d()
                    
                    
                    Shpere_pose.translation=[i['position'][2],
                                            -i['position'][0],
                                             i['position'][1]]  #长方体在几何中心
                    # 球形本来就没有角度一说，我们也不用欧拉角
                    Shpere_pose.quaternion=self.eularToQua(0,0,0)
                    self.plannerspline.AddCollectionObject("unity_rfmove_"+str(i), moveit_Sphere,Shpere_pose)

                elif i['type'] == 'capsule':
                    print(i)        
                    moveit_Cylinder=moveit.Cylinder(i['radius']+thread,
                                                    i['height']+thread)
        
                    # 在moveit这一侧其实是圆柱体
                    Cylinder_pose=moveit.EigenAffine3d()

                    # [-u_t[1], u_t[2], u_t[0]],
                    # print(i['position'])
                    Cylinder_pose.translation=[i['position'][2],
                                              -i['position'][0],
                                               i['position'][1]]  #都是在几何中心
                   
                    Cylinder_pose.quaternion=[-i['rotation'][2],i['rotation'][0],-i['rotation'][1],i['rotation'][3]]
                    print(Cylinder_pose.quaternion)

                    self.plannerspline.AddCollectionObject("unity_rfmove_"+str(i),moveit_Cylinder,Cylinder_pose)
                    

    def AddObject(self,shape_obj):
        #冗余量 防止刮
        thread=0.02
        
        #moveit 端的变换矩阵
        H=np.zeros((4,4))
        H[0:3,0:3]=self.eulerAngle2rotationMat([shape_obj.pose[3],shape_obj.pose[4],shape_obj.pose[5]])
      
        H[0:3,3]=np.array([shape_obj.pose[0],shape_obj.pose[1],shape_obj.pose[2]])
        H[3,3]=1

        

        if(shape_obj.shape=='Box'):
            # 最新版的不需要中心补偿            
            H_z=np.array([[1.0,   0.0   ,0.0   ,0.0],
                          [0.0,   1.0   ,0.0   ,0.0],
                          [0.0,   0.0   ,1.0   ,0.0],
                          [0.0,   0.0   ,0.0   ,1.0]]) 
            H_set=H.dot(H_z)
            #取回 平移部分  x,y,z
            u_t=H_set[0:3,3]
            #取回 欧拉角部分  r,p,y 
            u_r=self.rotationMatrixToEulerAngles(H_set[0:3,0:3])
            # 设定moveit部分
            moveit_box=moveit.Box(shape_obj.scale[0]+thread,shape_obj.scale[1]+thread,shape_obj.scale[2]+thread)
            box_pose=moveit.EigenAffine3d()
            box_pose.translation=[shape_obj.pose[0],shape_obj.pose[1],shape_obj.pose[2]]  #长方体在几何中心
            
            box_pose.quaternion=self.eularToQua(shape_obj.pose[3],shape_obj.pose[4],shape_obj.pose[5]) # x,y,z
            self.plannerspline.AddCollectionObject(str(shape_obj.id),moveit_box,box_pose)

            print(shape_obj.scale)
            print(box_pose.quaternion)
            print(box_pose.translation)

            
            # 设定unity部分 
            self.env.asset_channel.set_action(
                'InstanceObject',
                name='GameObject_Box',
                id=shape_obj.id
            )
            self.env.game_object_channel.set_action(
                'SetTransform',
                id=shape_obj.id,
                position=[-u_t[1],             u_t[2],              u_t[0]],
                rotation=[u_r[1]*180/math.pi, -u_r[2]*180/math.pi, -u_r[0]*180/math.pi],
                scale=[shape_obj.scale[1],shape_obj.scale[2],shape_obj.scale[0]]
            )
            self.env.game_object_channel.set_action(
                'SetColor',
                id=shape_obj.id,
                color=shape_obj.color
            )
           
        if(shape_obj.shape=="Capsule"):
             # 最新版的不需要中心补偿            
            H_z=np.array([[1.0,   0.0   ,0.0   ,0.0],
                          [0.0,   1.0   ,0.0   ,0.0],
                          [0.0,   0.0   ,1.0   ,0.0],
                          [0.0,   0.0   ,0.0   ,1.0]]) 

            H_set=H.dot(H_z)
            #取回 平移部分  x,y,z
            u_t=H_set[0:3,3]
            #取回 欧拉角部分  r,p,y
            u_r=self.rotationMatrixToEulerAngles(H_set[0:3,0:3])
         
            # 设定moveit部分
            moveit_Cylinder=moveit.Cylinder(shape_obj.scale[0]+thread,shape_obj.scale[1]+thread)  #r,l
            Cylinder_pose=moveit.EigenAffine3d()
            Cylinder_pose.translation=[shape_obj.pose[0],shape_obj.pose[1],shape_obj.pose[2]]  #都是在几何中心
            Cylinder_pose.quaternion=self.eularToQua(shape_obj.pose[3],shape_obj.pose[4],shape_obj.pose[5])
            self.plannerspline.AddCollectionObject(str(shape_obj.id),moveit_Cylinder,Cylinder_pose)
            
            # 设定unity部分 
            self.env.asset_channel.set_action(
                'InstanceObject',
                name='GameObject_Capsule',
                id=shape_obj.id
            )
            self.env.game_object_channel.set_action(
                'SetTransform',
                id=shape_obj.id,
                position=[-u_t[1],             u_t[2],              u_t[0]],
                rotation=[u_r[1]*180/math.pi, -u_r[2]*180/math.pi, -u_r[0]*180/math.pi],
                scale=[shape_obj.scale[0],shape_obj.scale[0],shape_obj.scale[1]]
            )
            self.env.game_object_channel.set_action(
                'SetColor',
                id=shape_obj.id,
                color=shape_obj.color
            )
        
        if(shape_obj.shape=="Sphere"):
            H_z=np.array([[1.0,   0.0   ,0.0   ,0.0],
                          [0.0,   1.0   ,0.0   ,0.0],
                          [0.0,   0.0   ,1.0   ,0.0],
                          [0.0,   0.0   ,0.0   ,1.0]])
            H_set=H.dot(H_z)
            #取回 平移部分  x,y,z
            u_t=H_set[0:3,3]
            #取回 欧拉角部分  r,p,y
            u_r=self.rotationMatrixToEulerAngles(H_set[0:3,0:3])
        

            # 设定moveit部分
            moveit_Sphere=moveit.Sphere(shape_obj.scale[0]+thread)  #r
            Shpere_pose=moveit.EigenAffine3d()
            Shpere_pose.translation=[shape_obj.pose[0],shape_obj.pose[1],shape_obj.pose[2]]  #都是在几何中心
            Shpere_pose.quaternion=self.eularToQua(shape_obj.pose[3],shape_obj.pose[4],shape_obj.pose[5])
            self.plannerspline.AddCollectionObject(str(shape_obj.id),moveit_Sphere,Shpere_pose)
            
          
            # 设定unity部分 
            self.env.asset_channel.set_action(
                'InstanceObject',
                name='GameObject_Sphere',
                id=shape_obj.id
            )
            self.env.game_object_channel.set_action(
                'SetTransform',
                id=shape_obj.id,
                position=[-u_t[1],             u_t[2],              u_t[0]],
                rotation=[u_r[1]*180/math.pi, -u_r[2]*180/math.pi, -u_r[0]*180/math.pi],
                scale=[shape_obj.scale[0],shape_obj.scale[0],shape_obj.scale[0]]
            )
            
            self.env.game_object_channel.set_action(
                'SetColor',
                id=shape_obj.id,
                color=shape_obj.color
            )
            pass

    # 获取仿真器当前末端的Translation:x,y,z
    # 获取仿真器当前末端的Rotation:x,y,z 单位:rad
    def get_TCP_t(self):
        T_t=self.env.GetJointPosition()["grasp_point"]
        return T_t

    def get_TCP_r(self):
        T_r_deg=self.env.GetJointPosition()["grasp_eluer"]
        T_r=[T_r_deg[0]*math.pi/180.0,T_r_deg[1]*math.pi/180.0,T_r_deg[2]*math.pi/180.0]
        return T_r

    def get_last_planner_waypoint(self):
        last_index=len(self.time_joint_positions)-1
        return [self.time_joint_positions[last_index][0]*math.pi/180,
                self.time_joint_positions[last_index][1]*math.pi/180,
                self.time_joint_positions[last_index][2]*math.pi/180,
                self.time_joint_positions[last_index][3]*math.pi/180,
                self.time_joint_positions[last_index][4]*math.pi/180,
                self.time_joint_positions[last_index][5]*math.pi/180,
                self.time_joint_positions[last_index][6]*math.pi/180]
    
    # 运行仿真器
    def run(self,extr_time):
        self.env.SetJointPositionContinue(self.time_joint_positions)
        '''
        while(1):
            
            if(self.count>len(self.time_joint_positions)+extr_time):
                self.lock.acquire()
                self.count=0
                self.time_joint_positions=[]
                self.lock.release()
                break
            
            time.sleep(0.02)
        '''
        for i in range(len(self.ompltimelist)+extr_time):
            self.env._step()

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

    # 判断旋转矩阵有效性
    def isRotationMatrix(self,R) :
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

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

    # 使用delta重新计算waypoint link7输入的位置
    def setrfuniverPosition(self,poslist):  
        waypoints=[]
        for i in range(len(poslist)):
            H=np.zeros((4,4))
            H[0:3,0:3]=self.eulerAngle2rotationMat(poslist[i][3:])
            H[0:3,3]=np.array(poslist[i][0:3])
            H[3,3]=1
            H_set=H.dot(self.delta)
            # yaw ,pitch ,roll
            H_t=H_set[0:3,3]
            H_r=self.rotationMatrixToEulerAngles(H_set[0:3,0:3])
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

    # 计算误差矩阵End->Link7
    def computeEnd2Link7(self,
                         gEuler,   # 设定Link7的位置,欧拉角,wx,wy,wz
                         gTrans,   # 设定Link7的位置,平移部分,x,y,z
                         REuler,   # 运行结果的位置，欧拉角,wx,wy,wz
                         RTrans):  # 运行结果的位置，平移部分,x,y,z
        # 计算设定目标，计算Link7的目标矩阵
        gH=np.zeros((4,4))
        gH[0:3,0:3]=self.eulerAngle2rotationMat(gEuler)
        gH[0:3,3]=np.array(gTrans)
        gH[3,3]=1
        # 计算结果设定，计算End的结果矩阵
        RH=np.zeros((4,4))
        RH[0:3,0:3]=self.eulerAngle2rotationMat(REuler)
        RH[0:3,3]=np.array(RTrans)
        RH[3,3]=1
        #计算End->Link7 的H
        H=np.linalg.inv(RH).dot(gH)
        return H
        


    

if __name__=="__main__":

    # 避障1，检测unity中的物体,进行物体规避
    move2=Rf_Move_Rfuniverse()

    #初始化位置
    home=[0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]  #制定启动的home状态，第一次使用必须制定
    # home=[] 为从当前位置开始规划
    poslist=[[0.6,0.0,0.68,0,math.pi/2,0],
              [0.0,0.6,0.68,0,math.pi/2,math.pi/2],
              [-0.6,0.0,0.68,0,math.pi/2,math.pi],
              [-0.9,0.0,0.38,0,math.pi/2,math.pi],
              [0.0,-0.9,0.38,0,math.pi/2,3*math.pi/2],
              [0.9,0.0,0.38,0,math.pi/2,2*math.pi]]
             
    # 检测环境中的碰撞对象，然后加入到规划中
    # 注销这一行就会碰撞
    move2.detectUnityObject()
    move2.planner(home=home,poslist=poslist)
    
    move2.run(100)
    print(move2.get_TCP_t())
    print(move2.get_TCP_r())

    # 基本使用方法3
    # 这里的home放空，可以对当前路径进行规划
   
    home=[]  #放空home，标示以当前路径作为启动路径
    poslist2=[[0.6,0.0,0.68,0,math.pi/2,0],
              [0.0,0.6,0.68,0,math.pi/2,math.pi/2],
              [-0.6,0.0,0.68,0,math.pi/2,math.pi],
              [-0.9,0.0,0.38,0,math.pi/2,math.pi],
              [0.0,-0.9,0.38,0,math.pi/2,3*math.pi/2],
              [0.9,0.0,0.38,0,math.pi/2,2*math.pi]]
    move2.planner(home=home,poslist=poslist2)
    move2.run(100)  #100表示运行完后在，停留100个周期

    # 避障2  添加添加圆形对象
    # 注销即可尝试
    '''
    move2=Rf_Move_Rfuniverse()
    home=[0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]

    poslist= [[0.33,0.1,0.48,0,math.pi,math.pi/2],
              [0.33,0.4,0.58,0,math.pi,math.pi/2],
              [0.30,-0.3,0.44,0,math.pi,math.pi/2],
              [0.33,-0.5,0.54,0,math.pi,math.pi/2],
              [0.25,0.5,0.54,0,math.pi,math.pi/2],
              [0.25,0.5,0.34,0,math.pi,math.pi/2],
              [-0.40,0.5,0.34,0,math.pi,math.pi/2]]
    
    box_1=ShapeObject(shape="Sphere",
                        id=1,
                        scale=[0.06],
                        pose=[0.0,0.3,0.2,0,0,0])

    box_2=ShapeObject(shape="Sphere",
                      id=2,
                      scale=[0.10],
                      pose=[0.0,0.5,0.4,0,0,0])

    box_3=ShapeObject(shape="Sphere",
                      id=3,
                      scale=[0.06],
                      pose=[0.4,0.0,0.4,0,0,0])

    box_4=ShapeObject(shape="Sphere",
                      id=4,
                      scale=[0.1],
                      pose=[0.4,0.0,0.9,0,0,0])
    
    box_5=ShapeObject(shape="Sphere",
                      id=5,
                      scale=[0.1],
                      pose=[-0.0,-0.4,0.6,0,0,0])
    
    box_6=ShapeObject(shape="Sphere",
                      id=6,
                      scale=[0.1],
                      pose=[0.2,-0.4,0.3,0,0,0])
    
    box_7=ShapeObject(shape="Sphere",
                      id=7,
                      scale=[0.1],
                      pose=[0.1,-0.4,0.8,0,0,0])
    
    box_8=ShapeObject(shape="Sphere",
                      id=8,
                      scale=[0.10],
                      pose=[0.0,0.5,0.8,0,0,0])

    move2.AddObject(box_1)
    move2.AddObject(box_2)
    move2.AddObject(box_3)
    move2.AddObject(box_4)
    move2.AddObject(box_5)
    move2.AddObject(box_6)
    move2.AddObject(box_7)
    move2.AddObject(box_8)
    

    
    move2.planner(poslist=poslist)
    move2.run(500)
    print(move2.get_TCP_t())
    print(move2.get_TCP_r())
    '''

    # 避障碍3 Box对象案例
    # 注销即可尝试
    '''
    move2=Rf_Move_Rfuniverse()
    home=[0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]

    poslist= [[0.33,0.1,0.48,0,math.pi,math.pi/2],
              [0.33,0.4,0.58,0,math.pi,math.pi/2],
              [0.30,-0.3,0.44,0,math.pi,math.pi/2],
              [0.33,-0.5,0.54,0,math.pi,math.pi/2],
              [0.25,0.5,0.54,0,math.pi,math.pi/2],
              [0.25,0.5,0.34,0,math.pi,math.pi/2],
              [-0.40,0.5,0.34,0,math.pi,math.pi/2]]
    
    box1=ShapeObject(shape="Box",
                     id=1,
                     scale=[0.06,0.06,0.4],
                     pose=[0.0,0.3,0.2,0,0,0])
    box2=ShapeObject(shape="Box",
                     id=2,
                     scale=[1,0.06,0.6],    #[长，高，宽]
                     pose=[0.6,0,0.9,7*math.pi/12,math.pi/12,0])
    box3=ShapeObject(shape="Box",
                     id=3,
                     scale=[0.3,0.06,0.4],
                     pose=[0.4,0.0,0.4,math.pi/2,0,0])
    box4=ShapeObject(shape="Box",
                     id=4,
                     scale=[0.06,0.06,0.6],
                     pose=[-0.2,0.5,0.4,math.pi/2,math.pi/6,0])
    
    move2.AddObject(box1)
    move2.AddObject(box2)
    move2.AddObject(box3)
    move2.AddObject(box4)
    
    move2.planner(poslist=poslist)
    move2.run(1000000000)
    print(move2.get_TCP_t())
    print(move2.get_TCP_r())
    '''

    # 计算link7和unity之间的误差矩阵
    # 用于计算unity和link7之间的补偿举证，已经计算好了，可以不管以下内容
    '''
    move=Rf_Move_Rfuniverse()  # 初始化对象
    
    # 取消原来的误差矩阵
    #delatH=np.array([[1, 0,  0,  0],
    #                 [0, 1,  0,  0],
    #                 [0, 0,  1,  0],
    #                 [0, 0,  0,  1]])
    
    #使用默认路径，用于测试
    #move.setDelta(DeltaH=delatH) 
    home=[0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]
    poslist=[[0.33,0,0.58,math.pi/3,math.pi/3,-math.pi/3]]  #x,y,z,wx,wy,wz
    move.planner(home=home,poslist=poslist)

    #运行规划
    move.run(100)
    #输出执行结果
    print(move.get_TCP_t())
    print(move.get_TCP_r())

    #运行程序，获取误差矩阵End->Link7，
    H=move.computeEnd2Link7(
                          [0,0,0],
                          [0.33,0,0.58],
                          [0.0032342442567790558, -0.0036667915520783635, 0.791228688550986],
                          [0.329399436712265, -0.0022427323274314404, 0.7949707508087158])
    print(H)  #最后的补偿矩阵
    '''
    
    # 基本使用方法1
    # 使用默认路径
    '''
    move1=Rf_Move_Rfuniverse()
    move1.planner()
    move1.run(10)
    print(move1.get_TCP_t())
    print(move1.get_TCP_r())
    '''
    
    # 基本使用方法2
    # 直接对link7进行规划
    '''
    move2=Rf_Move_Rfuniverse()
    delatH=np.array([[1, 0,  0,  0],
                     [0, 1,  0,  0],
                     [0, 0,  1,  0],
                     [0, 0,  0,  1]])
    move2.setDelta(DeltaH=delatH)  #重新设置与link7之间的误差矩阵，一般不需要使用
    home=[0.0, -0.785398163397448279, 0.0, -2.356194490192344837, 0.0, 1.570796326794896558, 0.785398163397448279]  #制定启动的home状态，第一次使用必须制定
    poslist=[[0.33,0,0.58,0,math.pi,0],  #测试路径
             [0.33,0.1,0.68,0,math.pi/3,0],
             [0.55,0.0,0.48,0,math.pi/2,0]]
    move2.planner(home=home,poslist=poslist)
    print(move2.get_last_planner_waypoint())  #答应最后路径点
    move2.run(100)
    print(move2.get_TCP_t())
    print(move2.get_TCP_r())
    '''
   
    
    

    
    
    
    

    
   
    
    





                 









