import pybullet as p
import sys
sys.path.append("../install/lib")
import moveit_noros as moveit
import time
import pybullet_data
import threading
import numpy as np

def registerBulletBody(bodyId, loader):
    numJoints = p.getNumJoints(bodyId)
    for i in range(numJoints):
        loader.registerBtJointInfo(p.getJointInfo(bodyId, i))

def syncBulletState(bodyId, kinematic_state):
    for i in range(p.getNumJoints(bodyId)):
        kinematic_state.setJointPosition(p.getJointInfo(boxId, i)[1], p.getJointState(boxId, i)[0])

class controlThread(threading.Thread):
    def __init__(self, bodyId, jointIndex):
        threading.Thread.__init__(self)
        self.bodyId = bodyId
        self.jointIndex = jointIndex
    def run(self):
        p.setJointMotorControl2(self.bodyId, self.jointIndex, p.POSITION_CONTROL, targetPosition = 3.1415, maxVelocity = 3)

class randomPoseThread(threading.Thread):
    def __init__(self, bodyId, joint_model_group, loader):
        threading.Thread.__init__(self)
        self.bodyId = bodyId
        self.joint_group = joint_model_group
        self.loader = loader
        self.random_state = loader.newRobotState()
        self.kinematic_state = loader.newRobotState()
        self.tip_name = joint_model_group.getEndEffectorTips()[0]
        self.joint_names = joint_model_group.getJointModelNames()
    def run(self):
        while(True):
            self.random_state.setToRandomPositions(self.joint_group)
            self.random_state.enforceBounds(self.joint_group)
            end_effector_ = self.random_state.getGlobalLinkTransform(self.tip_name)
            found_ik = self.kinematic_state.setFromIK(self.joint_group, end_effector_)
            if found_ik:
                print("IK FOUND")
                for joint_name in self.joint_names:
                    joint_index = self.loader.getBtJointIndex(joint_name)
                    #print("Set joint %d to %f" % (joint_index, self.kinematic_state.getJointPosition(joint_name)))
                    p.setJointMotorControl2(self.bodyId, joint_index, p.POSITION_CONTROL, targetPosition = self.kinematic_state.getJointPosition(joint_name), maxVelocity = 3)
                time.sleep(3)
                syncBulletState(self.bodyId, self.kinematic_state)
            else:
                print("CAN NOT FIND IK")

print("== Initialize Pybullet == ")
physicsClient = p.connect(p.GUI)
p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

p.setAdditionalSearchPath("../resources")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("r300_robot.urdf", startPos, startOrientation, useFixedBase=1)
#for joint_index in range(p.getNumJoints(boxId)):
#    p.setJointMotorControl2(boxId, joint_index, p.VELOCITY_CONTROL, 0, 0)

print("== Initialize Moveit ==")
loader = moveit.createRobotModelLoaderFromFile("../resources/r300_robot.urdf", "../resources/r300_configuration/config/r300.srdf")
kinematic_solver_loader = moveit.createKinematicsLoaderFromFile("../resources/r300_configuration/config/kinematics.yaml")
loader.loadKinematicsSolvers(kinematic_solver_loader)

robot_model = loader.getModel()
groups = robot_model.JointModelGroups
PLANNING_GROUP = 'left_arm'

print("== Get Joint Infos ==")
registerBulletBody(boxId, loader)
#print("After register")
#loader.printJointMap()

print("== Sync Joint State ==")
kinematic_state = loader.newRobotState()
random_state = loader.newRobotState()
syncBulletState(boxId, kinematic_state)

print("== Joint Model Groups ==")
#for group in groups:
#    print("... %s" % group.getName())
joint_model_group = robot_model.getJointModelGroup(PLANNING_GROUP)
print("Tips of %s" % PLANNING_GROUP)
print(joint_model_group.getEndEffectorTips())

controler = randomPoseThread(boxId, joint_model_group, loader)
controler.start()

while True:
    p.stepSimulation()
    time.sleep(1./240.)
    #print("%f\r" % p.getJointState(boxId, joint_index)[0]),
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos, cubeOrn)
controler.join()
p.disconnect()