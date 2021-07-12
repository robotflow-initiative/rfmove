import pybullet as p
import sys
sys.path.append("../install/lib")
import moveit_noros as moveit
import pybullet_data
import time

print("== Initialize Pybullet == ")
physicsClient = p.connect(p.GUI_SERVER)
#client2 = p.connect(p.SHARED_MEMORY)
#cppClient = moveit.Bullet3Hardware(physicsClient)


p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

p.setAdditionalSearchPath("../resources")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("pr2.urdf", startPos, startOrientation, useFixedBase=1)

print("== Connect throught cpp ==")

bulletController = moveit.PybulletHardware(p, boxId)

print("boxID %d" % boxId)
print("joint number from python %d" % p.getNumJoints(boxId))

print("body number at end %d" % p.getNumBodies())
print("body number at end from controller %d" % bulletController.getNumBodies())

bulletController.printJointInfo()
bulletController.printJointState()

#p.setJointMotorControl2(boxId, , p.POSITION_CONTROL, targetPosition = self.kinematic_state.getJointPosition(joint_name), maxVelocity = 3)
#cppClient.debugGravity()
while True:
    p.stepSimulation()
    time.sleep(1./240.)
    #print("%f\r" % p.getJointState(boxId, joint_index)[0]),
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos, cubeOrn)