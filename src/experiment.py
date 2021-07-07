import sys
sys.path.append("../install/lib")
import numpy
import moveit_noros as moveit

robot_loader = moveit.createRobotModelLoaderFromFile("../resources/panda_arm_hand.urdf", "../resources/panda_arm_hand.srdf")
robot_model = robot_loader.getModel()

print("Current bound for panda_joint1")
panda_joint1 = robot_model.getJointModel("panda_joint1")
for bound in panda_joint1.getVariableBounds():
    print(bound)

limit_loader = moveit.createJointLimitsLoaderFromFile("../resources/joint_limits.yaml")
robot_loader.loadJointLimits(limit_loader)

print("Bound after loading joint limit")
for bound in panda_joint1.getVariableBounds():
    print(bound)