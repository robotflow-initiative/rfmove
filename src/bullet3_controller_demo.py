import pybullet as p
import sys
sys.path.append("../install/lib")
import moveit_noros as moveit
import pybullet_data
import time
import numpy as np
import threading

def getTransform(bulletHardware, planning_scene, link_name):
    moveit.syncBulletState(bulletHardware, planning_scene)
    return planning_scene.getCurrentStateNonConst().getGlobalLinkTransform(link_name)

print("== Initialize Pybullet == ")
physicsClient = p.connect(p.GUI_SERVER)
#client2 = p.connect(p.SHARED_MEMORY)
#cppClient = moveit.Bullet3Hardware(physicsClient)
p.setRealTimeSimulation(1)

p.setGravity(0,0,-10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

p.setAdditionalSearchPath("../resources")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("pr2.urdf", startPos, startOrientation, useFixedBase=1)

print("== Initialize moveit ==")
robot_loader = moveit.createRobotModelLoaderFromFile("../resources/pr2.urdf", "../resources/pr2_config/pr2.srdf")
kinematic_solver_loader = moveit.createKinematicsLoaderFromFile("../resources/pr2_config/kinematics.yaml")
robot_loader.loadKinematicsSolvers(kinematic_solver_loader)
robot_model = robot_loader.getModel()

pconfig = moveit.createPlannerConfigurationFromFile("../resources/pr2_config/ompl_planning.yaml")
planner = moveit.PlannerManager(robot_model, pconfig)

bulletHardware = moveit.PybulletHardware(p, boxId)

planning_scene = robot_loader.newPlanningScene()
PLANNING_GROUP = "left_arm"
joint_model_group = robot_model.getJointModelGroup(PLANNING_GROUP)
tip_link = joint_model_group.getEndEffectorTips()[0]
base_frame = robot_model.getModelFrame()
end_transform = getTransform(bulletHardware, planning_scene, tip_link)

req = moveit.MotionPlanRequest(PLANNING_GROUP)
pose = moveit.PoseStamped(base_frame, end_transform.translation, end_transform.quaternion.value)
pose_goal = moveit.constructGoalConstraints(tip_link, pose, 0.01, 0.01)
req.addGoal(pose_goal)
context = planner.getPlanningContext(planning_scene, req)
response = context.solve()
response.trajectory.computeTimeStamps()