import pybullet as p
import sys
sys.path.append("../install/lib")
import moveit_noros as moveit
import pybullet_data
import time
import numpy as np

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

print("== Initialize moveit ==")
robot_loader = moveit.createRobotModelLoaderFromFile("../resources/pr2.urdf", "../resources/pr2_config/pr2.srdf")
kinematic_solver_loader = moveit.createKinematicsLoaderFromFile("../resources/pr2_config/kinematics.yaml")
robot_loader.loadKinematicsSolvers(kinematic_solver_loader)
robot_model = robot_loader.getModel()

bulletHardware = moveit.PybulletHardware(p, boxId)

print("== Load planner configuration ==")
pconfig = moveit.createPlannerConfigurationFromFile("../resources/pr2_config/ompl_planning.yaml")
print(pconfig)

print("== Create planner ==")
planner = moveit.PlannerManager(robot_model, pconfig)

print("== Create planning scene ==")
PLANNING_GROUP = "left_arm"
planning_scene = robot_loader.newPlanningScene()
#planning_scene.getCurrentStateNonConst().setToDefaultValues()
#moveit.syncBulletState(bulletHardware, planning_scene)



print("== Create pose goal ==")
req = moveit.MotionPlanRequest(PLANNING_GROUP)
pose = moveit.PoseStamped("l_shoulder_pan_link", np.array([0.3,0.4,0.75]), np.array([0,0,0,1]))
pose_goal = moveit.constructGoalConstraints("l_wrist_roll_link", pose, 0.01, 0.01)
req.addGoal(pose_goal)

print("== Create context ==")

context = planner.getPlanningContext(planning_scene, req)

print("== Solve ==")
#response = context.solve()
#response.trajectory.computeTimeStamps()





print("boxID %d" % boxId)
print("joint number from python %d" % p.getNumJoints(boxId))

print("body number at end %d" % p.getNumBodies())
print("body number at end from controller %d" % bulletHardware.getNumBodies())

bulletHardware.printJointInfo()
bulletHardware.printJointState()

for jmg in robot_model.getJointModelGroups():
    print(jmg.getName())
jmg = robot_model.getJointModelGroup("left_arm")
print(jmg.getJointModelNames()[0])
for tip in jmg.getEndEffectorTips():
    print(tip)
print(jmg.getLinkModelNames()[0])
print(jmg.getLinkModelNames()[-1])


testHandler = bulletHardware.getJointHandler("l_shoulder_lift_joint")
testHandler.setPosVel(-1, 1)

print(p.getJointStates(boxId, [0,1,2,3]))
#p.setJointMotorControl2(boxId, 65, p.POSITION_CONTROL, -1, 1)

#p.setJointMotorControl2(boxId, , p.POSITION_CONTROL, targetPosition = self.kinematic_state.getJointPosition(joint_name), maxVelocity = 3)
#cppClient.debugGravity()
while True:
    p.stepSimulation()
    time.sleep(1./240.)
    #print("%f\r" % p.getJointState(boxId, joint_index)[0]),
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos, cubeOrn)