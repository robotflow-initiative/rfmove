import sys
sys.path.append("../install/lib")
import moveit_noros as moveit
import numpy as np

print("==== Implement Motion Plan Tutorials. ====")
print("== Load robot model ==")
robot_loader = moveit.createRobotModelLoaderFromFile("../resources/panda_arm_hand.urdf", "../resources/panda_arm_hand.srdf")
kinematic_solver_loader = moveit.createKinematicsLoaderFromFile("../resources/kinematics.yaml")
robot_loader.loadKinematicsSolvers(kinematic_solver_loader)
kinematic_model = robot_loader.getModel()
print("Model Frame: %s" % kinematic_model.getModelFrame())

print("== Load planner configuration ==")
pconfig = moveit.createPlannerConfigurationFromFile("../resources/ompl_planning.yaml")
print(pconfig)

print("== Create planner ==")
planner = moveit.PlannerManager(kinematic_model, pconfig)

print("== Create planning scene ==")
PLANNING_GROUP = "panda_arm"
planning_scene = robot_loader.newPlanningScene()
planning_scene.getCurrentStateNonConst().setToDefaultValues()

print("== Create pose goal ==")
req = moveit.MotionPlanRequest(PLANNING_GROUP)
pose = moveit.PoseStamped("panda_link0", np.array([0.3,0.4,0.75]), np.array([0,0,0,1]))
pose_goal = moveit.constructGoalConstraints("panda_link8", pose, 0.01, 0.01)
req.addGoal(pose_goal)

print("== Create context ==")
# An error would occur because we haven't specified request start state
context = planner.getPlanningContext(planning_scene, req)

print("== Solve ==")
response = context.solve()
response.trajectory.computeTimeStamps()
print("Positions:")
for waypoint in response.trajectory.waypoints:
    print(waypoint.getJointPosition("panda_joint1"))
print("Velocity:")
for waypoint in response.trajectory.waypoints:
    print(waypoint.getJointVelocity("panda_joint1"))
print("Durations:")
print(response.trajectory.durations)
print(sum(response.trajectory.durations))

print("== Create spline trajectory ==")
spline_trajectory = moveit.SplineTrajectory(response.trajectory)
print("Duration")
print(spline_trajectory.duration)
