import sys

import numpy

sys.path.append("../install/lib")
import moveit_noros as moveit
import numpy as np
import matplotlib.pyplot as plt

print("==== Implement Motion Plan Tutorials. ====")
print("== Load robot model ==")
robot_loader = moveit.createRobotModelLoaderFromFile("../resources/panda_arm_hand_urdf_convert.urdf", "../resources/panda_arm_hand.srdf")
kinematic_solver_loader = moveit.createKinematicsLoaderFromFile("../resources/kinematics.yaml")
robot_loader.loadKinematicsSolvers(kinematic_solver_loader)
kinematic_model = robot_loader.getModel()
print("Model Frame: %s" % kinematic_model.getModelFrame())

print("== Load planner configuration ==")
pconfig = moveit.createPlannerConfigurationFromFile("../resources/ompl_planning.yaml")

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
#print("Positions:")
#for waypoint in response.trajectory.waypoints:
#    print(waypoint.getJointPosition("panda_joint1"))
#print("Velocity:")
#for waypoint in response.trajectory.waypoints:
#    print(waypoint.getJointVelocity("panda_joint1"))
#print("Durations:")
#print(response.trajectory.durations)
#print(sum(response.trajectory.durations))

print("== Create spline trajectory ==")
spline_trajectory = moveit.computeSpline(response.trajectory)
#print("Duration")
#print(spline_trajectory.duration)

print("== Sample spline trajectory ==")
sample = spline_trajectory.sample_by_interval("panda_joint1", 0.01)
#print(sample.position)

fig = plt.figure(num = 1, figsize=(16, 9), dpi = 120)
origin_position = fig.add_subplot(2, 2, 1)
sample_position = fig.add_subplot(2, 2, 2)
origin_velocity = fig.add_subplot(2,2,3)
sample_velocity = fig.add_subplot(2,2,4)
origin_position.set_title("Origin Trajectory Position")
origin_position.set_xlabel("time")
origin_position.set_ylabel("position")
sample_position.set_title("Sampled Trajectory Position (1000Hz)")
sample_position.set_xlabel("time")
sample_position.set_ylabel("position")
origin_velocity.set_title("Origin Trajectory Velocity")
origin_velocity.set_xlabel("time")
origin_velocity.set_ylabel("velocity")
sample_velocity.set_title("Sampled Trajectory Velocity (1000Hz)")
sample_velocity.set_xlabel("time")
sample_velocity.set_ylabel("velocity")

# plot origin position
y_origin_position = []
for waypoint in response.trajectory.waypoints:
    y_origin_position.append(waypoint.getJointPosition("panda_joint1"))
time_stamp_origin = []
current_time_stamp_origin = 0
for duration in response.trajectory.durations:
    time_stamp_origin.append(duration + current_time_stamp_origin)
    current_time_stamp_origin+=duration
plot1 = origin_position.plot(time_stamp_origin, y_origin_position, linestyle='-', marker='o', markersize = 3)

# plot sampled position
time_stamp_sample = np.linspace(spline_trajectory.start_time, spline_trajectory.end_time, sample.position.size)
plot2 = sample_position.plot(time_stamp_sample, sample.position)

# plot origin velocity
y_origin_velocity = []
for waypoint in response.trajectory.waypoints:
    y_origin_velocity.append(waypoint.getJointVelocity("panda_joint1"))
plot3 = origin_velocity.plot(time_stamp_origin, y_origin_velocity, linestyle='-', marker='o', markersize = 3, color='orange')

# plot sampled velocity
time_stamp_sample = np.linspace(spline_trajectory.start_time, spline_trajectory.end_time, sample.velocity.size)
plot4 = sample_velocity.plot(time_stamp_sample, sample.velocity, color = 'orange')

# get tip transforms
tip_transforms = spline_trajectory.tip_transforms
#for tip in tip_transforms:
    #print(tip)

plt.show()
