import sys
sys.path.append("../install/lib")
import numpy
import moveit_noros as moveit

print("==== Implement Planning Scene Tutorial From Moveit Tutorials. ====")
print("== Load robot model ==")
robot_loader = moveit.createRobotModelLoaderFromFile("../resources/panda_arm_hand.urdf", "../resources/panda_arm_hand.srdf")
kinematic_solver_loader = moveit.createKinematicsLoaderFromFile("../resources/kinematics.yaml")
robot_loader.loadKinematicsSolvers(kinematic_solver_loader)
print("model count after loader initialize %d" % robot_loader.countModelUse())
kinematic_model = robot_loader.getModel()
print("model count after model fetch %d" % robot_loader.countModelUse())
print("Model Frame: %s" % kinematic_model.getModelFrame())

planning_scene = robot_loader.newPlanningScene()

print("== Collision Checking ==")
collision_request = moveit.CollisionRequest()
collision_result = moveit.CollisionResult()
planning_scene.checkSelfCollision(collision_request, collision_result)
if collision_result.collision:
    print("Test 1: Current state is in self collision")
else:
    print("Test 1: Current state is NOT in self collision")

print("== Change The State ==")
current_state = planning_scene.getCurrentStateNonConst()
current_state.setToRandomPositions()
collision_result.clear()
planning_scene.checkSelfCollision(collision_request, collision_result)
if collision_result.collision:
    print("Test 2: Current state is in self collision" )
else:
    print("Test 2: Current state is NOT in self collision" )

print("== Checking for a group ==")
collision_request.group_name = "hand"
current_state.setToRandomPositions()
collision_result.clear()
planning_scene.checkSelfCollision(collision_request, collision_result)
if collision_result.collision:
    print("Test 3: Current state is in self collision" )
else:
    print("Test 3: Current state is NOT in self collision" )

print("== Getting Contact Information ==")
joint_values = [0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0] # Must collision
joint_model_group = kinematic_model.getJointModelGroup("panda_arm")
current_state.setJointGroupPositions(joint_model_group, joint_values)
if current_state.satisfiesBounds(joint_model_group):
    print("Test 4: Current state is valid")
else:
    print("Test 4: Current state is NOT valid")

collision_request.contacts = True
collision_request.max_contacts = 1000
collision_result.clear()
planning_scene.checkSelfCollision(collision_request, collision_result)
if collision_result.collision:
    print("Test 5: Current state is in self collision" )
else:
    print("Test 5: Current state is NOT in self collision" )

contactMap = collision_result.contacts
print(contactMap)

print("model count before model deletion %d" % robot_loader.countModelUse())
del kinematic_model
print("model count after model deletion %d" % robot_loader.countModelUse())