import sys
sys.path.append("../install/lib")
import numpy
import moveit_noros as moveit

print("==== Implement Robot Model and Robot State Tutorial From Moveit Tutorials. ====")
print("== Load robot model ==")
robot_loader = moveit.createRobotModelLoaderFromFile("../resources/franka_convert.urdf", "../resources/panda_arm_hand.srdf")
kinematic_solver_loader = moveit.createKinematicsLoaderFromFile("../resources/kinematics.yaml")
robot_loader.loadKinematicsSolvers(kinematic_solver_loader)
kinematic_model = robot_loader.getModel()
print("Model Frame: %s" % kinematic_model.getModelFrame())

PLANNING_GROUP = "panda_arm"
kinematic_state = robot_loader.newRobotState()
kinematic_state.setToDefaultValues()

joint_model_group = kinematic_model.getJointModelGroup(PLANNING_GROUP)
joint_names = joint_model_group.getVariableNames()
joint_values = kinematic_state.copyJointGroupPositions(joint_model_group)
print("== Current joint values ==")
for i in range(len(joint_names)):
    print("\t%s:\t%f" % (joint_names[i], joint_values[i]))

print("== Test Joint Limits ==")
print("Set joint value out of bound.")
joint_values=joint_values=[-1.3177302367227017, -1.7353981633974485, 0.8012309797768007, -2.295736269770167, 1.9069654787403831, 2.296614604292792, 2.5637651905206544]
kinematic_state.setJointGroupPositions(joint_model_group, joint_values)
if kinematic_state.satisfiesBounds():
    print("Current state is valid.")
else:
    print("Current state is invalid.")
print("Enforce robot state bounds.")
kinematic_state.enforceBounds()
if kinematic_state.satisfiesBounds():
    print("Current state is valid.")
else:
    print("Current state is invalid.")

print("== Forward Kinematics ==")
print("Set to random position")
#kinematic_state.setToRandomPositions(joint_model_group)
end_effector_state = kinematic_state.getGlobalLinkTransform("panda_link7")
print(end_effector_state)
#print("Translation:")
#print(end_effector_state.translation())
#print("Rotation:")
#print(end_effector_state.rotation())


print("== Inverse Kinematics ==")
found_ik = kinematic_state.setFromIK(joint_model_group, end_effector_state)
if found_ik:
    joint_values = kinematic_state.copyJointGroupPositions(joint_model_group)
    for i in range(len(joint_names)):
        print("\t%s:\t%f" % (joint_names[i], joint_values[i]))
else:
    print("Did not find IK solution")

print("== Get the Jacobian ==")
reference_point_position = numpy.array([0.0, 0.0, 0.0], dtype='double')
print(kinematic_state.getJacobian(joint_model_group, joint_model_group.getLinkModelNames()[-1], reference_point_position))
