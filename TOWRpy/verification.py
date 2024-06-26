import roboticstoolbox as rtb
import numpy as np
import pybullet as p
import pybullet_data
# Load your robot model
robot = rtb.models.DH.Puma560()

# Define a joint configuration
q = [0, 0, 0, 0, 0, 0]

# Calculate the end-effector position using the Robotics Toolbox
T_rt = robot.fkine(q)
end_effector_pos_rt = T_rt.t

# Initialize PyBullet and load the same robot
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
anymal = p.loadURDF("anymal.urdf", useFixedBase=True)

# Set the robot's joint configuration in PyBullet
for i, joint_angle in enumerate(q):
    p.resetJointState(anymal, i, joint_angle)

# Calculate the end-effector position using PyBullet
end_effector_state_pb = p.getLinkState(anymal, end_effector_link_index)
end_effector_pos_pb = end_effector_state_pb[4]

print("End-effector position (Robotics Toolbox):", end_effector_pos_rt)
print("End-effector position (PyBullet):", end_effector_pos_pb)

# Compare positions
assert np.allclose(end_effector_pos_rt, end_effector_pos_pb), "End-effector positions do not match!"
