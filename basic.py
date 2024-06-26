import pybullet as p
import time
import pybullet_data
# from collections import defaultdict
import numpy as np
import copy


class Robot:
    def __init__(self, p, robotId):
        self.p = p
        self.robotId = robotId
        self.time_step = 1/240.0

    def get_base_position(self):
        base_positon, base_orientation = p.getBasePositionAndOrientation(
            self.robotId)
        return base_positon, base_orientation

    def getcontrollablejointIds(self):
        jointTypeList = []
        numJoints = p.getNumJoints(self.robotId)
        for joint in range(numJoints):
            info = p.getJointInfo(robotId, joint)
            jointTypeList.append(info[2])

        jointIds = [j for j in range(
            numJoints) if jointTypeList[j] != p.JOINT_FIXED]
        return jointIds

    def getMassMatrix(self, q):
        # q should of len=12
        M = self.p.calculateMassMatrix(
            robotId, q, flags=self.p.URDF_USE_INERTIA_FROM_FILE)
        return M

    def getJointProperties(self, jointId):
        joint_state = self.p.getJointState(self.robotId, jointId)
        joint_state_prop = {}
        joint_state_prop["position"] = joint_state[0]
        joint_state_prop["velocity"] = joint_state[1]
        joint_state_prop["reactionforces"] = joint_state[2]
        joint_state_prop["lastmotortorque"] = joint_state[3]
        return joint_state_prop

    def disableVelocityControl(self, jointIds):
        maxForce = 0
        mode = self.p.VELOCITY_CONTROL
        for joint in jointIds:
            self.p.setJointMotorControl2(self.robotId, joint,
                                         controlMode=mode, force=maxForce)

    def enableTorqueControl(self, jointIds):
        for joint in jointIds:
            self.p.setJointMotorControl2(
                self.robotId, joint, controlMode=self.p.TORQUE_CONTROL, force=100.0)

    def enableTorqueSensor(self, jointIds):
        for joint in jointIds:
            self.p.enableJointForceTorqueSensor(
                self.robotId, joint, enableSensor=1)

    def getPositionVelocity(self, jointIds):
        position = []
        velocity = []
        for joint in jointIds:
            joint_state = self.getJointProperties(joint)
            position.append(joint_state["position"])
            velocity.append(joint_state["velocity"])
        return (position, velocity)

    def ik_leg(self, leg_index, ee):
        angles = []
        angles = self.p.calculateInverseKinematics(
            self.robotId, (leg_index+1)*5, ee)
        if (leg_index == 0):
            return (angles[0:3])
        elif (leg_index == 1):
            return (angles[3:6])
        elif (leg_index == 2):
            return (angles[6:9])
        elif (leg_index == 3):
            return (angles[9:12])
        return angles

    def getJointStates(self):
        joint_states = self.p.getJointStates(
            self.robotId, self.getcontrollablejointIds())
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def calculateDynamicMatrices(self):
        joint_pos, joint_vel, _ = self.getJointStates()
        n_dof = len(self.getcontrollablejointIds())
        InertiaMatrix = np.asarray(
            p.calculateMassMatrix(self.robotId, joint_pos))
        GravityMatrix = np.asarray(p.calculateInverseDynamics(
            self.robotId, [0]*6+[1]+joint_pos, [0.0] * (n_dof+7), [0.0] * (n_dof+7), flags=1))
        CoriolisMatrix = np.asarray(p.calculateInverseDynamics(
            self.robotId, [0]*6+[1]+joint_pos, [0.0]*7+joint_vel, [0.0] * (n_dof+7), flags=1)) - GravityMatrix
        return InertiaMatrix, GravityMatrix, CoriolisMatrix

    # def getTrajectory(self, thi, thf, tf, dt):
    #     desired_position, desired_velocity, desired_acceleration = [], [], []
    #     t = 0
    #     while t <= tf:
    #         # th = thi+((thf-thi)/tf)*(t-(tf/(2*np.pi))*np.sin((2*np.pi/tf)*t))
    #         th = [0]*12
    #         dth = [0]*12
    #         ddth = [0]*12
    #         desired_position.append(th)
    #         desired_velocity.append(dth)
    #         desired_acceleration.append(ddth)
    #         t += dt
    #     desired_position = np.array(desired_position)
    #     desired_velocity = np.array(desired_velocity)
    #     desired_acceleration = np.array(desired_acceleration)
    #     return desired_position, desired_velocity, desired_acceleration

    # def doInverseDynamics(self, th_initial, th_final, final_time=2):
    #     self.p.setRealTimeSimulation(False)
    #     q_d, dq_d, ddq_d = self.getTrajectory(
    #         th_initial, th_final, tf=final_time, dt=self.time_step)
    #     traj_points = q_d.shape[0]
    #     print('#Trajectory points:', traj_points)
    #     # kd = 0.7
    #     n = 0
    #     while n < traj_points:
    #         tau = self.p.calculateInverseDynamics(
    #             self.robotId, [0]*6+[1]+list(q_d[n]), [0]*7+list(dq_d[n]), [0]*7+list(ddq_d[n]), flags=1)
    #         # tau += kd * dq_d[n] #if joint damping is turned off, this torque will not be required
    #         # print(tau)
    #         tau = tau[7:]
    #         # torque control
    #         p.setJointMotorControlArray(self.robotId, self.getcontrollablejointIds(),
    #                                     controlMode=self.p.TORQUE_CONTROL,
    #                                     forces=tau)
    #         theta, _, _ = self.getJointStates()
    #         print('n:{}::th:{}'.format(n, theta))

    #         p.stepSimulation()
    #         time.sleep(self.time_step)
    #         n += 1
    #     print('Desired joint angles:', th_final)
    #     # self.p.disconnect()


if __name__ == "__main__":
    # physicsClient = p.connect(p.GUI)
    physicsClient = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    # p.setPhysicsEngineParameter(fixedTimeStep=0.005,
    #                             numSolverIterations=50,
    #                             erp=0.2,
    #                             contactERP=0.2,
    #                             frictionERP=0.2)

    hz = 240.0
    dt = 1.0 / hz
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    robotpositon = [0, 0, 0.52]
    robotOrientation = p.getQuaternionFromEuler([0, 0, 0])
    planeID = p.loadURDF("plane.urdf")
    p.changeDynamics(bodyUniqueId=planeID,
                     linkIndex=-1,
                     lateralFriction=1.0)
    robotId = p.loadURDF(
        "/home/keerthivasan/catkin_quad/src/anymal_b_simple_description/urdf/anymal.urdf", robotpositon, robotOrientation, useFixedBase=False, flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_IMPLICIT_CYLINDER | p.URDF_USE_INERTIA_FROM_FILE)
    robot = Robot(p, robotId)
    jointIds = robot.getcontrollablejointIds()
    robot.disableVelocityControl(jointIds)
    robot.enableTorqueControl(jointIds)

    prev_qd = [0.0]*12
    time_count = 0
    applied_torque = None
    kp = 260
    kd = 10
    ref_q = [0, 0.3, -0.3]*2+[0, -0.3, 0.3]*2
    ref_qd = [0.0, 0.0, 0.0]*4
    num_joints = p.getNumJoints(robotId)
    end_effector_links = []

    # Iterate through all the joints to find end-effectors
    for joint_index in range(num_joints):
        joint_info = p.getJointInfo(robotId, joint_index)
        link_name = joint_info[12].decode('utf-8')  # Link name
        print(link_name)
        # Assuming end-effectors have specific naming or are the last link in each leg chain
        if 'foot' in link_name.lower() or 'end' in link_name.lower():
            end_effector_links.append(joint_index)
    print(end_effector_links)
    # while True:
    #     # robot.doInverseDynamics([0]*12, [0]*12)
    #     q, qd = robot.getPositionVelocity(jointIds)
    #     # tau = self.p.calculateInverseDynamics(
    #     #     self.robotId, [0]*6+[1]+list(q_d[n]), [0]*7+list(dq_d[n]), [0]*7+list(ddq_d[n]), flags=1)
    #     tau = -kp*(np.array(q)-np.array(ref_q))-kd*(np.array(qd)-np.array(ref_qd))
    #     # applied_torque = None
    #     # print(robot.ik_leg(1, [0.1, 0.1, 0]))
    #     p.setJointMotorControlArray(
    #         robotId, jointIds, controlMode=p.TORQUE_CONTROL, forces=tau)
    #     # print(robot.calculateDynamicMatrices())
    #     p.stepSimulation()
    #     time.sleep(1/240.0)

p.disconnect()
