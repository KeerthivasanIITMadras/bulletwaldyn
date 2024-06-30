import pybullet as p
import time
import pybullet_data
# from collections import defaultdict
import numpy as np
import copy
from collections import OrderedDict


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
                self.robotId, joint, controlMode=self.p.TORQUE_CONTROL, force=10.0)

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
        p.setRealTimeSimulation(False)
        for i in range(1):
            angles = self.p.calculateInverseKinematics(
                self.robotId, (leg_index+1)*5, ee)
            p.stepSimulation()
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


def getContact(robotID, planeID):
    contactproperties = p.getContactPoints(robotID, planeID)
    # 9,10,12
    forces = OrderedDict()
    for contact in contactproperties:
        if contact[3] == 5:
            forces["LF_FOOT"] = [contact[9], contact[10], contact[12]]
        if contact[3] == 10:
            forces["RF_FOOT"] = [contact[9], contact[10], contact[12]]
        if contact[3] == 15:
            forces["LH_FOOT"] = [contact[9], contact[10], contact[12]]
        if contact[3] == 20:
            forces["RH_FOOT"] = [contact[9], contact[10], contact[12]]
    return forces


def get_sensor_data(robot, joint_id, link_id):
    sensor_data = OrderedDict()
    base_pos, base_quat = p.getBasePositionAndOrientation(robot)
    sensor_data['base_pos'] = np.asarray(base_pos)
    sensor_data['base_quat'] = np.asarray(base_quat)
    base_lin_vel, base_ang_vel = p.getBaseVelocity(robot)
    sensor_data['base_lin_vel'] = np.asarray(base_lin_vel)
    sensor_data['base_ang_vel'] = np.asarray(base_ang_vel)
    sensor_data['joint_pos'] = OrderedDict()
    sensor_data['joint_vel'] = OrderedDict()
    for k, v in joint_id.items():
        js = p.getJointState(robot, v)
        sensor_data['joint_pos'][k] = js[0]
        sensor_data['joint_vel'][k] = js[1]
    return sensor_data


def get_jacobian(robot, link_idx, sensor_data):
    link_state = p.getLinkState(robot,
                                link_idx,
                                computeLinkVelocity=1,
                                computeForwardKinematics=1)
    jpos = list(sensor_data['joint_pos'].values())
    jvel = list(sensor_data['joint_vel'].values())
    zeros = [0.] * len(jvel)
    jac = p.calculateJacobian(robot, link_idx, link_state[2], jpos, zeros,
                              zeros)
    nv = len(jac[0][0])
    ret = np.zeros((6, nv))
    for row in range(3):
        for col in range(nv):
            ret[row, col] = jac[1][row][col]  # angular
            ret[row + 3, col] = jac[0][row][col]  # linear
    return ret


def get_robot_config(robot):
    nq, nv, na, joint_id, link_id = 0, 0, 0, OrderedDict(), OrderedDict()
    link_id[(p.getBodyInfo(robot)[0]).decode("utf-8")] = -1
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        if info[2] != p.JOINT_FIXED:
            joint_id[info[1].decode("utf-8")] = info[0]
        link_id[info[12].decode("utf-8")] = info[0]
        nq = max(nq, info[3])
        nv = max(nv, info[4])
    nq += 1
    nv += 1
    na = len(joint_id)
    return nq, nv, na, joint_id, link_id


if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)
    # physicsClient = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
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
    while True:
        print(robot.calculateDynamicMatrices()[2])
        p.stepSimulation()
        time.sleep(1/240.0)
    # while True:
    #     forces = getContact(robotId, planeID)
    #     tau_contact = np.zeros(12)
    #     nq, nv, na, joint_id, link_id = get_robot_config(robotId)
    #     i = 0
    #     while i <= 9:
    #         # angle_pair = q[i:i+3]
    #         # print(angle_pair)
    #         sensor_data = get_sensor_data(robotId, joint_id, link_id)
    #         LF_J = np.array(get_jacobian(
    #             robotId, link_id['LF_FOOT'], sensor_data))[:, 6:9]
    #         RF_J = np.array(get_jacobian(
    #             robotId, link_id['RF_FOOT'], sensor_data))[:, 9:12]
    #         LH_J = np.array(get_jacobian(
    #             robotId, link_id['LH_FOOT'], sensor_data))[:, 12:15]
    #         RH_J = np.array(get_jacobian(
    #             robotId, link_id['RH_FOOT'], sensor_data))[:, 15:18]
    #         J = np.concatenate((LF_J, np.concatenate(
    #             (RF_J, np.concatenate((LH_J, RH_J), axis=1)), axis=1)), axis=1)
    #         # angle_pair = [angle_pair[0]]+[0]+angle_pair[1:3]
    #         # J = np.array(kin.robot.jacob0(angle_pair))
    #         F = [0, 0, -70]
    #         LF = RF = LH = RH = F
    #         J_T = np.transpose(J)
    #         if i == 0:
    #             if forces.get('LF_FOOT') == None:
    #                 LF = [0, 0, 0]
    #             tau_contact_4 = np.dot(J_T, np.array(LF+[0, 0, 0]))
    #             tau_contact[0:3] = tau_contact_4[0:3]
    #         if i == 3:
    #             if forces.get('RF_FOOT') == None:
    #                 RF = [0, 0, 0]
    #             tau_contact_4 = np.dot(J_T, np.array(RF+[0, 0, 0]))
    #             tau_contact[3:6] = tau_contact_4[3:6]
    #         if i == 6:
    #             if forces.get('LH_FOOT') == None:
    #                 LH = [0, 0, 0]
    #             tau_contact_4 = np.dot(J_T, np.array(LH+[0, 0, 0]))
    #             tau_contact[6:9] = tau_contact_4[6:9]
    #         if i == 9:
    #             if forces.get('RH_FOOT') == None:
    #                 RH = [0, 0, 0]
    #             tau_contact_4 = np.dot(J_T, np.array(RH+[0, 0, 0]))
    #             tau_contact[9:12] = tau_contact_4[9:12]
    #         i += 3
    #     prev_q, prev_qd = robot.getPositionVelocity(jointIds)
        
    #     kp = 37
    #     kd = 2*np.sqrt(kp)
    #     ref_q = [0, 0, 0]*4
    #     ref_qd = [0.0, 0.0, 0.0]*4
    #     robot.disableVelocityControl(jointIds)
    #     robot.enableTorqueControl(jointIds)
    #     accel_req = kp*(np.array(ref_q)-np.array(prev_q)) + \
    #         kd*(np.array(ref_qd)-np.array(prev_qd))
    #     tau_inv = p.calculateInverseDynamics(
    #         robotId, list(robotpositon)+list(robotOrientation)+list(prev_q), [0]*7+prev_qd, [0]*7+list(accel_req), flags=1)
    #     tau = np.array(tau_inv[7:])-np.array(tau_contact)
    #     for i in range(20):
    #         p.setJointMotorControlArray(
    #             robotId, robot.getcontrollablejointIds(), controlMode=p.TORQUE_CONTROL, forces=list(tau))
    #         p.stepSimulation()
        
    #     time.sleep(1/240.0)

p.disconnect()
