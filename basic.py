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
        M = p.calculateMassMatrix(
            robotId, q, flags=self.p.URDF_USE_INERTIA_FROM_FILE)
        return M

    def getJointProperties(self, jointId):
        joint_state = p.getJointState(self.robotId, jointId)
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

    def computeGravityCompensationControl(self, jointIds):
        curr_pos, _ = self.getPositionVelocity(jointIds)
        grav_comp_torque = self.p.calculateInverseDynamics(self.robotId, [0]*6+[1]+curr_pos,
                                                           [0] * 19,
                                                           [0] * 19, flags=1)
        return grav_comp_torque

    def computeRBDInertiaMatrixAndNonLinearEffects(self, joint_pos, joint_vel):
        rbd_inertia_matrix = self.p.calculateMassMatrix(
            self.robotId, list(joint_pos))
        rbd_non_linear_effects = self.p.calculateInverseDynamics(
            self.robotId, joint_pos, joint_vel, [0.0] * 19)
        return np.asarray(rbd_inertia_matrix), np.asarray(rbd_non_linear_effects)

    def computeForwardDynamics(self, joint_pos, joint_vel, torque, is_using_damping=False):
        [rbd_inertia_matrix, rbd_non_linear_effects
         ] = self.computeRBDInertiaMatrixAndNonLinearEffects(joint_pos, joint_vel)

        temp = torque - rbd_non_linear_effects

        if is_using_damping:
            damping_const = 0.5  # from URDF
            temp -= damping_const * np.array(joint_vel)
        qdd = list(np.matmul(np.linalg.inv(
            rbd_inertia_matrix), (temp).reshape(19)))
        return qdd


if __name__ == "__main__":
    physicsClient = p.connect(p.GUI)
    # physicsClient = p.connect(p.DIRECT)
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
    joint_config = [
        0.03, 0.4, -0.8, 0,   # LF
        -0.03, 0.4, -0.8, 0,  # RF
        0.03, -0.4, 0.8, 0,   # LH
        -0.03, -0.4, 0.8, 0,  # RH
        0
    ]
    # print(robot.getJacobian())
    # mass matrix is 18x18 shape
    prev_qd = [0.0]*12
    time_count = 0
    applied_torque = None
    while True:
        [q, qd] = robot.getPositionVelocity(jointIds)
        qdd = list((np.array(qd) - np.array(prev_qd))/dt)
        # torques = p.calculateInverseDynamics(
        #     robotId, [0]*6+[1]+joint_config, [0.0]*19, [0.0]*19, flags=1 | p.URDF_USE_INERTIA_FROM_FILE)
        # torques_req = torques[7:]
        print("time_count = ", time_count)
        print("q   = ", np.array(q))
        print("qd  = ", np.array(qd))
        print("qdd = ", np.array(qdd))
        print('')
        bullet_tau = np.array(p.calculateInverseDynamics(
            robotId, [0]*6+[1] + q, [0]*7 + qd, [0]*7+qdd, flags=1))

        grav_comp_torque = robot.computeGravityCompensationControl(jointIds)
        applied_torque = -np.array(
            grav_comp_torque[7:])+np.array(bullet_tau[7:])
        # print(torques_req)
        # final_torques = []
        # for i in range(len(torques_prev)):
        #     final_torques.append(
        #         1.0*(torques_req[i]-torques_prev[i])+0.05*(torques_req[i]-torques_prev[i])/12)
        p.setJointMotorControlArray(
            robotId, jointIds, controlMode=p.TORQUE_CONTROL, forces=applied_torque)
        computed_qdd_wo_damping = robot.computeForwardDynamics(
            [0]*6+[1]+q, [0]*7 + qd, applied_torque, False)
        p.stepSimulation()
        time.sleep(1/240.0)
        prev_q = copy.deepcopy(q)
        prev_qd = copy.deepcopy(qd)
        time_count += 1

p.disconnect()
