# import pybullet as p
# import os
# import time
# import numpy as np
# import pybullet_data
# import time
# from ctypes import *
# import roboticstoolbox as rtb
# from roboticstoolbox import DHRobot, RevoluteDH
# import math
# import matplotlib.pyplot as plt
# lib = CDLL("build/libtowr_anymal_dll.so")

# file_path = "./rsc/anymal/"


# class Kinematics:
#     def __init__(self):
#         self.L1 = 0
#         self.L2 = 0.25
#         self.L3 = 0.32125

#         self.robot = DHRobot([RevoluteDH(a=self.L1), RevoluteDH(offset=math.pi/2, alpha=math.pi /
#                                                                 2), RevoluteDH(a=self.L2), RevoluteDH(a=self.L3)])

#     def check(self):
#         print(self.robot)


# class Trajectory_data2(Structure):
#     _fields_ = [
#         ('base_linear', c_float*3),
#         ('base_angular', c_float*3),
#         ('ee_linear', c_float*12),
#         ('ee_force', c_float*12)

#     ]


# def print_traj2(a, no_of_samples):
#     time = 0.00
#     for i in range(no_of_samples+1):
#         print("\n\ni :", i, " time :", i*2.0/no_of_samples)
#         print("base_linear:", a[i].base_linear[0], "\t",
#               a[i].base_linear[1], "\t", a[i].base_linear[2])
#         print("base_angular:", a[i].base_angular[0], "\t",
#               a[i].base_angular[1], "\t", a[i].base_angular[2])
#         for j in range(4):
#             print("leg_no:", j)
#             print("\tee_linear_leg:", a[i].ee_linear[0+3*j], "\t",
#                   a[i].ee_linear[1+3*j], "\t", a[i].ee_linear[2+3*j])
#             # print("\tJoint_angles:",towr_joint_angles[i][0+3*j:3+3*j])
#             print("\tee_force_leg:", a[i].ee_force[0+3*j], "\t",
#                   a[i].ee_force[1+3*j], "\t", a[i].ee_force[2+3*j])
#             # print("\tJoint_Torques:",towr_joint_torques[i][0+3*j:3+3*j])


# class Controller:
#     def __init__(self, robotId):
#         self.robotId = robotId
#         self.P = 480
#         self.D = 5

#     def getcontrollablejointIds(self):
#         jointTypeList = []
#         numJoints = p.getNumJoints(self.robotId)
#         for joint in range(numJoints):
#             info = p.getJointInfo(self.robotId, joint)
#             jointTypeList.append(info[2])

#         jointIds = [j for j in range(
#             numJoints) if jointTypeList[j] != p.JOINT_FIXED]
#         return jointIds

#     def enableTorqueControl(self, jointIds):
#         for joint in jointIds:
#             p.setJointMotorControl2(
#                 self.robotId, joint, controlMode=p.TORQUE_CONTROL, force=100.0)

#     def disableVelocityControl(self, jointIds):
#         maxForce = 0
#         mode = p.VELOCITY_CONTROL
#         for joint in jointIds:
#             p.setJointMotorControl2(self.robotId, joint,
#                                     controlMode=mode, force=maxForce)

#     def getJointProperties(self, jointId):
#         joint_state = p.getJointState(self.robotId, jointId)
#         joint_state_prop = {}
#         joint_state_prop["position"] = joint_state[0]
#         joint_state_prop["velocity"] = joint_state[1]
#         joint_state_prop["reactionforces"] = joint_state[2]
#         joint_state_prop["lastmotortorque"] = joint_state[3]
#         return joint_state_prop

#     def getPositionVelocity(self, jointIds):
#         position = []
#         velocity = []
#         for joint in jointIds:
#             joint_state = self.getJointProperties(joint)
#             position.append(joint_state["position"])
#             velocity.append(joint_state["velocity"])
#         return (position, velocity)

#     def forcecontrol(self, ref_q, ref_qd):
#         q, qd = self.getPositionVelocity(self.getcontrollablejointIds())
#         tau = -self.P*(np.array(q)-np.array(ref_q)) -\
#             self.D*(np.array(qd)-np.array(ref_qd))
#         return tau


# def cal_towr_code(init_pos, init_ang, target_pos, target_angle, ee1, ee2, ee3, ee4, terrain_id, gait_pattern):
#     target = float_array_3()
#     base_i_p = float_array_3()
#     base_i_a = float_array_3()
#     target_a = float_array_3()
#     ee_1 = float_array_3()
#     ee_2 = float_array_3()
#     ee_3 = float_array_3()
#     ee_4 = float_array_3()
#     for i in range(3):
#         target[i] = target_pos[i]
#         base_i_p[i] = init_pos[i]
#         base_i_a[i] = init_ang[i]
#         target_a[i] = target_angle[i]
#         ee_1[i] = ee1[i]
#         ee_2[i] = ee2[i]
#         ee_3[i] = ee3[i]
#         ee_4[i] = ee4[i]
#     arr_size = no_of_samples+1
#     lib.Trajectory.restype = None
#     lib.Trajectory.argtypes = [Trajectory_data2*(arr_size), c_float*3, c_float*3,
#                                c_float*3, c_float*3,
#                                c_float*3, c_float*3,
#                                c_float*3, c_float*3, c_int, c_int, c_int]
#     print("\nTarget:")
#     for i in range(3):
#         print(target[i])

#     traj_array = Trajectory_data2*(arr_size)
#     Result_traj = traj_array()

#     # for i in range(no_of_samples):
#     # print("i+1 :",i+1," 2/(i+i) :",2.0/(i+1)," (i)*2.0/(i+1) :",(i)*2.0/(i+1))
#     lib.Trajectory(Result_traj, base_i_p, base_i_a,
#                    ee_1, ee_2, ee_3, ee_4,
#                    target, target_a, no_of_samples, terrain_id, gait_pattern)
#     # print_traj2(Result_traj,arr_size,no_of_samples)
#     return (Result_traj)


# def set_anymal(anymal, base_pos, base_quat, ee1, ee2, ee3, ee4, control):
#     angles = []
#     tau = None
#     for i in range(1):
#         p.resetBasePositionAndOrientation(anymal, base_pos, base_quat)
#         leg_LF = p.calculateInverseKinematics(
#             anymal, 5, ee1)  # eight_vertx_LF[j])
#         leg_RF = p.calculateInverseKinematics(
#             anymal, 10, ee2)  # eight_vertx_RF[j])
#         leg_LH = p.calculateInverseKinematics(
#             anymal, 15, ee3)  # eight_vertx_LH[j])
#         leg_RH = p.calculateInverseKinematics(
#             anymal, 20, ee4)  # eight_vertx_RH[j])
#         q = leg_LF[0:3]+leg_RF[3:6]+leg_LH[6:9]+leg_RH[9:12]
#         qd = [0.0]*12
#         tau = list(control.forcecontrol(q, qd))
#         print(q)
#         # p.setJointMotorControlArray(bodyUniqueId=anymal,
#         #                             jointIndices=[1, 2, 3],
#         #                             controlMode=p.POSITION_CONTROL,
#         #                             targetPositions=leg_LF[0:3],
#         #                             )
#         # p.setJointMotorControlArray(bodyUniqueId=anymal,
#         #                             jointIndices=[6, 7, 8],
#         #                             controlMode=p.POSITION_CONTROL,
#         #                             targetPositions=leg_RF[3:6],
#         #                             )
#         # p.setJointMotorControlArray(bodyUniqueId=anymal,
#         #                             jointIndices=[11, 12, 13],
#         #                             controlMode=p.POSITION_CONTROL,
#         #                             targetPositions=leg_LH[6:9],
#         #                             )
#         # p.setJointMotorControlArray(bodyUniqueId=anymal,
#         #                             jointIndices=[16, 17, 18],
#         #                             controlMode=p.POSITION_CONTROL,
#         #                             targetPositions=leg_RH[9:12],
#         #                             )
#         # p.stepSimulation()
#         angles = leg_LF[0:3]+leg_RF[3:6]+leg_LH[6:9]+leg_RH[9:12]
#     # p.setJointMotorControlArray(
#     #     anymal, control.getcontrollablejointIds(), controlMode=p.TORQUE_CONTROL, forces=tau)
#     return angles


# def draw_frame(point, draw_target=False):
#     if draw_target:
#         p.addUserDebugLine(
#             point, [point[0]+0.2, point[1], point[2]], [0, 1, 0], 5)
#         p.addUserDebugLine(
#             point, [point[0], point[1]+0.2, point[2]], [0, 1, 0], 5)
#         p.addUserDebugLine(
#             point, [point[0], point[1], point[2]+0.2], [0, 1, 0], 5)
#     else:
#         p.addUserDebugLine(
#             point, [point[0]+0.1, point[1], point[2]], [1, 0, 0], 5)
#         p.addUserDebugLine(
#             point, [point[0], point[1]+0.1, point[2]], [0, 1, 0], 5)
#         p.addUserDebugLine(
#             point, [point[0], point[1], point[2]+0.1], [0, 0, 1], 5)


# def ik_leg(anymal, leg_index, ee):
#     angles = []
#     for i in range(20):
#         angles = p.calculateInverseKinematics(anymal, (leg_index+1)*5, ee)
#         p.stepSimulation()
#     if (leg_index == 0):
#         return (angles[0:3])
#     elif (leg_index == 1):
#         return (angles[3:6])
#     elif (leg_index == 2):
#         return (angles[6:9])
#     elif (leg_index == 3):
#         return (angles[9:12])


# no_of_samples = 100

# horizon = no_of_samples + 1
# float_array_3 = c_float*3
# init_base_pos = [0, 0, 0.54]  # 0.53 for stairs
# init_base_quat = [0, 0, 0, 1]

# target_pos = [1.8, 0, 0.54]
# target_angle = [0, 0, 0]

# kp = 5
# kd = 50
# log = 0
# coeff_fric = 0.85
# '''
#   0                  FlatID,
#   2                  StairsID,
# '''
# terrain_id = 0
# '''
# overlap-walk -0
# fly trot - 1
# pace - 2
# bound - 3
# gallop - 4
# '''


# def run_towr_tracking_position_control(target_pos, target_angle, terrain_id, gait_pattern):
#     current_base_pos = init_base_pos
#     current_base_ang = p.getEulerFromQuaternion(init_base_quat)
#     target_anymal = [target_pos[0], target_pos[1]+2, target_pos[2]]
#     p.connect(p.GUI)
#     # p.connect(p.DIRECT)

#     p.resetSimulation()
#     plane = p.loadURDF(os.path.join(
#         pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)  # 5,10,15,20
#     p.changeDynamics(plane, -1, lateralFriction=coeff_fric)
#     if terrain_id == 2:
#         boxHalfLength = 0.2
#         boxHalfWidth = 2.5
#         boxHalfHeight = 0.1
#         sh_colBox = p.createCollisionShape(
#             p.GEOM_BOX, halfExtents=[boxHalfLength, boxHalfWidth, 0])
#         sh_final_col = p.createCollisionShape(
#             p.GEOM_BOX, halfExtents=[0.5, boxHalfWidth, 0])
#         boxOrigin = 1+boxHalfLength
#         step1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sh_colBox,
#                                   basePosition=[boxOrigin, 1, boxHalfHeight],
#                                   baseOrientation=[0.0, 0.0, 0.0, 1])

#         step2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sh_final_col,
#                                   basePosition=[
#                                       boxOrigin+0.5+boxHalfLength, 1, 0.05 + 2*boxHalfHeight],
#                                   baseOrientation=[0.0, 0.0, 0.0, 1])
#         p.changeDynamics(step1, -1, lateralFriction=coeff_fric)
#         p.changeDynamics(step2, -1, lateralFriction=coeff_fric)

#     draw_frame(target_pos, draw_target=True)
#     draw_frame(target_anymal, draw_target=True)
#     p.setGravity(0, 0, -10)

#     anymal = p.loadURDF(file_path+"anymal.urdf", [0, 2, 0.5],
#                         flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE)
#     vis_anymal = p.loadURDF(file_path+"anymal_vis.urdf", [
#         0, 0, 0.5], flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE)
#     kin = Kinematics()
#     control = Controller(anymal)
#     control2 = Controller(vis_anymal)
#     maxForce = 0
#     mode = p.VELOCITY_CONTROL
#     for joint in control.getcontrollablejointIds():
#         p.setJointMotorControl2(anymal, joint,
#                                 controlMode=mode, force=maxForce)
#     for joint in control2.getcontrollablejointIds():
#         p.setJointMotorControl2(vis_anymal, joint,
#                                 controlMode=mode, force=maxForce)

#     for joint in control.getcontrollablejointIds():
#         p.setJointMotorControl2(
#             anymal, joint, controlMode=p.TORQUE_CONTROL, force=100.0)
#     for joint in control2.getcontrollablejointIds():
#         p.setJointMotorControl2(
#             vis_anymal, joint, controlMode=p.TORQUE_CONTROL, force=100.0)

#     set_anymal(vis_anymal, init_base_pos, init_base_quat, [init_base_pos[0]+0.34, init_base_pos[1]+0.19, init_base_pos[2]-0.42],
#                [init_base_pos[0]+0.34, init_base_pos[1]-0.19, init_base_pos[2]-0.42],
#                [init_base_pos[0]-0.34, init_base_pos[1]+0.19, init_base_pos[2]-0.42],
#                [init_base_pos[0]-0.34, init_base_pos[1]-0.19, init_base_pos[2]-0.42], control2)

#     set_anymal(anymal, [init_base_pos[0], init_base_pos[1]+2, init_base_pos[2]], init_base_quat,
#                [init_base_pos[0]+0.34, init_base_pos[1] +
#                    2+0.19, init_base_pos[2]-0.42],
#                [init_base_pos[0]+0.34, init_base_pos[1] +
#                    2-0.19, init_base_pos[2]-0.42],
#                [init_base_pos[0]-0.34, init_base_pos[1] +
#                    2+0.19, init_base_pos[2]-0.42],
#                [init_base_pos[0]-0.34, init_base_pos[1]+2-0.19, init_base_pos[2]-0.42], control)
#     if (save_log):
#         log = p.startStateLogging(
#             loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=log_name)
#     reached_or_out_or_fallen = False

#     while (reached_or_out_or_fallen != True):

#         # takes the current ee positions of anymal, to compute from that position
#         ee1 = list(p.getLinkState(anymal, 5)[0])
#         ee2 = list(p.getLinkState(anymal, 10)[0])
#         ee3 = list(p.getLinkState(anymal, 15)[0])
#         ee4 = list(p.getLinkState(anymal, 20)[0])

#         ee1[1] = ee1[1] - 2
#         ee2[1] = ee2[1] - 2
#         ee3[1] = ee3[1] - 2
#         ee4[1] = ee4[1] - 2

#         # print("ee1:",ee1,"ee2:",ee2,"ee3:",ee3,"ee4:",ee4)
#         draw_frame(current_base_pos)
#         Result_traj = cal_towr_code(init_pos=current_base_pos, init_ang=current_base_ang,
#                                     target_pos=target_pos, target_angle=target_angle,
#                                     ee1=ee1, ee2=ee2, ee3=ee3, ee4=ee4,
#                                     terrain_id=terrain_id, gait_pattern=gait_pattern)

#         for towr_sample_index in range(horizon):
#             towr_quat = p.getQuaternionFromEuler(
#                 Result_traj[towr_sample_index].base_angular)
#             p.resetBasePositionAndOrientation(
#                 vis_anymal, Result_traj[towr_sample_index].base_linear, towr_quat)

#             a = p.getBasePositionAndOrientation(vis_anymal)[0]
#             if towr_sample_index != 0:
#                 p.addUserDebugLine(b, a, [1, 0, 0], 3)
#             angles = set_anymal(vis_anymal, Result_traj[towr_sample_index].base_linear, towr_quat,
#                                 Result_traj[towr_sample_index].ee_linear[0:3],
#                                 Result_traj[towr_sample_index].ee_linear[3:6],
#                                 Result_traj[towr_sample_index].ee_linear[6:9],
#                                 Result_traj[towr_sample_index].ee_linear[9:12], control2)
#             base_linear = Result_traj[towr_sample_index].base_linear
#             base_angular = p.getQuaternionFromEuler(
#                 Result_traj[towr_sample_index].base_angular)
#             b = a
#             LF = Result_traj[towr_sample_index].ee_force[0:3]
#             RF = Result_traj[towr_sample_index].ee_force[3:6]
#             LH = Result_traj[towr_sample_index].ee_force[6:9]
#             RH = Result_traj[towr_sample_index].ee_force[9:12]
#             current_base_anymal = p.getBasePositionAndOrientation(anymal)
#             base, orient = p.getBasePositionAndOrientation(anymal)
#             tau_inv = p.calculateInverseDynamics(
#                 anymal, list(base)+list(orient)+list(LF)+list(RF)+list(LH)+list(RH), list(base_linear)+list(base_angular)+[0.0]*12, [0]*7+[0.0]*12, flags=1)

#             # total_forces = np.array(LF+RF+LH+RH).T
#             '''ee 5,10,15,20'''
#             # position, velocity = control.getPositionVelocity(
#             #     control.getcontrollablejointIds())
#             # print(len(velocity))
#             # link_state = p.getLinkState(anymal,
#             #                             5,
#             #                             computeLinkVelocity=1,
#             #                             computeForwardKinematics=1)
#             # jact, jacr = p.calculateJacobian(
#             #     anymal, 5, link_state[2], angles, velocity, [0]*12)
#             # print(np.array(jacr).shape)

#             q = list(angles)
#             qd = [0.0]*12
#             tau = list(control.forcecontrol(q, qd))
#             i = 0
#             tau_contact = np.zeros(12)
#             while i <= 9:
#                 angle_pair = q[i:i+3]
#                 # print(angle_pair)
#                 angle_pair = [angle_pair[0]]+[0]+angle_pair[1:3]
#                 J = np.array(kin.robot.jacob0(angle_pair))
#                 J_T = np.transpose(J)
#                 if i == 0:
#                     tau_contact_4 = np.dot(J_T, np.array(LF+[0, 0, 0]))
#                     tau_contact[0:3] = [tau_contact_4[0],
#                                         tau_contact_4[2], tau_contact_4[3]]
#                 if i == 3:
#                     tau_contact_4 = np.dot(J_T, np.array(RF+[0, 0, 0]))
#                     tau_contact[3:6] = [tau_contact_4[0],
#                                         tau_contact_4[2], tau_contact_4[3]]
#                 if i == 6:
#                     tau_contact_4 = np.dot(J_T, np.array(LH+[0, 0, 0]))
#                     tau_contact[6:9] = [tau_contact_4[0],
#                                         tau_contact_4[2], tau_contact_4[3]]
#                 if i == 9:
#                     tau_contact_4 = np.dot(J_T, np.array(RH+[0, 0, 0]))
#                     tau_contact[6:9] = [tau_contact_4[0],
#                                         tau_contact_4[2], tau_contact_4[3]]
#                 i += 3
#             tau = list(np.array(tau_inv[7:])+np.array(tau)-tau_contact)
#             p.setJointMotorControlArray(
#                 anymal, control.getcontrollablejointIds(), controlMode=p.TORQUE_CONTROL, forces=tau)
#             p.stepSimulation()
#             # time.sleep(0.01)

#             # p.setJointMotorControlArray(
#             #     anymal, control.getcontrollablejointIds(), controlMode=p.TORQUE_CONTROL, forces=tau)
#             # for i in range(4):

#             #     leg_state = p.getJointStates(anymal, [1+5*i, 2+5*i, 3+5*i])

#             #     previous_angles = [leg_state[0][0],
#             #                        leg_state[1][0],
#             #                        leg_state[2][0]]
#             #     previous_velocities = [leg_state[0][1],
#             #                            leg_state[1][1],
#             #                            leg_state[2][1]]

#             #     # print("angles:",previous_angles,"vel:",previous_velocities)
#             #     if (i == 0):
#             #         dezired_leg_angles = angles[0:3]
#             #     elif (i == 1):
#             #         dezired_leg_angles = angles[3:6]
#             #     elif (i == 2):
#             #         dezired_leg_angles = angles[6:9]
#             #     elif (i == 3):
#             #         dezired_leg_angles = angles[9:12]

#             #     p.setJointMotorControlArray(bodyUniqueId=anymal,
#             #                                 jointIndices=[1+5*i, 2+5*i, 3+5*i],
#             #                                 controlMode=p.POSITION_CONTROL,
#             #                                 targetPositions=dezired_leg_angles,
#             #                                 targetVelocities=[0, 0, 0],)

#         if ((abs(target_anymal[0]) < abs(current_base_anymal[0][0])
#                 and abs(target_anymal[1]) < abs(current_base_anymal[0][1]))
#                 or current_base_anymal[0][2] < 0.3):
#             reached_or_out_or_fallen = True
#             # current_base_pos = [2,current_base_anymal[1]-2,current_base_anymal[2]]
#         else:
#             current_base_pos = [current_base_anymal[0][0],
#                                 current_base_anymal[0][1]-2, current_base_anymal[0][2]]
#             current_base_ang = p.getEulerFromQuaternion(current_base_anymal[1])


# gait = 1
# log_name = "./media/testFile.mp4"
# save_log = True
# if __name__ == "__main__":
#     # for i in range(no_of_gaits):
#     # print("Gait_type:",i)

#     run_towr_tracking_position_control(
#         target_pos, target_angle, terrain_id=terrain_id, gait_pattern=gait)
#     if (save_log):
#         p.stopStateLogging(log)
#     # kin = Kinematics()
#     # kin.check()
#     # 2nd joint should always be 0
#     # print(kin.robot.fkine([3.14, 0, 0, 0]))
#     # kin.robot.teach([0,0,0,0])
#     # print(kin.robot.jacob0())

import pybullet as p
import os
import time
import numpy as np
import pybullet_data
import time
from ctypes import *
# import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
import math
import matplotlib.pyplot as plt
import ctypes
from collections import OrderedDict

lib = CDLL("build/libtowr_anymal_dll.so", mode=ctypes.RTLD_GLOBAL)

file_path = "/home/keerthivasan/catkin_quad/src/pybullet_sim/TOWRpy/rsc/anymal/"


# class Kinematics:
#     def __init__(self):
#         self.L1 = 0
#         self.L2 = 0.25
#         self.L3 = 0.32125

#         self.robot = DHRobot([RevoluteDH(a=self.L1), RevoluteDH(offset=math.pi/2, alpha=math.pi /
#                                                                 2), RevoluteDH(a=self.L2), RevoluteDH(a=self.L3)])

#     def check(self):
#         print(self.robot)


class Trajectory_data2(Structure):
    _fields_ = [
        ('base_linear', c_float*3),
        ('base_angular', c_float*3),
        ('ee_linear', c_float*12),
        ('ee_force', c_float*12)

    ]


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


def print_traj2(a, no_of_samples):
    time = 0.00
    for i in range(no_of_samples+1):
        print("\n\ni :", i, " time :", i*2.0/no_of_samples)
        print("base_linear:", a[i].base_linear[0], "\t",
              a[i].base_linear[1], "\t", a[i].base_linear[2])
        print("base_angular:", a[i].base_angular[0], "\t",
              a[i].base_angular[1], "\t", a[i].base_angular[2])
        for j in range(4):
            print("leg_no:", j)
            print("\tee_linear_leg:", a[i].ee_linear[0+3*j], "\t",
                  a[i].ee_linear[1+3*j], "\t", a[i].ee_linear[2+3*j])
            # print("\tJoint_angles:",towr_joint_angles[i][0+3*j:3+3*j])
            print("\tee_force_leg:", a[i].ee_force[0+3*j], "\t",
                  a[i].ee_force[1+3*j], "\t", a[i].ee_force[2+3*j])
            # print("\tJoint_Torques:",towr_joint_torques[i][0+3*j:3+3*j])


class Controller:
    def __init__(self, robotId):
        self.robotId = robotId
        self.P = 280
        self.D = 22

    def getcontrollablejointIds(self):
        jointTypeList = []
        numJoints = p.getNumJoints(self.robotId)
        for joint in range(numJoints):
            info = p.getJointInfo(self.robotId, joint)
            jointTypeList.append(info[2])

        jointIds = [j for j in range(
            numJoints) if jointTypeList[j] != p.JOINT_FIXED]
        return jointIds

    def enableTorqueControl(self, jointIds):
        for joint in jointIds:
            p.setJointMotorControl2(
                self.robotId, joint, controlMode=p.TORQUE_CONTROL, force=100.0)

    def disableVelocityControl(self, jointIds):
        maxForce = 0
        mode = p.VELOCITY_CONTROL
        for joint in jointIds:
            p.setJointMotorControl2(self.robotId, joint,
                                    controlMode=mode, force=maxForce)

    def getJointProperties(self, jointId):
        joint_state = p.getJointState(self.robotId, jointId)
        joint_state_prop = {}
        joint_state_prop["position"] = joint_state[0]
        joint_state_prop["velocity"] = joint_state[1]
        joint_state_prop["reactionforces"] = joint_state[2]
        joint_state_prop["lastmotortorque"] = joint_state[3]
        return joint_state_prop

    def getPositionVelocity(self, jointIds):
        position = []
        velocity = []
        for joint in jointIds:
            joint_state = self.getJointProperties(joint)
            position.append(joint_state["position"])
            velocity.append(joint_state["velocity"])
        return (position, velocity)

    def forcecontrol(self, ref_q, ref_qd):
        q, qd = self.getPositionVelocity(self.getcontrollablejointIds())
        tau = -self.P*(np.array(q)-np.array(ref_q)) -\
            self.D*(np.array(qd)-np.array(ref_qd))
        return tau

    def getJointStates(self):
        joint_states = p.getJointStates(
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


def cal_towr_code(init_pos, init_ang, target_pos, target_angle, ee1, ee2, ee3, ee4, terrain_id, gait_pattern):
    target = float_array_3()
    base_i_p = float_array_3()
    base_i_a = float_array_3()
    target_a = float_array_3()
    ee_1 = float_array_3()
    ee_2 = float_array_3()
    ee_3 = float_array_3()
    ee_4 = float_array_3()
    for i in range(3):
        target[i] = target_pos[i]
        base_i_p[i] = init_pos[i]
        base_i_a[i] = init_ang[i]
        target_a[i] = target_angle[i]
        ee_1[i] = ee1[i]
        ee_2[i] = ee2[i]
        ee_3[i] = ee3[i]
        ee_4[i] = ee4[i]
    arr_size = no_of_samples+1
    lib.Trajectory.restype = None
    lib.Trajectory.argtypes = [Trajectory_data2*(arr_size), c_float*3, c_float*3,
                               c_float*3, c_float*3,
                               c_float*3, c_float*3,
                               c_float*3, c_float*3, c_int, c_int, c_int]
    print("\nTarget:")
    for i in range(3):
        print(target[i])

    traj_array = Trajectory_data2*(arr_size)
    Result_traj = traj_array()

    # for i in range(no_of_samples):
    # print("i+1 :",i+1," 2/(i+i) :",2.0/(i+1)," (i)*2.0/(i+1) :",(i)*2.0/(i+1))
    lib.Trajectory(Result_traj, base_i_p, base_i_a,
                   ee_1, ee_2, ee_3, ee_4,
                   target, target_a, no_of_samples, terrain_id, gait_pattern)
    # print_traj2(Result_traj,arr_size,no_of_samples)
    return (Result_traj)


def set_anymal(anymal, base_pos, base_quat, ee1, ee2, ee3, ee4, control):
    angles = []
    tau = None
    for i in range(15):
        p.resetBasePositionAndOrientation(anymal, base_pos, base_quat)
        leg_LF = p.calculateInverseKinematics(
            anymal, 5, ee1)  # eight_vertx_LF[j])
        leg_RF = p.calculateInverseKinematics(
            anymal, 10, ee2)  # eight_vertx_RF[j])
        leg_LH = p.calculateInverseKinematics(
            anymal, 15, ee3)  # eight_vertx_LH[j])
        leg_RH = p.calculateInverseKinematics(
            anymal, 20, ee4)  # eight_vertx_RH[j])
        q = leg_LF[0:3]+leg_RF[3:6]+leg_LH[6:9]+leg_RH[9:12]
        qd = [0.0]*12
        tau = list(control.forcecontrol(q, qd))
        # print(q)
        # p.setJointMotorControlArray(bodyUniqueId=anymal,
        #                             jointIndices=[1, 2, 3],
        #                             controlMode=p.POSITION_CONTROL,
        #                             targetPositions=leg_LF[0:3],
        #                             )
        # p.setJointMotorControlArray(bodyUniqueId=anymal,
        #                             jointIndices=[6, 7, 8],
        #                             controlMode=p.POSITION_CONTROL,
        #                             targetPositions=leg_RF[3:6],
        #                             )
        # p.setJointMotorControlArray(bodyUniqueId=anymal,
        #                             jointIndices=[11, 12, 13],
        #                             controlMode=p.POSITION_CONTROL,
        #                             targetPositions=leg_LH[6:9],
        #                             )
        # p.setJointMotorControlArray(bodyUniqueId=anymal,
        #                             jointIndices=[16, 17, 18],
        #                             controlMode=p.POSITION_CONTROL,
        #                             targetPositions=leg_RH[9:12],
        #                             )
        p.setJointMotorControlArray(
            anymal, control.getcontrollablejointIds(), controlMode=p.TORQUE_CONTROL, forces=tau)
        p.stepSimulation()

        angles = leg_LF[0:3]+leg_RF[3:6]+leg_LH[6:9]+leg_RH[9:12]

    return angles


def draw_frame(point, draw_target=False):
    if draw_target:
        p.addUserDebugLine(
            point, [point[0]+0.2, point[1], point[2]], [0, 1, 0], 5)
        p.addUserDebugLine(
            point, [point[0], point[1]+0.2, point[2]], [0, 1, 0], 5)
        p.addUserDebugLine(
            point, [point[0], point[1], point[2]+0.2], [0, 1, 0], 5)
    else:
        p.addUserDebugLine(
            point, [point[0]+0.1, point[1], point[2]], [1, 0, 0], 5)
        p.addUserDebugLine(
            point, [point[0], point[1]+0.1, point[2]], [0, 1, 0], 5)
        p.addUserDebugLine(
            point, [point[0], point[1], point[2]+0.1], [0, 0, 1], 5)


def ik_leg(anymal, leg_index, ee):
    angles = []
    for i in range(20):
        angles = p.calculateInverseKinematics(anymal, (leg_index+1)*5, ee)
        p.stepSimulation()
    if (leg_index == 0):
        return (angles[0:3])
    elif (leg_index == 1):
        return (angles[3:6])
    elif (leg_index == 2):
        return (angles[6:9])
    elif (leg_index == 3):
        return (angles[9:12])


no_of_samples = 120

horizon = no_of_samples + 1
float_array_3 = c_float*3
init_base_pos = [0, 0, 0.54]  # 0.53 for stairs
init_base_quat = [0, 0, 0, 1]

target_pos = [1.5, 0, 0.54]
target_angle = [0, 0, 0]

kp = 5
kd = 50
log = 0
coeff_fric = 0.85
'''
  0                  FlatID,
  2                  StairsID,
'''
terrain_id = 0
'''
overlap-walk -0
fly trot - 1
pace - 2
bound - 3
gallop - 4
'''


def run_towr_tracking_position_control(target_pos, target_angle, terrain_id, gait_pattern):
    current_base_pos = init_base_pos
    current_base_ang = p.getEulerFromQuaternion(init_base_quat)
    target_anymal = [target_pos[0], target_pos[1]+2, target_pos[2]]
    p.connect(p.GUI)
    # p.connect(p.DIRECT)

    p.resetSimulation()
    plane = p.loadURDF(os.path.join(
        pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)  # 5,10,15,20
    p.changeDynamics(plane, -1, lateralFriction=coeff_fric)
    if terrain_id == 2:
        boxHalfLength = 0.2
        boxHalfWidth = 2.5
        boxHalfHeight = 0.1
        sh_colBox = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[boxHalfLength, boxHalfWidth, 0])
        sh_final_col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.5, boxHalfWidth, 0])
        boxOrigin = 1+boxHalfLength
        step1 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sh_colBox,
                                  basePosition=[boxOrigin, 1, boxHalfHeight],
                                  baseOrientation=[0.0, 0.0, 0.0, 1])

        step2 = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=sh_final_col,
                                  basePosition=[
                                      boxOrigin+0.5+boxHalfLength, 1, 0.05 + 2*boxHalfHeight],
                                  baseOrientation=[0.0, 0.0, 0.0, 1])
        p.changeDynamics(step1, -1, lateralFriction=coeff_fric)
        p.changeDynamics(step2, -1, lateralFriction=coeff_fric)

    draw_frame(target_pos, draw_target=True)
    draw_frame(target_anymal, draw_target=True)
    p.setGravity(0, 0, -9.81)

    anymal = p.loadURDF(file_path+"anymal.urdf", [0, 2, 0.5],
                        flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE)
    vis_anymal = p.loadURDF(file_path+"anymal_vis.urdf", [
        0, 0, 0.5], flags=p.URDF_USE_SELF_COLLISION | p.URDF_USE_INERTIA_FROM_FILE)
    # kin = Kinematics()
    control = Controller(anymal)
    control2 = Controller(vis_anymal)
    maxForce = 0
    mode = p.VELOCITY_CONTROL
    for joint in control.getcontrollablejointIds():
        p.setJointMotorControl2(anymal, joint,
                                controlMode=mode, force=maxForce)
    for joint in control2.getcontrollablejointIds():
        p.setJointMotorControl2(vis_anymal, joint,
                                controlMode=mode, force=maxForce)

    for joint in control.getcontrollablejointIds():
        p.setJointMotorControl2(
            anymal, joint, controlMode=p.TORQUE_CONTROL, force=100.0)
    for joint in control2.getcontrollablejointIds():
        p.setJointMotorControl2(
            vis_anymal, joint, controlMode=p.TORQUE_CONTROL, force=100.0)

    set_anymal(vis_anymal, init_base_pos, init_base_quat, [init_base_pos[0]+0.34, init_base_pos[1]+0.19, init_base_pos[2]-0.42],
               [init_base_pos[0]+0.34, init_base_pos[1]-0.19, init_base_pos[2]-0.42],
               [init_base_pos[0]-0.34, init_base_pos[1]+0.19, init_base_pos[2]-0.42],
               [init_base_pos[0]-0.34, init_base_pos[1]-0.19, init_base_pos[2]-0.42], control2)

    set_anymal(anymal, [init_base_pos[0], init_base_pos[1]+2, init_base_pos[2]], init_base_quat,
               [init_base_pos[0]+0.34, init_base_pos[1] +
                   2+0.19, init_base_pos[2]-0.42],
               [init_base_pos[0]+0.34, init_base_pos[1] +
                   2-0.19, init_base_pos[2]-0.42],
               [init_base_pos[0]-0.34, init_base_pos[1] +
                   2+0.19, init_base_pos[2]-0.42],
               [init_base_pos[0]-0.34, init_base_pos[1]+2-0.19, init_base_pos[2]-0.42], control)
    if (save_log):
        log = p.startStateLogging(
            loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=log_name)
    reached_or_out_or_fallen = False

    while (reached_or_out_or_fallen != True):

        # takes the current ee positions of anymal, to compute from that position
        ee1 = list(p.getLinkState(anymal, 5)[0])
        ee2 = list(p.getLinkState(anymal, 10)[0])
        ee3 = list(p.getLinkState(anymal, 15)[0])
        ee4 = list(p.getLinkState(anymal, 20)[0])

        ee1[1] = ee1[1] - 2
        ee2[1] = ee2[1] - 2
        ee3[1] = ee3[1] - 2
        ee4[1] = ee4[1] - 2

        # print("ee1:",ee1,"ee2:",ee2,"ee3:",ee3,"ee4:",ee4)
        draw_frame(current_base_pos)
        Result_traj = cal_towr_code(init_pos=current_base_pos, init_ang=current_base_ang,
                                    target_pos=target_pos, target_angle=target_angle,
                                    ee1=ee1, ee2=ee2, ee3=ee3, ee4=ee4,
                                    terrain_id=terrain_id, gait_pattern=gait_pattern)
        nq, nv, na, joint_id, link_id = get_robot_config(anymal)
        counter = 0
        legsinswing = [False, False, False, False]
        for towr_sample_index in range(horizon):
            towr_quat = p.getQuaternionFromEuler(
                Result_traj[towr_sample_index].base_angular)
            p.resetBasePositionAndOrientation(
                vis_anymal, Result_traj[towr_sample_index].base_linear, towr_quat)

            a = p.getBasePositionAndOrientation(vis_anymal)[0]
            if towr_sample_index != 0:
                p.addUserDebugLine(b, a, [1, 0, 0], 3)
            angles = set_anymal(vis_anymal, Result_traj[towr_sample_index].base_linear, towr_quat,
                                Result_traj[towr_sample_index].ee_linear[0:3],
                                Result_traj[towr_sample_index].ee_linear[3:6],
                                Result_traj[towr_sample_index].ee_linear[6:9],
                                Result_traj[towr_sample_index].ee_linear[9:12], control2)
            base_linear = Result_traj[towr_sample_index].base_linear
            base_angular = p.getQuaternionFromEuler(
                Result_traj[towr_sample_index].base_angular)
            b = a
            for i in range(25):
                LF = Result_traj[towr_sample_index].ee_force[0:3]
                RF = Result_traj[towr_sample_index].ee_force[3:6]
                LH = Result_traj[towr_sample_index].ee_force[6:9]
                RH = Result_traj[towr_sample_index].ee_force[9:12]
                current_base_anymal = p.getBasePositionAndOrientation(anymal)
                base, orient = p.getBasePositionAndOrientation(anymal)
                Kp = 100
                Kd = 2*math.sqrt(Kp)
                angles_prev, angle_vel_prev = control.getPositionVelocity(
                    control.getcontrollablejointIds())
                error_q = np.array(angles)-np.array(angles_prev)
                error_qd = np.zeros(12)-np.array(angle_vel_prev)
                accel_req = Kp*error_q+Kd*error_qd
                tau_inv = p.calculateInverseDynamics(
                    anymal, list(base)+list(orient)+list(angles), list(base_linear)+list(base_angular)+angle_vel_prev, [4, 0, 0, 0.1, 0.1, 0.1, 1]+list(accel_req), flags=1)
                # p.stepSimulation()
                # total_forces = np.array(LF+RF+LH+RH).T
                '''ee 5,10,15,20'''
                # position, velocity = control.getPositionVelocity(
                #     control.getcontrollablejointIds())
                # print(len(velocity))
                # link_state = p.getLinkState(anymal,
                #                             5,
                #                             computeLinkVelocity=1,
                #                             computeForwardKinematics=1)
                # jact, jacr = p.calculateJacobian(
                #     anymal, 5, link_state[2], angles, velocity, [0]*12)
                # print(np.array(jacr).shape)

                # q = list(angles)
                # qd = [0.0]*12
                q = list(angles)
                qd = [0.0]*12
                tau = list(control.forcecontrol(q, qd))
                # tau = np.array(accel_req)
                # p.setJointMotorControlArray(
                #     anymal, control.getcontrollablejointIds(), controlMode=p.TORQUE_CONTROL, forces=list(tau))
                tau_contact = np.zeros(12)
                # p.stepSimulation()

                # print(forces)
                while i <= 9:
                    # angle_pair = q[i:i+3]
                    # print(angle_pair)
                    forces = getContact(anymal, plane)
                    sensor_data = get_sensor_data(anymal, joint_id, link_id)
                    LF_J = np.array(get_jacobian(
                        anymal, link_id['LF_FOOT'], sensor_data))[:, 6:9]
                    RF_J = np.array(get_jacobian(
                        anymal, link_id['RF_FOOT'], sensor_data))[:, 9:12]
                    LH_J = np.array(get_jacobian(
                        anymal, link_id['LH_FOOT'], sensor_data))[:, 12:15]
                    RH_J = np.array(get_jacobian(
                        anymal, link_id['RH_FOOT'], sensor_data))[:, 15:18]
                    J = np.concatenate((LF_J, np.concatenate(
                        (RF_J, np.concatenate((LH_J, RH_J), axis=1)), axis=1)), axis=1)
                    # angle_pair = [angle_pair[0]]+[0]+angle_pair[1:3]
                    # J = np.array(kin.robot.jacob0(angle_pair))
                    J_T = np.transpose(J)

                    if i == 0:
                        if forces.get('LF_FOOT') == None:
                            LF = [0, 0, 0]
                            legsinswing[0] = True
                        elif forces.get('LF_FOOT')[2] > 20:
                            legsinswing[0] = False
                        tau_contact_4 = np.dot(J_T, np.array(LF+[0, 0, 0]))
                        tau_contact[0:3] = tau_contact_4[0:3]
                    if i == 3:
                        if forces.get('RF_FOOT') == None:
                            RF = [0, 0, 0]
                            legsinswing[1] = True
                        elif forces.get('RF_FOOT')[2] > 20:
                            legsinswing[1] = False
                        tau_contact_4 = np.dot(J_T, np.array(RF+[0, 0, 0]))
                        tau_contact[3:6] = tau_contact_4[3:6]
                    if i == 6:
                        if forces.get('LH_FOOT') == None:
                            LH = [0, 0, 0]
                            legsinswing[2] = True
                        elif forces.get('LH_FOOT')[2] > 20:
                            legsinswing[2] = False
                        tau_contact_4 = np.dot(J_T, np.array(LH+[0, 0, 0]))
                        tau_contact[6:9] = tau_contact_4[6:9]
                    if i == 9:
                        if forces.get('RH_FOOT') == None:
                            RH = [0, 0, 0]
                            legsinswing[3] = True
                        elif forces.get('RH_FOOT')[2] > 20:
                            legsinswing[3] = False
                        tau_contact_4 = np.dot(J_T, np.array(RH+[0, 0, 0]))
                        tau_contact[9:12] = tau_contact_4[9:12]
                    i += 3
                # tau_contact = tau_contact[7:]
                # tau = list(np.array(tau_inv[7:])+np.array(tau)-tau_contact)
                tau_inv = np.array(tau_inv)
                tau_req = np.zeros(12)
                tau = np.array(tau)
                gravity = np.array(control.calculateDynamicMatrices()[2])[7:]
                if legsinswing[0] is True:
                    tau_req[0:3] = tau_inv[7:10]
                elif legsinswing[1] is True:
                    tau_req[3:6] = tau_inv[10:13]
                elif legsinswing[2] is True:
                    tau_req[6:9] = tau_inv[13:16]
                elif legsinswing[3] is True:
                    tau_req[9:12] = tau_inv[16:19]

                if legsinswing[0] is False:
                    tau_req[0:3] = 1*tau[0:3]+1 * \
                        tau_contact[0:3]+(-2)*gravity[0:3]
                elif legsinswing[1] is False:
                    tau_req[3:6] = 1*tau[3:6]+1 * \
                        tau_contact[3:6]+(-2)*gravity[3:6]
                elif legsinswing[2] is False:
                    tau_req[6:9] = 1*tau[6:9]+1 * \
                        tau_contact[6:9]+(-2)*gravity[6:9]
                elif legsinswing[3] is False:
                    tau_req[9:12] = 1*tau[9:12]+0 * \
                        tau_contact[9:12]+(-2)*gravity[9:12]
                # tau = list(tau+np.array(tau_inv[7:])+tau_contact)
                if counter > 20:
                    p.setJointMotorControlArray(
                        anymal, control.getcontrollablejointIds(), controlMode=p.TORQUE_CONTROL, forces=list(tau_req))
                else:
                    p.setJointMotorControlArray(
                        anymal, control.getcontrollablejointIds(), controlMode=p.TORQUE_CONTROL, forces=list(tau))
                p.stepSimulation()
            counter += 1
            time.sleep(1/240.0)

            # p.setJointMotorControlArray(tautau_inv[7:]tau_inv[7:]_inv[7:]tau_inv[7:]
            #     anymal, control.getcontrollablejointIds(), controlMode=p.TORQUE_CONTROL, forces=tau)
            # for i in range(4):

            #     leg_state = p.getJointStates(anymal, [1+5*i, 2+5*i, 3+5*i])

            #     previous_angles = [leg_state[0][0],
            #                        leg_state[1][0],
            #                        leg_state[2][0]]
            #     previous_velocities = [leg_state[0][1],
            #                            leg_state[1][1],
            #                            leg_state[2][1]]

            #     # print("angles:",previous_angles,"vel:",previous_velocities)
            #     if (i == 0):
            #         dezired_leg_angles = angles[0:3]
            #     elif (i == 1):
            #         dezired_leg_angles = angles[3:6]
            #     elif (i == 2):
            #         dezired_leg_angles = angles[6:9]
            #     elif (i == 3):
            #         dezired_leg_angles = angles[9:12]

            #     p.setJointMotorControlArray(bodyUniqueId=anymal,
            #                                 jointIndices=[1+5*i, 2+5*i, 3+5*i],
            #                                 controlMode=p.POSITION_CONTROL,
            #                                 targetPositions=dezired_leg_angles,
            #                                 targetVelocities=[0, 0, 0],)

        if ((abs(target_anymal[0]) < abs(current_base_anymal[0][0])
                 and abs(target_anymal[1]) < abs(current_base_anymal[0][1]))
                ):
            reached_or_out_or_fallen = True
            if abs(target_anymal[0]) - abs(current_base_anymal[0][0]) < 0.3:
                print("Reached")
            else:
                print("Not reached :: Failure Detected")
            # current_base_pos = [2,current_base_anymal[1]-2,current_base_anymal[2]]
        else:
            current_base_pos = [current_base_anymal[0][0],
                                current_base_anymal[0][1]-2, current_base_anymal[0][2]]
            current_base_ang = p.getEulerFromQuaternion(current_base_anymal[1])


gait = 1
log_name = "./media/testFile.mp4"
save_log = True
if __name__ == "__main__":
    # for i in range(no_of_gaits):
    # print("Gait_type:",i)

    run_towr_tracking_position_control(
        target_pos, target_angle, terrain_id=terrain_id, gait_pattern=gait)
    if (save_log):
        p.stopStateLogging(log)
    # kin = Kinematics()
    # kin.check()
    # 2nd joint should always be 0
    # print(kin.robot.fkine([3.14, 0, 0, 0]))
    # kin.robot.teach([0,0,0,0])
    # print(kin.robot.jacob0())
