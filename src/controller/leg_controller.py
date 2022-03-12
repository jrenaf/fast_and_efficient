import copy
import math
import numpy as np
from typing import Any, Mapping
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation
from src.robots.motors import MotorCommand
from src.robots.robot import Robot

import lp_python_interface


def lerp(a: float, b: float, t: float) -> float:
    return (a + t * (b - a))


def interpGRFArray(array_1: lp_python_interface.GRFArray,
                   array_2: lp_python_interface.GRFArray,
                   t_interp: float) -> lp_python_interface.GRFArray:
    interp_array = lp_python_interface.GRFArray

    # Interp grf array
    for i in range(array_1.feet.size()):
        interp_array.vectors[i].x = lerp(array_1.vectors[i].x,
                                         array_2.vectors[i].x, t_interp)
        interp_array.vectors[i].y = lerp(array_1.vectors[i].y,
                                         array_2.vectors[i].y, t_interp)
        interp_array.vectors[i].z = lerp(array_1.vectors[i].z,
                                         array_2.vectors[i].z, t_interp)
    return interp_array


def interpRobotState(state_1: lp_python_interface.RobotState,
                     state_2: lp_python_interface.RobotState,
                     t_interp: float) -> lp_python_interface.RobotState:
    # Interp individual elements, t_interp is the new in the new mpc-afterward timeframe
    interp_state = lp_python_interface.RobotState()
    interp_state.body = interpOdometry(state_1.body, state_2.body, t_interp)
    interp_state.joints = interpJointState(state_1.joints, state_2.joints,
                                           t_interp)
    interp_state.feet = interpMultiFootState(state_1.feet, state_2.feet,
                                             t_interp, interp_state.feet)
    return interp_state


def interpOdometry(state_1: lp_python_interface.BodyState,
                   state_2: lp_python_interface.BodyState,
                   t_interp: float) -> lp_python_interface.BodyState:
    interp_state = lp_python_interface.BodyState()

    # Interp body position
    interp_state.pose.position.x = lerp(state_1.pose.position.x,
                                        state_2.pose.position.x, t_interp)
    interp_state.pose.position.y = lerp(state_1.pose.position.y,
                                        state_2.pose.position.y, t_interp)
    interp_state.pose.position.z = lerp(state_1.pose.position.z,
                                        state_2.pose.position.z, t_interp)

    # Interp body orientation with slerp
    # tf2::Quaternion q_1, q_2, q_interp
    # tf2::convert(state_1.pose.orientation, q_1)
    # tf2::convert(state_2.pose.orientation, q_2)
    # q_interp = q_1.slerp(q_2, t_interp)
    # interp_state.pose.orientation = tf2::toMsg(q_interp)
    q_1 = [
        state_1.pose.orientation.x, state_1.pose.orientation.y,
        state_1.pose.orientation.z, state_1.pose.orientation.w
    ]
    q_2 = [
        state_2.pose.orientation.x, state_2.pose.orientation.y,
        state_2.pose.orientation.z, state_2.pose.orientation.w
    ]
    r_1 = Rotation.from_quat(q_1)
    r_2 = Rotation.from_quat(q_2)
    slerp = Slerp([1, 2], [r_1, r_2])
    r_interp = slerp(t_interp)
    q_interp = Rotation.as_quat(r_interp)
    interp_state.pose.orientation.x = q_interp[0]
    interp_state.pose.orientation.y = q_interp[1]
    interp_state.pose.orientation.z = q_interp[2]
    interp_state.pose.orientation.w = q_interp[3]

    # Interp twist
    interp_state.twist.linear.x = lerp(state_1.twist.linear.x,
                                       state_2.twist.linear.x, t_interp)
    interp_state.twist.linear.y = lerp(state_1.twist.linear.y,
                                       state_2.twist.linear.y, t_interp)
    interp_state.twist.linear.z = lerp(state_1.twist.linear.z,
                                       state_2.twist.linear.z, t_interp)

    interp_state.twist.angular.x = lerp(state_1.twist.angular.x,
                                        state_2.twist.angular.x, t_interp)
    interp_state.twist.angular.y = lerp(state_1.twist.angular.y,
                                        state_2.twist.angular.y, t_interp)
    interp_state.twist.angular.z = lerp(state_1.twist.angular.z,
                                        state_2.twist.angular.z, t_interp)
    return interp_state


def interpMultiFootState(
        state_1: lp_python_interface.MultiFootState,
        state_2: lp_python_interface.MultiFootState,
        t_interp: float) -> lp_python_interface.MultiFootState:

    interp_state = lp_python_interface.MultiFootState()
    # Interp foot state
    interp_state.feet.resize(state_1.feet.size())
    for i in range(interp_state.feet.size()):

        interp_state.feet[i].position.x = lerp(state_1.feet[i].position.x,
                                               state_2.feet[i].position.x,
                                               t_interp)
        interp_state.feet[i].position.y = lerp(state_1.feet[i].position.y,
                                               state_2.feet[i].position.y,
                                               t_interp)
        interp_state.feet[i].position.z = lerp(state_1.feet[i].position.z,
                                               state_2.feet[i].position.z,
                                               t_interp)

        # Interp foot velocity
        interp_state.feet[i].velocity.x = lerp(state_1.feet[i].velocity.x,
                                               state_2.feet[i].velocity.x,
                                               t_interp)
        interp_state.feet[i].velocity.y = lerp(state_1.feet[i].velocity.y,
                                               state_2.feet[i].velocity.y,
                                               t_interp)
        interp_state.feet[i].velocity.z = lerp(state_1.feet[i].velocity.z,
                                               state_2.feet[i].velocity.z,
                                               t_interp)

        # Interp foot acceleration
        interp_state.feet[i].acceleration.x = lerp(
            state_1.feet[i].acceleration.x, state_2.feet[i].acceleration.x,
            t_interp)
        interp_state.feet[i].acceleration.y = lerp(
            state_1.feet[i].acceleration.y, state_2.feet[i].acceleration.y,
            t_interp)
        interp_state.feet[i].acceleration.z = lerp(
            state_1.feet[i].acceleration.z, state_2.feet[i].acceleration.z,
            t_interp)

        # Set contact state to the first state
        interp_state.feet[i].contact = state_1.feet[i].contact
    return interp_state


def interpJointState(joint_1: lp_python_interface.JointState,
                     joint_2: lp_python_interface.JointState,
                     t_interp: float) -> lp_python_interface.JointState:
    # Interp joints
    interp_joint = lp_python_interface.JointState()
    interp_joint.name.resize(joint_1.position.size())
    interp_joint.position.resize(joint_1.position.size())
    interp_joint.velocity.resize(joint_1.position.size())
    interp_joint.effort.resize(joint_1.position.size())
    for i in range(joint_1.position.size()):
        interp_joint.name[i] = joint_1.name[i]
        interp_joint.position[i] = lerp(joint_1.position[i],
                                        joint_2.position[i], t_interp)
        interp_joint.velocity[i] = lerp(joint_1.velocity[i],
                                        joint_2.velocity[i], t_interp)
        interp_joint.effort[i] = lerp(joint_1.effort[i], joint_2.effort[i],
                                      t_interp)
    return interp_joint


class LegController:
    def __init__(self, robot: Robot, 
                 robot_state: lp_python_interface.RobotState = None, 
                 leg_array: lp_python_interface.MultiFootPlanContinuous = None,
                 grf_array: lp_python_interface.GRFArray = None,
                 interval: float = 0.03):
        self._robot = robot
        self._robot_state = robot_state
        self._leg_array = leg_array
        self._grf_array = grf_array
        self._interval = interval

    def reset(self, current_time: float) -> None:
        return

    @property
    def robot_state(self):
        return self._robot_state

    @robot_state.setter
    def robot_state(self, robot_state: lp_python_interface.RobotState) -> None:
        self._robot_state = robot_state

    @property
    def leg_array(self):
        return self._leg_array

    @leg_array.setter
    def leg_array(
            self,
            leg_array: lp_python_interface.MultiFootPlanContinuous) -> None:
        self._leg_array = leg_array

    @property
    def grf_array(self):
        return self._grf_array

    @grf_array.setters
    def grf_array(self, grf_array: lp_python_interface.GRFArray) -> None:
        self._grf_array = grf_array

    def receive_local_plan(self,
                           last_local_plan: lp_python_interface.RobotPlan):
        self._last_local_plan = last_local_plan  # actually robotplan is enough? MultiFootPlanDiscrete and MultiFootPlanContinuous not used

    def get_action(self, t_elipsed: float) -> Mapping[Any, Any]:

        # Interpolate: How to do this? flow:
        # after solving one mpc, what we did is to start a new clock,
        # using the simulation time inside this new clock timeframe to interpolate
        # the grf, and footstate.
        # But initially we need to be careful about when the mpc happens and the time delay it causes.
        # so we let mpc to happen only after finish one step,  basically fourfoot on ground
        # that means we also not allowed to use time to track global_plan,
        # but using a record to record which node will be tracked next
        t_segment = math.floor(
            t_elipsed / self._interval
        )  # I guess there is index difference between each segment?
        t_interp = t_elipsed % self._interval
        target_state = interpRobotState(
            self.last_local_plan.states[t_segment],
            self.last_local_plan.states[t_segment + 1], t_interp)
        target_grf = interpGRFArray(self.last_local_plan.grf[t_segment],
                                    self.last_local_plan.grf[t_segment + 1],
                                    t_interp)
        foot_traj = target_state.feet
        grf = target_grf
        all_joint_inputs = {}
        action = {}

        for joint in range(target_state.joint.position.size()):
            all_joint_inputs[joint] = (target_state.joint.position[joint],
                                       target_state.joint.velocity[joint], 0)

        for leg, _ in enumerate(self._robot._foot_link_ids):
            if foot_traj.feet[leg].contact == True:  # stance leg
                motor_torques = self._robot.map_contact_force_to_joint_torques(
                    leg, [
                        target_grf.vectors[leg].x, target_grf.vectors[leg].y,
                        target_grf.vectors[leg].z
                    ])
                for joint, torque in motor_torques.items():
                    to_list = list(all_joint_inputs[joint])
                    to_list[2] = torque
                    all_joint_inputs[joint] = to_list
            # else:
            #     foot_position = foot_traj.feet[leg].position
            #     joint_ids, joint_angles = (
            #         self._robot.get_motor_angles_from_foot_position(
            #             leg, foot_position))

        kps = self._robot.motor_group.kps
        kds = self._robot.motor_group.kds
        for joint, joint_input in all_joint_inputs.items():
            #leg_id = joint_angle_leg_id[
            action[joint] = MotorCommand(desired_position=joint_input[0],
                                         kp=kps[joint],
                                         desired_velocity=joint_input[1],
                                         kd=kds[joint],
                                         desired_extra_torque=joint_input[2])

        return action

