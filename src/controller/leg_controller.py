import copy
import math
import numpy as np
from typing import Any
import time
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation 

import lp_python_interface

def lerp(a: float, b: float, t: float) -> float:
	return (a + t * (b - a))

def interpRobotState(state_1: lp_python_interface.RobotState, state_2: lp_python_interface.RobotState, t_interp: float) -> lp_python_interface.RobotState:
  # Interp individual elements, t_interp is the new in the new mpc-afterward timeframe
  interp_state = lp_python_interface.RobotState()
  interp_state.body = interpOdometry(state_1.body, state_2.body, t_interp)
  interp_state.joints = interpJointState(state_1.joints, state_2.joints, t_interp)
  interp_state.feet = interpMultiFootState(state_1.feet, state_2.feet, t_interp, interp_state.feet)
  return interp_state


def interpOdometry(state_1: lp_python_interface.BodyState, state_2: lp_python_interface.BodyState,
                   t_interp: float) -> lp_python_interface.BodyState:
  interp_state = lp_python_interface.BodyState()

  # Interp body position
  interp_state.pose.position.x = lerp(
      state_1.pose.position.x, state_2.pose.position.x, t_interp)
  interp_state.pose.position.y = lerp(
      state_1.pose.position.y, state_2.pose.position.y, t_interp)
  interp_state.pose.position.z = lerp(
      state_1.pose.position.z, state_2.pose.position.z, t_interp)

  # Interp body orientation with slerp
  # tf2::Quaternion q_1, q_2, q_interp
  # tf2::convert(state_1.pose.orientation, q_1)
  # tf2::convert(state_2.pose.orientation, q_2)
  # q_interp = q_1.slerp(q_2, t_interp)
  # interp_state.pose.orientation = tf2::toMsg(q_interp) 
  q_1 = [state_1.pose.orientation.x, state_1.pose.orientation.y, state_1.pose.orientation.z, state_1.pose.orientation.w]
  q_2 = [state_2.pose.orientation.x, state_2.pose.orientation.y, state_2.pose.orientation.z, state_2.pose.orientation.w]
  r_1 = Rotation.from_quat(q_1)
  r_2 = Rotation.from_quat(q_2)
  slerp = Slerp([1,2], [r_1, r_2])
  r_interp = slerp(t_interp)  
  q_interp = Rotation.as_quat(r_interp)
  interp_state.pose.orientation.x = q_interp[0]
  interp_state.pose.orientation.y = q_interp[1]
  interp_state.pose.orientation.z = q_interp[2]
  interp_state.pose.orientation.w = q_interp[3]

  # Interp twist
  interp_state.twist.linear.x = lerp(
      state_1.twist.linear.x, state_2.twist.linear.x, t_interp)
  interp_state.twist.linear.y = lerp(
      state_1.twist.linear.y, state_2.twist.linear.y, t_interp)
  interp_state.twist.linear.z = lerp(
      state_1.twist.linear.z, state_2.twist.linear.z, t_interp)

  interp_state.twist.angular.x = lerp(
      state_1.twist.angular.x, state_2.twist.angular.x, t_interp)
  interp_state.twist.angular.y = lerp(
      state_1.twist.angular.y, state_2.twist.angular.y, t_interp)
  interp_state.twist.angular.z = lerp(
      state_1.twist.angular.z, state_2.twist.angular.z, t_interp)
  return interp_state

def interpMultiFootState(state_1: lp_python_interface.MultiFootState,
                          quad_msgs::MultiFootState state_2, double t_interp,
                          quad_msgs::MultiFootState &interp_state) {
  interpHeader(state_1.header, state_2.header, t_interp, interp_state.header);

  // Interp foot state
  interp_state.feet.resize(state_1.feet.size());
  for (int i = 0; i < interp_state.feet.size(); i++) {
    interp_state.feet[i].header = interp_state.header;

    interp_state.feet[i].position.x = math_utils::lerp(
        state_1.feet[i].position.x, state_2.feet[i].position.x, t_interp);
    interp_state.feet[i].position.y = math_utils::lerp(
        state_1.feet[i].position.y, state_2.feet[i].position.y, t_interp);
    interp_state.feet[i].position.z = math_utils::lerp(
        state_1.feet[i].position.z, state_2.feet[i].position.z, t_interp);

    // Interp foot velocity
    interp_state.feet[i].velocity.x = math_utils::lerp(
        state_1.feet[i].velocity.x, state_2.feet[i].velocity.x, t_interp);
    interp_state.feet[i].velocity.y = math_utils::lerp(
        state_1.feet[i].velocity.y, state_2.feet[i].velocity.y, t_interp);
    interp_state.feet[i].velocity.z = math_utils::lerp(
        state_1.feet[i].velocity.z, state_2.feet[i].velocity.z, t_interp);

    // Interp foot acceleration
    interp_state.feet[i].acceleration.x =
        math_utils::lerp(state_1.feet[i].acceleration.x,
                         state_2.feet[i].acceleration.x, t_interp);
    interp_state.feet[i].acceleration.y =
        math_utils::lerp(state_1.feet[i].acceleration.y,
                         state_2.feet[i].acceleration.y, t_interp);
    interp_state.feet[i].acceleration.z =
        math_utils::lerp(state_1.feet[i].acceleration.z,
                         state_2.feet[i].acceleration.z, t_interp);

    // Set contact state to the first state
    interp_state.feet[i].contact = state_1.feet[i].contact;
  }
}

def interpJointState(state_1: lp_python_interface.JointState,
                     state_2: lp_python_interface.JointState, t_interp: float) -> lp_python_interface.JointState:
  # Interp joints
  interp_state = interp_state = lp_python_interface.JointState()
  interp_state.name.resize(state_1.position.size())
  interp_state.position.resize(state_1.position.size())
  interp_state.velocity.resize(state_1.position.size())
  interp_state.effort.resize(state_1.position.size())
  for i in range(state_1.position.size()): 
    interp_state.name[i] = state_1.name[i]
    interp_state.position[i] = lerp(state_1.position[i], state_2.position[i], t_interp)
    interp_state.velocity[i] = lerp(state_1.velocity[i], state_2.velocity[i], t_interp)
    interp_state.effort[i] = lerp(state_1.effort[i], state_2.effort[i], t_interp)
  return interp_state

class SwingLegController:
    def __init__(self, robot_state: Any, leg_array:Any, grf_array:Any):
        self._robot_state = robot_state
        self._leg_array =  leg_array
        self._grf_array = grf_array
        self._clock = time.time()

    def reset(self):
        self._clock = 0.0

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
    def leg_array(self, leg_array: lp_python_interface.MultiFootPlanContinuous) -> None:
        self._leg_array = leg_array

    @property
    def grf_array(self):
        return self._grf_array
            
    @grf_array.setter
    def grf_array(self, grf_array: lp_python_interface.GRFArray) -> None:
        self._grf_array = grf_array

    def get_action(self):

        #  Interpolate: How to do this? flow:
        # after solving one mpc, what we did is to start a new clock, 
        # using the simulation time inside this new clock timeframe to interpolate
        # the grf, and footstate. 
        # But initially we need to be careful about when the mpc happens and the time delay it causes.
        # so we let mpc to happen only after finish one step,  basically fourfoot on ground
        # that means we also not allowed to use time to track global_plan, 
        # but using a record to record which node will be tracked next 
        a =1
