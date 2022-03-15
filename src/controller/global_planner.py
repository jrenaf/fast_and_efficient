import gbp_python_interface
import lp_python_interface

import pybullet as p
import numpy as np
from typing import Any
from src.robots.robot import Robot


class GlobalPlanner(object):
    def __init__(
        self,
        pybullet_client: Any,
		robot: Robot,
        terrain_map: np.ndarray, # terrain_map = np.zeros((100, 100), dtype='float64'),
        origin: np.array = np.array((0., 0.), dtype='float64'),
		robot_state: Any = None,
		goal_point: Any = None  # = np.array((0, 0))
		# terrain_params: dict = {} # we shouldn't actually need this
    ) -> None:
        self._pybullet_client = pybullet_client
        self._robot = robot
        self._terrain_map = terrain_map
        self._origin = origin
        if robot_state == None:
            self._robot_state = lp_python_interface.RobotState()
        else:
            self._robot_state = robot_state
        if goal_point == None:
            self._goal_point = lp_python_interface.Point()
        else:
            self._goal_point = goal_point
        self._debug_trajectory_array = []
        # self._terrain_params = terrain_params
        self.reset(0)

    @property
    def terrain_map(self):
        return self._terrain_map

    @terrain_map.setter
    def terrain_map(self, terrain_map:  np.ndarray) -> None:
        self._terrain_map = terrain_map

    @property
    def robot_state(self):
        return self._robot_state

    @property
    def goal_point(self):
        return self._goal_point

    def set_goal_point(self, goal_point:  np.array) -> None:
        self._goal_point.x  = goal_point[0]
        self._goal_point.y = goal_point[1]
        self._goal_point.z = 0.

    # @property
    # def terrain_params(self):
    #     return self._terrain_params

    # @terrain_params.setter
    # def terrain_params(self, terrain_params:  dict) -> None:
    #     self._terrain_params = terrain_params

    def get_plan(self) -> lp_python_interface.GlobalPlan():
        self._global_planner = gbp_python_interface.GlobalBodyPlanner()
        self._global_plan = self._global_planner.getPlan(
			self._robot_state, self._goal_point, self._terrain_map, self._origin)
        return self._global_plan

    def _retrieve_robot_state(self) -> None:
        self._robot_state.body.pose.orientation.x = self._robot.base_orientation_quat[0]
        self._robot_state.body.pose.orientation.y = self._robot.base_orientation_quat[1]
        self._robot_state.body.pose.orientation.z = self._robot.base_orientation_quat[2]
        self._robot_state.body.pose.orientation.w = self._robot.base_orientation_quat[3]

        self._robot_state.body.pose.position.x = self._robot.base_position[0]
        self._robot_state.body.pose.position.y = self._robot.base_position[1]
        self._robot_state.body.pose.position.z = self._robot.base_position[2]

        self._robot_state.body.twist.linear.x = self._robot.base_velocity[0]
        self._robot_state.body.twist.linear.y = self._robot.base_velocity[1]
        self._robot_state.body.twist.linear.z = self._robot.base_velocity[2]

        self._robot_state.body.twist.angular.x = self._robot.base_rpy_rate[0]
        self._robot_state.body.twist.angular.y = self._robot.base_rpy_rate[1]
        self._robot_state.body.twist.angular.z = self._robot.base_rpy_rate[2]

    def visualize(self) -> None:
        if self._global_plan.robot_plan.states:
            for i in range(0, len(self._global_plan.robot_plan.states)-1):
                debug_id = p.addUserDebugLine(
                       [self._global_plan.robot_plan.states[i].body.pose.position.x,
                       self._global_plan.robot_plan.states[i].body.pose.position.y,
                       self._global_plan.robot_plan.states[i].body.pose.position.z], 
                       [self._global_plan.robot_plan.states[i].body.pose.position.x,
                       self._global_plan.robot_plan.states[i+1].body.pose.position.y,
                       self._global_plan.robot_plan.states[i+1].body.pose.position.z], [1, 0, 0], 3)
                print([self._global_plan.robot_plan.states[i+1].body.pose.position.x,
                       self._global_plan.robot_plan.states[i].body.pose.position.y,
                       self._global_plan.robot_plan.states[i].body.pose.position.z])
                self._debug_trajectory_array.append(debug_id)

    def reset(self, current_time: float) -> None:
        if not self._debug_trajectory_array:
            for id in range (len(self._debug_trajectory_array)):
                p.removeVisualShape(id)
        self._debug_trajectory_array = []

    def update(self) -> lp_python_interface.GlobalPlan():
        self._retrieve_robot_state()
        self._global_plan = self.get_plan()
        self.visualize()
        return self._global_plan.robot_plan
