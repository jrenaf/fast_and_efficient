import gbp_python_interface
import lp_python_interface

import numpy as np
from typing import Any
from src.robots.robot import Robot
from src.controller.global_planner import GlobalPlanner

class LocalPlanner:
	def __init__(
		self,
		pybullet_client: Any,
		robot: Robot,
        terrain_map: np.ndarray, # terrain_map = np.zeros((100, 100), dtype='float64'),
        origin: np.array = np.array((0., 0.), dtype='float64'),
        global_plan: lp_python_interface.GlobalPlan = None,
		robot_state: lp_python_interface.RobotState = None,
		gait_pattern: lp_python_interface.GaitPattern = None
	):	
		self._pybullet_client = pybullet_client
		self._robot = robot
		self._terrain_map = terrain_map
		self._origin = origin
		self._global_plan = global_plan # self._global_plan_msg = self.gbp.get_plan()
		self._robot_state = robot_state # self.robot_state_msg = self.gbp.robot_state_msg
		self._gait_pattern = gait_pattern
		self._time_since_reset =0.
		# self.gait_pattern_msg = lp_python_interface.GaitPattern()
		# self.gait_pattern_msg.phase_offset = [0.0, 0.5, 0.5, 0.0]
		# self.gait_pattern_msg.duty_cycle = [0.3, 0.3, 0.3, 0.3]
		self.reset(0)
		# self.get_robot_state()

	@property
	def terrain_map(self):
		return self._terrain_map

	@terrain_map.setter
	def terrain_map(self, terrain_map:  np.ndarray) -> None:
		self._robot_state = terrain_map

	@property
	def terrain_map(self):
		return self._terrain_map

	@terrain_map.setter
	def terrain_map(self, terrain_map:  np.ndarray) -> None:
		self._robot_state = terrain_map

	@property
	def robot_state(self):
		return self._robot_state

	@property
	def gait_pattern(self):
		return self._gait_pattern

	@terrain_map.setter
	def gait_pattern(self, gait_pattern:  lp_python_interface.GaitPattern) -> None:
		self._gait_pattern = gait_pattern

	def get_local_plan(self) -> lp_python_interface.LocalPlan():
		self._local_planner = lp_python_interface.LocalPlanner()
		self._retrieve_robot_state()
		self._local_plan = self._local_planner.getPlan(self._robot_state, 
		    self._global_plan.robot_plan, self._gait_pattern, self._terrain_map, self._origin)
		return self._local_plan

	def _retrieve_robot_state(self):
		# self._robot_state.feet.feet = [lp_python_interface.FootState()]*4
		self._robot_state.body.pose.orientation.x = self._robot.base_orientation_quat[0]
		self._robot_state.body.pose.orientation.y = self._robot.base_orientation_quat[1]
		self._robot_state.body.pose.orientation.z = self._robot.base_orientation_quat[2]
		self._robot_state.body.pose.orientation.w = self._robot.base_orientation_quat[3]

		self._robot_state.body.pose.pose.x = self._robot.base_position[0]
		self._robot_state.body.pose.pose.y = self._robot.base_position[1]
		self._robot_state.body.pose.pose.z = self._robot.base_position[2]

		self._robot_state.body.twist.linear.x = self._robot.base_velocity[0]
		self._robot_state.body.twist.linear.y = self._robot.base_velocity[1]
		self._robot_state.body.twist.linear.z = self._robot.base_velocity[2]

		self._robot_state.body.twist.angular.x = self._robot.base_rpy_rate[0]
		self._robot_state.body.twist.angular.y = self._robot.base_rpy_rate[1]
		self._robot_state.body.twist.angular.z = self._robot.base_rpy_rate[2]

		self._robot_state.feet.feet = lp_python_interface.FootStateVector()
		for foot in range(self._robot._foot_link_ids.size()):
			link_state = self._robot.pybullet_client.getLinkState(self._robot.quadruped, foot)
			link_position_world = link_state[0]
			link_velocity_world = link_state[6]
			foot_state= lp_python_interface.FootState()
			foot_state.position.x = link_position_world[0]
			foot_state.position.y = link_position_world[1]
			foot_state.position.z = link_position_world[2]
			foot_state.velocity.x = link_velocity_world[0]
			foot_state.velocity.y = link_velocity_world[1]
			foot_state.velocity.z = link_velocity_world[2]
			foot_state.acceleration.x = 0.0
			foot_state.acceleration.y = 0.0
			foot_state.acceleration.z = 0.0
			self._robot_state.feet.feet.append(foot_state)

	def reset(self, current_time: float) -> None:
		return

	def update(self) -> lp_python_interface.RobotPlan():
		self._retrieve_robot_state()
		self._local_plan = self.get_plan()
		return self._global_plan.local_plan