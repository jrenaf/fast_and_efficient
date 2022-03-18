import gbp_python_interface
import lp_python_interface

import ml_collections
import numpy as np
from typing import Any, List
from src.robots.robot import Robot
from src.controller.global_planner import GlobalPlanner


class LocalPlanner:

    def __init__(
            self,
            pybullet_client: Any,
            robot: Robot,
            sim_conf: ml_collections.ConfigDict,
            terrain_map: np.ndarray,  # terrain_map = np.zeros((100, 100), dtype='float64'),
            origin,
            gait_pattern = None,
            global_plan = None,
            robot_state = None
            ):
        self._pybullet_client = pybullet_client
        self._robot = robot
        self._sim_conf = sim_conf
        self._terrain_map = terrain_map
        self._origin = origin
        self._global_plan = global_plan  # self._global_plan_msg = self.gbp.get_plan()
        if robot_state == None:
            self._robot_state = lp_python_interface.RobotState()
        else:
            self._robot_state = robot_state  # self.robot_state_msg = self.gbp.robot_state_msg
        self._gait_pattern = gait_pattern
        self._time_since_reset = 0.

        self._local_planner = lp_python_interface.LocalPlanner(self._terrain_map, self._origin)
        self.reset(0)
        # self.get_robot_state()

    @property
    def terrain_map(self):
        return self._terrain_map

    @terrain_map.setter
    def terrain_map(self, terrain_map: np.ndarray) -> None:
        self._robot_state = terrain_map

    @property
    def robot_state(self):
        return self._robot_state

    @property
    def gait_pattern(self):
        return self._gait_pattern

    @gait_pattern.setter
    def gait_pattern(self,
                     gait_pattern) -> None:
        self._gait_pattern = gait_pattern

    @property
    def global_plan(self):
        return self._gait_pattern

    def set_global_plan(self,
                        global_plan) -> None:
        self._global_plan = global_plan

    def _get_plan(self) -> lp_python_interface.LocalPlan():
        plan_index = round(self._robot.time_since_reset / self._sim_conf.timestep_per_index)
        self._local_plan = self._local_planner.getPlan(
            self._robot_state, self._global_plan.robot_plan,
            self._gait_pattern, False, self._terrain_map, self._origin,
            plan_index)
        return self._local_plan

    def _retrieve_robot_state(self):
        # self._robot_state.feet.feet = [lp_python_interface.FootState()]*4
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

        self._robot_state.feet.feet = lp_python_interface.FootStateVector()
        for foot in range(len(self._robot._foot_link_ids)):
            link_state = self._robot.pybullet_client.getLinkState(
                self._robot.quadruped, self._robot._foot_link_ids[foot], computeLinkVelocity=1)
            print(len(link_state))
            link_position_world = link_state[0]
            link_velocity_world = link_state[6]
            foot_state = lp_python_interface.FootState()
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
        self._local_plan = self._get_plan()
        return self._global_plan
