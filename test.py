# from scipy.spatial.transform import Rotation 
# from scipy.spatial.transform import Slerp
# q_1 = [0.0, 0.0, 0.0, 1.0]
# q_2 = [1.0, 0.0, 0.0, 0.0]
# r_1=  Rotation.from_quat(q_1)
# r_2 = Rotation.from_quat(q_2)
# slerp = Slerp([1,2], [r_1, r_2])
# r_interp = slerp(0.5)  
# q_interp = Rotation.as_quat(r_interp)
# print(q_interp)
a = {}
a['m'] = (1,2,3)
a['n'] = (4,5,6)
k = list(a['m'])
k[2] = 100
a['m'] = k
print(a['m'])


import gbp_python_interface
import lp_python_interface

import numpy as np

class test_gbp:
	def __init__(
		self,
		terrain_map = np.zeros((100, 100), dtype='float64'),
		origin = [0, 0]
	):	
		self.terrain_map = terrain_map
		self.origin = [0, 0]
		self.robot_state_msg = lp_python_interface.RobotState()
		self.robot_state_msg.body.pose.orientation.w = 1.
		self.robot_state_msg.body.pose.orientation.x = 0.
		self.robot_state_msg.body.pose.orientation.y = 0.
		self.robot_state_msg.body.pose.orientation.z = 0.

		self.robot_state_msg.body.pose.position.x = 0.
		self.robot_state_msg.body.pose.position.y = 0.
		self.robot_state_msg.body.pose.position.z = 0.3

		self.robot_state_msg.body.twist.linear.x = 0.
		self.robot_state_msg.body.twist.linear.y = 0.
		self.robot_state_msg.body.twist.linear.z = 0.

		self.robot_state_msg.body.twist.angular.x = 0.
		self.robot_state_msg.body.twist.angular.y = 0.
		self.robot_state_msg.body.twist.angular.z = 0.

		self.goal_point_msg = lp_python_interface.Point()
		self.goal_point_msg.x = 1
		self.goal_point_msg.y = 1
		self.goal_point_msg.z = 0.

		
	def get_plan(self):
		global_planner = gbp_python_interface.GlobalBodyPlanner()
		self.global_plan = lp_python_interface.GlobalPlan()
		self.global_plan = global_planner.getPlan(
			self.robot_state_msg, self.goal_point_msg, self.terrain_map, self.origin)
		print(type(self.global_plan.robot_plan.plan_indices))
		return self.global_plan
		# print(self.global_plan.discrete_robot_plan.states[0].body.pose.position.x)
		
		# for i in range(len(self.global_plan.robot_plan.states)):
		# print(self.global_plan.robot_plan.states[i].body.pose.position.x)
	
class test_lp:
	def __init__(
		self,
		terrain_map = np.zeros((100, 100), dtype='float64'),
		origin = [0, 0]
	):
		self.terrain_map = terrain_map
		self.origin = origin
		self.gbp = test_gbp()
		self.global_plan_msg = self.gbp.get_plan()
		self.robot_state_msg = self.gbp.robot_state_msg
		self.gait_pattern_msg = lp_python_interface.GaitPattern()
		self.gait_pattern_msg.phase_offset = [0.0, 0.5, 0.5, 0.0]
		self.gait_pattern_msg.duty_cycle = [0.3, 0.3, 0.3, 0.3]
		self.get_robot_state()
		print(self.global_plan_msg.robot_plan.state_timestamp)
		print(self.global_plan_msg.robot_plan.global_plan_timestamp)
		print("length: ", len(self.robot_state_msg.feet.feet))

	def get_local_plan(self):
		local_planner = lp_python_interface.LocalPlanner()
		self.local_plan = lp_python_interface.LocalPlan()
		self.local_plan = local_planner.getPlan(self.robot_state_msg, 
		    self.global_plan_msg.robot_plan, self.gait_pattern_msg, self.terrain_map, self.origin)
		print(len(self.local_plan.local_plan.states))
		print(self.local_plan.local_plan.states[1].body.pose.position.x)
		print(self.local_plan.local_plan.states[1].body.pose.position.y)
		print(self.local_plan.local_plan.states[1].body.pose.position.z)


	def get_robot_state(self):
		# self.robot_state_msg.feet.feet = [lp_python_interface.FootState()]*4
		self.robot_state_msg.feet.feet = lp_python_interface.FootStateVector()
		foot_state= lp_python_interface.FootState()
		foot_state.position.x = 0.2263
		foot_state.position.y = 0.17098
		foot_state.position.z = 0.02
		foot_state.velocity.x = 0.0
		foot_state.velocity.y = 0.0
		foot_state.velocity.z = 0.0
		foot_state.acceleration.x = 0.0
		foot_state.acceleration.y = 0.0
		foot_state.acceleration.z = 0.0
		self.robot_state_msg.feet.feet.append(foot_state)
		foot_state.position.x = -0.2263
		foot_state.position.y = 0.17098
		foot_state.position.z = 0.02
		self.robot_state_msg.feet.feet.append(foot_state)
		foot_state.position.x = 0.2263
		foot_state.position.y = -0.17098
		foot_state.position.z = 0.02
		self.robot_state_msg.feet.feet.append(foot_state)
		foot_state.position.x = -0.2263
		foot_state.position.y = -0.17098
		foot_state.position.z = 0.02
		self.robot_state_msg.feet.feet.append(foot_state)
		print("length: ", len(self.robot_state_msg.feet.feet))

		# to set the traj_index and timestamp 
		
print("python here1")
lp = test_lp()
print("python here2")
lp.get_local_plan()
print("python here3")
	