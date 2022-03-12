"""A model based controller framework."""
from absl import logging

from datetime import datetime
import enum
import ml_collections
import numpy as np
import os
import pickle
import pybullet
from pybullet_utils import bullet_client
import threading
import time
from typing import Tuple

from src.convex_mpc_controller import com_velocity_estimator
from src.convex_mpc_controller import offset_gait_generator
from src.convex_mpc_controller import raibert_swing_leg_controller
from src.convex_mpc_controller import torque_stance_leg_controller_mpc
from src.convex_mpc_controller.gait_configs import crawl, trot, flytrot
from src.robots import a1
from src.robots import spirit
from src.robots.motors import MotorCommand
from src.robots.motors import MotorControlMode
from src.robots.terrain import randomRockyTerrain
from src.controller.leg_controller import LegController
from src.controller.global_planner import GlobalPlanner
from src.controller.local_planner import LocalPlanner

import gbp_python_interface
import lp_python_interface


class ControllerMode(enum.Enum):
    DOWN = 1
    STAND = 2
    WALK = 3
    TERMINATE = 4


class GaitType(enum.Enum):
    CRAWL = 1
    TROT = 2
    FLYTROT = 3


def get_sim_conf():
    config = ml_collections.ConfigDict()
    config.timestep: float = 0.001
    config.action_repeat: int = 1
    config.reset_time_s: float = 3.
    config.standup_time_s: float = 3.
    config.num_solver_iterations: int = 30
    config.init_position: Tuple[float, float, float] = (0., 0., 0.32)
    config.init_rack_position: Tuple[float, float, float] = [0., 0., 1]
    config.on_rack: bool = False
    return config


class Locomotion(object):
    """Generates the quadruped locomotion.

  The actual effect of this controller depends on the composition of each
  individual subcomponent.

  """
    def __init__(self,
                 use_real_robot: bool = False,
                 show_gui: bool = False,
                 logdir: str = 'logs/'):
        """Initializes the class.

    Args:
      robot: A robot instance.
      gait_generator: Generates the leg swing/stance pattern.
      state_estimator: Estimates the state of the robot (e.g. center of mass
        position or velocity that may not be observable from sensors).
      swing_leg_controller: Generates motor actions for swing legs.
      stance_leg_controller: Generates motor actions for stance legs.
      clock: A real or fake clock source.
    """
        self._use_real_robot = use_real_robot
        self._show_gui = show_gui
        self._setup_robot_and_terrain()
        self._setup_planner_and_controllers()

        self.reset_robot()
        self.reset_planners()
        self._reset_time = self._clock() # timer: calculate the simulation time
        self._global_plan_reset_time = self._reset_time
        self._local_plan_reset_time = self._reset_time
        self._logs = []
        self._logdir = logdir

        self._node_interval = 0.03
        self._global_plan = None
        self._local_plan = None

        self.run_thread = threading.Thread(target=self.run)
        self.run_thread.start()

    def _setup_robot_and_terrain(self):
        # Construct robot
        if self._show_gui:
            p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
        else:
            p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)

        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setAdditionalSearchPath('src/data')
        self.pybullet_client = p
        p.setPhysicsEngineParameter(numSolverIterations=30)
        p.setTimeStep(0.002)
        p.setGravity(0, 0, -9.8)
        p.setPhysicsEngineParameter(enableConeFriction=0)

        # Construct terrain:
        self._terrain = randomRockyTerrain()
        self._terrain.generate()
        self._ground_id = self._terrain.terrainBody
        self._terrain_map = self._terrain.sensedHeightMapSquare([0.0, 0.0],
                                                                [100, 100])
        self._goal = [3, 3]

        # Construct robot class:
        self._robot = spirit.Spirit(pybullet_client=p,
                                    sim_conf=get_sim_conf(),
                                    motor_control_mode=MotorControlMode.HYBRID)

        if self._show_gui and not self._use_real_robot:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        self._clock = lambda: self._robot.time_since_reset

    def _setup_planner_and_controller(self):
        self._gait_pattern = lp_python_interface.GaitPattern()
        self._gait_pattern.phase_offset = [0.0, 0.5, 0.5, 0.0]
        self._gait_pattern.duty_cycle = [0.3, 0.3, 0.3, 0.3]
        self._global_planner = GlobalPlanner(self.pybullet_client, self._robot,
                                             self._terrain_map, goal_point=self._goal)
        self._local_planner = LocalPlanner(self.pybullet_client, self._robot,
                                           self._terrain_map, self._gait_pattern)
        self._leg_controller = LegController(self._robot)

    def reset_robot(self):
        self._robot.reset()
        self._robot.stand_up()
        if self._show_gui and not self._use_real_robot:
            self.pybullet_client.configureDebugVisualizer(
                self.pybullet_client.COV_s ENABLE_RENDERING, 1)

    def reset_planners(self):
        # Resetting other components
        self._reset_time = self._clock() # 0.0
        self._global_plan_reset_time = self._reset_time
        self._local_plan_reset_time = self._reset_time
        self._global_planner.reset(self._time_since_reset)
        self._local_planner.reset(self._time_since_reset)
        self._leg_controller.reset(self._time_since_reset)

    @property
    def time_since_global_planner_update(self):
        return self._time_since_global_plan_update

    @property
    def time_since_local_planner_update(self):
        return self._time_since_local_plan_update

    def update(self):
        self._time_since_reset = self._clock() - self._reset_time
        self._time_since_global_plan_update = self._clock() - self._global_plan_reset_time
        self._time_since_local_plan_update = self._clock() - self._local_plan_reset_time

    def get_action(self):
        """Returns the control ouputs (e.g. positions/torques) for all motors."""
        self._leg_controller.receive_local_plan(self._local_plan)
        action = self._leg_controller.get_action(self._time_since_reset)
        return action
    
    # def get_action(self):
    #   """Returns the control ouputs (e.g. positions/torques) for all motors."""
    #   swing_action = self._swing_controller.get_action()
    #   stance_action, qp_sol = self._stance_controller.get_action()

    #   actions = []
    #   for joint_id in range(self._robot.num_motors):
    #     if joint_id in swing_action:
    #       actions.append(swing_action[joint_id])
    #     else:
    #       assert joint_id in stance_action
    #       actions.append(stance_action[joint_id])

    #   vectorized_action = MotorCommand(
    #       desired_position=[action.desired_position for action in actions],
    #       kp=[action.kp for action in actions],
    #       desired_velocity=[action.desired_velocity for action in actions],
    #       kd=[action.kd for action in actions],
    #       desired_extra_torque=[
    #           action.desired_extra_torque for action in actions
    #       ])

    #   return vectorized_action, dict(qp_sol=qp_sol)

    # def _handle_mode_switch(self):
    #     if self._mode == self._desired_mode:
    #         return
    #     self._mode = self._desired_mode
    #     if self._desired_mode == ControllerMode.DOWN:
    #         logging.info("Entering joint damping mode.")
    #         self._flush_logging()
    #     elif self._desired_mode == ControllerMode.STAND:
    #         logging.info("Standing up.")
    #         self.reset_robot()
    #     else:
    #         logging.info("Walking.")
    #         self.reset_controllers()
    #         self._start_logging()

    def run(self):
        logging.info("main thread started...")
        self._global_plan = self._global_planner.update()
        self._global_plan_reset_time = self._clock
        while True:
            self.update() # update time
            if self._time_since_local_plan_update == self._node_interval:
                self._local_plan = self._local_planner.update()
                self._global_plan_reset_time = self._clock
        
            action = self.get_action()
            self._robot.step(action)
            self._update_logging(action)
            time.sleep(0.001) # nothing to do with simulation step

            # Camera setup:
            if self._show_gui:
                self.pybullet_client.resetDebugVisualizerCamera(
                    cameraDistance=1.0,
                    cameraYaw=30 +
                    self._robot.base_orientation_rpy[2] / np.pi * 180,
                    cameraPitch=-30,
                    cameraTargetPosition=self._robot.base_position,
                )

    @property
    def is_safe(self):
        if self.mode != ControllerMode.WALK:
            return True
        rot_mat = np.array(
            self._robot.pybullet_client.getMatrixFromQuaternion(
                self._state_estimator.com_orientation_quat_ground_frame)
        ).reshape((3, 3))
        up_vec = rot_mat[2, 2]
        base_height = self._robot.base_position[2]
        return up_vec > 0.85 and base_height > 0.18

    def _start_logging(self):
        self._logs = []

    def _update_logging(self, action):
        frame = dict(
            desired_speed=(self._swing_controller.desired_speed,
                           self._swing_controller.desired_twisting_speed),
            timestamp=self._time_since_reset,
            base_rpy=self._robot.base_orientation_rpy,
            motor_angles=self._robot.motor_angles,
            base_vel=self._robot.motor_velocities,
            #base_vels_body_frame=self._state_estimator.com_velocity_body_frame,
            base_rpy_rate=self._robot.base_rpy_rate,
            motor_vels=self._robot.motor_velocities,
            motor_torques=self._robot.motor_torques,
            #contacts=self._robot.foot_contacts,
            #desired_grf=qp_sol,
            robot_action=action,
            #gait_generator_phase=self._gait_generator.current_phase.copy(),
            #gait_generator_state=self._gait_generator.leg_state,
            #ground_orientation=self._state_estimator.
            #ground_orientation_world_frame,
        )
        self._logs.append(frame)

    def _flush_logging(self):
        if not os.path.exists(self._logdir):
            os.makedirs(self._logdir)
        filename = 'log_{}.pkl'.format(
            datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
        pickle.dump(self._logs, open(os.path.join(self._logdir, filename),
                                     'wb'))
        logging.info("Data logged to: {}".format(
            os.path.join(self._logdir, filename)))
