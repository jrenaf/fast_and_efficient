"""Base class for all robots."""
import ml_collections
import numpy as np
from typing import Any
from typing import Sequence
from typing import Tuple

from src.robots.motors import MotorControlMode
from src.robots.motors import MotorGroup
from src.robots.motors import MotorModel
from src.robots.robot import Robot


class Spirit(Robot):
  """Spirit Robot."""
  def __init__(
      self,
      pybullet_client: Any = None,
      sim_conf: ml_collections.ConfigDict = None,
      urdf_path: str = "spirit/spirit.urdf",
      base_joint_names: Tuple[str, ...] = (),
      foot_joint_names: Tuple[str, ...] = (
          "jtoe0",
          "jtoe1",
          "jtoe2",
          "jtoe3",
      ),
      motor_control_mode: MotorControlMode = MotorControlMode.POSITION,
      stand_positions: Tuple[float] = np.array(
        0.0, 0.76, 1.52, 0.0, 0.76, 1.52, 0.0, 0.76, 1.52, 0.0, 0.76, 1.52,), 
      mpc_body_height: float = 0.3,
      mpc_body_mass: float = 11.5,
      mpc_body_inertia: Tuple[float] = np.array(
          (0.05, 0, 0, 0, 0.1, 0, 0, 0, 0.1)), # (0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.2)),
  ) -> None:
    """Constructs an A1 robot and resets it to the initial states.
        Initializes a tuple with a single MotorGroup containing 12 MotoroModels.
        Each MotorModel is by default configured for the parameters of the A1.
        """
    motors = MotorGroup((
        MotorModel(
            name="8",
            motor_control_mode=motor_control_mode,
            init_position=0.0,
            min_position=-0.707,
            max_position=0.707,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=10,
            kd=0.1,
        ),
        MotorModel(
            name="0",
            motor_control_mode=motor_control_mode,
            init_position=0.76,
            min_position=-6.28318530718,
            max_position=6.28318530718,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=10,
            kd=0.1,
        ),
        MotorModel(
            name="1",
            motor_control_mode=motor_control_mode,
            init_position=1.52,
            min_position=0.0,
            max_position=3.14159265359,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=30, ############################
            kd=0.1,
        ),
         MotorModel(
            name="9",
            motor_control_mode=motor_control_mode,
            init_position=0.0,
            min_position=-0.707,
            max_position=0.707,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=10,
            kd=0.1,
        ),
        MotorModel(
            name="2",
            motor_control_mode=motor_control_mode,
            init_position=0.76,
            min_position=-6.28318530718,
            max_position=6.28318530718,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=10,
            kd=0.1,
        ),
        MotorModel(
            name="3",
            motor_control_mode=motor_control_mode,
            init_position=1.52,
            min_position=0.0,
            max_position=3.14159265359,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=50, ################################Left Rear
            kd=0.1,
        ),
         MotorModel(
            name="10",
            motor_control_mode=motor_control_mode,
            init_position=0.0,
            min_position=-0.707,
            max_position=0.707,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=10,
            kd=0.1,
        ),
        MotorModel(
            name="4",
            motor_control_mode=motor_control_mode,
            init_position=0.76,
            min_position=-6.28318530718,
            max_position=6.28318530718,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=10,
            kd=0.1,
        ),
        MotorModel(
            name="5",
            motor_control_mode=motor_control_mode,
            init_position=1.52,
            min_position=0.0,
            max_position=3.14159265359,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=30, ############################
            kd=0.1,
        ),
        MotorModel(
            name="11",
            motor_control_mode=motor_control_mode,
            init_position=0.0,
            min_position=-0.707,
            max_position=0.707,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=10,
            kd=0.1,
        ),
        MotorModel(
            name="6",
            motor_control_mode=motor_control_mode,
            init_position=0.76,
            min_position=-6.28318530718,
            max_position=6.28318530718,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=10,
            kd=0.1,
        ),
        MotorModel(
            name="7",
            motor_control_mode=motor_control_mode,
            init_position=1.52,
            min_position=0.0,
            max_position=3.14159265359,
            min_velocity=-30,
            max_velocity=30,
            min_torque=-40,
            max_torque=40,
            kp=50, ############################ Right Rear
            kd=0.1,
        ),
    ))
    self._mpc_body_height = mpc_body_height
    self._mpc_body_mass = mpc_body_mass
    self._mpc_body_inertia = mpc_body_inertia

    super().__init__(
        pybullet_client=pybullet_client,
        sim_conf=sim_conf,
        urdf_path=urdf_path,
        motors=motors,
        base_joint_names=base_joint_names,
        foot_joint_names=foot_joint_names,
        stand_positions=stand_positions
    )

  @property
  def mpc_body_height(self):
    return self._mpc_body_height

  @mpc_body_height.setter
  def mpc_body_height(self, mpc_body_height: float):
    self._mpc_body_height = mpc_body_height

  @property
  def mpc_body_mass(self):
    return self._mpc_body_mass

  @mpc_body_mass.setter
  def mpc_body_mass(self, mpc_body_mass: float):
    self._mpc_body_mass = mpc_body_mass

  @property
  def mpc_body_inertia(self):
    return self._mpc_body_inertia

  @mpc_body_inertia.setter
  def mpc_body_inertia(self, mpc_body_inertia: Sequence[float]):
    self._mpc_body_inertia = mpc_body_inertia

  @property
  def hip_positions_in_base_frame(self):
    return (
        (0.2263, -0.07, 0),
        (-0.2263, 0.07, 0),
        (0.2263, -0.07, 0),
        (-0.2263, -0.07, 0),
    )

  @property
  def num_motors(self):
    return 12
