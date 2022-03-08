'''
To run: python3 -m src.robots.spirit_robot_exercise_example
'''
from absl import app
from absl import flags

from src.robots import spirit
from src.robots.motors import MotorCommand
from src.robots.terrain import randomRockyTerrain

import numpy as np
import time
import enum
import pybullet
from pybullet_utils import bullet_client

import ml_collections
import gbp_python_interface
import lp_python_interface


def get_action(robot, t):
    mid_action = np.array([0.0, 0.76, 1.52] * 4)
    amplitude = np.array([0.0, 0.2,  0.2] * 4) *0
    freq = 1.0
    return MotorCommand(desired_position=mid_action+
                      amplitude * np.sin(2 * np.pi * freq * t),
                      kp=robot.motor_group.kps,
                      desired_velocity=np.zeros(robot.num_motors),
                      kd=robot.motor_group.kds)       

def get_sim_conf():
  config = ml_collections.ConfigDict()
  config.timestep: float = 0.002
  config.action_repeat: int = 1
  config.reset_time_s: float = 3.
  config.num_solver_iterations: int = 30
  config.init_position: Tuple[float, float, float] = (0., 0., 0.35)
  config.init_rack_position: Tuple[float, float, float] = [0., 0., 1]
  config.on_rack: bool = False
  return config

def main(_):
    p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
    p.setAdditionalSearchPath('src/data')
    p.loadURDF("plane.urdf")
    p.setGravity(0.0, 0.0, -9.8)
    
    #terrain = randomRockyTerrain()
    #terrain.generate()

    #terrain_map = terrain.sensedHeightMapSquare([0.0,0.0], [100,100]) #direction?

    robot = spirit.Spirit(pybullet_client=p, sim_conf=get_sim_conf())
    robot.reset() # init

    for i in range(100000):
        time.sleep(0.1)
        action = get_action(robot, robot.time_since_reset)
        robot.step(action)
        time.sleep(0.001)
        print(robot.base_orientation_rpy)

if __name__ == "__main__":
    app.run(main)

