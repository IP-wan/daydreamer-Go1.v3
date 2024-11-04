"""Replays pre-recorded actions on the robot."""
from absl import app
from absl import flags

import numpy as np
import pybullet # pytype:disable=import-error
import pybullet_data
from pybullet_utils import bullet_client
import time

# from motion_imitation.robots import go1
from motion_imitation.robots import go1_robot
# from motion_imitation.envs import env_builder
from motion_imitation.robots import robot_config

flags.DEFINE_string('traj_dir', None, 'directory of trajectory file.')
FLAGS = flags.FLAGS


def main(_):
  traj = dict(np.load(FLAGS.traj_dir))

  p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
  p.setPhysicsEngineParameter(numSolverIterations=30)
  p.setTimeStep(0.001)
  p.setGravity(0, 0, -10)
  p.setPhysicsEngineParameter(enableConeFriction=0)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  p.loadURDF("plane.urdf")

  robot = go1_robot.Go1Robot(
      p,
      motor_control_mode=robot_config.MotorControlMode.HYBRID,
      enable_action_interpolation=False,
      reset_time=2)

  # env = env_builder.build_regular_env(
  #     go1.Go1,
  #     motor_control_mode=robot_config.MotorControlMode.HYBRID,
  #     enable_rendering=True,
  #     on_rack=False,
  #     wrap_trajectory_generator=False)
  # robot = env.robot
  input("Press Enter Key to Start...")
  for action in traj['action'][:100]:
    robot.Step(action)
    time.sleep(0.01)
  robot.Terminate()


if __name__ == "__main__":
  app.run(main)
