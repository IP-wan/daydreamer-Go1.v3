"""Estimates base velocity for Go1 robot from accelerometer readings."""
import numpy as np
from filterpy.kalman import KalmanFilter
from motion_imitation.utilities.moving_window_filter import MovingWindowFilter


class VelocityEstimator:
  """Estimates base velocity of Go1 robot.

  The velocity estimator consists of 2 parts:
  1) A state estimator for CoM velocity.

  Two sources of information are used:
  The integrated reading of accelerometer and the velocity estimation from
  contact legs. The readings are fused together using a Kalman Filter.

  2) A moving average filter to smooth out velocity readings
  """
  def __init__(self,
               robot,
               accelerometer_variance=0.1,
               sensor_variance=0.1,
               initial_variance=0.1,
               moving_window_filter_size=120):
    """Initiates the velocity estimator.

    See filterpy documentation in the link below for more details.
    https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html

    Args:
      robot: the robot class for velocity estimation.
      accelerometer_variance: оценка шума при считывании показаний акселерометра.
      sensor_variance: оценка шума для измерения скорости двигателя.
      initial_covariance: ковариационная оценка начального состояния.
    """
    self.robot = robot
    # dim_x=3 - Размер вектора состояния; dim_z=3 - Размер вектора измерений; dim_u=3 -
    self.filter = KalmanFilter(dim_x=3, dim_z=3, dim_u=3)
    # Начальное состояние.
    self.filter.x = np.zeros(3)
    # Задаем дисперсию
    self._initial_variance = initial_variance
    # np.eye(k=0) - возвращает матрицу, т. е. двумерный массив, имеющего 1 по диагонали и 0 в других местах
    # относительно определенной позиции, т. е. k-го значения.
    # Ковариационная матрица — это многомерный аналог дисперсии, для случая, когда у нас не одна случайная величина,
    # а случайный вектор.
    # Ковариация показывает, насколько переменные зависят друг от друга.

    # Ковариационная матрица для начального состояния
    self.filter.P = np.eye(3) * self._initial_variance  # State covariance
    # Ковариационная матрица ошибки модели.
    self.filter.Q = np.eye(3) * accelerometer_variance
    # Ковариационная матрица шума измерений. Маленькое значение означает точность оценки.
    self.filter.R = np.eye(3) * sensor_variance

    # Матрица наблюдений. Функция измерения.
    self.filter.H = np.eye(3)  # measurement function (y=H*x)
    # Матрица перехода между состояниями 
    self.filter.F = np.eye(3)  # state transition matrix
    # Матрица управления, которая прикладывается к вектору управляющих воздействий
    self.filter.B = np.eye(3)

    # Скользящее среднее представляет собой среднее арифметическое значение наблюдений в определенном окне или периоде,
    # который "скользит" вдоль временного ряда. Скользящее среднее помогает сгладить краткосрочные колебания и шумы,
    # выявлять тренды и улавливать долгосрочные закономерности в данных.
    # Чтобы вычислить скользящее среднее, нужно определить размер окна (количество наблюдений, включенных в среднее) и,
    # начиная с первого значения во временном ряду, взять среднее арифметическое значение наблюдений в этом окне. Затем
    # окно сдвигается на одно наблюдение вперед, и процесс повторяется до тех пор, пока окно не достигнет конца
    # временного ряда.
    self._window_size = moving_window_filter_size
    self.moving_window_filter_x = MovingWindowFilter(
       window_size=self._window_size)
    self.moving_window_filter_y = MovingWindowFilter(
       window_size=self._window_size)
    self.moving_window_filter_z = MovingWindowFilter(
       window_size=self._window_size)
    self._estimated_velocity = np.zeros(3)
    # Устанавливаем последнюю временную метку
    self._last_timestamp = 0

  def reset(self):
    # Сбрасываем в состоянии инициализации
    self.filter.x = np.zeros(3)
    self.filter.P = np.eye(3) * self._initial_variance
    self.moving_window_filter_x = MovingWindowFilter(
       window_size=self._window_size)
    self.moving_window_filter_y = MovingWindowFilter(
       window_size=self._window_size)
    self.moving_window_filter_z = MovingWindowFilter(
       window_size=self._window_size)
    self._last_timestamp = 0

  def _compute_delta_time(self, current_time):
    if self._last_timestamp == 0.:
      # First timestamp received, return an estimated delta_time.
      delta_time_s = self.robot.time_step
    else:
      delta_time_s = current_time - self._last_timestamp
    self._last_timestamp = current_time
    return delta_time_s

  def update(self, current_time):
    """Propagate current state estimate with new accelerometer reading."""
    delta_time_s = self._compute_delta_time(current_time)
    sensor_acc = self.robot.GetBaseAcceleration()
    base_orientation = self.robot.GetBaseOrientation()
    rot_mat = self.robot.pybullet_client.getMatrixFromQuaternion(
        base_orientation)
    rot_mat = np.array(rot_mat).reshape((3, 3))
    calibrated_acc = rot_mat.dot(sensor_acc) + np.array([0., 0., -9.8])
    self.filter.predict(u=calibrated_acc * delta_time_s)

    # Correct estimation using contact legs
    observed_velocities = []
    foot_contact = self.robot.GetFootContacts()
    for leg_id in range(4):
      if foot_contact[leg_id]:
        jacobian = self.robot.ComputeJacobian(leg_id)
        # Only pick the jacobian related to joint motors
        joint_velocities = self.robot.GetMotorVelocities()[leg_id *
                                                           3:(leg_id + 1) * 3]
        leg_velocity_in_base_frame = jacobian.dot(joint_velocities)
        base_velocity_in_base_frame = -leg_velocity_in_base_frame[:3]
        observed_velocities.append(rot_mat.dot(base_velocity_in_base_frame))

    if observed_velocities:
      observed_velocities = np.mean(observed_velocities, axis=0)
      self.filter.update(observed_velocities)

    vel_x = self.moving_window_filter_x.calculate_average(self.filter.x[0])
    vel_y = self.moving_window_filter_y.calculate_average(self.filter.x[1])
    vel_z = self.moving_window_filter_z.calculate_average(self.filter.x[2])
    self._estimated_velocity = np.array([vel_x, vel_y, vel_z])

  @property
  def estimated_velocity(self):
    return self._estimated_velocity.copy()
