"""Moving window filter to smooth out sensor readings."""
# Расчет суммы и среднего окна скользящей средней.

import collections

class MovingWindowFilter(object):
  """A stable O(1) moving filter for incoming data streams.

  We implement the Neumaier's algorithm to calculate the moving window average,
  which is numerically stable.
  Мы реализуем алгоритм Ноймайера для расчета скользящей средней.
  Алгоритм Ноймайер значительно уменьшает числовую ошибку в итоговой сумме, полученной путем сложения последовательности
  чисел с плавающей точкой конечной точности, по сравнению с очевидным подходом. Это делается путем сохранения отдельной
  текущей компенсации (переменной для накопления небольших ошибок), в действительности расширяя точность суммы
  на точность переменной компенсации.
  """

  def __init__(self, window_size: int):
    """Initializes the class.

    Args:
      window_size: размер окна.
    """
    assert window_size > 0
    self._window_size = window_size
    # Класс collections.deque() это обобщение стеков и очередей и представляет собой двустороннюю очередь.
    self._value_deque = collections.deque(maxlen=window_size)
    # Сумма значений попадающая в окно.
    self._sum = 0
    # Расчетное значение - компенсация потери числовой точности при расчете вещественных чисел
    self._correction = 0

  def _neumaier_sum(self, value: float):
    """Суммарное компенсирование скользящей средней используя алгоритм Neumaier.

    Для дополнительной информации посетите:
    https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements

    Args:
      value: Новое значение будет добавляться в окно.
    """

    new_sum = self._sum + value
    if abs(self._sum) >= abs(value):
      # Если self._sum больше, младшие цифры значения теряются.
      self._correction += (self._sum - new_sum) + value
    else:
      # Младшие цифры суммы теряются
      self._correction += (value - new_sum) + self._sum

    self._sum = new_sum

  def calculate_average(self, new_value: float) -> float:
    """Двусторонняя очередь deque() поддерживает поточно-ориентированные, эффективные по памяти операции добавления и
    извлечения элементов последовательности с любой стороны с примерно одинаковой производительностью O(1)
    в любом направлении.

    Args:
      new_value: The new value to enter the moving window.

    Returns:
      Среднее значение окна.

    """
    deque_len = len(self._value_deque)
    if deque_len < self._value_deque.maxlen:
      pass
    else:
      # Крайнее левое значение необходимо вычесть из скользящей суммы.
      self._neumaier_sum(-self._value_deque[0])

    self._neumaier_sum(new_value)
    self._value_deque.append(new_value)

    return (self._sum + self._correction) / self._window_size
