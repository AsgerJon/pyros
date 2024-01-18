"""Sets by membership"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod
from typing import Callable


class AbstractElement:
  """An element"""

  @abstractmethod
  def view(self) -> int:
    """An elemental implementation must assign an integer to each element.
    Then if element A and element B have the same view, they are the same
    element."""

  def __eq__(self, other: AbstractElement) -> bool:
    """Defines equality"""
    if self.view() == other.view():
      return True
    return False


class MetaSet(type):
  """Metaclass for sets"""

  def __add__(cls, other: Set) -> Set:
    """Defines the union of two sets"""
    return Set(lambda x: x in cls or x in other)

  def __mul__(self, other: Set) -> Set:
    """Defines the intersection of two sets"""
    return Set(lambda x: x in self and x in other)

  def __sub__(cls, other: Set) -> Set:
    """Defines the difference of two sets"""
    return Set(lambda x: x in cls and x not in other)

  def __neg__(self) -> Set:
    """Defines the complement of a set"""
    return Set(lambda x: x not in self)


class Set(metaclass=MetaSet):
  """Sets by membership"""

  def __init__(self, membership: Callable) -> None:
    """Initializes the set"""
    self.__membership_function__ = membership

  def __contains__(self, view: int) -> bool:
    """Defines whether the element is in the set"""


class UniversalSet(Set):
  """The universal set"""

  def __contains__(self, element: AbstractElement) -> bool:
    """Defines whether the element is in the set"""
    return True


class EmptySet(Set):
  """The empty set"""

  def __contains__(self, element: AbstractElement) -> bool:
    """Defines whether the element is in the set"""
    return False


class Number(AbstractElement):
  """A number"""

  numbers = []

  def __new__(cls) -> Number:
    number = super().__new__(cls)
    cls.numbers.append(number)
    return number

  def view(self) -> int:
    """An elemental implementation must assign an integer to each element.
    Then if element A and element B have the same view, they are the same
    element."""
    out = 0
    for e in self.__class__.numbers:
      if e is self:
        return out
      else:
        out += 1
    raise Exception('Elemental implementation error')
