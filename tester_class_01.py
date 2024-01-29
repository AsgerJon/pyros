"""TESTER class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from morevistutils.fields import TypedField, ListField


class Point:
  """3d point"""

  x = TypedField(int, )
  y = TypedField(int, )
  z = TypedField(int, )

  vals = ListField(int)

  def __str__(self, ) -> str:
    """String representation"""
    clsName = self.__class__.__qualname__
    return '%s(%d, %d, %d)' % (clsName, self.x, self.y, self.z)

  def __iter__(self) -> Point:
    self.vals = [self.x, self.y, self.z]
    return self

  def __next__(self, ) -> int:
    if self.vals:
      return self.vals.pop(0)
    raise StopIteration
