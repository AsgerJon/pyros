"""TESTER"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtWidgets import QWidget

from morevistside.widgets import WidgetField

type(QWidget)


class Pog(type(QWidget)):
  """The tale of the stolen metaclass!"""

  @classmethod
  def __prepare__(mcls, name: str, bases: tuple, **kwargs) -> dict:
    return super().__prepare__()

  def __str__(cls) -> str:
    """String representation"""
    out = []
    for (key, val) in cls.__class__.__dict__.items():
      out.append('%s: %s' % (key, type(val)))
    return '\n'.join(out)


class Champ(metaclass=Pog):
  """LMAO"""

  def __init__(self, *args, **kwargs) -> None:
    print(*args, **kwargs)

  def __str__(self) -> str:
    """String representation"""
    out = []
    for (key, val) in self.__class__.__dict__.items():
      out.append('%s: %s' % (key, type(val)))
    return '\n'.join(out)
