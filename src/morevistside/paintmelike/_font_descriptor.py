"""FontDescriptor provides a descriptor class for font"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any
from warnings import warn

from PySide6.QtGui import QFont

from morevistside.paintmelike import AbstractStencil
from morevistutils.waitaminute import WaitForIt


class FontDescriptor(AbstractStencil):
  """FontDescriptor provides a descriptor class for font"""

  __default_family__ = 'Courier'
  __default_size__ = 18

  def __init__(self, *args, **kwargs) -> None:
    AbstractStencil.__init__(self, *args, **kwargs)
    self._font = None
    if args:
      if isinstance(args[0], QFont):
        self._font = args[0]
      if self._font is None:
        self._font = QFont()
        for arg in args:
          if isinstance(arg, str):
            self._font.setFamily(arg)
          if isinstance(arg, int):
            self._font.setPointSize(arg)
    if self._font is None:
      self._font = QFont()
      self._font.setFamily('Courier')
      self._font.setPointSize(18)
    self.__default_font__ = self._font

  def __get__(self, instance: Any, owner: type) -> QFont:
    """Getter-function for the font"""
    warn(WaitForIt('Font descriptor not fully implemented'))
    return self._font
