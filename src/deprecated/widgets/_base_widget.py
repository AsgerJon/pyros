"""BaseWidget provides a baseclass for widgets requiring size control. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from PySide6.QtCore import QSize
from PySide6.QtWidgets import QWidget

from morevistside import parseParent
from morevistutils import maybeTypes


class BaseWidget(QWidget):
  """BaseWidget provides a baseclass for widgets requiring size control. """

  __default_min_width__ = 32
  __default_min_height__ = 32
  __default_max_width__ = 1024
  __default_max_height__ = 1024

  def __init__(self, *args, **kwargs) -> None:
    intArgs = maybeTypes(int, *args)
    if len(intArgs) < 2:
      minSize = QSize(self.__default_min_width__,
                      self.__default_min_height__)
    else:
      minSize = QSize(*intArgs[:2])
    if len(intArgs) < 4:
      if len(intArgs) > 1:
        maxSize = minSize
      else:
        maxSize = QSize(self.__default_max_width__,
                        self.__default_max_height__)
    else:
      maxSize = QSize(*intArgs[2:4])
    parent = parseParent(*args)
    QWidget.__init__(self, parent)
    self.setMinimumSize(minSize)
    self.setMaximumSize(maxSize)
