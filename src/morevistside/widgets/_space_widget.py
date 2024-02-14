"""SpaceWidget provides a widget with space awareness"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtCore import QSize
from PySide6.QtWidgets import QWidget, QSizePolicy
from vistutils import stringList, maybe
from vistutils.fields import Field

from morevistside import parseParent


@dataclass
class SizeRange:
  """Immutable dataclass for size range"""
  minWidth: int
  maxWidth: int
  minHeight: int
  maxHeight: int

  minSize = Field()
  maxSize = Field()

  @minSize.GET
  def getMinSize(self) -> QSize:
    """Getter-function for minimum size"""
    return QSize(self.minWidth, self.minHeight)

  @maxSize.GET
  def getMaxSize(self) -> QSize:
    """Getter-function for maximum size"""
    return QSize(self.maxWidth, self.maxHeight)


class SpaceWidget(QWidget):
  """SpaceWidget provides a widget with space awareness"""

  @classmethod
  def _parseSizes(cls, *args, **kwargs) -> SizeRange:
    """Parses arguments to find size range"""
    minWidthKeys = stringList("""minWidth, minW, minimumWidth, minimumW""")
    maxWidthKeys = stringList("""maxWidth, maxW, maximumWidth, maximumW""")
    minHeightKeys = stringList("""minHeight, minH, minimumHeight, 
    minimumH""")
    maxHeightKeys = stringList("""maxHeight, maxH, maximumHeight, 
    maximumH""")
    allKeys = [minWidthKeys, maxWidthKeys, minHeightKeys, maxHeightKeys]
    allKwargs = []
    allArgs = []
    for keys in allKeys:
      for key in keys:
        if key in kwargs:
          val = kwargs.get(key)
          if isinstance(val, int):
            allKwargs.append(val)
          else:
            allKwargs.append(None)
    minWKwarg, maxWKwarg, minHKwarg, maxHKwarg = allKwargs[:4]
    intArgs = [arg for arg in args if isinstance(arg, int)]
    for arg in [*intArgs, *[None for _ in range(4)][:4]]:
      if isinstance(arg, int):
        allArgs.append(arg)
      else:
        allArgs.append(None)
    minWArg, maxWArg, minHArg, maxHArg = allArgs[:4]
    allDefaults = [64, 256, 64, 256]
    minWDefault, maxWDefault, minHDefault, maxHDefault = allDefaults[:4]
    minWidth = maybe(minWKwarg, minWArg, minWDefault)
    maxWidth = maybe(maxWKwarg, maxWArg, maxWDefault)
    minHeight = maybe(minHKwarg, minHArg, minHDefault)
    maxHeight = maybe(maxHKwarg, maxHArg, maxHDefault)
    return SizeRange(minWidth, maxWidth, minHeight, maxHeight)

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QWidget.__init__(self, parent, )
    sizes = self._parseSizes(*args, **kwargs)
    self.setMinimumSize(sizes.minSize)
    self.setMaximumSize(sizes.maxSize)
    expanding = QSizePolicy.Policy.MinimumExpanding
    contracting = QSizePolicy.Policy.Maximum
    self.setSizePolicy(QSizePolicy(contracting, expanding))
