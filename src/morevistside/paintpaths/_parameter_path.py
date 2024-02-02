"""ParameterPath provides a subclass of QPainterPath. Instances hereof all
need a canvas parameter which denotes a rectangle. Subclasses are free to
have any number of parameters in addition to the canvas. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtGui import QPainterPath

from morevistside.paintpaths import CanvasPath


class ParameterPath(QPainterPath):
  """ParameterPath provides a subclass of QPainterPath. Instances hereof all
  need a canvas parameter which denotes a rectangle. Subclasses are free to
  have any number of parameters in addition to the canvas. """

  @staticmethod
  def _parse(*args, ) -> tuple[int, int]:
    """Parses positional arguments for a width and height. """
    raise NotImplementedError('use kwargs lol')

  def __init__(self, *args, **kwargs) -> None:
    QPainterPath.__init__(self, )
    width = kwargs.get('width', 1)
    height = kwargs.get('height', 1)
    self.initialize(*args)

  @abstractmethod
  def initialize(self, canvas: CanvasPath, *parameters) -> None:
    """Subclasses must implement exactly how the instance initializes
    itself to the given parameters. """
