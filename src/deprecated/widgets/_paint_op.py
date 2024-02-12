"""PaintOp provides a class description for a paint operation. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtGui import QPainter, QPaintEvent


class PaintOp:
  """PaintOp provides a class description for a paint operation. """

  def __init__(self, owner: type) -> None:
    self._owner = owner
    self._painter = None
    self._event = None

  @abstractmethod
  def _setupPainter(self, ) -> None:
    """This abstract method defines what preparations must be made to the
    painter. The painter is at private name '_painter'."""

  @abstractmethod
  def _apply(self, ) -> None:
    """This method defines what must actually happen during the paint
    event."""

  def __call__(self, painter: QPainter, event: QPaintEvent) -> None:
    """Applies the paint operation"""
    self._painter = painter
    self._event = event
    self._setupPainter()
    self._apply()
