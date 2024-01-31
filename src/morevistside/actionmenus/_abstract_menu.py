"""The AbstractMenu class subclasses QMenu providing the menu class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from PySide6.QtGui import QAction
from PySide6.QtWidgets import QMenu
from morevistside import parseParent


class AbstractMenu(QMenu):
  """The AbstractMenu class subclasses QMenu providing the menu class."""

  def __init__(self, *args, **kwargs) -> None:
    parent = parseParent(*args)
    QMenu.__init__(self, parent)

  @abstractmethod
  def getActions(self) -> list[QAction]:
    """Getter-function for the ordered list of actions in this menu."""
