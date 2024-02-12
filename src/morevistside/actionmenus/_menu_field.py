"""MenuField is used by MenuBar to define its menus."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import json
import os
from typing import Any

from PySide6.QtGui import QIcon, QPixmap
from PySide6.QtWidgets import QMenu, QMainWindow, QWidget
from icecream import ic
from vistutils import getProjectRoot
from vistutils.fields import ClassField
from vistutils.waitaminute import typeMsg

from morevistutils import getIconsPath

ic.configureOutput(includeContext=True)


class MenuField(ClassField):
  """MenuField is used by MenuBar to define its menus."""

  def _menuFactory(self, instance: Any, owner: type) -> None:
    """Instantiates QMenu and populates it on the given instance"""
    mainWindow = self._getMainWindow()
    title = self._getTitle()
    icon = self._iconFactory()
    tooltip = self._getTooltip()
    menu = QMenu()

  def _iconFactory(self, ) -> QIcon:
    """Creates a QIcon based on the menu title."""
    root = getProjectRoot()
    fileName = '%s.png' % self._getTitle()
    iconDir = getIconsPath()
    iconPath = os.path.normpath(os.path.join(iconDir, fileName))
    pix = None
    try:
      pix = QPixmap(iconPath)
    except FileNotFoundError as fileNotFoundError:
      try:
        iconPath = os.path.normpath(os.path.join(iconDir, 'debug.png'))
        pix = QPixmap(iconPath)
      except FileNotFoundError as f2:
        raise f2 from fileNotFoundError
    if pix is None:
      raise NameError(self._getTitle())
    if isinstance(pix, QPixmap):
      return QIcon(pix)
    e = typeMsg('pix', pix, QPixmap)
    raise TypeError(e)

  def __init__(self, title: str, *args, **kwargs) -> None:
    self.__main_window__ = None
    ClassField.__init__(self, QMenu, creator=self._menuFactory)
    self.__menu_title__ = title
    self.__disk_data__ = json.loads()

  def __prepare_owner__(self, owner: type) -> type:
    """Implementation of parent method."""
    if issubclass(owner, QMainWindow):
      self.__main_window__ = owner
      return owner
    e = """Expected owner to be a subclass of QMainWindow"""
    raise TypeError

  def _getMainWindow(self) -> QWidget:
    """Getter-function for the main window class"""
    return self.__main_window__

  def _getTitle(self) -> str:
    """Getter-function for the title"""
    return self.__menu_title__

  def _getTooltip(self) -> str:
    """Tries to read from file or returns empty string"""
