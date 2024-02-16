"""The projectPaths locates various directories."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys
from icecream import ic

ic.configureOutput(includeContext=True)


def _getModuleRoot(moduleName: str) -> str:
  """Getter-function for directory of named module"""
  module_ = sys.modules.get(moduleName, None)
  if module_ is None:
    raise NameError(moduleName)
  fid = os.path.normpath(os.path.dirname(os.path.abspath(module_.__file__)))
  if os.path.exists(fid):
    if os.path.isdir(fid):
      return fid
    raise NotADirectoryError(fid)
  raise ModuleNotFoundError(fid)


def getProjectRoot() -> str:
  """Getter-function for """
  return _getModuleRoot('__main__')


def getIconsPath() -> str:
  """Getter-function for directory of all the icons."""
  root = getProjectRoot()
  return os.path.join(root, 'src', 'morevistside', 'actionmenus', 'icons')


def getMenusPath() -> str:
  """Getter-function for directory of all the menus and actions."""
  return _getModuleRoot('menus')
