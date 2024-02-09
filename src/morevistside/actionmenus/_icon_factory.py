"""The getIcon and getPixmap function returns a QIcon and QPixmap
representation of the named icon."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Optional

from PySide6.QtGui import QPixmap, QIcon
from icecream import ic
from vistutils import getProjectRoot
from vistutils.waitaminute import typeMsg

ic.configureOutput(includeContext=True)


def _fixCase(name: str) -> str:
  """Returns the name with the capitalization fixed"""
  out = []
  for char in name:
    if char.isupper():
      out.append('_%s' % char.lower())
    else:
      out.append(char)
  return ''.join(out)


def getFids() -> dict[str, str]:
  """Getter-function for name: icon file"""
  root = getProjectRoot()
  there = os.path.join(root, 'src', 'morevistside', 'actionmenus', 'icons')
  data = {}
  for item in os.listdir(there):
    base = os.path.basename(item)
    name = os.path.splitext(base)[0]
    fid = os.path.normpath(os.path.join(there, item))
    data[name] = fid
  return data


def _getFid(name, ) -> Optional[str]:
  """Looks up name in icon dir"""
  data = getFids()
  keys = [name, name.lower()]
  for key in keys:
    if key in data:
      return data.get(key)
  raise NameError(name)


def getPix(name: str, **kwargs) -> QPixmap:
  """Returns a QPixmap representation of the named icon"""
  fid = _getFid(name)
  return QPixmap(fid)


def getIcon(name: str, **kwargs) -> Optional[QIcon]:
  """Returns the QPixmap representation of the named icon"""
  return QIcon(getPix(_fixCase(name)))
