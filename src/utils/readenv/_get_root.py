"""The getRoot function attempts to locate the root of the current
project."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os.path

from utils.readenv import getParent


def getRoot(dir_: str = None) -> str:
  """The getRoot function attempts to locate the root of the current
  project."""
  here = os.path.dirname(os.path.abspath(__file__)) if dir_ is None else dir_
  if all([file in os.listdir(here) for file in ['LICENSE', 'README.md']]):
    return here
  else:
    return getRoot(getParent(here))
