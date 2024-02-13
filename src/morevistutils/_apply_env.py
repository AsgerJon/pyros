"""The applyEnv function loads the environment files.json and applies their
data to the current environment. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
import sys

from morevistutils import loadEnv, getProjectRoot


def applyEnv() -> None:
  """The applyEnv function loads the environment files.json and applies their
  data to the current environment. """
  root = getProjectRoot()
  srcPath = os.path.join(root, 'src')
  sys.path.append(os.path.normpath(srcPath))

  data = loadEnv()
  for (key, val) in data.items():
    os.environ[key] = val
