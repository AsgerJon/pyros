"""The getEnvFiles returns the path to .env file or the .env.example if
.env is not found. If neither is found, the method raises
FileNotFoundError."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from icecream import ic
from vistutils import getProjectRoot, monoSpace

ic.configureOutput(includeContext=True)


def getEnvFiles() -> str:
  """The getEnvFiles returns the path to .env and .env.example"""
  root = getProjectRoot()
  ic(root)
  fids = [os.path.join(root, name) for name in ['.env', '.env.example']]
  for fid in fids:
    if os.path.exists(fid):
      if os.path.isfile(fid):
        return fid
      e = """Expected file, but found directory at: '%s' """
      raise IsADirectoryError(monoSpace(e % os.path.abspath(fid)))
  raise FileNotFoundError(fids[0]) from FileNotFoundError(fids[1])
