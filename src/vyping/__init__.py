"""The vyping module exposes python version dependant objects from the
typing and typing extension modules. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import sys

if sys.version_info.minor < 9:
  pass

elif sys.version_info.minor < 10:
  from typing import NoReturn as Never

elif sys.version_info.minor < 11:
  from typing import Never

elif sys.version_info.minor < 12:
  pass

elif sys.version_info.minor < 13:
  pass

__all__ = ['Never', ]
