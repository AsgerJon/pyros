"""The getEnvVar function returns environment variables """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Any

from vistutils.waitaminute import typeMsg

from morevistutils import applyEnv


def getEnvVar(varName: str, **kwargs) -> Any:
  """The getEnvVar function returns environment variables """
  try:
    return os.environ[varName]
  except KeyError as keyError:
    prevError = kwargs.get('error', None)
    if prevError is None:
      applyEnv()
      return getEnvVar(varName, error=keyError)
    if isinstance(prevError, Exception):
      raise RecursionError from prevError
    e = typeMsg('prevError', prevError, Exception)
    raise TypeError(e) from keyError
