"""The maybe function and related functions provide None-aware type
filtering."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any


def maybe(*args, ) -> Any:
  """Returns the first positional argument that is different from None"""
  for arg in args:
    if arg is not None:
      return arg


def maybeType(*args, ) -> Any:
  """Returns the first positional argument belonging to given type"""
