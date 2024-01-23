"""The QuickField class is a descriptor for use with the Acc class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any

from vistutils.fields import AbstractField


class QuickField(AbstractField):
  """The QuickField class is a descriptor for use with the Acc class"""

  def __prepare_owner__(self, owner: type) -> type:
    return owner
