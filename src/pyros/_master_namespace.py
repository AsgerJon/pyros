"""MasterNamespace provides the namespace class used by the MetaMaster
metaclass to create the RosMaster class."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from utils.metas import AbstractNamespace


class MasterNamespace(AbstractNamespace):
  """MasterNamespace provides the namespace class used by the MetaMaster
  metaclass to create the RosMaster class."""

  def __init__(self, *args, **kwargs) -> None:
    AbstractNamespace.__init__(self, *args, **kwargs)
