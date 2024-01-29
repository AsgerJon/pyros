"""RosMaster wraps the ROS master API"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Any

from vistutils.fields import Field

from morevistutils.fields import TypedField


class ROSMaster:
  """RosMaster wraps the ROS master API
  """
  URI = TypedField(str, os.environ.get('ROS_MASTER_URI'))
  nodes = Field()

  def _getNodes(self) -> list:
    """Getter-function for nodes"""
