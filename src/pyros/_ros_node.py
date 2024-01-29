"""RosNode encapsulates the logic relating to nodes in the ROS framework."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from rospy import init_node

from morevistutils.fields import TypedField
from pyros import MetaRos


class RosNode(metaclass=MetaRos):
  """RosNode encapsulates the logic relating to nodes in the ROS
  framework."""
  nodeName = TypedField(str, 'Test')

  def __init__(self, *args, **kwargs) -> None:
    pass
