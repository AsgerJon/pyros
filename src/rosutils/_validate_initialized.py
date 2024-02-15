"""The validateInitialized function validates that ROS is initialized."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from rosgraph import is_master_online

from rosutils import initializeRosNode, getNodeStatus


def validateInitialized(nodeName: str, **kwargs) -> bool:
  """Initialize the ROS environment."""
  if is_master_online():
    if getNodeStatus(nodeName):
      return True
    if kwargs.get('_recursion', False):
      raise RecursionError
    initializeRosNode(nodeName, )
    return validateInitialized(nodeName, _recursion=True)

  raise ConnectionError
