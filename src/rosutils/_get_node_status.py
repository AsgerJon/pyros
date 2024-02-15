"""The getNodeStatus function tests if given node is running."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from rosnode import get_node_names


def getNodeStatus(nodeName: str) -> bool:
  """The getNodeStatus function tests if given node is running."""
  for name in get_node_names():
    if name == nodeName:
      return True
  else:
    return False
