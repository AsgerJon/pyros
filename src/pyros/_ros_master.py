"""RosMaster subclasses the MasterProxy defined in rospy"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from rospy import MasterProxy

from morevistutils.fields import EnvField


class RosMaster(MasterProxy, ):
  """RosMaster subclasses the MasterProxy defined in rospy"""
  URI = EnvField('ROS_MASTER_URI')

  def __init__(self, *args, **kwargs) -> None:
    MasterProxy.__init__(self, self.URI)
