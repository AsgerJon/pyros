"""RosMaster subclasses the MasterProxy defined in rospy"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from roslaunch import Master

from morevistutils.fields import EnvField


class RosMaster(Master, ):
  """RosMaster subclasses the MasterProxy defined in rospy"""
  URI = EnvField('ROS_MASTER_URI')

  def __init__(self, *args, **kwargs) -> None:
    Master.__init__(self, Master.ROSMASTER, self.URI, None)

  def __bool__(self) -> bool:
    """Returns Falsy if not running else True"""
    return True if self.is_running() else False
