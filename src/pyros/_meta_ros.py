"""MetaRos provides a common metaclass shared by classes wrapping the ros
framework. This metaclass is certain to invoke its __prepare__ method
before any of its derived classes can be created. Thus, it allows for a
convenient hook ensuring initialization."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Never

from rospy import MasterProxy
from vistutils import applyEnv
from vistutils.fields import Field
from vistutils.metas import AbstractMetaclass, Bases
from vistutils.metas import BaseNamespace as BNS


class MetaRos(AbstractMetaclass):
  """MetaRos provides a common metaclass shared by classes wrapping the ros
  framework. This metaclass is certain to invoke its __prepare__ method
  before any of its derived classes can be created. Thus, it allows for a
  convenient hook ensuring initialization."""
  __ros_master_obj__ = None
  __ros_master_uri__ = None
  rosMaster = Field()
  uri = Field()

  @classmethod
  def _createRosMaster(mcls, ) -> None:
    """Creator-function for RosMaster object"""
    mcls.rosMaster = MasterProxy(mcls.uri)

  @classmethod
  @rosMaster.GET
  def getRosMaster(mcls, **kwargs) -> MasterProxy:
    """Getter-function for ros master object"""
    if mcls.__ros_master_obj__ is None:
      if kwargs.get('_recursion', False):
        raise RecursionError
      mcls._createRosMaster()
      return mcls.getRosMaster(_recursion=True)
    return mcls.__ros_master_obj__

  @classmethod
  @rosMaster.SET
  def _setRosMaster(mcls, *_) -> Never:
    """Illegal accessor function"""
    raise TypeError

  @classmethod
  @rosMaster.DEL
  def _delRosMaster(mcls, *_) -> Never:
    """Illegal accessor function"""
    raise TypeError

  @classmethod
  @uri.GET
  def _getRosMasterURI(mcls) -> str:
    """Getter-function for ros master uri"""
    return os.environ.get('ROS_MASTER_URI')

  @classmethod
  def __prepare__(mcls, name: str, bases: Bases, **kwargs) -> BNS:
    applyEnv()
    namespace = BNS(name, bases, **kwargs)
    namespace.setMetaclass(mcls)
    return namespace
