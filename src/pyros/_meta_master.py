"""MetaMaster provides a metaclass specifically for the creation of teh
RosMaster class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen

from __future__ import annotations

import os
from subprocess import run, PIPE
from subprocess import CompletedProcess as Res
from typing import Any

from vistutils import applyEnv
from vistutils.metas import Bases, Namespace, BaseNamespace


class MetaMaster(type):
  """MetaMaster provides a metaclass specifically for the creation of
  the RosMaster class"""

  @staticmethod
  def _validate() -> Res:
    """Validates that ROS Master is reachable"""
    return run('rosnode machine', shell=True, stdout=PIPE, stderr=PIPE)

  @staticmethod
  def getReady() -> list[dict[str, str]]:
    """Initialization of services required for class creations. """
    updates = applyEnv(returnVal=True)
    if isinstance(updates, list):
      return updates

  @classmethod
  def __prepare__(mcls, name: str, bases: Bases, **kwargs) -> Namespace:
    """Before creating the namespace object, the getReady method is
    invoked. This method must only return if it has completed
    initialization without encountering errors."""
    nameSpace = BaseNamespace(name, bases, **kwargs)
    updates = mcls.getReady()
    for update in updates:
      nameSpace |= update
    print(os.environ['ROS_MASTER_URI'])
    res = mcls._validate()
    if res.stderr:
      raise RuntimeError(res.stderr.decode('utf-8'))
    nameSpace['__ros_machine__'] = res.stdout.strip()
    print(res)
    return nameSpace

  def __new__(mcls, name: str, bases: Bases, nameSpace: Namespace,
              **kwargs) -> type:
    cls = type.__new__(mcls, name, bases, nameSpace, **kwargs)
    singletonInstance = cls()
    setattr(singletonInstance, '__metaclass__', mcls)
    setattr(singletonInstance, '__class__', cls)
    setattr(cls, '__singleton_instance__', singletonInstance)
    return cls

  def __call__(cls, *args, **kwargs) -> Any:
    """Checks if singleton object exists. If so returns it or allows it to
    be created and then returned."""
    if hasattr(cls, '__singleton_instance__', ):
      return getattr(cls, '__singleton_instance__')
    singletonInstance = type.__call__(cls, *args, **kwargs)
    setattr(cls, '__singleton_instance__', singletonInstance)
    return singletonInstance
