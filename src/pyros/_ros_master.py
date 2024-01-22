"""RosMaster encapsulates the central service responsible for looking up
nodes, topics and services in a particular ROS network."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os

from pyros import MetaMaster, CustomField
from icecream import ic
from vistutils.cli import CLI
from vistutils.fields import apply

ic.configureOutput(includeContext=True)


class RosMaster(metaclass=MetaMaster):
  """Encapsulation of running ROS Master"""

  machine = CustomField()
  machineCLI = CLI('rosnode machine')
  nodes = CustomField('rosnode list')
  nodesCLI = CLI('rosnode list')
  URI = CustomField('')
  IP = CustomField('ROS_IP')

  @apply('machine', 'get')
  def _getRosMachine(self) -> str:
    """Getter-function for ros machine"""
    res = self.machineCLI()
    if res.err:
      raise RuntimeError(res.err)
    return res.out

  @apply('nodes', 'get')
  def _getRosNodes(self) -> list[str]:
    """Getter-function for ros nodes"""
    res = self.nodesCLI()
    if res.err:
      raise RuntimeError(res.err)
    lines = res.out.split('\n')
    return [line.strip() for line in lines]

  @apply('URI', 'get')
  def _getURI(self) -> str:
    """Getter-function for ROS_MASTER_URI"""
    return os.environ['ROS_MASTER_URI']

  @apply('IP', 'get')
  def _getIP(self) -> str:
    """Getter-function for ROS_IP"""
    return os.environ['ROS_IP']

  def __init__(self, *args, **kwargs) -> None:
    self.__iter_contents__ = []

  def _resetIterContents(self) -> None:
    """Resets the iter contents to nodes returned by the nodes field"""
    self.__iter_contents__ = self._getRosNodes()

  def __iter__(self) -> RosMaster:
    """Implementation of iteration"""
    self._resetIterContents()
    return self

  def __next__(self, ) -> str:
    """Implementation of iteration"""
    try:
      return self.__iter_contents__.pop(0)
    except IndexError:
      raise StopIteration

  def __contains__(self, node: str) -> bool:
    """Defines if node is defined"""
    for item in self:
      if item == node:
        return True
    return True
