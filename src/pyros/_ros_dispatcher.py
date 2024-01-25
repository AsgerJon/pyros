"""The RosDispatcher decorates a class such that a temp file containing a
copy of the code becomes available. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import os
from typing import Any

from icecream import ic

from dispatcher import AbstractDispatcher
from morevistutils import getProjectRoot

ic.configureOutput(includeContext=True)


class RosDispatcher(AbstractDispatcher):
  """The RosDispatcher decorates a class such that a temp file containing a
  copy of the code becomes available. """

  __fallback_temp_dir__ = getProjectRoot()

  def __init__(self, *args, **kwargs) -> None:
    AbstractDispatcher.__init__(self, *args, **kwargs)
    self.__if_name_equal_main__ = []

  @classmethod
  def _getTempDir(cls) -> str:
    """Getter-function for the temporary directory. """
    return os.environ.get('ROS_TEMP', cls.__fallback_temp_dir__)

  def _getTempName(self) -> str:
    """Getter-function for the temp file name"""
    c = 0
    ic(self._getInnerObject())
    ic(self.__class__.__qualname__)
    fmtSpec = 'ROS_%s_%%d.py' % self._getInnerObject().__qualname__
    filePath = lambda n: os.path.join(self._getTempDir(), fmtSpec % n)
    while os.path.exists(filePath(c)):
      c += 1
    return os.path.basename(filePath(c - 1))

  @classmethod
  def getHashbang(cls) -> str:
    """Getter-function for the hashbang to be placed at the top of the
    temp file. This defaults to: #!/usr/bin/env python3"""
    return '#!/opt/ros/noetic/bin/python3'

  def _getMainCode(self) -> list[str]:
    """Getter-function for the main code to be contained in the if
    statement: <if __name__ == '__main__':>"""
    code = """
    topic = %s()
    topic.begin()
    """
    return [line.strip() for line in code.split('\n') if line]

  def terminal(self, *args, **kwargs) -> list[str | list[str]]:
    """Subclasses must implement this method to define the terminal
    commands that will invoke the dispatch script. """
    dirPath = self._getTempDir()
    tempName = self._getTempName()
    return ['cd %s' % dirPath, 'python3 %s' % tempName]

  @classmethod
  def getImports(cls) -> list[str]:
    """Extra explicit imports"""
    return ['import rospy', 'from rospy import Publisher, Rate, is_shutdown']

  @classmethod
  def getTempComment(cls) -> list[str]:
    """Getter-function for str"""
    return ['test']
