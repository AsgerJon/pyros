"""TimeTest provides testing for computation speed different array
classes. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

import numba as nb
import numpy as np
from vistutils import stringList, searchKey, maybe
from vistutils.fields import Field

from morevistutils import getEnvVar

ArrayLike = np.array | list

bufferDefault = getEnvVar('BUFFER_DEFAULT')
sampleDefault = getEnvVar('SAMPLE_DEFAULT')


class TimeTest:
  """TimeTest provides testing for computation speed different array
  classes. """

  testClass = Field()

  def __init__(self, testClass: ArrayLike, **kwargs) -> None:
    self.__test_class__ = testClass

    bufferKeys = stringList("""buffer, bufferCapacity, temp, capacity""")
    bufferTypes = (int,)
    sampleKeys = stringList("""sampleSize, sample, test, full""")
    sampleTypes = (int,)
    bufferCapacityKwarg = searchKey(*bufferKeys, *bufferTypes, **kwargs)
    sampleSizeKwarg = searchKey(*sampleKeys, *sampleTypes, **kwargs)
    self.__buffer_capacity__ = maybe(bufferCapacityKwarg, bufferDefault)
    self.__sample_size__ = maybe(sampleSizeKwarg, sampleDefault)
    self.__data_type_numpy__ = np.float64
    self.__data_type_python__ = float
    self.__data_type_numba__ = nb.float64

  def getTestClass(self) -> type:
    """Getter-function for test class"""
    return self.__test_class__

  def instantiateTestClassCode(self) -> str:
    """Generates code instantiating contained class"""
    genCode = """<className>(buffer=<bufferCapacity>, )"""
    return genCode % (self.getTestClass(),)

  # def generateSetupCode(self) -> str:
  #   """Generates setup code"""
  #   genCode = """from __main__ import <testClass><br>
  #                data = <testClass>(%d, dtype=<dataType>"""
  #
  #   genCode = genCode.replace('<testClass>', testClass)
