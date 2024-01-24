"""FlexSpace provides an abstract baseclass for namespace objects used by
metaclasses. Custom Metaclasses implementing the __prepare__ method in a
can subclass FlexSpace and instantiate it on the metaclass itself,
the name and bases of the class to be created and the keyword arguments.
The subclass of FlexSpace must implement the 'getNamespace' method such
that it returns the actual namespace to be used in class creation. This
allows custom metaclasses a great deal of flexibility in managing the
class body execution."""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from abc import abstractmethod

from vistutils.metas import AbstractNamespace, BaseNamespace


class FlexSpace(BaseNamespace):
  """Flexible namespace object"""

  def __init__(self, *args, **kwargs) -> None:
    BaseNamespace.__init__(self, *args, **kwargs)

  @abstractmethod
  def getNamespace(self, *args, **kwargs) -> dict:
    """Subclasses must implement this method. """
  