"""TypedField provides a strongly typed descriptor class"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable
import sys
from vistutils.fields import AbstractField

from morevistutils.fields import DataType

if sys.version_info.minor < 11:
  from typing import NoReturn as Never
else:
  from typing import Never


class TypedField(AbstractField):
  """TypedField provides a strongly typed descriptor class"""

  @classmethod
  def _getMemberListName(cls) -> str:
    """Getter-function for name of instance list"""
    return '__%s_instances__' % cls.__qualname__

  def __init__(self, *args, **kwargs) -> None:
    AbstractField.__init__(self, *args, **kwargs)
    typeDict = DataType(*args, )
    self.__field_type__ = typeDict.type
    self.__field_default__ = typeDict.defVal

  def _getPrivateName(self, ) -> str:
    """Getter-function for the private name of this field"""
    return '_%s' % self.__field_name__

  def _getFieldType(self) -> type:
    """Getter-function for field type"""
    if isinstance(self.__field_type__, type):
      return self.__field_type__
    raise TypeError

  def _getDefaultValue(self) -> Any:
    """Getter-function for default value"""
    fieldType = self._getFieldType()
    if isinstance(self.__field_default__, fieldType):
      return self.__field_default__
    raise TypeError

  @classmethod
  def _extraInitFactory(cls, owner: type) -> Callable:
    """Creates the addendum to the __init__"""

    def __extra_init__(this: Any, *args, **kwargs) -> None:
      """Additional logic for init"""
      fields = getattr(owner, cls._getMemberListName(), None)
      if fields is None:
        raise ValueError
      args = [*args, ]
      while len(fields) > len(args):
        args.append(None)
      for (field, arg) in zip(fields, args):
        if isinstance(field, TypedField):
          type_ = field._getFieldType()
          defVal = field._getDefaultValue()
          pvtName = field._getPrivateName()
        else:
          raise TypeError
        if arg is None:
          arg = defVal
        if isinstance(type_, type):
          if isinstance(arg, type_):
            setattr(this, pvtName, arg)
          else:
            raise TypeError
        else:
          raise TypeError

    return __extra_init__

  def __prepare_owner__(self, owner: type) -> type:
    """Implementation of abstract method"""
    memberListName = self._getMemberListName()
    if hasattr(owner, memberListName):
      existingFields = getattr(owner, memberListName)
      setattr(owner, memberListName, (*existingFields, self))
      return owner

    baseInit = getattr(owner, '__init__')
    if baseInit is object.__init__:
      oldInit = lambda *__, **_: None
    elif callable(baseInit):
      oldInit = baseInit
    else:
      raise TypeError

    __extra_init__ = self.__class__._extraInitFactory(owner)

    # print(__extra_init__)
    # print('self: ', getattr(__extra_init__, '__self__', 'no self'))

    def newInit(this, *args, **kwargs) -> None:
      """Replacement __init__"""
      __extra_init__(this, *args, **kwargs)
      oldInit(this, *args, **kwargs)

    setattr(owner, '__init__', newInit)
    clsName = self.__class__.__qualname__
    setattr(owner, self._getMemberListName(), [self, ])
    return owner

  def __get__(self, instance: Any, owner: type) -> Any:
    """Getter-function implementation"""
    pvtName = self._getPrivateName()
    defVal = self._getDefaultValue()
    if instance is None:
      return self.__get__(owner, owner)
    if hasattr(instance, pvtName):
      return getattr(instance, pvtName)
    setattr(instance, pvtName, defVal)
    return self.__get__(instance, owner)

  def __set__(self, instance: Any, value: Any) -> None:
    """Setter-function implementation"""
    pvtName = self._getPrivateName()
    fieldType = self._getFieldType()
    if isinstance(value, fieldType):
      return setattr(instance, pvtName, value)
    raise TypeError

  def __delete__(self, instance: Any) -> Never:
    """Illegal deleter function"""
    raise TypeError
