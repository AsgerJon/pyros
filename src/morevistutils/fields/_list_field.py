"""ListField is a subclass of TypedField. The __get__ returns a list,
possibly empty, of object belonging to the type indicated. The __set__
method appends the value instead of having it replace the existing value.
The __delete__ empties the dictionary. """
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations

from typing import Any, Callable

from morevistutils.fields import TypedField, DataType


class ListField(TypedField):
  """ListField is a subclass of TypedField. The __get__ returns a list,
  possibly empty, of object belonging to the type indicated. The __set__
  method appends the value instead of having it replace the existing value.
  The __delete__ empties the dictionary. """

  __instances__ = '__list_field__'

  @classmethod
  def _getInstancesName(cls) -> str:
    """Getter-function for name of instance list"""
    return cls.__instances__

  @classmethod
  def _extraInitFactory(cls, owner: type) -> Callable:
    """Creates the addendum to the __init__"""

    def __extra_init__(this: Any, *args, **kwargs) -> None:
      """Additional logic for init"""

      listFields = getattr(owner, cls._getInstancesName(), None)
      if listFields is None:
        listFields = []
      args = [*args, ]

      fieldArgs = {}
      for listField in listFields:
        if not isinstance(listField, ListField):
          raise TypeError
        dataType = listField._getDataType()
        defaultMembers = listField._getDefaultValue()
        if fieldArgs.get(dataType, None) is not None:
          raise TypeError
        fieldArgs[dataType] = [*defaultMembers, ]

      for arg in args:
        for (dataType, existing) in fieldArgs.items():
          if isinstance(arg, dataType):
            existing.append(arg)
            break

      for (field, existing) in zip(listFields, fieldArgs):
        if not isinstance(field, ListField):
          raise TypeError
        pvtName = field._getPrivateName()
        setattr(this, pvtName, existing)

    return __extra_init__

  def __init__(self, *args, **kwargs) -> None:
    type_ = list
    defVal = []
    TypedField.__init__(self, type_, defVal)
    self.__data_type__ = None
    self.__field_default__ = []
    args = [*args, ]
    if len(args) > 1:
      if isinstance(args[0], type):
        self.__data_type__ = args.pop(0)
      else:
        self.__data_type__ = type(args[0])
      for arg in args:
        if not isinstance(arg, self.__data_type__):
          raise TypeError
        self.__field_default__.append(arg)

  def _getFieldType(self) -> type:
    """Reimplemented to always return 'list'."""
    return list

  def _getDataType(self) -> type:
    """Getter-function for the member type"""
    return self.__data_type__

  def _getDefaultValue(self) -> list:
    """Getter-function for default value"""
    return [arg for arg in self.__field_default__]

  def __set__(self, instance: Any, value: Any) -> None:
    """Appends or extends the list as appropriate"""
    if isinstance(value, tuple):
      return self.__set__(instance, [*value, ])
    if isinstance(value, list):
      existing = self.__get__(instance, instance.__class__)
      return TypedField.__set__(self, instance, [*existing, *value])
    if isinstance(value, self._getDataType()):
      existing = self.__get__(instance, instance.__class__)
      return TypedField.__set__(self, instance, [*existing, value])

  def __delete__(self, instance: Any) -> None:
    """Empties the dictionary"""
    pvtName = self._getPrivateName()
