"""RosTopic"""
#  MIT Licence
#  Copyright (c) 2024 Asger Jon Vistisen
from __future__ import annotations


class Topic(object):
  """Base class of L{Publisher} and L{Subscriber}"""

  def __init__(self, name, data_class, reg_type):
    """
    @param name: graph resource name of topic, e.g. 'laser'.
    @type  name: str
    @param data_class: message class for serialization
    @type  data_class: L{Message}
    @param reg_type Registration.PUB or Registration.SUB
    @type  reg_type: str
    @raise ValueError: if parameters are invalid
    """

    if not name or not isstring(name):
      raise ValueError("topic name is not a non-empty string")
    try:
      if python3 == 1:
        name.encode("utf-8")
      else:
        name = name.encode("utf-8")
    except UnicodeError:
      raise ValueError("topic name must be ascii/utf-8 compatible")
    if data_class is None:
      raise ValueError("topic parameter 'data_class' is not initialized")
    if not type(data_class) == type:
      raise ValueError("data_class [%s] is not a class" % data_class)
    if not issubclass(data_class, genpy.Message):
      raise ValueError("data_class [%s] is not a message data class" %
                       data_class.__class__.__name__)
    # #2202
    if not rosgraph.names.is_legal_name(name):
      import warnings
      warnings.warn(
        "'%s' is not a legal ROS graph resource name. This may cause "
        "problems with other ROS tools" % name,
        stacklevel=2)

    # this is a bit ugly, but necessary due to the fact that we allow
    # topics and services to be initialized before the node
    if not rospy.core.is_initialized():
      self.resolved_name = rospy.names.resolve_name_without_node_name(name)
    else:
      # init_node() has been called, so we can do normal resolution
      self.resolved_name = resolve_name(name)

    self.name = self.resolved_name  # #1810 for backwards compatibility

    self.data_class = data_class
    self.type = data_class._type
    self.md5sum = data_class._md5sum
    self.reg_type = reg_type
    self.impl = get_topic_manager().acquire_impl(reg_type,
                                                 self.resolved_name,
                                                 data_class)

  def get_num_connections(self):
    """
    get the number of connections to other ROS nodes for this topic. For a
    Publisher,
    this corresponds to the number of nodes subscribing. For a Subscriber,
    the number
    of publishers.
    @return: number of connections
    @rtype: int
    """
    return self.impl.get_num_connections()

  def unregister(self):
    """
    unpublish/unsubscribe from topic. Topic instance is no longer
    valid after this call. Additional calls to unregister() have no effect.
    """
    # as we don't guard unregister, have to protect value of
    # resolved_name for release_impl call
    resolved_name = self.resolved_name
    if resolved_name and self.impl:
      get_topic_manager().release_impl(self.reg_type, resolved_name)
      self.impl = self.resolved_name = self.type = self.md5sum = (
        self).data_class = None

  def __iter__(self) -> Any:
    """blabla"""
