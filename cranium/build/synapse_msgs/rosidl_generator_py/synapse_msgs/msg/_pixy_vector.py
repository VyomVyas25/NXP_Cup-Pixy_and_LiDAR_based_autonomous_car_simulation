# generated from rosidl_generator_py/resource/_idl.py.em
# with input from synapse_msgs:msg/PixyVector.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PixyVector(type):
    """Metaclass of message 'PixyVector'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('synapse_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'synapse_msgs.msg.PixyVector')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pixy_vector
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pixy_vector
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pixy_vector
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pixy_vector
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pixy_vector

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PixyVector(metaclass=Metaclass_PixyVector):
    """Message class 'PixyVector'."""

    __slots__ = [
        '_header',
        '_m0_x0',
        '_m0_y0',
        '_m0_x1',
        '_m0_y1',
        '_m1_x0',
        '_m1_y0',
        '_m1_x1',
        '_m1_y1',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'm0_x0': 'int32',
        'm0_y0': 'int32',
        'm0_x1': 'int32',
        'm0_y1': 'int32',
        'm1_x0': 'int32',
        'm1_y0': 'int32',
        'm1_x1': 'int32',
        'm1_y1': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.m0_x0 = kwargs.get('m0_x0', int())
        self.m0_y0 = kwargs.get('m0_y0', int())
        self.m0_x1 = kwargs.get('m0_x1', int())
        self.m0_y1 = kwargs.get('m0_y1', int())
        self.m1_x0 = kwargs.get('m1_x0', int())
        self.m1_y0 = kwargs.get('m1_y0', int())
        self.m1_x1 = kwargs.get('m1_x1', int())
        self.m1_y1 = kwargs.get('m1_y1', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.m0_x0 != other.m0_x0:
            return False
        if self.m0_y0 != other.m0_y0:
            return False
        if self.m0_x1 != other.m0_x1:
            return False
        if self.m0_y1 != other.m0_y1:
            return False
        if self.m1_x0 != other.m1_x0:
            return False
        if self.m1_y0 != other.m1_y0:
            return False
        if self.m1_x1 != other.m1_x1:
            return False
        if self.m1_y1 != other.m1_y1:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def m0_x0(self):
        """Message field 'm0_x0'."""
        return self._m0_x0

    @m0_x0.setter
    def m0_x0(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'm0_x0' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'm0_x0' field must be an integer in [-2147483648, 2147483647]"
        self._m0_x0 = value

    @builtins.property
    def m0_y0(self):
        """Message field 'm0_y0'."""
        return self._m0_y0

    @m0_y0.setter
    def m0_y0(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'm0_y0' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'm0_y0' field must be an integer in [-2147483648, 2147483647]"
        self._m0_y0 = value

    @builtins.property
    def m0_x1(self):
        """Message field 'm0_x1'."""
        return self._m0_x1

    @m0_x1.setter
    def m0_x1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'm0_x1' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'm0_x1' field must be an integer in [-2147483648, 2147483647]"
        self._m0_x1 = value

    @builtins.property
    def m0_y1(self):
        """Message field 'm0_y1'."""
        return self._m0_y1

    @m0_y1.setter
    def m0_y1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'm0_y1' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'm0_y1' field must be an integer in [-2147483648, 2147483647]"
        self._m0_y1 = value

    @builtins.property
    def m1_x0(self):
        """Message field 'm1_x0'."""
        return self._m1_x0

    @m1_x0.setter
    def m1_x0(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'm1_x0' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'm1_x0' field must be an integer in [-2147483648, 2147483647]"
        self._m1_x0 = value

    @builtins.property
    def m1_y0(self):
        """Message field 'm1_y0'."""
        return self._m1_y0

    @m1_y0.setter
    def m1_y0(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'm1_y0' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'm1_y0' field must be an integer in [-2147483648, 2147483647]"
        self._m1_y0 = value

    @builtins.property
    def m1_x1(self):
        """Message field 'm1_x1'."""
        return self._m1_x1

    @m1_x1.setter
    def m1_x1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'm1_x1' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'm1_x1' field must be an integer in [-2147483648, 2147483647]"
        self._m1_x1 = value

    @builtins.property
    def m1_y1(self):
        """Message field 'm1_y1'."""
        return self._m1_y1

    @m1_y1.setter
    def m1_y1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'm1_y1' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'm1_y1' field must be an integer in [-2147483648, 2147483647]"
        self._m1_y1 = value
