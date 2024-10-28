# generated from rosidl_generator_py/resource/_idl.py.em
# with input from synapse_msgs:msg/TrafficStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrafficStatus(type):
    """Metaclass of message 'TrafficStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'GREEN_LIGHT': 0,
        'YELLOW_LIGHT': 1,
        'RED_LIGHT': 2,
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
                'synapse_msgs.msg.TrafficStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__traffic_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__traffic_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__traffic_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__traffic_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__traffic_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'GREEN_LIGHT': cls.__constants['GREEN_LIGHT'],
            'YELLOW_LIGHT': cls.__constants['YELLOW_LIGHT'],
            'RED_LIGHT': cls.__constants['RED_LIGHT'],
        }

    @property
    def GREEN_LIGHT(self):
        """Message constant 'GREEN_LIGHT'."""
        return Metaclass_TrafficStatus.__constants['GREEN_LIGHT']

    @property
    def YELLOW_LIGHT(self):
        """Message constant 'YELLOW_LIGHT'."""
        return Metaclass_TrafficStatus.__constants['YELLOW_LIGHT']

    @property
    def RED_LIGHT(self):
        """Message constant 'RED_LIGHT'."""
        return Metaclass_TrafficStatus.__constants['RED_LIGHT']


class TrafficStatus(metaclass=Metaclass_TrafficStatus):
    """
    Message class 'TrafficStatus'.

    Constants:
      GREEN_LIGHT
      YELLOW_LIGHT
      RED_LIGHT
    """

    __slots__ = [
        '_stop_sign',
        '_traffic_light',
    ]

    _fields_and_field_types = {
        'stop_sign': 'boolean',
        'traffic_light': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.stop_sign = kwargs.get('stop_sign', bool())
        self.traffic_light = kwargs.get('traffic_light', int())

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
        if self.stop_sign != other.stop_sign:
            return False
        if self.traffic_light != other.traffic_light:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def stop_sign(self):
        """Message field 'stop_sign'."""
        return self._stop_sign

    @stop_sign.setter
    def stop_sign(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'stop_sign' field must be of type 'bool'"
        self._stop_sign = value

    @builtins.property
    def traffic_light(self):
        """Message field 'traffic_light'."""
        return self._traffic_light

    @traffic_light.setter
    def traffic_light(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'traffic_light' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'traffic_light' field must be an unsigned integer in [0, 255]"
        self._traffic_light = value
