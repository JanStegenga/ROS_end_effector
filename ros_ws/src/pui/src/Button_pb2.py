# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: Button.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)




DESCRIPTOR = _descriptor.FileDescriptor(
  name='Button.proto',
  package='rs',
  serialized_pb='\n\x0c\x42utton.proto\x12\x08rs\"%\n\x06\x42utton\x12\n\n\x02id\x18\x01 \x02(\x05\x12\x0f\n\x07pressed\x18\x02 \x02(\x08')




_BUTTON = _descriptor.Descriptor(
  name='Button',
  full_name='rs.Button',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='rs.Button.id', index=0,
      number=1, type=5, cpp_type=1, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pressed', full_name='rs.Button.pressed', index=1,
      number=2, type=8, cpp_type=7, label=2,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=26,
  serialized_end=63,
)

DESCRIPTOR.message_types_by_name['Button'] = _BUTTON

class Button(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _BUTTON

  # @@protoc_insertion_point(class_scope:roboship.Button)


# @@protoc_insertion_point(module_scope)