# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: rsClientRequest.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import Button_pb2
import Mouse3D_pb2
import Status_pb2
import VehicleOrientation_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='rsClientRequest.proto',
  package='rs',
  serialized_pb='\n\x1brsClientRequest.proto\x12\x08rs\x1a\x0c\x42utton.proto\x1a\rMouse3D.proto\x1a\x0cStatus.proto\x1a\x18VehicleOrientation.proto\"\xe8\x01\n\rClientRequest\x12\n\n\x02id\x18\x01 \x02(\t\x12\"\n\x07mouse3d\x18\x02 \x03(\x0b\x32\x11.rs.Mouse3D\x12!\n\x07\x62uttons\x18\x03 \x03(\x0b\x32\x10.rs.Button\x12\x15\n\rrequestStatus\x18\x04 \x01(\x08\x12\"\n\x1arequestVehicleCameraStream\x18\x05 \x01(\x08\x12&\n\x1erequestManipulatorCameraStream\x18\x06 \x01(\x08\x12!\n\x19requestVehicleOrientation\x18\x07 \x01(\x08\"\xc5\x01\n\x0eServerResponse\x12\n\n\x02id\x18\x01 \x02(\t\x12\r\n\x05\x65rror\x18\x02 \x01(\t\x12 \n\x06status\x18\x03 \x01(\x0b\x32\x10.rs.Status\x12\x1b\n\x13vehicleCameraStream\x18\x04 \x01(\t\x12\x1f\n\x17manipulatorCameraStream\x18\x05 \x01(\t\x12\x38\n\x12vehicleOrientation\x18\x06 \x01(\x0b\x32\x1c.rs.VehicleOrientation')




_CLIENTREQUEST = _descriptor.Descriptor(
  name='ClientRequest',
  full_name='rs.ClientRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='rs.ClientRequest.id', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='mouse3d', full_name='rs.ClientRequest.mouse3d', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='buttons', full_name='rs.ClientRequest.buttons', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='requestStatus', full_name='rs.ClientRequest.requestStatus', index=3,
      number=4, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='requestVehicleCameraStream', full_name='rs.ClientRequest.requestVehicleCameraStream', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='requestManipulatorCameraStream', full_name='rs.ClientRequest.requestManipulatorCameraStream', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='requestVehicleOrientation', full_name='rs.ClientRequest.requestVehicleOrientation', index=6,
      number=7, type=8, cpp_type=7, label=1,
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
  serialized_start=111,
  serialized_end=343,
)


_SERVERRESPONSE = _descriptor.Descriptor(
  name='ServerResponse',
  full_name='rs.ServerResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='rs.ServerResponse.id', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='error', full_name='rs.ServerResponse.error', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='status', full_name='rs.ServerResponse.status', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vehicleCameraStream', full_name='rs.ServerResponse.vehicleCameraStream', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='manipulatorCameraStream', full_name='rs.ServerResponse.manipulatorCameraStream', index=4,
      number=5, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vehicleOrientation', full_name='rs.ServerResponse.vehicleOrientation', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=346,
  serialized_end=543,
)

_CLIENTREQUEST.fields_by_name['mouse3d'].message_type = Mouse3D_pb2._MOUSE3D
_CLIENTREQUEST.fields_by_name['buttons'].message_type = Button_pb2._BUTTON
_SERVERRESPONSE.fields_by_name['status'].message_type = Status_pb2._STATUS
_SERVERRESPONSE.fields_by_name['vehicleOrientation'].message_type = VehicleOrientation_pb2._VEHICLEORIENTATION
DESCRIPTOR.message_types_by_name['ClientRequest'] = _CLIENTREQUEST
DESCRIPTOR.message_types_by_name['ServerResponse'] = _SERVERRESPONSE

class ClientRequest(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _CLIENTREQUEST

  # @@protoc_insertion_point(class_scope:rs.ClientRequest)

class ServerResponse(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _SERVERRESPONSE

  # @@protoc_insertion_point(class_scope:rs.ServerResponse)


# @@protoc_insertion_point(module_scope)