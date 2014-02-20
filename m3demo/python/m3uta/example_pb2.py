# Generated by the protocol buffer compiler.  DO NOT EDIT!

from google.protobuf import descriptor
from google.protobuf import message
from google.protobuf import reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


DESCRIPTOR = descriptor.FileDescriptor(
  name='example.proto',
  package='',
  serialized_pb='\n\rexample.proto\x1a\x14\x63omponent_base.proto\";\n\x0fM3ExampleStatus\x12\x1b\n\x04\x62\x61se\x18\x01 \x01(\x0b\x32\r.M3BaseStatus\x12\x0b\n\x03\x66oo\x18\x02 \x01(\x01\"@\n\x0eM3ExampleParam\x12\x0e\n\x06max_fx\x18\x01 \x01(\x01\x12\x0e\n\x06max_fy\x18\x02 \x01(\x01\x12\x0e\n\x06max_fz\x18\x03 \x01(\x01\"F\n\x10M3ExampleCommand\x12\x0e\n\x06\x65nable\x18\x01 \x01(\x08\x12\n\n\x02\x66x\x18\x02 \x01(\x01\x12\n\n\x02\x66y\x18\x03 \x01(\x01\x12\n\n\x02\x66z\x18\x04 \x01(\x01\x42\x02H\x01')




_M3EXAMPLESTATUS = descriptor.Descriptor(
  name='M3ExampleStatus',
  full_name='M3ExampleStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='base', full_name='M3ExampleStatus.base', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='foo', full_name='M3ExampleStatus.foo', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=39,
  serialized_end=98,
)


_M3EXAMPLEPARAM = descriptor.Descriptor(
  name='M3ExampleParam',
  full_name='M3ExampleParam',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='max_fx', full_name='M3ExampleParam.max_fx', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='max_fy', full_name='M3ExampleParam.max_fy', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='max_fz', full_name='M3ExampleParam.max_fz', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=100,
  serialized_end=164,
)


_M3EXAMPLECOMMAND = descriptor.Descriptor(
  name='M3ExampleCommand',
  full_name='M3ExampleCommand',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    descriptor.FieldDescriptor(
      name='enable', full_name='M3ExampleCommand.enable', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='fx', full_name='M3ExampleCommand.fx', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='fy', full_name='M3ExampleCommand.fy', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    descriptor.FieldDescriptor(
      name='fz', full_name='M3ExampleCommand.fz', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=166,
  serialized_end=236,
)

import component_base_pb2

_M3EXAMPLESTATUS.fields_by_name['base'].message_type = component_base_pb2._M3BASESTATUS

class M3ExampleStatus(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3EXAMPLESTATUS
  
  # @@protoc_insertion_point(class_scope:M3ExampleStatus)

class M3ExampleParam(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3EXAMPLEPARAM
  
  # @@protoc_insertion_point(class_scope:M3ExampleParam)

class M3ExampleCommand(message.Message):
  __metaclass__ = reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _M3EXAMPLECOMMAND
  
  # @@protoc_insertion_point(class_scope:M3ExampleCommand)

# @@protoc_insertion_point(module_scope)
