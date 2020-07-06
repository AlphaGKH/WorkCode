// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: common/proto/vehicle_state.proto

#include "common/proto/vehicle_state.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace common {
class VehicleStateDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<VehicleState>
      _instance;
} _VehicleState_default_instance_;
}  // namespace common
namespace protobuf_common_2fproto_2fvehicle_5fstate_2eproto {
static void InitDefaultsVehicleState() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::common::_VehicleState_default_instance_;
    new (ptr) ::common::VehicleState();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::common::VehicleState::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_VehicleState =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsVehicleState}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_VehicleState.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleState, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleState, x_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleState, y_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleState, theta_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleState, kappa_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleState, linear_velocity_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleState, linear_acceleration_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleState, timestamp_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::common::VehicleState)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::common::_VehicleState_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "common/proto/vehicle_state.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n common/proto/vehicle_state.proto\022\006comm"
      "on\"\213\001\n\014VehicleState\022\t\n\001x\030\001 \001(\001\022\t\n\001y\030\002 \001("
      "\001\022\r\n\005theta\030\003 \001(\001\022\r\n\005kappa\030\004 \001(\001\022\027\n\017linea"
      "r_velocity\030\005 \001(\001\022\033\n\023linear_acceleration\030"
      "\006 \001(\001\022\021\n\ttimestamp\030\007 \001(\001b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 192);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "common/proto/vehicle_state.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_common_2fproto_2fvehicle_5fstate_2eproto
namespace common {

// ===================================================================

void VehicleState::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int VehicleState::kXFieldNumber;
const int VehicleState::kYFieldNumber;
const int VehicleState::kThetaFieldNumber;
const int VehicleState::kKappaFieldNumber;
const int VehicleState::kLinearVelocityFieldNumber;
const int VehicleState::kLinearAccelerationFieldNumber;
const int VehicleState::kTimestampFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

VehicleState::VehicleState()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_common_2fproto_2fvehicle_5fstate_2eproto::scc_info_VehicleState.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:common.VehicleState)
}
VehicleState::VehicleState(const VehicleState& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&x_, &from.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&timestamp_) -
    reinterpret_cast<char*>(&x_)) + sizeof(timestamp_));
  // @@protoc_insertion_point(copy_constructor:common.VehicleState)
}

void VehicleState::SharedCtor() {
  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&timestamp_) -
      reinterpret_cast<char*>(&x_)) + sizeof(timestamp_));
}

VehicleState::~VehicleState() {
  // @@protoc_insertion_point(destructor:common.VehicleState)
  SharedDtor();
}

void VehicleState::SharedDtor() {
}

void VehicleState::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* VehicleState::descriptor() {
  ::protobuf_common_2fproto_2fvehicle_5fstate_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_common_2fproto_2fvehicle_5fstate_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const VehicleState& VehicleState::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_common_2fproto_2fvehicle_5fstate_2eproto::scc_info_VehicleState.base);
  return *internal_default_instance();
}


void VehicleState::Clear() {
// @@protoc_insertion_point(message_clear_start:common.VehicleState)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&timestamp_) -
      reinterpret_cast<char*>(&x_)) + sizeof(timestamp_));
  _internal_metadata_.Clear();
}

bool VehicleState::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:common.VehicleState)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // double x = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &x_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double y = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &y_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double theta = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &theta_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double kappa = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &kappa_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double linear_velocity = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(41u /* 41 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &linear_velocity_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double linear_acceleration = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(49u /* 49 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &linear_acceleration_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double timestamp = 7;
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(57u /* 57 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &timestamp_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:common.VehicleState)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:common.VehicleState)
  return false;
#undef DO_
}

void VehicleState::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:common.VehicleState)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double x = 1;
  if (this->x() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->x(), output);
  }

  // double y = 2;
  if (this->y() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->y(), output);
  }

  // double theta = 3;
  if (this->theta() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->theta(), output);
  }

  // double kappa = 4;
  if (this->kappa() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->kappa(), output);
  }

  // double linear_velocity = 5;
  if (this->linear_velocity() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(5, this->linear_velocity(), output);
  }

  // double linear_acceleration = 6;
  if (this->linear_acceleration() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(6, this->linear_acceleration(), output);
  }

  // double timestamp = 7;
  if (this->timestamp() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(7, this->timestamp(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:common.VehicleState)
}

::google::protobuf::uint8* VehicleState::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:common.VehicleState)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double x = 1;
  if (this->x() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->x(), target);
  }

  // double y = 2;
  if (this->y() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->y(), target);
  }

  // double theta = 3;
  if (this->theta() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->theta(), target);
  }

  // double kappa = 4;
  if (this->kappa() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->kappa(), target);
  }

  // double linear_velocity = 5;
  if (this->linear_velocity() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(5, this->linear_velocity(), target);
  }

  // double linear_acceleration = 6;
  if (this->linear_acceleration() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(6, this->linear_acceleration(), target);
  }

  // double timestamp = 7;
  if (this->timestamp() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(7, this->timestamp(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:common.VehicleState)
  return target;
}

size_t VehicleState::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:common.VehicleState)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // double x = 1;
  if (this->x() != 0) {
    total_size += 1 + 8;
  }

  // double y = 2;
  if (this->y() != 0) {
    total_size += 1 + 8;
  }

  // double theta = 3;
  if (this->theta() != 0) {
    total_size += 1 + 8;
  }

  // double kappa = 4;
  if (this->kappa() != 0) {
    total_size += 1 + 8;
  }

  // double linear_velocity = 5;
  if (this->linear_velocity() != 0) {
    total_size += 1 + 8;
  }

  // double linear_acceleration = 6;
  if (this->linear_acceleration() != 0) {
    total_size += 1 + 8;
  }

  // double timestamp = 7;
  if (this->timestamp() != 0) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void VehicleState::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:common.VehicleState)
  GOOGLE_DCHECK_NE(&from, this);
  const VehicleState* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const VehicleState>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:common.VehicleState)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:common.VehicleState)
    MergeFrom(*source);
  }
}

void VehicleState::MergeFrom(const VehicleState& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:common.VehicleState)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.x() != 0) {
    set_x(from.x());
  }
  if (from.y() != 0) {
    set_y(from.y());
  }
  if (from.theta() != 0) {
    set_theta(from.theta());
  }
  if (from.kappa() != 0) {
    set_kappa(from.kappa());
  }
  if (from.linear_velocity() != 0) {
    set_linear_velocity(from.linear_velocity());
  }
  if (from.linear_acceleration() != 0) {
    set_linear_acceleration(from.linear_acceleration());
  }
  if (from.timestamp() != 0) {
    set_timestamp(from.timestamp());
  }
}

void VehicleState::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:common.VehicleState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void VehicleState::CopyFrom(const VehicleState& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:common.VehicleState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool VehicleState::IsInitialized() const {
  return true;
}

void VehicleState::Swap(VehicleState* other) {
  if (other == this) return;
  InternalSwap(other);
}
void VehicleState::InternalSwap(VehicleState* other) {
  using std::swap;
  swap(x_, other->x_);
  swap(y_, other->y_);
  swap(theta_, other->theta_);
  swap(kappa_, other->kappa_);
  swap(linear_velocity_, other->linear_velocity_);
  swap(linear_acceleration_, other->linear_acceleration_);
  swap(timestamp_, other->timestamp_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata VehicleState::GetMetadata() const {
  protobuf_common_2fproto_2fvehicle_5fstate_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_common_2fproto_2fvehicle_5fstate_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace common
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::common::VehicleState* Arena::CreateMaybeMessage< ::common::VehicleState >(Arena* arena) {
  return Arena::CreateInternal< ::common::VehicleState >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)