// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: common/proto/vehicle_param.proto

#include "common/proto/vehicle_param.pb.h"

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
class VehicleParamDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<VehicleParam>
      _instance;
} _VehicleParam_default_instance_;
}  // namespace common
namespace protobuf_common_2fproto_2fvehicle_5fparam_2eproto {
static void InitDefaultsVehicleParam() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::common::_VehicleParam_default_instance_;
    new (ptr) ::common::VehicleParam();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::common::VehicleParam::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_VehicleParam =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsVehicleParam}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_VehicleParam.base);
}

::google::protobuf::Metadata file_level_metadata[1];
const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, brand_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, front_edge_to_center_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, back_edge_to_center_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, left_edge_to_center_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, right_edge_to_center_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, length_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, width_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, height_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, min_turn_radius_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, max_acceleration_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, max_deceleration_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, max_steer_angle_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, max_steer_angle_rate_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, min_steer_angle_rate_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, steer_ratio_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, wheel_base_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::common::VehicleParam, wheel_rolling_radius_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::common::VehicleParam)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::common::_VehicleParam_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "common/proto/vehicle_param.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, file_level_enum_descriptors, NULL);
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
      "\n common/proto/vehicle_param.proto\022\006comm"
      "on\"\301\003\n\014VehicleParam\022#\n\005brand\030\001 \001(\0162\024.com"
      "mon.VehicleBrand\022\034\n\024front_edge_to_center"
      "\030\002 \001(\001\022\033\n\023back_edge_to_center\030\003 \001(\001\022\033\n\023l"
      "eft_edge_to_center\030\004 \001(\001\022\034\n\024right_edge_t"
      "o_center\030\005 \001(\001\022\016\n\006length\030\006 \001(\001\022\r\n\005width\030"
      "\007 \001(\001\022\016\n\006height\030\010 \001(\001\022\027\n\017min_turn_radius"
      "\030\t \001(\001\022\030\n\020max_acceleration\030\n \001(\001\022\030\n\020max_"
      "deceleration\030\013 \001(\001\022\027\n\017max_steer_angle\030\014 "
      "\001(\001\022\034\n\024max_steer_angle_rate\030\r \001(\001\022\034\n\024min"
      "_steer_angle_rate\030\016 \001(\001\022\023\n\013steer_ratio\030\017"
      " \001(\001\022\022\n\nwheel_base\030\020 \001(\001\022\034\n\024wheel_rollin"
      "g_radius\030\021 \001(\001*#\n\014VehicleBrand\022\007\n\003RAY\020\000\022"
      "\n\n\006LEAPS1\020\001b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 539);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "common/proto/vehicle_param.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_common_2fproto_2fvehicle_5fparam_2eproto
namespace common {
const ::google::protobuf::EnumDescriptor* VehicleBrand_descriptor() {
  protobuf_common_2fproto_2fvehicle_5fparam_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_common_2fproto_2fvehicle_5fparam_2eproto::file_level_enum_descriptors[0];
}
bool VehicleBrand_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
      return true;
    default:
      return false;
  }
}


// ===================================================================

void VehicleParam::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int VehicleParam::kBrandFieldNumber;
const int VehicleParam::kFrontEdgeToCenterFieldNumber;
const int VehicleParam::kBackEdgeToCenterFieldNumber;
const int VehicleParam::kLeftEdgeToCenterFieldNumber;
const int VehicleParam::kRightEdgeToCenterFieldNumber;
const int VehicleParam::kLengthFieldNumber;
const int VehicleParam::kWidthFieldNumber;
const int VehicleParam::kHeightFieldNumber;
const int VehicleParam::kMinTurnRadiusFieldNumber;
const int VehicleParam::kMaxAccelerationFieldNumber;
const int VehicleParam::kMaxDecelerationFieldNumber;
const int VehicleParam::kMaxSteerAngleFieldNumber;
const int VehicleParam::kMaxSteerAngleRateFieldNumber;
const int VehicleParam::kMinSteerAngleRateFieldNumber;
const int VehicleParam::kSteerRatioFieldNumber;
const int VehicleParam::kWheelBaseFieldNumber;
const int VehicleParam::kWheelRollingRadiusFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

VehicleParam::VehicleParam()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_common_2fproto_2fvehicle_5fparam_2eproto::scc_info_VehicleParam.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:common.VehicleParam)
}
VehicleParam::VehicleParam(const VehicleParam& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&front_edge_to_center_, &from.front_edge_to_center_,
    static_cast<size_t>(reinterpret_cast<char*>(&brand_) -
    reinterpret_cast<char*>(&front_edge_to_center_)) + sizeof(brand_));
  // @@protoc_insertion_point(copy_constructor:common.VehicleParam)
}

void VehicleParam::SharedCtor() {
  ::memset(&front_edge_to_center_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&brand_) -
      reinterpret_cast<char*>(&front_edge_to_center_)) + sizeof(brand_));
}

VehicleParam::~VehicleParam() {
  // @@protoc_insertion_point(destructor:common.VehicleParam)
  SharedDtor();
}

void VehicleParam::SharedDtor() {
}

void VehicleParam::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* VehicleParam::descriptor() {
  ::protobuf_common_2fproto_2fvehicle_5fparam_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_common_2fproto_2fvehicle_5fparam_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const VehicleParam& VehicleParam::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_common_2fproto_2fvehicle_5fparam_2eproto::scc_info_VehicleParam.base);
  return *internal_default_instance();
}


void VehicleParam::Clear() {
// @@protoc_insertion_point(message_clear_start:common.VehicleParam)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&front_edge_to_center_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&brand_) -
      reinterpret_cast<char*>(&front_edge_to_center_)) + sizeof(brand_));
  _internal_metadata_.Clear();
}

bool VehicleParam::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:common.VehicleParam)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(16383u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // .common.VehicleBrand brand = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          set_brand(static_cast< ::common::VehicleBrand >(value));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double front_edge_to_center = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &front_edge_to_center_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double back_edge_to_center = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &back_edge_to_center_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double left_edge_to_center = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &left_edge_to_center_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double right_edge_to_center = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(41u /* 41 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &right_edge_to_center_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double length = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(49u /* 49 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &length_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double width = 7;
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(57u /* 57 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &width_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double height = 8;
      case 8: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(65u /* 65 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &height_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double min_turn_radius = 9;
      case 9: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(73u /* 73 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &min_turn_radius_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double max_acceleration = 10;
      case 10: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(81u /* 81 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &max_acceleration_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double max_deceleration = 11;
      case 11: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(89u /* 89 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &max_deceleration_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double max_steer_angle = 12;
      case 12: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(97u /* 97 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &max_steer_angle_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double max_steer_angle_rate = 13;
      case 13: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(105u /* 105 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &max_steer_angle_rate_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double min_steer_angle_rate = 14;
      case 14: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(113u /* 113 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &min_steer_angle_rate_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double steer_ratio = 15;
      case 15: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(121u /* 121 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &steer_ratio_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double wheel_base = 16;
      case 16: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(129u /* 129 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &wheel_base_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double wheel_rolling_radius = 17;
      case 17: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(137u /* 137 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &wheel_rolling_radius_)));
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
  // @@protoc_insertion_point(parse_success:common.VehicleParam)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:common.VehicleParam)
  return false;
#undef DO_
}

void VehicleParam::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:common.VehicleParam)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .common.VehicleBrand brand = 1;
  if (this->brand() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      1, this->brand(), output);
  }

  // double front_edge_to_center = 2;
  if (this->front_edge_to_center() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->front_edge_to_center(), output);
  }

  // double back_edge_to_center = 3;
  if (this->back_edge_to_center() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->back_edge_to_center(), output);
  }

  // double left_edge_to_center = 4;
  if (this->left_edge_to_center() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->left_edge_to_center(), output);
  }

  // double right_edge_to_center = 5;
  if (this->right_edge_to_center() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(5, this->right_edge_to_center(), output);
  }

  // double length = 6;
  if (this->length() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(6, this->length(), output);
  }

  // double width = 7;
  if (this->width() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(7, this->width(), output);
  }

  // double height = 8;
  if (this->height() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(8, this->height(), output);
  }

  // double min_turn_radius = 9;
  if (this->min_turn_radius() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(9, this->min_turn_radius(), output);
  }

  // double max_acceleration = 10;
  if (this->max_acceleration() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(10, this->max_acceleration(), output);
  }

  // double max_deceleration = 11;
  if (this->max_deceleration() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(11, this->max_deceleration(), output);
  }

  // double max_steer_angle = 12;
  if (this->max_steer_angle() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(12, this->max_steer_angle(), output);
  }

  // double max_steer_angle_rate = 13;
  if (this->max_steer_angle_rate() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(13, this->max_steer_angle_rate(), output);
  }

  // double min_steer_angle_rate = 14;
  if (this->min_steer_angle_rate() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(14, this->min_steer_angle_rate(), output);
  }

  // double steer_ratio = 15;
  if (this->steer_ratio() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(15, this->steer_ratio(), output);
  }

  // double wheel_base = 16;
  if (this->wheel_base() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(16, this->wheel_base(), output);
  }

  // double wheel_rolling_radius = 17;
  if (this->wheel_rolling_radius() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(17, this->wheel_rolling_radius(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:common.VehicleParam)
}

::google::protobuf::uint8* VehicleParam::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:common.VehicleParam)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .common.VehicleBrand brand = 1;
  if (this->brand() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      1, this->brand(), target);
  }

  // double front_edge_to_center = 2;
  if (this->front_edge_to_center() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->front_edge_to_center(), target);
  }

  // double back_edge_to_center = 3;
  if (this->back_edge_to_center() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->back_edge_to_center(), target);
  }

  // double left_edge_to_center = 4;
  if (this->left_edge_to_center() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->left_edge_to_center(), target);
  }

  // double right_edge_to_center = 5;
  if (this->right_edge_to_center() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(5, this->right_edge_to_center(), target);
  }

  // double length = 6;
  if (this->length() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(6, this->length(), target);
  }

  // double width = 7;
  if (this->width() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(7, this->width(), target);
  }

  // double height = 8;
  if (this->height() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(8, this->height(), target);
  }

  // double min_turn_radius = 9;
  if (this->min_turn_radius() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(9, this->min_turn_radius(), target);
  }

  // double max_acceleration = 10;
  if (this->max_acceleration() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(10, this->max_acceleration(), target);
  }

  // double max_deceleration = 11;
  if (this->max_deceleration() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(11, this->max_deceleration(), target);
  }

  // double max_steer_angle = 12;
  if (this->max_steer_angle() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(12, this->max_steer_angle(), target);
  }

  // double max_steer_angle_rate = 13;
  if (this->max_steer_angle_rate() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(13, this->max_steer_angle_rate(), target);
  }

  // double min_steer_angle_rate = 14;
  if (this->min_steer_angle_rate() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(14, this->min_steer_angle_rate(), target);
  }

  // double steer_ratio = 15;
  if (this->steer_ratio() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(15, this->steer_ratio(), target);
  }

  // double wheel_base = 16;
  if (this->wheel_base() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(16, this->wheel_base(), target);
  }

  // double wheel_rolling_radius = 17;
  if (this->wheel_rolling_radius() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(17, this->wheel_rolling_radius(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:common.VehicleParam)
  return target;
}

size_t VehicleParam::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:common.VehicleParam)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // double front_edge_to_center = 2;
  if (this->front_edge_to_center() != 0) {
    total_size += 1 + 8;
  }

  // double back_edge_to_center = 3;
  if (this->back_edge_to_center() != 0) {
    total_size += 1 + 8;
  }

  // double left_edge_to_center = 4;
  if (this->left_edge_to_center() != 0) {
    total_size += 1 + 8;
  }

  // double right_edge_to_center = 5;
  if (this->right_edge_to_center() != 0) {
    total_size += 1 + 8;
  }

  // double length = 6;
  if (this->length() != 0) {
    total_size += 1 + 8;
  }

  // double width = 7;
  if (this->width() != 0) {
    total_size += 1 + 8;
  }

  // double height = 8;
  if (this->height() != 0) {
    total_size += 1 + 8;
  }

  // double min_turn_radius = 9;
  if (this->min_turn_radius() != 0) {
    total_size += 1 + 8;
  }

  // double max_acceleration = 10;
  if (this->max_acceleration() != 0) {
    total_size += 1 + 8;
  }

  // double max_deceleration = 11;
  if (this->max_deceleration() != 0) {
    total_size += 1 + 8;
  }

  // double max_steer_angle = 12;
  if (this->max_steer_angle() != 0) {
    total_size += 1 + 8;
  }

  // double max_steer_angle_rate = 13;
  if (this->max_steer_angle_rate() != 0) {
    total_size += 1 + 8;
  }

  // double min_steer_angle_rate = 14;
  if (this->min_steer_angle_rate() != 0) {
    total_size += 1 + 8;
  }

  // double steer_ratio = 15;
  if (this->steer_ratio() != 0) {
    total_size += 1 + 8;
  }

  // double wheel_base = 16;
  if (this->wheel_base() != 0) {
    total_size += 2 + 8;
  }

  // double wheel_rolling_radius = 17;
  if (this->wheel_rolling_radius() != 0) {
    total_size += 2 + 8;
  }

  // .common.VehicleBrand brand = 1;
  if (this->brand() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::EnumSize(this->brand());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void VehicleParam::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:common.VehicleParam)
  GOOGLE_DCHECK_NE(&from, this);
  const VehicleParam* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const VehicleParam>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:common.VehicleParam)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:common.VehicleParam)
    MergeFrom(*source);
  }
}

void VehicleParam::MergeFrom(const VehicleParam& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:common.VehicleParam)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.front_edge_to_center() != 0) {
    set_front_edge_to_center(from.front_edge_to_center());
  }
  if (from.back_edge_to_center() != 0) {
    set_back_edge_to_center(from.back_edge_to_center());
  }
  if (from.left_edge_to_center() != 0) {
    set_left_edge_to_center(from.left_edge_to_center());
  }
  if (from.right_edge_to_center() != 0) {
    set_right_edge_to_center(from.right_edge_to_center());
  }
  if (from.length() != 0) {
    set_length(from.length());
  }
  if (from.width() != 0) {
    set_width(from.width());
  }
  if (from.height() != 0) {
    set_height(from.height());
  }
  if (from.min_turn_radius() != 0) {
    set_min_turn_radius(from.min_turn_radius());
  }
  if (from.max_acceleration() != 0) {
    set_max_acceleration(from.max_acceleration());
  }
  if (from.max_deceleration() != 0) {
    set_max_deceleration(from.max_deceleration());
  }
  if (from.max_steer_angle() != 0) {
    set_max_steer_angle(from.max_steer_angle());
  }
  if (from.max_steer_angle_rate() != 0) {
    set_max_steer_angle_rate(from.max_steer_angle_rate());
  }
  if (from.min_steer_angle_rate() != 0) {
    set_min_steer_angle_rate(from.min_steer_angle_rate());
  }
  if (from.steer_ratio() != 0) {
    set_steer_ratio(from.steer_ratio());
  }
  if (from.wheel_base() != 0) {
    set_wheel_base(from.wheel_base());
  }
  if (from.wheel_rolling_radius() != 0) {
    set_wheel_rolling_radius(from.wheel_rolling_radius());
  }
  if (from.brand() != 0) {
    set_brand(from.brand());
  }
}

void VehicleParam::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:common.VehicleParam)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void VehicleParam::CopyFrom(const VehicleParam& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:common.VehicleParam)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool VehicleParam::IsInitialized() const {
  return true;
}

void VehicleParam::Swap(VehicleParam* other) {
  if (other == this) return;
  InternalSwap(other);
}
void VehicleParam::InternalSwap(VehicleParam* other) {
  using std::swap;
  swap(front_edge_to_center_, other->front_edge_to_center_);
  swap(back_edge_to_center_, other->back_edge_to_center_);
  swap(left_edge_to_center_, other->left_edge_to_center_);
  swap(right_edge_to_center_, other->right_edge_to_center_);
  swap(length_, other->length_);
  swap(width_, other->width_);
  swap(height_, other->height_);
  swap(min_turn_radius_, other->min_turn_radius_);
  swap(max_acceleration_, other->max_acceleration_);
  swap(max_deceleration_, other->max_deceleration_);
  swap(max_steer_angle_, other->max_steer_angle_);
  swap(max_steer_angle_rate_, other->max_steer_angle_rate_);
  swap(min_steer_angle_rate_, other->min_steer_angle_rate_);
  swap(steer_ratio_, other->steer_ratio_);
  swap(wheel_base_, other->wheel_base_);
  swap(wheel_rolling_radius_, other->wheel_rolling_radius_);
  swap(brand_, other->brand_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata VehicleParam::GetMetadata() const {
  protobuf_common_2fproto_2fvehicle_5fparam_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_common_2fproto_2fvehicle_5fparam_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace common
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::common::VehicleParam* Arena::CreateMaybeMessage< ::common::VehicleParam >(Arena* arena) {
  return Arena::CreateInternal< ::common::VehicleParam >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)