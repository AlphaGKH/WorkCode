// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: perception/proto/ogm_config.proto

#ifndef PROTOBUF_INCLUDED_perception_2fproto_2fogm_5fconfig_2eproto
#define PROTOBUF_INCLUDED_perception_2fproto_2fogm_5fconfig_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_perception_2fproto_2fogm_5fconfig_2eproto 

namespace protobuf_perception_2fproto_2fogm_5fconfig_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[2];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_perception_2fproto_2fogm_5fconfig_2eproto
namespace perception {
class Lidar2DConfig;
class Lidar2DConfigDefaultTypeInternal;
extern Lidar2DConfigDefaultTypeInternal _Lidar2DConfig_default_instance_;
class OgmConfig;
class OgmConfigDefaultTypeInternal;
extern OgmConfigDefaultTypeInternal _OgmConfig_default_instance_;
}  // namespace perception
namespace google {
namespace protobuf {
template<> ::perception::Lidar2DConfig* Arena::CreateMaybeMessage<::perception::Lidar2DConfig>(Arena*);
template<> ::perception::OgmConfig* Arena::CreateMaybeMessage<::perception::OgmConfig>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace perception {

// ===================================================================

class Lidar2DConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:perception.Lidar2DConfig) */ {
 public:
  Lidar2DConfig();
  virtual ~Lidar2DConfig();

  Lidar2DConfig(const Lidar2DConfig& from);

  inline Lidar2DConfig& operator=(const Lidar2DConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Lidar2DConfig(Lidar2DConfig&& from) noexcept
    : Lidar2DConfig() {
    *this = ::std::move(from);
  }

  inline Lidar2DConfig& operator=(Lidar2DConfig&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Lidar2DConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Lidar2DConfig* internal_default_instance() {
    return reinterpret_cast<const Lidar2DConfig*>(
               &_Lidar2DConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Lidar2DConfig* other);
  friend void swap(Lidar2DConfig& a, Lidar2DConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Lidar2DConfig* New() const final {
    return CreateMaybeMessage<Lidar2DConfig>(NULL);
  }

  Lidar2DConfig* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Lidar2DConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Lidar2DConfig& from);
  void MergeFrom(const Lidar2DConfig& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Lidar2DConfig* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // double scaningAngle = 1;
  void clear_scaningangle();
  static const int kScaningAngleFieldNumber = 1;
  double scaningangle() const;
  void set_scaningangle(double value);

  // double maxDistance = 2;
  void clear_maxdistance();
  static const int kMaxDistanceFieldNumber = 2;
  double maxdistance() const;
  void set_maxdistance(double value);

  // int32 linesNumber = 3;
  void clear_linesnumber();
  static const int kLinesNumberFieldNumber = 3;
  ::google::protobuf::int32 linesnumber() const;
  void set_linesnumber(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:perception.Lidar2DConfig)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double scaningangle_;
  double maxdistance_;
  ::google::protobuf::int32 linesnumber_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_perception_2fproto_2fogm_5fconfig_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class OgmConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:perception.OgmConfig) */ {
 public:
  OgmConfig();
  virtual ~OgmConfig();

  OgmConfig(const OgmConfig& from);

  inline OgmConfig& operator=(const OgmConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  OgmConfig(OgmConfig&& from) noexcept
    : OgmConfig() {
    *this = ::std::move(from);
  }

  inline OgmConfig& operator=(OgmConfig&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const OgmConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const OgmConfig* internal_default_instance() {
    return reinterpret_cast<const OgmConfig*>(
               &_OgmConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(OgmConfig* other);
  friend void swap(OgmConfig& a, OgmConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline OgmConfig* New() const final {
    return CreateMaybeMessage<OgmConfig>(NULL);
  }

  OgmConfig* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<OgmConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const OgmConfig& from);
  void MergeFrom(const OgmConfig& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(OgmConfig* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // .perception.Lidar2DConfig lidar2d_config = 1;
  bool has_lidar2d_config() const;
  void clear_lidar2d_config();
  static const int kLidar2DConfigFieldNumber = 1;
  private:
  const ::perception::Lidar2DConfig& _internal_lidar2d_config() const;
  public:
  const ::perception::Lidar2DConfig& lidar2d_config() const;
  ::perception::Lidar2DConfig* release_lidar2d_config();
  ::perception::Lidar2DConfig* mutable_lidar2d_config();
  void set_allocated_lidar2d_config(::perception::Lidar2DConfig* lidar2d_config);

  // int32 width = 2;
  void clear_width();
  static const int kWidthFieldNumber = 2;
  ::google::protobuf::int32 width() const;
  void set_width(::google::protobuf::int32 value);

  // int32 length = 3;
  void clear_length();
  static const int kLengthFieldNumber = 3;
  ::google::protobuf::int32 length() const;
  void set_length(::google::protobuf::int32 value);

  // double width_resolution = 4;
  void clear_width_resolution();
  static const int kWidthResolutionFieldNumber = 4;
  double width_resolution() const;
  void set_width_resolution(double value);

  // double length_resolution = 5;
  void clear_length_resolution();
  static const int kLengthResolutionFieldNumber = 5;
  double length_resolution() const;
  void set_length_resolution(double value);

  // double left_width = 6;
  void clear_left_width();
  static const int kLeftWidthFieldNumber = 6;
  double left_width() const;
  void set_left_width(double value);

  // double back_lenght = 7;
  void clear_back_lenght();
  static const int kBackLenghtFieldNumber = 7;
  double back_lenght() const;
  void set_back_lenght(double value);

  // bool enable_ogm_fill = 8;
  void clear_enable_ogm_fill();
  static const int kEnableOgmFillFieldNumber = 8;
  bool enable_ogm_fill() const;
  void set_enable_ogm_fill(bool value);

  // bool enable_ogm_expand = 9;
  void clear_enable_ogm_expand();
  static const int kEnableOgmExpandFieldNumber = 9;
  bool enable_ogm_expand() const;
  void set_enable_ogm_expand(bool value);

  // @@protoc_insertion_point(class_scope:perception.OgmConfig)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::perception::Lidar2DConfig* lidar2d_config_;
  ::google::protobuf::int32 width_;
  ::google::protobuf::int32 length_;
  double width_resolution_;
  double length_resolution_;
  double left_width_;
  double back_lenght_;
  bool enable_ogm_fill_;
  bool enable_ogm_expand_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_perception_2fproto_2fogm_5fconfig_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Lidar2DConfig

// double scaningAngle = 1;
inline void Lidar2DConfig::clear_scaningangle() {
  scaningangle_ = 0;
}
inline double Lidar2DConfig::scaningangle() const {
  // @@protoc_insertion_point(field_get:perception.Lidar2DConfig.scaningAngle)
  return scaningangle_;
}
inline void Lidar2DConfig::set_scaningangle(double value) {
  
  scaningangle_ = value;
  // @@protoc_insertion_point(field_set:perception.Lidar2DConfig.scaningAngle)
}

// double maxDistance = 2;
inline void Lidar2DConfig::clear_maxdistance() {
  maxdistance_ = 0;
}
inline double Lidar2DConfig::maxdistance() const {
  // @@protoc_insertion_point(field_get:perception.Lidar2DConfig.maxDistance)
  return maxdistance_;
}
inline void Lidar2DConfig::set_maxdistance(double value) {
  
  maxdistance_ = value;
  // @@protoc_insertion_point(field_set:perception.Lidar2DConfig.maxDistance)
}

// int32 linesNumber = 3;
inline void Lidar2DConfig::clear_linesnumber() {
  linesnumber_ = 0;
}
inline ::google::protobuf::int32 Lidar2DConfig::linesnumber() const {
  // @@protoc_insertion_point(field_get:perception.Lidar2DConfig.linesNumber)
  return linesnumber_;
}
inline void Lidar2DConfig::set_linesnumber(::google::protobuf::int32 value) {
  
  linesnumber_ = value;
  // @@protoc_insertion_point(field_set:perception.Lidar2DConfig.linesNumber)
}

// -------------------------------------------------------------------

// OgmConfig

// .perception.Lidar2DConfig lidar2d_config = 1;
inline bool OgmConfig::has_lidar2d_config() const {
  return this != internal_default_instance() && lidar2d_config_ != NULL;
}
inline void OgmConfig::clear_lidar2d_config() {
  if (GetArenaNoVirtual() == NULL && lidar2d_config_ != NULL) {
    delete lidar2d_config_;
  }
  lidar2d_config_ = NULL;
}
inline const ::perception::Lidar2DConfig& OgmConfig::_internal_lidar2d_config() const {
  return *lidar2d_config_;
}
inline const ::perception::Lidar2DConfig& OgmConfig::lidar2d_config() const {
  const ::perception::Lidar2DConfig* p = lidar2d_config_;
  // @@protoc_insertion_point(field_get:perception.OgmConfig.lidar2d_config)
  return p != NULL ? *p : *reinterpret_cast<const ::perception::Lidar2DConfig*>(
      &::perception::_Lidar2DConfig_default_instance_);
}
inline ::perception::Lidar2DConfig* OgmConfig::release_lidar2d_config() {
  // @@protoc_insertion_point(field_release:perception.OgmConfig.lidar2d_config)
  
  ::perception::Lidar2DConfig* temp = lidar2d_config_;
  lidar2d_config_ = NULL;
  return temp;
}
inline ::perception::Lidar2DConfig* OgmConfig::mutable_lidar2d_config() {
  
  if (lidar2d_config_ == NULL) {
    auto* p = CreateMaybeMessage<::perception::Lidar2DConfig>(GetArenaNoVirtual());
    lidar2d_config_ = p;
  }
  // @@protoc_insertion_point(field_mutable:perception.OgmConfig.lidar2d_config)
  return lidar2d_config_;
}
inline void OgmConfig::set_allocated_lidar2d_config(::perception::Lidar2DConfig* lidar2d_config) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete lidar2d_config_;
  }
  if (lidar2d_config) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      lidar2d_config = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, lidar2d_config, submessage_arena);
    }
    
  } else {
    
  }
  lidar2d_config_ = lidar2d_config;
  // @@protoc_insertion_point(field_set_allocated:perception.OgmConfig.lidar2d_config)
}

// int32 width = 2;
inline void OgmConfig::clear_width() {
  width_ = 0;
}
inline ::google::protobuf::int32 OgmConfig::width() const {
  // @@protoc_insertion_point(field_get:perception.OgmConfig.width)
  return width_;
}
inline void OgmConfig::set_width(::google::protobuf::int32 value) {
  
  width_ = value;
  // @@protoc_insertion_point(field_set:perception.OgmConfig.width)
}

// int32 length = 3;
inline void OgmConfig::clear_length() {
  length_ = 0;
}
inline ::google::protobuf::int32 OgmConfig::length() const {
  // @@protoc_insertion_point(field_get:perception.OgmConfig.length)
  return length_;
}
inline void OgmConfig::set_length(::google::protobuf::int32 value) {
  
  length_ = value;
  // @@protoc_insertion_point(field_set:perception.OgmConfig.length)
}

// double width_resolution = 4;
inline void OgmConfig::clear_width_resolution() {
  width_resolution_ = 0;
}
inline double OgmConfig::width_resolution() const {
  // @@protoc_insertion_point(field_get:perception.OgmConfig.width_resolution)
  return width_resolution_;
}
inline void OgmConfig::set_width_resolution(double value) {
  
  width_resolution_ = value;
  // @@protoc_insertion_point(field_set:perception.OgmConfig.width_resolution)
}

// double length_resolution = 5;
inline void OgmConfig::clear_length_resolution() {
  length_resolution_ = 0;
}
inline double OgmConfig::length_resolution() const {
  // @@protoc_insertion_point(field_get:perception.OgmConfig.length_resolution)
  return length_resolution_;
}
inline void OgmConfig::set_length_resolution(double value) {
  
  length_resolution_ = value;
  // @@protoc_insertion_point(field_set:perception.OgmConfig.length_resolution)
}

// double left_width = 6;
inline void OgmConfig::clear_left_width() {
  left_width_ = 0;
}
inline double OgmConfig::left_width() const {
  // @@protoc_insertion_point(field_get:perception.OgmConfig.left_width)
  return left_width_;
}
inline void OgmConfig::set_left_width(double value) {
  
  left_width_ = value;
  // @@protoc_insertion_point(field_set:perception.OgmConfig.left_width)
}

// double back_lenght = 7;
inline void OgmConfig::clear_back_lenght() {
  back_lenght_ = 0;
}
inline double OgmConfig::back_lenght() const {
  // @@protoc_insertion_point(field_get:perception.OgmConfig.back_lenght)
  return back_lenght_;
}
inline void OgmConfig::set_back_lenght(double value) {
  
  back_lenght_ = value;
  // @@protoc_insertion_point(field_set:perception.OgmConfig.back_lenght)
}

// bool enable_ogm_fill = 8;
inline void OgmConfig::clear_enable_ogm_fill() {
  enable_ogm_fill_ = false;
}
inline bool OgmConfig::enable_ogm_fill() const {
  // @@protoc_insertion_point(field_get:perception.OgmConfig.enable_ogm_fill)
  return enable_ogm_fill_;
}
inline void OgmConfig::set_enable_ogm_fill(bool value) {
  
  enable_ogm_fill_ = value;
  // @@protoc_insertion_point(field_set:perception.OgmConfig.enable_ogm_fill)
}

// bool enable_ogm_expand = 9;
inline void OgmConfig::clear_enable_ogm_expand() {
  enable_ogm_expand_ = false;
}
inline bool OgmConfig::enable_ogm_expand() const {
  // @@protoc_insertion_point(field_get:perception.OgmConfig.enable_ogm_expand)
  return enable_ogm_expand_;
}
inline void OgmConfig::set_enable_ogm_expand(bool value) {
  
  enable_ogm_expand_ = value;
  // @@protoc_insertion_point(field_set:perception.OgmConfig.enable_ogm_expand)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace perception

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_perception_2fproto_2fogm_5fconfig_2eproto
