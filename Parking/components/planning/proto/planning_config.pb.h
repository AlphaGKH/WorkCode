// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: planning/proto/planning_config.proto

#ifndef PROTOBUF_INCLUDED_planning_2fproto_2fplanning_5fconfig_2eproto
#define PROTOBUF_INCLUDED_planning_2fproto_2fplanning_5fconfig_2eproto

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
#include "planning/proto/reference_line_provider_config.pb.h"
#include "planning/proto/fp_path_plan_config.pb.h"
#include "planning/proto/poly_speed_plan_config.pb.h"
#include "perception/proto/ogm_config.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_planning_2fproto_2fplanning_5fconfig_2eproto 

namespace protobuf_planning_2fproto_2fplanning_5fconfig_2eproto {
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
}  // namespace protobuf_planning_2fproto_2fplanning_5fconfig_2eproto
namespace planning {
class PlanningConfig;
class PlanningConfigDefaultTypeInternal;
extern PlanningConfigDefaultTypeInternal _PlanningConfig_default_instance_;
class TrajectoryStitchingConfig;
class TrajectoryStitchingConfigDefaultTypeInternal;
extern TrajectoryStitchingConfigDefaultTypeInternal _TrajectoryStitchingConfig_default_instance_;
}  // namespace planning
namespace google {
namespace protobuf {
template<> ::planning::PlanningConfig* Arena::CreateMaybeMessage<::planning::PlanningConfig>(Arena*);
template<> ::planning::TrajectoryStitchingConfig* Arena::CreateMaybeMessage<::planning::TrajectoryStitchingConfig>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace planning {

// ===================================================================

class TrajectoryStitchingConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:planning.TrajectoryStitchingConfig) */ {
 public:
  TrajectoryStitchingConfig();
  virtual ~TrajectoryStitchingConfig();

  TrajectoryStitchingConfig(const TrajectoryStitchingConfig& from);

  inline TrajectoryStitchingConfig& operator=(const TrajectoryStitchingConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  TrajectoryStitchingConfig(TrajectoryStitchingConfig&& from) noexcept
    : TrajectoryStitchingConfig() {
    *this = ::std::move(from);
  }

  inline TrajectoryStitchingConfig& operator=(TrajectoryStitchingConfig&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const TrajectoryStitchingConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrajectoryStitchingConfig* internal_default_instance() {
    return reinterpret_cast<const TrajectoryStitchingConfig*>(
               &_TrajectoryStitchingConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(TrajectoryStitchingConfig* other);
  friend void swap(TrajectoryStitchingConfig& a, TrajectoryStitchingConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline TrajectoryStitchingConfig* New() const final {
    return CreateMaybeMessage<TrajectoryStitchingConfig>(NULL);
  }

  TrajectoryStitchingConfig* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<TrajectoryStitchingConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const TrajectoryStitchingConfig& from);
  void MergeFrom(const TrajectoryStitchingConfig& from);
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
  void InternalSwap(TrajectoryStitchingConfig* other);
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

  // double replan_lateral_distance_threshold = 2;
  void clear_replan_lateral_distance_threshold();
  static const int kReplanLateralDistanceThresholdFieldNumber = 2;
  double replan_lateral_distance_threshold() const;
  void set_replan_lateral_distance_threshold(double value);

  // double replan_longitudinal_distance_threshold = 3;
  void clear_replan_longitudinal_distance_threshold();
  static const int kReplanLongitudinalDistanceThresholdFieldNumber = 3;
  double replan_longitudinal_distance_threshold() const;
  void set_replan_longitudinal_distance_threshold(double value);

  // double trajectory_stitching_preserved_length = 4;
  void clear_trajectory_stitching_preserved_length();
  static const int kTrajectoryStitchingPreservedLengthFieldNumber = 4;
  double trajectory_stitching_preserved_length() const;
  void set_trajectory_stitching_preserved_length(double value);

  // bool enable_trajectory_stitcher = 1;
  void clear_enable_trajectory_stitcher();
  static const int kEnableTrajectoryStitcherFieldNumber = 1;
  bool enable_trajectory_stitcher() const;
  void set_enable_trajectory_stitcher(bool value);

  // @@protoc_insertion_point(class_scope:planning.TrajectoryStitchingConfig)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double replan_lateral_distance_threshold_;
  double replan_longitudinal_distance_threshold_;
  double trajectory_stitching_preserved_length_;
  bool enable_trajectory_stitcher_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_planning_2fproto_2fplanning_5fconfig_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class PlanningConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:planning.PlanningConfig) */ {
 public:
  PlanningConfig();
  virtual ~PlanningConfig();

  PlanningConfig(const PlanningConfig& from);

  inline PlanningConfig& operator=(const PlanningConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  PlanningConfig(PlanningConfig&& from) noexcept
    : PlanningConfig() {
    *this = ::std::move(from);
  }

  inline PlanningConfig& operator=(PlanningConfig&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const PlanningConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PlanningConfig* internal_default_instance() {
    return reinterpret_cast<const PlanningConfig*>(
               &_PlanningConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(PlanningConfig* other);
  friend void swap(PlanningConfig& a, PlanningConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline PlanningConfig* New() const final {
    return CreateMaybeMessage<PlanningConfig>(NULL);
  }

  PlanningConfig* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<PlanningConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const PlanningConfig& from);
  void MergeFrom(const PlanningConfig& from);
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
  void InternalSwap(PlanningConfig* other);
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

  // .planning.ReferenceLineProviderConfig refline_provider_config = 1;
  bool has_refline_provider_config() const;
  void clear_refline_provider_config();
  static const int kReflineProviderConfigFieldNumber = 1;
  private:
  const ::planning::ReferenceLineProviderConfig& _internal_refline_provider_config() const;
  public:
  const ::planning::ReferenceLineProviderConfig& refline_provider_config() const;
  ::planning::ReferenceLineProviderConfig* release_refline_provider_config();
  ::planning::ReferenceLineProviderConfig* mutable_refline_provider_config();
  void set_allocated_refline_provider_config(::planning::ReferenceLineProviderConfig* refline_provider_config);

  // .planning.FpPathPlanConfig fp_path_plan_config = 2;
  bool has_fp_path_plan_config() const;
  void clear_fp_path_plan_config();
  static const int kFpPathPlanConfigFieldNumber = 2;
  private:
  const ::planning::FpPathPlanConfig& _internal_fp_path_plan_config() const;
  public:
  const ::planning::FpPathPlanConfig& fp_path_plan_config() const;
  ::planning::FpPathPlanConfig* release_fp_path_plan_config();
  ::planning::FpPathPlanConfig* mutable_fp_path_plan_config();
  void set_allocated_fp_path_plan_config(::planning::FpPathPlanConfig* fp_path_plan_config);

  // .perception.OgmConfig ogm_config = 3;
  bool has_ogm_config() const;
  void clear_ogm_config();
  static const int kOgmConfigFieldNumber = 3;
  private:
  const ::perception::OgmConfig& _internal_ogm_config() const;
  public:
  const ::perception::OgmConfig& ogm_config() const;
  ::perception::OgmConfig* release_ogm_config();
  ::perception::OgmConfig* mutable_ogm_config();
  void set_allocated_ogm_config(::perception::OgmConfig* ogm_config);

  // .planning.PolySpeedPlanConfig poly_speed_plan_config = 4;
  bool has_poly_speed_plan_config() const;
  void clear_poly_speed_plan_config();
  static const int kPolySpeedPlanConfigFieldNumber = 4;
  private:
  const ::planning::PolySpeedPlanConfig& _internal_poly_speed_plan_config() const;
  public:
  const ::planning::PolySpeedPlanConfig& poly_speed_plan_config() const;
  ::planning::PolySpeedPlanConfig* release_poly_speed_plan_config();
  ::planning::PolySpeedPlanConfig* mutable_poly_speed_plan_config();
  void set_allocated_poly_speed_plan_config(::planning::PolySpeedPlanConfig* poly_speed_plan_config);

  // .planning.TrajectoryStitchingConfig traj_stitching_config = 5;
  bool has_traj_stitching_config() const;
  void clear_traj_stitching_config();
  static const int kTrajStitchingConfigFieldNumber = 5;
  private:
  const ::planning::TrajectoryStitchingConfig& _internal_traj_stitching_config() const;
  public:
  const ::planning::TrajectoryStitchingConfig& traj_stitching_config() const;
  ::planning::TrajectoryStitchingConfig* release_traj_stitching_config();
  ::planning::TrajectoryStitchingConfig* mutable_traj_stitching_config();
  void set_allocated_traj_stitching_config(::planning::TrajectoryStitchingConfig* traj_stitching_config);

  // @@protoc_insertion_point(class_scope:planning.PlanningConfig)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::planning::ReferenceLineProviderConfig* refline_provider_config_;
  ::planning::FpPathPlanConfig* fp_path_plan_config_;
  ::perception::OgmConfig* ogm_config_;
  ::planning::PolySpeedPlanConfig* poly_speed_plan_config_;
  ::planning::TrajectoryStitchingConfig* traj_stitching_config_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_planning_2fproto_2fplanning_5fconfig_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// TrajectoryStitchingConfig

// bool enable_trajectory_stitcher = 1;
inline void TrajectoryStitchingConfig::clear_enable_trajectory_stitcher() {
  enable_trajectory_stitcher_ = false;
}
inline bool TrajectoryStitchingConfig::enable_trajectory_stitcher() const {
  // @@protoc_insertion_point(field_get:planning.TrajectoryStitchingConfig.enable_trajectory_stitcher)
  return enable_trajectory_stitcher_;
}
inline void TrajectoryStitchingConfig::set_enable_trajectory_stitcher(bool value) {
  
  enable_trajectory_stitcher_ = value;
  // @@protoc_insertion_point(field_set:planning.TrajectoryStitchingConfig.enable_trajectory_stitcher)
}

// double replan_lateral_distance_threshold = 2;
inline void TrajectoryStitchingConfig::clear_replan_lateral_distance_threshold() {
  replan_lateral_distance_threshold_ = 0;
}
inline double TrajectoryStitchingConfig::replan_lateral_distance_threshold() const {
  // @@protoc_insertion_point(field_get:planning.TrajectoryStitchingConfig.replan_lateral_distance_threshold)
  return replan_lateral_distance_threshold_;
}
inline void TrajectoryStitchingConfig::set_replan_lateral_distance_threshold(double value) {
  
  replan_lateral_distance_threshold_ = value;
  // @@protoc_insertion_point(field_set:planning.TrajectoryStitchingConfig.replan_lateral_distance_threshold)
}

// double replan_longitudinal_distance_threshold = 3;
inline void TrajectoryStitchingConfig::clear_replan_longitudinal_distance_threshold() {
  replan_longitudinal_distance_threshold_ = 0;
}
inline double TrajectoryStitchingConfig::replan_longitudinal_distance_threshold() const {
  // @@protoc_insertion_point(field_get:planning.TrajectoryStitchingConfig.replan_longitudinal_distance_threshold)
  return replan_longitudinal_distance_threshold_;
}
inline void TrajectoryStitchingConfig::set_replan_longitudinal_distance_threshold(double value) {
  
  replan_longitudinal_distance_threshold_ = value;
  // @@protoc_insertion_point(field_set:planning.TrajectoryStitchingConfig.replan_longitudinal_distance_threshold)
}

// double trajectory_stitching_preserved_length = 4;
inline void TrajectoryStitchingConfig::clear_trajectory_stitching_preserved_length() {
  trajectory_stitching_preserved_length_ = 0;
}
inline double TrajectoryStitchingConfig::trajectory_stitching_preserved_length() const {
  // @@protoc_insertion_point(field_get:planning.TrajectoryStitchingConfig.trajectory_stitching_preserved_length)
  return trajectory_stitching_preserved_length_;
}
inline void TrajectoryStitchingConfig::set_trajectory_stitching_preserved_length(double value) {
  
  trajectory_stitching_preserved_length_ = value;
  // @@protoc_insertion_point(field_set:planning.TrajectoryStitchingConfig.trajectory_stitching_preserved_length)
}

// -------------------------------------------------------------------

// PlanningConfig

// .planning.ReferenceLineProviderConfig refline_provider_config = 1;
inline bool PlanningConfig::has_refline_provider_config() const {
  return this != internal_default_instance() && refline_provider_config_ != NULL;
}
inline const ::planning::ReferenceLineProviderConfig& PlanningConfig::_internal_refline_provider_config() const {
  return *refline_provider_config_;
}
inline const ::planning::ReferenceLineProviderConfig& PlanningConfig::refline_provider_config() const {
  const ::planning::ReferenceLineProviderConfig* p = refline_provider_config_;
  // @@protoc_insertion_point(field_get:planning.PlanningConfig.refline_provider_config)
  return p != NULL ? *p : *reinterpret_cast<const ::planning::ReferenceLineProviderConfig*>(
      &::planning::_ReferenceLineProviderConfig_default_instance_);
}
inline ::planning::ReferenceLineProviderConfig* PlanningConfig::release_refline_provider_config() {
  // @@protoc_insertion_point(field_release:planning.PlanningConfig.refline_provider_config)
  
  ::planning::ReferenceLineProviderConfig* temp = refline_provider_config_;
  refline_provider_config_ = NULL;
  return temp;
}
inline ::planning::ReferenceLineProviderConfig* PlanningConfig::mutable_refline_provider_config() {
  
  if (refline_provider_config_ == NULL) {
    auto* p = CreateMaybeMessage<::planning::ReferenceLineProviderConfig>(GetArenaNoVirtual());
    refline_provider_config_ = p;
  }
  // @@protoc_insertion_point(field_mutable:planning.PlanningConfig.refline_provider_config)
  return refline_provider_config_;
}
inline void PlanningConfig::set_allocated_refline_provider_config(::planning::ReferenceLineProviderConfig* refline_provider_config) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(refline_provider_config_);
  }
  if (refline_provider_config) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      refline_provider_config = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, refline_provider_config, submessage_arena);
    }
    
  } else {
    
  }
  refline_provider_config_ = refline_provider_config;
  // @@protoc_insertion_point(field_set_allocated:planning.PlanningConfig.refline_provider_config)
}

// .planning.FpPathPlanConfig fp_path_plan_config = 2;
inline bool PlanningConfig::has_fp_path_plan_config() const {
  return this != internal_default_instance() && fp_path_plan_config_ != NULL;
}
inline const ::planning::FpPathPlanConfig& PlanningConfig::_internal_fp_path_plan_config() const {
  return *fp_path_plan_config_;
}
inline const ::planning::FpPathPlanConfig& PlanningConfig::fp_path_plan_config() const {
  const ::planning::FpPathPlanConfig* p = fp_path_plan_config_;
  // @@protoc_insertion_point(field_get:planning.PlanningConfig.fp_path_plan_config)
  return p != NULL ? *p : *reinterpret_cast<const ::planning::FpPathPlanConfig*>(
      &::planning::_FpPathPlanConfig_default_instance_);
}
inline ::planning::FpPathPlanConfig* PlanningConfig::release_fp_path_plan_config() {
  // @@protoc_insertion_point(field_release:planning.PlanningConfig.fp_path_plan_config)
  
  ::planning::FpPathPlanConfig* temp = fp_path_plan_config_;
  fp_path_plan_config_ = NULL;
  return temp;
}
inline ::planning::FpPathPlanConfig* PlanningConfig::mutable_fp_path_plan_config() {
  
  if (fp_path_plan_config_ == NULL) {
    auto* p = CreateMaybeMessage<::planning::FpPathPlanConfig>(GetArenaNoVirtual());
    fp_path_plan_config_ = p;
  }
  // @@protoc_insertion_point(field_mutable:planning.PlanningConfig.fp_path_plan_config)
  return fp_path_plan_config_;
}
inline void PlanningConfig::set_allocated_fp_path_plan_config(::planning::FpPathPlanConfig* fp_path_plan_config) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(fp_path_plan_config_);
  }
  if (fp_path_plan_config) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      fp_path_plan_config = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, fp_path_plan_config, submessage_arena);
    }
    
  } else {
    
  }
  fp_path_plan_config_ = fp_path_plan_config;
  // @@protoc_insertion_point(field_set_allocated:planning.PlanningConfig.fp_path_plan_config)
}

// .perception.OgmConfig ogm_config = 3;
inline bool PlanningConfig::has_ogm_config() const {
  return this != internal_default_instance() && ogm_config_ != NULL;
}
inline const ::perception::OgmConfig& PlanningConfig::_internal_ogm_config() const {
  return *ogm_config_;
}
inline const ::perception::OgmConfig& PlanningConfig::ogm_config() const {
  const ::perception::OgmConfig* p = ogm_config_;
  // @@protoc_insertion_point(field_get:planning.PlanningConfig.ogm_config)
  return p != NULL ? *p : *reinterpret_cast<const ::perception::OgmConfig*>(
      &::perception::_OgmConfig_default_instance_);
}
inline ::perception::OgmConfig* PlanningConfig::release_ogm_config() {
  // @@protoc_insertion_point(field_release:planning.PlanningConfig.ogm_config)
  
  ::perception::OgmConfig* temp = ogm_config_;
  ogm_config_ = NULL;
  return temp;
}
inline ::perception::OgmConfig* PlanningConfig::mutable_ogm_config() {
  
  if (ogm_config_ == NULL) {
    auto* p = CreateMaybeMessage<::perception::OgmConfig>(GetArenaNoVirtual());
    ogm_config_ = p;
  }
  // @@protoc_insertion_point(field_mutable:planning.PlanningConfig.ogm_config)
  return ogm_config_;
}
inline void PlanningConfig::set_allocated_ogm_config(::perception::OgmConfig* ogm_config) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(ogm_config_);
  }
  if (ogm_config) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      ogm_config = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, ogm_config, submessage_arena);
    }
    
  } else {
    
  }
  ogm_config_ = ogm_config;
  // @@protoc_insertion_point(field_set_allocated:planning.PlanningConfig.ogm_config)
}

// .planning.PolySpeedPlanConfig poly_speed_plan_config = 4;
inline bool PlanningConfig::has_poly_speed_plan_config() const {
  return this != internal_default_instance() && poly_speed_plan_config_ != NULL;
}
inline const ::planning::PolySpeedPlanConfig& PlanningConfig::_internal_poly_speed_plan_config() const {
  return *poly_speed_plan_config_;
}
inline const ::planning::PolySpeedPlanConfig& PlanningConfig::poly_speed_plan_config() const {
  const ::planning::PolySpeedPlanConfig* p = poly_speed_plan_config_;
  // @@protoc_insertion_point(field_get:planning.PlanningConfig.poly_speed_plan_config)
  return p != NULL ? *p : *reinterpret_cast<const ::planning::PolySpeedPlanConfig*>(
      &::planning::_PolySpeedPlanConfig_default_instance_);
}
inline ::planning::PolySpeedPlanConfig* PlanningConfig::release_poly_speed_plan_config() {
  // @@protoc_insertion_point(field_release:planning.PlanningConfig.poly_speed_plan_config)
  
  ::planning::PolySpeedPlanConfig* temp = poly_speed_plan_config_;
  poly_speed_plan_config_ = NULL;
  return temp;
}
inline ::planning::PolySpeedPlanConfig* PlanningConfig::mutable_poly_speed_plan_config() {
  
  if (poly_speed_plan_config_ == NULL) {
    auto* p = CreateMaybeMessage<::planning::PolySpeedPlanConfig>(GetArenaNoVirtual());
    poly_speed_plan_config_ = p;
  }
  // @@protoc_insertion_point(field_mutable:planning.PlanningConfig.poly_speed_plan_config)
  return poly_speed_plan_config_;
}
inline void PlanningConfig::set_allocated_poly_speed_plan_config(::planning::PolySpeedPlanConfig* poly_speed_plan_config) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(poly_speed_plan_config_);
  }
  if (poly_speed_plan_config) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      poly_speed_plan_config = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, poly_speed_plan_config, submessage_arena);
    }
    
  } else {
    
  }
  poly_speed_plan_config_ = poly_speed_plan_config;
  // @@protoc_insertion_point(field_set_allocated:planning.PlanningConfig.poly_speed_plan_config)
}

// .planning.TrajectoryStitchingConfig traj_stitching_config = 5;
inline bool PlanningConfig::has_traj_stitching_config() const {
  return this != internal_default_instance() && traj_stitching_config_ != NULL;
}
inline void PlanningConfig::clear_traj_stitching_config() {
  if (GetArenaNoVirtual() == NULL && traj_stitching_config_ != NULL) {
    delete traj_stitching_config_;
  }
  traj_stitching_config_ = NULL;
}
inline const ::planning::TrajectoryStitchingConfig& PlanningConfig::_internal_traj_stitching_config() const {
  return *traj_stitching_config_;
}
inline const ::planning::TrajectoryStitchingConfig& PlanningConfig::traj_stitching_config() const {
  const ::planning::TrajectoryStitchingConfig* p = traj_stitching_config_;
  // @@protoc_insertion_point(field_get:planning.PlanningConfig.traj_stitching_config)
  return p != NULL ? *p : *reinterpret_cast<const ::planning::TrajectoryStitchingConfig*>(
      &::planning::_TrajectoryStitchingConfig_default_instance_);
}
inline ::planning::TrajectoryStitchingConfig* PlanningConfig::release_traj_stitching_config() {
  // @@protoc_insertion_point(field_release:planning.PlanningConfig.traj_stitching_config)
  
  ::planning::TrajectoryStitchingConfig* temp = traj_stitching_config_;
  traj_stitching_config_ = NULL;
  return temp;
}
inline ::planning::TrajectoryStitchingConfig* PlanningConfig::mutable_traj_stitching_config() {
  
  if (traj_stitching_config_ == NULL) {
    auto* p = CreateMaybeMessage<::planning::TrajectoryStitchingConfig>(GetArenaNoVirtual());
    traj_stitching_config_ = p;
  }
  // @@protoc_insertion_point(field_mutable:planning.PlanningConfig.traj_stitching_config)
  return traj_stitching_config_;
}
inline void PlanningConfig::set_allocated_traj_stitching_config(::planning::TrajectoryStitchingConfig* traj_stitching_config) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete traj_stitching_config_;
  }
  if (traj_stitching_config) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      traj_stitching_config = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, traj_stitching_config, submessage_arena);
    }
    
  } else {
    
  }
  traj_stitching_config_ = traj_stitching_config;
  // @@protoc_insertion_point(field_set_allocated:planning.PlanningConfig.traj_stitching_config)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace planning

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_planning_2fproto_2fplanning_5fconfig_2eproto
