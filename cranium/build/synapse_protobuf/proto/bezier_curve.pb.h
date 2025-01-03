// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: bezier_curve.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_bezier_5fcurve_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_bezier_5fcurve_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3012000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3012004 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_bezier_5fcurve_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_bezier_5fcurve_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_bezier_5fcurve_2eproto;
namespace synapse {
namespace msgs {
class BezierCurve;
class BezierCurveDefaultTypeInternal;
extern BezierCurveDefaultTypeInternal _BezierCurve_default_instance_;
}  // namespace msgs
}  // namespace synapse
PROTOBUF_NAMESPACE_OPEN
template<> ::synapse::msgs::BezierCurve* Arena::CreateMaybeMessage<::synapse::msgs::BezierCurve>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace synapse {
namespace msgs {

// ===================================================================

class BezierCurve PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:synapse.msgs.BezierCurve) */ {
 public:
  inline BezierCurve() : BezierCurve(nullptr) {};
  virtual ~BezierCurve();

  BezierCurve(const BezierCurve& from);
  BezierCurve(BezierCurve&& from) noexcept
    : BezierCurve() {
    *this = ::std::move(from);
  }

  inline BezierCurve& operator=(const BezierCurve& from) {
    CopyFrom(from);
    return *this;
  }
  inline BezierCurve& operator=(BezierCurve&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const BezierCurve& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const BezierCurve* internal_default_instance() {
    return reinterpret_cast<const BezierCurve*>(
               &_BezierCurve_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(BezierCurve& a, BezierCurve& b) {
    a.Swap(&b);
  }
  inline void Swap(BezierCurve* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(BezierCurve* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline BezierCurve* New() const final {
    return CreateMaybeMessage<BezierCurve>(nullptr);
  }

  BezierCurve* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<BezierCurve>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const BezierCurve& from);
  void MergeFrom(const BezierCurve& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(BezierCurve* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "synapse.msgs.BezierCurve";
  }
  protected:
  explicit BezierCurve(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_bezier_5fcurve_2eproto);
    return ::descriptor_table_bezier_5fcurve_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kXFieldNumber = 2,
    kYFieldNumber = 3,
    kZFieldNumber = 4,
    kYawFieldNumber = 5,
    kTimeStopFieldNumber = 1,
  };
  // repeated double x = 2;
  int x_size() const;
  private:
  int _internal_x_size() const;
  public:
  void clear_x();
  private:
  double _internal_x(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_x() const;
  void _internal_add_x(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_x();
  public:
  double x(int index) const;
  void set_x(int index, double value);
  void add_x(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      x() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_x();

  // repeated double y = 3;
  int y_size() const;
  private:
  int _internal_y_size() const;
  public:
  void clear_y();
  private:
  double _internal_y(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_y() const;
  void _internal_add_y(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_y();
  public:
  double y(int index) const;
  void set_y(int index, double value);
  void add_y(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      y() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_y();

  // repeated double z = 4;
  int z_size() const;
  private:
  int _internal_z_size() const;
  public:
  void clear_z();
  private:
  double _internal_z(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_z() const;
  void _internal_add_z(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_z();
  public:
  double z(int index) const;
  void set_z(int index, double value);
  void add_z(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      z() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_z();

  // repeated double yaw = 5;
  int yaw_size() const;
  private:
  int _internal_yaw_size() const;
  public:
  void clear_yaw();
  private:
  double _internal_yaw(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_yaw() const;
  void _internal_add_yaw(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_yaw();
  public:
  double yaw(int index) const;
  void set_yaw(int index, double value);
  void add_yaw(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      yaw() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_yaw();

  // uint64 time_stop = 1;
  void clear_time_stop();
  ::PROTOBUF_NAMESPACE_ID::uint64 time_stop() const;
  void set_time_stop(::PROTOBUF_NAMESPACE_ID::uint64 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint64 _internal_time_stop() const;
  void _internal_set_time_stop(::PROTOBUF_NAMESPACE_ID::uint64 value);
  public:

  // @@protoc_insertion_point(class_scope:synapse.msgs.BezierCurve)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > x_;
  mutable std::atomic<int> _x_cached_byte_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > y_;
  mutable std::atomic<int> _y_cached_byte_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > z_;
  mutable std::atomic<int> _z_cached_byte_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > yaw_;
  mutable std::atomic<int> _yaw_cached_byte_size_;
  ::PROTOBUF_NAMESPACE_ID::uint64 time_stop_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_bezier_5fcurve_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// BezierCurve

// uint64 time_stop = 1;
inline void BezierCurve::clear_time_stop() {
  time_stop_ = PROTOBUF_ULONGLONG(0);
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 BezierCurve::_internal_time_stop() const {
  return time_stop_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint64 BezierCurve::time_stop() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.BezierCurve.time_stop)
  return _internal_time_stop();
}
inline void BezierCurve::_internal_set_time_stop(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  
  time_stop_ = value;
}
inline void BezierCurve::set_time_stop(::PROTOBUF_NAMESPACE_ID::uint64 value) {
  _internal_set_time_stop(value);
  // @@protoc_insertion_point(field_set:synapse.msgs.BezierCurve.time_stop)
}

// repeated double x = 2;
inline int BezierCurve::_internal_x_size() const {
  return x_.size();
}
inline int BezierCurve::x_size() const {
  return _internal_x_size();
}
inline void BezierCurve::clear_x() {
  x_.Clear();
}
inline double BezierCurve::_internal_x(int index) const {
  return x_.Get(index);
}
inline double BezierCurve::x(int index) const {
  // @@protoc_insertion_point(field_get:synapse.msgs.BezierCurve.x)
  return _internal_x(index);
}
inline void BezierCurve::set_x(int index, double value) {
  x_.Set(index, value);
  // @@protoc_insertion_point(field_set:synapse.msgs.BezierCurve.x)
}
inline void BezierCurve::_internal_add_x(double value) {
  x_.Add(value);
}
inline void BezierCurve::add_x(double value) {
  _internal_add_x(value);
  // @@protoc_insertion_point(field_add:synapse.msgs.BezierCurve.x)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
BezierCurve::_internal_x() const {
  return x_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
BezierCurve::x() const {
  // @@protoc_insertion_point(field_list:synapse.msgs.BezierCurve.x)
  return _internal_x();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
BezierCurve::_internal_mutable_x() {
  return &x_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
BezierCurve::mutable_x() {
  // @@protoc_insertion_point(field_mutable_list:synapse.msgs.BezierCurve.x)
  return _internal_mutable_x();
}

// repeated double y = 3;
inline int BezierCurve::_internal_y_size() const {
  return y_.size();
}
inline int BezierCurve::y_size() const {
  return _internal_y_size();
}
inline void BezierCurve::clear_y() {
  y_.Clear();
}
inline double BezierCurve::_internal_y(int index) const {
  return y_.Get(index);
}
inline double BezierCurve::y(int index) const {
  // @@protoc_insertion_point(field_get:synapse.msgs.BezierCurve.y)
  return _internal_y(index);
}
inline void BezierCurve::set_y(int index, double value) {
  y_.Set(index, value);
  // @@protoc_insertion_point(field_set:synapse.msgs.BezierCurve.y)
}
inline void BezierCurve::_internal_add_y(double value) {
  y_.Add(value);
}
inline void BezierCurve::add_y(double value) {
  _internal_add_y(value);
  // @@protoc_insertion_point(field_add:synapse.msgs.BezierCurve.y)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
BezierCurve::_internal_y() const {
  return y_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
BezierCurve::y() const {
  // @@protoc_insertion_point(field_list:synapse.msgs.BezierCurve.y)
  return _internal_y();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
BezierCurve::_internal_mutable_y() {
  return &y_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
BezierCurve::mutable_y() {
  // @@protoc_insertion_point(field_mutable_list:synapse.msgs.BezierCurve.y)
  return _internal_mutable_y();
}

// repeated double z = 4;
inline int BezierCurve::_internal_z_size() const {
  return z_.size();
}
inline int BezierCurve::z_size() const {
  return _internal_z_size();
}
inline void BezierCurve::clear_z() {
  z_.Clear();
}
inline double BezierCurve::_internal_z(int index) const {
  return z_.Get(index);
}
inline double BezierCurve::z(int index) const {
  // @@protoc_insertion_point(field_get:synapse.msgs.BezierCurve.z)
  return _internal_z(index);
}
inline void BezierCurve::set_z(int index, double value) {
  z_.Set(index, value);
  // @@protoc_insertion_point(field_set:synapse.msgs.BezierCurve.z)
}
inline void BezierCurve::_internal_add_z(double value) {
  z_.Add(value);
}
inline void BezierCurve::add_z(double value) {
  _internal_add_z(value);
  // @@protoc_insertion_point(field_add:synapse.msgs.BezierCurve.z)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
BezierCurve::_internal_z() const {
  return z_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
BezierCurve::z() const {
  // @@protoc_insertion_point(field_list:synapse.msgs.BezierCurve.z)
  return _internal_z();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
BezierCurve::_internal_mutable_z() {
  return &z_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
BezierCurve::mutable_z() {
  // @@protoc_insertion_point(field_mutable_list:synapse.msgs.BezierCurve.z)
  return _internal_mutable_z();
}

// repeated double yaw = 5;
inline int BezierCurve::_internal_yaw_size() const {
  return yaw_.size();
}
inline int BezierCurve::yaw_size() const {
  return _internal_yaw_size();
}
inline void BezierCurve::clear_yaw() {
  yaw_.Clear();
}
inline double BezierCurve::_internal_yaw(int index) const {
  return yaw_.Get(index);
}
inline double BezierCurve::yaw(int index) const {
  // @@protoc_insertion_point(field_get:synapse.msgs.BezierCurve.yaw)
  return _internal_yaw(index);
}
inline void BezierCurve::set_yaw(int index, double value) {
  yaw_.Set(index, value);
  // @@protoc_insertion_point(field_set:synapse.msgs.BezierCurve.yaw)
}
inline void BezierCurve::_internal_add_yaw(double value) {
  yaw_.Add(value);
}
inline void BezierCurve::add_yaw(double value) {
  _internal_add_yaw(value);
  // @@protoc_insertion_point(field_add:synapse.msgs.BezierCurve.yaw)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
BezierCurve::_internal_yaw() const {
  return yaw_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
BezierCurve::yaw() const {
  // @@protoc_insertion_point(field_list:synapse.msgs.BezierCurve.yaw)
  return _internal_yaw();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
BezierCurve::_internal_mutable_yaw() {
  return &yaw_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
BezierCurve::mutable_yaw() {
  // @@protoc_insertion_point(field_mutable_list:synapse.msgs.BezierCurve.yaw)
  return _internal_mutable_yaw();
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace synapse

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_bezier_5fcurve_2eproto
