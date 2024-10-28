// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: magnetic_field.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_magnetic_5ffield_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_magnetic_5ffield_2eproto

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
#include "header.pb.h"
#include "vector3.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_magnetic_5ffield_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_magnetic_5ffield_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_magnetic_5ffield_2eproto;
namespace synapse {
namespace msgs {
class MagneticField;
class MagneticFieldDefaultTypeInternal;
extern MagneticFieldDefaultTypeInternal _MagneticField_default_instance_;
}  // namespace msgs
}  // namespace synapse
PROTOBUF_NAMESPACE_OPEN
template<> ::synapse::msgs::MagneticField* Arena::CreateMaybeMessage<::synapse::msgs::MagneticField>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace synapse {
namespace msgs {

// ===================================================================

class MagneticField PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:synapse.msgs.MagneticField) */ {
 public:
  inline MagneticField() : MagneticField(nullptr) {};
  virtual ~MagneticField();

  MagneticField(const MagneticField& from);
  MagneticField(MagneticField&& from) noexcept
    : MagneticField() {
    *this = ::std::move(from);
  }

  inline MagneticField& operator=(const MagneticField& from) {
    CopyFrom(from);
    return *this;
  }
  inline MagneticField& operator=(MagneticField&& from) noexcept {
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
  static const MagneticField& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MagneticField* internal_default_instance() {
    return reinterpret_cast<const MagneticField*>(
               &_MagneticField_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(MagneticField& a, MagneticField& b) {
    a.Swap(&b);
  }
  inline void Swap(MagneticField* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(MagneticField* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline MagneticField* New() const final {
    return CreateMaybeMessage<MagneticField>(nullptr);
  }

  MagneticField* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<MagneticField>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const MagneticField& from);
  void MergeFrom(const MagneticField& from);
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
  void InternalSwap(MagneticField* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "synapse.msgs.MagneticField";
  }
  protected:
  explicit MagneticField(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_magnetic_5ffield_2eproto);
    return ::descriptor_table_magnetic_5ffield_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMagneticFieldCovarianceFieldNumber = 3,
    kHeaderFieldNumber = 1,
    kMagneticFieldFieldNumber = 2,
  };
  // repeated double magnetic_field_covariance = 3;
  int magnetic_field_covariance_size() const;
  private:
  int _internal_magnetic_field_covariance_size() const;
  public:
  void clear_magnetic_field_covariance();
  private:
  double _internal_magnetic_field_covariance(int index) const;
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      _internal_magnetic_field_covariance() const;
  void _internal_add_magnetic_field_covariance(double value);
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      _internal_mutable_magnetic_field_covariance();
  public:
  double magnetic_field_covariance(int index) const;
  void set_magnetic_field_covariance(int index, double value);
  void add_magnetic_field_covariance(double value);
  const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
      magnetic_field_covariance() const;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
      mutable_magnetic_field_covariance();

  // .synapse.msgs.Header header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::synapse::msgs::Header& header() const;
  ::synapse::msgs::Header* release_header();
  ::synapse::msgs::Header* mutable_header();
  void set_allocated_header(::synapse::msgs::Header* header);
  private:
  const ::synapse::msgs::Header& _internal_header() const;
  ::synapse::msgs::Header* _internal_mutable_header();
  public:
  void unsafe_arena_set_allocated_header(
      ::synapse::msgs::Header* header);
  ::synapse::msgs::Header* unsafe_arena_release_header();

  // .synapse.msgs.Vector3 magnetic_field = 2;
  bool has_magnetic_field() const;
  private:
  bool _internal_has_magnetic_field() const;
  public:
  void clear_magnetic_field();
  const ::synapse::msgs::Vector3& magnetic_field() const;
  ::synapse::msgs::Vector3* release_magnetic_field();
  ::synapse::msgs::Vector3* mutable_magnetic_field();
  void set_allocated_magnetic_field(::synapse::msgs::Vector3* magnetic_field);
  private:
  const ::synapse::msgs::Vector3& _internal_magnetic_field() const;
  ::synapse::msgs::Vector3* _internal_mutable_magnetic_field();
  public:
  void unsafe_arena_set_allocated_magnetic_field(
      ::synapse::msgs::Vector3* magnetic_field);
  ::synapse::msgs::Vector3* unsafe_arena_release_magnetic_field();

  // @@protoc_insertion_point(class_scope:synapse.msgs.MagneticField)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedField< double > magnetic_field_covariance_;
  mutable std::atomic<int> _magnetic_field_covariance_cached_byte_size_;
  ::synapse::msgs::Header* header_;
  ::synapse::msgs::Vector3* magnetic_field_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_magnetic_5ffield_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MagneticField

// .synapse.msgs.Header header = 1;
inline bool MagneticField::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool MagneticField::has_header() const {
  return _internal_has_header();
}
inline const ::synapse::msgs::Header& MagneticField::_internal_header() const {
  const ::synapse::msgs::Header* p = header_;
  return p != nullptr ? *p : *reinterpret_cast<const ::synapse::msgs::Header*>(
      &::synapse::msgs::_Header_default_instance_);
}
inline const ::synapse::msgs::Header& MagneticField::header() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.MagneticField.header)
  return _internal_header();
}
inline void MagneticField::unsafe_arena_set_allocated_header(
    ::synapse::msgs::Header* header) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:synapse.msgs.MagneticField.header)
}
inline ::synapse::msgs::Header* MagneticField::release_header() {
  auto temp = unsafe_arena_release_header();
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::synapse::msgs::Header* MagneticField::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:synapse.msgs.MagneticField.header)
  
  ::synapse::msgs::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::synapse::msgs::Header* MagneticField::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::synapse::msgs::Header>(GetArena());
    header_ = p;
  }
  return header_;
}
inline ::synapse::msgs::Header* MagneticField::mutable_header() {
  // @@protoc_insertion_point(field_mutable:synapse.msgs.MagneticField.header)
  return _internal_mutable_header();
}
inline void MagneticField::set_allocated_header(::synapse::msgs::Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header)->GetArena();
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:synapse.msgs.MagneticField.header)
}

// .synapse.msgs.Vector3 magnetic_field = 2;
inline bool MagneticField::_internal_has_magnetic_field() const {
  return this != internal_default_instance() && magnetic_field_ != nullptr;
}
inline bool MagneticField::has_magnetic_field() const {
  return _internal_has_magnetic_field();
}
inline const ::synapse::msgs::Vector3& MagneticField::_internal_magnetic_field() const {
  const ::synapse::msgs::Vector3* p = magnetic_field_;
  return p != nullptr ? *p : *reinterpret_cast<const ::synapse::msgs::Vector3*>(
      &::synapse::msgs::_Vector3_default_instance_);
}
inline const ::synapse::msgs::Vector3& MagneticField::magnetic_field() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.MagneticField.magnetic_field)
  return _internal_magnetic_field();
}
inline void MagneticField::unsafe_arena_set_allocated_magnetic_field(
    ::synapse::msgs::Vector3* magnetic_field) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(magnetic_field_);
  }
  magnetic_field_ = magnetic_field;
  if (magnetic_field) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:synapse.msgs.MagneticField.magnetic_field)
}
inline ::synapse::msgs::Vector3* MagneticField::release_magnetic_field() {
  auto temp = unsafe_arena_release_magnetic_field();
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::synapse::msgs::Vector3* MagneticField::unsafe_arena_release_magnetic_field() {
  // @@protoc_insertion_point(field_release:synapse.msgs.MagneticField.magnetic_field)
  
  ::synapse::msgs::Vector3* temp = magnetic_field_;
  magnetic_field_ = nullptr;
  return temp;
}
inline ::synapse::msgs::Vector3* MagneticField::_internal_mutable_magnetic_field() {
  
  if (magnetic_field_ == nullptr) {
    auto* p = CreateMaybeMessage<::synapse::msgs::Vector3>(GetArena());
    magnetic_field_ = p;
  }
  return magnetic_field_;
}
inline ::synapse::msgs::Vector3* MagneticField::mutable_magnetic_field() {
  // @@protoc_insertion_point(field_mutable:synapse.msgs.MagneticField.magnetic_field)
  return _internal_mutable_magnetic_field();
}
inline void MagneticField::set_allocated_magnetic_field(::synapse::msgs::Vector3* magnetic_field) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(magnetic_field_);
  }
  if (magnetic_field) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(magnetic_field)->GetArena();
    if (message_arena != submessage_arena) {
      magnetic_field = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, magnetic_field, submessage_arena);
    }
    
  } else {
    
  }
  magnetic_field_ = magnetic_field;
  // @@protoc_insertion_point(field_set_allocated:synapse.msgs.MagneticField.magnetic_field)
}

// repeated double magnetic_field_covariance = 3;
inline int MagneticField::_internal_magnetic_field_covariance_size() const {
  return magnetic_field_covariance_.size();
}
inline int MagneticField::magnetic_field_covariance_size() const {
  return _internal_magnetic_field_covariance_size();
}
inline void MagneticField::clear_magnetic_field_covariance() {
  magnetic_field_covariance_.Clear();
}
inline double MagneticField::_internal_magnetic_field_covariance(int index) const {
  return magnetic_field_covariance_.Get(index);
}
inline double MagneticField::magnetic_field_covariance(int index) const {
  // @@protoc_insertion_point(field_get:synapse.msgs.MagneticField.magnetic_field_covariance)
  return _internal_magnetic_field_covariance(index);
}
inline void MagneticField::set_magnetic_field_covariance(int index, double value) {
  magnetic_field_covariance_.Set(index, value);
  // @@protoc_insertion_point(field_set:synapse.msgs.MagneticField.magnetic_field_covariance)
}
inline void MagneticField::_internal_add_magnetic_field_covariance(double value) {
  magnetic_field_covariance_.Add(value);
}
inline void MagneticField::add_magnetic_field_covariance(double value) {
  _internal_add_magnetic_field_covariance(value);
  // @@protoc_insertion_point(field_add:synapse.msgs.MagneticField.magnetic_field_covariance)
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
MagneticField::_internal_magnetic_field_covariance() const {
  return magnetic_field_covariance_;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >&
MagneticField::magnetic_field_covariance() const {
  // @@protoc_insertion_point(field_list:synapse.msgs.MagneticField.magnetic_field_covariance)
  return _internal_magnetic_field_covariance();
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
MagneticField::_internal_mutable_magnetic_field_covariance() {
  return &magnetic_field_covariance_;
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedField< double >*
MagneticField::mutable_magnetic_field_covariance() {
  // @@protoc_insertion_point(field_mutable_list:synapse.msgs.MagneticField.magnetic_field_covariance)
  return _internal_mutable_magnetic_field_covariance();
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace synapse

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_magnetic_5ffield_2eproto
