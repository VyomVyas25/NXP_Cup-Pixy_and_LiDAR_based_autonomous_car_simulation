// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: pixy_vector.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_pixy_5fvector_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_pixy_5fvector_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_pixy_5fvector_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_pixy_5fvector_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_pixy_5fvector_2eproto;
namespace synapse {
namespace msgs {
class PixyVector;
class PixyVectorDefaultTypeInternal;
extern PixyVectorDefaultTypeInternal _PixyVector_default_instance_;
}  // namespace msgs
}  // namespace synapse
PROTOBUF_NAMESPACE_OPEN
template<> ::synapse::msgs::PixyVector* Arena::CreateMaybeMessage<::synapse::msgs::PixyVector>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace synapse {
namespace msgs {

// ===================================================================

class PixyVector PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:synapse.msgs.PixyVector) */ {
 public:
  inline PixyVector() : PixyVector(nullptr) {};
  virtual ~PixyVector();

  PixyVector(const PixyVector& from);
  PixyVector(PixyVector&& from) noexcept
    : PixyVector() {
    *this = ::std::move(from);
  }

  inline PixyVector& operator=(const PixyVector& from) {
    CopyFrom(from);
    return *this;
  }
  inline PixyVector& operator=(PixyVector&& from) noexcept {
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
  static const PixyVector& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PixyVector* internal_default_instance() {
    return reinterpret_cast<const PixyVector*>(
               &_PixyVector_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(PixyVector& a, PixyVector& b) {
    a.Swap(&b);
  }
  inline void Swap(PixyVector* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(PixyVector* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline PixyVector* New() const final {
    return CreateMaybeMessage<PixyVector>(nullptr);
  }

  PixyVector* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<PixyVector>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const PixyVector& from);
  void MergeFrom(const PixyVector& from);
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
  void InternalSwap(PixyVector* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "synapse.msgs.PixyVector";
  }
  protected:
  explicit PixyVector(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_pixy_5fvector_2eproto);
    return ::descriptor_table_pixy_5fvector_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kHeaderFieldNumber = 1,
    kM0X0FieldNumber = 2,
    kM0Y0FieldNumber = 3,
    kM0X1FieldNumber = 4,
    kM0Y1FieldNumber = 5,
    kM1X0FieldNumber = 6,
    kM1Y0FieldNumber = 7,
    kM1X1FieldNumber = 8,
    kM1Y1FieldNumber = 9,
  };
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

  // uint32 m0_x0 = 2;
  void clear_m0_x0();
  ::PROTOBUF_NAMESPACE_ID::uint32 m0_x0() const;
  void set_m0_x0(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_m0_x0() const;
  void _internal_set_m0_x0(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // uint32 m0_y0 = 3;
  void clear_m0_y0();
  ::PROTOBUF_NAMESPACE_ID::uint32 m0_y0() const;
  void set_m0_y0(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_m0_y0() const;
  void _internal_set_m0_y0(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // uint32 m0_x1 = 4;
  void clear_m0_x1();
  ::PROTOBUF_NAMESPACE_ID::uint32 m0_x1() const;
  void set_m0_x1(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_m0_x1() const;
  void _internal_set_m0_x1(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // uint32 m0_y1 = 5;
  void clear_m0_y1();
  ::PROTOBUF_NAMESPACE_ID::uint32 m0_y1() const;
  void set_m0_y1(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_m0_y1() const;
  void _internal_set_m0_y1(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // uint32 m1_x0 = 6;
  void clear_m1_x0();
  ::PROTOBUF_NAMESPACE_ID::uint32 m1_x0() const;
  void set_m1_x0(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_m1_x0() const;
  void _internal_set_m1_x0(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // uint32 m1_y0 = 7;
  void clear_m1_y0();
  ::PROTOBUF_NAMESPACE_ID::uint32 m1_y0() const;
  void set_m1_y0(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_m1_y0() const;
  void _internal_set_m1_y0(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // uint32 m1_x1 = 8;
  void clear_m1_x1();
  ::PROTOBUF_NAMESPACE_ID::uint32 m1_x1() const;
  void set_m1_x1(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_m1_x1() const;
  void _internal_set_m1_x1(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // uint32 m1_y1 = 9;
  void clear_m1_y1();
  ::PROTOBUF_NAMESPACE_ID::uint32 m1_y1() const;
  void set_m1_y1(::PROTOBUF_NAMESPACE_ID::uint32 value);
  private:
  ::PROTOBUF_NAMESPACE_ID::uint32 _internal_m1_y1() const;
  void _internal_set_m1_y1(::PROTOBUF_NAMESPACE_ID::uint32 value);
  public:

  // @@protoc_insertion_point(class_scope:synapse.msgs.PixyVector)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::synapse::msgs::Header* header_;
  ::PROTOBUF_NAMESPACE_ID::uint32 m0_x0_;
  ::PROTOBUF_NAMESPACE_ID::uint32 m0_y0_;
  ::PROTOBUF_NAMESPACE_ID::uint32 m0_x1_;
  ::PROTOBUF_NAMESPACE_ID::uint32 m0_y1_;
  ::PROTOBUF_NAMESPACE_ID::uint32 m1_x0_;
  ::PROTOBUF_NAMESPACE_ID::uint32 m1_y0_;
  ::PROTOBUF_NAMESPACE_ID::uint32 m1_x1_;
  ::PROTOBUF_NAMESPACE_ID::uint32 m1_y1_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_pixy_5fvector_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PixyVector

// .synapse.msgs.Header header = 1;
inline bool PixyVector::_internal_has_header() const {
  return this != internal_default_instance() && header_ != nullptr;
}
inline bool PixyVector::has_header() const {
  return _internal_has_header();
}
inline const ::synapse::msgs::Header& PixyVector::_internal_header() const {
  const ::synapse::msgs::Header* p = header_;
  return p != nullptr ? *p : *reinterpret_cast<const ::synapse::msgs::Header*>(
      &::synapse::msgs::_Header_default_instance_);
}
inline const ::synapse::msgs::Header& PixyVector::header() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.PixyVector.header)
  return _internal_header();
}
inline void PixyVector::unsafe_arena_set_allocated_header(
    ::synapse::msgs::Header* header) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:synapse.msgs.PixyVector.header)
}
inline ::synapse::msgs::Header* PixyVector::release_header() {
  auto temp = unsafe_arena_release_header();
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::synapse::msgs::Header* PixyVector::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:synapse.msgs.PixyVector.header)
  
  ::synapse::msgs::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::synapse::msgs::Header* PixyVector::_internal_mutable_header() {
  
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::synapse::msgs::Header>(GetArena());
    header_ = p;
  }
  return header_;
}
inline ::synapse::msgs::Header* PixyVector::mutable_header() {
  // @@protoc_insertion_point(field_mutable:synapse.msgs.PixyVector.header)
  return _internal_mutable_header();
}
inline void PixyVector::set_allocated_header(::synapse::msgs::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:synapse.msgs.PixyVector.header)
}

// uint32 m0_x0 = 2;
inline void PixyVector::clear_m0_x0() {
  m0_x0_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::_internal_m0_x0() const {
  return m0_x0_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::m0_x0() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.PixyVector.m0_x0)
  return _internal_m0_x0();
}
inline void PixyVector::_internal_set_m0_x0(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  
  m0_x0_ = value;
}
inline void PixyVector::set_m0_x0(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_m0_x0(value);
  // @@protoc_insertion_point(field_set:synapse.msgs.PixyVector.m0_x0)
}

// uint32 m0_y0 = 3;
inline void PixyVector::clear_m0_y0() {
  m0_y0_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::_internal_m0_y0() const {
  return m0_y0_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::m0_y0() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.PixyVector.m0_y0)
  return _internal_m0_y0();
}
inline void PixyVector::_internal_set_m0_y0(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  
  m0_y0_ = value;
}
inline void PixyVector::set_m0_y0(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_m0_y0(value);
  // @@protoc_insertion_point(field_set:synapse.msgs.PixyVector.m0_y0)
}

// uint32 m0_x1 = 4;
inline void PixyVector::clear_m0_x1() {
  m0_x1_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::_internal_m0_x1() const {
  return m0_x1_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::m0_x1() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.PixyVector.m0_x1)
  return _internal_m0_x1();
}
inline void PixyVector::_internal_set_m0_x1(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  
  m0_x1_ = value;
}
inline void PixyVector::set_m0_x1(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_m0_x1(value);
  // @@protoc_insertion_point(field_set:synapse.msgs.PixyVector.m0_x1)
}

// uint32 m0_y1 = 5;
inline void PixyVector::clear_m0_y1() {
  m0_y1_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::_internal_m0_y1() const {
  return m0_y1_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::m0_y1() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.PixyVector.m0_y1)
  return _internal_m0_y1();
}
inline void PixyVector::_internal_set_m0_y1(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  
  m0_y1_ = value;
}
inline void PixyVector::set_m0_y1(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_m0_y1(value);
  // @@protoc_insertion_point(field_set:synapse.msgs.PixyVector.m0_y1)
}

// uint32 m1_x0 = 6;
inline void PixyVector::clear_m1_x0() {
  m1_x0_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::_internal_m1_x0() const {
  return m1_x0_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::m1_x0() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.PixyVector.m1_x0)
  return _internal_m1_x0();
}
inline void PixyVector::_internal_set_m1_x0(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  
  m1_x0_ = value;
}
inline void PixyVector::set_m1_x0(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_m1_x0(value);
  // @@protoc_insertion_point(field_set:synapse.msgs.PixyVector.m1_x0)
}

// uint32 m1_y0 = 7;
inline void PixyVector::clear_m1_y0() {
  m1_y0_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::_internal_m1_y0() const {
  return m1_y0_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::m1_y0() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.PixyVector.m1_y0)
  return _internal_m1_y0();
}
inline void PixyVector::_internal_set_m1_y0(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  
  m1_y0_ = value;
}
inline void PixyVector::set_m1_y0(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_m1_y0(value);
  // @@protoc_insertion_point(field_set:synapse.msgs.PixyVector.m1_y0)
}

// uint32 m1_x1 = 8;
inline void PixyVector::clear_m1_x1() {
  m1_x1_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::_internal_m1_x1() const {
  return m1_x1_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::m1_x1() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.PixyVector.m1_x1)
  return _internal_m1_x1();
}
inline void PixyVector::_internal_set_m1_x1(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  
  m1_x1_ = value;
}
inline void PixyVector::set_m1_x1(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_m1_x1(value);
  // @@protoc_insertion_point(field_set:synapse.msgs.PixyVector.m1_x1)
}

// uint32 m1_y1 = 9;
inline void PixyVector::clear_m1_y1() {
  m1_y1_ = 0u;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::_internal_m1_y1() const {
  return m1_y1_;
}
inline ::PROTOBUF_NAMESPACE_ID::uint32 PixyVector::m1_y1() const {
  // @@protoc_insertion_point(field_get:synapse.msgs.PixyVector.m1_y1)
  return _internal_m1_y1();
}
inline void PixyVector::_internal_set_m1_y1(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  
  m1_y1_ = value;
}
inline void PixyVector::set_m1_y1(::PROTOBUF_NAMESPACE_ID::uint32 value) {
  _internal_set_m1_y1(value);
  // @@protoc_insertion_point(field_set:synapse.msgs.PixyVector.m1_y1)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace synapse

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_pixy_5fvector_2eproto
