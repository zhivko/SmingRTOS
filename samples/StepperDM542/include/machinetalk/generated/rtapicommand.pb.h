// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: rtapicommand.proto

#ifndef PROTOBUF_rtapicommand_2eproto__INCLUDED
#define PROTOBUF_rtapicommand_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
#include "nanopb.pb.h"
// @@protoc_insertion_point(includes)

namespace pb {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_rtapicommand_2eproto();
void protobuf_AssignDesc_rtapicommand_2eproto();
void protobuf_ShutdownFile_rtapicommand_2eproto();

class RTAPICommand;

// ===================================================================

class RTAPICommand : public ::google::protobuf::Message {
 public:
  RTAPICommand();
  virtual ~RTAPICommand();

  RTAPICommand(const RTAPICommand& from);

  inline RTAPICommand& operator=(const RTAPICommand& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const RTAPICommand& default_instance();

  void Swap(RTAPICommand* other);

  // implements Message ----------------------------------------------

  RTAPICommand* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const RTAPICommand& from);
  void MergeFrom(const RTAPICommand& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required int32 instance = 1;
  inline bool has_instance() const;
  inline void clear_instance();
  static const int kInstanceFieldNumber = 1;
  inline ::google::protobuf::int32 instance() const;
  inline void set_instance(::google::protobuf::int32 value);

  // optional string modname = 2;
  inline bool has_modname() const;
  inline void clear_modname();
  static const int kModnameFieldNumber = 2;
  inline const ::std::string& modname() const;
  inline void set_modname(const ::std::string& value);
  inline void set_modname(const char* value);
  inline void set_modname(const char* value, size_t size);
  inline ::std::string* mutable_modname();
  inline ::std::string* release_modname();
  inline void set_allocated_modname(::std::string* modname);

  // optional int32 rt_msglevel = 3;
  inline bool has_rt_msglevel() const;
  inline void clear_rt_msglevel();
  static const int kRtMsglevelFieldNumber = 3;
  inline ::google::protobuf::int32 rt_msglevel() const;
  inline void set_rt_msglevel(::google::protobuf::int32 value);

  // optional int32 user_msglevel = 4;
  inline bool has_user_msglevel() const;
  inline void clear_user_msglevel();
  static const int kUserMsglevelFieldNumber = 4;
  inline ::google::protobuf::int32 user_msglevel() const;
  inline void set_user_msglevel(::google::protobuf::int32 value);

  // repeated string argv = 5;
  inline int argv_size() const;
  inline void clear_argv();
  static const int kArgvFieldNumber = 5;
  inline const ::std::string& argv(int index) const;
  inline ::std::string* mutable_argv(int index);
  inline void set_argv(int index, const ::std::string& value);
  inline void set_argv(int index, const char* value);
  inline void set_argv(int index, const char* value, size_t size);
  inline ::std::string* add_argv();
  inline void add_argv(const ::std::string& value);
  inline void add_argv(const char* value);
  inline void add_argv(const char* value, size_t size);
  inline const ::google::protobuf::RepeatedPtrField< ::std::string>& argv() const;
  inline ::google::protobuf::RepeatedPtrField< ::std::string>* mutable_argv();

  // optional string threadname = 6;
  inline bool has_threadname() const;
  inline void clear_threadname();
  static const int kThreadnameFieldNumber = 6;
  inline const ::std::string& threadname() const;
  inline void set_threadname(const ::std::string& value);
  inline void set_threadname(const char* value);
  inline void set_threadname(const char* value, size_t size);
  inline ::std::string* mutable_threadname();
  inline ::std::string* release_threadname();
  inline void set_allocated_threadname(::std::string* threadname);

  // optional int32 threadperiod = 7;
  inline bool has_threadperiod() const;
  inline void clear_threadperiod();
  static const int kThreadperiodFieldNumber = 7;
  inline ::google::protobuf::int32 threadperiod() const;
  inline void set_threadperiod(::google::protobuf::int32 value);

  // optional bool use_fp = 8;
  inline bool has_use_fp() const;
  inline void clear_use_fp();
  static const int kUseFpFieldNumber = 8;
  inline bool use_fp() const;
  inline void set_use_fp(bool value);

  // optional int32 cpu = 9;
  inline bool has_cpu() const;
  inline void clear_cpu();
  static const int kCpuFieldNumber = 9;
  inline ::google::protobuf::int32 cpu() const;
  inline void set_cpu(::google::protobuf::int32 value);

  // optional string comp = 10;
  inline bool has_comp() const;
  inline void clear_comp();
  static const int kCompFieldNumber = 10;
  inline const ::std::string& comp() const;
  inline void set_comp(const ::std::string& value);
  inline void set_comp(const char* value);
  inline void set_comp(const char* value, size_t size);
  inline ::std::string* mutable_comp();
  inline ::std::string* release_comp();
  inline void set_allocated_comp(::std::string* comp);

  // optional string func = 11;
  inline bool has_func() const;
  inline void clear_func();
  static const int kFuncFieldNumber = 11;
  inline const ::std::string& func() const;
  inline void set_func(const ::std::string& value);
  inline void set_func(const char* value);
  inline void set_func(const char* value, size_t size);
  inline ::std::string* mutable_func();
  inline ::std::string* release_func();
  inline void set_allocated_func(::std::string* func);

  // optional string instname = 12;
  inline bool has_instname() const;
  inline void clear_instname();
  static const int kInstnameFieldNumber = 12;
  inline const ::std::string& instname() const;
  inline void set_instname(const ::std::string& value);
  inline void set_instname(const char* value);
  inline void set_instname(const char* value, size_t size);
  inline ::std::string* mutable_instname();
  inline ::std::string* release_instname();
  inline void set_allocated_instname(::std::string* instname);

  // optional int32 flags = 13;
  inline bool has_flags() const;
  inline void clear_flags();
  static const int kFlagsFieldNumber = 13;
  inline ::google::protobuf::int32 flags() const;
  inline void set_flags(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:pb.RTAPICommand)
 private:
  inline void set_has_instance();
  inline void clear_has_instance();
  inline void set_has_modname();
  inline void clear_has_modname();
  inline void set_has_rt_msglevel();
  inline void clear_has_rt_msglevel();
  inline void set_has_user_msglevel();
  inline void clear_has_user_msglevel();
  inline void set_has_threadname();
  inline void clear_has_threadname();
  inline void set_has_threadperiod();
  inline void clear_has_threadperiod();
  inline void set_has_use_fp();
  inline void clear_has_use_fp();
  inline void set_has_cpu();
  inline void clear_has_cpu();
  inline void set_has_comp();
  inline void clear_has_comp();
  inline void set_has_func();
  inline void clear_has_func();
  inline void set_has_instname();
  inline void clear_has_instname();
  inline void set_has_flags();
  inline void clear_has_flags();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::std::string* modname_;
  ::google::protobuf::int32 instance_;
  ::google::protobuf::int32 rt_msglevel_;
  ::google::protobuf::RepeatedPtrField< ::std::string> argv_;
  ::google::protobuf::int32 user_msglevel_;
  ::google::protobuf::int32 threadperiod_;
  ::std::string* threadname_;
  bool use_fp_;
  ::google::protobuf::int32 cpu_;
  ::std::string* comp_;
  ::std::string* func_;
  ::std::string* instname_;
  ::google::protobuf::int32 flags_;
  friend void  protobuf_AddDesc_rtapicommand_2eproto();
  friend void protobuf_AssignDesc_rtapicommand_2eproto();
  friend void protobuf_ShutdownFile_rtapicommand_2eproto();

  void InitAsDefaultInstance();
  static RTAPICommand* default_instance_;
};
// ===================================================================


// ===================================================================

// RTAPICommand

// required int32 instance = 1;
inline bool RTAPICommand::has_instance() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void RTAPICommand::set_has_instance() {
  _has_bits_[0] |= 0x00000001u;
}
inline void RTAPICommand::clear_has_instance() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void RTAPICommand::clear_instance() {
  instance_ = 0;
  clear_has_instance();
}
inline ::google::protobuf::int32 RTAPICommand::instance() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.instance)
  return instance_;
}
inline void RTAPICommand::set_instance(::google::protobuf::int32 value) {
  set_has_instance();
  instance_ = value;
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.instance)
}

// optional string modname = 2;
inline bool RTAPICommand::has_modname() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void RTAPICommand::set_has_modname() {
  _has_bits_[0] |= 0x00000002u;
}
inline void RTAPICommand::clear_has_modname() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void RTAPICommand::clear_modname() {
  if (modname_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    modname_->clear();
  }
  clear_has_modname();
}
inline const ::std::string& RTAPICommand::modname() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.modname)
  return *modname_;
}
inline void RTAPICommand::set_modname(const ::std::string& value) {
  set_has_modname();
  if (modname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    modname_ = new ::std::string;
  }
  modname_->assign(value);
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.modname)
}
inline void RTAPICommand::set_modname(const char* value) {
  set_has_modname();
  if (modname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    modname_ = new ::std::string;
  }
  modname_->assign(value);
  // @@protoc_insertion_point(field_set_char:pb.RTAPICommand.modname)
}
inline void RTAPICommand::set_modname(const char* value, size_t size) {
  set_has_modname();
  if (modname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    modname_ = new ::std::string;
  }
  modname_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:pb.RTAPICommand.modname)
}
inline ::std::string* RTAPICommand::mutable_modname() {
  set_has_modname();
  if (modname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    modname_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:pb.RTAPICommand.modname)
  return modname_;
}
inline ::std::string* RTAPICommand::release_modname() {
  clear_has_modname();
  if (modname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = modname_;
    modname_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void RTAPICommand::set_allocated_modname(::std::string* modname) {
  if (modname_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete modname_;
  }
  if (modname) {
    set_has_modname();
    modname_ = modname;
  } else {
    clear_has_modname();
    modname_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:pb.RTAPICommand.modname)
}

// optional int32 rt_msglevel = 3;
inline bool RTAPICommand::has_rt_msglevel() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void RTAPICommand::set_has_rt_msglevel() {
  _has_bits_[0] |= 0x00000004u;
}
inline void RTAPICommand::clear_has_rt_msglevel() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void RTAPICommand::clear_rt_msglevel() {
  rt_msglevel_ = 0;
  clear_has_rt_msglevel();
}
inline ::google::protobuf::int32 RTAPICommand::rt_msglevel() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.rt_msglevel)
  return rt_msglevel_;
}
inline void RTAPICommand::set_rt_msglevel(::google::protobuf::int32 value) {
  set_has_rt_msglevel();
  rt_msglevel_ = value;
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.rt_msglevel)
}

// optional int32 user_msglevel = 4;
inline bool RTAPICommand::has_user_msglevel() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void RTAPICommand::set_has_user_msglevel() {
  _has_bits_[0] |= 0x00000008u;
}
inline void RTAPICommand::clear_has_user_msglevel() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void RTAPICommand::clear_user_msglevel() {
  user_msglevel_ = 0;
  clear_has_user_msglevel();
}
inline ::google::protobuf::int32 RTAPICommand::user_msglevel() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.user_msglevel)
  return user_msglevel_;
}
inline void RTAPICommand::set_user_msglevel(::google::protobuf::int32 value) {
  set_has_user_msglevel();
  user_msglevel_ = value;
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.user_msglevel)
}

// repeated string argv = 5;
inline int RTAPICommand::argv_size() const {
  return argv_.size();
}
inline void RTAPICommand::clear_argv() {
  argv_.Clear();
}
inline const ::std::string& RTAPICommand::argv(int index) const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.argv)
  return argv_.Get(index);
}
inline ::std::string* RTAPICommand::mutable_argv(int index) {
  // @@protoc_insertion_point(field_mutable:pb.RTAPICommand.argv)
  return argv_.Mutable(index);
}
inline void RTAPICommand::set_argv(int index, const ::std::string& value) {
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.argv)
  argv_.Mutable(index)->assign(value);
}
inline void RTAPICommand::set_argv(int index, const char* value) {
  argv_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:pb.RTAPICommand.argv)
}
inline void RTAPICommand::set_argv(int index, const char* value, size_t size) {
  argv_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:pb.RTAPICommand.argv)
}
inline ::std::string* RTAPICommand::add_argv() {
  return argv_.Add();
}
inline void RTAPICommand::add_argv(const ::std::string& value) {
  argv_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:pb.RTAPICommand.argv)
}
inline void RTAPICommand::add_argv(const char* value) {
  argv_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:pb.RTAPICommand.argv)
}
inline void RTAPICommand::add_argv(const char* value, size_t size) {
  argv_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:pb.RTAPICommand.argv)
}
inline const ::google::protobuf::RepeatedPtrField< ::std::string>&
RTAPICommand::argv() const {
  // @@protoc_insertion_point(field_list:pb.RTAPICommand.argv)
  return argv_;
}
inline ::google::protobuf::RepeatedPtrField< ::std::string>*
RTAPICommand::mutable_argv() {
  // @@protoc_insertion_point(field_mutable_list:pb.RTAPICommand.argv)
  return &argv_;
}

// optional string threadname = 6;
inline bool RTAPICommand::has_threadname() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void RTAPICommand::set_has_threadname() {
  _has_bits_[0] |= 0x00000020u;
}
inline void RTAPICommand::clear_has_threadname() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void RTAPICommand::clear_threadname() {
  if (threadname_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    threadname_->clear();
  }
  clear_has_threadname();
}
inline const ::std::string& RTAPICommand::threadname() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.threadname)
  return *threadname_;
}
inline void RTAPICommand::set_threadname(const ::std::string& value) {
  set_has_threadname();
  if (threadname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    threadname_ = new ::std::string;
  }
  threadname_->assign(value);
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.threadname)
}
inline void RTAPICommand::set_threadname(const char* value) {
  set_has_threadname();
  if (threadname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    threadname_ = new ::std::string;
  }
  threadname_->assign(value);
  // @@protoc_insertion_point(field_set_char:pb.RTAPICommand.threadname)
}
inline void RTAPICommand::set_threadname(const char* value, size_t size) {
  set_has_threadname();
  if (threadname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    threadname_ = new ::std::string;
  }
  threadname_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:pb.RTAPICommand.threadname)
}
inline ::std::string* RTAPICommand::mutable_threadname() {
  set_has_threadname();
  if (threadname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    threadname_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:pb.RTAPICommand.threadname)
  return threadname_;
}
inline ::std::string* RTAPICommand::release_threadname() {
  clear_has_threadname();
  if (threadname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = threadname_;
    threadname_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void RTAPICommand::set_allocated_threadname(::std::string* threadname) {
  if (threadname_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete threadname_;
  }
  if (threadname) {
    set_has_threadname();
    threadname_ = threadname;
  } else {
    clear_has_threadname();
    threadname_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:pb.RTAPICommand.threadname)
}

// optional int32 threadperiod = 7;
inline bool RTAPICommand::has_threadperiod() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void RTAPICommand::set_has_threadperiod() {
  _has_bits_[0] |= 0x00000040u;
}
inline void RTAPICommand::clear_has_threadperiod() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void RTAPICommand::clear_threadperiod() {
  threadperiod_ = 0;
  clear_has_threadperiod();
}
inline ::google::protobuf::int32 RTAPICommand::threadperiod() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.threadperiod)
  return threadperiod_;
}
inline void RTAPICommand::set_threadperiod(::google::protobuf::int32 value) {
  set_has_threadperiod();
  threadperiod_ = value;
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.threadperiod)
}

// optional bool use_fp = 8;
inline bool RTAPICommand::has_use_fp() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void RTAPICommand::set_has_use_fp() {
  _has_bits_[0] |= 0x00000080u;
}
inline void RTAPICommand::clear_has_use_fp() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void RTAPICommand::clear_use_fp() {
  use_fp_ = false;
  clear_has_use_fp();
}
inline bool RTAPICommand::use_fp() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.use_fp)
  return use_fp_;
}
inline void RTAPICommand::set_use_fp(bool value) {
  set_has_use_fp();
  use_fp_ = value;
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.use_fp)
}

// optional int32 cpu = 9;
inline bool RTAPICommand::has_cpu() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void RTAPICommand::set_has_cpu() {
  _has_bits_[0] |= 0x00000100u;
}
inline void RTAPICommand::clear_has_cpu() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void RTAPICommand::clear_cpu() {
  cpu_ = 0;
  clear_has_cpu();
}
inline ::google::protobuf::int32 RTAPICommand::cpu() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.cpu)
  return cpu_;
}
inline void RTAPICommand::set_cpu(::google::protobuf::int32 value) {
  set_has_cpu();
  cpu_ = value;
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.cpu)
}

// optional string comp = 10;
inline bool RTAPICommand::has_comp() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void RTAPICommand::set_has_comp() {
  _has_bits_[0] |= 0x00000200u;
}
inline void RTAPICommand::clear_has_comp() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void RTAPICommand::clear_comp() {
  if (comp_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    comp_->clear();
  }
  clear_has_comp();
}
inline const ::std::string& RTAPICommand::comp() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.comp)
  return *comp_;
}
inline void RTAPICommand::set_comp(const ::std::string& value) {
  set_has_comp();
  if (comp_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    comp_ = new ::std::string;
  }
  comp_->assign(value);
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.comp)
}
inline void RTAPICommand::set_comp(const char* value) {
  set_has_comp();
  if (comp_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    comp_ = new ::std::string;
  }
  comp_->assign(value);
  // @@protoc_insertion_point(field_set_char:pb.RTAPICommand.comp)
}
inline void RTAPICommand::set_comp(const char* value, size_t size) {
  set_has_comp();
  if (comp_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    comp_ = new ::std::string;
  }
  comp_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:pb.RTAPICommand.comp)
}
inline ::std::string* RTAPICommand::mutable_comp() {
  set_has_comp();
  if (comp_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    comp_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:pb.RTAPICommand.comp)
  return comp_;
}
inline ::std::string* RTAPICommand::release_comp() {
  clear_has_comp();
  if (comp_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = comp_;
    comp_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void RTAPICommand::set_allocated_comp(::std::string* comp) {
  if (comp_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete comp_;
  }
  if (comp) {
    set_has_comp();
    comp_ = comp;
  } else {
    clear_has_comp();
    comp_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:pb.RTAPICommand.comp)
}

// optional string func = 11;
inline bool RTAPICommand::has_func() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void RTAPICommand::set_has_func() {
  _has_bits_[0] |= 0x00000400u;
}
inline void RTAPICommand::clear_has_func() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void RTAPICommand::clear_func() {
  if (func_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    func_->clear();
  }
  clear_has_func();
}
inline const ::std::string& RTAPICommand::func() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.func)
  return *func_;
}
inline void RTAPICommand::set_func(const ::std::string& value) {
  set_has_func();
  if (func_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    func_ = new ::std::string;
  }
  func_->assign(value);
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.func)
}
inline void RTAPICommand::set_func(const char* value) {
  set_has_func();
  if (func_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    func_ = new ::std::string;
  }
  func_->assign(value);
  // @@protoc_insertion_point(field_set_char:pb.RTAPICommand.func)
}
inline void RTAPICommand::set_func(const char* value, size_t size) {
  set_has_func();
  if (func_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    func_ = new ::std::string;
  }
  func_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:pb.RTAPICommand.func)
}
inline ::std::string* RTAPICommand::mutable_func() {
  set_has_func();
  if (func_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    func_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:pb.RTAPICommand.func)
  return func_;
}
inline ::std::string* RTAPICommand::release_func() {
  clear_has_func();
  if (func_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = func_;
    func_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void RTAPICommand::set_allocated_func(::std::string* func) {
  if (func_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete func_;
  }
  if (func) {
    set_has_func();
    func_ = func;
  } else {
    clear_has_func();
    func_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:pb.RTAPICommand.func)
}

// optional string instname = 12;
inline bool RTAPICommand::has_instname() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void RTAPICommand::set_has_instname() {
  _has_bits_[0] |= 0x00000800u;
}
inline void RTAPICommand::clear_has_instname() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void RTAPICommand::clear_instname() {
  if (instname_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    instname_->clear();
  }
  clear_has_instname();
}
inline const ::std::string& RTAPICommand::instname() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.instname)
  return *instname_;
}
inline void RTAPICommand::set_instname(const ::std::string& value) {
  set_has_instname();
  if (instname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    instname_ = new ::std::string;
  }
  instname_->assign(value);
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.instname)
}
inline void RTAPICommand::set_instname(const char* value) {
  set_has_instname();
  if (instname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    instname_ = new ::std::string;
  }
  instname_->assign(value);
  // @@protoc_insertion_point(field_set_char:pb.RTAPICommand.instname)
}
inline void RTAPICommand::set_instname(const char* value, size_t size) {
  set_has_instname();
  if (instname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    instname_ = new ::std::string;
  }
  instname_->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:pb.RTAPICommand.instname)
}
inline ::std::string* RTAPICommand::mutable_instname() {
  set_has_instname();
  if (instname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    instname_ = new ::std::string;
  }
  // @@protoc_insertion_point(field_mutable:pb.RTAPICommand.instname)
  return instname_;
}
inline ::std::string* RTAPICommand::release_instname() {
  clear_has_instname();
  if (instname_ == &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    return NULL;
  } else {
    ::std::string* temp = instname_;
    instname_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
    return temp;
  }
}
inline void RTAPICommand::set_allocated_instname(::std::string* instname) {
  if (instname_ != &::google::protobuf::internal::GetEmptyStringAlreadyInited()) {
    delete instname_;
  }
  if (instname) {
    set_has_instname();
    instname_ = instname;
  } else {
    clear_has_instname();
    instname_ = const_cast< ::std::string*>(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  }
  // @@protoc_insertion_point(field_set_allocated:pb.RTAPICommand.instname)
}

// optional int32 flags = 13;
inline bool RTAPICommand::has_flags() const {
  return (_has_bits_[0] & 0x00001000u) != 0;
}
inline void RTAPICommand::set_has_flags() {
  _has_bits_[0] |= 0x00001000u;
}
inline void RTAPICommand::clear_has_flags() {
  _has_bits_[0] &= ~0x00001000u;
}
inline void RTAPICommand::clear_flags() {
  flags_ = 0;
  clear_has_flags();
}
inline ::google::protobuf::int32 RTAPICommand::flags() const {
  // @@protoc_insertion_point(field_get:pb.RTAPICommand.flags)
  return flags_;
}
inline void RTAPICommand::set_flags(::google::protobuf::int32 value) {
  set_has_flags();
  flags_ = value;
  // @@protoc_insertion_point(field_set:pb.RTAPICommand.flags)
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace pb

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_rtapicommand_2eproto__INCLUDED
