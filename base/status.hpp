#pragma once

#include <array>
#include <ostream>
#include <source_location>

#include "base/core.hpp"

namespace simon {

class StatusCode;
class StatusBase;
class StatusKind;
class StatusKindDomainInterface;
class StatusDomainInterface;
class StatusDetached;
class Status;

struct StatusConditionEntry final {
  std::string message;
};

struct StatusIncidentEntry final {
  std::string message;
  std::source_location location;
  int platform_error = 0;
};

//==============================================================================
//
class StatusCode final {
 public:
  constexpr explicit StatusCode(std::size_t domain,     //
                                std::size_t condition,  //
                                std::size_t incident)
      : bits_{(least_16_bits_as_uint64(domain) << DOMAIN_SHIFT) |
              (least_16_bits_as_uint64(condition) << CONDITION_SHIFT) |
              (least_16_bits_as_uint64(incident) << INCIDENT_SHIFT)} {
    CHECK_POSTCONDITION(std::cmp_equal(domain, domain_bits()));
    CHECK_POSTCONDITION(std::cmp_equal(condition, condition_bits()));
    CHECK_POSTCONDITION(std::cmp_equal(incident, incident_bits()));
  }

  DECLARE_COPY_DEFAULT_CONSTEXPR(StatusCode);
  DECLARE_MOVE_DEFAULT_CONSTEXPR(StatusCode);

  constexpr StatusCode() = default;
  constexpr ~StatusCode() = default;

  constexpr std::size_t domain_bits() const noexcept {
    return (bits_ & DOMAIN_MASK) >> DOMAIN_SHIFT;
  }

  constexpr void set_domain_bits(std::size_t bits) noexcept {
    bits_ &= ~DOMAIN_MASK;
    bits_ |= least_16_bits_as_uint64(bits) << DOMAIN_SHIFT;
    CHECK_POSTCONDITION(std::cmp_equal(bits, domain_bits()));
  }

  constexpr void set_domain_bits(StatusCode code) noexcept {
    bits_ &= ~DOMAIN_MASK;
    bits_ |= code.bits_ & DOMAIN_MASK;
  }

  constexpr std::size_t condition_bits() const noexcept {
    return (bits_ & CONDITION_MASK) >> CONDITION_SHIFT;
  }

  constexpr void set_condition_bits(std::size_t bits) noexcept {
    bits_ &= ~CONDITION_MASK;
    bits_ |= least_16_bits_as_uint64(bits) << CONDITION_SHIFT;
    CHECK_POSTCONDITION(std::cmp_equal(bits, condition_bits()));
  }

  constexpr void set_condition_bits(StatusCode code) noexcept {
    bits_ &= ~CONDITION_MASK;
    bits_ |= code.bits_ & CONDITION_MASK;
  }

  constexpr std::size_t incident_bits() const noexcept {
    return (bits_ & INCIDENT_MASK) >> INCIDENT_SHIFT;
  }

  constexpr void set_incident_bits(std::size_t bits) noexcept {
    bits_ &= ~INCIDENT_MASK;
    bits_ |= least_16_bits_as_uint64(bits) << INCIDENT_SHIFT;
    CHECK_POSTCONDITION(std::cmp_equal(bits, incident_bits()));
  }

  constexpr void set_incident_bits(StatusCode code) noexcept {
    bits_ &= ~INCIDENT_MASK;
    bits_ |= code.bits_ & INCIDENT_MASK;
  }

  constexpr bool is_same_kind(StatusCode that) const noexcept {
    return (bits_ & KIND_COMPARE_MASK) == (that.bits_ & KIND_COMPARE_MASK);
  }

  constexpr bool is_same_code(StatusCode that) const noexcept {
    return (bits_ & CODE_COMPARE_MASK) == (that.bits_ & CODE_COMPARE_MASK);
  }

 private:
  std::uint64_t bits_ = 0;

  constexpr static std::uint64_t DOMAIN_MASK = 0x0000'FFFF'0000'0000;
  constexpr static std::uint64_t DOMAIN_SIZE = 16;
  constexpr static std::uint64_t DOMAIN_SHIFT = 32;

  constexpr static std::uint64_t CONDITION_MASK = 0x0000'0000'FFFF'0000;
  constexpr static std::uint64_t CONDITION_SIZE = 16;
  constexpr static std::uint64_t CONDITION_SHIFT = 16;

  constexpr static std::uint64_t INCIDENT_MASK = 0x0000'0000'0000'FFFF;
  constexpr static std::uint64_t INCIDENT_SIZE = 16;
  constexpr static std::uint64_t INCIDENT_SHIFT = 0;

  constexpr static std::uint64_t KIND_COMPARE_MASK =
      DOMAIN_MASK | CONDITION_MASK;
  constexpr static std::uint64_t CODE_COMPARE_MASK =
      DOMAIN_MASK | CONDITION_MASK | INCIDENT_MASK;

  constexpr static std::uint64_t least_16_bits_as_uint64(std::size_t bits) {
    constexpr std::uint64_t SHIFT = 64 - 16;
    return ((reinterpret_cast<std::uint64_t>(bits) << SHIFT) >> SHIFT);
  }

  friend constexpr bool operator<(StatusCode lhs, StatusCode rhs) noexcept {
    return lhs.bits_ < rhs.bits_;
  }

  friend std::ostream &operator<<(std::ostream &out, StatusCode self) noexcept {
    out << std::hex << self.bits_;
    return out;
  }
};

//==============================================================================
//
class StatusKindInspectInterface {
 public:
  constexpr virtual std::string_view status_kind_message_of(
      StatusKind const *) const noexcept = 0;
};

class StatusKindBuilderInterface {
 public:
  constexpr virtual StatusCode make_status_kind_code(
      std::size_t condition_code) const noexcept = 0;
};

class StatusBaseInspectInterface {
 public:
  virtual std::string_view status_base_message_of(
      StatusBase const *) const noexcept = 0;

  virtual std::source_location status_base_location_of(
      StatusBase const *) const noexcept = 0;

  virtual int status_base_platform_error_of(
      StatusBase const *) const noexcept = 0;
};

class StatusBaseBuilderInterface {
 public:
  virtual StatusCode make_status_base_code(
      std::size_t condition_code, std::size_t incident_code) const noexcept = 0;
};

class StatusConditionEquivalenceInterface {
 public:
  virtual bool has_equivalent_condition_of(StatusBase const *,
                                           StatusBase const *) const noexcept {
    return false;
  }
  virtual bool has_equivalent_condition_of(StatusBase const *,
                                           StatusKind const *) const noexcept {
    return false;
  }
};

class StatusDomainInterface : public StatusBaseInspectInterface,
                              public StatusKindInspectInterface,
                              public StatusConditionEquivalenceInterface {};

class StatusKindBuilderBase : public StatusKindBuilderInterface {
 public:
  DECLARE_COPY_DEFAULT_CONSTEXPR(StatusKindBuilderBase);
  DECLARE_MOVE_DEFAULT_CONSTEXPR(StatusKindBuilderBase);

  constexpr StatusKindBuilderBase() = default;
  constexpr ~StatusKindBuilderBase() = default;

  constexpr explicit StatusKindBuilderBase(std::size_t domain_code,
                                           std::string_view domain_name)
      : domain_code_{domain_code, 0U, 0U}, name_{domain_name} {}

  constexpr std::string_view name() const noexcept { return name_; }

  constexpr StatusCode make_status_kind_code(
      std::size_t condition_code) const noexcept final {
    return StatusCode{domain_code_.domain_bits(), condition_code, 0U};
  }

 private:
  StatusCode domain_code_;
  std::string_view name_;
};

class StatusDomainBuilderBase : public StatusKindBuilderInterface,
                                public StatusBaseBuilderInterface {
 public:
  DECLARE_COPY_DEFAULT(StatusDomainBuilderBase);
  DECLARE_MOVE_DEFAULT(StatusDomainBuilderBase);

  StatusDomainBuilderBase() = default;
  ~StatusDomainBuilderBase() = default;

  explicit StatusDomainBuilderBase(std::size_t domain_code,
                                   std::string_view domain_name)
      : domain_code_{domain_code, 0U, 0U}, name_{domain_name} {}

  std::string_view name() const noexcept { return name_; }

  StatusCode make_status_kind_code(
      std::size_t condition_code) const noexcept final {
    return StatusCode{domain_code_.domain_bits(),  //
                      condition_code,              //
                      0U};
  }

  StatusCode make_status_base_code(
      std::size_t condition_code,
      std::size_t incident_code) const noexcept final {
    return StatusCode{domain_code_.domain_bits(),  //
                      condition_code,              //
                      incident_code};
  }

 private:
  StatusCode domain_code_;
  std::string name_;
};

namespace impl {
Status make_status(  //
    StatusCode, StatusDomainInterface const *) noexcept;
StatusDetached make_status_detached(  //
    StatusCode, StatusDomainInterface const *) noexcept;
constexpr StatusKind make_status_kind(  //
    StatusCode, StatusKindInspectInterface const *) noexcept;
}  // namespace impl

constexpr std::size_t domain_code_of(StatusKind const *) noexcept;
constexpr std::size_t domain_code_of(StatusBase const *) noexcept;
constexpr std::size_t condition_code_of(StatusKind const *) noexcept;
constexpr std::size_t condition_code_of(StatusBase const *) noexcept;
constexpr std::size_t incident_code_of(StatusBase const *) noexcept;

//==============================================================================
//
class StatusKind final {
 public:
  DECLARE_COPY_DEFAULT_CONSTEXPR(StatusKind);
  DECLARE_MOVE_DEFAULT_CONSTEXPR(StatusKind);

  constexpr StatusKind() = default;
  constexpr ~StatusKind() = default;

  constexpr std::string_view message() const noexcept {
    return domain_->status_kind_message_of(this);
  }

  constexpr bool operator==(StatusKind that) const noexcept {
    return kind_code_.is_same_kind(that.kind_code_);
  }

 private:
  constexpr explicit StatusKind(StatusCode code,
                                StatusKindInspectInterface const *domain)
      : kind_code_{code}, domain_{domain} {}

  StatusCode kind_code_;
  StatusKindInspectInterface const *domain_ = nullptr;

  friend class StatusBase;

  friend constexpr StatusKind impl::make_status_kind(
      StatusCode, StatusKindInspectInterface const *) noexcept;

  friend constexpr std::size_t domain_code_of(StatusKind const *) noexcept;
  friend constexpr std::size_t condition_code_of(StatusKind const *) noexcept;

  friend constexpr bool operator!=(StatusKind a, StatusKind b) noexcept {
    return !a.operator==(b);
  }

  friend constexpr bool operator<(StatusKind lhs, StatusKind rhs) noexcept {
    return lhs.kind_code_ < rhs.kind_code_;
  }

  friend std::ostream &operator<<(std::ostream &out, StatusKind self) noexcept {
    out << self.message();
    return out;
  }
};

//==============================================================================
//
class StatusBase {
 public:
  DECLARE_COPY_DEFAULT(StatusBase);
  DECLARE_MOVE_DEFAULT(StatusBase);

  StatusBase() = delete;
  ~StatusBase() = default;

  bool operator==(StatusBase that) const noexcept {
    return status_code_.is_same_code(that.status_code_);
  }

  bool operator==(StatusKind that) const noexcept {
    return status_code_.is_same_kind(that.kind_code_);
  }

 protected:
  explicit StatusBase(StatusCode code) : status_code_{code} {}

  StatusCode status_code_;

  friend constexpr std::size_t domain_code_of(StatusBase const *) noexcept;
  friend constexpr std::size_t condition_code_of(StatusBase const *) noexcept;
  friend constexpr std::size_t incident_code_of(StatusBase const *) noexcept;

  friend bool operator==(StatusKind lhs, StatusBase rhs) noexcept {
    return rhs.operator==(lhs);
  }

  friend bool operator!=(StatusBase lhs, StatusKind rhs) noexcept {
    return !lhs.operator==(rhs);
  }

  friend bool operator!=(StatusKind lhs, StatusBase rhs) noexcept {
    return !rhs.operator==(lhs);
  }

  friend bool operator<(StatusBase lhs, StatusBase rhs) noexcept {
    return lhs.status_code_ < rhs.status_code_;
  }
};

//==============================================================================
//
class StatusDetached final : public StatusBase {
 public:
  DECLARE_COPY_DEFAULT(StatusDetached);
  DECLARE_MOVE_DEFAULT(StatusDetached);

  StatusDetached() = delete;
  ~StatusDetached() = default;

  std::string_view message() const noexcept { return entry_.message; }
  std::source_location location() const noexcept { return entry_.location; }
  int platform_error() const noexcept { return entry_.platform_error; }

 private:
  explicit StatusDetached(StatusCode code, StatusDomainInterface const *domain)
      : StatusBase{code} {
    entry_.message = domain->status_base_message_of(this);
    entry_.location = domain->status_base_location_of(this);
    entry_.platform_error = domain->status_base_platform_error_of(this);
  }

  StatusIncidentEntry entry_;

  friend StatusDetached impl::make_status_detached(  //
      StatusCode, StatusDomainInterface const *) noexcept;

  friend std::ostream &operator<<(std::ostream &out,
                                  StatusDetached self) noexcept {
    out << self.message();
    return out;
  }
};

//==============================================================================
//
class Status final : public StatusBase {
 public:
  DECLARE_COPY_DEFAULT(Status);
  DECLARE_MOVE_DEFAULT(Status);

  Status() = delete;
  ~Status() = default;

  std::string_view message() const noexcept {
    return domain_->status_base_message_of(this);
  }
  std::source_location location() const noexcept {
    return domain_->status_base_location_of(this);
  }

  int platform_error() const noexcept {
    return domain_->status_base_platform_error_of(this);
  }

  StatusKind kind() const noexcept {
    return impl::make_status_kind(status_code_, domain_);
  }

  StatusDetached detach_copy() const noexcept {
    return impl::make_status_detached(status_code_, domain_);
  }

  bool has_equivalent_condition_as(Status that) const noexcept {
    return *this == that || domain_->has_equivalent_condition_of(this, &that);
  }

  bool has_equivalent_condition_as(StatusKind that) const noexcept {
    return *this == that || domain_->has_equivalent_condition_of(this, &that);
  }

 private:
  explicit Status(StatusCode code, StatusDomainInterface const *domain)
      : StatusBase{code}, domain_{domain} {}

  StatusDomainInterface const *domain_ = nullptr;

  friend Status impl::make_status(  //
      StatusCode, const StatusDomainInterface *) noexcept;

  friend std::ostream &operator<<(std::ostream &out, Status self) noexcept {
    out << self.message();
    return out;
  }
};

//------------------------------------------------------------------------------
//
namespace impl {
inline Status make_status(  //
    StatusCode code, const StatusDomainInterface *domain) noexcept {
  return Status{code, domain};
}

inline StatusDetached make_status_detached(  //
    StatusCode code, const StatusDomainInterface *domain) noexcept {
  return StatusDetached{code, domain};
}

inline constexpr StatusKind make_status_kind(  //
    StatusCode code, StatusKindInspectInterface const *domain) noexcept {
  return StatusKind{code, domain};
}
}  // namespace impl

inline constexpr std::size_t domain_code_of(StatusKind const *self) noexcept {
  return self->kind_code_.domain_bits();
}

inline constexpr std::size_t domain_code_of(StatusBase const *self) noexcept {
  return self->status_code_.domain_bits();
}

inline constexpr std::size_t condition_code_of(
    StatusKind const *self) noexcept {
  return self->kind_code_.condition_bits();
}

inline constexpr std::size_t condition_code_of(
    StatusBase const *self) noexcept {
  return self->status_code_.condition_bits();
}

inline constexpr std::size_t incident_code_of(StatusBase const *self) noexcept {
  return self->status_code_.incident_bits();
}

//==============================================================================
//
template <typename ConditionEnumType, std::size_t ConditionCount>
class EnumStatusKindConditionMixin {
 public:
  constexpr StatusKind watch_kind(ConditionEnumType condition) const noexcept {
    return handle_watch_kind(condition);
  }

 protected:
  virtual StatusKind handle_watch_kind(
      ConditionEnumType condition) const noexcept = 0;

  StatusKind do_watch_kind(                       //
      ConditionEnumType condition,                //
      StatusKindBuilderInterface const *builder,  //
      StatusKindInspectInterface const *domain) const noexcept {
    auto condition_code = static_cast<std::size_t>(condition);
    return impl::make_status_kind(
        builder->make_status_kind_code(condition_code), domain);
  }

  constexpr std::string_view status_kind_message_of(
      StatusKind const *kind) const noexcept {
    return conditions_[condition_code_of(kind)].message;
  }

  static const std::array<StatusConditionEntry, ConditionCount> conditions_;
};

//==============================================================================
// NOTE: To avoid the cost of reference counting each return code's incident
// location and message, the domain is limited to tracking `IncidentCountMax`
// number of concurrent incidents, which are stored in a simple array-backed
// ring buffer. If `Status` incident information must be accurately stored for
// longer than the buffer might be expected to overlap, then `StatusDetached`
// should be used instead, which keeps its own local copy.
//
template <typename ConditionEnumType,  //
          std::size_t IncidentCountMax>
class EnumStatusIncidentMixin {
 public:
  Status raise_status(              //
      ConditionEnumType condition,  //
      std::string message = {}) noexcept {
    std::source_location location{};  // Empty.
    return handle_raise_incident(condition, StatusIncidentEntry{
                                                .message = std::move(message),
                                                .location = std::move(location),
                                                .platform_error = 0,
                                            });
  }

  Status raise_status_here(         //
      ConditionEnumType condition,  //
      std::string message = {},     //
      std::source_location location =
          std::source_location::current()) noexcept {
    return handle_raise_incident(condition, StatusIncidentEntry{
                                                .message = std::move(message),
                                                .location = std::move(location),
                                                .platform_error = 0,
                                            });
  }

  Status raise_error(               //
      ConditionEnumType condition,  //
      int platform_error,           //
      std::string message = {}) noexcept {
    std::source_location location{};  // Empty.
    return handle_raise_incident(condition,
                                 StatusIncidentEntry{
                                     .message = std::move(message),
                                     .location = std::move(location),
                                     .platform_error = platform_error,
                                 });
  }

  Status raise_error_here(          //
      ConditionEnumType condition,  //
      int platform_error,           //
      std::string message = {},
      std::source_location location =
          std::source_location::current()) noexcept {
    return handle_raise_incident(condition,
                                 StatusIncidentEntry{
                                     .message = std::move(message),
                                     .location = std::move(location),
                                     .platform_error = platform_error,
                                 });
  }

 protected:
  virtual Status handle_raise_incident(ConditionEnumType condition,
                                       StatusIncidentEntry entry) noexcept = 0;

  Status do_raise_incident(                       //
      ConditionEnumType condition,                //
      StatusIncidentEntry entry,                  //
      StatusBaseBuilderInterface const *builder,  //
      StatusDomainInterface const *domain) noexcept {
    std::size_t current = next_incident_++;
    next_incident_ %= incidents_.size();
    incidents_[current] = std::move(entry);

    auto condition_code = static_cast<std::size_t>(condition);
    return impl::make_status(
        builder->make_status_base_code(condition_code, current), domain);
  }

  std::string_view status_base_message_of(
      StatusBase const *status) const noexcept {
    return incidents_[incident_code_of(status)].message;
  }

  std::source_location status_base_location_of(
      StatusBase const *status) const noexcept {
    return incidents_[incident_code_of(status)].location;
  }

  int status_base_platform_error_of(StatusBase const *status) const noexcept {
    return incidents_[incident_code_of(status)].platform_error;
  }

 private:
  std::size_t next_incident_ = 0;
  std::array<StatusIncidentEntry, IncidentCountMax> incidents_;
};

//==============================================================================
// Kind-only enum-based domain type.
//
template <typename ConditionEnumType,  //
          std::size_t ConditionCount>
class EnumStatusKindDomain
    : public EnumStatusKindConditionMixin<ConditionEnumType, ConditionCount>,
      public StatusKindInspectInterface,
      public StatusKindBuilderBase {
 public:
  using StatusKindBuilderBase::StatusKindBuilderBase;

  constexpr std::string_view status_kind_message_of(
      StatusKind const *kind) const noexcept override {
    return EnumStatusKindConditionMixin<
        ConditionEnumType, ConditionCount>::status_kind_message_of(kind);
  }

 private:
  StatusKind handle_watch_kind(
      ConditionEnumType condition) const noexcept final {
    return EnumStatusKindConditionMixin<ConditionEnumType,
                                        ConditionCount>::do_watch_kind(  //
        condition, this, this);
  }
};

constexpr std::size_t DEFAULT_INCIDENT_COUNT = 16;

//==============================================================================
// Main enum-based domain type.
//
template <typename ConditionEnumType,  //
          std::size_t ConditionCount,  //
          std::size_t IncidentCountMax = DEFAULT_INCIDENT_COUNT>
class EnumStatusDomain
    : public StatusDomainBuilderBase,
      public StatusDomainInterface,
      public EnumStatusKindConditionMixin<ConditionEnumType, ConditionCount>,
      public EnumStatusIncidentMixin<ConditionEnumType, IncidentCountMax> {
 public:
  using StatusDomainBuilderBase::StatusDomainBuilderBase;

  constexpr std::string_view status_kind_message_of(
      StatusKind const *kind) const noexcept override {
    return EnumStatusKindConditionMixin<
        ConditionEnumType, ConditionCount>::status_kind_message_of(kind);
  }

  std::string_view status_base_message_of(
      StatusBase const *status) const noexcept override {
    return EnumStatusIncidentMixin<
        ConditionEnumType, IncidentCountMax>::status_base_message_of(status);
  }

  std::source_location status_base_location_of(
      StatusBase const *status) const noexcept override {
    return EnumStatusIncidentMixin<
        ConditionEnumType, IncidentCountMax>::status_base_location_of(status);
  }

  int status_base_platform_error_of(
      StatusBase const *status) const noexcept override {
    return EnumStatusIncidentMixin<ConditionEnumType, IncidentCountMax>::
        status_base_platform_error_of(status);
  }

 private:
  StatusKind handle_watch_kind(
      ConditionEnumType condition) const noexcept final {
    return EnumStatusKindConditionMixin<ConditionEnumType,
                                        ConditionCount>::do_watch_kind(  //
        condition, this, this);
  }

  Status handle_raise_incident(ConditionEnumType condition,
                               StatusIncidentEntry entry) noexcept final {
    return EnumStatusIncidentMixin<ConditionEnumType, IncidentCountMax>::
        do_raise_incident(condition, std::move(entry), this, this);
  }
};

//==============================================================================
//
template <typename ConditionEnumType,  //
          std::size_t ConditionCount,  //
          std::size_t IncidentCountMax = DEFAULT_INCIDENT_COUNT>
EnumStatusDomain<ConditionEnumType, ConditionCount, IncidentCountMax> &
static_enum_status_domain() noexcept {
  static std::size_t domain_code = allocate_static_increment();
  static EnumStatusDomain<ConditionEnumType, ConditionCount> domain{
      domain_code, std::format("static_enum_status_domain_{}", domain_code)};
  return domain;
}

template <typename ConditionEnumType,  //
          std::size_t ConditionCount>
EnumStatusDomain<ConditionEnumType, ConditionCount, 1u> &
thread_local_enum_status_domain() noexcept {
  thread_local std::size_t domain_code = allocate_static_increment();
  thread_local EnumStatusDomain<ConditionEnumType, ConditionCount, 1u> domain{
      domain_code,
      std::format("thread_local_enum_status_domain_{}", domain_code)};
  return domain;
}

//------------------------------------------------------------------------------
//
template <typename ConditionEnumType>
Status raise(ConditionEnumType condition, std::string message = {}) noexcept {
  return static_enum_status_domain<ConditionEnumType,
                                   static_cast<std::size_t>(
                                       ConditionEnumType::COUNT)>()
      .raise_status(condition, std::move(message));
}

template <typename ConditionEnumType>
Status raise_here(
    ConditionEnumType condition, std::string message = {},
    std::source_location location = std::source_location::current()) noexcept {
  return static_enum_status_domain<ConditionEnumType,
                                   static_cast<std::size_t>(
                                       ConditionEnumType::COUNT)>()
      .raise_status_here(condition, std::move(message), std::move(location));
}

template <typename ConditionEnumType>
StatusKind watch(ConditionEnumType condition) noexcept {
  return static_enum_status_domain<ConditionEnumType,
                                   static_cast<std::size_t>(
                                       ConditionEnumType::COUNT)>()
      .watch_kind(condition);
}

//------------------------------------------------------------------------------
//
template <typename ConditionEnumType>
Status raise_thread_local(ConditionEnumType condition,
                          std::string message = {}) noexcept {
  return thread_local_enum_status_domain<ConditionEnumType,
                                         static_cast<std::size_t>(
                                             ConditionEnumType::COUNT)>()
      .raise_status(condition, std::move(message));
}

template <typename ConditionEnumType>
Status raise_here_thread_local(
    ConditionEnumType condition, std::string message = {},
    std::source_location location = std::source_location::current()) noexcept {
  return thread_local_enum_status_domain<ConditionEnumType,
                                         static_cast<std::size_t>(
                                             ConditionEnumType::COUNT)>()
      .raise_status_here(condition, std::move(message), std::move(location));
}

template <typename ConditionEnumType>
StatusKind watch_thread_local(ConditionEnumType condition) noexcept {
  return thread_local_enum_status_domain<ConditionEnumType,
                                         static_cast<std::size_t>(
                                             ConditionEnumType::COUNT)>()
      .watch_kind(condition);
}

}  // namespace simon
