#pragma once

#include <array>
#include <map>
#include <ostream>
#include <source_location>

#include "base/core.hpp"

namespace simon {

namespace impl {
class BitCode {
 public:
  BitCode(std::size_t domain,     //
          std::size_t condition,  //
          std::size_t incident)
      : bits_{(least_16_bits_as_uint64(domain) << DOMAIN_SHIFT) |
              (least_16_bits_as_uint64(condition) << CONDITION_SHIFT) |
              (least_16_bits_as_uint64(incident) << INCIDENT_SHIFT)} {
    CHECK_POSTCONDITION(std::cmp_equal(domain, domain_bits()));
    CHECK_POSTCONDITION(std::cmp_equal(condition, condition_bits()));
    CHECK_POSTCONDITION(std::cmp_equal(incident, incident_bits()));
  }

  DECLARE_COPY_DEFAULT(BitCode);
  DECLARE_MOVE_DEFAULT(BitCode);

  BitCode() = default;
  ~BitCode() = default;

  std::size_t domain_bits() const noexcept {
    return (bits_ & DOMAIN_MASK) >> DOMAIN_SHIFT;
  }

  void set_domain_bits(std::size_t bits) noexcept {
    bits_ &= ~DOMAIN_MASK;
    bits_ |= least_16_bits_as_uint64(bits) << DOMAIN_SHIFT;
    CHECK_POSTCONDITION(std::cmp_equal(bits, domain_bits()));
  }

  void set_domain_bits(BitCode code) noexcept {
    bits_ &= ~DOMAIN_MASK;
    bits_ |= code.bits_ & DOMAIN_MASK;
  }

  std::size_t condition_bits() const noexcept {
    return (bits_ & CONDITION_MASK) >> CONDITION_SHIFT;
  }

  void set_condition_bits(std::size_t bits) noexcept {
    bits_ &= ~CONDITION_MASK;
    bits_ |= least_16_bits_as_uint64(bits) << CONDITION_SHIFT;
    CHECK_POSTCONDITION(std::cmp_equal(bits, condition_bits()));
  }

  void set_condition_bits(BitCode code) noexcept {
    bits_ &= ~CONDITION_MASK;
    bits_ |= code.bits_ & CONDITION_MASK;
  }

  std::size_t incident_bits() const noexcept {
    return (bits_ & INCIDENT_MASK) >> INCIDENT_SHIFT;
  }

  void set_incident_bits(std::size_t bits) noexcept {
    bits_ &= ~INCIDENT_MASK;
    bits_ |= least_16_bits_as_uint64(bits) << INCIDENT_SHIFT;
    CHECK_POSTCONDITION(std::cmp_equal(bits, incident_bits()));
  }

  void set_incident_bits(BitCode code) noexcept {
    bits_ &= ~INCIDENT_MASK;
    bits_ |= code.bits_ & INCIDENT_MASK;
  }

  bool has_all_same_domain_bits(BitCode that) const noexcept {
    return (bits_ & DOMAIN_COMPARE_MASK) ==  //
           (that.bits_ & DOMAIN_COMPARE_MASK);
  }

  bool has_all_same_condition_bits(BitCode that) const noexcept {
    return (bits_ & CONDITION_COMPARE_MASK) ==  //
           (that.bits_ & CONDITION_COMPARE_MASK);
  }

  bool has_all_same_incident_bits(BitCode that) const noexcept {
    return (bits_ & INCIDENT_COMPARE_MASK) ==  //
           (that.bits_ & INCIDENT_COMPARE_MASK);
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

  constexpr static std::uint64_t DOMAIN_COMPARE_MASK = DOMAIN_MASK;
  constexpr static std::uint64_t CONDITION_COMPARE_MASK =
      DOMAIN_MASK | CONDITION_MASK;
  constexpr static std::uint64_t INCIDENT_COMPARE_MASK =
      DOMAIN_MASK | CONDITION_MASK | INCIDENT_MASK;

  static std::uint64_t least_16_bits_as_uint64(std::size_t bits) {
    constexpr std::uint64_t SHIFT = 64 - 16;
    return ((reinterpret_cast<std::uint64_t>(bits) << SHIFT) >> SHIFT);
  }
};

struct ConditionEntry final {
  std::string message;
};

struct IncidentEntry final {
  std::source_location location;
  std::string message;
};
}  // namespace impl

class StatusCode;
class StatusCodeLocal;
class StatusCondition;

class StatusDomainBase {
 public:
  DECLARE_COPY_DELETE(StatusDomainBase);
  DECLARE_MOVE_DELETE(StatusDomainBase);

  StatusDomainBase() = delete;
  ~StatusDomainBase() = default;

  explicit StatusDomainBase(std::size_t domain_code,
                            std::string_view domain_name)
      : code_{domain_code}, name_{domain_name} {}

  std::string_view name() const noexcept { return name_; }

  virtual std::string_view message_of(StatusCondition) const noexcept = 0;
  virtual std::string_view message_of(StatusCode) const noexcept = 0;
  virtual std::source_location location_of(StatusCode) const noexcept = 0;
  virtual bool has_equivalent_condition(StatusCode,
                                        StatusCondition) const noexcept = 0;

 protected:
  friend class StatusCode;
  StatusCode make_status_code(impl::BitCode) const noexcept;
  std::size_t incident_code(StatusCode) const noexcept;
  std::size_t condition_code(StatusCode) const noexcept;
  std::size_t domain_code(StatusCode) const noexcept;

  friend class StatusCondition;
  StatusCondition make_status_condition(impl::BitCode) const noexcept;
  std::size_t condition_code(StatusCondition) const noexcept;
  std::size_t domain_code(StatusCondition) const noexcept;

  std::size_t domain_code() const noexcept { return code_; }

 private:
  std::size_t code_;
  std::string name_;
};

class StatusCondition {
 public:
  DECLARE_COPY_DEFAULT(StatusCondition);
  DECLARE_MOVE_DEFAULT(StatusCondition);

  StatusCondition() = delete;
  ~StatusCondition() = default;

  std::string_view message() const noexcept {
    return domain_->message_of(*this);
  }

 private:
  friend class StatusCode;
  friend class StatusDomainBase;

  explicit StatusCondition(impl::BitCode code, StatusDomainBase const *domain)
      : code_{code}, domain_{domain} {}

  impl::BitCode code_;
  StatusDomainBase const *domain_ = nullptr;

  friend std::ostream &operator<<(std::ostream &out, StatusCondition self) {
    out << self.message();
    return out;
  }
};

class StatusCode {
 public:
  DECLARE_COPY_DEFAULT(StatusCode);
  DECLARE_MOVE_DEFAULT(StatusCode);

  StatusCode() = delete;
  ~StatusCode() = default;

  std::string_view message() const noexcept {
    return domain_->message_of(*this);
  }
  std::source_location location() const noexcept {
    return domain_->location_of(*this);
  }

  bool has_condition(StatusCondition condition) const noexcept {
    return operator==(condition) ||
           domain_->has_equivalent_condition(*this, condition);
  }

  bool operator==(StatusCode that) const noexcept {
    return code_.has_all_same_incident_bits(that.code_);
  }

  bool operator==(StatusCondition that) const noexcept {
    return code_.has_all_same_condition_bits(that.code_);
  }

  StatusCodeLocal make_status_code_local() const noexcept;

 private:
  friend class StatusCodeLocal;
  friend class StatusDomainBase;

  explicit StatusCode(impl::BitCode code, StatusDomainBase const *domain)
      : code_{code}, domain_{domain} {}

  impl::BitCode code_;
  StatusDomainBase const *domain_ = nullptr;

  friend bool operator==(StatusCode a, StatusCondition b) noexcept {
    return a.operator==(b);
  }

  friend bool operator==(StatusCondition a, StatusCode b) noexcept {
    return b.operator==(a);
  }

  friend std::ostream &operator<<(std::ostream &out, StatusCode self) {
    out << self.message();
    return out;
  }
};

class StatusCodeLocal : public StatusCode {
 public:
  DECLARE_COPY_DEFAULT(StatusCodeLocal);
  DECLARE_MOVE_DEFAULT(StatusCodeLocal);

  StatusCodeLocal() = delete;
  ~StatusCodeLocal() = default;

  std::string_view message() const noexcept { return entry_.message; }
  std::source_location location() const noexcept { return entry_.location; }

 private:
  friend class StatusCode;

  explicit StatusCodeLocal(impl::BitCode code, StatusDomainBase const *domain)
      : StatusCode{code, domain} {
    entry_.location = domain->location_of(*this);
    entry_.message = domain->message_of(*this);
  }

  impl::IncidentEntry entry_;

  friend std::ostream &operator<<(std::ostream &out, StatusCodeLocal self) {
    out << self.message();
    return out;
  }
};

inline StatusCode StatusDomainBase::make_status_code(
    impl::BitCode code) const noexcept {
  return StatusCode{code, this};
}

inline std::size_t StatusDomainBase::incident_code(
    StatusCode incident) const noexcept {
  return incident.code_.incident_bits();
}

inline std::size_t StatusDomainBase::condition_code(
    StatusCode incident) const noexcept {
  return incident.code_.condition_bits();
}

inline std::size_t StatusDomainBase::domain_code(
    StatusCode incident) const noexcept {
  return incident.code_.domain_bits();
}

inline StatusCondition StatusDomainBase::make_status_condition(
    impl::BitCode code) const noexcept {
  return StatusCondition{code, this};
}

inline std::size_t StatusDomainBase::condition_code(
    StatusCondition condition) const noexcept {
  return condition.code_.condition_bits();
}

inline std::size_t StatusDomainBase::domain_code(
    StatusCondition condition) const noexcept {
  return condition.code_.domain_bits();
}

inline StatusCodeLocal StatusCode::make_status_code_local() const noexcept {
  return StatusCodeLocal{code_, domain_};
}

// NOTE: To avoid the cost of reference counting each return code's incident
// location and message, the domain is limited to tracking `IncidentCountMax`
// number of concurrent incidents, which are stored in a simple array-backed
// ring buffer. If `StatusCode` incident information must be accurately stored
// for longer than the buffer might be expected to overlap, then
// `StatusCodeLocal` should be used instead.
//
template <typename ConditionEnumType,  //
          std::size_t ConditionCount,  //
          std::size_t IncidentCountMax = 16>
class EnumStatusDomain : public StatusDomainBase {
 public:
  DECLARE_COPY_DELETE(EnumStatusDomain);
  DECLARE_MOVE_DELETE(EnumStatusDomain);

  EnumStatusDomain() = delete;
  ~EnumStatusDomain() = default;

  explicit EnumStatusDomain(std::size_t domain_code,
                            std::string_view domain_name)
      : StatusDomainBase{domain_code, domain_name} {}

  std::string_view message_of(
      StatusCondition condition) const noexcept override {
    return conditions_[condition_code(condition)].message;
  }

  std::string_view message_of(StatusCode incident) const noexcept override {
    return incidents_[incident_code(incident)].message;
  }

  std::source_location location_of(
      StatusCode incident) const noexcept override {
    return incidents_[incident_code(incident)].location;
  }

  bool has_equivalent_condition(
      StatusCode incident, StatusCondition condition) const noexcept override;

  StatusCode make_status_code(
      ConditionEnumType condition, std::string message = {},
      std::source_location location = std::source_location::current()) {
    std::size_t current = next_incident_++;
    next_incident_ %= incidents_.size();

    incidents_[current].location = std::move(location);
    incidents_[current].message = std::move(message);

    auto condition_code = static_cast<std::size_t>(condition);
    return make_status_code(
        impl::BitCode{domain_code(), condition_code, current});
  }

  StatusCondition make_status_condition(ConditionEnumType condition) {
    auto condition_code = static_cast<std::size_t>(condition);
    return make_status_condition(
        impl::BitCode{domain_code(), condition_code, 0U});
  }

 private:
  std::size_t next_incident_ = 0;
  std::array<impl::IncidentEntry, IncidentCountMax> incidents_;

 protected:
  static const std::array<impl::ConditionEntry, ConditionCount> conditions_;
};

enum class PosixCondition : std::size_t {
  TOOBIG,        // Argument list too long.
  ACCES,         // Permission denied.
  ADDRINUSE,     // Address in use.
  ADDRNOTAVAIL,  // Address not available.
  AFNOSUPPORT,   // Address family not supported.
  AGAIN,         // Resource unavailable, try again (may be the same value as
                 // [EWOULDBLOCK]).
  ALREADY,       // Connection already in progress.
  BADF,          // Bad file descriptor.
  BADMSG,        // Bad message.
  BUSY,          // Device or resource busy.
  CANCELED,      // Operation canceled.
  CHILD,         // No child processes.
  CONNABORTED,   // Connection aborted.
  CONNREFUSED,   // Connection refused.
  CONNRESET,     // Connection reset.
  DEADLK,        // Resource deadlock would occur.
  DESTADDRREQ,   // Destination address required.
  DOM,           // Mathematics argument out of domain of function.
  DQUOT,         // Reserved.
  EXIST,         // File exists.
  FAULT,         // Bad address.
  FBIG,          // File too large.
  HOSTUNREACH,   // Host is unreachable.
  IDRM,          // Identifier removed.
  ILSEQ,         // Illegal byte sequence.
  INPROGRESS,    // Operation in progress.
  INTR,          // Interrupted function.
  INVAL,         // Invalid argument.
  IO,            // I/O error.
  ISCONN,        // Socket is connected.
  ISDIR,         // Is a directory.
  LOOP,          // Too many levels of symbolic links.
  MFILE,         // File descriptor value too large.
  MLINK,         // Too many links.
  MSGSIZE,       // Message too large.
  MULTIHOP,      // Reserved.
  NAMETOOLONG,   // Filename too long.
  NETDOWN,       // Network is down.
  NETRESET,      // Connection aborted by network.
  NETUNREACH,    // Network unreachable.
  NFILE,         // Too many files open in system.
  NOBUFS,        // No buffer space available.
  NODATA,      // [OB XSR] [Option Start] No message is available on the STREAM
               // head read queue. [Option End]
  NODEV,       // No such device.
  NOENT,       // No such file or directory.
  NOEXEC,      // Executable file format error.
  NOLCK,       // No locks available.
  NOLINK,      // Reserved.
  NOMEM,       // Not enough space.
  NOMSG,       // No message of the desired type.
  NOPROTOOPT,  // Protocol not available.
  NOSPC,       // No space left on device.
  NOSR,        // [OB XSR] [Option Start] No STREAM resources. [Option End]
  NOSTR,       // [OB XSR] [Option Start] Not a STREAM. [Option End]
  NOSYS,       // Functionality not supported.
  NOTCONN,     // The socket is not connected.
  NOTDIR,      // Not a directory or a symbolic link to a directory.
  NOTEMPTY,    // Directory not empty.
  NOTRECOVERABLE,  // State not recoverable.
  NOTSOCK,         // Not a socket.
  NOTSUP,          // Not supported (may be the same value as [EOPNOTSUPP]).
  NOTTY,           // Inappropriate I/O control operation.
  NXIO,            // No such device or address.
  OPNOTSUPP,  // Operation not supported on socket (may be the same value as
              // [ENOTSUP]).
  OVERFLOW,   // Value too large to be stored in data type.
  OWNERDEAD,  // Previous owner died.
  PERM,       // Operation not permitted.
  PIPE,       // Broken pipe.
  PROTO,      // Protocol error.
  PROTONOSUPPORT,  // Protocol not supported.
  PROTOTYPE,       // Protocol wrong type for socket.
  RANGE,           // Result too large.
  ROFS,            // Read-only file system.
  SPIPE,           // Invalid seek.
  SRCH,            // No such process.
  STALE,           // Reserved.
  TIME,        // [OB XSR] [Option Start] Stream ioctl() timeout. [Option End]
  TIMEDOUT,    // Connection timed out.
  TXTBSY,      // Text file busy.
  WOULDBLOCK,  // Operation would block (may be the same value as [EAGAIN]).
  XDEV,        // Cross-device link.
  COUNT,
};

namespace impl {
constexpr std::size_t POSIX_CONDITION_COUNT =
    static_cast<std::size_t>(PosixCondition::COUNT);
constexpr std::size_t POSIX_DOMAIN_CODE = 0x1;
}  // namespace impl

class PosixStatusDomain final
    : public EnumStatusDomain<PosixCondition, impl::POSIX_CONDITION_COUNT> {
 public:
  DECLARE_COPY_DELETE(PosixStatusDomain);
  DECLARE_MOVE_DELETE(PosixStatusDomain);

  PosixStatusDomain() : EnumStatusDomain{impl::POSIX_DOMAIN_CODE, "posix"} {}
  ~PosixStatusDomain() = default;
};

enum class Win32Condition : std::size_t {
  INVALID_FUNCTION,     // Incorrect function.
  FILE_NOT_FOUND,       // The system cannot find the file specified.
  PATH_NOT_FOUND,       // The system cannot find the path specified.
  TOO_MANY_OPEN_FILES,  // The system cannot open the file.
  ACCESS_DENIED,        // Access is denied.
  INVALID_HANDLE,       // The handle is invalid.
  ARENA_TRASHED,        // The storage control blocks were destroyed.
  NOT_ENOUGH_MEMORY,    // Not enough storage is available to process this
  INVALID_BLOCK,        // The storage control block address is invalid.
  BAD_ENVIRONMENT,      // The environment is incorrect.
  BAD_FORMAT,           // An attempt was made to load a program with an
  INVALID_ACCESS,       // The access code is invalid.
  INVALID_DATA,         // The data is invalid.
  OUTOFMEMORY,          // Not enough storage is available to complete this
  INVALID_DRIVE,        // The system cannot find the drive specified.
  CURRENT_DIRECTORY,    // The directory cannot be removed.
  NOT_SAME_DEVICE,      // The system cannot move the file to a different
  NO_MORE_FILES,        // There are no more files.
  WRITE_PROTECT,        // The media is write protected.
  BAD_UNIT,             // The system cannot find the device specified.
  NOT_READY,            // The device is not ready.
  BAD_COMMAND,          // The device does not recognize the command.
  CRC,                  // Data error (cyclic redundancy check).
  BAD_LENGTH,         // The program issued a command but the command length is
  SEEK,               // The drive cannot locate a specific area or track on the
  NOT_DOS_DISK,       // The specified disk or diskette cannot be accessed.
  SECTOR_NOT_FOUND,   // The drive cannot find the sector requested.
  OUT_OF_PAPER,       // The printer is out of paper.
  WRITE_FAULT,        // The system cannot write to the specified device.
  READ_FAULT,         // The system cannot read from the specified device.
  GEN_FAILURE,        // A device attached to the system is not functioning.
  SHARING_VIOLATION,  // The process cannot access the file because it
  LOCK_VIOLATION,     // The process cannot access the file because another
  WRONG_DISK,         // The wrong diskette is in the drive. Insert %2 (Volume
  SHARING_BUFFER_EXCEEDED,  // Too many files opened for sharing.
  HANDLE_EOF,               // Reached the end of the file.
  HANDLE_DISK_FULL,         // The disk is full.
  NOT_SUPPORTED,            // The network request is not supported.
  REM_NOT_LIST,             // The remote computer is not available.
  DUP_NAME,                 // A duplicate name exists on the network.
  BAD_NETPATH,              // The network path was not found.
  NETWORK_BUSY,             // The network is busy.
  DEV_NOT_EXIST,            // The specified network resource or device is no
  TOO_MANY_CMDS,            // The network BIOS command limit has been reached.
  ADAP_HDW_ERR,             // A network adapter hardware error occurred.
  BAD_NET_RESP,             // The specified server cannot perform the requested
  UNEXP_NET_ERR,            // An unexpected network error occurred.
  BAD_REM_ADAP,             // The remote adapter is not compatible.
  PRINTQ_FULL,              // The printer queue is full.
  NO_SPOOL_SPACE,           // Space to store the file waiting to be printed is
  PRINT_CANCELED,           // Your file waiting to be printed was deleted.
  NETNAME_DELETED,          // The specified network name is no longer
  NETWORK_ACCESS_DENIED,    // Network access is denied.
  BAD_DEV_TYPE,             // The network resource type is not correct.
  BAD_NET_NAME,             // The network name cannot be found.
  TOO_MANY_NAMES,           // The name limit for the local computer network
  TOO_MANY_SESS,            // The network BIOS session limit was exceeded.
  SHARING_PAUSED,           // The remote server has been paused or is in the
  REQ_NOT_ACCEP,            // No more connections can be made to this remote
  REDIR_PAUSED,             // The specified printer or disk device has been
  FILE_EXISTS,              // The file exists.
  CANNOT_MAKE,              // The directory or file cannot be created.
  FAIL_I24,                 // Fail on INT 24.
  OUT_OF_STRUCTURES,        // Storage to process this request is not
  ALREADY_ASSIGNED,         // The local device name is already in use.
  INVALID_PASSWORD,         // The specified network password is not correct.
  INVALID_PARAMETER,        // The parameter is incorrect.
  NET_WRITE_FAULT,          // A write fault occurred on the network.
  NO_PROC_SLOTS,            // The system cannot start another process at this
  TOO_MANY_SEMAPHORES,      // Cannot create another system semaphore.
  EXCL_SEM_ALREADY_OWNED,   // The exclusive semaphore is owned by
  SEM_IS_SET,               // The semaphore is set and cannot be closed.
  TOO_MANY_SEM_REQUESTS,    // The semaphore cannot be set again.
  INVALID_AT_INTERRUPT_TIME,  // Cannot request exclusive semaphores at
  SEM_OWNER_DIED,             // The previous ownership of this semaphore has
  SEM_USER_LIMIT,             // Insert the diskette for drive %1.
  DISK_CHANGE,      // The program stopped because an alternate diskette was
  DRIVE_LOCKED,     // The disk is in use or locked by another process.
  BROKEN_PIPE,      // The pipe has been ended.
  OPEN_FAILED,      // The system cannot open the device or file specified.
  BUFFER_OVERFLOW,  // The file name is too long.
  DISK_FULL,        // There is not enough space on the disk.
  NO_MORE_SEARCH_HANDLES,  // No more internal file identifiers
  INVALID_TARGET_HANDLE,   // The target internal file identifier is
  INVALID_CATEGORY,        // The IOCTL call made by the application program
  INVALID_VERIFY_SWITCH,   // The verify-on-write switch parameter value
  BAD_DRIVER_LEVEL,        // The system does not support the command
  CALL_NOT_IMPLEMENTED,    // This function is not supported on this
  SEM_TIMEOUT,             // The semaphore timeout period has expired.
  INSUFFICIENT_BUFFER,     // The data area passed to a system call is too
  INVALID_NAME,          // The filename, directory name, or volume label syntax
  INVALID_LEVEL,         // The system call level is not correct.
  NO_VOLUME_LABEL,       // The disk has no volume label.
  MOD_NOT_FOUND,         // The specified module could not be found.
  PROC_NOT_FOUND,        // The specified procedure could not be found.
  WAIT_NO_CHILDREN,      // There are no child processes to wait for.
  CHILD_NOT_COMPLETE,    // The %1 application cannot be run in Win32
  DIRECT_ACCESS_HANDLE,  // Attempt to use a file handle to an open disk
  NEGATIVE_SEEK,         // An attempt was made to move the file pointer before
  SEEK_ON_DEVICE,        // The file pointer cannot be set on the specified
  IS_JOIN_TARGET,        // A JOIN or SUBST command cannot be used for a drive
  IS_JOINED,        // An attempt was made to use a JOIN or SUBST command on a
  IS_SUBSTED,       // An attempt was made to use a JOIN or SUBST command on
  NOT_JOINED,       // The system tried to delete the JOIN of a drive that is
  NOT_SUBSTED,      // The system tried to delete the substitution of a
  JOIN_TO_JOIN,     // The system tried to join a drive to a directory on a
  SUBST_TO_SUBST,   // The system tried to substitute a drive to a
  JOIN_TO_SUBST,    // The system tried to join a drive to a directory on
  SUBST_TO_JOIN,    // The system tried to SUBST a drive to a directory on
  BUSY_DRIVE,       // The system cannot perform a JOIN or SUBST at this
  SAME_DRIVE,       // The system cannot join or substitute a drive to or for
  DIR_NOT_ROOT,     // The directory is not a subdirectory of the root
  DIR_NOT_EMPTY,    // The directory is not empty.
  IS_SUBST_PATH,    // The path specified is being used in a substitute.
  IS_JOIN_PATH,     // Not enough resources are available to process this
  PATH_BUSY,        // The path specified cannot be used at this time.
  IS_SUBST_TARGET,  // An attempt was made to join or substitute a drive
  SYSTEM_TRACE,     // System trace information was not specified in your
  INVALID_EVENT_COUNT,  // The number of specified semaphore events for
  TOO_MANY_MUXWAITERS,  // DosMuxSemWait did not execute; too many
  INVALID_LIST_FORMAT,  // The DosMuxSemWait list is not correct.
  LABEL_TOO_LONG,       // The volume label you entered exceeds the label
  TOO_MANY_TCBS,        // Cannot create another thread.
  SIGNAL_REFUSED,       // The recipient process has refused the signal.
  DISCARDED,          // The segment is already discarded and cannot be locked.
  NOT_LOCKED,         // The segment is already unlocked.
  BAD_THREADID_ADDR,  // The address for the thread ID is not correct.
  BAD_ARGUMENTS,      // The argument string passed to DosExecPgm is not
                      // correct.
  BAD_PATHNAME,       // The specified path is invalid.
  SIGNAL_PENDING,     // A signal is already pending.
  MAX_THRDS_REACHED,  // No more threads can be created in the system.
  LOCK_FAILED,        // Unable to lock a region of a file.
  BUSY,               // The requested resource is in use.
  CANCEL_VIOLATION,   // A lock request was not outstanding for the
  ATOMIC_LOCKS_NOT_SUPPORTED,  // The file system does not support
  INVALID_SEGMENT_NUMBER,      // The system detected a segment number that
  INVALID_ORDINAL,             // The operating system cannot run %1.
  ALREADY_EXISTS,              // Cannot create a file when that file already
  INVALID_FLAG_NUMBER,         // The flag passed is not correct.
  SEM_NOT_FOUND,  // The specified system semaphore name was not found.
  INVALID_STARTING_CODESEG,   // The operating system cannot run %1.
  INVALID_STACKSEG,           // The operating system cannot run %1.
  INVALID_MODULETYPE,         // The operating system cannot run %1.
  INVALID_EXE_SIGNATURE,      // Cannot run %1 in Win32 mode.
  EXE_MARKED_INVALID,         // The operating system cannot run %1.
  BAD_EXE_FORMAT,             // %1 is not a valid Win32 application.
  ITERATED_DATA_EXCEEDS_64k,  // The operating system cannot run %1.
  INVALID_MINALLOCSIZE,       // The operating system cannot run %1.
  DYNLINK_FROM_INVALID_RING,  // The operating system cannot run this
  IOPL_NOT_ENABLED,          // The operating system is not presently configured
  INVALID_SEGDPL,            // The operating system cannot run %1.
  AUTODATASEG_EXCEEDS_64k,   // The operating system cannot run this
  RING2SEG_MUST_BE_MOVABLE,  // The code segment cannot be greater than
  RELOC_CHAIN_XEEDS_SEGLIM,  // The operating system cannot run %1.
  INFLOOP_IN_RELOC_CHAIN,    // The operating system cannot run %1.
  ENVVAR_NOT_FOUND,          // The system could not find the environment option
  NO_SIGNAL_SENT,            // No process in the command subtree has a signal
  FILENAME_EXCED_RANGE,      // The filename or extension is too long.
  RING2_STACK_IN_USE,        // The ring 2 stack is in use.
  META_EXPANSION_TOO_LONG,   // The global filename characters, * or ?,
  INVALID_SIGNAL_NUMBER,     // The signal being posted is not correct.
  THREAD_1_INACTIVE,         // The signal handler cannot be set.
  LOCKED,                    // The segment is locked and cannot be reallocated.
  TOO_MANY_MODULES,          // Too many dynamic-link modules are attached to
  NESTING_NOT_ALLOWED,       // Cannot nest calls to LoadModule.
  EXE_MACHINE_TYPE_MISMATCH,  // The image file %1 is valid, but is for
  BAD_PIPE,                   // The pipe state is invalid.
  PIPE_BUSY,                  // All pipe instances are busy.
  NO_DATA,                    // The pipe is being closed.
  PIPE_NOT_CONNECTED,         // No process is on the other end of the pipe.
  MORE_DATA,                  // More data is available.
  VC_DISCONNECTED,            // The session was canceled.
  INVALID_EA_NAME,            // The specified extended attribute name was
  EA_LIST_INCONSISTENT,       // The extended attributes are inconsistent.
  WAIT_TIMEOUT,               // The wait operation timed out.
  NO_MORE_ITEMS,              // No more data is available.
  CANNOT_COPY,                // The copy functions cannot be used.
  DIRECTORY,                  // The directory name is invalid.
  EAS_DIDNT_FIT,       // The extended attributes did not fit in the buffer.
  EA_FILE_CORRUPT,     // The extended attribute file on the mounted file
  EA_TABLE_FULL,       // The extended attribute table file is full.
  INVALID_EA_HANDLE,   // The specified extended attribute handle is
  EAS_NOT_SUPPORTED,   // The mounted file system does not support
  NOT_OWNER,           // Attempt to release mutex not owned by caller.
  TOO_MANY_POSTS,      // Too many posts were made to a semaphore.
  PARTIAL_COPY,        // Only part of a ReadProcessMemory or
  OPLOCK_NOT_GRANTED,  // The oplock request is denied.
  INVALID_OPLOCK_PROTOCOL,  // An invalid oplock acknowledgment was
  MR_MID_NOT_FOUND,         // The system cannot find message text for message
  INVALID_ADDRESS,          // Attempt to access invalid address.
  ARITHMETIC_OVERFLOW,      // Arithmetic result exceeded 32 bits.
  PIPE_CONNECTED,           // There is a process on other end of the pipe.
  PIPE_LISTENING,     // Waiting for a process to open the other end of the
  EA_ACCESS_DENIED,   // Access to the extended attribute was denied.
  OPERATION_ABORTED,  // The I/O operation has been aborted because of
  IO_INCOMPLETE,      // Overlapped I/O event is not in a signaled state.
  IO_PENDING,         // Overlapped I/O operation is in progress.
  NOACCESS,           // Invalid access to memory location.
  SWAPERROR,          // Error performing inpage operation.
  COUNT
};

namespace impl {
constexpr std::size_t WIN32_CONDITION_COUNT =
    static_cast<std::size_t>(Win32Condition::COUNT);
constexpr std::size_t WIN32_DOMAIN_CODE = 0x2;
}  // namespace impl

class Win32StatusDomain final
    : public EnumStatusDomain<Win32Condition, impl::WIN32_CONDITION_COUNT> {
 public:
  DECLARE_COPY_DELETE(Win32StatusDomain);
  DECLARE_MOVE_DELETE(Win32StatusDomain);

  Win32StatusDomain() : EnumStatusDomain{impl::WIN32_DOMAIN_CODE, "win32"} {}
  ~Win32StatusDomain() = default;
};

}  // namespace simon
