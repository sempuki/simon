#pragma once

#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <print>
#include <source_location>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#ifdef __GNUC__  // Clang also supports this header.
#include <cxxabi.h>
#endif

#define DECLARE_COPY_DEFAULT(class_name__)              \
  class_name__(const class_name__&) noexcept = default; \
  class_name__& operator=(const class_name__&) noexcept = default;

#define DECLARE_COPY_DELETE(class_name__)     \
  class_name__(const class_name__&) = delete; \
  class_name__& operator=(const class_name__&) = delete;

#define DECLARE_MOVE_DEFAULT(class_name__)         \
  class_name__(class_name__&&) noexcept = default; \
  class_name__& operator=(class_name__&&) noexcept = default;

#define DECLARE_MOVE_DELETE(class_name__) \
  class_name__(class_name__&&) = delete;  \
  class_name__& operator=(class_name__&&) = delete;

#define DECLARE_COPY_DEFAULT_CONSTEXPR(class_name__)              \
  constexpr class_name__(const class_name__&) noexcept = default; \
  constexpr class_name__& operator=(const class_name__&) noexcept = default;

#define DECLARE_MOVE_DEFAULT_CONSTEXPR(class_name__)         \
  constexpr class_name__(class_name__&&) noexcept = default; \
  constexpr class_name__& operator=(class_name__&&) noexcept = default;

#define DEFINE_MOVE_FROM_SWAP(class_name__)               \
  class_name__(class_name__&& that) noexcept {            \
    using std::swap;                                      \
    swap(*this, that);                                    \
  }                                                       \
  class_name__& operator=(class_name__&& that) noexcept { \
    if (this != &that) {                                  \
      using std::swap;                                    \
      class_name__ temp{std::move(that)};                 \
      swap(*this, temp);                                  \
    }                                                     \
    return *this;                                         \
  }

#define DERIVE_FINAL_WITH_CONSTRUCTORS(derived_name__, base_name__) \
  class derived_name__ final : public base_name__ {                 \
   public:                                                          \
    using base_name__::base_name__;                                 \
  };

#define DECLARE_USED(expression__) ((void)(sizeof(expression__)))
#define DECLARE_UNUSED(expression__) ((void)(sizeof(expression__)))

#define CHECK_CONTRACT__(condition__, kind__)                        \
  if (!(condition__)) [[unlikely]] {                                 \
    const auto current_location__ = std::source_location::current(); \
    throw std::logic_error(std::format(                              \
        "[{}] {} Failed {}: {}", current_location__.file_name(),     \
        current_location__.function_name(), kind__, #condition__));  \
  }

#define CHECK_PRECONDITION(precondition__) \
  CHECK_CONTRACT__(precondition__, "Precondition")
#define CHECK_POSTCONDITION(postcondition__) \
  CHECK_CONTRACT__(postcondition__, "Postcondition")
#define CHECK_INVARIANT(invariant__) CHECK_CONTRACT__(invariant__, "Invariant")
#define CHECK_UNREACHABLE() CHECK_CONTRACT__(false, "Unreachable")

namespace simon {

inline std::size_t allocate_static_increment() {
  static std::atomic<std::size_t> increment = 0;
  return increment++;
}

template <std::integral ToType, std::integral FromType>
ToType narrow_cast(FromType from) {
  ToType to(from);
  CHECK_PRECONDITION(std::cmp_equal(to, from))
  return to;
}

template <typename Type>
class CheckedPointer {
 public:
  DECLARE_COPY_DEFAULT(CheckedPointer);
  DECLARE_MOVE_DEFAULT(CheckedPointer);

  CheckedPointer() = default;
  ~CheckedPointer() = default;

  explicit CheckedPointer(Type& value) : arg_{&value} {}
  template <typename Derived>
    requires(std::is_base_of_v<Type, Derived> && !std::is_same_v<Type, Derived>)
  CheckedPointer(CheckedPointer<Derived> that) : arg_{that.get()} {}

  explicit operator bool() const { return arg_; }

  Type& operator*() const {
    CHECK_PRECONDITION(arg_);
    return *arg_;
  }
  Type* operator->() const {
    CHECK_PRECONDITION(arg_);
    return arg_;
  }
  Type* get() const {
    CHECK_PRECONDITION(arg_);
    return arg_;
  }

 protected:
  Type* arg_ = nullptr;
};

struct Unused {};
inline constexpr Unused unused;

// Out argument:
// - May be explicitly initialized.
// - May be implicitly unused.
// - May upcast.
// - May be forwarded by copy/move.
// - May *not* manage lifetimes.
// - May *not* be converted to InOut.
template <typename Type>
class Out : public CheckedPointer<Type> {
  using BaseType = CheckedPointer<Type>;

 public:
  DECLARE_COPY_DEFAULT(Out);
  DECLARE_MOVE_DEFAULT(Out);

  Out() = delete;
  ~Out() = default;

  Out(Unused) {}
  explicit Out(Type& value) : BaseType{value} {}

  template <typename Derived>
    requires(std::is_base_of_v<Type, Derived> && !std::is_same_v<Type, Derived>)
  Out(Out<Derived> that) : BaseType{that} {}
};

// In-out argument:
// - Must be explicitly initialized.
// - May *not* be unused.
// - May upcast.
// - May be forwarded by copy/move.
// - May *not* manage lifetimes.
// - May be converted to Out.
template <typename Type>
class InOut : public Out<Type> {
  using BaseType = Out<Type>;

 public:
  DECLARE_COPY_DEFAULT(InOut);
  DECLARE_MOVE_DEFAULT(InOut);

  InOut() = delete;
  ~InOut() = default;

  explicit InOut(Type& value) : BaseType{value} { CHECK_INVARIANT(this->arg_); }

  template <typename Derived>
    requires(std::is_base_of_v<Type, Derived> && !std::is_same_v<Type, Derived>)
  InOut(InOut<Derived> that) : BaseType{that} {
    CHECK_INVARIANT(this->arg_);
  }
};

// Class member dependency:
// - Must be explicitly initialized.
// - May *not* be unused.
// - May upcast.
// - May be forwarded by copy/move.
// - May *not* manage lifetimes.
// - May be converted to InOut.
template <typename Type>
class Depend final : public InOut<Type> {
  using BaseType = InOut<Type>;

 public:
  using BaseType::BaseType;
};

template <typename T>
Out(T& value) -> Out<T>;
template <typename T>
InOut(T& value) -> InOut<T>;
template <typename T>
Depend(T& value) -> Depend<T>;

template <class... CallableTypes>
struct Overloaded : CallableTypes... {
  using CallableTypes::operator()...;
};

template <class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

template <typename CallableType, typename... ArgumentTypes,
          typename ContinuationType>
void invoke_with_continuation(ContinuationType&& continuation,
                              CallableType&& callable,
                              ArgumentTypes&&... arguments) {
  using CallableResultType =
      std::invoke_result_t<CallableType, ArgumentTypes...>;

  if constexpr (std::is_invocable_v<ContinuationType, CallableResultType>) {
    std::invoke(
        continuation,
        std::invoke(callable, std::forward<ArgumentTypes>(arguments)...));
  } else {
    std::invoke(callable, std::forward<ArgumentTypes>(arguments)...);
    std::invoke(continuation);
  }
}

struct Empty final {};

#ifdef __GNUC__  // Clang also supports this header.
inline std::string demangle(const std::string& name) {
  char buffer[1024];  // Avoid realloc() within demangle.
  size_t inout_size = std::size(buffer);
  int out_status = 0;
  abi::__cxa_demangle(name.c_str(), buffer, &inout_size, &out_status);
  return out_status ? name : buffer;
}
#elif
inline std::string demangle(const std::string& name) { return name; }
#endif

template <typename Type>
std::string to_type_string() {
  return demangle(typeid(Type).name());
}

template <typename Type>
std::string to_type_string(Type&& object) {
  return demangle(typeid(decltype(object)).name());
}

template <typename ObjectType>
void dump_object_bytes(const ObjectType& object) {
  std::print("** {}: ", to_type_string<ObjectType>());
  auto* begin_address = reinterpret_cast<const char*>(std::addressof(object));
  auto* end_address = reinterpret_cast<const char*>(std::addressof(object) + 1);
  for (; begin_address != end_address; ++begin_address) {
    std::print("{:02x} ", *begin_address);
  }
  std::print("\n");
}

inline std::size_t stable_hash(std::string_view str) {
  static const auto shuffle_ = [](std::uint64_t block) {
    return  // clang-format off
      ((block & 0xFFFF'0000'0000'0000) >> 16) |
      ((block & 0x0000'FFFF'0000'0000) >> 32) |
      ((block & 0x0000'0000'FFFF'0000) << 32) |
      ((block & 0x0000'0000'0000'FFFF) << 16);
    // clang-format on
  };

  static const auto diffuse_ = [](std::uint64_t block, std::uint64_t a,
                                  std::uint64_t b) {
    return (block * a) ^ (~block * b);
  };

  constexpr std::uint64_t m1 = 0xC2B2AE35C2B2AE35;
  constexpr std::uint64_t m2 = 0x42F0E1EBA9EA3693;
  constexpr std::uint64_t m3 = 0xC96C5795D7870F42;
  constexpr std::uint64_t block_size = 8;

  std::size_t i = 0;
  std::uint64_t block;
  std::uint64_t result = diffuse_(str.size(), m1, m2);

  for (; i < str.size() && block_size <= str.size(); i += block_size) {
    std::memcpy(&block, str.data() + i, block_size);
    result = shuffle_(result) ^ diffuse_(block, ~m2, m3);
  }

  for (; i < str.size(); i++) {
    result = shuffle_(result) ^ diffuse_(str[i], m3, ~m1);
  }

  return diffuse_(result, m2, ~m3);
}

}  // namespace simon
