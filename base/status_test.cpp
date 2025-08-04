#include "base/status.hpp"

#include <functional>
#include <sstream>

#include "base/testing.hpp"

namespace simon {

constexpr bool StatusCodeShouldHaveSameBits() {
  StatusCode code;
  code.set_incident_bits(4u);
  code.set_condition_bits(5u);
  code.set_domain_bits(6u);

  return code.incident_bits() == 4u &&   //
         code.condition_bits() == 5u &&  //
         code.domain_bits() == 6u;
}

TEST_CASE("StatusCode") {
  StatusCode code;

  SECTION("ShouldDefaultToZero") {
    // Postconditions.
    REQUIRE(code.domain_bits() == 0u);
    REQUIRE(code.condition_bits() == 0u);
    REQUIRE(code.incident_bits() == 0u);
  }

  SECTION("ShouldConstructWithIncidentConditionDomainBits") {
    // Under Test.
    StatusCode other{4u, 5u, 6u};

    // Postconditions.
    REQUIRE(other.domain_bits() == 4u);
    REQUIRE(other.condition_bits() == 5u);
    REQUIRE(other.incident_bits() == 6u);
  }

  SECTION("ShouldHaveSameDomainBitsWhenSet") {
    // Under Test.
    code.set_domain_bits(5u);

    // Postconditions.
    REQUIRE(code.domain_bits() == 5u);
  }

  SECTION("ShouldHaveSameConditionBitsWhenSet") {
    // Under Test.
    code.set_condition_bits(5u);

    // Postconditions.
    REQUIRE(code.condition_bits() == 5u);
  }

  SECTION("ShouldHaveSameIncidentBitsWhenSet") {
    // Under Test.
    code.set_incident_bits(5u);

    // Postconditions.
    REQUIRE(code.incident_bits() == 5u);
  }

  StatusCode other;
  code.set_incident_bits(4u);
  code.set_condition_bits(5u);
  code.set_domain_bits(6u);

  SECTION("ShouldBeSameKindWithSameConditionAndDomain") {
    // Under Test.
    other.set_incident_bits(7u);
    other.set_condition_bits(5u);
    other.set_domain_bits(6u);

    // Postconditions.
    REQUIRE(code.is_same_kind(other));
  }

  SECTION("ShouldNotBeSameKindWithDifferentCondition") {
    // Under Test.
    other.set_incident_bits(4u);
    other.set_condition_bits(7u);
    other.set_domain_bits(6u);

    // Postconditions.
    REQUIRE(!code.is_same_kind(other));
  }

  SECTION("ShouldNotBeSameKindWithDifferentDomain") {
    // Under Test.
    other.set_incident_bits(4u);
    other.set_condition_bits(5u);
    other.set_domain_bits(7u);

    // Postconditions.
    REQUIRE(!code.is_same_kind(other));
  }

  SECTION("ShouldBeSameCodeWithSameIncidentConditionDomain") {
    // Under Test.
    other.set_incident_bits(4u);
    other.set_condition_bits(5u);
    other.set_domain_bits(6u);

    // Postconditions.
    REQUIRE(code.is_same_code(other));
  }

  SECTION("ShouldNotBeSameCodeWithDifferentIncident") {
    // Under Test.
    other.set_incident_bits(7u);
    other.set_condition_bits(5u);
    other.set_domain_bits(6u);

    // Postconditions.
    REQUIRE(!code.is_same_code(other));
  }

  SECTION("ShouldNotBeSameCodeWithDifferentCondition") {
    // Under Test.
    other.set_incident_bits(4u);
    other.set_condition_bits(7u);
    other.set_domain_bits(6u);

    // Postconditions.
    REQUIRE(!code.is_same_code(other));
  }

  SECTION("ShouldNotBeSameCodeWithDifferentDomain") {
    // Under Test.
    other.set_incident_bits(4u);
    other.set_condition_bits(5u);
    other.set_domain_bits(7u);

    // Postconditions.
    REQUIRE(!code.is_same_code(other));
  }

  SECTION("ShouldBeConstexprBits") {
    // Under Test.
    constexpr bool pass = StatusCodeShouldHaveSameBits();

    // Postconditions.
    REQUIRE(pass);
  }
}

enum class Quark { UP, DOWN, TOP, BOTTOM, STRANGE, CHARM, COUNT };
constexpr std::size_t QUARK_CONDITION_COUNT =
    static_cast<std::size_t>(Quark::COUNT);

class QuarkStatusDomain final
    : public EnumStatusDomain<Quark, QUARK_CONDITION_COUNT> {
  using BaseType = EnumStatusDomain<Quark, QUARK_CONDITION_COUNT>;

 public:
  using BaseType::BaseType;

  std::string_view status_kind_message_of(
      const StatusKind* kind) const noexcept override {
    message_of_condition++;
    return BaseType::status_kind_message_of(kind);
  }

  std::string_view status_base_message_of(
      const StatusBase* status) const noexcept override {
    message_of_incident++;
    return BaseType::status_base_message_of(status);
  }

  std::source_location status_base_location_of(
      const StatusBase* status) const noexcept override {
    location_of_incident++;
    return BaseType::status_base_location_of(status);
  }

  mutable std::size_t message_of_condition = 0;
  mutable std::size_t message_of_incident = 0;
  mutable std::size_t location_of_incident = 0;
};

class QuarkKindDomain final
    : public EnumStatusKindDomain<Quark, QUARK_CONDITION_COUNT> {
  using BaseType = EnumStatusKindDomain<Quark, QUARK_CONDITION_COUNT>;

 public:
  using BaseType::BaseType;
};

template <>
const std::array<StatusConditionEntry, QUARK_CONDITION_COUNT>
    EnumStatusKindConditionMixin<Quark, QUARK_CONDITION_COUNT>::conditions_ = {
        StatusConditionEntry{"UP"},       //
        StatusConditionEntry{"DOWN"},     //
        StatusConditionEntry{"TOP"},      //
        StatusConditionEntry{"BOTTOM"},   //
        StatusConditionEntry{"STRANGE"},  //
        StatusConditionEntry{"CHARM"},    //
};

QuarkStatusDomain domain{42u, "quark"};
std::source_location location0 = std::source_location::current();
std::source_location location1 = std::source_location::current();
std::string message0 = "mark";
std::string message1 = "bark";
constexpr int platform_error0 = -50;
constexpr int platform_error1 = -42;

TEST_CASE("EnumStatusDomain") {
  SECTION("ShouldNotFetchStatusLocationByDefaultWhenRaiseStatus") {
    // Under Test.
    Status status = domain.raise_status(Quark::UP);

    // Postconditions.
    REQUIRE(domain.location_of_incident == 0u);
  }

  SECTION("ShouldNotFetchStatusMessageByDefaultWhenRaiseStatus") {
    // Under Test.
    Status status = domain.raise_status(Quark::UP);

    // Postconditions.
    REQUIRE(domain.message_of_incident == 0u);
  }

  SECTION("ShouldFetchStatusMessageOnDemandWhenRaiseStatus") {
    // Preconditions.
    Status status = domain.raise_status(Quark::UP, message0);

    // Under Test.
    std::string_view message = status.message();

    // Postconditions.
    REQUIRE(domain.message_of_incident > 0u);
    REQUIRE(message.size() > 0u);
  }

  SECTION("ShouldFetchStatusLocationOnDemandWhenRaiseStatusHere") {
    // Preconditions.
    Status status = domain.raise_status_here(Quark::UP);

    // Under Test.
    std::source_location location = status.location();

    // Postconditions.
    REQUIRE(domain.location_of_incident > 0u);
    REQUIRE(location.line() > 0u);
  }
}

TEST_CASE("Status") {
  SECTION("ShouldHaveCurrentLocationAsRaiseStatusHere") {
    // Preconditions.
    auto raised_location = std::source_location::current();
    Status status = domain.raise_status_here(Quark::UP);

    // Under Test.
    std::source_location location = status.location();

    // Postconditions.
    REQUIRE(location.line() - 1 == raised_location.line());
  }

  // Precondition.
  Status status = domain.raise_status_here(Quark::UP, message0, location0);

  SECTION("ShouldHaveSameMessageAsWhenRaiseStatusHere") {
    // Under Test.
    std::string_view status_message = status.message();

    // Postconditions.
    REQUIRE(status_message == message0);
    REQUIRE(status_message != message1);
  }

  SECTION("ShouldHaveSameLocationAsWhenRaiseStatusHere") {
    // Under Test.
    std::source_location location = status.location();

    // Postconditions.
    REQUIRE(location.line() == location0.line());
    REQUIRE(location.line() != location1.line());
  }

  // Precondition.
  Status error_status = domain.raise_error_here(  //
      Quark::UP, platform_error0, message0, location0);

  SECTION("ShouldHaveSameMessageAsWhenRaiseErrorHere") {
    // Under Test.
    std::string_view status_message = error_status.message();

    // Postconditions.
    REQUIRE(status_message == message0);
    REQUIRE(status_message != message1);
  }

  SECTION("ShouldHaveSameLocationAsWhenRaiseErrorHere") {
    // Under Test.
    std::source_location location = error_status.location();

    // Postconditions.
    REQUIRE(location.line() == location0.line());
    REQUIRE(location.line() != location1.line());
  }

  SECTION("ShouldHaveSamePlatformCodeAsWhenRaiseErrorHere") {
    // Under Test.
    int platform_error = error_status.platform_error();

    // Postconditions.
    REQUIRE(platform_error == platform_error0);
    REQUIRE(platform_error != platform_error1);
  }

  // Under Test.
  StatusDetached detached = status.detach_copy();

  SECTION("ShouldHaveSameMessageAsDetachedCopy") {
    // Postconditions.
    REQUIRE(detached.message() == status.message());
  }

  SECTION("ShouldHaveSameLocationAsDetachedCopy") {
    // Postconditions.
    REQUIRE(detached.location().line() == status.location().line());
  }

  SECTION("ShouldHaveSameLocationAsDetachedCopy") {
    // Postconditions.
    REQUIRE(detached.platform_error() == status.platform_error());
  }

  SECTION("ShouldBeEqualToStatusWithSameIncident") {
    // Under Test.
    Status copy = status;

    // Postconditions.
    REQUIRE(copy == status);
    REQUIRE(status == copy);
  }

  SECTION("ShouldNotBeEqualToStatusWithDifferentIncident") {
    // Under Test.
    Status other = domain.raise_status(Quark::UP);

    // Postconditions.
    REQUIRE(other != status);
    REQUIRE(status != other);
  }

  SECTION("ShouldNotBeEqualToStatusWithDifferentCondition") {
    // Under Test.
    Status other = domain.raise_status(Quark::DOWN);

    // Postconditions.
    REQUIRE(other != status);
    REQUIRE(status != other);
  }

  SECTION("ShouldBeEqualToKindWithSameConditionAndDomain") {
    // Under Test.
    Status other = domain.raise_status(Quark::UP);

    // Postconditions.
    REQUIRE(status == other.kind());
    REQUIRE(other.kind() == status);
  }

  SECTION("ShouldNotBeEqualToKindWithDifferentCondition") {
    // Under Test.
    Status other = domain.raise_status(Quark::DOWN);

    // Postconditions.
    REQUIRE(status != other.kind());
    REQUIRE(other.kind() != status);
  }

  SECTION("ShouldHaveEqualKindWithSameConditionAndDomain") {
    // Under Test.
    Status other = domain.raise_status(Quark::UP);

    // Postconditions.
    REQUIRE(status.kind() == other.kind());
    REQUIRE(other.kind() == status.kind());
  }

  SECTION("ShouldNotHaveEqualKindWithDifferentCondition") {
    // Under Test.
    Status other = domain.raise_status(Quark::DOWN);

    // Postconditions.
    REQUIRE(status.kind() != other.kind());
    REQUIRE(other.kind() != status.kind());
  }

  SECTION("ShouldBeEquivalentToStatusWithSameCondition") {
    // Under Test.
    Status other = domain.raise_status(Quark::UP);

    // Postconditions.
    REQUIRE(status.has_equivalent_condition_as(other.kind()));
    REQUIRE(other.has_equivalent_condition_as(status.kind()));
  }

  SECTION("ShouldNotBeEquivalentToStatusWithDifferentCondition") {
    // Under Test.
    Status other = domain.raise_status(Quark::DOWN);

    // Postconditions.
    REQUIRE(!status.has_equivalent_condition_as(other.kind()));
    REQUIRE(!other.has_equivalent_condition_as(status.kind()));
  }

  SECTION("ShouldBeOutStreamable") {
    // Preconditions.
    std::stringstream ss;

    // Under Test.
    ss << status;

    // Postconditions.
    REQUIRE(ss.str().size() > 0u);
    REQUIRE(ss.str() == status.message());
  }
}

TEST_CASE("StatusKind") {
  StatusKind down = domain.watch_kind(Quark::DOWN);
  StatusKind up = domain.watch_kind(Quark::UP);
  Status status = domain.raise_status(Quark::UP);

  SECTION("ShouldHaveMessageDescribingCondition") {
    // Under Test.
    std::string_view message0 = up.message();
    std::string_view message1 = down.message();

    // Postconditions.
    REQUIRE(message0 == "UP");
    REQUIRE(message1 == "DOWN");
  }

  SECTION("ShouldBeTrueWhenComparedWithIncidentOfSameCondition") {
    // Postconditions.
    REQUIRE(status == up);
    REQUIRE(up == status);
  }

  SECTION("ShouldBeFalseWhenComparedWithIncidentOfDifferentCondition") {
    // Postconditions.
    REQUIRE(status != down);
    REQUIRE(down != status);
  }

  SECTION("ShouldBeEquivalentToIncidentOfSameCondition") {
    // Postconditions.
    REQUIRE(status.has_equivalent_condition_as(up));
  }

  SECTION("ShouldNotBeEquivalentToIncidentOfDifferentCondition") {
    // Postconditions.
    REQUIRE(!status.has_equivalent_condition_as(down));
  }

  SECTION("ShouldBeOutStreamable") {
    // Preconditions.
    std::stringstream ss;

    // Under Test.
    ss << up;

    // Postconditions.
    REQUIRE(ss.str().size() > 0u);
    REQUIRE(ss.str() == up.message());
  }
}

constexpr QuarkKindDomain quarks{42u, "quark"};

constexpr bool StatusKindShouldHaveSameMessage() {
  StatusKind kind = quarks.watch_kind(Quark::CHARM);
  return kind.message() == "CHARM";
}
constexpr bool StatusKindShouldCompareSame() {
  StatusKind kind_a = quarks.watch_kind(Quark::CHARM);
  StatusKind kind_b = quarks.watch_kind(Quark::CHARM);
  return kind_a == kind_b;
}
constexpr bool StatusKindShouldCompareDifferent() {
  StatusKind kind_a = quarks.watch_kind(Quark::CHARM);
  StatusKind kind_b = quarks.watch_kind(Quark::STRANGE);
  return kind_a != kind_b;
}

TEST_CASE("EnumStatusKindDomain") {
  SECTION("ShouldHaveConstexprName") {
    // Under Test.
    constexpr std::string_view name = quarks.name();

    // Postconditions.
    REQUIRE(name == "quark");
  }
}

TEST_CASE("StaticEnumStatusDomain") {
  SECTION("ShouldRaiseStatusFromDifferentDomain") {
    // Preconditions.
    Status other = domain.raise_status(Quark::TOP);

    // Under Test.
    Status status0 = raise(Quark::TOP);
    Status status1 = raise(Quark::TOP);

    // Postconditions.
    REQUIRE(status0.kind() != other.kind());
    REQUIRE(status0.kind() == status1.kind());
  }

  SECTION("ShouldRaiseMultipleStatusWithDifferentMessages") {
    // Under Test.
    Status status0 = raise(Quark::TOP, message0);
    Status status1 = raise(Quark::TOP, message1);

    // Postconditions.
    REQUIRE(status0.message() == message0);
    REQUIRE(status1.message() == message1);
  }

  SECTION("ShouldRaiseMultipleStatusHereWithDifferentLocations") {
    // Under Test.
    Status status0 = raise_here(Quark::TOP, message0, location0);
    Status status1 = raise_here(Quark::TOP, message1, location1);

    // Postconditions.
    REQUIRE(status0.location().line() == location0.line());
    REQUIRE(status1.location().line() == location1.line());
  }

  SECTION("ShouldRaiseStatusHereWithLocationByDefault") {
    // Under Test.
    Status status = raise_here(Quark::TOP);

    // Postconditions.
    REQUIRE(status.location().line() > 0u);
  }

  SECTION("ShouldWatchForCondition") {
    // Preconditions.
    Status status = raise(Quark::TOP);
    StatusKind kind = watch(Quark::TOP);

    // Under Test.
    bool found = false;
    if (status == kind || kind == status) {
      found = true;
    }

    // Postconditions.
    REQUIRE(found);
  }
}

TEST_CASE("ThreadLocalEnumStatusDomain") {
  SECTION("ShouldRaiseStatusFromDifferentDomain") {
    // Preconditions.
    Status other = domain.raise_status(Quark::TOP);

    // Under Test.
    Status status = raise_thread_local(Quark::TOP);

    // Postconditions.
    REQUIRE(status.kind() != other.kind());
  }

  SECTION("ShouldRaiseMultipleStatusWithSingleMessagePerThread") {
    // Under Test.
    Status status0 = raise_thread_local(Quark::TOP, message0);
    Status status1 = raise_thread_local(Quark::TOP, message1);

    // Postconditions.
    REQUIRE(status0.message() != message0);
    REQUIRE(status0.message() == message1);
    REQUIRE(status1.message() == message1);
  }

  SECTION("ShouldRaiseMultipleStatusHereWithSingleLocationPerThread") {
    // Under Test.
    Status status0 = raise_here_thread_local(Quark::TOP, message0, location0);
    Status status1 = raise_here_thread_local(Quark::TOP, message1, location1);

    // Postconditions.
    REQUIRE(status0.location().line() != location0.line());
    REQUIRE(status0.location().line() == location1.line());
    REQUIRE(status1.location().line() == location1.line());
  }

  SECTION("ShouldRaiseStatusHereWithLocationByDefault") {
    // Under Test.
    Status status = raise_here_thread_local(Quark::TOP);

    // Postconditions.
    REQUIRE(status.location().line() > 0u);
  }

  SECTION("ShouldWatchForCondition") {
    // Preconditions.
    Status status = raise_thread_local(Quark::TOP);
    StatusKind kind = watch_thread_local(Quark::TOP);

    // Under Test.
    bool found = false;
    if (status == kind || kind == status) {
      found = true;
    }

    // Postconditions.
    REQUIRE(found);
  }
}

TEST_CASE("PosixStatusDomain") {
  PosixStatusDomain posix_domain;

  SECTION("ShouldRaisePosixEAGAINWithMessage") {
    // Preconditions.
    StatusKind kind = posix_domain.watch_kind(PosixError::AGAIN);

    // Under Test.
    Status status = posix_domain.raise_status(PosixError::AGAIN);

    // Postconditions.
    REQUIRE(status == kind);
    REQUIRE(kind.message() == "Resource unavailable, try again.");
  }
}

TEST_CASE("Win32StatusDomain") {
  Win32StatusDomain win32_domain;

  SECTION("ShouldRaiseWin32ERROR_BUSYWithMessage") {
    // Preconditions.
    StatusKind kind = win32_domain.watch_kind(Win32Error::BUSY);

    // Under Test.
    Status status = win32_domain.raise_status(Win32Error::BUSY);

    // Postconditions.
    REQUIRE(status == kind);
    REQUIRE(kind.message() == "ERROR_BUSY");
  }
}

}  // namespace simon
