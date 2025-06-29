#include "base/status.hpp"

#include <functional>
#include <sstream>

#include "base/testing.hpp"

namespace simon {

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
}

enum class TestCondition { UP, DOWN, TOP, BOTTOM, STRANGE, CHARM, COUNT };
constexpr std::size_t TEST_CONDITION_COUNT =
    static_cast<std::size_t>(TestCondition::COUNT);

class TestStatusDomain final
    : public EnumStatusDomain<TestCondition, TEST_CONDITION_COUNT> {
 public:
  using EnumStatusDomain<TestCondition, TEST_CONDITION_COUNT>::EnumStatusDomain;

  std::string_view message_of(StatusKind kind) const noexcept override {
    message_of_condition++;
    return conditions_[condition_code_of(kind)].message;
  }

  std::string_view message_of(Status status) const noexcept override {
    message_of_incident++;
    return incidents_[incident_code_of(status)].message;
  }

  std::source_location location_of(Status status) const noexcept override {
    location_of_incident++;
    return incidents_[incident_code_of(status)].location;
  }

  bool has_equivalent_condition_of(Status status,
                                   StatusKind kind) const noexcept override {
    equivalent_condition_of++;
    return false;
  }

  mutable std::size_t message_of_condition = 0;
  mutable std::size_t message_of_incident = 0;
  mutable std::size_t location_of_incident = 0;
  mutable std::size_t equivalent_condition_of = 0;
};

template <>
const std::array<StatusConditionEntry, TEST_CONDITION_COUNT>
    EnumStatusDomain<TestCondition, TEST_CONDITION_COUNT>::conditions_ = {
        StatusConditionEntry{"UP"},       //
        StatusConditionEntry{"DOWN"},     //
        StatusConditionEntry{"TOP"},      //
        StatusConditionEntry{"BOTTOM"},   //
        StatusConditionEntry{"STRANGE"},  //
        StatusConditionEntry{"CHARM"},    //
};

TestStatusDomain domain{42u, "test"};
std::source_location location0 = std::source_location::current();
std::source_location location1 = std::source_location::current();
std::string message0 = "mark";
std::string message1 = "bark";

TEST_CASE("StatusDomain") {
  SECTION("ShouldNotFetchStatusLocationByDefault") {
    // Under Test.
    Status status = domain.raise_incident(TestCondition::UP);

    // Postconditions.
    REQUIRE(domain.location_of_incident == 0u);
  }

  SECTION("ShouldNotFetchStatusMessageByDefault") {
    // Under Test.
    Status status = domain.raise_incident(TestCondition::UP);

    // Postconditions.
    REQUIRE(domain.message_of_incident == 0u);
  }

  SECTION("ShouldFetchStatusMessageOnDemand") {
    // Preconditions.
    Status status = domain.raise_incident(TestCondition::UP, message0);

    // Under Test.
    std::string_view message = status.message();

    // Postconditions.
    REQUIRE(domain.message_of_incident > 0u);
    REQUIRE(message.size() > 0u);
  }

  SECTION("ShouldFetchStatusLocationOnDemand") {
    // Preconditions.
    Status status = domain.raise_incident_here(TestCondition::UP);

    // Under Test.
    std::source_location location = status.location();

    // Postconditions.
    REQUIRE(domain.location_of_incident > 0u);
    REQUIRE(location.line() > 0u);
  }

  // SECTION("ShouldBeConstexpr") {
  //   // Under Test.
  //   constexpr TestStatusDomain domain{42u, "test"};

  //   // Postconditions.
  //   REQUIRE(true);
  // }
}

TEST_CASE("Status") {
  SECTION("ShouldHaveCurrentLocationWhenRaisedHere") {
    // Preconditions.
    auto raised_location = std::source_location::current();
    Status status = domain.raise_incident_here(TestCondition::UP);

    // Under Test.
    std::source_location location = status.location();

    // Postconditions.
    REQUIRE(location.line() - 1 == raised_location.line());
  }

  Status status =
      domain.raise_incident_here(TestCondition::UP, message0, location0);

  SECTION("ShouldHaveSameMessageWhenRaisedHere") {
    // Under Test.
    std::string_view status_message = status.message();

    // Postconditions.
    REQUIRE(status_message == message0);
  }

  SECTION("ShouldHaveSameLocationWhenRaisedHere") {
    // Under Test.
    std::source_location location = status.location();

    // Postconditions.
    REQUIRE(location.line() == location0.line());
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
    Status other = domain.raise_incident(TestCondition::UP);

    // Postconditions.
    REQUIRE(other != status);
    REQUIRE(status != other);
  }

  SECTION("ShouldNotBeEqualToStatusWithDifferentCondition") {
    // Under Test.
    Status other = domain.raise_incident(TestCondition::DOWN);

    // Postconditions.
    REQUIRE(other != status);
    REQUIRE(status != other);
  }

  SECTION("ShouldBeEqualToKindWithSameConditionAndDomain") {
    // Under Test.
    Status other = domain.raise_incident(TestCondition::UP);

    // Postconditions.
    REQUIRE(status == other.kind());
    REQUIRE(other.kind() == status);
  }

  SECTION("ShouldNotBeEqualToKindWithDifferentCondition") {
    // Under Test.
    Status other = domain.raise_incident(TestCondition::DOWN);

    // Postconditions.
    REQUIRE(status != other.kind());
    REQUIRE(other.kind() != status);
  }

  SECTION("ShouldHaveEqualKindWithSameConditionAndDomain") {
    // Under Test.
    Status other = domain.raise_incident(TestCondition::UP);

    // Postconditions.
    REQUIRE(status.kind() == other.kind());
    REQUIRE(other.kind() == status.kind());
  }

  SECTION("ShouldNotHaveEqualKindWithDifferentCondition") {
    // Under Test.
    Status other = domain.raise_incident(TestCondition::DOWN);

    // Postconditions.
    REQUIRE(status.kind() != other.kind());
    REQUIRE(other.kind() != status.kind());
  }

  SECTION("ShouldBeEquivalentToStatusWithSameCondition") {
    // Under Test.
    Status other = domain.raise_incident(TestCondition::UP);

    // Postconditions.
    REQUIRE(status.has_equivalent_condition_as(other));
    REQUIRE(other.has_equivalent_condition_as(status));
  }

  SECTION("ShouldNotBeEquivalentToStatusWithDifferentCondition") {
    // Under Test.
    Status other = domain.raise_incident(TestCondition::DOWN);

    // Postconditions.
    REQUIRE(!status.has_equivalent_condition_as(other));
    REQUIRE(!other.has_equivalent_condition_as(status));
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
  StatusKind down = domain.watch_condition(TestCondition::DOWN);
  StatusKind up = domain.watch_condition(TestCondition::UP);
  Status status = domain.raise_incident(TestCondition::UP);

  SECTION("ShouldHaveMessageDescribingCondition") {
    // Under Test.
    std::string_view message = up.message();

    // Postconditions.
    REQUIRE(message == "UP");
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

  // SECTION("ShouldBeConstexpr") {
  //   // Under Test.
  //   constexpr StatusKind kind = domain.watch_condition(TestCondition::UP);

  //   // Postconditions.
  //   REQUIRE(status == kind);
  // }
}

TEST_CASE("StaticEnumStatusDomain") {
  SECTION("ShouldRaiseStatusFromStaticDomain") {
    // Preconditions.
    Status other = domain.raise_incident(TestCondition::TOP);

    // Under Test.
    Status status0 = raise(TestCondition::TOP);
    Status status1 = raise(TestCondition::TOP);

    // Postconditions.
    REQUIRE(status0.kind() != other.kind());
    REQUIRE(status0.kind() == status1.kind());
  }

  SECTION("ShouldRaiseStatusWithMessage") {
    // Under Test.
    Status status0 = raise(TestCondition::TOP, message0);
    Status status1 = raise(TestCondition::TOP, message1);

    // Postconditions.
    REQUIRE(status0.message() == message0);
    REQUIRE(status1.message() == message1);
  }

  SECTION("ShouldRaiseStatusHereWithLocation") {
    // Under Test.
    Status status0 = raise_here(TestCondition::TOP, message0, location0);
    Status status1 = raise_here(TestCondition::TOP, message1, location1);

    // Postconditions.
    REQUIRE(status0.location().line() == location0.line());
    REQUIRE(status1.location().line() == location1.line());
  }

  SECTION("ShouldRaiseStatusHereWithLocationByDefault") {
    // Under Test.
    Status status0 = raise_here(TestCondition::TOP);
    Status status1 = raise_here(TestCondition::TOP);

    // Postconditions.
    REQUIRE(status0.location().line() > 0u);
    REQUIRE(status1.location().line() > 0u);
  }

  SECTION("ShouldWatchForCondition") {
    // Preconditions.
    Status status = raise(TestCondition::TOP);
    StatusKind kind = watch(TestCondition::TOP);

    // Under Test.
    bool found = false;
    if (status == kind || kind == status) {
      found = true;
    }

    // Postconditions.
    REQUIRE(found);
  }
}

}  // namespace simon
