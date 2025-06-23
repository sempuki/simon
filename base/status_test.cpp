#include "base/status.hpp"

#include <sstream>

#include "base/testing.hpp"

namespace simon {

TEST_CASE("BitCode") {
  impl::BitCode code;

  SECTION("ShouldDefaultToZero") {
    // Postconditions.
    REQUIRE(code.domain_bits() == 0u);
    REQUIRE(code.condition_bits() == 0u);
    REQUIRE(code.incident_bits() == 0u);
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

  impl::BitCode other;

  SECTION("ShouldHaveAllSameDomainBitsAsOtherWhenSetSame") {
    // Preconditions.
    other.set_domain_bits(5u);

    // Under Test.
    code.set_domain_bits(5u);

    // Postconditions.
    REQUIRE(code.has_all_same_domain_bits_as(other));
  }

  SECTION("ShouldNotHaveAllSameDomainBitsAsOtherWhenSetDifferent") {
    // Preconditions.
    other.set_domain_bits(6u);

    // Under Test.
    code.set_domain_bits(5u);

    // Postconditions.
    REQUIRE(!code.has_all_same_domain_bits_as(other));
  }

  SECTION("ShouldHaveAllSameConditionBitsAsOtherWhenSetSame") {
    // Preconditions.
    other.set_domain_bits(5u);
    other.set_condition_bits(6u);

    // Under Test.
    code.set_domain_bits(5u);
    code.set_condition_bits(6u);

    // Postconditions.
    REQUIRE(code.has_all_same_condition_bits_as(other));
  }

  SECTION("ShouldNotHaveAllSameConditionBitsAsOtherWhenSetDifferent") {
    // Preconditions.
    other.set_domain_bits(6u);
    other.set_condition_bits(5u);

    // Under Test.
    code.set_domain_bits(5u);
    code.set_condition_bits(6u);

    // Postconditions.
    REQUIRE(!code.has_all_same_condition_bits_as(other));
  }

  SECTION("ShouldHaveAllSameIncidentBitsAsOtherWhenSetSame") {
    // Preconditions.
    other.set_domain_bits(5u);
    other.set_condition_bits(6u);
    other.set_incident_bits(7u);

    // Under Test.
    code.set_domain_bits(5u);
    code.set_condition_bits(6u);
    code.set_incident_bits(7u);

    // Postconditions.
    REQUIRE(code.has_all_same_indicent_bits_as(other));
  }

  SECTION("ShouldNotHaveAllSameIncidentBitsAsOtherWhenSetDifferent") {
    // Preconditions.
    other.set_domain_bits(6u);
    other.set_condition_bits(5u);
    other.set_incident_bits(7u);

    // Under Test.
    code.set_domain_bits(5u);
    code.set_condition_bits(6u);
    code.set_incident_bits(7u);

    // Postconditions.
    REQUIRE(!code.has_all_same_indicent_bits_as(other));
  }
}

enum class TestCondition { UP, DOWN, TOP, BOTTOM, STRANGE, CHARM, COUNT };
constexpr std::size_t TEST_CONDITION_COUNT =
    static_cast<std::size_t>(TestCondition::COUNT);

class TestStatusDomain final
    : public EnumStatusDomain<TestCondition, TEST_CONDITION_COUNT> {
 public:
  using EnumStatusDomain<TestCondition, TEST_CONDITION_COUNT>::EnumStatusDomain;

  std::string_view message_of(
      StatusCondition condition) const noexcept override {
    message_of_condition++;
    return conditions_[condition_code_of(condition)].message;
  }

  std::string_view message_of(StatusCode incident) const noexcept override {
    message_of_incident++;
    return incidents_[incident_code_of(incident)].message;
  }

  std::source_location location_of(
      StatusCode incident) const noexcept override {
    location_of_incident++;
    return incidents_[incident_code_of(incident)].location;
  }

  bool has_equivalent_condition_of(
      StatusCode incident, StatusCondition condition) const noexcept override {
    equivalent_condition_of++;
    return false;
  }

  mutable std::size_t message_of_condition = 0;
  mutable std::size_t message_of_incident = 0;
  mutable std::size_t location_of_incident = 0;
  mutable std::size_t equivalent_condition_of = 0;
};

template <>
const std::array<impl::ConditionEntry, TEST_CONDITION_COUNT>
    EnumStatusDomain<TestCondition, TEST_CONDITION_COUNT>::conditions_ = {
        impl::ConditionEntry{"UP"},       //
        impl::ConditionEntry{"DOWN"},     //
        impl::ConditionEntry{"TOP"},      //
        impl::ConditionEntry{"BOTTOM"},   //
        impl::ConditionEntry{"STRANGE"},  //
        impl::ConditionEntry{"CHARM"},    //
};

TestStatusDomain domain{42u, "test"};
std::string message0 = "mark";
std::string message1 = "bark";

TEST_CASE("StatusDomain") {
  SECTION("ShouldNotGetLocationOfStatusCodeByDefault") {
    // Under Test.
    StatusCode status = domain.raise_incident(TestCondition::UP);

    // Postconditions.
    REQUIRE(domain.location_of_incident == 0u);
  }

  SECTION("ShouldNotGetMessageOfStatusCodeByDefault") {
    // Under Test.
    StatusCode status = domain.raise_incident(TestCondition::UP);

    // Postconditions.
    REQUIRE(domain.message_of_incident == 0u);
  }

  SECTION("ShouldGetLocationOfStatusCodeOnDemand") {
    // Preconditions.
    StatusCode status = domain.raise_incident(TestCondition::UP);

    // Under Test.
    std::source_location location = status.location();

    // Postconditions.
    REQUIRE(domain.location_of_incident > 0u);
    REQUIRE(location.line() > 0u);
  }

  SECTION("ShouldGetMessageOfStatusCodeOnDemand") {
    // Preconditions.
    StatusCode status = domain.raise_incident(TestCondition::UP, message0);

    // Under Test.
    std::string_view message = status.message();

    // Postconditions.
    REQUIRE(domain.message_of_incident > 0u);
    REQUIRE(message.size() > 0u);
  }
}

TEST_CASE("StatusCode") {
  std::source_location incident_location = std::source_location::current();
  StatusCode status = domain.raise_incident(TestCondition::UP, message0);

  SECTION("ShouldHaveSameLocationAsRaisedIncident") {
    // Under Test.
    std::source_location location = status.location();

    // Postconditions.
    REQUIRE(location.line() - 1 == incident_location.line());
  }

  SECTION("ShouldHaveSameMessageAsRaisedIncident") {
    // Under Test.
    std::string_view status_message = status.message();

    // Postconditions.
    REQUIRE(status_message == message0);
  }

  SECTION("ShouldBeEqualForSameIncident") {
    // Under Test.
    StatusCode copy = status;

    // Postconditions.
    REQUIRE(copy == status);
  }

  SECTION("ShouldNotBeEqualForDifferentIncidentOfSameCondition") {
    // Under Test.
    StatusCode other = domain.raise_incident(TestCondition::UP, message0);

    // Postconditions.
    REQUIRE(other != status);
  }

  SECTION("ShouldNotBeEqualForDifferentIncidentOfDifferentCondition") {
    // Under Test.
    StatusCode other = domain.raise_incident(TestCondition::DOWN, message0);

    // Postconditions.
    REQUIRE(other != status);
  }

  SECTION("ShouldBeOutStreamable") {
    // Preconditions.
    std::stringstream ss;

    // Under Test.
    ss << status;

    // Postconditions.
    REQUIRE(ss.str().size() > 0u);
    REQUIRE(ss.str() == message0);
  }
}

TEST_CASE("StatusCondition") {
  SECTION("ShouldHaveMessageDescribingCondition") {
    // Preconditions.
    StatusCondition up = domain.watch_condition(TestCondition::UP);

    // Under Test.
    std::string_view message = up.message();

    // Postconditions.
    REQUIRE(message == "UP");
  }

  StatusCode status = domain.raise_incident(TestCondition::UP);

  SECTION("ShouldBeTrueWhenComparedWithIncidentOfSameCondition") {
    // Under Test.
    StatusCondition up = domain.watch_condition(TestCondition::UP);

    // Postconditions.
    REQUIRE(status == up);
  }

  SECTION("ShouldBeFalseWhenComparedWithIncidentOfDifferentCondition") {
    // Under Test.
    StatusCondition down = domain.watch_condition(TestCondition::DOWN);

    // Postconditions.
    REQUIRE(status != down);
  }
}

}  // namespace simon
