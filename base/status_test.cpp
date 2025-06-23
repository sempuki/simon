#include "base/status.hpp"

#include <type_traits>

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
    REQUIRE(code.has_all_same_domain_bits(other));
  }

  SECTION("ShouldNotHaveAllSameDomainBitsAsOtherWhenSetDifferent") {
    // Preconditions.
    other.set_domain_bits(6u);

    // Under Test.
    code.set_domain_bits(5u);

    // Postconditions.
    REQUIRE(!code.has_all_same_domain_bits(other));
  }

  SECTION("ShouldHaveAllSameConditionBitsAsOtherWhenSetSame") {
    // Preconditions.
    other.set_domain_bits(5u);
    other.set_condition_bits(6u);

    // Under Test.
    code.set_domain_bits(5u);
    code.set_condition_bits(6u);

    // Postconditions.
    REQUIRE(code.has_all_same_condition_bits(other));
  }

  SECTION("ShouldNotHaveAllSameConditionBitsAsOtherWhenSetDifferent") {
    // Preconditions.
    other.set_domain_bits(6u);
    other.set_condition_bits(5u);

    // Under Test.
    code.set_domain_bits(5u);
    code.set_condition_bits(6u);

    // Postconditions.
    REQUIRE(!code.has_all_same_condition_bits(other));
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
    REQUIRE(code.has_all_same_incident_bits(other));
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
    REQUIRE(!code.has_all_same_incident_bits(other));
  }
}

enum class TestCondition { UP, DOWN, TOP, BOTTOM, STRANGE, CHARM };
constexpr std::size_t TEST_CONDITION_COUNT = 6;

class TestStatusDomain final
    : public EnumStatusDomain<TestCondition, TEST_CONDITION_COUNT> {
 public:
  using EnumStatusDomain<TestCondition, TEST_CONDITION_COUNT>::EnumStatusDomain;

  std::string_view message_of(
      StatusCondition condition) const noexcept override {
    message_of_condition++;
    return conditions_[condition_code(condition)].message;
  }

  std::string_view message_of(StatusCode incident) const noexcept override {
    message_of_incident++;
    return incidents_[incident_code(incident)].message;
  }

  std::source_location location_of(
      StatusCode incident) const noexcept override {
    location_of_incident++;
    return incidents_[incident_code(incident)].location;
  }

  mutable std::size_t message_of_condition = 0;
  mutable std::size_t message_of_incident = 0;
  mutable std::size_t location_of_incident = 0;
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

TEST_CASE("StatusDomain") {
  TestStatusDomain domain{42u, "test"};

  SECTION("ShouldNotGetLocationOfStatusCodeByDefault") {
    // Under Test.
    StatusCode status = domain.raise(TestCondition::UP);

    // Postconditions.
    REQUIRE(domain.location_of_incident == 0u);
  }

  SECTION("ShouldNotGetMessageOfStatusCodeByDefault") {
    // Under Test.
    StatusCode status = domain.raise(TestCondition::UP);

    // Postconditions.
    REQUIRE(domain.message_of_incident == 0u);
  }

  SECTION("ShouldGetLocationOfStatusCodeOnDemand") {
    // Preconditions.
    StatusCode status = domain.raise(TestCondition::UP);

    // Under Test.
    std::source_location location = status.location();

    // Postconditions.
    REQUIRE(domain.location_of_incident > 0u);
    REQUIRE(location.line() > 0u);
  }

  SECTION("ShouldGetMessageOfStatusCodeOnDemand") {
    // Preconditions.
    StatusCode status = domain.raise(TestCondition::UP, "bark");

    // Under Test.
    std::string_view message = status.message();

    // Postconditions.
    REQUIRE(domain.message_of_incident > 0u);
    REQUIRE(message == "bark");
  }
}

}  // namespace simon
