#include <gtest/gtest.h>
#include <optional>

#include <Arduino.h>  // Emul: set_millis64(), get_duty_writes(), get_pin_writes()

#include "../Software/src/communication/contactorcontrol/comm_contactorcontrol.h"
#include "../Software/src/datalayer/datalayer.h"
#include "../Software/src/devboard/hal/hal.h"

// Covers the secondary battery contactor in handle_contactors_battery2(): that it
// is driven through the PWM path when the user enabled economizing, that it is
// pulled in at full duty before being dropped to the hold duty, and that with the
// setting off it still falls back to a plain digitalWrite.
//
// The third battery takes the identical code path with its own pin, but the
// LilyGo HAL the host build uses has no TRIPLE_BATTERY_CONTACTORS_PIN, so there is
// nothing meaningful to drive here.
//
// Assertions are on the duty actually written to the pin: the whole point of the
// setting is the current the coil ends up holding at, which no state variable
// records.

namespace {

// Mirrors the file-scope FSM in comm_contactorcontrol.cpp. Must match it.
enum SeqState { DISCONNECTED, START_PRECHARGE, PRECHARGE, POSITIVE, PRECHARGE_OFF, COMPLETED, SHUTDOWN_REQUESTED };

// Duties and timings from comm_contactorcontrol.cpp, where they are #defines and
// so not reachable from here. Kept in step with it deliberately: if the ladder is
// retimed or the resolution changes these tests must be revisited, not silently pass.
constexpr uint32_t kFullDuty = 1023;
constexpr uint32_t kOffDuty = 0;
constexpr unsigned long kPullInMs = 1000;
constexpr unsigned long kBootMs = 100000;

}  // namespace

extern SeqState contactorStatus;

class ContactorEconomizeBattery2Test : public ::testing::Test {
 protected:
  void SetUp() override {
    datalayer = DataLayer();
    init_hal();
    set_millis64(kBootMs);
    contactorStatus = COMPLETED;        // Battery 2 only joins once the main ladder is done
    contactor_control_enabled = false;  // Isolate: only the secondary contactor is under test
    contactor_control_enabled_double_battery = true;
    contactor_control_inverted_logic = false;
    pwm_contactor_control = true;
    pwm_hold_duty = 250;
    datalayer.system.status.battery2_allowed_contactor_closing = true;
    second_contactors = esp32hal->SECOND_BATTERY_CONTACTORS_PIN();
    clear_duty_writes();
    clear_pin_writes();
  }

  void TearDown() override {
    contactorStatus = DISCONNECTED;
    contactor_control_enabled_double_battery = false;
    pwm_contactor_control = false;
    set_millis64(0);
  }

  // Advances the clock and runs one pass of the secondary contactor handler.
  static void tick_at(unsigned long ms) {
    set_millis64(ms);
    handle_contactors_battery2();
  }

  // The duty last written to the contactor pin, or nothing if it was never driven.
  static std::optional<uint32_t> last_duty(uint8_t pin) {
    std::optional<uint32_t> duty;
    for (const auto& write : get_duty_writes()) {
      if (write.pin == pin) {
        duty = write.duty;
      }
    }
    return duty;
  }

  uint8_t second_contactors = 0;
};

// The pin has to be handed to the LEDC peripheral at init, or every later
// ledcWrite() lands on an unattached pin and the contactor stays at full current.
TEST_F(ContactorEconomizeBattery2Test, InitDrivesThePinThroughPwmWhenEconomizingIsEnabled) {
  ASSERT_TRUE(init_contactors());

  EXPECT_EQ(last_duty(second_contactors), kOffDuty);
  EXPECT_TRUE(get_pin_writes().empty());  // Nothing driven by digitalWrite
}

// Closing at the hold duty would risk the coil never seating, so the contactor
// gets the same pull-in window the main pair has between closing and PRECHARGE_OFF.
TEST_F(ContactorEconomizeBattery2Test, PullsInAtFullDutyThenEconomizes) {
  tick_at(kBootMs);
  EXPECT_EQ(last_duty(second_contactors), kFullDuty);
  EXPECT_TRUE(datalayer.system.status.contactors_battery2_engaged);

  tick_at(kBootMs + kPullInMs - 1);
  EXPECT_EQ(last_duty(second_contactors), kFullDuty);

  tick_at(kBootMs + kPullInMs);
  EXPECT_EQ(last_duty(second_contactors), pwm_hold_duty);
}

// Opening has to go through the PWM path too: a digitalWrite(LOW) on a pin owned
// by the LEDC peripheral would not necessarily release the coil.
TEST_F(ContactorEconomizeBattery2Test, ReleasesAtZeroDuty) {
  tick_at(kBootMs);
  tick_at(kBootMs + kPullInMs);
  ASSERT_EQ(last_duty(second_contactors), pwm_hold_duty);

  datalayer.system.status.battery2_allowed_contactor_closing = false;
  tick_at(kBootMs + kPullInMs + 10);

  EXPECT_EQ(last_duty(second_contactors), kOffDuty);
  EXPECT_FALSE(datalayer.system.status.contactors_battery2_engaged);
}

// A contactor that dropped out and closed again is a fresh mechanical pull-in,
// so the window must restart rather than economize immediately.
TEST_F(ContactorEconomizeBattery2Test, ReclosingRestartsThePullInWindow) {
  tick_at(kBootMs);
  tick_at(kBootMs + kPullInMs);
  ASSERT_EQ(last_duty(second_contactors), pwm_hold_duty);

  datalayer.system.status.battery2_allowed_contactor_closing = false;
  tick_at(kBootMs + kPullInMs + 10);
  datalayer.system.status.battery2_allowed_contactor_closing = true;

  tick_at(kBootMs + kPullInMs + 20);
  EXPECT_EQ(last_duty(second_contactors), kFullDuty);

  tick_at(kBootMs + kPullInMs + 20 + kPullInMs);
  EXPECT_EQ(last_duty(second_contactors), pwm_hold_duty);
}

// With the setting off the contactor must keep behaving exactly as before: a
// plain GPIO output, never touched by the LEDC peripheral.
TEST_F(ContactorEconomizeBattery2Test, PlainGpioWhenEconomizingIsDisabled) {
  pwm_contactor_control = false;
  ASSERT_TRUE(init_contactors());
  clear_duty_writes();
  clear_pin_writes();

  tick_at(kBootMs);
  tick_at(kBootMs + kPullInMs);

  EXPECT_TRUE(get_duty_writes().empty());
  ASSERT_FALSE(get_pin_writes().empty());
  EXPECT_EQ(get_pin_writes().back().pin, second_contactors);
  EXPECT_EQ(get_pin_writes().back().value, HIGH);
  EXPECT_TRUE(datalayer.system.status.contactors_battery2_engaged);
}
