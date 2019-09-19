#include <gtest/gtest.h>
#include <iodrivers_base/FixtureGTest.hpp>
#include <motors_weg_cvw300/Driver.hpp>
#include "Helpers.hpp"

using namespace motors_weg_cvw300;

struct DriverAddress5 : public Driver {
    DriverAddress5()
        : Driver(5) {
    }
};

struct DriverTest : public testing::Test,
                    public iodrivers_base::Fixture<DriverAddress5>,
                    public Helpers<DriverTest> {
    DriverTest()
        : Helpers<DriverTest>(*this) {
    }
};

TEST_F(DriverTest, it_reads_motor_parameters) {
    IODRIVERS_BASE_MOCK();

    EXPECT_MODBUS_READ(5, false, 401, { 10 }); // 10A nominal
    EXPECT_MODBUS_READ(5, false, 402, { 500 }); // 500rpm nominal
    EXPECT_MODBUS_READ(5, false, 404, { 1 }); // 6 kW
    driver.readMotorParameters();

    ASSERT_FLOAT_EQ(10, driver.getRatedCurrent());
    ASSERT_FLOAT_EQ(500.0 * 2.0 * M_PI / 60, driver.getRatedSpeed());
    ASSERT_NEAR(114.59, driver.getRatedTorque(), 1e-2);
}

TEST_F(DriverTest, it_prepares_the_unit_for_serial_control) {
    IODRIVERS_BASE_MOCK();

    EXPECT_MODBUS_WRITE(5, 222, 5); // REM reference == Serial
    EXPECT_MODBUS_WRITE(5, 226, 5); // REM Serial-FWD
    EXPECT_MODBUS_WRITE(5, 227, 2); // REM RUN-STOP serial
    EXPECT_MODBUS_WRITE(5, 228, 0); // JOG disable
    EXPECT_MODBUS_WRITE(5, 683, 0); // zero speed command
    EXPECT_MODBUS_WRITE(5, 682, 0x90); // configure for remote and reset fault
    driver.prepare();
}

TEST_F(DriverTest, it_enables_the_drive) {
    IODRIVERS_BASE_MOCK();

    EXPECT_MODBUS_WRITE(5, 682, 0x17); // energize
    driver.enable();
}

TEST_F(DriverTest, it_disables_the_drive) {
    IODRIVERS_BASE_MOCK();

    EXPECT_MODBUS_WRITE(5, 683, 0); // reset reference speed
    EXPECT_MODBUS_WRITE(5, 682, 0x10); // disable power stage
    driver.disable();
}

TEST_F(DriverTest, it_configures_the_serial_watchdog) {
    IODRIVERS_BASE_MOCK();

    EXPECT_MODBUS_WRITE(5, 313, 2); // disable all
    EXPECT_MODBUS_WRITE(5, 314, 154); // 15.4s
    driver.writeSerialWatchdog(base::Time::fromMilliseconds(15400),
                               configuration::DISCONNECT);
}

TEST_F(DriverTest, it_changes_the_control_type) {
    IODRIVERS_BASE_MOCK();

    EXPECT_MODBUS_WRITE(5, 202, 1); // sensorless
    driver.writeControlType(configuration::CONTROL_SENSORLESS);
}

struct JointLimitTest : public DriverTest {
    base::JointLimitRange range;
};

TEST_F(JointLimitTest, it_raises_if_min_and_max_speed_limits_are_different) {
    IODRIVERS_BASE_MOCK(); // make sure no commands are sent
    range.min.speed = -10;
    range.max.speed = 5;
    ASSERT_THROW(driver.writeJointLimits(range), std::invalid_argument);
}

TEST_F(JointLimitTest, it_raises_if_only_min_speed_is_set) {
    IODRIVERS_BASE_MOCK(); // make sure no commands are sent
    range.min.speed = 5;
    ASSERT_THROW(driver.writeJointLimits(range), std::invalid_argument);
}

TEST_F(JointLimitTest, it_raises_if_only_max_speed_is_set) {
    IODRIVERS_BASE_MOCK(); // make sure no commands are sent
    range.max.speed = 5;
    ASSERT_THROW(driver.writeJointLimits(range), std::invalid_argument);
}

TEST_F(JointLimitTest, it_raises_if_attempting_to_set_a_min_position_limit) {
    IODRIVERS_BASE_MOCK(); // make sure no commands are sent
    range.min.position = -10;
    ASSERT_THROW(driver.writeJointLimits(range), std::invalid_argument);
}

TEST_F(JointLimitTest, it_raises_if_attempting_to_set_a_max_position_limit) {
    IODRIVERS_BASE_MOCK(); // make sure no commands are sent
    range.max.position = -10;
    ASSERT_THROW(driver.writeJointLimits(range), std::invalid_argument);
}

TEST_F(JointLimitTest, it_raises_if_attempting_to_set_a_min_raw_limit) {
    IODRIVERS_BASE_MOCK(); // make sure no commands are sent
    range.min.raw = -10;
    ASSERT_THROW(driver.writeJointLimits(range), std::invalid_argument);
}

TEST_F(JointLimitTest, it_raises_if_attempting_to_set_a_max_raw_limit) {
    IODRIVERS_BASE_MOCK(); // make sure no commands are sent
    range.max.raw = -10;
    ASSERT_THROW(driver.writeJointLimits(range), std::invalid_argument);
}

TEST_F(JointLimitTest, it_writes_nothing_if_the_limits_are_unset) {
    IODRIVERS_BASE_MOCK();
    driver.writeJointLimits(range);
}

TEST_F(JointLimitTest, it_writes_the_speed_limits) {
    IODRIVERS_BASE_MOCK();
    range.min.speed = -5;
    range.max.speed = 5;

    EXPECT_MODBUS_WRITE(5, 134, 5 / (2 * M_PI) * 60);
    driver.writeJointLimits(range);
}

TEST_F(JointLimitTest, it_throws_if_a_min_effort_limit_is_set_but_the_rated_torque_is_unknown) {
    IODRIVERS_BASE_MOCK();
    range.min.effort = -5.2;
    ASSERT_THROW(driver.writeJointLimits(range), std::invalid_argument);
}

TEST_F(JointLimitTest, it_throws_if_a_max_effort_limit_is_set_but_the_rated_torque_is_unknown) {
    IODRIVERS_BASE_MOCK();
    range.max.effort = 5.2;
    ASSERT_THROW(driver.writeJointLimits(range), std::invalid_argument);
}

TEST_F(JointLimitTest, it_writes_the_min_torque) {
    IODRIVERS_BASE_MOCK();
    range.min.effort = -5.2;
    driver.setRatedTorque(10);

    EXPECT_MODBUS_WRITE(5, 170, 520); // 52% of rated torque
    driver.writeJointLimits(range);
}

TEST_F(JointLimitTest, it_writes_the_max_torque) {
    IODRIVERS_BASE_MOCK();
    range.max.effort = 5.2;
    driver.setRatedTorque(10);

    EXPECT_MODBUS_WRITE(5, 169, 520); // 52% of rated torque
    driver.writeJointLimits(range);
}

TEST_F(DriverTest, it_throws_if_attempting_to_set_a_speed_command_without_a_rated_speed) {
    IODRIVERS_BASE_MOCK(); // to make sure no message is sent
    ASSERT_THROW(driver.writeSpeedCommand(5.2), std::invalid_argument);
}

TEST_F(DriverTest, it_writes_a_positive_speed_command) {
    IODRIVERS_BASE_MOCK();
    driver.setRatedSpeed(10);

    EXPECT_MODBUS_WRITE(5, 683, 4259); // 52% of rated speed
    driver.writeSpeedCommand(5.2);
}

TEST_F(DriverTest, it_writes_a_negative_speed_command) {
    IODRIVERS_BASE_MOCK();
    driver.setRatedSpeed(10);

    EXPECT_MODBUS_WRITE(5, 683, -4259);
    driver.writeSpeedCommand(-5.2);
}

TEST_F(DriverTest, it_writes_the_ramp_configuration) {
    IODRIVERS_BASE_MOCK();
    configuration::Ramps ramps;
    ramps.acceleration_time = base::Time::fromMilliseconds(1200);
    ramps.deceleration_time = base::Time::fromMilliseconds(5100);
    ramps.type = configuration::RAMP_S_CURVE;

    EXPECT_MODBUS_WRITE(5, 100, 12);
    EXPECT_MODBUS_WRITE(5, 101, 51);
    EXPECT_MODBUS_WRITE(5, 104, 1);
    driver.writeRampConfiguration(ramps);
}

TEST_F(DriverTest, it_writes_the_vectorial_controller_settings) {
    IODRIVERS_BASE_MOCK();
    configuration::VectorialControlSettings settings;
    settings.speed_P = 1.1;
    settings.speed_I = 2.2;
    settings.speed_D = 3.3;
    settings.current_P = 4.4;
    settings.current_I = 5.5;
    settings.flux_P = 6.6;
    settings.flux_I = 7.7;
    settings.flux_nominal = 0.8;
    settings.flux_maximal = 1.2;

    EXPECT_MODBUS_WRITE(5, 161, 11);
    EXPECT_MODBUS_WRITE(5, 162, 2200);
    EXPECT_MODBUS_WRITE(5, 166, 330);
    EXPECT_MODBUS_WRITE(5, 167, 440);
    EXPECT_MODBUS_WRITE(5, 168, 5500);
    EXPECT_MODBUS_WRITE(5, 175, 66);
    EXPECT_MODBUS_WRITE(5, 176, 7700);
    EXPECT_MODBUS_WRITE(5, 178, 80);
    EXPECT_MODBUS_WRITE(5, 179, 120);
    driver.writeVectorialControlSettings(settings);
}

TEST_F(DriverTest, it_reads_the_current_state) {
    IODRIVERS_BASE_MOCK();

    driver.setRatedCurrent(100);
    driver.setRatedTorque(42);

    EXPECT_MODBUS_READ(5, false, 2,
        { 15, // speed 0002
          (uint16_t)-12, // current 0003
          421, // battery voltage 0004
          502, // frequency 0005
          4, // inverter status 0006
          128, // output voltage 0007
          0, // 0008
          243 // torque 0009
        }
    );

    CurrentState state = driver.readCurrentState();
    ASSERT_FLOAT_EQ(-15 * 2 * M_PI / 60, state.motor.speed);
    ASSERT_FLOAT_EQ(10.206, state.motor.effort);
    ASSERT_FLOAT_EQ(-1.2, state.motor.raw);
    ASSERT_FLOAT_EQ(42.1, state.battery_voltage);
    ASSERT_FLOAT_EQ(12.8, state.inverter_output_voltage);
    ASSERT_FLOAT_EQ(50.2, state.inverter_output_frequency);
    ASSERT_EQ(STATUS_AUTOTUNING, state.inverter_status);
}

TEST_F(DriverTest, it_determines_the_motor_direction_with_torque_positive_and_current_positive) {
    IODRIVERS_BASE_MOCK();
    driver.setRatedCurrent(100);
    driver.setRatedTorque(42);

    EXPECT_MODBUS_READ(5, false, 2, { 1, 1, 0, 0, 0, 0, 0, 1 });

    CurrentState state = driver.readCurrentState();
    ASSERT_TRUE(state.motor.speed > 0);
}

TEST_F(DriverTest, it_determines_the_motor_direction_with_torque_negative_and_current_negative) {
    IODRIVERS_BASE_MOCK();
    driver.setRatedCurrent(100);
    driver.setRatedTorque(42);

    EXPECT_MODBUS_READ(5, false, 2, { 1, (uint16_t)-1, 0, 0, 0, 0, 0, (uint16_t)-1 });

    CurrentState state = driver.readCurrentState();
    ASSERT_TRUE(state.motor.speed > 0);
}

TEST_F(DriverTest, it_determines_the_motor_direction_with_torque_positive_and_current_negative) {
    IODRIVERS_BASE_MOCK();
    driver.setRatedCurrent(100);
    driver.setRatedTorque(42);

    EXPECT_MODBUS_READ(5, false, 2, { 1, (uint16_t)-1, 0, 0, 0, 0, 0, 1 });

    CurrentState state = driver.readCurrentState();
    ASSERT_TRUE(state.motor.speed < 0);
}

TEST_F(DriverTest, it_determines_the_motor_direction_with_torque_negative_and_current_positive) {
    IODRIVERS_BASE_MOCK();
    driver.setRatedCurrent(100);
    driver.setRatedTorque(42);

    EXPECT_MODBUS_READ(5, false, 2, { 1, 1, 0, 0, 0, 0, 0, (uint16_t)-1 });

    CurrentState state = driver.readCurrentState();
    ASSERT_TRUE(state.motor.speed < 0);
}

TEST_F(DriverTest, it_reads_the_temperatures) {
    IODRIVERS_BASE_MOCK();

    driver.setRatedCurrent(100);
    driver.setRatedTorque(42);

    EXPECT_MODBUS_READ(
        5, false, 30,
        { 1, 2, 3, 4, (uint16_t)-5 }
    );

    InverterTemperatures temps = driver.readTemperatures();
    ASSERT_FLOAT_EQ(0.1, temps.mosfet.getCelsius());
    ASSERT_FLOAT_EQ(0.4, temps.mosfet2.getCelsius());
    ASSERT_FLOAT_EQ(-0.5, temps.air.getCelsius());
}
