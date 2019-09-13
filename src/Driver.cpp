#include <motors_weg_cvw300/Driver.hpp>

using namespace std;
using namespace base;
using namespace motors_weg_cvw300;

Driver::Driver(int address)
    : m_address(address) {
}

void Driver::readMotorParameters() {
    m_rated_current = readSingleRegister<float>(R_MOTOR_NOMINAL_CURRENT);
    m_rated_speed =
        readSingleRegister<float>(R_MOTOR_NOMINAL_SPEED) * 2 * M_PI / 60;
    int rated_power_i = readSingleRegister<uint16_t>(R_MOTOR_NOMINAL_POWER);
    float rated_power = 0;
    if (rated_power_i == 0)
        rated_power = 3;
    else if (rated_power_i == 1)
        rated_power = 6;
    else if (rated_power_i == 2)
        rated_power = 12;
    else {
        throw std::invalid_argument("readMotorParameters(): unexpected rated "
                                    "power value in reply");
    }

    m_rated_torque = rated_power / m_rated_speed;
}

void Driver::enable() {
    writeSingleRegister<int16_t>(R_REM_REFERENCE_SELECTION,
                                 configuration::REFERENCE_SERIAL);
    writeSingleRegister<int16_t>(R_REM_DIRECTION_SELECTION,
                                 configuration::DIRECTION_SERIAL_CCW);
    writeSingleRegister<int16_t>(R_REM_RUN_STOP_SELECTION,
                                 configuration::RUN_STOP_SERIAL);
    writeSingleRegister<int16_t>(R_REM_JOG_SELECTION, 0);
    writeSingleRegister<int16_t>(R_SERIAL_REFERENCE_SPEED, 0);
    writeSingleRegister<int16_t>(
        R_SERIAL_STATUS_WORD,
        configuration::SERIAL_CONTROL_ON |
        configuration::SERIAL_GENERAL |
        configuration::SERIAL_DIRECTION_POSITIVE |
        configuration::SERIAL_MODE_REMOTE |
        configuration::SERIAL_RESET_FAULT
    );
}

void Driver::disable() {
    writeSingleRegister<int16_t>(R_SERIAL_REFERENCE_SPEED, 0);
    writeSingleRegister<int16_t>(
        R_SERIAL_STATUS_WORD,
        configuration::SERIAL_DIRECTION_POSITIVE
    );
}

void Driver::writeSerialWatchdog(base::Time const& time,
                                 configuration::CommunicationErrorAction action) {
    writeSingleRegister<uint16_t>(R_SERIAL_ERROR_ACTION, action);
    writeSingleRegister<float>(R_SERIAL_WATCHDOG, time.toSeconds() * 10);
}

template<typename T>
uint16_t encodeRegister(T value);

template<> uint16_t encodeRegister(int16_t value) {
    return reinterpret_cast<uint16_t&>(value);
}

template<> uint16_t encodeRegister(float value) {
    return encodeRegister<int16_t>(value);
}

template<typename T>
T decodeRegister(uint16_t value);

template<> int16_t decodeRegister(uint16_t value) {
    return reinterpret_cast<int16_t&>(value);
}

template<> float decodeRegister(uint16_t value) {
    return static_cast<float>(decodeRegister<int16_t>(value));
}

template<typename T>
T Driver::readSingleRegister(int register_id) {
    uint16_t value = modbus::Master::readSingleRegister(m_address, false, register_id);
    return decodeRegister<T>(value);
}

template<typename T>
void Driver::writeSingleRegister(int register_id, T value) {
    uint16_t raw = encodeRegister<T>(value);
    modbus::Master::writeSingleRegister(m_address, register_id, raw);
}

void Driver::writeControlType(configuration::ControlType type) {
    writeSingleRegister<int16_t>(R_CONTROL_TYPE, type);
}

void Driver::writeJointLimits(base::JointLimitRange const& limits) {
    m_limits = limits;

    auto max = limits.max;
    auto min = limits.min;

    if (max.hasSpeed()) {
        if (max.speed + min.speed > 1e-6) {
            throw std::invalid_argument("WEG CVW300 controller does not support "
                                        "having different limits for positive and "
                                        "negative movements");
        }
        writeSingleRegister<uint16_t>(R_MAX_SPEED_REFERENCE,
                                      max.speed * 60 / 2 / M_PI);
    }

    writeJointTorqueLimit(max.effort, R_MAX_FORWARD_TORQUE);
    writeJointTorqueLimit(min.effort, R_MAX_REVERSE_TORQUE);
}

void Driver::writeSpeedCommand(float command) {
    int16_t scaled_command = 8192 * command / m_rated_speed;
    writeSingleRegister(R_SERIAL_REFERENCE_SPEED, scaled_command);
}

void Driver::writeJointTorqueLimit(float limit, int register_id) {
    if (base::isUnknown(limit)) {
        return;
    }
    else if (base::isUnknown(m_rated_torque)) {
        throw std::invalid_argument("need to set rated torque with setMotorRatings "
                                    "before you can set a torque limit");
    }

    writeSingleRegister<uint16_t>(register_id, limit / m_rated_torque * 1000);
}

void Driver::writeRampConfiguration(configuration::Ramps const& ramps) {
    writeSingleRegister<float>(R_RAMP_ACCELERATION_TIME,
                               ramps.acceleration_time.toSeconds() * 10);
    writeSingleRegister<float>(R_RAMP_DECELERATION_TIME,
                               ramps.deceleration_time.toSeconds() * 10);
    writeSingleRegister<uint16_t>(R_RAMP_TYPE, ramps.type);
}

void Driver::writeVectorialControlSettings(
    configuration::VectorialControlSettings const& settings
) {
    writeSingleRegister<float>(R_GAIN_SPEED_P, settings.speed_P * 10);
    writeSingleRegister<float>(R_GAIN_SPEED_I, settings.speed_I * 1000);
    writeSingleRegister<float>(R_GAIN_SPEED_D, settings.speed_D * 100);
    writeSingleRegister<float>(R_GAIN_CURRENT_P, settings.current_P * 100);
    writeSingleRegister<float>(R_GAIN_CURRENT_I, settings.current_I * 1000);
    writeSingleRegister<float>(R_GAIN_FLUX_P, settings.flux_P * 10);
    writeSingleRegister<float>(R_GAIN_FLUX_I, settings.flux_I * 1000);
    writeSingleRegister<float>(R_FLUX_NOMINAL, settings.flux_nominal * 100);
    writeSingleRegister<float>(R_FLUX_MAXIMAL, settings.flux_maximal * 100);
}

CurrentState Driver::readCurrentState() {
    uint16_t values[32];
    readRegisters(values, m_address,
                  false, R_MOTOR_SPEED, R_MOTOR_TORQUE - R_MOTOR_SPEED + 1);

    CurrentState state;
    state.motor.speed = decodeRegister<float>(
        values[R_MOTOR_SPEED - R_MOTOR_SPEED]) * 2 * M_PI / 60;
    state.motor.raw = decodeRegister<float>(
        values[R_INVERTER_OUTPUT_CURRENT - R_MOTOR_SPEED]) / 10;
    state.battery_voltage = decodeRegister<float>(
        values[R_BATTERY_VOLTAGE - R_MOTOR_SPEED]) / 10;
    state.inverter_output_frequency = decodeRegister<float>(
        values[R_INVERTER_OUTPUT_FREQUENCY - R_MOTOR_SPEED]);
    state.inverter_status = static_cast<InverterStatus>(
        values[R_INVERTER_STATUS - R_MOTOR_SPEED]
    );
    state.inverter_output_voltage = decodeRegister<float>(
        values[R_INVERTER_OUTPUT_VOLTAGE - R_MOTOR_SPEED]) / 10;
    state.motor.effort  = decodeRegister<float>(
        values[R_MOTOR_TORQUE - R_MOTOR_SPEED]) / 10000 * m_rated_torque;

    if (state.motor.raw * state.motor.effort < 0) {
        state.motor.speed *= -1;
    }

    return state;
}

InverterTemperatures Driver::readTemperatures() {
    InverterTemperatures temperatures;
    uint16_t raw[5];
    readRegisters(raw, m_address, false, R_TEMPERATURE_MOSFET, 5);
    temperatures.mosfet  = Temperature::fromCelsius(
        decodeRegister<float>(raw[0]) / 10
    );
    temperatures.mosfet2 = Temperature::fromCelsius(
        decodeRegister<float>(raw[3]) / 10
    );
    temperatures.air     = Temperature::fromCelsius(
        decodeRegister<float>(raw[4]) / 10
    );
    return temperatures;
}


