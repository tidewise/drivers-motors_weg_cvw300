#include <motors_weg_cvw300/Driver.hpp>

using namespace std;
using namespace base;
using namespace motors_weg_cvw300;

Driver::Driver(int address)
    : m_address(address) {
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

void Driver::setMotorSynchronousVelocity(float velocity) {
    m_motor_synchronous_velocity = velocity;
}

void Driver::setEncoderTicksPerTurn(int ticks_per_turn) {
    m_encoder_ticks_to_rad = 1.0 / ticks_per_turn * (2 * M_PI);
}

void Driver::setMotorRatings(float torque, float current) {
    m_rated_torque = torque;
    m_rated_current = current;
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
    int16_t scaled_command = 8192 * command / m_motor_synchronous_velocity;
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


