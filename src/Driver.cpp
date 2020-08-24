#include <motors_weg_cvw300/Driver.hpp>
#include <base/Angle.hpp>
#include <modbus/RTU.hpp>

using namespace std;
using namespace base;
using namespace motors_weg_cvw300;

Driver::Driver(int address)
    : m_address(address) {
    // default of 7ms is too low for the weg controller at 57600
    setInterframeDelay(base::Time::fromMilliseconds(20));
}

MotorRatings Driver::readMotorRatings() {
    MotorRatings ratings = m_ratings;
    ratings.encoder_count = readSingleRegister<uint16_t>(R_ENCODER_COUNT);
    ratings.current = readSingleRegister<float>(R_MOTOR_NOMINAL_CURRENT) / 10;
    ratings.speed =
        readSingleRegister<float>(R_MOTOR_NOMINAL_SPEED) * 2 * M_PI / 60;
    int rated_power_i = readSingleRegister<uint16_t>(R_MOTOR_NOMINAL_POWER);
    if (rated_power_i == 0) {
        ratings.power = 3000;
    }
    else if (rated_power_i == 1) {
        ratings.power = 6000;
    }
    else if (rated_power_i == 2) {
        ratings.power = 12000;
    }
    else {
        throw std::invalid_argument("readMotorRatings(): unexpected rated "
                                    "power value in reply");
    }

    ratings.torque = ratings.power / ratings.speed;
    m_ratings = ratings;
    return ratings;
}

void Driver::setEncoderScale(uint16_t scale) {
    m_ratings.encoder_scale = scale;
}

MotorRatings Driver::getMotorRatings() const {
    return m_ratings;
}

void Driver::setMotorRatings(MotorRatings const& ratings) {
    m_ratings = ratings;
}

void Driver::setUseEncoderFeedback(bool use) {
    m_use_encoder_feedback = use;
}

bool Driver::getUseEncoderFeedback() const {
    return m_use_encoder_feedback;
}

void Driver::prepare() {
    writeSingleRegister<int16_t>(R_REM_REFERENCE_SELECTION,
                                 configuration::REFERENCE_SERIAL);
    writeSingleRegister<int16_t>(R_REM_DIRECTION_SELECTION,
                                 configuration::DIRECTION_SERIAL_CW);
    writeSingleRegister<int16_t>(R_REM_RUN_STOP_SELECTION,
                                 configuration::RUN_STOP_SERIAL);
    writeSingleRegister<int16_t>(R_REM_JOG_SELECTION, 0);
    writeSingleRegister<int16_t>(R_SERIAL_REFERENCE_SPEED, 0);
    writeSingleRegister<int16_t>(
        R_SERIAL_STATUS_WORD,
        configuration::SERIAL_MODE_REMOTE |
        configuration::SERIAL_RESET_FAULT
    );
}

void Driver::enable() {
    writeSingleRegister<int16_t>(
        R_SERIAL_STATUS_WORD,
        configuration::SERIAL_CONTROL_ON |
        configuration::SERIAL_GENERAL |
        configuration::SERIAL_DIRECTION_POSITIVE |
        configuration::SERIAL_MODE_REMOTE
    );
}

void Driver::disable() {
    writeSingleRegister<int16_t>(R_SERIAL_REFERENCE_SPEED, 0);
    writeSingleRegister<int16_t>(
        R_SERIAL_STATUS_WORD,
        configuration::SERIAL_MODE_REMOTE
    );
}

void Driver::writeSerialWatchdog(base::Time const& time,
                                 configuration::CommunicationErrorAction action) {
    writeSingleRegister<uint16_t>(R_SERIAL_ERROR_ACTION, action);
    writeSingleRegister<float>(R_SERIAL_WATCHDOG, time.toSeconds() * 10);
}

void Driver::configSave() {
    for (int i = 0; i < 3; ++i) {
        // Writing this causes an invalid CRC, and then re-reading it fails as
        // well. Write it three times blindly :(
        try {
            modbus::Master::writeSingleRegister(m_address, R_CONFIG_SAVE, 1);
        }
        catch (modbus::RTU::InvalidCRC const&) {
        }
        usleep(100000);
    }
}

template<typename T>
uint16_t encodeRegister(T value);

template<> uint16_t encodeRegister(uint16_t value) {
    return value;
}

template<> uint16_t encodeRegister(int16_t value) {
    return reinterpret_cast<uint16_t&>(value);
}

template<> uint16_t encodeRegister(float value) {
    return encodeRegister<int16_t>(value);
}

template<typename T>
T decodeRegister(uint16_t value);

template<> uint16_t decodeRegister(uint16_t value) {
    return value;
}

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

    if (min.hasPosition() || max.hasPosition()) {
        throw std::invalid_argument("WEG CVW300 controller does not support "
                                    "position limits");
    }
    else if (min.hasRaw() || max.hasRaw()) {
        throw std::invalid_argument("WEG CVW300 controller does not support "
                                    "raw limits");
    }
    else if (min.hasSpeed() ^ max.hasSpeed()) {
        throw std::invalid_argument("WEG CVW300 controller does not support "
                                    "having different limits for positive and "
                                    "negative movements");
    }

    if (max.hasSpeed()) {
        if (std::abs(max.speed + min.speed) > 1e-6) {
            throw std::invalid_argument("WEG CVW300 controller does not support "
                                        "having different limits for positive and "
                                        "negative movements");
        }
        writeSingleRegister<uint16_t>(R_MAX_SPEED_REFERENCE,
                                      max.speed * 60 / 2 / M_PI);
    }

    if (max.hasEffort()) {
        writeJointTorqueLimit(max.effort, R_MAX_FORWARD_TORQUE);
    }
    if (min.hasEffort()) {
        writeJointTorqueLimit(min.effort, R_MAX_REVERSE_TORQUE);
    }
}

void Driver::writeSpeedCommand(float command) {
    if (base::isUnset(m_ratings.speed)) {
        throw std::invalid_argument("writeSpeedCommand: define the rated speed before "
                                    "attempting to send a speed command");
    }
    int16_t scaled_command = 8192 * command / m_ratings.speed;
    writeSingleRegister(R_SERIAL_REFERENCE_SPEED, scaled_command);
}

void Driver::writeJointTorqueLimit(float limit, int register_id) {
    if (base::isUnknown(limit)) {
        return;
    }
    else if (base::isUnknown(m_ratings.torque)) {
        throw std::invalid_argument("need to set rated torque with setMotorRatings "
                                    "before you can set a torque limit");
    }

    writeSingleRegister<uint16_t>(register_id, std::abs(limit) / m_ratings.torque * 1000);
}

void Driver::writeRampConfiguration(configuration::Ramps const& ramps) {
    writeSingleRegister<float>(R_RAMP_ACCELERATION_TIME,
                               ramps.acceleration_time.toSeconds());
    writeSingleRegister<float>(R_RAMP_DECELERATION_TIME,
                               ramps.deceleration_time.toSeconds());
    writeSingleRegister<uint16_t>(R_RAMP_TYPE, ramps.type);
}

CurrentState Driver::readCurrentState() {
    uint16_t values[R_ENCODER_SPEED + 2];

    readRegisters(values + R_MOTOR_SPEED, m_address, false, R_MOTOR_SPEED, 8);
    readRegisters(values + R_MOTOR_OVERLOAD, m_address, false, R_MOTOR_OVERLOAD,
                  m_use_encoder_feedback ? 3 : 1);

    CurrentState state;
    if (m_use_encoder_feedback) {
        state.motor.speed = decodeRegister<float>(
            values[R_ENCODER_SPEED]) * 2 * M_PI / 60;
        if (m_ratings.encoder_scale) {
            uint32_t ticks_per_turn = m_ratings.encoder_count * m_ratings.encoder_scale;
            float position = static_cast<float>(
                values[R_ENCODER_PULSE_COUNTER] % ticks_per_turn
            ) / ticks_per_turn * 2 * M_PI;
            state.motor.position = base::Angle::normalizeRad(position);
        }
    }
    else {
        state.motor.speed = decodeRegister<float>(
            values[R_MOTOR_SPEED]) * 2 * M_PI / 60;
    }
    state.motor_overload_ratio = decodeRegister<float>(values[R_MOTOR_OVERLOAD]) / 100;
    state.motor.raw = decodeRegister<float>(
        values[R_INVERTER_OUTPUT_CURRENT]) / 10;
    state.battery_voltage = decodeRegister<float>(
        values[R_BATTERY_VOLTAGE]) / 10;
    state.inverter_output_frequency = decodeRegister<float>(
        values[R_INVERTER_OUTPUT_FREQUENCY]) / 10;
    state.inverter_status = static_cast<InverterStatus>(
        values[R_INVERTER_STATUS]
    );
    state.inverter_output_voltage = decodeRegister<float>(
        values[R_INVERTER_OUTPUT_VOLTAGE]) / 10;
    state.motor.effort  = decodeRegister<float>(
        values[R_MOTOR_TORQUE]) / 1000 * m_ratings.torque;

    if (state.motor.raw * state.motor.effort < 0) {
        state.motor.speed *= -1;
    }

    return state;
}

FaultState Driver::readFaultState() {
    uint16_t values[R_CURRENT_ALARM + 2];
    readRegisters(values + R_CURRENT_ALARM, m_address, false, R_CURRENT_ALARM, 2);

    FaultState state;
    state.time = base::Time::now();
    state.current_alarm = values[R_CURRENT_ALARM];
    state.current_fault = values[R_CURRENT_FAULT];
    return state;
}

InverterTemperatures Driver::readTemperatures() {
    InverterTemperatures temperatures;
    temperatures.mosfet = Temperature::fromCelsius(
        readSingleRegister<float>(R_TEMPERATURE_MOSFET) / 10
    );
    temperatures.air = Temperature::fromCelsius(
        readSingleRegister<float>(R_TEMPERATURE_AIR) / 10
    );
    return temperatures;
}
