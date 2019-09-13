#ifndef MOTORS_WEG_CVW300_DRIVER_HPP
#define MOTORS_WEG_CVW300_DRIVER_HPP

#include <base/Float.hpp>
#include <base/JointLimitRange.hpp>
#include <modbus/Master.hpp>
#include <motors_weg_cvw300/Configuration.hpp>
#include <motors_weg_cvw300/InverterTemperatures.hpp>
#include <motors_weg_cvw300/CurrentState.hpp>

namespace motors_weg_cvw300 {
    /**
     * Driver for the WEG CVW300 controller
     */
    class Driver : public modbus::Master {
        int m_address;
        float m_rated_speed = base::unknown<float>();
        float m_rated_torque = base::unknown<float>();
        float m_rated_current = base::unknown<float>();

        base::JointLimitRange m_limits;

        enum Registers {
            R_MOTOR_SPEED = 2,
            R_INVERTER_OUTPUT_CURRENT = 3,
            R_BATTERY_VOLTAGE = 4,
            R_INVERTER_OUTPUT_FREQUENCY = 5,
            R_INVERTER_STATUS = 6,
            R_INVERTER_OUTPUT_VOLTAGE = 7,
            R_MOTOR_TORQUE = 9,
            R_TEMPERATURE_MOSFET = 30,
            R_ENCODER_VELOCITY = 38,
            R_ENCODER_PULSE_COUNTER = 39,
            R_MAX_SPEED_REFERENCE = 134,
            R_MAX_SPEED_ABSOLUTE = 134,
            R_CONTROL_TYPE = 202,
            R_MAX_FORWARD_TORQUE = 169,
            R_MAX_REVERSE_TORQUE = 170,
            R_SERIAL_ERROR_ACTION = 313,
            R_SERIAL_WATCHDOG = 314,
            R_REM_REFERENCE_SELECTION = 222,
            R_REM_DIRECTION_SELECTION = 226,
            R_REM_RUN_STOP_SELECTION = 227,
            R_REM_JOG_SELECTION = 228,

            R_SERIAL_STATUS_WORD = 682,
            R_SERIAL_REFERENCE_SPEED = 683
        };

        template<typename T>
        T readSingleRegister(int register_id);

        template<typename T>
        void writeSingleRegister(int register_id, T value);

        void writeJointTorqueLimit(float limit, int register_id);

    public:
        Driver(int address);

        /** Read needed motor parameters from the controller */
        void readMotorParameters();

        /** Enable the motor control, and give control to the serial interface */
        void enable();

        /**
         * Disable the motor control, and give control away from to the serial
         * interface
         */
        void disable();

        /** Set the watchdog and the action to perform when it triggers */
        void writeSerialWatchdog(base::Time const& time,
                                 configuration::CommunicationErrorAction action =
                                     configuration::STOP_WITH_RAMP);

        /** Set the type of control */
        void writeControlType(configuration::ControlType type);

        /** Configure the limits */
        void writeJointLimits(base::JointLimitRange const& limits);

        /** Send a speed command */
        void writeSpeedCommand(float command);

        CurrentState readCurrentState();

        InverterTemperatures readTemperatures();
    };
}

#endif