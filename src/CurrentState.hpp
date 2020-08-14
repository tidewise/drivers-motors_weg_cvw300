#ifndef MOTORS_WEG_CVW300_CURRENTSTATE_HPP
#define MOTORS_WEG_CVW300_CURRENTSTATE_HPP

#include <motors_weg_cvw300/InverterStatus.hpp>
#include <base/JointState.hpp>

namespace motors_weg_cvw300 {
    /**
     * Current parameters of the motor and inverter
     */
    struct CurrentState {
        base::JointState motor;

        float battery_voltage;
        float inverter_output_voltage;
        float inverter_output_frequency;

        /** Motor overload in percent (between 0 and 1) */
        float motor_overload_ratio;

        InverterStatus inverter_status;
    };
}

#endif