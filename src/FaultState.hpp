#ifndef MOTORS_WEG_CVW300_FAULTSTATE_HPP
#define MOTORS_WEG_CVW300_FAULTSTATE_HPP

#include <base/Time.hpp>

namespace motors_weg_cvw300 {
    /**
     * Reading of the fault-related information from the registers
     */
    struct FaultState {
        base::Time time;

        uint16_t current_fault = 0;
        uint16_t current_alarm = 0;
    };
}

#endif