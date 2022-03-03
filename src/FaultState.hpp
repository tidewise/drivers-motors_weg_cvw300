#ifndef MOTORS_WEG_CVW300_FAULTSTATE_HPP
#define MOTORS_WEG_CVW300_FAULTSTATE_HPP

#include <base/Float.hpp>
#include <base/Time.hpp>

namespace motors_weg_cvw300 {
    /**
     * Reading of the fault-related information from the registers
     *
     * The status information is directly reported by the inverter as the readings
     * "at the point of the fault". It is therefore more accurate than the direct
     * readings to analyze why the fault actually happened
     */
    struct FaultState {
        base::Time time;

        uint16_t current_fault = 0;
        uint16_t current_alarm = 0;

        /** Code of the last occuring faults
         *
         * fault_history[0] is the latest fault, which is equal to current_fault
         * if the inverter is currently in fault state
         */
        uint16_t fault_history[5];

        /** Motor current at the point of last fault */
        float current = base::unknown<float>();
        /** Battery voltage at the point of last fault */
        float battery_voltage = base::unknown<float>();
        /** Motor speed at the point of last fault */
        float speed = base::unknown<float>();
        /** Speed command at the point of last fault */
        float command = base::unknown<float>();
        /** Inverter frequency on the output (motor) side, in Hz, at the point of
         * last fault */
        float inverter_output_frequency = base::unknown<float>();
        /** Inverter voltage on the output (motor) side, at the point of last fault */
        float inverter_output_voltage = base::unknown<float>();
    };
}

#endif