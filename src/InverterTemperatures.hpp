#ifndef MOTORS_WEG_CVW300_INVERTERTEMPERATURES_HPP
#define MOTORS_WEG_CVW300_INVERTERTEMPERATURES_HPP

#include <base/Temperature.hpp>

namespace motors_weg_cvw300 {
    /** Temperatures reported by the inverter
     */
    struct InverterTemperatures {
        base::Temperature mosfet;
        base::Temperature air;
    };
}

#endif