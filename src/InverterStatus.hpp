#ifndef MOTORS_WEG_CVW300_INVERTERSTATUS_HPP
#define MOTORS_WEG_CVW300_INVERTERSTATUS_HPP

namespace motors_weg_cvw300 {
    enum InverterStatus {
        STATUS_READY = 0,
        STATUS_RUN = 1,
        STATUS_UNDERVOLTAGE = 2,
        STATUS_FAULT = 3,
        STATUS_AUTOTUNING = 4,
        STATUS_CONFIGURATION = 5,
        STATUS_DC_BRAKING = 6,
        STATUS_UNKNOWN = 7
    };
}

#endif