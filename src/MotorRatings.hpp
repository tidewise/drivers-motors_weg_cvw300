#ifndef MOTORS_WEG_CVW300_MOTOR_RATINGS_HPP
#define MOTORS_WEG_CVW300_MOTOR_RATINGS_HPP

#include <base/Float.hpp>

namespace motors_weg_cvw300 {
    struct MotorRatings {
        /** Motor nominal current */
        float current = base::unknown<float>();

        /** Motor nominal speed */
        float speed = base::unknown<float>();

        /** Motor nominal torque */
        float torque = base::unknown<float>();

        /** Motor power in W */
        float power = base::unknown<float>();
    };
}

#endif