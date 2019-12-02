#ifndef MOTORS_WEG_CVW300_MOTOR_RATINGS_HPP
#define MOTORS_WEG_CVW300_MOTOR_RATINGS_HPP

#include <base/Float.hpp>

namespace motors_weg_cvw300 {
    struct MotorRatings {
        /** Number of encoder ticks */
        uint16_t encoder_count = 0;

        /**
         * Scaling factor to be applied on encoder_count to have the number of
         * ticks per turn
         *
         * The driver will not report position if this parmaeter is not set.
         */
        uint16_t encoder_scale = 0;

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