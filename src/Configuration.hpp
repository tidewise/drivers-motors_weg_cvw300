#ifndef MOTORS_WEG_CVW300_CONFIGURATION_HPP
#define MOTORS_WEG_CVW300_CONFIGURATION_HPP

namespace motors_weg_cvw300 {
    namespace configuration {
        enum ControlType {
            CONTROL_VECTORIAL = 0,
            CONTROL_SENSORLESS = 1,
            CONTROL_ENCODER = 2
        };

        enum CommunicationErrorAction {
            /** Do nothing */
            INACTIVE = 0,
            /** Stop with configured ramp */
            STOP_WITH_RAMP = 1,
            /** Stop all control, let the motor continue on inertia */
            DISCONNECT = 2,
            /** Switch to LOCAL control mode */
            SWITCH_TO_LOCAL = 3,
            /** Force a fault */
            FAULT = 5
        };

        enum ReferenceSource {
            REFERENCE_HMI = 0,
            REFERENCE_SERIAL = 5
        };

        enum DirectionSource {
            /** Direction is selected by serial, "forward" is clockwise */
            DIRECTION_SERIAL_CW = 5,
            /** Direction is selected by serial, "forward" is counter-clockwise */
            DIRECTION_SERIAL_CCW = 6
        };

        enum RunStopSource {
            RUN_STOP_INACTIVE = 0,
            RUN_STOP_SERIAL = 2
        };

        enum StopMode {
            /** Stop by applying ramps */
            STOP_MODE_RAMP,
            /** Let the motor stop by itself */
            STOP_MODE_COAST,
            /** Stop immediately (zero ramp) */
            STOP_MODE_FAST,
            /** Stop by applying ramps, reset torque current reference */
            STOP_MODE_RAMP_WITH_ZERO_IQ,
            /** Stop immediately, reset torque current reference */
            STOP_MODE_FAST_WITH_ZERO_IQ
        };

        enum SerialStatusBits {
            /** Set this bit to enable motor movement
             *
             * If unset, the motor will stop, appying a ramp
             */
            SERIAL_CONTROL_ON = 1,
            /** Enable general operations
             *
             * If unset, the controller will stop
             */
            SERIAL_GENERAL = 2,
            /** Turn in the positive direction if set, negative otherwise */
            SERIAL_DIRECTION_POSITIVE = 4,
            /** Set the mode as remote or local */
            SERIAL_MODE_REMOTE = 16,
            /** Reset existing faults */
            SERIAL_RESET_FAULT = 128
        };
    }
}

#endif