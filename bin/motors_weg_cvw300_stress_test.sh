#! /usr/bin/env sh

uri=$1
id=$2
speed=$3
propulsion_enable_gpio=$4
fault_reset_gpio=$5
time_between_estop_reset=$6
time_disabled=$7
total_run_time=$8

usage()
{
    echo "motors_weg_cvw300_stress_test URI ID SPEED PROPULSION_ENABLE_GPIO MOTOR_CONTROLLER_FAULT_RESET_GPIO TIME_BETWEEN_ESTOP_RESETS TOTAL_RUN_TIME"
    echo
    echo "   URI: the URI of the motor controller (i.e. serial:///dev/rs485_port_controller)"
    echo "   ID: the ID of the motor controller (i.e. 1)"
    echo "   SPEED: the speed command to be executed by the motor (i.e. 50)"
    echo "   PROPULSION_ENABLE_GPIO: the path to the propulsion enable GPIO (i.e. /dev/gpio_propulsion_enable)"
    echo "   MOTOR_CONTROLLER_FAULT_RESET_GPIO: the path to the motor controller fault reset GPIO (i.e. /dev/gpio_motor_controller_fault_reset_enable)"
    echo "   TIME_BETWEEN_ESTOP_RESETS: time elapsed before enabling estop and starting the control cycle in seconds (i.e. 60)"
    echo "   TIME_DISABLED: time that the propulsion stays disabled (i.e. 5)"
    echo "   TOTAL_RUN_TIME: total time to run the test in seconds (i.e. 3600)"
}

control_cycle() {
    deadline=$(($(date +%s) + $time_between_estop_reset))
    echo "Enabling propulsion and resetting fault state"
    echo 1 > $propulsion_enable_gpio/value
    echo 1 > $fault_reset_gpio/value
    echo 0 > $fault_reset_gpio/value

    echo "Enabling motor speed command: $speed"
    motors_weg_cvw300_ctl $uri $id speed $speed

    while [ $(date +%s) -lt $deadline ]
    do :
    done

    echo "Disabling propulsion"
    motors_weg_cvw300_ctl $uri $id speed 0
    echo 0 > $propulsion_enable_gpio/value

    disabled_deadline=$(($(date +%s) + $time_disabled))
    while [ $(date +%s) -lt $disabled_deadline ]
    do :
    done
}

if [ $# -ne 8 ];
then
    usage
    exit 1
fi

if ! command -v motors_weg_cvw300_ctl 2>&1 >/dev/null;
then
    echo "Before running this command you must source the syskit environment"
    exit 1
fi

program_deadline=$(($(date +%s) + $total_run_time))
while [ $(date +%s) -lt $program_deadline ]
do
    control_cycle
done;