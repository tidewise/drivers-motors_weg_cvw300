#! /usr/bin/env sh

uri=$1
id=$2
speed=$3
propulsion_enable_gpio=$4
time_between_estop_reset=$5
time_disabled=$6
total_run_time=$7

usage()
{
    echo "motors_weg_cvw300_stress_test URI ID SPEED PROPULSION_ENABLE_GPIO TIME_BETWEEN_ESTOP_RESETS TOTAL_RUN_TIME"
    echo
    echo "   URI: the URI of the motor controller (i.e. serial:///dev/rs485_port_controller)"
    echo "   ID: the ID of the motor controller (i.e. 1)"
    echo "   SPEED: the speed command to be executed by the motor (i.e. 50)"
    echo "   PROPULSION_ENABLE_GPIO: the path to the propulsion enable GPIO (i.e. /dev/gpio_propulsion_enable)"
    echo "   TIME_BETWEEN_ESTOP_RESETS: time elapsed before enabling estop and starting the control cycle in seconds (i.e. 60)"
    echo "   TIME_DISABLED: time that the propulsion stays disabled (i.e. 5)"
    echo "   TOTAL_RUN_TIME: total time to run the test in seconds (i.e. 3600)"
}

control_cycle() {
    deadline=$(($(date +%s) + $time_between_estop_reset))
    echo
    echo "Enabling propulsion"
    echo 1 > $propulsion_enable_gpio/value

    fault_before_reset=$(motors_weg_cvw300_ctl $uri $id fault-state | grep "Current Fault: " | cut -d' ' -f3)
    echo "Trying to reset fault: $fault_before_reset"
    motors_weg_cvw300_ctl $uri $id prepare
    fault_after_reset=$(motors_weg_cvw300_ctl $uri $id fault-state | grep "Current Fault: " | cut -d' ' -f3)

    if [ $fault_after_reset -ne 0 ]
    then
       echo "Resetting from fault $fault_before_reset failed as a fault state remained: $fault_after_reset"
       exit 1
    fi

    echo "Enabling motor speed command: $speed"
    while [ $(date +%s) -lt $deadline ]
    do
       motors_weg_cvw300_ctl $uri $id speed $speed 0.5
    done

    echo "Disabling propulsion"
    motors_weg_cvw300_ctl $uri $id speed 0 0.5
    echo 0 > $propulsion_enable_gpio/value
    fault_after_disable=$(motors_weg_cvw300_ctl $uri $id fault-state | grep "Current Fault: " | cut -d' ' -f3)
    echo "Fault after disabling propulsion: $fault_after_disable"

    disabled_deadline=$(($(date +%s) + $time_disabled))
    while [ $(date +%s) -lt $disabled_deadline ]
    do :
    done
}

if [ $# -ne 7 ];
then
    usage
    exit 1
fi

if ! command -v motors_weg_cvw300_ctl 2>&1 >/dev/null;
then
    echo "Before running this command you must source the syskit environment"
    exit 1
fi

motors_weg_cvw300_ctl $uri $id setup

program_deadline=$(($(date +%s) + $total_run_time))
while [ $(date +%s) -lt $program_deadline ]
do
    control_cycle
done;
