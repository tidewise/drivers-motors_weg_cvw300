#include <iostream>
#include <motors_weg_cvw300/Driver.hpp>
#include <base/Angle.hpp>
#include <iomanip>

using namespace std;
using namespace motors_weg_cvw300;

#define STATUS_CASE(name) case STATUS_##name: return #name;
std::string statusToString(InverterStatus status) {
    switch (status) {
        STATUS_CASE(READY)
        STATUS_CASE(RUN)
        STATUS_CASE(UNDERVOLTAGE)
        STATUS_CASE(FAULT)
        STATUS_CASE(AUTOTUNING)
        STATUS_CASE(CONFIGURATION)
        STATUS_CASE(DC_BRAKING)
        STATUS_CASE(UNKNOWN)
    }
    return "UNKNOWN";
}

void usage(ostream& stream) {
    stream << "usage: motors_weg_cvw300_ctl URI ID CMD\n"
           << "Factory defaults: 19200, ID=1\n"
           << "\n"
           << "Available Commands\n"
           << "  status [--encoder]: query the controller status\n"
           << "  poll [--encoder]: repeatedly display the motor state\n"
           << endl;
}

int main(int argc, char** argv)
{
    if (argc < 4) {
        bool error = argc == 1 ? 0 : 1;
        usage(error ? cerr : cout);
        return error;
    }

    string uri = argv[1];
    int id = std::atoi(argv[2]);
    string cmd = argv[3];
    bool encoder = false;
    if (argc > 4) {
        encoder = (argv[4] == string("--encoder"));
    }

    if (cmd == "status") {
        Driver driver(id);
        driver.openURI(uri);
        driver.setUseEncoderFeedback(encoder);
        auto ratings = driver.readMotorRatings();
        cout << "Ratings:\n"
             << "Power: " << ratings.power << " W\n"
             << "Current: " << ratings.current << " A\n"
             << "Speed: " << ratings.speed / 2 / M_PI * 180 << " deg/s "
                << "(" << ratings.speed / 2 / M_PI * 60 << " rpm)\n"
             << "Torque: " << ratings.torque << " N.m\n"
             << "Encoder Count: " << ratings.encoder_count << " ticks p. turn\n";
        auto state = driver.readCurrentState();
        cout << "Battery Voltage: " << state.battery_voltage << " V\n"
             << "Inverter Output Voltage: "  << state.inverter_output_voltage << " V\n"
             << "Inverter Output Frequency: "
                << state.inverter_output_frequency << " Hz\n"
             << "Status: "  << statusToString(state.inverter_status) << "\n"
             << "Position: "
                << base::Angle::fromRad(state.motor.position).getDeg() << " deg\n"
             << "Speed: " << state.motor.speed / 2 / M_PI << "\n"
             << "Torque: " << state.motor.effort << "\n"
             << "Current: " << state.motor.raw << "\n";
    }
    else if (cmd == "poll") {
        Driver driver(id);
        driver.openURI(uri);
        driver.setUseEncoderFeedback(encoder);
        driver.readMotorRatings();
        std::cout << "Status; Bat (V); Output (V); Output (Hz); "
                     "Position (deg); Speed (rpm); orque (N.m); Current (A)\n";

        while (true) {
            auto state = driver.readCurrentState();
            cout << setw(5) << setprecision(1) << fixed;
            cout << setw(10) << statusToString(state.inverter_status) << " "
                 << setw(5) << state.battery_voltage << " "
                 << setw(5) << state.inverter_output_voltage << " "
                 << setw(5) << state.inverter_output_frequency << " "
                 << setw(6) << base::Angle::fromRad(state.motor.position).getDeg() << " "
                 << setw(7) << state.motor.speed / 2 / M_PI * 60 << " "
                 << setw(6) << state.motor.effort << " "
                 << setw(6) << state.motor.raw << "\n";
        }
    }
    else if (cmd == "cfg-dump") {
        modbus::Master modbus;
        modbus.openURI(uri);

        for (int i = 0; i < 1100; ++i) {
            try {
                uint16_t value = modbus.readSingleRegister(id, false, i);
                std::cout << i << " " << value << "\n";
            }
            catch (modbus::RequestException const &) {
            }
        }
    }
    else {
        cerr << "unknown command '" << cmd << "'";
        usage(cerr);
        return 1;
    }
    return 0;
}
