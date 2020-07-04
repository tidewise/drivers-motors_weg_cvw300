#include <iostream>
#include <motors_weg_cvw300/Driver.hpp>
#include <base/Angle.hpp>
#include <iomanip>
#include <fstream>

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
           << "  prepare: configures the drive and resets failure(s)\n"
           << "  cfg-dump: output all configuration variables\n"
           << "  cfg-load: set configuration from a dump file\n"
           << "  cfg-diff: compare configuration of a file with the controller's\n"
           << "  cfg-save: make in-memory configuration permanent\n"
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

    if (cmd == "status") {
        bool encoder = false;
        if (argc == 5) {
            encoder = (argv[4] == string("--encoder"));
        }
        else if (argc > 5) {
            cerr << "too many arguments to 'status'\n" << std::endl;
            usage(cerr);
            return 1;
        }

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

        cout << "\n\nState:\n";
        auto state = driver.readCurrentState();
        auto fault_state = driver.readFaultState();
        cout << "Battery Voltage: " << state.battery_voltage << " V\n"
             << "Inverter Output Voltage: "  << state.inverter_output_voltage << " V\n"
             << "Inverter Output Frequency: "
                << state.inverter_output_frequency << " Hz\n"
             << "Status: "  << statusToString(state.inverter_status) << "\n"
             << "  Current alarm: " << fault_state.current_alarm << "\n"
             << "  Current fault: " << fault_state.current_fault << "\n"
             << "Position: "
                << base::Angle::fromRad(state.motor.position).getDeg() << " deg\n"
             << "Speed: " << state.motor.speed / 2 / M_PI << "\n"
             << "Torque: " << state.motor.effort << "\n"
             << "Current: " << state.motor.raw << "\n";

        cout << "\n\nTemperatures:\n";
        auto temperatures = driver.readTemperatures();
        cout << "Air: " << temperatures.air << "\n"
             << "Mosfet: " << temperatures.mosfet << "\n";
    }
    else if (cmd == "poll") {
        bool encoder = false;
        if (argc == 5) {
            encoder = (argv[4] == string("--encoder"));
        }
        else {
            cerr << "too many arguments to 'poll'\n" << std::endl;
            usage(cerr);
            return 1;
        }

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
        if (argc != 4) {
            cerr << "too many arguments to 'cfg-dump'\n" << std::endl;
            usage(cerr);
            return 1;
        }

        modbus::Master modbus;
        modbus.openURI(uri);

        for (int i = 1; i < 1060; ++i) {
            uint16_t value;
            try {
                value = modbus.readSingleRegister(id, false, i);
            }
            catch (modbus::RequestException const &) {
                continue;
            }

            try {
                modbus.writeSingleRegister(id, i, value);
                std::cout << i << " " << value << " rw\n";
            }
            catch (modbus::RequestException const &) {
                std::cout << i << " " << value << " ro\n";
            }
        }
    }
    else if (cmd == "cfg-load") {
        if (argc != 5) {
            cerr << "too " << (argc < 5 ? "few" : "many")
                 << " arguments to 'cfg-dump'\n" << std::endl;
            usage(cerr);
            return 1;
        }

        string dumpfile = string(argv[4]);
        ifstream in(dumpfile);

        modbus::Master modbus;
        modbus.openURI(uri);

        while (true) {
            int param;
            int value;
            string mode;
            in >> param >> value >> mode;
            if (!in) {
                break;
            }

            if (mode != "rw" && mode != "ro") {
                cerr << "unexpected mode '" << mode << "' for param "
                     << param << std::endl;
                return 1;
            }

            if (mode == "rw") {
                cout << param << ": " << value << endl;
                modbus.writeSingleRegister(id, param, value);
            }
        }
    }
    else if (cmd == "cfg-diff") {
        if (argc != 5) {
            cerr << "too " << (argc < 5 ? "few" : "many")
                 << " arguments to 'cfg-dump'\n" << std::endl;
            usage(cerr);
            return 1;
        }

        string dumpfile = string(argv[4]);
        ifstream in(dumpfile);

        modbus::Master modbus;
        modbus.openURI(uri);

        while (true) {
            int param;
            int expected;
            string mode;
            in >> param >> expected >> mode;
            if (!in) {
                break;
            }

            if (mode != "rw" && mode != "ro") {
                cerr << "unexpected mode '" << mode << "' for param "
                     << param << std::endl;
                return 1;
            }

            if (mode == "rw") {
                uint16_t current = modbus.readSingleRegister(id, false, param);
                if (current != expected) {
                    cout << "Param: " << param << "\n"
                         << "  controller: " << current << "\n"
                         << "  file: " << expected << endl;
                }
            }
        }
    }
    else if (cmd == "cfg-save") {
        Driver driver(id);
        driver.openURI(uri);
        driver.configSave();
    }
    else {
        cerr << "unknown command '" << cmd << "'";
        usage(cerr);
        return 1;
    }
    return 0;
}
