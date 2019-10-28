#include <iostream>
#include <motors_weg_cvw300/Driver.hpp>

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
           << "  status: query the controller status\n"
           << "  read-holding REG LENGTH: read holding registers\n"
           << "  read-input REG LENGTH: read input registers\n"
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
        Driver driver(id);
        driver.openURI(uri);
        auto state = driver.readCurrentState();
        cout << "Battery Voltage: " << state.battery_voltage << " V\n"
             << "Inverter Output Voltage: "  << state.inverter_output_voltage << " V\n"
             << "Inverter Output Frequency: "
                << state.inverter_output_frequency << " Hz\n"
             << "Status: "  << statusToString(state.inverter_status) << "\n";
    }
    else {
        cerr << "unknown command '" << cmd << "'";
        usage(cerr);
        return 1;
    }
    return 0;
}
