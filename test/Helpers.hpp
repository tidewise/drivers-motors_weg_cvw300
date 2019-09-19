#ifndef MOTORS_WEG_CVW300_HELPERS_HPP
#define MOTORS_WEG_CVW300_HELPERS_HPP

#include <gtest/gtest.h>
#include <modbus/RTU.hpp>

template<typename Test>
struct Helpers {
    Test& test;

    Helpers(Test& test);

    void EXPECT_MODBUS_WRITE(
        int address, int registerID, uint16_t value
    );

    void EXPECT_MODBUS_READ(
        int address, bool input,
        int register_id, std::vector<uint16_t> expected_values
    );
};

template<typename Test>
Helpers<Test>::Helpers(Test& test)
    : test(test) {
}

template<typename Test>
void Helpers<Test>::EXPECT_MODBUS_WRITE(
    int address, int registerID, uint16_t value
) {
    uint8_t requestFrame[256];
    uint8_t* requestEnd = modbus::RTU::formatWriteRegister(
        requestFrame, address, registerID, value
    );

    uint8_t responseFrame[256];
    uint8_t* responseEnd = modbus::RTU::formatFrame(
        responseFrame, requestFrame[0], requestFrame[1],
        nullptr, nullptr
    );

    test.EXPECT_REPLY(std::vector<std::uint8_t>(requestFrame, requestEnd),
                      std::vector<std::uint8_t>(responseFrame, responseEnd));
}

template<typename Test>
void Helpers<Test>::EXPECT_MODBUS_READ(
    int address, bool input, int start, std::vector<uint16_t> expected_values
) {
    uint8_t requestFrame[256];
    uint8_t* requestEnd = modbus::RTU::formatReadRegisters(
        requestFrame, address, input, start, expected_values.size()
    );

    uint8_t responseFrame[256];
    uint8_t size_in_bytes = expected_values.size() * 2;
    std::vector<uint8_t> responsePayload{ size_in_bytes };
    for (auto v : expected_values) {
        responsePayload.push_back((v >> 8) & 0xFF);
        responsePayload.push_back(v & 0xFF);
    }
    uint8_t* responseEnd = modbus::RTU::formatFrame(
        responseFrame, requestFrame[0], requestFrame[1],
        &responsePayload[0], &responsePayload[0] + responsePayload.size()
    );

    test.EXPECT_REPLY(std::vector<std::uint8_t>(requestFrame, requestEnd),
                      std::vector<std::uint8_t>(responseFrame, responseEnd));
}

#endif