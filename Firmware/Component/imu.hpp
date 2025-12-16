#ifndef __IMU_HPP
#define __IMU_HPP

#include <cstdint>
#include <cstddef>

#define IMU_RX_DATA_LENGTH (11*3*2)
#define IMU_TX_DATA_LENGTH (5)

class IMU {
public:
    enum DataType {
        kAccX = 0,
        kAccY,
        kAccZ,
        kTemperature,
        kOmegaX,
        kOmegaY,
        kOmegaZ,
        kVoltage,
        kAngleX,
        kAngleY,
        kAngleZ,
        kVersion
    };

    enum Mode {
        kAcc = 0,
        kOmega,
        kAngle,
        kAuto
    };

    const uint8_t get_acc_header[5] = {0xFF, 0xAA, 0x27, 0x34, 0x00};
    const uint8_t get_omega_header[5] = {0xFF, 0xAA, 0x27, 0x37, 0x00};
    const uint8_t get_angle_header[5] = {0xFF, 0xAA, 0x27, 0x3D, 0x00};

    IMU() {};
    ~IMU() = default;

    void decode(uint8_t raw_data[IMU_RX_DATA_LENGTH]);
    float get_data(const DataType type) const { return data_[type]; }
    void get_data(float out_data[9]) const;
    // void update(uint8_t imu_tx_date[5]);
    // void set_update_mode(const Mode mode) { mode_ = mode; }

private:
    float data_[12] = {0};
    bool sumcrc(const uint8_t raw_data[11]);
    // Mode mode_ = kAuto;
};

#endif // __IMU_HPP