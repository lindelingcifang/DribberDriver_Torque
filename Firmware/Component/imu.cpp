#include "imu.hpp"

const float imu_k[3] = {16.0f * 9.8f / 32768.0f, 2000.0f / 32768.0f, 180.0f / 32768.0f};
const float lsm56ds3_gyro_k = 245.0f / 32768.0f;

void IMU::decode(uint8_t raw_data[IMU_RX_DATA_LENGTH])
{
    for (uint8_t j = 0; j < 33; j++)
    {
        if (raw_data[j] != 0x55)
            continue;

        for (uint8_t i = 0; i < 3; i++)
        {
            if (raw_data[j + 0 + i * 11] == 0x55 && raw_data[j + 1 + i * 11] == (0x51 + i))
            {

                if (sumcrc(&(raw_data[j + 0 + i * 11])))
                {
                    data_[0 + i * 4] = (short)(((short)raw_data[j + 3 + i * 11] << 8) | raw_data[j + 2 + i * 11]) * imu_k[i];
                    data_[1 + i * 4] = (short)(((short)raw_data[j + 5 + i * 11] << 8) | raw_data[j + 4 + i * 11]) * imu_k[i];
                    data_[2 + i * 4] = (short)(((short)raw_data[j + 7 + i * 11] << 8) | raw_data[j + 6 + i * 11]) * imu_k[i];
                    // data_[kVoltage] = (short)(((short)raw_data[9] << 8) | raw_data[8]) / 100.0;
                }
            }
        }
    }
}

bool IMU::sumcrc(const uint8_t raw_data[11])
{
    uint16_t sum = 0x0;
    for (size_t i = 0; i < 10; i++)
    {
        sum += raw_data[i];
    }
    uint8_t crc = sum & 0xFF;
    return (crc == raw_data[10]);
}

void IMU::get_data(float out_data[9]) const
{
    out_data[0] = data_[kAccX];
    out_data[1] = data_[kAccY];
    out_data[2] = data_[kAccZ];
    // correct mapping: omega X/Y/Z
    out_data[3] = data_[kOmegaX];
    out_data[4] = data_[kOmegaY];
    out_data[5] = data_[kOmegaZ];
    
    out_data[6] = data_[kAngleX];
    out_data[7] = data_[kAngleY];
    out_data[8] = data_[kAngleZ];
}

void IMU::lsm6ds3_init() {
    HAL_Delay(20);
    auto write_reg = [this](uint8_t addr, uint8_t val) {
        uint16_t tx = (static_cast<uint16_t>(addr & 0x7F) << 8) | val; // write: bit7=0
        (void)HAL_SPI_Transmit(hspi_, (uint8_t*)&tx, 1, HAL_MAX_DELAY);
    };

    write_reg(0x19, 0x38); // CTRL10_C: Gyro X/Y/Z enable
    write_reg(0x11, 0x60); // CTRL2_G: 416Hz, high-performance, 245 dps
    write_reg(0x0D, 0x02); // INT1_CTRL: Gyro data-ready on INT1
    write_reg(0x12, 0x04); // CTRL3_C: IF_INC=1 (auto address increment)
}

void IMU::process_spi_data() {
    data_[kOmegaX] = (short)(((short)(lsm6ds3_rx_data[1] >> 8) << 8) | (lsm6ds3_rx_data[0] & 0xFF)) * lsm56ds3_gyro_k;
    data_[kOmegaY] = (short)(((short)(lsm6ds3_rx_data[2] >> 8) << 8) | (lsm6ds3_rx_data[1] & 0xFF)) * lsm56ds3_gyro_k;
    data_[kOmegaZ] = (short)(((short)(lsm6ds3_rx_data[3] >> 8) << 8) | (lsm6ds3_rx_data[2] & 0xFF)) * lsm56ds3_gyro_k;
}