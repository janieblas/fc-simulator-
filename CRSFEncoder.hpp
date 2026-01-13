#ifndef CRSF_ENCODER_HPP
#define CRSF_ENCODER_HPP

#include <vector>
#include <cstdint>
#include "JoystickManager.hpp"

class CRSFEncoder {
public:
    static std::vector<uint8_t> encode(const RCData& data) {
        std::vector<uint8_t> frame;
        frame.reserve(26);

        // 1. Header
        frame.push_back(0xEE); // Address: Flight Controller
        frame.push_back(24);   // Length (Type + Payload + CRC)
        frame.push_back(0x16); // Type: RC Channels Packed

        // 2. Convertir floats a valores de 11 bits (CRSF Standard)
        uint16_t channels11[16] = {0};
        for (int i = 0; i < 8; i++) {
            // Mapeo lineal: float [-1,1] a int [172, 1811]
            channels11[i] = static_cast<uint16_t>((data.channels[i] + 1.0f) * 819.5f + 172.0f);
        }

        // 3. Bit-Packing (Meter 11 bits en bytes de 8 bits)
        // Este es el algoritmo estándar de CRSF
        std::vector<uint8_t> payload(22, 0);
        
        payload[0]  = (uint8_t)(channels11[0] & 0xFF);
        payload[1]  = (uint8_t)((channels11[0] >> 8) | (channels11[1] << 3));
        payload[2]  = (uint8_t)((channels11[1] >> 5) | (channels11[2] << 6));
        payload[3]  = (uint8_t)((channels11[2] >> 2) & 0xFF);
        payload[4]  = (uint8_t)((channels11[2] >> 10) | (channels11[3] << 1));
        payload[5]  = (uint8_t)((channels11[3] >> 7) | (channels11[4] << 4));
        payload[6]  = (uint8_t)((channels11[4] >> 4) | (channels11[5] << 7));
        payload[7]  = (uint8_t)((channels11[5] >> 1) & 0xFF);
        payload[8]  = (uint8_t)((channels11[5] >> 9) | (channels11[6] << 2));
        payload[9]  = (uint8_t)((channels11[6] >> 6) | (channels11[7] << 5));
        payload[10] = (uint8_t)((channels11[7] >> 3) & 0xFF);
        // ... (se puede extender a 16 canales, aquí hacemos los primeros 8)

        frame.insert(frame.end(), payload.begin(), payload.end());

        // 4. CRC8
        frame.push_back(calculateCRC(payload, 0x16));

        return frame;
    }

private:
    static uint8_t calculateCRC(const std::vector<uint8_t>& data, uint8_t type) {
        uint8_t crc = 0;
        // El CRC en CRSF incluye el Type y el Payload
        crc = crc8(crc, type);
        for (uint8_t b : data) {
            crc = crc8(crc, b);
        }
        return crc;
    }

    static uint8_t crc8(uint8_t crc, uint8_t a) {
        crc ^= a;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc <<= 1;
        }
        return crc;
    }
};

#endif