#include "JoystickManager.hpp"
#include "CRSFEncoder.hpp"
#include "UDPTransport.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <vector>

int main() {
    JoystickManager rcInput;
    // Iniciamos el transporte hacia localhost (mismo PC) puerto 5005
    UDPTransport messenger("127.0.0.1", 5005);

    if (!rcInput.initialize()) {
        std::cerr << "Error: No se pudo conectar el RadioMaster Pocket." << std::endl;
        return 1;
    }

    std::cout << "--- SIMULADOR DE RECEPTOR ELRS ---" << std::endl;
    std::cout << "Transmitiendo CRSF sobre UDP (Puerto 5005)..." << std::endl;

    while (true) {
        // 1. Capturar palancas [-1.0, 1.0]
        rcInput.update();
        RCData data = rcInput.getLatestData();

        // 2. Empaquetar en protocolo CRSF (26 bytes binarios)
        std::vector<uint8_t> crsfFrame = CRSFEncoder::encode(data);

        // 3. ENVIAR AL "FLIGHT SYSTEM" (Python)
        messenger.sendFrame(crsfFrame);

        // 4. Feedback visual en consola
        std::cout << "\r"
                  << "[TX] R:" << std::setw(5) << std::fixed << std::setprecision(2) << data.channels[0]
                  << " P:" << std::setw(5) << data.channels[1]
                  << " T:" << std::setw(5) << data.channels[2]
                  << " Y:" << std::setw(5) << data.channels[3]
                  << " | Pkt: 0x" << std::hex << (int)crsfFrame[0] << "..." << std::dec
                  << "    " << std::flush;

        // Frecuencia de 50Hz (20ms) para simular un enlace ELRS estándar
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
    }

    return 0;
}