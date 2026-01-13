#ifndef JOYSTICK_MANAGER_HPP
#define JOYSTICK_MANAGER_HPP

#include <SDL2/SDL.h>
#include <vector>
#include <string>

// Estructura que representa el estado del receptor simulado
struct RCData {
    float channels[8] = {0.0f};
    bool buttons[8] = {false};
    uint32_t timestamp = 0;
};

class JoystickManager {
public:
    JoystickManager();
    ~JoystickManager();

    bool initialize();
    void update(); // Procesa eventos de SDL
    RCData getLatestData();
    std::string getDeviceName() const;

private:
    SDL_Joystick* m_joystick = nullptr;
    RCData m_currentData;
    
    const int16_t AXIS_MAX = 32767;
    const int16_t DEADZONE = 500;

    float normalizeAxis(int16_t raw);
};

#endif