#include "JoystickManager.hpp"
#include <algorithm>

JoystickManager::JoystickManager() {}

JoystickManager::~JoystickManager() {
    if (m_joystick) SDL_JoystickClose(m_joystick);
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
}

bool JoystickManager::initialize() {
    if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) < 0) return false;
    if (SDL_NumJoysticks() < 1) return false;

    m_joystick = SDL_JoystickOpen(0);
    return m_joystick != nullptr;
}

float JoystickManager::normalizeAxis(int16_t raw) {
    if (std::abs(raw) < DEADZONE) return 0.0f;
    float normalized = static_cast<float>(raw) / AXIS_MAX;
    return std::max(-1.0f, std::min(1.0f, normalized));
}

void JoystickManager::update() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        // Solo consumimos eventos para mantener SDL vivo
    }

    SDL_JoystickUpdate();

    // Leer canales (Ejes)
    int numAxes = SDL_JoystickNumAxes(m_joystick);
    for (int i = 0; i < 8; i++) {
        if (i < numAxes) {
            m_currentData.channels[i] = normalizeAxis(SDL_JoystickGetAxis(m_joystick, i));
        }
    }

    // Leer botones
    int numButtons = SDL_JoystickNumButtons(m_joystick);
    for (int i = 0; i < 8; i++) {
        if (i < numButtons) {
            m_currentData.buttons[i] = SDL_JoystickGetButton(m_joystick, i);
        }
    }
    
    m_currentData.timestamp = SDL_GetTicks();
}

RCData JoystickManager::getLatestData() {
    return m_currentData;
}

std::string JoystickManager::getDeviceName() const {
    return m_joystick ? SDL_JoystickName(m_joystick) : "None";
}