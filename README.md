# fc-simulator-
# Guía Completa: Control de Avión RC en Gazebo 11 con Radiomaster Pocket

Esta guía te permitirá controlar un avión RC (Cessna) en Gazebo 11 usando tu radio Radiomaster Pocket a través de UDP.

---

## Requisitos Previos

- Ubuntu 22.04 (puede ser en VirtualBox)
- Radiomaster Pocket configurado como joystick USB
- Conexión a internet

---

## Parte 1: Instalación de Gazebo 11

### 1.1 Agregar el repositorio oficial de Gazebo

```bash
# Descargar e instalar la clave GPG
wget https://packages.osrfoundation.org/gazebo.key -O - | gpg --dearmor | sudo tee /usr/share/keyrings/gazebo-archive-keyring.gpg > /dev/null

# Agregar el repositorio
echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

# Actualizar la lista de paquetes
sudo apt update
```

### 1.2 Instalar Gazebo 11 y archivos de desarrollo

```bash
# Instalar Gazebo
sudo apt install gazebo -y

# Instalar archivos de desarrollo (IMPORTANTE)
sudo apt install libgazebo-dev -y
```

### 1.3 Verificar la instalación

```bash
gazebo --version
```

Deberías ver: `Gazebo multi-robot simulator, version 11.x.x`

---

## Parte 2: Optimización de VirtualBox (si aplica)

Si estás usando VirtualBox, configura:

1. **RAM:** Mínimo 4 GB, ideal 8 GB
2. **CPU:** Mínimo 2 núcleos, ideal 4
3. **Memoria de video:** Máximo disponible (128 MB)
4. **Aceleración 3D:** Habilitada (Configuración → Pantalla)

**Nota:** Gazebo en VirtualBox puede ser lento. Para desarrollo serio, considera dual boot.

---

## Parte 3: Programa de Control (C++)

### 3.1 Estructura del proyecto

Crea un directorio para tu proyecto:

```bash
mkdir -p ~/control_gazebo
cd ~/control_gazebo
```

### 3.2 Instalar dependencias

```bash
sudo apt install libsdl2-dev build-essential -y
```

### 3.3 Crear los archivos del programa

#### Archivo: `JoystickManager.hpp`

```cpp
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
    void update();
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
```

#### Archivo: `JoystickManager.cpp`

```cpp
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
```

#### Archivo: `UDPTransport.hpp`

```cpp
#ifndef UDP_TRANSPORT_HPP
#define UDP_TRANSPORT_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>

class UDPTransport {
private:
    int m_sockfd;
    struct sockaddr_in m_destAddr;

public:
    UDPTransport(const std::string& ip, int port) {
        m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sockfd < 0) {
            std::cerr << "Error al crear el socket UDP" << std::endl;
        }
        
        std::memset(&m_destAddr, 0, sizeof(m_destAddr));
        m_destAddr.sin_family = AF_INET;
        m_destAddr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &m_destAddr.sin_addr);
    }
    
    ~UDPTransport() {
        if (m_sockfd >= 0) close(m_sockfd);
    }
    
    void sendFrame(const std::vector<uint8_t>& frame) {
        if (m_sockfd < 0) return;
        
        sendto(m_sockfd, frame.data(), frame.size(), 0,
               (struct sockaddr*)&m_destAddr, sizeof(m_destAddr));
    }
};

#endif
```

#### Archivo: `main.cpp`

```cpp
#include "JoystickManager.hpp"
#include "UDPTransport.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <sstream>

int main() {
    JoystickManager rcInput;
    
    // Transporte UDP simple para Gazebo (puerto 5006)
    UDPTransport gazeboTransport("127.0.0.1", 5006);
    
    if (!rcInput.initialize()) {
        std::cerr << "Error: No se pudo conectar el RadioMaster Pocket." << std::endl;
        return 1;
    }
    
    std::cout << "--- CONTROL DIRECTO PARA GAZEBO ---" << std::endl;
    std::cout << "Transmitiendo datos a Gazebo (Puerto 5006)..." << std::endl;
    std::cout << "Presiona Ctrl+C para salir." << std::endl;
    std::cout << std::endl;
    
    while (true) {
        // 1. Capturar palancas del control [-1.0, 1.0]
        rcInput.update();
        RCData data = rcInput.getLatestData();
        
        // 2. Crear mensaje simple para Gazebo
        // Formato: "T:0.75,A:0.20,E:-0.10,R:0.05"
        std::ostringstream gazeboData;
        gazeboData << std::fixed << std::setprecision(3)
                   << "T:" << data.channels[2]  // Throttle (Gas)
                   << ",A:" << data.channels[0] // Aileron (Roll/Alerones)
                   << ",E:" << data.channels[1] // Elevator (Pitch/Elevador)
                   << ",R:" << data.channels[3]; // Rudder (Yaw/Timón)
        
        std::string dataStr = gazeboData.str();
        
        // 3. Convertir string a bytes y enviar
        std::vector<uint8_t> gazeboFrame(dataStr.begin(), dataStr.end());
        gazeboTransport.sendFrame(gazeboFrame);
        
        // 4. Mostrar datos en consola
        std::cout << "\r"
                  << "[GAZEBO] "
                  << "Roll:" << std::setw(6) << std::fixed << std::setprecision(2) << data.channels[0]
                  << " | Pitch:" << std::setw(6) << data.channels[1]
                  << " | Throttle:" << std::setw(6) << data.channels[2]
                  << " | Yaw:" << std::setw(6) << data.channels[3]
                  << " → " << dataStr
                  << "    " << std::flush;
        
        // Frecuencia de 50Hz (20ms) - estándar RC
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
    }
    
    return 0;
}
```

### 3.4 Compilar el programa

```bash
cd ~/control_gazebo

g++ -std=c++17 main.cpp JoystickManager.cpp -o control_gazebo -lSDL2
```

### 3.5 Probar el programa

```bash
./control_gazebo
```

Deberías ver los valores de los canales cambiando cuando mueves los sticks.

---

## Parte 4: Plugin de Gazebo para UDP

### 4.1 Crear directorio del plugin

```bash
mkdir -p ~/.gazebo/plugins
cd ~/.gazebo/plugins
```

### 4.2 Crear el archivo del plugin

**Archivo: `udp_control_plugin.cc`**

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <cstring>
#include <iostream>

namespace gazebo
{
  class UDPControlPlugin : public ModelPlugin
  {
    private: 
      physics::ModelPtr model;
      physics::LinkPtr bodyLink;
      event::ConnectionPtr updateConnection;
      
      int sockfd;
      std::thread udpThread;
      std::atomic<bool> running;
      
      // Valores de control del RadioMaster
      std::atomic<float> throttle;
      std::atomic<float> aileron;
      std::atomic<float> elevator;
      std::atomic<float> rudder;

    public: 
      UDPControlPlugin() : running(false), throttle(0), aileron(0), elevator(0), rudder(0)
      {
        std::cout << "[UDP Control Plugin] Construido" << std::endl;
      }

      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        this->model = _model;
        
        // Obtener el link principal del modelo
        this->bodyLink = this->model->GetLink("rc_cessna::base_link");
        if (!this->bodyLink)
        {
          // Intentar con otro nombre común
          this->bodyLink = this->model->GetLink("base_link");
        }
        
        if (!this->bodyLink)
        {
          std::cerr << "[UDP Control Plugin] ERROR: No se encontró el link 'body'" << std::endl;
          std::cerr << "Links disponibles:" << std::endl;
          auto links = this->model->GetLinks();
          for (auto link : links)
          {
            std::cerr << "  - " << link->GetName() << std::endl;
          }
          return;
        }
        
        std::cout << "[UDP Control Plugin] Link encontrado: " << this->bodyLink->GetName() << std::endl;
        
        // Crear socket UDP
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0)
        {
          std::cerr << "[UDP Control Plugin] ERROR: No se pudo crear el socket" << std::endl;
          return;
        }
        
        // Configurar socket para no bloquearse
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000; // 10ms timeout
        setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        // Configurar dirección del servidor (escuchar en puerto 5006)
        struct sockaddr_in servaddr;
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(5006);
        
        if (bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
        {
          std::cerr << "[UDP Control Plugin] ERROR: No se pudo hacer bind al puerto 5006" << std::endl;
          close(sockfd);
          return;
        }
        
        std::cout << "[UDP Control Plugin] Escuchando en puerto 5006 UDP" << std::endl;
        
        // Iniciar thread para recibir datos UDP
        running = true;
        udpThread = std::thread(&UDPControlPlugin::ReceiveUDP, this);
        
        // Conectar al update loop de Gazebo
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&UDPControlPlugin::OnUpdate, this));
        
        std::cout << "[UDP Control Plugin] Plugin cargado correctamente" << std::endl;
      }
      
      void ReceiveUDP()
      {
        char buffer[1024];
        
        while(running)
        {
          int n = recv(sockfd, buffer, sizeof(buffer) - 1, 0);
          
          if (n > 0)
          {
            buffer[n] = '\0'; // Null terminator
            
            // Parsear datos: "T:0.75,A:0.20,E:-0.10,R:0.05"
            float t, a, e, r;
            int parsed = sscanf(buffer, "T:%f,A:%f,E:%f,R:%f", &t, &a, &e, &r);
            
            if (parsed == 4)
            {
              throttle.store(t);
              aileron.store(a);
              elevator.store(e);
              rudder.store(r);
              
              // Debug cada 50 mensajes (1 segundo a 50Hz)
              static int count = 0;
              if (++count % 50 == 0)
              {
                std::cout << "[UDP Control Plugin] Recibido: T=" << t 
                          << " A=" << a << " E=" << e << " R=" << r << std::endl;
              }
            }
          }
        }
      }
      
      void OnUpdate()
      {
        if (!this->bodyLink) return;
        
        // Leer valores atómicos
        float t = throttle.load();
        float a = aileron.load();
        float e = elevator.load();
        float r = rudder.load();
        
        // THROTTLE: Solo aplicar fuerza si el throttle es POSITIVO
        // t va de -1 (stick abajo) a +1 (stick arriba)
        // Solo queremos fuerza cuando t > 0
        float forwardForce = 0.0f;
        if (t > 0.0f) {
          forwardForce = t * 300.0f; // De 0 a 300N cuando t va de 0 a 1
        }
        
        ignition::math::Vector3d thrustForce(forwardForce, 0, 0);
        this->bodyLink->AddRelativeForce(thrustForce);
        
        // CONTROLES DE VUELO: Torques para rotación
        float rollTorque = a * 50.0f;    // Aileron → Roll
        float pitchTorque = e * 50.0f;   // Elevator → Pitch
        float yawTorque = r * 30.0f;     // Rudder → Yaw
        
        ignition::math::Vector3d controlTorque(rollTorque, pitchTorque, yawTorque);
        this->bodyLink->AddRelativeTorque(controlTorque);
      }
      
      ~UDPControlPlugin()
      {
        running = false;
        if (udpThread.joinable())
        {
          udpThread.join();
        }
        if (sockfd >= 0)
        {
          close(sockfd);
        }
        std::cout << "[UDP Control Plugin] Plugin destruido" << std::endl;
      }
  };
  
  // Registrar el plugin con Gazebo
  GZ_REGISTER_MODEL_PLUGIN(UDPControlPlugin)
}
```

### 4.3 Compilar el plugin

```bash
cd ~/.gazebo/plugins

g++ -fPIC -shared udp_control_plugin.cc -o libudp_control_plugin.so \
    $(pkg-config --cflags gazebo) \
    $(pkg-config --libs gazebo) \
    -pthread \
    -std=c++17
```

### 4.4 Instalar el plugin

```bash
sudo cp libudp_control_plugin.so /usr/lib/x86_64-linux-gnu/
```

### 4.5 Verificar la instalación

```bash
ls -lh /usr/lib/x86_64-linux-gnu/libudp_control_plugin.so
```

Deberías ver el archivo (aproximadamente 1.4 MB).

---

## Parte 5: Crear el Mundo de Gazebo

### 5.1 Crear directorio para mundos

```bash
mkdir -p ~/gazebo_worlds
cd ~/gazebo_worlds
```

### 5.2 Crear archivo del mundo

**Archivo: `cessna_control.world`**

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- Iluminación -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Suelo -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Modelo del Cessna con plugin de control -->
    <model name="rc_cessna">
      <pose>0 0 0.3 0 0 0</pose>
      <include>
        <uri>https://fuel.ignitionrobotics.org/1.0/px4/models/rc_cessna</uri>
      </include>
      
      <!-- Plugin de control UDP -->
      <plugin name="udp_control" filename="libudp_control_plugin.so"/>
    </model>
    
    <!-- Configuración de física -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Gravedad -->
    <gravity>0 0 -9.8</gravity>
  </world>
</sdf>
```

---

## Parte 6: Uso del Sistema

### 6.1 Preparación

1. **Conecta tu Radiomaster Pocket** al puerto USB
2. **Verifica que se detecte:**
   ```bash
   ls /dev/input/js*
   ```
   Deberías ver `/dev/input/js0`

### 6.2 Ejecutar el sistema

**Terminal 1 - Programa de control:**
```bash
cd ~/control_gazebo
./control_gazebo
```

Deberías ver los valores de los canales actualizándose.

**Terminal 2 - Gazebo:**
```bash
gazebo ~/gazebo_worlds/cessna_control.world --verbose
```

### 6.3 Verificar que funciona

En la Terminal 2, deberías ver:
```
[UDP Control Plugin] Construido
[UDP Control Plugin] Link encontrado: base_link
[UDP Control Plugin] Escuchando en puerto 5006 UDP
[UDP Control Plugin] Plugin cargado correctamente
[UDP Control Plugin] Recibido: T=0.5 A=0.0 E=0.0 R=0.0
```

### 6.4 Controlar el avión

1. **Resetear posición:** Presiona `Ctrl+R` en Gazebo o ve a `Edit → Reset Model Poses`
2. **Throttle (Gas):** Stick izquierdo arriba/abajo
   - Abajo (T=-1): Motor apagado
   - Arriba (T=+1): Motor al máximo
3. **Aileron (Roll):** Stick derecho izquierda/derecha
4. **Elevator (Pitch):** Stick derecho arriba/abajo
5. **Rudder (Yaw):** Normalmente en Canal 4

---

## Parte 7: Ajustes y Calibración

### 7.1 Ajustar fuerzas del plugin

Si el avión es muy rápido o muy lento, edita el plugin:

```bash
nano ~/.gazebo/plugins/udp_control_plugin.cc
```

Modifica estos valores en la función `OnUpdate()`:

```cpp
// Aumentar o disminuir estas constantes:
forwardForce = t * 300.0f;    // Fuerza del motor (default: 300)
float rollTorque = a * 50.0f;  // Fuerza de roll (default: 50)
float pitchTorque = e * 50.0f; // Fuerza de pitch (default: 50)
float yawTorque = r * 30.0f;   // Fuerza de yaw (default: 30)
```

Después de modificar, recompila:
```bash
cd ~/.gazebo/plugins
g++ -fPIC -shared udp_control_plugin.cc -o libudp_control_plugin.so \
    $(pkg-config --cflags gazebo) \
    $(pkg-config --libs gazebo) \
    -pthread \
    -std=c++17
sudo cp libudp_control_plugin.so /usr/lib/x86_64-linux-gnu/
```

### 7.2 Verificar mapeo de canales

Si los controles están invertidos o en el canal incorrecto, edita `main.cpp`:

```cpp
// Ajusta estos índices según tu configuración:
<< "T:" << data.channels[2]  // Throttle (prueba con 0, 1, 2, 3)
<< ",A:" << data.channels[0] // Aileron
<< ",E:" << data.channels[1] // Elevator
<< ",R:" << data.channels[3]; // Rudder
```

Recompila:
```bash
g++ -std=c++17 main.cpp JoystickManager.cpp -o control_gazebo -lSDL2
```

---

## Solución de Problemas Comunes

### Problema 1: "No se pudo conectar el RadioMaster Pocket"

**Solución:**
```bash
# Verificar que el joystick se detecta
ls /dev/input/js*

# Dar permisos
sudo chmod 666 /dev/input/js0
```

### Problema 2: "Failed to load plugin libudp_control_plugin.so"

**Solución:**
```bash
# Verificar que el plugin existe
ls -lh /usr/lib/x86_64-linux-gnu/libudp_control_plugin.so

# Si no existe, copiarlo de nuevo
sudo cp ~/.gazebo/plugins/libudp_control_plugin.so /usr/lib/x86_64-linux-gnu/
```

### Problema 3: El avión no aparece en Gazebo

**Solución:**
```bash
# En Gazebo: Presiona Ctrl+R o ve a Edit → Reset Model Poses

# O limpiar caché:
killall gzserver gzclient
rm -rf ~/.gazebo/paging
rm -rf /tmp/gazebo*
```

### Problema 4: El avión se mueve aunque el throttle esté en mínimo

**Verificar en Terminal 1:**
- Cuando el stick de throttle está abajo, T debería ser cercano a -1.0
- Si T > 0, el avión acelerará

**Solución:** Calibra tu radio en EdgeTX para que el rango sea exactamente de -1.0 a +1.0

### Problema 5: Gazebo muy lento en VirtualBox

**Optimizaciones:**
1. Aumentar RAM a 8 GB
2. Asignar 4 núcleos de CPU
3. Habilitar aceleración 3D
4. Considerar usar dual boot para mejor rendimiento

---

## Archivos Finales del Proyecto

```
~/control_gazebo/
├── main.cpp
├── JoystickManager.hpp
├── JoystickManager.cpp
├── UDPTransport.hpp
└── control_gazebo (ejecutable)

~/.gazebo/plugins/
├── udp_control_plugin.cc
└── libudp_control_plugin.so

~/gazebo_worlds/
└── cessna_control.world

/usr/lib/x86_64-linux-gnu/
└── libudp_control_plugin.so
```

---

## Notas Adicionales

### Limitaciones conocidas:
- El modelo Cessna no tiene física aerodinámica realista
- En VirtualBox el rendimiento es limitado
- Los errores sobre sensores GPS y barómetro son normales (el modelo los incluye pero Gazebo 11 no los soporta)

### Mejoras futuras:
- Agregar soporte para más canales (flaps, tren de aterrizaje, etc.)
- Implementar telemetría (enviar datos del avión de vuelta al control)
- Usar PX4-SITL para simulación más realista
- Agregar interfaz gráfica para monitoreo

---

## Recursos Adicionales

- **Documentación de Gazebo:** http://gazebosim.org/tutorials
- **Fuel Models:** https://app.gazebosim.org/fuel/models
- **SDL2 Documentation:** https://wiki.libsdl.org/

---

¡Disfruta volando tu Cessna virtual! 🛩️
