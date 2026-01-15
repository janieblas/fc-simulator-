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
        
        // THROTTLE: Fuerza hacia adelante (en el eje X local del avión)
        // Mapear de [-1,1] a [0, 200] Newtons
        float forwardForce = 0.0f;
	if(t > 0.0f){
	    forwardForce = t * 150.0f; // De 0 a 300N cuando t va de 0 a 1
	}
        
        ignition::math::Vector3d thrustForce(forwardForce, 0, 0);
        this->bodyLink->AddRelativeForce(thrustForce);
        
        // CONTROLES DE VUELO: Torques para rotación
        // Ajusta estos valores según el comportamiento deseado
        float rollTorque = a * 4.0f;   // Aileron → Roll (eje X)
        float pitchTorque = e * 4.0f;  // Elevator → Pitch (eje Y)
        float yawTorque = r * 2.0f;    // Rudder → Yaw (eje Z)
        
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
