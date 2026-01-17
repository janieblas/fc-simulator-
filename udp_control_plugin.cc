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
      event::ConnectionPtr updateConnection;
      
      physics::JointPtr motorJoint;
      physics::JointPtr servo0Joint;
      physics::JointPtr servo1Joint;
      physics::JointPtr servo2Joint;
      physics::JointPtr rudderJoint;
      physics::JointPtr leftFlapJoint;
      physics::JointPtr rightFlapJoint;

      int sockfd;
      std::thread udpThread;
      std::atomic<bool> running;
      
      std::atomic<float> throttle{0}, aileron{0}, elevator{0}, rudder{0};

    public: 
      UDPControlPlugin() : running(false) {}

      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        this->model = _model;
        
        this->motorJoint = this->model->GetJoint("rotor_puller_joint");
        this->servo0Joint = this->model->GetJoint("servo_0");
        this->servo1Joint = this->model->GetJoint("servo_1");
        this->servo2Joint = this->model->GetJoint("servo_2");
        this->rudderJoint = this->model->GetJoint("rudder_joint");
        this->leftFlapJoint = this->model->GetJoint("left_flap_joint");
        this->rightFlapJoint = this->model->GetJoint("right_flap_joint");

        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in servaddr;
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(5006);
        bind(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr));
        
        running = true;
        udpThread = std::thread(&UDPControlPlugin::ReceiveUDP, this);
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&UDPControlPlugin::OnUpdate, this));
        
        std::cout << "[UDP Plugin t10] Sistema de Actuadores listo" << std::endl;
      }
      
      void ReceiveUDP()
      {
        char buffer[1024];
        while(running)
        {
          int n = recv(sockfd, buffer, sizeof(buffer) - 1, 0);
          if (n > 0)
          {
            buffer[n] = '\0';
            float t, a, e, r;
            if (sscanf(buffer, "T:%f,A:%f,E:%f,R:%f", &t, &a, &e, &r) == 4)
            {
              throttle.store(t); 
              aileron.store(a); 
              elevator.store(e); 
              rudder.store(r);
            }
          }
        }
      }
      
      void OnUpdate()
      {
        float t = throttle.load();
        float vA = aileron.load();
        float vE = elevator.load();
        float vR = rudder.load();

        if (t < 0) t = 0; 

        physics::LinkPtr baseLink = this->model->GetLink("base_link");
        if (!baseLink) return;

        if (this->motorJoint) {
            this->motorJoint->SetVelocity(0, t * 400.0); 
            ignition::math::Vector3d fuerza(t * 12.5, 0, 0); 
            ignition::math::Vector3d puntoAplicacion(0.1, 0, 0.1); 
            baseLink->AddLinkForce(fuerza, puntoAplicacion);
        }  

        float maxRad = 0.5; 
        if (servo0Joint) servo0Joint->SetPosition(0, -vA * maxRad); 
        if (servo1Joint) servo1Joint->SetPosition(0, vA * maxRad);  
        if (servo2Joint) servo2Joint->SetPosition(0, -vE * maxRad);  
        if (rudderJoint) rudderJoint->SetPosition(0, vR * maxRad); 

        if (leftFlapJoint) leftFlapJoint->SetPosition(0, -vA * maxRad);
        if (rightFlapJoint) rightFlapJoint->SetPosition(0, vA * maxRad);
      }

      ~UDPControlPlugin()
      {
        running = false;
        if (udpThread.joinable()) udpThread.join();
        if (sockfd >= 0) close(sockfd);
      }
  };
  GZ_REGISTER_MODEL_PLUGIN(UDPControlPlugin)
}
