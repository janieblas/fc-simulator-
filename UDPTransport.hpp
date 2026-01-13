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
        // Crear socket UDP (SOCK_DGRAM)
        m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sockfd < 0) {
            std::cerr << "Error al crear el socket UDP" << std::endl;
        }
        
        // Configurar la dirección de destino
        std::memset(&m_destAddr, 0, sizeof(m_destAddr));
        m_destAddr.sin_family = AF_INET;
        m_destAddr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &m_destAddr.sin_addr);
    }

    // El destructor cierra el socket automáticamente
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