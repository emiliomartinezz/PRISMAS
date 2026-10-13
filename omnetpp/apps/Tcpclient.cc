//
// Cliente TCP real integrado en OMNeT++
// Usa sockets del sistema operativo para conectarse al gemelo digital.
//
// Paso 1: se elimina la dependencia de DroneController. Los mensajes que
// lleguen del gemelo (p. ej. los MOVE que sigue generando el gemelo de
// Lucas) se registran y se ignoran: en esta arquitectura OMNeT++ no mueve
// los UAV, solo los sigue.
//

#include "inet/common/INETDefs.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <errno.h>
#include "Tcpclient.h"

Define_Module(TcpClient);

// Variables globales para el socket real del cliente
static int realClientFd = -1;
static bool isConnected = false;

void TcpClient::initialize()
{
    const char *SERVER_IP = "127.0.0.1";
    const int SERVER_PORT = 50000;

    realClientFd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (realClientFd < 0) {
        std::cerr << "[Cliente] Error creando socket: " << strerror(errno) << std::endl;
        return;
    }

    // No bloqueante: OMNeT++ es monohilo
    ::fcntl(realClientFd, F_SETFL, O_NONBLOCK);

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);

    if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) <= 0) {
        std::cerr << "[Cliente] Direccion IP invalida" << std::endl;
        ::close(realClientFd);
        realClientFd = -1;
        return;
    }

    int ret = ::connect(realClientFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if (ret < 0 && errno != EINPROGRESS) {
        std::cerr << "[Cliente] Error conectando: " << strerror(errno) << std::endl;
        ::close(realClientFd);
        realClientFd = -1;
        return;
    }

    std::cout << "[Cliente] Intentando conexion con "
              << SERVER_IP << ":" << SERVER_PORT
              << " (el gemelo debe estar escuchando ANTES de lanzar OMNeT++)" << std::endl;

    cMessage *checkMsg = new cMessage("checkConnection");
    scheduleAt(simTime() + 0.1, checkMsg);
}

void TcpClient::handleMessage(cMessage *msg)
{
    if (!msg->isSelfMessage() || strcmp(msg->getName(), "checkConnection") != 0) {
        delete msg;
        return;
    }

    if (realClientFd < 0) {
        delete msg;   // sin socket no hay nada que vigilar
        return;
    }

    // 1) ¿Se ha completado el connect() no bloqueante?
    if (!isConnected) {
        int error = 0;
        socklen_t len = sizeof(error);
        int ret = ::getsockopt(realClientFd, SOL_SOCKET, SO_ERROR, &error, &len);

        if (ret == 0 && error == 0) {
            // Comprobamos que realmente esta conectado (no solo "en curso")
            struct sockaddr_in peer;
            socklen_t plen = sizeof(peer);
            if (::getpeername(realClientFd, (struct sockaddr *)&peer, &plen) == 0) {
                isConnected = true;
                std::cout << "[Cliente] Conexion establecida con el gemelo digital" << std::endl;
            }
        }
        else if (ret == 0 && error != 0) {
            std::cerr << "[Cliente] No se pudo conectar con el gemelo: " << strerror(error)
                      << ". La telemetria NO se reenviara." << std::endl;
            ::close(realClientFd);
            realClientFd = -1;
            delete msg;
            return;
        }
    }

    // 2) Leer (y descartar) lo que mande el gemelo
    if (isConnected) {
        char buffer[4096];
        ssize_t bytesRead = ::recv(realClientFd, buffer, sizeof(buffer) - 1, 0);

        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';
            std::string receivedMsg(buffer);

            if (receivedMsg == "SERVER_CLOSED") {
                std::cout << "[Cliente] El gemelo cerro la conexion" << std::endl;
                ::close(realClientFd);
                realClientFd = -1;
                isConnected = false;
                delete msg;
                return;
            }

            std::cout << "[Cliente] Mensaje del gemelo ignorado en paso 1 "
                         "(los UAV los mueve SUMO-UAV-Py): "
                      << receivedMsg.substr(0, 120) << std::endl;
        }
        else if (bytesRead == 0) {
            std::cout << "[Cliente] Gemelo desconectado" << std::endl;
            ::close(realClientFd);
            realClientFd = -1;
            isConnected = false;
            delete msg;
            return;
        }
    }

    scheduleAt(simTime() + 0.1, msg);
}

void TcpClient::sendToServer(const std::string& data)
{
    if (!isConnected || realClientFd < 0)
        return;

    // MSG_NOSIGNAL: si el gemelo se cae, send() devuelve EPIPE en lugar de
    // lanzar SIGPIPE y matar el proceso de OMNeT++.
    ssize_t sent = ::send(realClientFd, data.c_str(), data.size(), MSG_NOSIGNAL);

    if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        std::cerr << "[Cliente] Error enviando al gemelo: " << strerror(errno) << std::endl;
        ::close(realClientFd);
        realClientFd = -1;
        isConnected = false;
    }
}

void TcpClient::finish()
{
    if (realClientFd >= 0) {
        ::close(realClientFd);
        realClientFd = -1;
    }
    isConnected = false;
    std::cout << "[Cliente] Cerrando cliente TCP" << std::endl;
}
