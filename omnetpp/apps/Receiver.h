// Receiver.h
// Header para la aplicación Receiver

#ifndef __RECEIVER_H
#define __RECEIVER_H

#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/networklayer/common/L3Address.h"
#include "Tcpclient.h"

using namespace inet;

class Receiver : public ApplicationBase, public UdpSocket::ICallback
{
  private:
    // Puntero al cliente TCP para reenvío al servidor externo
    TcpClient *tcpClient = nullptr;

    // Handlers por tipo de mensaje CSV
    void handleUE_ReportMessage(const std::string &payload,
                               const L3Address &srcAddr, int srcPort);
    void handleGNB_ReportMessage(const std::string &payload,
                               const L3Address &srcAddr, int srcPort);

  protected:
    // Parámetros
    int localPort = -1;

    // Estado
    UdpSocket socket;

    // Estadísticas
    int packetsReceived = 0;
    simsignal_t packetReceivedSignal;

  protected:
    // ApplicationBase
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish() override;

    // Callbacks del socket UDP
    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override;
    virtual void socketClosed(UdpSocket *socket) override;

    // Ciclo de vida
    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;
};

#endif // __RECEIVER_H
