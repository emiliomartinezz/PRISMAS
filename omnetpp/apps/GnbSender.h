// GnbSender.h
// Aplicación que envía información del gNodeB: posición, UEs conectados

#ifndef __GNBSENDER_H
#define __GNBSENDER_H

#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/mobility/contract/IMobility.h"
#include <vector>
#include <set>
#include <string>

// Forward declarations
class Binder;
class LteMacEnb;

using namespace inet;
using namespace omnetpp;

class GnbSender : public ApplicationBase, public UdpSocket::ICallback
{
  protected:
    // Parámetros
    int localPort = -1;
    int destPort = -1;
    simtime_t sendInterval;
    simtime_t startTime;
    L3Address destAddr;

    // Estado
    UdpSocket socket;
    IMobility *mobility = nullptr;
    cMessage *selfMsg = nullptr;

    // Referencias a módulos Simu5G
    Binder *binder = nullptr;
    cModule *cellularNic = nullptr;

    // Identificación del gNodeB
    int macCellId = -1;
    int macNodeId = -1;
    int gnbIndex = -1;  // Índice del array gNodeB[x]
    std::string sumoId;  // NUEVO: id del UAV en SUMO que transporta este gNodeB

    // NUEVO: vector de la red donde viven los UEs ("car" con Veins)
    std::string ueVectorName;

    // Estadísticas
    int packetsSent = 0;
    simsignal_t packetSentSignal;

  protected:
    // ApplicationBase
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish() override;

    // Métodos auxiliares
    void sendGnbInfo();
    std::vector<int> getConnectedUeIds();
    int getNumConnectedUes();

    // Ciclo de vida
    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;

    // UdpSocket callbacks
    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override;
    virtual void socketClosed(UdpSocket *socket) override;

  public:
    virtual ~GnbSender();
};

#endif // __GNBSENDER_H
