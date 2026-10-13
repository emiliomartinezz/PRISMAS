// UESender.h
// Header para la aplicación UESender con métricas Simu5G

#ifndef __UESENDER_H
#define __UESENDER_H

#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "inet/networklayer/common/L3Address.h"
#include "inet/mobility/contract/IMobility.h"
#include <omnetpp.h>

// Forward declaration
class Binder;

using namespace inet;
using namespace omnetpp;

class UESender : public ApplicationBase, public UdpSocket::ICallback, public cListener
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

    // Identificación del UE
    int ueNodeId = -1;   // nrMacNodeId del UE
    int ueIndex = -1;    // Índice del array ue[X]

    // Referencias a módulos Simu5G para métricas
    cModule *cellularNic = nullptr;
    cModule *nrPhy = nullptr;
    Binder *binder = nullptr;

    // Últimos valores de métricas recibidos
    double lastSinr = -999.0;
    int lastMasterNodeId = -1;

    // Señales para suscribirse a métricas
    simsignal_t sinrSignal;

    // NUEVO: módulo en el que estamos suscritos a measuredSinrUl.
    // Con Veins los coches se destruyen al salir de SUMO, así que hay que
    // desuscribirse en finish() para no dejar un listener colgando.
    cModule *sinrSource = nullptr;

    // Estadísticas
    int packetsSent = 0;
    simsignal_t packetSentSignal;

  protected:
    // ApplicationBase
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish() override;

    cModule *myCellularNic = nullptr;

    // Envío
    void sendUE_Report();

    // Obtener macNodeId del gNodeB conectado ACTUAL desde el Binder
    int getCurrentMasterNodeId();
    int getMacNodeIdFromBinder();

    // cListener - Callback para capturar señales
    virtual void receiveSignal(cComponent *source, simsignal_t signalID,
                              double value, cObject *details) override;

    // Ciclo de vida
    virtual void handleStartOperation(LifecycleOperation *operation) override;
    virtual void handleStopOperation(LifecycleOperation *operation) override;
    virtual void handleCrashOperation(LifecycleOperation *operation) override;

    // UdpSocket callbacks
    virtual void socketDataArrived(UdpSocket *socket, Packet *packet) override;
    virtual void socketErrorArrived(UdpSocket *socket, Indication *indication) override;
    virtual void socketClosed(UdpSocket *socket) override;

  public:
    virtual ~UESender();
};

#endif // __UESENDER_H
