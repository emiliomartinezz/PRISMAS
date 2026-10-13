#ifndef __SOCKETCLIENT_TCPCLIENT_H_
#define __SOCKETCLIENT_TCPCLIENT_H_

#include <omnetpp.h>
#include <string>

using namespace omnetpp;

// Paso 1: TcpClient solo reenvia la telemetria al gemelo digital.
// Ya no depende de DroneController: la posicion de los UAV la decide
// SUMO-UAV-Py y (en el paso 2) el gemelo le mandara las ordenes al plugin.
class TcpClient : public cSimpleModule
{
  protected:
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;

  public:
    void sendToServer(const std::string& data);
};

#endif
