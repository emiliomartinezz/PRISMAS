#ifndef __SOCKETCLIENT_TCPCLIENT_H_
#define __SOCKETCLIENT_TCPCLIENT_H_

#include <omnetpp.h>
#include <chrono>
#include <string>

using namespace omnetpp;

struct mosquitto;

class TcpClient : public cSimpleModule
{
  protected:
    struct mosquitto *mosq = nullptr;
    cMessage *checkMsg = nullptr;

    std::string brokerHost;
    int brokerPort = 1883;
    std::string topicPrefix;

    bool brokerConnected = false;
    std::chrono::steady_clock::time_point lastReconnectAttempt;

    long published = 0;
    long dropped = 0;

    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;

    void serviceMqtt();
    void shutdownMqtt();
    std::string topicFor(const std::string& payload) const;

  public:
    virtual ~TcpClient();
    void sendToServer(const std::string& data);

    // Llamadas desde los callbacks de libmosquitto (mismo hilo)
    void onBrokerConnected(int rc);
    void onBrokerDisconnected(int rc);
};

#endif
