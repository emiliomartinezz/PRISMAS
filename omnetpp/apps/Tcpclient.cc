//
// Cliente TCP real integrado en OMNeT++
// Usa sockets del sistema operativo para conectarse al gemelo digital.
//
// Paso 1: se elimina la dependencia de DroneController. Los mensajes que
// lleguen del gemelo (p. ej. los MOVE que sigue generando el gemelo de
// Lucas) se registran y se ignoran: en esta arquitectura OMNeT++ no mueve
// los UAV, solo los sigue.
//

#include "Tcpclient.h"

#include <mosquitto.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include <regex>

Define_Module(TcpClient);

static const int KEEPALIVE_S = 30;
static const auto RECONNECT_EVERY = std::chrono::seconds(1);

// ---------------------------------------------------------------- callbacks
// libmosquitto los llama desde dentro de mosquitto_loop(), es decir, en el
// hilo de OMNeT++.

static void onConnect(struct mosquitto *, void *obj, int rc)
{
    static_cast<TcpClient *>(obj)->onBrokerConnected(rc);
}

static void onDisconnect(struct mosquitto *, void *obj, int rc)
{
    static_cast<TcpClient *>(obj)->onBrokerDisconnected(rc);
}

void TcpClient::onBrokerConnected(int rc)
{
    if (rc == 0) {
        brokerConnected = true;
        std::cout << "[MQTT] Conectado al broker " << brokerHost << ":" << brokerPort << std::endl;
    }
    else {
        brokerConnected = false;
        std::cerr << "[MQTT] El broker rechazo la conexion: "
                  << mosquitto_connack_string(rc) << std::endl;
    }
}

void TcpClient::onBrokerDisconnected(int rc)
{
    brokerConnected = false;
    if (rc != 0)
        std::cerr << "[MQTT] Conexion con el broker perdida; reintentando..." << std::endl;
}

// --------------------------------------------------------------------- ciclo

void TcpClient::initialize()
{
    brokerHost = par("brokerHost").stdstringValue();
    brokerPort = par("brokerPort");
    topicPrefix = par("topicPrefix").stdstringValue();

    mosquitto_lib_init();

    std::string clientId = topicPrefix + "-omnet-" + std::to_string(::getpid());
    mosq = mosquitto_new(clientId.c_str(), true, this);
    if (!mosq) {
        std::cerr << "[MQTT] No se pudo crear el cliente MQTT. "
                     "La telemetria NO se reenviara." << std::endl;
        return;
    }
    mosquitto_connect_callback_set(mosq, onConnect);
    mosquitto_disconnect_callback_set(mosq, onDisconnect);

    // Credenciales SOLO por variables de entorno (mqtt.env, fuera de git), para
    // que no acaben en omnetpp.ini. Sin ellas se conecta sin usuario.
    const char *user = std::getenv("PRISMAS_MQTT_USER");
    const char *pass = std::getenv("PRISMAS_MQTT_PASS");
    bool withAuth = user && *user;
    if (withAuth)
        mosquitto_username_pw_set(mosq, user, pass ? pass : "");

    std::cout << "[MQTT] Conectando al broker " << brokerHost << ":" << brokerPort
              << " (prefijo de topics '" << topicPrefix << "', "
              << (withAuth ? std::string("usuario '") + user + "'" : std::string("sin credenciales"))
              << ")" << std::endl;

    // Conexion no bloqueante: la termina mosquitto_loop()
    lastReconnectAttempt = std::chrono::steady_clock::now();
    int rc = mosquitto_connect_async(mosq, brokerHost.c_str(), brokerPort, KEEPALIVE_S);
    if (rc != MOSQ_ERR_SUCCESS)
        std::cerr << "[MQTT] Broker no disponible (" << mosquitto_strerror(rc)
                  << "); se reintentara cada segundo" << std::endl;

    // Esperar un poco a la conexion para no perder los primeros reports. Si el
    // broker no esta, la simulacion arranca igualmente.
    double waitLimit = par("connectTimeout").doubleValue();
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(static_cast<long>(waitLimit * 1000));
    while (!brokerConnected && std::chrono::steady_clock::now() < deadline) {
        serviceMqtt();
        ::usleep(20000);
    }
    if (!brokerConnected)
        std::cerr << "[MQTT] El broker aun no responde; la simulacion arranca igualmente "
                     "y la telemetria se enviara cuando conecte" << std::endl;

    checkMsg = new cMessage("checkConnection");
    scheduleAt(simTime() + 0.1, checkMsg);
}

void TcpClient::handleMessage(cMessage *msg)
{
    if (msg != checkMsg) {   // este modulo no recibe otros mensajes
        delete msg;
        return;
    }
    serviceMqtt();
    scheduleAt(simTime() + 0.1, checkMsg);
}

void TcpClient::serviceMqtt()
{
    if (!mosq)
        return;

    // Atiende la red sin bloquear: envia lo pendiente, lee CONNACK/PINGRESP
    // y manda el keepalive cuando toca.
    int rc = mosquitto_loop(mosq, 0, 1);
    if (rc == MOSQ_ERR_SUCCESS)
        return;

    // Sin conexion (broker caido, aun no estaba, o se perdio): reintentar
    brokerConnected = false;
    auto now = std::chrono::steady_clock::now();
    if (now - lastReconnectAttempt >= RECONNECT_EVERY) {
        lastReconnectAttempt = now;
        mosquitto_reconnect_async(mosq);
    }
}

// --------------------------------------------------------------------- envio

std::string TcpClient::topicFor(const std::string& payload) const
{
    static const std::regex ueIdx("\"ue_index\"\\s*:\\s*(-?\\d+)");
    static const std::regex gnbIdx("\"gnb_index\"\\s*:\\s*(-?\\d+)");
    std::smatch m;

    if (payload.find("\"UE_Report\"") != std::string::npos) {
        std::string idx = std::regex_search(payload, m, ueIdx) ? m[1].str() : "unknown";
        return topicPrefix + "/omnet/ue/" + idx + "/report";
    }
    if (payload.find("\"GNB_Report\"") != std::string::npos) {
        std::string idx = std::regex_search(payload, m, gnbIdx) ? m[1].str() : "unknown";
        return topicPrefix + "/omnet/gnb/" + idx + "/report";
    }
    return topicPrefix + "/omnet/other";
}

void TcpClient::sendToServer(const std::string& data)
{
    if (!mosq)
        return;

    // Sin broker no se acumula nada: la telemetria es periodica y una muestra
    // vieja no le sirve al gemelo.
    if (!brokerConnected) {
        if (dropped++ % 500 == 0)
            std::cerr << "[MQTT] Sin conexion con el broker: telemetria descartada ("
                      << dropped << " mensajes)" << std::endl;
        return;
    }

    std::string topic = topicFor(data);
    int rc = mosquitto_publish(mosq, nullptr, topic.c_str(),
                               static_cast<int>(data.size()), data.data(),
                               0 /* QoS */, false /* retain */);
    if (rc == MOSQ_ERR_SUCCESS) {
        published++;
    }
    else {
        brokerConnected = false;   // serviceMqtt() se encargara de reconectar
        if (dropped++ % 500 == 0)
            std::cerr << "[MQTT] Error publicando en " << topic << ": "
                      << mosquitto_strerror(rc) << std::endl;
    }
}

// ------------------------------------------------------------------- cierre

void TcpClient::shutdownMqtt()
{
    if (!mosq)
        return;
    if (brokerConnected) {
        mosquitto_loop(mosq, 100, 1);   // vaciar lo que quede por enviar
        mosquitto_disconnect(mosq);
        mosquitto_loop(mosq, 100, 1);
    }
    mosquitto_destroy(mosq);
    mosq = nullptr;
    mosquitto_lib_cleanup();
    brokerConnected = false;
}

void TcpClient::finish()
{
    recordScalar("mqttPublished", published);
    recordScalar("mqttDropped", dropped);
    std::cout << "[MQTT] Fin: " << published << " mensajes publicados, "
              << dropped << " descartados" << std::endl;
    shutdownMqtt();
}

TcpClient::~TcpClient()
{
    cancelAndDelete(checkMsg);
    shutdownMqtt();
}
