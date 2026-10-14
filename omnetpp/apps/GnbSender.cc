// GnbSender.cc
// Aplicación que envía información periódica del gNodeB al servidor
// Formato: JSON {"type":"GNB_Report",...}
//
// Cambios paso 1 (coches = UEs creados por Veins):
//   - getConnectedUeIds() recorre el vector dinámico indicado en
//     'ueVectorName' (por defecto "car") en lugar de ue[0..numUe-1],
//     saltando los huecos que deja Veins cuando un coche sale de SUMO.
//   - El GNB_Report incluye "sumo_id" (uav0, uav1...), que es la clave que
//     necesitará el gemelo para mandar órdenes al plugin en el paso 2.
//   - El GNB_Report incluye "uav_state" (MISSION, RETURNING, CHARGING,
//     STANDBY, ABSENT o UNKNOWN) para que el gemelo ignore los UAVs que no
//     están en misión.

#include "GnbSender.h"
#include "UavFollowerMobility.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/packet/chunk/BytesChunk.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/ModuleAccess.h"
#include "common/binder/Binder.h"
#include "stack/mac/layer/LteMacEnb.h"
#include <sstream>
#include <iomanip>
#include <algorithm>

Define_Module(GnbSender);

GnbSender::~GnbSender()
{
    cancelAndDelete(selfMsg);
}

void GnbSender::initialize(int stage)
{
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        // Leer parámetros
        localPort = par("localPort");
        destPort = par("destPort");
        sendInterval = par("sendInterval");
        startTime = par("startTime");
        ueVectorName = par("ueVectorName").stdstringValue();

        packetsSent = 0;
        packetSentSignal = registerSignal("packetSent");

        // Crear mensaje de timer
        selfMsg = new cMessage("sendTimer");

        EV_INFO << "GnbSender initialized - will start at " << startTime << endl;
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        // Obtener módulo de movilidad
        mobility = check_and_cast<IMobility*>(
            getParentModule()->getSubmodule("mobility")
        );

        // Si el gNodeB sigue a un UAV de SUMO, guardamos su id
        if (auto *follower = dynamic_cast<UavFollowerMobility*>(mobility))
            sumoId = follower->getSumoId();

        // Configurar socket UDP
        socket.setOutputGate(gate("socketOut"));
        socket.setCallback(this);

        // Resolver dirección destino
        const char *destAddrStr = par("destAddress");
        destAddr = L3AddressResolver().resolve(destAddrStr);

        if (localPort != -1)
            socket.bind(localPort);

        socket.connect(destAddr, destPort);

        EV_INFO << "Socket configured - dest: " << destAddr
                << ":" << destPort << endl;

        // ===== OBTENER INFORMACIÓN DEL GNODEB =====

        cModule *gnbModule = getParentModule();

        if (gnbModule->hasPar("macCellId")) {
            macCellId = gnbModule->par("macCellId").intValue();
        }
        if (gnbModule->hasPar("macNodeId")) {
            macNodeId = gnbModule->par("macNodeId").intValue();
        }

        // Índice del array (gNodeB[0], gNodeB[1], etc.)
        if (gnbModule->isVector())
            gnbIndex = gnbModule->getIndex();

        std::cout << "[GnbSender] gNodeB[" << gnbIndex << "] initialized"
                  << " (macCellId=" << macCellId
                  << ", macNodeId=" << macNodeId
                  << ", sumoId=" << (sumoId.empty() ? "-" : sumoId) << ")" << std::endl;

        // Obtener referencia al Binder
        binder = check_and_cast<Binder*>(
            getSimulation()->getSystemModule()->getSubmodule("binder")
        );

        if (!binder) {
            throw cRuntimeError("GnbSender: Binder module not found");
        }

        // Obtener referencia a cellularNic
        cellularNic = gnbModule->getSubmodule("cellularNic");
        if (!cellularNic) {
            throw cRuntimeError("GnbSender: cellularNic module not found");
        }

        // Programar primer envío
        scheduleAt(simTime() + startTime, selfMsg);

        std::cout << "[GnbSender] First send scheduled at t="
                  << (simTime() + startTime) << "s" << std::endl;
    }
}

std::vector<int> GnbSender::getConnectedUeIds()
{
    std::vector<int> connectedUes;

    if (!binder) {
        return connectedUes;
    }

    try {
        // ============================================================
        // Recorremos los UEs que existen AHORA en la red y preguntamos al
        // Binder cuál es su master node (binder->getNextHop(ueId)).
        //
        // Con Veins los UEs (coches) se crean y destruyen durante la
        // simulación: el vector car[] crece y los coches que salen de SUMO
        // dejan huecos (getSubmodule devuelve nullptr).
        // ============================================================
        cModule *network = getSimulation()->getSystemModule();
        const char *vec = ueVectorName.c_str();

        if (!network->hasSubmoduleVector(vec))
            return connectedUes;

        int vecSize = network->getSubmoduleVectorSize(vec);

        for (int i = 0; i < vecSize; i++) {
            cModule *ueModule = network->getSubmodule(vec, i);
            if (!ueModule) continue;   // hueco: el coche ya salió de SUMO

            // nrMacNodeId del UE (ID NR asignado por Simu5G)
            MacNodeId ueNrId = 0;
            if (ueModule->hasPar("nrMacNodeId")) {
                ueNrId = ueModule->par("nrMacNodeId").intValue();
            }
            // Fallback: macNodeId
            if (ueNrId == 0 && ueModule->hasPar("macNodeId")) {
                ueNrId = ueModule->par("macNodeId").intValue();
            }

            if (ueNrId == 0) continue;   // aún inicializándose

            MacNodeId masterNode = binder->getNextHop(ueNrId);

            if ((int)masterNode == macNodeId) {
                connectedUes.push_back(ueNrId);
            }
        }

        if (connectedUes.empty()) {
            EV_DEBUG << "[GnbSender] gNodeB[" << gnbIndex
                     << "] No UEs found connected (macNodeId=" << macNodeId << ")" << endl;
        }

    } catch (const std::exception& e) {
        EV_ERROR << "Error getting connected UEs: " << e.what() << endl;
        std::cout << "[GnbSender] ERROR in getConnectedUeIds: " << e.what() << std::endl;
    }

    return connectedUes;
}

int GnbSender::getNumConnectedUes()
{
    return getConnectedUeIds().size();
}

void GnbSender::handleMessageWhenUp(cMessage *msg)
{
    if (msg == selfMsg) {
        sendGnbInfo();
        scheduleAt(simTime() + sendInterval, selfMsg);
    }
    else {
        socket.processMessage(msg);
    }
}

void GnbSender::sendGnbInfo()
{
    Coord pos = mobility->getCurrentPosition();
    std::vector<int> connectedUes = getConnectedUeIds();

    // Estado del UAV que transporta el gNodeB. Sin UavFollowerMobility no hay
    // estado: "UNKNOWN" (el gemelo lo trata como activo)
    std::string uavState = "UNKNOWN";
    if (auto *follower = dynamic_cast<UavFollowerMobility*>(mobility))
        uavState = follower->getUavState();
    int numConnected = connectedUes.size();

    // Formato JSON
    std::ostringstream json;
    json << std::fixed << std::setprecision(3);
    json << "{"
         << "\"type\":\"GNB_Report\","
         << "\"timestamp\":" << simTime().dbl() << ","
         << "\"gnb_id\":" << macNodeId << ","
         << "\"gnb_index\":" << gnbIndex << ","
         << "\"sumo_id\":\"" << sumoId << "\","
         << "\"uav_state\":\"" << uavState << "\","
         << "\"position\":{"
         << "\"x\":" << pos.x << ","
         << "\"y\":" << pos.y << ","
         << "\"z\":" << pos.z
         << "},"
         << "\"num_connected\":" << numConnected << ","
         << "\"connected_ues\":[";

    for (size_t i = 0; i < connectedUes.size(); i++) {
        json << connectedUes[i];
        if (i < connectedUes.size() - 1) json << ",";
    }

    json << "]}";

    std::string data = json.str();

    auto packet = new Packet("GnbInfoPacket");
    auto payload = makeShared<BytesChunk>(
        (const uint8_t*)data.c_str(),
        data.size()
    );
    packet->insertAtBack(payload);

    socket.send(packet);
    packetsSent++;
    emit(packetSentSignal, packetsSent);

    // Log detallado
    std::cout << "\n┌────────────────────────────────────────┐" << std::endl;
    std::cout << "[gNodeB[" << gnbIndex << "] / " << sumoId << "] Info Packet #"
              << packetsSent << " SENT" << std::endl;
    std::cout << "  Time       : " << std::fixed << std::setprecision(2)
              << simTime() << " s" << std::endl;
    std::cout << "  Position   : (" << (int)pos.x << ", " << (int)pos.y
              << ", " << (int)pos.z << ") m" << std::endl;
    std::cout << "  UAV state  : " << uavState << std::endl;
    std::cout << "  Connected  : " << numConnected << " UEs";

    if (numConnected > 0) {
        std::cout << " [";
        for (size_t i = 0; i < connectedUes.size(); i++) {
            std::cout << connectedUes[i];
            if (i < connectedUes.size() - 1) std::cout << ", ";
        }
        std::cout << "]";
    }
    std::cout << std::endl;
    std::cout << "└────────────────────────────────────────┘\n" << std::endl;

    EV_INFO << "GnbInfo sent: pos(" << pos.x << "," << pos.y << "," << pos.z << ")"
            << " UEs=" << numConnected << endl;
}

void GnbSender::handleStartOperation(LifecycleOperation *operation)
{
    EV_INFO << "GnbSender started" << endl;
}

void GnbSender::handleStopOperation(LifecycleOperation *operation)
{
    cancelEvent(selfMsg);
    socket.close();
    EV_INFO << "GnbSender stopped" << endl;
}

void GnbSender::handleCrashOperation(LifecycleOperation *operation)
{
    cancelEvent(selfMsg);
    socket.destroy();
}

void GnbSender::socketDataArrived(UdpSocket *socket, Packet *packet)
{
    delete packet;
}

void GnbSender::socketErrorArrived(UdpSocket *socket, Indication *indication)
{
    delete indication;
}

void GnbSender::socketClosed(UdpSocket *socket)
{
}

void GnbSender::finish()
{
    recordScalar("packetsSent", packetsSent);
    ApplicationBase::finish();
}
