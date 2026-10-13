// UESender.cc
// Aplicación que envía posición y métricas de red del UE
// Formato: JSON {"type":"UE_Report","ue_id":...,...}
//
// Cambios paso 1 (coches = UEs creados/destruidos por Veins):
//   - La suscripción a measuredSinrUl se hace en el cellularNic de ESTE UE
//     en lugar de en el systemModule. nrChannelModel cuelga de cellularNic,
//     así que la señal llega igual, pero cada UE ya no recibe las emisiones
//     de todos los demás (con N coches, la suscripción global cuesta O(N^2)).
//   - Desuscripción explícita en finish(): Veins llama a callFinish() antes
//     de borrar el coche cuando sale de SUMO.

#include "UESender.h"

#include "inet/common/packet/Packet.h"
#include "inet/common/packet/chunk/BytesChunk.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/ModuleAccess.h"
#include "common/binder/Binder.h"
#include <sstream>
#include <iomanip>

Define_Module(UESender);

UESender::~UESender()
{
    cancelAndDelete(selfMsg);
}

void UESender::initialize(int stage)
{
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        localPort = par("localPort");
        destPort = par("destPort");
        sendInterval = par("sendInterval");
        startTime = par("startTime");

        packetsSent = 0;
        packetSentSignal = registerSignal("packetSent");

        selfMsg = new cMessage("sendTimer");

        lastSinr = -999.0;
        lastMasterNodeId = -1;

        // Modulo de cellularNic de ESTE UE. Se usa para filtrar en receiveSignal:
        // como la suscripcion a la senal de SINR es GLOBAL (en el systemModule),
        // cada UESender recibe las emisiones de todos los UEs y debe quedarse SOLO
        // con las que provienen de su propio cellularNic.
        myCellularNic = nullptr;

        EV_INFO << "UESender initialized - will start at " << startTime << endl;
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        mobility = check_and_cast<IMobility *>(
            getParentModule()->getSubmodule("mobility")
        );

        socket.setOutputGate(gate("socketOut"));
        socket.setCallback(this);

        const char *destAddrStr = par("destAddress");
        destAddr = L3AddressResolver().resolve(destAddrStr);

        if (localPort != -1)
            socket.bind(localPort);

        socket.connect(destAddr, destPort);

        EV_INFO << "Socket configured - dest: " << destAddr
                << ":" << destPort << endl;

        // ===== OBTENER UE ID =====
        cModule *ueModule = getParentModule();

        // Obtener nrMacNodeId como identificador único del UE
        if (ueModule->hasPar("nrMacNodeId")) {
            ueNodeId = ueModule->par("nrMacNodeId").intValue();
        }
        if (ueNodeId <= 0 && ueModule->hasPar("macNodeId")) {
            ueNodeId = ueModule->par("macNodeId").intValue();
        }

        // Índice del array car[X] (lo asigna Veins al crear el coche)
        if (ueModule->isVector())
            ueIndex = ueModule->getIndex();

        std::cout << "[UESender] UE[" << ueIndex << "] initialized"
                  << " (nrMacNodeId=" << ueNodeId << ")" << std::endl;

        // ===== ACCESO A MÓDULOS SIMU5G =====

        cellularNic = ueModule->getSubmodule("cellularNic");

        if (cellularNic) {
            EV_INFO << "CellularNic found: " << cellularNic->getFullPath() << endl;

            // Guardamos la referencia al cellularNic de ESTE UE para el filtro
            // por origen de receiveSignal.
            myCellularNic = cellularNic;

            nrPhy = cellularNic->getSubmodule("nrPhy");
            if (!nrPhy)
                nrPhy = cellularNic->getSubmodule("phy");

            if (nrPhy) {
                EV_INFO << "PHY module found: " << nrPhy->getFullPath() << endl;
            }
            else {
                EV_WARN << "PHY module not found" << endl;
                std::cout << "[UESender] WARNING: PHY module not found!" << std::endl;
            }

            // ================================================================
            // SUSCRIPCION A measuredSinrUl  (GLOBAL + FILTRO POR ORIGEN)
            // ================================================================
            // La senal measuredSinrUl la EMITE el modulo nrChannelModel (dentro
            // de cellularNic), no nrPhy. En OMNeT una suscripcion solo recibe las
            // senales emitidas por el modulo suscrito y las que burbujean desde
            // sus DESCENDIENTES. Como nrPhy y nrChannelModel son hermanos (ambos
            // cuelgan de cellularNic), suscribirse a nrPhy NO captura la senal:
            // por eso el SINR salia -999 siempre.
            //
            // Solucion robusta: suscribirse en el systemModule (la raiz), que SI
            // es ancestro de nrChannelModel y por tanto recibe la senal de todos
            // los UEs. Para que cada UE se quede solo con SU medida, receiveSignal
            // filtra por origen comprobando que el emisor cuelga de ESTE
            // cellularNic (ver receiveSignal). Asi cada UE obtiene su SINR de forma
            // independiente, igual que la posicion.
            // PASO 1: basta con suscribirse en cellularNic, que es ANCESTRO de
            // nrChannelModel (la señal burbujea hacia arriba hasta él) y solo
            // recibe las emisiones de este UE. Se mantiene el filtro por
            // origen de receiveSignal como red de seguridad.
            sinrSignal = registerSignal("measuredSinrUl");
            sinrSource = cellularNic;
            sinrSource->subscribe(sinrSignal, this);

            std::cout << "[UESender] UE[" << ueIndex
                      << "] subscribed to measuredSinrUl en "
                      << sinrSource->getFullPath() << std::endl;
        }
        else {
            EV_WARN << "CellularNic not found!" << endl;
            std::cout << "[UESender] WARNING: CellularNic not found!" << std::endl;
        }

        binder = check_and_cast<Binder*>(
            getSimulation()->getSystemModule()->getSubmodule("binder")
        );

        if (binder) {
            std::cout << "[UESender] Binder found: " << binder->getFullPath() << std::endl;
            EV_INFO << "Binder module found" << endl;
        } else {
            std::cout << "[UESender] ERROR: Binder not found!" << std::endl;
            throw cRuntimeError("UESender: Binder module is required but not found");
        }

        scheduleAt(simTime() + startTime, selfMsg);

        std::cout << "[UESender] First send at t="
                  << (simTime() + startTime) << "s" << std::endl;
    }
}

void UESender::receiveSignal(cComponent *source, simsignal_t signalID,
                                    double value, cObject *details)
{
    // Solo interesa la senal de SINR UL.
    if (signalID != sinrSignal)
        return;

    // ------------------------------------------------------------------
    // FILTRO POR ORIGEN
    // ------------------------------------------------------------------
    // La suscripcion es global (systemModule), por lo que aqui llegan las
    // emisiones de measuredSinrUl de TODOS los UEs. Nos quedamos unicamente con
    // las que provienen del cellularNic de ESTE UE: recorremos la cadena de
    // ancestros del modulo emisor y comprobamos que pasa por nuestro
    // myCellularNic. Asi cada UE captura su propio SINR de forma independiente y
    // se evita que el valor de un UE contamine al de otro (causa de que antes
    // todos reportaran el mismo SINR).
    if (myCellularNic != nullptr) {
        cModule *srcModule = dynamic_cast<cModule *>(source);
        bool belongsToThisUe = false;
        for (cModule *m = srcModule; m != nullptr; m = m->getParentModule()) {
            if (m == myCellularNic) {
                belongsToThisUe = true;
                break;
            }
        }
        if (!belongsToThisUe)
            return;   // Emision de otro UE: se ignora.
    }

    lastSinr = value;
}

int UESender::getMacNodeIdFromBinder()
{
    if (!binder) {
        std::cout << "[UESender] ERROR: Binder reference is null" << std::endl;
        return -1;
    }

    cModule *ueModule = getParentModule();
    if (!ueModule) {
        return -1;
    }

    try {
        MacNodeId ueId = ueNodeId;

        if (ueId <= 0) {
            return -1;
        }

        MacNodeId masterNodeId = binder->getNextHop(ueId);

        std::cout << "[UESender] UE[" << ueIndex << "] ID: " << ueId
                  << " → Master Node: " << masterNodeId << std::endl;

        return masterNodeId;

    } catch (const std::exception& e) {
        EV_ERROR << "Exception in getMacNodeIdFromBinder: " << e.what() << endl;
        std::cout << "[UESender] Exception: " << e.what() << std::endl;
    }

    return -1;
}

int UESender::getCurrentMasterNodeId()
{
    return getMacNodeIdFromBinder();
}

void UESender::handleMessageWhenUp(cMessage *msg)
{
    if (msg == selfMsg) {
        sendUE_Report();
        scheduleAt(simTime() + sendInterval, selfMsg);
    }
    else {
        socket.processMessage(msg);
    }
}

void UESender::sendUE_Report()
{
    Coord pos = mobility->getCurrentPosition();

    lastMasterNodeId = getCurrentMasterNodeId();

    // Formato JSON con ue_id incluido desde origen
    std::ostringstream json;
    json << std::fixed << std::setprecision(3);
    json << "{"
         << "\"type\":\"UE_Report\","
         << "\"ue_id\":" << ueNodeId << ","
         << "\"ue_index\":" << ueIndex << ","
         << "\"timestamp\":" << simTime().dbl() << ","
         << "\"position\":{"
         << "\"x\":" << pos.x << ","
         << "\"y\":" << pos.y << ","
         << "\"z\":" << pos.z
         << "},"
         << "\"network\":{"
         << "\"sinr\":" << lastSinr << ","
         << "\"master_id\":" << lastMasterNodeId
         << "}"
         << "}";

    std::string data = json.str();

    auto packet = new Packet("UE_ReportPacket");
    auto payload = makeShared<BytesChunk>(
        (const uint8_t*)data.c_str(),
        data.size()
    );
    packet->insertAtBack(payload);

    socket.send(packet);
    packetsSent++;
    emit(packetSentSignal, packetsSent);

    std::cout << "\n┌────────────────────────────────────────┐" << std::endl;
    std::cout << "[UE[" << ueIndex << "]] Packet #" << packetsSent << " SENT" << std::endl;
    std::cout << "  Time       : " << std::fixed << std::setprecision(2) << simTime() << " s" << std::endl;
    std::cout << "  Position   : (" << (int)pos.x << ", " << (int)pos.y << ", " << (int)pos.z << ") m" << std::endl;
    std::cout << "  SINR       : " << std::fixed << std::setprecision(2) << lastSinr << " dB" << std::endl;
    std::cout << "  Connected  : gNodeB[" << (lastMasterNodeId - 1) << "] (macNodeId=" << lastMasterNodeId << ")" << std::endl;
    std::cout << "└────────────────────────────────────────┘\n" << std::endl;

    EV_INFO << "UE_Report sent: (" << pos.x << "," << pos.y << "," << pos.z << ")"
            << " SINR=" << lastSinr << " macNodeId=" << lastMasterNodeId << endl;
}

void UESender::handleStartOperation(LifecycleOperation *operation)
{
    EV_INFO << "UESender started" << endl;
}

void UESender::handleStopOperation(LifecycleOperation *operation)
{
    cancelEvent(selfMsg);
    socket.close();
}

void UESender::handleCrashOperation(LifecycleOperation *operation)
{
    cancelEvent(selfMsg);
    socket.destroy();
}

void UESender::socketDataArrived(UdpSocket *socket, Packet *packet)
{
    delete packet;
}

void UESender::socketErrorArrived(UdpSocket *socket, Indication *indication)
{
    delete indication;
}

void UESender::socketClosed(UdpSocket *socket)
{
}

void UESender::finish()
{
    // Desuscribirse antes de que Veins borre el coche
    if (sinrSource != nullptr && sinrSource->isSubscribed(sinrSignal, this))
        sinrSource->unsubscribe(sinrSignal, this);
    sinrSource = nullptr;

    recordScalar("packetsSent", packetsSent);
    ApplicationBase::finish();
}
