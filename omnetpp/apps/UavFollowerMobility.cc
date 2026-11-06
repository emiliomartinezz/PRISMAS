//
// UavFollowerMobility.cc
//

#include "UavFollowerMobility.h"

#include <iostream>
#include <sstream>
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using namespace omnetpp;

Define_Module(UavFollowerMobility);

void UavFollowerMobility::initialize(int stage)
{
    MovingMobilityBase::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        readAltitudeFromSumo = par("readAltitudeFromSumo").boolValue();
        altitudeParameter = par("altitudeParameter").stdstringValue();
        currentAltitude = par("defaultAltitude").doubleValue();
        readStateFromSumo = par("readStateFromSumo").boolValue();
        stateParameter = par("stateParameter").stdstringValue();

        parkInactive = par("parkInactive").boolValue();
        std::istringstream states(par("parkStates").stdstringValue());
        for (std::string s; states >> s; )
            parkStates.insert(s);
        parkPosition = inet::Coord(par("parkX").doubleValue(),
                                   par("parkY").doubleValue(),
                                   par("parkZ").doubleValue());

        sumoId = par("sumoId").stdstringValue();
        if (sumoId.empty()) {
            cModule *host = getParentModule();   // gNodeB[i]
            if (!host->isVector())
                throw cRuntimeError("UavFollowerMobility: sumoId vacio y '%s' no es un vector",
                                    host->getFullPath().c_str());
            sumoId = "uav" + std::to_string(host->getIndex());
        }

        const char *mgrName = par("managerModule");
        cModule *mgr = getSimulation()->getSystemModule()->getSubmodule(mgrName);
        manager = dynamic_cast<veins::TraCIScenarioManager *>(mgr);
        if (manager == nullptr)
            throw cRuntimeError("UavFollowerMobility: no hay un TraCIScenarioManager llamado '%s' en la red",
                                mgrName);

        // Visibles en el inspector de Qtenv
        WATCH(uavState);
        WATCH(presentInSumo);
        WATCH(currentAltitude);
        WATCH(parked);

        std::cout << "[UavFollowerMobility] " << getParentModule()->getFullName()
                  << " seguira al vehiculo SUMO '" << sumoId << "'"
                  << (readAltitudeFromSumo ? " (altura: parametro TraCI '" + altitudeParameter + "')"
                                           : " (altura fija)")
                  << (parkInactive ? "; se aparca si el UAV no esta en mision" : "")
                  << std::endl;
    }
}

void UavFollowerMobility::handleSelfMessage(cMessage *message)
{
    // Unico punto seguro para hablar con SUMO: es un evento propio, no una
    // llamada perezosa desde dentro del paso de Veins (ver cabecera).
    if (readAltitudeFromSumo && presentInSumo)
        readAltitude();
    if (readStateFromSumo && presentInSumo)
        readState();

    MovingMobilityBase::handleSelfMessage(message);
}

void UavFollowerMobility::readAltitude()
{
    veins::TraCICommandInterface *traci = manager->getCommandInterface();
    if (traci == nullptr)
        return;

    // SUMO devuelve "" si el vehiculo no tiene ese parametro
    std::string value;
    traci->vehicle(sumoId).getParameter(altitudeParameter, value);

    if (value.empty()) {
        if (!altitudeWarningShown) {
            std::cout << "[UavFollowerMobility] AVISO: '" << sumoId << "' devuelve vacio en el parametro '"
                      << altitudeParameter << "'. Se usa z=" << currentAltitude
                      << " m. Comprueba que main_.py llama a publish_altitude() y que"
                         " 'OMNeT Mode' esta a true en config.json." << std::endl;
            altitudeWarningShown = true;
        }
        return;
    }

    try {
        currentAltitude = std::stod(value);
    }
    catch (const std::exception&) {
        if (!altitudeWarningShown) {
            std::cout << "[UavFollowerMobility] AVISO: valor de altura no numerico '" << value
                      << "' en '" << sumoId << "'" << std::endl;
            altitudeWarningShown = true;
        }
    }
}

void UavFollowerMobility::readState()
{
    veins::TraCICommandInterface *traci = manager->getCommandInterface();
    if (traci == nullptr)
        return;

    // SUMO devuelve "" si el plugin no publica el estado: se deja "UNKNOWN"
    std::string value;
    traci->vehicle(sumoId).getParameter(stateParameter, value);
    if (value.empty() || value == uavState)
        return;

    std::cout << "[UavFollowerMobility] '" << sumoId << "' estado " << uavState
              << " -> " << value << " en t=" << simTime() << "s" << std::endl;
    uavState = value;
}

void UavFollowerMobility::setParked(bool park)
{
    if (park) {
        lastPosition = parkPosition;
        lastVelocity = inet::Coord::ZERO;
    }
    if (park == parked)
        return;
    parked = park;
    std::cout << "[UavFollowerMobility] " << getParentModule()->getFullName()
              << (park ? " aparcado" : " desaparcado")
              << " ('" << sumoId << "' en " << getUavState() << ") en t=" << simTime() << "s"
              << std::endl;
}

void UavFollowerMobility::move()
{
    // Mapa id SUMO -> modulo gestionado por Veins
    const auto& hosts = manager->getManagedHosts();
    auto it = hosts.find(sumoId);
    cModule *uavModule = (it != hosts.end()) ? it->second : nullptr;

    if (uavModule == nullptr) {
        if (presentInSumo) {
            std::cout << "[UavFollowerMobility] '" << sumoId << "' ha desaparecido de SUMO en t="
                      << simTime() << "s; " << getParentModule()->getFullName()
                      << (parkInactive ? " se aparca" : " se queda en la ultima posicion")
                      << std::endl;
        }
        presentInSumo = false;
        lastVelocity = inet::Coord::ZERO;
        // Si el UAV ya existio y se ha ido, se aparca. Si aun no ha aparecido
        // nunca, el gNodeB se queda en su posicion inicial.
        if (parkInactive && hasUavPosition)
            setParked(true);
        return;
    }

    auto *src = dynamic_cast<inet::IMobility *>(uavModule->getSubmodule("mobility"));
    if (src == nullptr)
        throw cRuntimeError("UavFollowerMobility: '%s' no tiene submodulo 'mobility' (IMobility)",
                            uavModule->getFullPath().c_str());

    if (!presentInSumo) {
        presentInSumo = true;
        std::cout << "[UavFollowerMobility] " << getParentModule()->getFullName()
                  << " enganchado a '" << sumoId << "' (" << uavModule->getFullName()
                  << ") en t=" << simTime() << "s" << std::endl;
    }

    // Posicion real del UAV: se guarda siempre, este o no aparcado el gNodeB
    const inet::Coord& p = src->getCurrentPosition();
    uavPosition = inet::Coord(p.x, p.y, currentAltitude);
    hasUavPosition = true;

    if (parkInactive && parkStates.count(uavState) > 0) {
        setParked(true);
        return;
    }

    setParked(false);
    lastPosition = uavPosition;
    lastVelocity = src->getCurrentVelocity();
    lastVelocity.z = 0;
}

