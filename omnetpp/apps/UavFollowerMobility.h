#ifndef __DRONESTFG_UAVFOLLOWERMOBILITY_H_
#define __DRONESTFG_UAVFOLLOWERMOBILITY_H_

#include <string>
#include "inet/mobility/base/MovingMobilityBase.h"

namespace veins { class TraCIScenarioManager; }

/**
 * UavFollowerMobility: el gNodeB[i] sigue al UAV "uav<i>" de SUMO-UAV-Py.
 *
 *   SUMO-UAV-Py --moveToXY(x,y) + setParameter("altitude")--> SUMO
 *     --TraCI--> VeinsInetManager --> uav[k].mobility (x, y)
 *     --> gNodeB[i].mobility (esta clase): x, y del uav[k] y z del parametro
 *
 * x, y: se leen del mapa getManagedHosts() del manager (sin llamadas TraCI).
 * z: una consulta TraCI en handleSelfMessage(), es decir, UNA VEZ por
 *    updateInterval y como evento propio de OMNeT++.
 *
 *    IMPORTANTE: la consulta NO puede hacerse dentro de move(). INET invoca
 *    move() de forma perezosa cada vez que alguien pide la posicion, y eso
 *    puede ocurrir en mitad del paso de Veins, que en ese momento tiene su
 *    propia transaccion abierta con SUMO sobre el mismo socket TraCI. Meter
 *    ahi una peticion la intercala con la de Veins y rompe el protocolo.
 *    handleSelfMessage() es un evento aparte, asi que ahi es seguro.
 *
 * Si el UAV no existe (aun no creado, o eliminado por falta de bateria),
 * el gNodeB se queda en su ultima posicion.
 */
class UavFollowerMobility : public inet::MovingMobilityBase
{
  protected:
    std::string sumoId;
    veins::TraCIScenarioManager *manager = nullptr;
    bool presentInSumo = false;

    // Altura
    bool readAltitudeFromSumo = true;
    std::string altitudeParameter;
    double currentAltitude = 25.0;
    bool altitudeWarningShown = false;

  protected:
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleSelfMessage(omnetpp::cMessage *message) override;
    virtual void move() override;
    void readAltitude();

  public:
    const std::string& getSumoId() const { return sumoId; }
    bool isPresentInSumo() const { return presentInSumo; }
};

#endif


