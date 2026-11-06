#ifndef __DRONESTFG_UAVFOLLOWERMOBILITY_H_
#define __DRONESTFG_UAVFOLLOWERMOBILITY_H_

#include <set>
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
 * estado: parametro TraCI "uav_state", leido junto a la altura.
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
 * Aparcamiento: si el UAV esta en uno de los estados de 'parkStates'
 * (STANDBY, CHARGING) o ha desaparecido de SUMO, el gNodeB se coloca en la
 * posicion de aparcamiento, lejos del escenario, para que no capte coches ni
 * interfiera. La posicion real del UAV se guarda aparte (getUavPosition()).
 *
 * Antes de que el UAV aparezca por primera vez en SUMO, el gNodeB se queda en
 * su posicion inicial (initialX/Y/Z).
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

    // Estado del UAV (MISSION, RETURNING, CHARGING, STANDBY) publicado por el plugin
    bool readStateFromSumo = true;
    std::string stateParameter;
    std::string uavState = "UNKNOWN";

    // Aparcamiento
    bool parkInactive = true;
    std::set<std::string> parkStates;
    inet::Coord parkPosition;
    bool parked = false;

    // Posicion real del UAV en el escenario (aunque el gNodeB este aparcado)
    inet::Coord uavPosition;
    bool hasUavPosition = false;

  protected:
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleSelfMessage(omnetpp::cMessage *message) override;
    virtual void move() override;
    void readAltitude();
    void readState();
    void setParked(bool park);

  public:
    const std::string& getSumoId() const { return sumoId; }
    bool isPresentInSumo() const { return presentInSumo; }
    bool isParked() const { return parked; }
    // "ABSENT" si el UAV no esta en SUMO; "UNKNOWN" si el plugin no publica el estado
    std::string getUavState() const { return presentInSumo ? uavState : "ABSENT"; }
    // Posicion real del UAV (la de SUMO), aunque el gNodeB este aparcado.
    // Si aun no se conoce, la posicion actual del gNodeB.
    inet::Coord getUavPosition() { return hasUavPosition ? uavPosition : getCurrentPosition(); }
};

#endif

