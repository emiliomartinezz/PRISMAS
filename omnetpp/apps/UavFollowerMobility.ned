//
// UavFollowerMobility.ned
// Movilidad del gNodeB embarcado en un UAV de SUMO-UAV-Py.
//
// No calcula trayectorias: copia la posicion del UAV "uav<i>" de SUMO.
//  - x, y: del modulo uav[k] que crea Veins (ya en coordenadas OMNeT++).
//  - z: SUMO es 2D, asi que el plugin publica la altura como parametro
//    generico del vehiculo ("altitude", ver plugin_patch) y este modulo la
//    lee por TraCI una vez por updateInterval. Si el parametro no existe,
//    usa 'defaultAltitude'.
//

package dronestfg.apps;

import inet.mobility.base.MovingMobilityBase;

simple UavFollowerMobility extends MovingMobilityBase
{
    parameters:
        @class(UavFollowerMobility);

        // Id del vehiculo en SUMO. Vacio -> "uav<indice del gNodeB>"
        string sumoId = default("");

        // Nombre del VeinsInetManager en la red
        string managerModule = default("manager");

        // Altura: la del plugin (parametro TraCI) o, si no hay, esta
        bool readAltitudeFromSumo = default(true);
        string altitudeParameter = default("altitude");
        double defaultAltitude @unit(m) = default(25m);

        // Estado del UAV (MISSION, RETURNING...) publicado por el plugin con
        // publish_state(); se incluye en el GNB_Report para el gemelo
        bool readStateFromSumo = default(true);
        string stateParameter = default("uav_state");

        // Posicion hasta que el plugin crea el UAV en SUMO
        // (MobilityBase lee initialX/Y/Z automaticamente)
        double initialX @unit(m) = default(0m);
        double initialY @unit(m) = default(0m);
        double initialZ @unit(m) = default(defaultAltitude);

        @display("i=block/cogwheel");
}