// Aparcamiento: mientras el UAV esta en uno de los estados de 'parkStates'
// (por defecto STANDBY y CHARGING, es decir, posado en la base) o cuando
// desaparece de SUMO, el gNodeB se lleva a (parkX, parkY, parkZ), lejos del
// escenario. Asi un UAV que no esta en mision no capta coches ni interfiere.
// La posicion real del UAV se sigue guardando y GnbSender la reporta al
// gemelo (getUavPosition()).
//
// IMPORTANTE: a esa distancia el modelo de canal de Simu5G (UMa) se sale de
// su rango de validez (5 km). Hay que poner en el .ini
//     **.tolerateMaxDistViolation = true
// para que en lugar de dar error aplique una atenuacion enorme (sin senal).
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

        // Aparcamiento de los gNodeBs cuyo UAV no da servicio
        bool parkInactive = default(true);
        string parkStates = default("STANDBY CHARGING");   // separados por espacios
        double parkX @unit(m) = default(-20000m);
        double parkY @unit(m) = default(650m);
        double parkZ @unit(m) = default(25m);

        // Posicion hasta que el plugin crea el UAV en SUMO
        // (MobilityBase lee initialX/Y/Z automaticamente)
        double initialX @unit(m) = default(0m);
        double initialY @unit(m) = default(0m);
        double initialZ @unit(m) = default(defaultAltitude);

        @display("i=block/cogwheel");
}