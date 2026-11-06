// Modelo de canal para gNodeBs y relays en UAVs (ver UavChannelModel.h).
//
// Se activa en el .ini con:
//     **.cellularNic.LteChannelModelType = "dronestfg.apps.UavChannelModel"
//
// Clasificacion por altura real (z):
//     z <= groundMaxHeight                  -> suelo  (coches)
//     groundMaxHeight < z <= towerMaxHeight -> torre  (gNB terrestre)
//     z > towerMaxHeight                    -> aereo  (UAV)
// Por tanto los UAVs tienen que volar por encima de towerMaxHeight.

package dronestfg.apps;

import simu5g.stack.phy.ChannelModel.NRChannelModel_3GPP38_901;

simple UavChannelModel extends NRChannelModel_3GPP38_901
{
    parameters:
        @class(UavChannelModel);

        double groundMaxHeight @unit(m) = default(5m);
        double towerMaxHeight @unit(m) = default(35m);

        // Entorno para Al-Hourani: suburban | urban | denseurban | highrise
        string airGroundEnvironment = default("urban");

        // Rejilla para volver a sortear la LoS (como la correlation_distance)
        double losGridSize @unit(m) = default(50m);
        int losSeed = default(0);

        // A partir de esta distancia 2D no hay senal (gNodeBs aparcados)
        double maxLinkDistance @unit(m) = default(5000m);
        double outOfRangeAttenuation = default(300);   // dB

        // Log de comprobacion (lo lee check_channel_model.py). Vacio = desactivado.
        // Funciona igual en Qtenv y en Cmdenv. Todos los modelos de canal
        // escriben en el mismo fichero.
        string attenuationLogFile = default("");
        // Como mucho una linea por enlace (UE) y direccion cada este tiempo,
        // para que el fichero no crezca sin control (se llama en cada TTI)
        double attenuationLogInterval @unit(s) = default(1s);
}
