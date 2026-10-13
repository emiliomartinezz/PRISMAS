//
// CarUE.ned
// Coche de SUMO convertido en UE 5G NR.
//
// Lo instancia VeinsInetManager en el vector car[] cuando un vehiculo de
// tipo car_ue entra en SUMO y lo destruye cuando sale. Hereda de MobileUE
// (NRUe + estadisticas de Lucas) y replica lo que hace simu5g.nodes.cars.NRCar
// (sin necesitar la feature "Simu5G Cars"):
//   - movilidad dirigida por SUMO (VeinsInetMobility)
//   - IP autoconfigurada, porque el Ipv4NetworkConfigurator global solo
//     actua en la inicializacion y estos nodos se crean despues
//

package dronestfg.nodes;

module CarUE extends MobileUE
{
    parameters:
        @display("i=device/pocketpc;is=vs");

        mobility.typename = "org.car2x.veins.subprojects.veins_inet.VeinsInetMobility";

        *.routingTableModule = default(absPath(".ipv4.routingTable"));
        ipv4.configurator.typename = "HostAutoConfigurator";
        ipv4.configurator.interfaces = "cellular lo0";
}
