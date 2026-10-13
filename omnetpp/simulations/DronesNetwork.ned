//
// DronesNetwork.ned  -  Paso 1: coches = UEs 5G, UAVs = gNodeBs
//
// Parte de la red original de dronesTFG (VeinsInetManager + visualizador)
// y le anade la infraestructura 5G de la red de Lucas (Simu5G).
//
//  - gNodeB[numGnb]: estaticos en el NED (backhaul, X2 e IP se resuelven en
//    la inicializacion) pero su movilidad (UavFollowerMobility) copia la
//    posicion del UAV correspondiente de SUMO.
//  - uav[]: los crea Veins para cada vehiculo uav_type (modulo UAV de
//    siempre: solo tiene VeinsInetMobility). Son las "anclas" que siguen
//    los gNodeB.
//  - car[]: los crea Veins para cada vehiculo car_ue (CarUE = UE 5G NR).
//

package dronestfg.simulations;

import org.car2x.veins.subprojects.veins_inet.VeinsInetManager;
import org.car2x.veins.visualizer.roads.RoadsCanvasVisualizer;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth10G;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import simu5g.common.binder.Binder;
import simu5g.common.carrierAggregation.CarrierAggregation;
import simu5g.nodes.Upf;
import simu5g.world.radio.LteChannelControl;
import dronestfg.nodes.UAV;
import dronestfg.nodes.MobileGnb;
import dronestfg.nodes.CarUE;


network DronesNetwork
{
    parameters:
        double playgroundSizeX @unit(m);
        double playgroundSizeY @unit(m);
        double playgroundSizeZ @unit(m);

        // Debe coincidir con "Number of UAVs" del config.json del plugin
        int numGnb = default(3);

        @display("bgb=1500,1300");

    submodules:
        // ---------------- Veins (ya estaba) ----------------
        manager: VeinsInetManager {
            parameters:
                @display("p=50,50");
        }
        // Antes se llamaba "roaddsVisualizer" y por eso las lineas
        // *.roadsVisualizer.* del ini no tenian efecto
        roadsVisualizer: RoadsCanvasVisualizer {
            parameters:
                @display("p=100,100");
        }

        // ---------------- Simu5G (de Lucas) ----------------
        channelControl: LteChannelControl {
            @display("p=-300,50;is=s");
        }
        binder: Binder {
            @display("p=-300,125;is=s");
        }
        carrierAggregation: CarrierAggregation {
            @display("p=-300,200;is=s");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=-300,275;is=s");
        }
        upf: Upf {
            @display("p=-300,450");
        }
        router: Router {
            @display("p=-300,550");
        }
        server: StandardHost {
            @display("p=-300,650;i=device/server");
        }

        gNodeB[numGnb]: MobileGnb {
            @display("p=300,200");
        }

        // ---------------- Vectores dinamicos (los rellena Veins) ----------------
        uav[0]: UAV {
            @display("p=200,200");
        }
        car[0]: CarUE {
            parameters:
                @display("p=-1000,-1000;is=vs");
        }

    connections allowunconnected:
        server.pppg++ <--> Eth10G <--> router.pppg++;
        router.pppg++ <--> Eth10G <--> upf.filterGate;

        // Mismo orden que en la red de Lucas
        for i=0..numGnb-1 {
            upf.pppg++ <--> Eth10G <--> gNodeB[i].ppp;
        }
        for i=0..numGnb-1 {
            router.pppg++ <--> Eth10G <--> gNodeB[i].pppMEHost;
        }

        // X2 en malla completa. Con numGnb=3 son exactamente las mismas
        // conexiones y en el mismo orden que las de Lucas, asi que sus
        // lineas x2App del ini siguen valiendo.
        for i=0..numGnb-1, for j=i+1..numGnb-1 {
            gNodeB[i].x2++ <--> Eth10G <--> gNodeB[j].x2++;
        }
}
