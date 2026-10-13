// GnbSender.ned
// Definicion NED para la aplicacion GnbSender

package dronestfg.apps;

simple GnbSender like inet.applications.contract.IApp
{
    parameters:
        int localPort = default(-1);  // Puerto local (aleatorio si -1)
        int destPort = default(5000); // Puerto destino en el servidor
        string destAddress;           // Direccion IP del servidor
        double sendInterval @unit(s) = default(2s); // Intervalo entre envios
        double startTime @unit(s) = default(1s);    // Tiempo de inicio

        // NUEVO: vector de la red donde estan los UEs. Con Veins los crea
        // dinamicamente VeinsInetManager en "car[]".
        string ueVectorName = default("car");

        // Estadisticas
        @signal[packetSent](type=long);
        @statistic[packetsSent](title="Packets Sent"; source=packetSent; record=count,vector;);

        @display("i=block/source");

    gates:
        input socketIn @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}
