// UESender.ned
// Definición NED para la aplicación UESender

package dronestfg.apps;

simple UESender like inet.applications.contract.IApp
{
    parameters:
        int localPort = default(-1);  // Puerto local (aleatorio si -1)
        int destPort = default(5000); // Puerto destino en el servidor
        string destAddress;           // Dirección IP del servidor
        double sendInterval @unit(s) = default(1s); // Intervalo entre envíos
        double startTime @unit(s) = default(1s);    // Tiempo de inicio
        
        // Estadísticas
        @signal[packetSent](type=long);
        @statistic[packetsSent](title="Packets Sent"; source=packetSent; record=count,vector;);
        
        @display("i=block/source");
        
    gates:
        input socketIn @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}