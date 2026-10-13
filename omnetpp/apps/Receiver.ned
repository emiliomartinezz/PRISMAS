package dronestfg.apps;

import inet.applications.contract.IApp;

simple Receiver like inet.applications.contract.IApp
{
    parameters:
        int localPort = default(5001);
        
        // NUEVO: Referencia al módulo TcpClient para reenvío
        string tcpClientModule = default("^.app[1]");

        @signal[packetReceived](type=long);
        @statistic[packetsReceived](source=packetReceived; record=count,vector);

        @display("i=block/sink");

    gates:
        input socketIn @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}