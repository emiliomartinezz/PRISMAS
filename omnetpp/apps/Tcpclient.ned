

package dronestfg.apps;


import inet.applications.tcpapp.TcpServerHostApp;

// Reenvia la telemetria al broker MQTT (ver Tcpclient.cc).
simple TcpClient extends TcpServerHostApp
{
    parameters:
        @class(TcpClient);
        string brokerHost = default("127.0.0.1");   // IP del broker MQTT
        int brokerPort = default(1883);
        string topicPrefix = default("prismas");
        double connectTimeout @unit(s) = default(2s);   // espera inicial al broker
        @display("i=block/app");
}