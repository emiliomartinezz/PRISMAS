

package dronestfg.apps;


import inet.applications.tcpapp.TcpServerHostApp;

simple TcpClient extends TcpServerHostApp
{
    parameters:
        @class(TcpClient);
        @display("i=block/app");
}