package dronestfg.nodes;

import simu5g.nodes.NR.gNodeB;

module MobileGnb extends gNodeB
{
    parameters:
        @display("i=device/dron");
        
        // Número de aplicaciones (añadimos CoverageReporter)
        numApps = default(1);
        
        // Estadísticas de posición
        @statistic[posX](source="xCoord(mobilityPos(mobilityStateChanged))"; record=vector;);
        @statistic[posY](source="yCoord(mobilityPos(mobilityStateChanged))"; record=vector;);
        
        // Estadísticas de cobertura
        @statistic[connectedUes](title="Connected UEs"; source="numConnectedUes"; record=vector,stats;);
        @statistic[coverageArea](title="Coverage Area Estimate"; source="coverageAreaEstimate"; record=vector,stats;);
}