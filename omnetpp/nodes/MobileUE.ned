package dronestfg.nodes;

import simu5g.nodes.NR.NRUe;

module MobileUE extends NRUe
{ 
    parameters:
        @display("i=device/pocketpc");
        
        // NO sobreescribir macCellId, masterId, etc. aquí
        
        // Estadísticas de posición del UE
        @statistic[ueX](source="xCoord(mobilityPos(mobilityStateChanged))"; record=vector;);
        @statistic[ueY](source="yCoord(mobilityPos(mobilityStateChanged))"; record=vector;);
        
        // Estadísticas de handover desde la perspectiva del UE
        @statistic[handoverExecuted](title="Handover Executed"; source="handoverInitiated"; record=count,vector;);
        @statistic[handoverStarted](title="Handover Started"; source="handoverStarted"; record=count,vector;);
        @statistic[servingCell](title="Serving Cell ID"; source="servingCellId"; record=vector;);
        
        // CRÍTICO: Habilitar métricas de calidad de señal
        @statistic[rsrp](title="RSRP"; source="rsrp"; record=vector,stats;);
        @statistic[rsrq](title="RSRQ"; source="rsrq"; record=vector,stats;);
        @statistic[sinr](title="SINR"; source="sinr"; record=vector,stats;);
        @statistic[averageCqiDl](title="Average CQI DL"; source="averageCqiDl"; record=vector,stats;);
        @statistic[measuredSinrDl](title="Measured SINR DL"; source="measuredSinrDl"; record=vector,stats;);
}
