// UavChannelModel.h
// Modelo de canal para gNodeBs y relays embarcados en UAVs.
//
// Hereda de NRChannelModel_3GPP38_901 (el que usan ahora gNodeBs y coches) y
// solo cambia el calculo de perdidas y de la visibilidad directa (LoS). El
// resto (fading, interferencia, SINR, CQI...) sigue siendo el de Simu5G.
//
// El tipo de enlace se decide con la altura REAL (z) de los dos extremos:
//   - suelo  <-> torre   : UMa de 3GPP TR 38.901 (el de Simu5G, como ahora)
//   - aereo  <-> suelo   : Al-Hourani (estacion base en UAV -> coche)
//   - aereo  <-> torre   : UMa-AV de 3GPP TR 36.777 (gNB terrestre -> MT del relay)
//   - aereo  <-> aereo   : espacio libre con LoS (UAV <-> UAV)
//
// Ademas corrige que en NRChannelModel la LoS se calcule una sola vez por UE y
// no cambie nunca: aqui se decide POR ENLACE y se vuelve a sortear cuando
// cualquiera de los dos extremos cambia de celda en una rejilla de losGridSize
// metros (consistencia espacial, reproducible con losSeed).

#ifndef __DRONESTFG_UAVCHANNELMODEL_H
#define __DRONESTFG_UAVCHANNELMODEL_H

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include "stack/phy/ChannelModel/NRChannelModel_3GPP38_901.h"

class UavChannelModel : public NRChannelModel_3GPP38_901
{
  protected:
    enum NodeClass { GROUND, TOWER, AERIAL };
    enum LinkType { LINK_TERRESTRIAL, LINK_AIR_GROUND, LINK_AIR_TOWER, LINK_AIR_AIR };

    // Clasificacion por altura (m)
    double groundMaxHeight_ = 5.0;    // z <= esto -> suelo (coches, peatones)
    double towerMaxHeight_ = 35.0;    // z <= esto -> torre; por encima -> aereo

    // Enlaces fuera de rango (p. ej. gNodeBs aparcados a -20 km)
    double maxLinkDistance_ = 5000.0;        // m (distancia 2D)
    double outOfRangeAttenuation_ = 300.0;   // dB

    // Sorteo de LoS con consistencia espacial
    double losGridSize_ = 50.0;   // m
    uint64_t losSeed_ = 0;

    // Parametros de Al-Hourani segun el entorno
    double ahA_ = 9.61;
    double ahB_ = 0.16;
    double ahEtaLos_ = 1.0;    // dB de perdida adicional en LoS
    double ahEtaNlos_ = 20.0;  // dB de perdida adicional en NLoS

    // Log de comprobacion a fichero (independiente de Qtenv/Cmdenv)
    std::string attLogFile_;               // vacio = desactivado
    omnetpp::simtime_t attLogInterval_;    // como mucho una linea por enlace y direccion
    std::map<std::pair<MacNodeId, int>, omnetpp::simtime_t> attLogLast_;

  protected:
    virtual void initialize(int stage) override;
    virtual void finish() override;

    void writeAttLog(MacNodeId nodeId, Direction dir, int link, bool los, double d3D,
                     double d2D, double zA, double zB, double att);

    NodeClass classify(double z) const;
    LinkType linkType(NodeClass a, NodeClass b) const;

    // Perdidas (dB). Distancias en metros; frecuencia la de la portadora.
    double freeSpacePathLoss(double d3D) const;
    double pathLossAirGround(double d3D, bool los) const;
    double pathLossUMaAV(double d3D, double hUav, bool los) const;

    // Probabilidades de LoS
    double losProbUMa(double d2D) const;                    // TR 38.901, UE en el suelo
    double losProbAirGround(double d2D, double dh) const;   // Al-Hourani
    double losProbUMaAV(double d2D, double hUav) const;     // TR 36.777

    // Sorteo determinista y simetrico (mismo resultado en UL y DL)
    bool drawLos(const inet::Coord& a, const inet::Coord& b, double p) const;

  public:
    virtual double getAttenuation(MacNodeId nodeId, Direction dir, inet::Coord coord, bool cqiDl) override;
};

#endif // __DRONESTFG_UAVCHANNELMODEL_H
