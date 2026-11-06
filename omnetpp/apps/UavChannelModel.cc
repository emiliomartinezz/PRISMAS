// UavChannelModel.cc
// Ver UavChannelModel.h para el resumen de modelos y la clasificacion.

#include "UavChannelModel.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>

Define_Module(UavChannelModel);

namespace {

// Fichero de log compartido por todos los modelos de canal de la simulacion
// (cada NIC tiene el suyo): se abre una sola vez para que las lineas no se
// mezclen.
std::ofstream& sharedAttLog(const std::string& path)
{
    static std::ofstream f;
    static std::string openPath;
    if (!f.is_open() || openPath != path) {
        if (f.is_open())
            f.close();
        f.open(path, std::ios::out | std::ios::trunc);
        if (!f)
            throw omnetpp::cRuntimeError("UavChannelModel: no se puede abrir '%s'", path.c_str());
        openPath = path;
    }
    return f;
}

// Mezclador de 64 bits (splitmix64): convierte una clave en un numero
// pseudoaleatorio bien distribuido. Se usa para el sorteo de LoS.
inline uint64_t splitmix64(uint64_t x)
{
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}

inline int64_t gridCell(double v, double g)
{
    return static_cast<int64_t>(std::floor(v / g));
}

} // namespace

void UavChannelModel::initialize(int stage)
{
    NRChannelModel_3GPP38_901::initialize(stage);

    if (stage != inet::INITSTAGE_LOCAL)
        return;

    groundMaxHeight_ = par("groundMaxHeight").doubleValue();
    towerMaxHeight_ = par("towerMaxHeight").doubleValue();
    maxLinkDistance_ = par("maxLinkDistance").doubleValue();
    outOfRangeAttenuation_ = par("outOfRangeAttenuation").doubleValue();
    losGridSize_ = par("losGridSize").doubleValue();
    losSeed_ = static_cast<uint64_t>(par("losSeed").intValue());
    attLogFile_ = par("attenuationLogFile").stdstringValue();
    attLogInterval_ = par("attenuationLogInterval").doubleValue();
    if (!attLogFile_.empty())
        sharedAttLog(attLogFile_);   // se abre (y se vacia) al arrancar

    if (groundMaxHeight_ >= towerMaxHeight_)
        throw omnetpp::cRuntimeError("UavChannelModel: groundMaxHeight debe ser menor que towerMaxHeight");
    if (losGridSize_ <= 0)
        throw omnetpp::cRuntimeError("UavChannelModel: losGridSize debe ser > 0");

    // Al-Hourani et al., "Optimal LAP Altitude for Maximum Coverage",
    // IEEE Wireless Commun. Letters, 2014 (parametros a, b, eta_LoS, eta_NLoS)
    std::string env = par("airGroundEnvironment").stdstringValue();
    if (env == "suburban")        { ahA_ = 4.88;  ahB_ = 0.43; ahEtaLos_ = 0.1; ahEtaNlos_ = 21.0; }
    else if (env == "urban")      { ahA_ = 9.61;  ahB_ = 0.16; ahEtaLos_ = 1.0; ahEtaNlos_ = 20.0; }
    else if (env == "denseurban") { ahA_ = 12.08; ahB_ = 0.11; ahEtaLos_ = 1.6; ahEtaNlos_ = 23.0; }
    else if (env == "highrise")   { ahA_ = 27.23; ahB_ = 0.08; ahEtaLos_ = 2.3; ahEtaNlos_ = 34.0; }
    else
        throw omnetpp::cRuntimeError("UavChannelModel: airGroundEnvironment '%s' no valido "
                                     "(suburban, urban, denseurban, highrise)", env.c_str());
}

void UavChannelModel::finish()
{
    NRChannelModel_3GPP38_901::finish();
    if (!attLogFile_.empty())
        sharedAttLog(attLogFile_).flush();
}

void UavChannelModel::writeAttLog(MacNodeId nodeId, Direction dir, int link, bool los,
                                  double d3D, double d2D, double zA, double zB, double att)
{
    if (attLogFile_.empty())
        return;
    auto key = std::make_pair(nodeId, static_cast<int>(dir));
    auto it = attLogLast_.find(key);
    omnetpp::simtime_t now = omnetpp::simTime();
    if (it != attLogLast_.end() && now - it->second < attLogInterval_)
        return;
    attLogLast_[key] = now;

    // Mismo formato que la linea EV: lo lee check_channel_model.py
    sharedAttLog(attLogFile_)
        << "t=" << now << " " << getFullPath()
        << " UavChannelModel::getAttenuation - link=" << link << " los=" << los
        << " d3D=" << d3D << " d2D=" << d2D << " zA=" << zA << " zB=" << zB
        << " att=" << att << " dB\n";
}

// ---------------------------------------------------------------------------
//  Clasificacion
// ---------------------------------------------------------------------------

UavChannelModel::NodeClass UavChannelModel::classify(double z) const
{
    if (z <= groundMaxHeight_)
        return GROUND;
    if (z <= towerMaxHeight_)
        return TOWER;
    return AERIAL;
}

UavChannelModel::LinkType UavChannelModel::linkType(NodeClass a, NodeClass b) const
{
    if (a == AERIAL && b == AERIAL)
        return LINK_AIR_AIR;
    if (a == AERIAL || b == AERIAL) {
        NodeClass other = (a == AERIAL) ? b : a;
        return (other == GROUND) ? LINK_AIR_GROUND : LINK_AIR_TOWER;
    }
    return LINK_TERRESTRIAL;
}

// ---------------------------------------------------------------------------
//  Perdidas
// ---------------------------------------------------------------------------

// FSPL (dB) = 20 log10(d[m]) + 20 log10(fc[GHz]) + 32.45
double UavChannelModel::freeSpacePathLoss(double d3D) const
{
    double fc = getCarrierFrequency();   // GHz
    return 20.0 * std::log10(std::max(d3D, 1.0)) + 20.0 * std::log10(fc) + 32.45;
}

// Al-Hourani: espacio libre + perdida adicional media segun LoS/NLoS
double UavChannelModel::pathLossAirGround(double d3D, bool los) const
{
    return freeSpacePathLoss(d3D) + (los ? ahEtaLos_ : ahEtaNlos_);
}

// 3GPP TR 36.777, Anexo B, UMa-AV (UE aereo, 22.5 m < h <= 300 m)
//   LoS : 28.0 + 22 log10(d3D) + 20 log10(fc)
//   NLoS: -17.5 + (46 - 7 log10(h)) log10(d3D) + 20 log10(40*pi*fc/3)
// Se toma max(LoS, NLoS) en NLoS, igual que hace 38.901, para que NLoS nunca
// salga con menos perdida que LoS.
double UavChannelModel::pathLossUMaAV(double d3D, double hUav, bool los) const
{
    double fc = getCarrierFrequency();   // GHz
    double h = std::min(std::max(hUav, 22.5), 300.0);
    double d = std::max(d3D, 1.0);

    double plLos = 28.0 + 22.0 * std::log10(d) + 20.0 * std::log10(fc);
    if (los)
        return plLos;

    double plNlos = -17.5 + (46.0 - 7.0 * std::log10(h)) * std::log10(d)
                    + 20.0 * std::log10(40.0 * M_PI * fc / 3.0);
    return std::max(plLos, plNlos);
}

// ---------------------------------------------------------------------------
//  Probabilidad de LoS
// ---------------------------------------------------------------------------

// TR 38.901 UMa para UE a menos de 13 m (C'(h) = 0)
double UavChannelModel::losProbUMa(double d2D) const
{
    if (d2D <= 18.0)
        return 1.0;
    return 18.0 / d2D + std::exp(-d2D / 63.0) * (1.0 - 18.0 / d2D);
}

// Al-Hourani: P_LoS = 1 / (1 + a * exp(-b * (theta - a))), theta en grados
double UavChannelModel::losProbAirGround(double d2D, double dh) const
{
    double thetaDeg = std::atan2(std::max(dh, 0.0), std::max(d2D, 0.0)) * 180.0 / M_PI;
    return 1.0 / (1.0 + ahA_ * std::exp(-ahB_ * (thetaDeg - ahA_)));
}

// TR 36.777 UMa-AV:
//   h > 100 m       -> P = 1
//   22.5 < h <= 100 -> P = 1 si d2D <= d1, si no d1/d2D + exp(-d2D/p1)(1 - d1/d2D)
//                      p1 = 4300 log10(h) - 3800,  d1 = max(460 log10(h) - 700, 18)
double UavChannelModel::losProbUMaAV(double d2D, double hUav) const
{
    if (hUav > 100.0)
        return 1.0;
    double h = std::max(hUav, 22.5);
    double d1 = std::max(460.0 * std::log10(h) - 700.0, 18.0);
    double p1 = 4300.0 * std::log10(h) - 3800.0;
    if (d2D <= d1)
        return 1.0;
    return d1 / d2D + std::exp(-d2D / p1) * (1.0 - d1 / d2D);
}

// Sorteo de LoS por enlace. La clave son las celdas de la rejilla de los dos
// extremos (ordenadas, para que UL y DL den lo mismo) y losSeed. Mientras
// ninguno de los dos cambie de celda, el resultado no cambia; al cambiar se
// vuelve a sortear.
bool UavChannelModel::drawLos(const inet::Coord& a, const inet::Coord& b, double p) const
{
    if (p >= 1.0)
        return true;
    if (p <= 0.0)
        return false;

    int64_t ca[3] = { gridCell(a.x, losGridSize_), gridCell(a.y, losGridSize_), gridCell(a.z, losGridSize_) };
    int64_t cb[3] = { gridCell(b.x, losGridSize_), gridCell(b.y, losGridSize_), gridCell(b.z, losGridSize_) };
    if (std::lexicographical_compare(cb, cb + 3, ca, ca + 3))
        std::swap(ca, cb);

    uint64_t h = splitmix64(losSeed_);
    for (int i = 0; i < 3; i++)
        h = splitmix64(h ^ static_cast<uint64_t>(ca[i]));
    for (int i = 0; i < 3; i++)
        h = splitmix64(h ^ static_cast<uint64_t>(cb[i]));

    double u = static_cast<double>(h >> 11) * (1.0 / 9007199254740992.0);   // [0, 1)
    return u < p;
}

// ---------------------------------------------------------------------------
//  Atenuacion (sustituye a NRChannelModel::getAttenuation)
// ---------------------------------------------------------------------------

double UavChannelModel::getAttenuation(MacNodeId nodeId, Direction dir, inet::Coord coord, bool cqiDl)
{
    const inet::Coord myCoord = phy_->getCoord();
    double threeDimDistance = myCoord.distance(coord);
    double twoDimDistance = getTwoDimDistance(myCoord, coord);

    // Enlaces fuera de rango: gNodeBs aparcados, etc. Sin senal.
    if (twoDimDistance > maxLinkDistance_)
        return outOfRangeAttenuation_;

    double speed = (dir == DL) ? computeSpeed(nodeId, myCoord) : computeSpeed(nodeId, coord);

    double hHigh = std::max(myCoord.z, coord.z);
    double hLow = std::min(myCoord.z, coord.z);
    LinkType lt = linkType(classify(myCoord.z), classify(coord.z));

    bool los;
    double attenuation;
    switch (lt) {
        case LINK_AIR_AIR:
            los = true;
            attenuation = freeSpacePathLoss(threeDimDistance);
            break;

        case LINK_AIR_GROUND:
            los = drawLos(myCoord, coord, losProbAirGround(twoDimDistance, hHigh - hLow));
            attenuation = pathLossAirGround(threeDimDistance, los);
            break;

        case LINK_AIR_TOWER:
            los = drawLos(myCoord, coord, losProbUMaAV(twoDimDistance, hHigh));
            attenuation = pathLossUMaAV(threeDimDistance, hHigh, los);
            break;

        case LINK_TERRESTRIAL:
        default:
            // Torre <-> suelo: formulas de Simu5G (scenario del .ini), pero con
            // la LoS por enlace en lugar de la fija por UE
            los = drawLos(myCoord, coord, losProbUMa(twoDimDistance));
            attenuation = computePathLoss(threeDimDistance, twoDimDistance, los);
            break;
    }

    // Se deja en losMap_ por si otras partes de Simu5G lo consultan
    losMap_[nodeId] = los;

    // Shadowing de Simu5G (no en UAV <-> UAV: espacio libre sin obstaculos)
    if (lt != LINK_AIR_AIR && nodeId < BGUE_MIN_ID && shadowing_)
        attenuation += computeShadowing(twoDimDistance, nodeId, speed, cqiDl);

    if (dir == DL) {
        emit(distance_, twoDimDistance);
        updatePositionHistory(nodeId, myCoord);
    }
    else {
        updatePositionHistory(nodeId, coord);
    }

    // Formato fijo: lo lee check_channel_model.py
    EV << "UavChannelModel::getAttenuation - link=" << lt << " los=" << los
       << " d3D=" << threeDimDistance << " d2D=" << twoDimDistance
       << " zA=" << myCoord.z << " zB=" << coord.z
       << " att=" << attenuation << " dB" << endl;

    writeAttLog(nodeId, dir, lt, los, threeDimDistance, twoDimDistance,
                myCoord.z, coord.z, attenuation);

    return attenuation;
}
