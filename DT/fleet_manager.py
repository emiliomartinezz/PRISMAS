"""
fleet_manager.py — despliegue bajo demanda de los UAVs desde el gemelo digital.

Los UAVs arrancan en STANDBY en su base. En cada ciclo de optimizacion el
gemelo:

  1. Busca UEs SIN COBERTURA: los que estan asociados a un gNodeB cuyo UAV no
     esta en mision (un dron posado en la base no cuenta como cobertura) o
     cuyo SINR esta por debajo de DEMAND_SINR_THRESHOLD.
  2. Los agrupa por cercania (radio DEPLOY_RADIUS). Cada grupo con al menos
     DEPLOY_MIN_UES UEs y sin ningun UAV activo cerca es una demanda.
  3. Si la demanda se mantiene DEPLOY_CONFIRM_TIME segundos seguidos, despierta
     el UAV en STANDBY mas cercano a cada grupo (orden DEPLOY al plugin, con el
     centroide del grupo como destino), respetando el limite de PUESTOS.
  4. Los UAVs en mision que se quedan sin UEs durante RECALL_IDLE_TIME vuelven
     a la base (orden RECALL) y, tras cargar, quedan otra vez en STANDBY.
     Tras cualquier despegue (ordenado por el gemelo o un relevo por bateria
     que decide el plugin) hay RECALL_GRACE segundos en los que no se cuenta
     el tiempo ocioso, para que el UAV llegue a su destino y los UEs hagan
     handover hacia el.

PUESTOS: FLEET_SLOTS es el numero de puestos ("Number of UAVs" del plugin).
Si el gemelo ve el doble de gNodeBs, es que el plugin tiene "Relay Mode" y cada
puesto i lo forman uav<i> y uav<i+FLEET_SLOTS> (la misma pareja que usa el
plugin para el relevo); si ve FLEET_SLOTS gNodeBs, cada UAV es un puesto. Esto
se deduce solo, no hay que configurarlo. Un puesto esta ocupado si uno de sus
UAVs esta en mision o tiene un DEPLOY en camino, y el gemelo solo despliega en
puestos libres: nunca hay mas de FLEET_SLOTS puestos dando servicio. Si uno de
la pareja esta cargando, puede salir el otro. Durante un relevo por bateria (lo
decide el plugin) los dos UAVs del puesto pueden estar en el aire a la vez
unos segundos.

Una vez en mision, el reposicionamiento fino lo sigue haciendo el algoritmo
del gemelo (centroide, centroide ponderado o k-means): este modulo solo decide
CUANTOS UAVs vuelan y CUALES.

Las coordenadas son las de OMNeT++, igual que las del resto del gemelo.

El fleet manager tiene su propio ritmo (FLEET_INTERVAL, por defecto 2 s de
simulacion), mas rapido que el reposicionamiento (10 s), para reaccionar pronto
cuando aparecen UEs sin cobertura. Su reloj es el timestamp mas reciente de los
GNB_Report, que llegan siempre aunque no quede ningun coche: asi los UAVs
ociosos se retiran tambien cuando todos los UEs han desaparecido.

Uso desde cualquier variante del gemelo:

    from fleet_manager import FleetManager
    self.fleet = FleetManager(self)                 # en __init__, tras crear self.plugin
    self.fleet.update(self.grid.last_update_time)   # tras procesar CADA lote de
                                                    # reports (el se autolimita a
                                                    # FLEET_INTERVAL)
"""

import math
from collections import defaultdict


# ================= CONFIGURACION DEL MODO BAJO DEMANDA =================

ENABLE_ON_DEMAND = True        # Debe coincidir con "On Demand Mode" del plugin
FLEET_INTERVAL = 2.0           # s de simulacion entre decisiones de flota

# Puestos: igual que "Number of UAVs" en config.json del plugin. Si hay
# parejas de relevo ("Relay Mode") se deduce solo del numero de gNodeBs.
FLEET_SLOTS = 3

# UEs que llevan mas de este tiempo sin reportar (el coche ha salido de SUMO)
# no cuentan como demanda. No se borran del grafo: solo se ignoran aqui.
UE_STALE_TIME = 5.0            # s

# Que se considera "sin cobertura"
DEMAND_SINR_THRESHOLD = 20.0   # dB. Por debajo, el UE cuenta como no cubierto
                               # aunque le sirva un UAV en mision. Igual que el
                               # umbral de cobertura del gemelo
                               # (SINR_THRESHOLD_COVERAGE), para que el criterio
                               # de despliegue y el del informe coincidan

# Despliegue
DEPLOY_RADIUS = 300.0          # m. Radio para agrupar UEs y para decidir si ya
                               # hay un UAV activo cerca de una demanda
DEPLOY_MIN_UES = 1             # UEs sin cobertura minimos para justificar un vuelo
DEPLOY_CONFIRM_TIME = 4.0      # s seguidos con demanda antes de despegar (filtra
                               # picos de un instante). 0 = despegar en cuanto se vea
DEPLOY_SPEED = 20.0            # m/s (informativo, el plugin usa su modelo)
DEPLOY_ALTITUDE = 0.0          # m. 0 = la decide el plugin ("Deploy Altitude (m)")
TRANSIT_FOLLOW_RADIUS = 600.0  # m. Antes de sacar otro UAV se mira si hay uno
                               # en el aire libre a menos de esta distancia: uno
                               # que aun va de camino (se mide desde su destino)
                               # o uno en mision sin UEs. Si lo hay, se le manda
                               # alli (MOVE) en vez de desplegar otro
ARRIVAL_RADIUS = 100.0         # m. A menos de esto de su destino, el UAV ha llegado
PENDING_TIMEOUT = 30.0         # s. Si el UAV no pasa a MISSION en este tiempo,
                               # se da la orden por perdida y se puede repetir

# Retirada
RECALL_MAX_UES = 0             # Un UAV con <= este numero de UEs esta ocioso
RECALL_IDLE_TIME = 60.0        # s seguidos ocioso antes de mandarlo a la base
RECALL_GRACE = 60.0            # s tras el despegue en los que no empieza a
                               # contar el tiempo ocioso (al principio aun no
                               # tiene UEs asociados). Minimo para retirar un
                               # UAV: RECALL_GRACE + RECALL_IDLE_TIME
MIN_ACTIVE_UAVS = 0            # UAVs que se mantienen en el aire aunque esten
                               # ociosos


class FleetManager:

    def __init__(self, dt):
        self.dt = dt
        self.enabled = ENABLE_ON_DEMAND
        self.pending = {}            # gnb_index -> {'t', 'x', 'y'} DEPLOY sin confirmar
        self.recalled = {}           # gnb_index -> t de la orden RECALL
        self.deployed_at = {}        # gnb_index -> t en que se confirmo MISSION
        self.idle_since = {}         # gnb_index -> t desde el que esta ocioso
        self.last_state = {}         # gnb_index -> estado en el ciclo anterior
        self.transit = {}            # gnb_index -> {'x','y'} destino de un UAV
                                     # desplegado que aun no ha llegado
        self.demand_since = None     # t desde el que hay demanda sin atender
        self._last_run = None        # t de la ultima decision de flota
        self.events = []             # Registro para el informe final
        self.relay_pairs = False     # se deduce de los gNodeBs que se ven
        self._layout_seen = None     # ultimo numero de gNodeBs analizado
        state = "ACTIVO" if self.enabled else "DESACTIVADO"
        print(f"[Fleet] Modo bajo demanda {state} (SINR<{DEMAND_SINR_THRESHOLD} dB, "
              f"radio {DEPLOY_RADIUS} m, min {DEPLOY_MIN_UES} UEs)")

    # ------------------------------------------------------------- lectura

    @staticmethod
    def _dist(x1, y1, x2, y2):
        return math.hypot(x1 - x2, y1 - y2)

    def slot_of(self, gnb_index):
        """Puesto al que pertenece el UAV que lleva gNodeB[gnb_index]."""
        return gnb_index % FLEET_SLOTS if self.relay_pairs else gnb_index

    def _detect_fleet_layout(self, gnbs):
        """Deduce si hay parejas de relevo a partir del numero de gNodeBs:
        2*FLEET_SLOTS -> parejas (Relay Mode); FLEET_SLOTS -> un UAV por puesto.
        Se repite en cada ciclo por si al principio aun no han reportado todos."""
        n = len(gnbs)
        if not n or n == self._layout_seen:
            return
        self._layout_seen = n
        self.relay_pairs = n >= 2 * FLEET_SLOTS
        if self.relay_pairs:
            print(f"[Fleet] {n} gNodeBs para {FLEET_SLOTS} puestos: parejas de relevo "
                  f"uav<i> / uav<i+{FLEET_SLOTS}>")
        else:
            print(f"[Fleet] {n} gNodeBs para {FLEET_SLOTS} puestos: un UAV por puesto")
        if n not in (FLEET_SLOTS, 2 * FLEET_SLOTS):
            print(f"[Fleet] AVISO: se esperaban {FLEET_SLOTS} o {2 * FLEET_SLOTS} gNodeBs "
                  f"(FLEET_SLOTS={FLEET_SLOTS}) y hay {n}. Revisa que FLEET_SLOTS coincida "
                  f"con 'Number of UAVs' y numGnb en omnetpp.ini (si aun no han "
                  f"reportado todos los gNodeBs, ignora este aviso).")

    def _gnbs(self):
        """Ultimo estado conocido de cada gNodeB, desde el grafo del gemelo."""
        out = []
        for _, nd in self.dt.G.nodes(data=True):
            if nd.get('node_type') != 'gnb':
                continue
            out.append({
                'last_update': nd.get('last_update', 0.0),
                'gnb_id': nd.get('gnb_id'),
                'gnb_index': nd.get('gnb_index'),
                'x': nd.get('pos_x', 0.0), 'y': nd.get('pos_y', 0.0),
                'state': nd.get('uav_state', 'UNKNOWN'),
            })
        return out

    def _ues(self, now):
        out = []
        for _, nd in self.dt.G.nodes(data=True):
            if nd.get('node_type') != 'ue':
                continue
            if now - nd.get('last_update', now) > UE_STALE_TIME:
                continue     # el coche ya no esta en la simulacion
            out.append({
                'ue_index': nd.get('ue_index'),
                'x': nd.get('pos_x', 0.0), 'y': nd.get('pos_y', 0.0),
                'sinr': nd.get('sinr', -999.0),
                'master_id': nd.get('master_id', -1),
            })
        return out

    # -------------------------------------------------------- bucle principal

    def update(self, sim_time=None):
        if not self.enabled:
            return
        gnbs = self._gnbs()
        # Reloj: el report mas reciente. Los GNB_Report llegan cada segundo
        # aunque no quede ningun coche, asi que el tiempo sigue avanzando.
        clock = [g['last_update'] for g in gnbs if g['last_update']]
        if sim_time is not None:
            clock.append(sim_time)
        if not clock:
            return
        sim_time = max(clock)
        if self._last_run is not None and sim_time - self._last_run < FLEET_INTERVAL:
            return
        self._last_run = sim_time
        ues = self._ues(sim_time)
        self._detect_fleet_layout(gnbs)
        self._detect_takeoffs(gnbs, sim_time)
        self._refresh_pending(gnbs, sim_time)

        active = [g for g in gnbs if g['state'] == 'MISSION'
                  and g['gnb_index'] not in self.recalled]
        # Puestos ocupados: algun UAV en mision (que no se este retirando) o
        # con un DEPLOY en camino
        busy_slots = {self.slot_of(g['gnb_index']) for g in active}
        busy_slots |= {self.slot_of(i) for i in self.pending}
        standby = [g for g in gnbs if g['state'] == 'STANDBY'
                   and g['gnb_index'] not in self.pending
                   and self.slot_of(g['gnb_index']) not in busy_slots]
        active_ids = {g['gnb_id'] for g in active}
        self._refresh_transit(gnbs, ues, sim_time)

        self._deploy(ues, active, standby, active_ids, sim_time)
        self._recall(ues, active, sim_time)

    def _detect_takeoffs(self, gnbs, t):
        """Cualquier UAV que pasa a MISSION empieza su periodo de gracia, lo
        haya despertado el gemelo (DEPLOY) o el plugin (relevo por bateria)."""
        for g in gnbs:
            idx = g['gnb_index']
            if g['state'] == 'MISSION' and self.last_state.get(idx) != 'MISSION':
                self.deployed_at[idx] = t
                self.idle_since.pop(idx, None)
                origin = 'gemelo' if idx in self.pending else 'plugin'
                self._log(t, 'TAKEOFF', idx, origin=origin)
                if origin == 'plugin':
                    print(f"[Fleet] gNodeB[{idx}] ha despegado por relevo del plugin "
                          f"(t={t:.1f}s): {RECALL_GRACE:.0f}s de gracia")
            self.last_state[idx] = g['state']

    def _refresh_pending(self, gnbs, t):
        by_index = {g['gnb_index']: g for g in gnbs}
        for idx in list(self.pending):
            g = by_index.get(idx)
            if g is not None and g['state'] == 'MISSION':
                self.deployed_at[idx] = t
                self.idle_since.pop(idx, None)
                del self.pending[idx]
                print(f"[Fleet] gNodeB[{idx}] confirmado en MISSION (t={t:.1f}s)")
            elif t - self.pending[idx]['t'] > PENDING_TIMEOUT:
                print(f"[Fleet] gNodeB[{idx}] no ha despegado en {PENDING_TIMEOUT}s; "
                      f"se libera para reintentar")
                del self.pending[idx]
        for idx in list(self.recalled):
            g = by_index.get(idx)
            if g is not None and g['state'] != 'MISSION':
                del self.recalled[idx]     # ya va de vuelta (RETURNING/CHARGING/...)
            elif t - self.recalled[idx] > PENDING_TIMEOUT:
                del self.recalled[idx]

    def _refresh_transit(self, gnbs, ues, t):
        """Un UAV deja de estar 'en camino' cuando llega a su destino, cuando
        acaba su periodo de gracia o cuando ya no esta en mision. (No basta con
        que sirva UEs: mientras vuela puede captar coches con mala senal.)"""
        by_index = {g['gnb_index']: g for g in gnbs}
        served = defaultdict(set)
        for u in ues:
            served[u['master_id']].add(u['ue_index'])
        self._served = served
        self._airborne = {g['gnb_index']: g for g in gnbs
                          if g['state'] == 'MISSION' and g['gnb_index'] not in self.recalled}
        for idx in list(self.transit):
            g = by_index.get(idx)
            if idx in self.pending:
                continue
            d = self.transit[idx]
            if g is None or g['state'] != 'MISSION' or idx in self.recalled:
                del self.transit[idx]
            elif self._dist(g['x'], g['y'], d['x'], d['y']) <= ARRIVAL_RADIUS:
                del self.transit[idx]
            elif t - self.deployed_at.get(idx, t) >= RECALL_GRACE:
                del self.transit[idx]

    # ------------------------------------------------------------ despliegue

    def _uncovered(self, ues, active_ids):
        out = []
        for u in ues:
            served_by_active = u['master_id'] in active_ids
            if not served_by_active:
                out.append(u)
            elif -998 < u['sinr'] < DEMAND_SINR_THRESHOLD:
                out.append(u)
        return out

    @staticmethod
    def _cluster(points, radius):
        """Agrupacion voraz: se toma el punto con mas vecinos dentro del radio,
        se forma un grupo con ellos y se repite con los que quedan."""
        remaining = list(points)
        clusters = []
        while remaining:
            best, best_nb = None, []
            for p in remaining:
                nb = [q for q in remaining
                      if math.hypot(p['x'] - q['x'], p['y'] - q['y']) <= radius]
                if len(nb) > len(best_nb):
                    best, best_nb = p, nb
            ids = {id(q) for q in best_nb}
            remaining = [q for q in remaining if id(q) not in ids]
            cx = sum(q['x'] for q in best_nb) / len(best_nb)
            cy = sum(q['y'] for q in best_nb) / len(best_nb)
            clusters.append({'x': cx, 'y': cy, 'ues': best_nb})
        clusters.sort(key=lambda c: -len(c['ues']))
        return clusters

    def _deploy(self, ues, active, standby, active_ids, t):
        uncovered = self._uncovered(ues, active_ids)
        clusters = [c for c in self._cluster(uncovered, DEPLOY_RADIUS)
                    if len(c['ues']) >= DEPLOY_MIN_UES]

        # Descartar demandas que ya atiende (o va a atender) otro UAV
        busy = [(g['x'], g['y']) for g in active] + \
               [(p['x'], p['y']) for p in self.pending.values()] + \
               [(d['x'], d['y']) for d in self.transit.values()]
        demand = [c for c in clusters
                  if all(self._dist(c['x'], c['y'], bx, by) > DEPLOY_RADIUS
                         for bx, by in busy)]

        # Si un UAV que va de camino tiene su destino cerca de una demanda, es
        # que los coches se han movido: se le corrige el destino en lugar de
        # sacar otro UAV
        demand = self._retarget_transit(demand, t)

        if not demand:
            self.demand_since = None
            return
        if self.demand_since is None:
            self.demand_since = t
        waited = t - self.demand_since
        print(f"[Fleet] {len(uncovered)} UEs sin cobertura, {len(demand)} zona(s) "
              f"con demanda ({waited:.0f}/{DEPLOY_CONFIRM_TIME:.0f} s)")
        if waited < DEPLOY_CONFIRM_TIME:
            return

        available = list(standby)
        if not available:
            print("[Fleet] Hay demanda pero no queda ningun puesto libre con un UAV en STANDBY")
            return

        for c in demand:
            if not available:
                break
            # El mas cercano; a igual distancia (pareja en la misma base), el de
            # indice menor (el principal)
            g = min(available, key=lambda s: (self._dist(s['x'], s['y'], c['x'], c['y']),
                                              s['gnb_index']))
            # El puesto queda ocupado: su pareja ya no esta disponible
            slot = self.slot_of(g['gnb_index'])
            available = [s for s in available if self.slot_of(s['gnb_index']) != slot]
            if self.dt.plugin.send_deploy(g['gnb_index'], c['x'], c['y'],
                                          DEPLOY_ALTITUDE, DEPLOY_SPEED):
                self.pending[g['gnb_index']] = {'t': t, 'x': c['x'], 'y': c['y']}
                self.transit[g['gnb_index']] = {'x': c['x'], 'y': c['y']}
                self._log(t, 'DEPLOY', g['gnb_index'], x=c['x'], y=c['y'],
                          num_ues=len(c['ues']))
                print(f"[Fleet] >>> DEPLOY gNodeB[{g['gnb_index']}] -> "
                      f"({c['x']:.0f}, {c['y']:.0f}) para {len(c['ues'])} UEs")
        self.demand_since = None

    def _retarget_transit(self, demand, t):
        # Candidatos en el aire: los que van de camino (se miden desde su
        # destino) y los que estan en mision (desde su posicion). Un UAV puede
        # encargarse de un grupo si todos los UEs que sirve estan en ese grupo:
        # moverlo no deja a nadie atras. Asi un UAV que sigue a coches que se
        # alejan se redirige a ellos en vez de sacar otro.
        airborne = getattr(self, '_airborne', {})
        cand = {idx: (d['x'], d['y']) for idx, d in self.transit.items()}
        for idx, g in airborne.items():
            if idx not in cand and idx not in self.pending:
                cand[idx] = (g['x'], g['y'])
        remaining = []
        used = set()
        for c in demand:
            ids = {u['ue_index'] for u in c['ues']}
            near = []
            for idx, (fx, fy) in cand.items():
                if idx in used:
                    continue
                g = airborne.get(idx)
                mine = self._served.get(g['gnb_id'], set()) if g else set()
                if mine <= ids:
                    near.append((self._dist(c['x'], c['y'], fx, fy), idx))
            near = [n for n in near if n[0] <= TRANSIT_FOLLOW_RADIUS]
            if not near:
                remaining.append(c)
                continue
            dist, idx = min(near)
            used.add(idx)
            self.transit[idx] = {'x': c['x'], 'y': c['y']}
            if idx in self.pending:
                self.pending[idx].update(x=c['x'], y=c['y'])
            if self.dt.plugin.send_moves([{'gnb_index': idx, 'x': c['x'], 'y': c['y'],
                                           'z': DEPLOY_ALTITUDE, 'speed': DEPLOY_SPEED}]):
                self._log(t, 'RETARGET', idx, x=c['x'], y=c['y'], num_ues=len(c['ues']))
                print(f"[Fleet] ~~> gNodeB[{idx}] (en el aire y libre) redirigido a "
                      f"({c['x']:.0f}, {c['y']:.0f}) en vez de sacar otro UAV")
        return remaining

    # -------------------------------------------------------------- retirada

    def _recall(self, ues, active, t):
        served = defaultdict(int)
        for u in ues:
            served[u['master_id']] += 1

        n_active = len(active)
        for g in sorted(active, key=lambda g: served[g['gnb_id']]):
            idx = g['gnb_index']
            if idx in self.pending:
                continue
            if t - self.deployed_at.get(idx, -1e9) < RECALL_GRACE:
                self.idle_since.pop(idx, None)   # en gracia: no cuenta como ocioso
                continue
            if served[g['gnb_id']] > RECALL_MAX_UES:
                self.idle_since.pop(idx, None)
                continue
            since = self.idle_since.setdefault(idx, t)
            if t - since < RECALL_IDLE_TIME:
                continue
            if n_active <= MIN_ACTIVE_UAVS:
                break
            if self.dt.plugin.send_recall(idx):
                self.recalled[idx] = t
                self.idle_since.pop(idx, None)
                n_active -= 1
                self._log(t, 'RECALL', idx)
                print(f"[Fleet] <<< RECALL gNodeB[{idx}]: {t - since:.0f}s sin UEs")

    # ------------------------------------------------------------- registro

    def _log(self, t, action, gnb_index, **extra):
        entry = {'sim_time': t, 'action': action, 'gnb_index': gnb_index}
        entry.update(extra)
        self.events.append(entry)