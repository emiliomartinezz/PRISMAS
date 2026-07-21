# -*- coding: utf-8 -*-
"""
dt_bridge.py — Puente entre SUMO-UAV-Py Extended y el Gemelo Digital (DT).

Este módulo hace que SUMO-UAV-Py "hable el idioma" del gemelo digital
desarrollado para OMNeT++/Simu5G, de forma que el DT funciona SIN CAMBIOS:

  - Cada UAV se publica como un gNodeB    →  mensajes {"type": "GNB_Report"}
  - Cada vehículo SUMO se publica como UE →  mensajes {"type": "UE_Report"}
    (el SINR se estima con el mismo modelo log-distance que usa
     coverage_optimizer.py, tomando como servidor el UAV con mejor señal)
  - El DT responde con {"type": "MOVE"} / {"type": "MOVE_BATCH"}, que este
    módulo encola para que UAVSimulation los aplique respetando la máquina
    de estados (MISSION / RETURNING / CHARGING / STANDBY), la batería, el
    viento y el modo relevo.

Principio de diseño de la fusión:
    El gemelo digital decide DÓNDE deben estar los UAVs;
    el modelo realista de SUMO-UAV-Py decide CÓMO y SI pueden ir
    (velocidad real, consumo, retorno a base, relevo).

Uso (ver PATCH_main_.md):
    bridge = DTBridge.from_config(sim.config)
    bridge.connect()
    ...
    bridge.maybe_send_reports(sim_time, uav_snapshot, vehicle_snapshot)
    for cmd in bridge.pop_commands(): ...
    bridge.close()
"""

import json
import math
import socket
import threading
import time
from collections import deque


# ══════════════════════════════════════════════════════════════════════════
#   Estimador de SINR (mismo modelo log-distance que coverage_optimizer.py)
# ══════════════════════════════════════════════════════════════════════════
class LinkBudgetEstimator:
    """
    Estima el SNR (≈ SINR sin interferencia) entre un punto en tierra y cada
    UAV usando el modelo de propagación log-distance:

        PL(d)  = PL(d0) + 10·n·log10(d/d0)
        PL(d0) = 20·log10(4·pi·d0·f/c)        (espacio libre a d0 = 1 m)
        Prx    = Ptx − PL(d)
        N      = −174 dBm/Hz + 10·log10(B)
        SNR    = Prx − N
    """

    UE_HEIGHT = 1.5  # m, altura del terminal en tierra (igual que el optimizador)

    def __init__(self, tx_power_dbm=30.0, freq_ghz=2.6,
                 path_loss_exp=2.7, bandwidth_mhz=20.0):
        self.tx_power_dbm = float(tx_power_dbm)
        self.freq_hz = float(freq_ghz) * 1e9
        self.n = float(path_loss_exp)
        bw_hz = float(bandwidth_mhz) * 1e6
        # Pérdida en espacio libre a d0 = 1 m
        c = 3e8
        self.pl_d0 = 20.0 * math.log10(4.0 * math.pi * 1.0 * self.freq_hz / c)
        # Potencia de ruido térmico
        self.noise_dbm = -174.0 + 10.0 * math.log10(bw_hz)

    def snr(self, ue_xy, uav_xyz):
        """SNR (dB) entre un UE en (x, y, UE_HEIGHT) y un UAV en (x, y, z)."""
        dx = uav_xyz[0] - ue_xy[0]
        dy = uav_xyz[1] - ue_xy[1]
        dz = uav_xyz[2] - self.UE_HEIGHT
        d = max(math.sqrt(dx * dx + dy * dy + dz * dz), 1.0)
        pl = self.pl_d0 + 10.0 * self.n * math.log10(d)
        prx = self.tx_power_dbm - pl
        return prx - self.noise_dbm

    def best_server(self, ue_xy, uav_map):
        """
        Devuelve (gnb_id_del_mejor_uav, snr_dB). uav_map es un dict
        {uav_id: {'x','y','z','gnb_id','airborne'}}. Solo se consideran
        UAVs en el aire (z > 0 y en estado operativo).
        """
        best_id, best_snr = -1, -1e9
        for info in uav_map.values():
            if not info.get('airborne', True):
                continue
            s = self.snr(ue_xy, (info['x'], info['y'], info['z']))
            if s > best_snr:
                best_snr, best_id = s, info['gnb_id']
        return best_id, best_snr


# ══════════════════════════════════════════════════════════════════════════
#   Puente TCP con el Gemelo Digital
# ══════════════════════════════════════════════════════════════════════════
class DTBridge:
    """
    Cliente TCP hacia el servidor del gemelo digital (por defecto
    127.0.0.1:50000). Publica telemetría con el mismo esquema JSON que los
    módulos UESender/GnbSender de OMNeT++ y recibe MOVE / MOVE_BATCH.
    """

    def __init__(self, host='127.0.0.1', port=50000,
                 report_interval=1.0, estimator=None, verbose=True):
        self.host = host
        self.port = int(port)
        self.report_interval = float(report_interval)
        self.estimator = estimator or LinkBudgetEstimator()
        self.verbose = verbose

        self.sock = None
        self.connected = False
        self._rx_thread = None
        self._stop = threading.Event()

        self._cmd_queue = deque()
        self._cmd_lock = threading.Lock()

        self._last_report_time = -1e9
        self._ue_id_map = {}      # id de vehículo SUMO (str) -> int estable
        self._next_ue_id = 1000   # ids numéricos para los UEs

    # ------------------------------------------------------------ factoría
    @classmethod
    def from_config(cls, config):
        """Construye el puente a partir del config.json de SUMO-UAV-Py."""
        est = LinkBudgetEstimator(
            tx_power_dbm=config.get("tx power dbm", 30.0),
            freq_ghz=config.get("Carrier frequency (GHz)", 2.6),
            path_loss_exp=config.get("Path loss Exponent", 2.7),
            bandwidth_mhz=config.get("Bandwidth (MHz)", 20.0),
        )
        return cls(
            host=config.get("DT Host", "127.0.0.1"),
            port=config.get("DT Port", 50000),
            report_interval=config.get("DT Report Interval (s)", 1.0),
            estimator=est,
        )

    # ------------------------------------------------------------ conexión
    def connect(self, max_retries=30, retry_delay=1.0):
        """Conecta con el DT (con reintentos) y arranca el hilo receptor."""
        for i in range(max_retries):
            try:
                self.sock = socket.create_connection(
                    (self.host, self.port), timeout=5.0)
                self.sock.settimeout(None)
                self.connected = True
                break
            except OSError:
                print(f"[DTBridge] Esperando al gemelo digital en "
                      f"{self.host}:{self.port}... ({i + 1}/{max_retries})")
                time.sleep(retry_delay)
        if not self.connected:
            raise RuntimeError(
                f"[DTBridge] No se pudo conectar con el DT en "
                f"{self.host}:{self.port}. ¿Está lanzado digital_twin_*.py?")

        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="DTBridge-RX", daemon=True)
        self._rx_thread.start()
        print(f"[DTBridge] Conectado al gemelo digital "
              f"({self.host}:{self.port}).")

    def close(self):
        self._stop.set()
        self.connected = False
        try:
            if self.sock:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
        except OSError:
            pass

    # ------------------------------------------------------------ hilo RX
    @staticmethod
    def _split_json_messages(raw):
        """Separa objetos JSON concatenados (mismo parser que usa el DT)."""
        messages, remaining = [], raw.strip()
        while remaining:
            remaining = remaining.strip()
            if not remaining:
                break
            if remaining.startswith('{'):
                depth = 0
                for i, ch in enumerate(remaining):
                    if ch == '{':
                        depth += 1
                    elif ch == '}':
                        depth -= 1
                        if depth == 0:
                            messages.append(remaining[:i + 1])
                            remaining = remaining[i + 1:]
                            break
                else:
                    # JSON incompleto: se devuelve como resto pendiente
                    return messages, remaining
            else:
                nxt = remaining.find('{')
                if nxt > 0:
                    remaining = remaining[nxt:]
                else:
                    break
        return messages, ""

    def _rx_loop(self):
        """Recibe respuestas del DT y encola los comandos MOVE."""
        buffer = ""
        while not self._stop.is_set():
            try:
                data = self.sock.recv(65536)
            except OSError:
                break
            if not data:
                print("[DTBridge] El DT cerró la conexión.")
                self.connected = False
                break
            buffer += data.decode('utf-8', errors='replace')
            msgs, buffer = self._split_json_messages(buffer)
            for m in msgs:
                self._handle_dt_message(m)

    def _handle_dt_message(self, message):
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            return
        mtype = data.get("type")
        if mtype == "MOVE":
            self._enqueue_move(data)
        elif mtype == "MOVE_BATCH":
            moves = data.get("moves", data.get("commands", []))
            for mv in moves:
                mv.setdefault("type", "MOVE")
                self._enqueue_move(mv)
        # respuestas {"status": ...} u otros tipos se ignoran silenciosamente

    def _enqueue_move(self, mv):
        try:
            cmd = {
                "gnb_index": int(mv["gnb_index"]),
                "x": float(mv["x"]),
                "y": float(mv["y"]),
                "z": float(mv.get("z", 0.0)),
                "speed": float(mv.get("speed", 0.0)),
            }
        except (KeyError, TypeError, ValueError):
            print(f"[DTBridge] MOVE malformado ignorado: {mv}")
            return
        with self._cmd_lock:
            self._cmd_queue.append(cmd)
        if self.verbose:
            print(f"[DTBridge] MOVE recibido: UAV[{cmd['gnb_index']}] -> "
                  f"({cmd['x']:.1f}, {cmd['y']:.1f}, {cmd['z']:.1f})")

    def pop_commands(self):
        """Vacía y devuelve la cola de comandos pendientes del DT."""
        with self._cmd_lock:
            cmds = list(self._cmd_queue)
            self._cmd_queue.clear()
        return cmds

    # ------------------------------------------------------------ telemetría
    def _ue_numeric_id(self, veh_id):
        if veh_id not in self._ue_id_map:
            self._ue_id_map[veh_id] = self._next_ue_id
            self._next_ue_id += 1
        return self._ue_id_map[veh_id]

    def maybe_send_reports(self, sim_time, uav_snapshot, vehicle_snapshot):
        """
        Envía la telemetría si ha pasado report_interval desde el último envío.

        uav_snapshot: dict {uav_id(int): {'x','y','z','state','battery'}}
            state ∈ {MISSION, RETURNING, CHARGING, STANDBY}
            battery ∈ [0, 1] fracción de batería restante (o None)
        vehicle_snapshot: dict {veh_id(str): (x, y)}
        """
        if not self.connected:
            return
        if sim_time - self._last_report_time < self.report_interval:
            return
        self._last_report_time = sim_time

        # Mapa UAV → gNodeB. Convención: gnb_id = uav_id + 1 (ids > 0),
        # gnb_index = uav_id (lo que usa el DT en los comandos MOVE).
        uav_map = {}
        for uav_id, info in uav_snapshot.items():
            uav_map[uav_id] = {
                'x': info['x'], 'y': info['y'], 'z': info['z'],
                'gnb_id': uav_id + 1,
                'airborne': (info['z'] > 0.0 and
                             info.get('state', 'MISSION') != 'CHARGING'),
            }

        payload_parts = []
       
        # --- Ajuste de coordenadas para que el DT coincida con la escena de SUMO ---
        OFFSET_X = 179.75
        OFFSET_Y = 619.12

        # ---- UE_Report por vehículo (SINR estimado + mejor servidor) ----
        served_by = {info['gnb_id']: [] for info in uav_map.values()}
        for veh_id, (vx, vy) in vehicle_snapshot.items():
            master_id, snr = self.estimator.best_server((vx, vy), uav_map)
            ue_id = self._ue_numeric_id(veh_id)
            if master_id > 0:
                served_by[master_id].append(ue_id)
            payload_parts.append(json.dumps({
                "type": "UE_Report",
                "ue_id": ue_id,
                "ue_index": ue_id,
                "timestamp": round(float(sim_time), 3),
                "position": {"x": round(vx - OFFSET_X, 2), "y": round(vy - OFFSET_Y, 2),
                             "z": LinkBudgetEstimator.UE_HEIGHT},
                "network": {"sinr": round(snr, 2),
                            "master_id": int(master_id)},
            }))

        # ---- GNB_Report por UAV (posición + UEs servidos + estado) ----
        for uav_id, info in uav_snapshot.items():
            gnb_id = uav_id + 1
            connected = served_by.get(gnb_id, [])
            payload_parts.append(json.dumps({
                "type": "GNB_Report",
                "gnb_id": gnb_id,
                "gnb_index": uav_id,
                "timestamp": round(float(sim_time), 3),
                "position": {"x": round(info['x'], 2),
                             "y": round(info['y'], 2),
                             "z": round(info['z'], 2)},
                "connected_ues": connected,
                "num_connected": len(connected),
                # Campos extra de la fusión (el DT actual los ignora;
                # disponibles para algoritmos conscientes de la energía):
                "uav_state": info.get('state', 'MISSION'),
                "battery": (round(info['battery'], 3)
                            if info.get('battery') is not None else None),
            }))

        try:
            self.sock.sendall("".join(payload_parts).encode('utf-8'))
        except OSError as e:
            print(f"[DTBridge] Error enviando telemetría: {e}")
            self.connected = False
