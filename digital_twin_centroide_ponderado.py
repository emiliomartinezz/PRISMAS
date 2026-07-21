#!/usr/bin/env python3
"""
===============================================================================
  GEMELO DIGITAL DE RED 5G/6G
===============================================================================
  Autor: Lucas Vilarino
  Proyecto: TFM MUIT - UPM
  Algoritmo de reposicionamiento: Centroide ponderado por SINR
===============================================================================
"""

import socket
import signal
import sys
import json
import math
import time
import threading
from pathlib import Path
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from matplotlib.colors import ListedColormap
from datetime import datetime
from collections import defaultdict

# ================= CONFIGURACION COMUN =================

HOST = "0.0.0.0"
PORT = 50000

RESULTS_DIR = "./coverage_results"
PLOTS_DIR = "./coverage_plots"

AREA_SIZE_X = 2000
AREA_SIZE_Y = 2000
GRID_CELL_SIZE = 100

SINR_THRESHOLD_COVERAGE = 20

QOS_LEVELS = {
    'Excellent': 20, 'Good': 13, 'Fair': 0, 'Poor': -10, 'No Service': -999
}

ENABLE_REALTIME_ANALYSIS = True
ANALYSIS_INTERVAL = 10.0

ENABLE_REALTIME_TOPOLOGY = True
TOPOLOGY_UPDATE_INTERVAL = 15.0

ENABLE_FINAL_REPORT = True

SESSION_TIMESTAMP = datetime.now().strftime("%Y%m%d_%H%M%S")


# ===============================================================================
#   GRID DE COBERTURA
# ===============================================================================

class CoverageGrid:
    def __init__(self, area_size_x=AREA_SIZE_X, area_size_y=AREA_SIZE_Y,
                 cell_size=GRID_CELL_SIZE):
        self.area_size_x = area_size_x
        self.area_size_y = area_size_y
        self.cell_size = cell_size
        self.grid_cols = int(np.ceil(area_size_x / cell_size))
        self.grid_rows = int(np.ceil(area_size_y / cell_size))
        self.coverage_map = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        self.sinr_map = np.full((self.grid_rows, self.grid_cols), -999.0)
        self.cell_id_map = np.full((self.grid_rows, self.grid_cols), -1, dtype=int)
        self.measurement_count = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        self.last_update_time = None
        print(f"[CoverageGrid] Initialized: {self.grid_rows}x{self.grid_cols} cells "
              f"({self.grid_rows * self.grid_cols} total)")

    def position_to_grid(self, x, y):
        col = max(0, min(int(x / self.cell_size), self.grid_cols - 1))
        row = max(0, min(int(y / self.cell_size), self.grid_rows - 1))
        return row, col

    def update_from_ue_measurement(self, x, y, sinr, cell_id, timestamp):
        row, col = self.position_to_grid(x, y)
        if sinr >= SINR_THRESHOLD_COVERAGE:
            self.coverage_map[row, col] = 1
        else:
            self.coverage_map[row, col] = 0
        if self.measurement_count[row, col] == 0:
            self.sinr_map[row, col] = sinr
        else:
            alpha = 0.3
            self.sinr_map[row, col] = alpha * sinr + (1 - alpha) * self.sinr_map[row, col]
        self.cell_id_map[row, col] = cell_id
        self.measurement_count[row, col] += 1
        self.last_update_time = timestamp

    def get_coverage_percentage(self):
        total_cells = self.grid_rows * self.grid_cols
        covered_cells = np.sum(self.coverage_map)
        return (covered_cells / total_cells) * 100

    def get_coverage_statistics(self):
        total_cells = self.grid_rows * self.grid_cols
        covered_cells = np.sum(self.coverage_map)
        uncovered_cells = total_cells - covered_cells
        measured_cells = np.sum(self.measurement_count > 0)
        unmeasured_cells = total_cells - measured_cells
        covered_sinr_values = self.sinr_map[self.coverage_map == 1]
        if len(covered_sinr_values) > 0:
            avg_sinr_covered = np.mean(covered_sinr_values)
            min_sinr_covered = np.min(covered_sinr_values)
            max_sinr_covered = np.max(covered_sinr_values)
        else:
            avg_sinr_covered = min_sinr_covered = max_sinr_covered = -999
        gnb_distribution = {}
        for gnb_id in np.unique(self.cell_id_map):
            if gnb_id >= 0:
                cells_served = np.sum(self.cell_id_map == gnb_id)
                gnb_distribution[int(gnb_id)] = {
                    'cells': int(cells_served),
                    'percentage': (cells_served / measured_cells * 100) if measured_cells > 0 else 0
                }
        return {
            'total_cells': total_cells,
            'covered_cells': int(covered_cells),
            'uncovered_cells': int(uncovered_cells),
            'coverage_percentage': (covered_cells / total_cells) * 100,
            'measured_cells': int(measured_cells),
            'unmeasured_cells': int(unmeasured_cells),
            'measurement_percentage': (measured_cells / total_cells) * 100,
            'avg_sinr_covered': float(avg_sinr_covered),
            'min_sinr_covered': float(min_sinr_covered),
            'max_sinr_covered': float(max_sinr_covered),
            'total_measurements': int(np.sum(self.measurement_count)),
            'gnb_distribution': gnb_distribution,
            'last_update': self.last_update_time
        }

    def get_qos_distribution(self):
        measured_mask = self.measurement_count > 0
        total_measured_cells = np.sum(measured_mask)
        if total_measured_cells == 0:
            return {level: {'cells': 0, 'percentage': 0.0} for level in QOS_LEVELS.keys()}
        measured_sinr = self.sinr_map[measured_mask]
        distribution = {}
        excellent_count = np.sum(measured_sinr >= 20)
        distribution['Excellent'] = {
            'cells': int(excellent_count),
            'percentage': (excellent_count / total_measured_cells) * 100}
        good_count = np.sum((measured_sinr >= 13) & (measured_sinr < 20))
        distribution['Good'] = {
            'cells': int(good_count),
            'percentage': (good_count / total_measured_cells) * 100}
        fair_count = np.sum((measured_sinr >= 0) & (measured_sinr < 13))
        distribution['Fair'] = {
            'cells': int(fair_count),
            'percentage': (fair_count / total_measured_cells) * 100}
        poor_count = np.sum((measured_sinr >= -10) & (measured_sinr < 0))
        distribution['Poor'] = {
            'cells': int(poor_count),
            'percentage': (poor_count / total_measured_cells) * 100}
        no_service_count = np.sum(measured_sinr < -10)
        distribution['No Service'] = {
            'cells': int(no_service_count),
            'percentage': (no_service_count / total_measured_cells) * 100}
        return distribution


# ===============================================================================
#   DETECTOR DE COVERAGE HOLES (DBSCAN)
# ===============================================================================

class CoverageHoleDetector:
    SEVERE_THRESHOLD = -10
    POOR_THRESHOLD = 0
    FAIR_THRESHOLD = 13
    COVERAGE_THRESHOLD = 20
    DBSCAN_EPS_CELLS = 2
    DBSCAN_MIN_SAMPLES = 2

    def __init__(self, coverage_grid):
        self.grid = coverage_grid
        self.holes = []
        self.hole_cells = []
        self.last_detection_time = None

    def detect(self):
        self.hole_cells = []
        self.holes = []
        self._identify_hole_cells()
        if not self.hole_cells:
            self.last_detection_time = self.grid.last_update_time
            return self.holes
        self._cluster_holes()
        self._characterize_regions()
        self.last_detection_time = self.grid.last_update_time
        return self.holes

    def _identify_hole_cells(self):
        grid = self.grid
        for row in range(grid.grid_rows):
            for col in range(grid.grid_cols):
                measured = grid.measurement_count[row, col] > 0
                sinr = grid.sinr_map[row, col]
                if measured and sinr < self.COVERAGE_THRESHOLD:
                    self.hole_cells.append({
                        'row': row, 'col': col,
                        'x': (col + 0.5) * grid.cell_size,
                        'y': (row + 0.5) * grid.cell_size,
                        'sinr': float(sinr),
                        'severity': self._classify_severity(sinr),
                        'type': 'measured',
                        'measurements': int(grid.measurement_count[row, col])})
                elif not measured:
                    neighbors_measured = 0
                    neighbors_poor = 0
                    for dr in [-1, 0, 1]:
                        for dc in [-1, 0, 1]:
                            if dr == 0 and dc == 0: continue
                            nr, nc = row + dr, col + dc
                            if 0 <= nr < grid.grid_rows and 0 <= nc < grid.grid_cols:
                                if grid.measurement_count[nr, nc] > 0:
                                    neighbors_measured += 1
                                    if grid.sinr_map[nr, nc] < self.COVERAGE_THRESHOLD:
                                        neighbors_poor += 1
                    if neighbors_measured >= 3 and neighbors_poor >= 2:
                        avg_sinr = self._avg_neighbor_sinr(row, col)
                        self.hole_cells.append({
                            'row': row, 'col': col,
                            'x': (col + 0.5) * grid.cell_size,
                            'y': (row + 0.5) * grid.cell_size,
                            'sinr': float(avg_sinr),
                            'severity': self._classify_severity(avg_sinr),
                            'type': 'inferred', 'measurements': 0})

    def _avg_neighbor_sinr(self, row, col):
        grid = self.grid
        sinr_values = []
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0: continue
                nr, nc = row + dr, col + dc
                if 0 <= nr < grid.grid_rows and 0 <= nc < grid.grid_cols:
                    if grid.measurement_count[nr, nc] > 0:
                        sinr_values.append(grid.sinr_map[nr, nc])
        return np.mean(sinr_values) if sinr_values else -999.0

    def _classify_severity(self, sinr):
        if sinr < self.SEVERE_THRESHOLD: return 'critical'
        elif sinr < self.POOR_THRESHOLD: return 'severe'
        elif sinr < self.FAIR_THRESHOLD: return 'moderate'
        else: return 'mild'

    def _cluster_holes(self):
        if len(self.hole_cells) < self.DBSCAN_MIN_SAMPLES:
            for i, cell in enumerate(self.hole_cells):
                cell['cluster_id'] = i
            return
        try:
            from sklearn.cluster import DBSCAN
            coords = np.array([[c['row'], c['col']] for c in self.hole_cells])
            clustering = DBSCAN(eps=self.DBSCAN_EPS_CELLS,
                                min_samples=self.DBSCAN_MIN_SAMPLES).fit(coords)
            for i, label in enumerate(clustering.labels_):
                self.hole_cells[i]['cluster_id'] = int(label)
        except ImportError:
            print("[CoverageHoleDetector] sklearn no disponible, usando clustering simple")
            self._cluster_holes_simple()

    def _cluster_holes_simple(self):
        visited = set()
        cluster_id = 0
        for i, cell in enumerate(self.hole_cells):
            if i in visited: continue
            queue = [i]
            visited.add(i)
            cluster_members = [i]
            while queue:
                current = queue.pop(0)
                cr = self.hole_cells[current]['row']
                cc = self.hole_cells[current]['col']
                for j, other in enumerate(self.hole_cells):
                    if j in visited: continue
                    dist = abs(other['row'] - cr) + abs(other['col'] - cc)
                    if dist <= self.DBSCAN_EPS_CELLS:
                        visited.add(j)
                        queue.append(j)
                        cluster_members.append(j)
            cid = cluster_id if len(cluster_members) >= self.DBSCAN_MIN_SAMPLES else -1
            for idx in cluster_members:
                self.hole_cells[idx]['cluster_id'] = cid
            if len(cluster_members) >= self.DBSCAN_MIN_SAMPLES:
                cluster_id += 1

    def _characterize_regions(self):
        self.holes = []
        clusters = defaultdict(list)
        for cell in self.hole_cells:
            clusters[cell['cluster_id']].append(cell)
        for cluster_id, cells in sorted(clusters.items()):
            if cluster_id == -1:
                for cell in cells:
                    self.holes.append(self._make_hole_entry(
                        f"isolated_{cell['row']}_{cell['col']}", 'isolated', [cell]))
                continue
            self.holes.append(self._make_hole_entry(
                f"region_{cluster_id}", 'cluster', cells))
        self.holes.sort(key=lambda h: (-h['severity_score'], -h['area_m2']))

    def _make_hole_entry(self, hole_id, hole_type, cells):
        xs = [c['x'] for c in cells]
        ys = [c['y'] for c in cells]
        sinrs = [c['sinr'] for c in cells if c['sinr'] > -999]
        severities = [c['severity'] for c in cells]
        measured_count = sum(1 for c in cells if c['type'] == 'measured')
        cs = self.grid.cell_size
        worst = self._worst_severity(severities)
        return {
            'id': hole_id, 'type': hole_type,
            'center': {'x': float(np.mean(xs)), 'y': float(np.mean(ys))},
            'area_m2': float(len(cells) * cs * cs),
            'num_cells': len(cells),
            'avg_sinr': float(np.mean(sinrs)) if sinrs else -999.0,
            'min_sinr': float(np.min(sinrs)) if sinrs else -999.0,
            'severity': worst,
            'severity_score': self._severity_score(worst),
            'measured_ratio': measured_count / len(cells) if cells else 0,
            'bounding_box': {
                'x_min': float(min(xs) - cs / 2), 'x_max': float(max(xs) + cs / 2),
                'y_min': float(min(ys) - cs / 2), 'y_max': float(max(ys) + cs / 2)}}

    def _severity_score(self, severity):
        return {'critical': 4, 'severe': 3, 'moderate': 2, 'mild': 1}.get(severity, 0)

    def _worst_severity(self, severities):
        for level in ['critical', 'severe', 'moderate', 'mild']:
            if level in severities: return level
        return 'mild'

    def get_summary(self):
        if not self.holes:
            return {'total_holes': 0, 'total_cells_affected': 0, 'total_area_m2': 0,
                    'by_severity': {}, 'regions': [], 'detection_time': self.last_detection_time}
        total_cells = sum(h['num_cells'] for h in self.holes)
        total_area = sum(h['area_m2'] for h in self.holes)
        by_severity = defaultdict(lambda: {'count': 0, 'cells': 0, 'area_m2': 0})
        for h in self.holes:
            s = h['severity']
            by_severity[s]['count'] += 1
            by_severity[s]['cells'] += h['num_cells']
            by_severity[s]['area_m2'] += h['area_m2']
        regions_summary = []
        for h in self.holes:
            regions_summary.append({
                'id': h['id'], 'type': h['type'], 'center': h['center'],
                'area_m2': h['area_m2'], 'num_cells': h['num_cells'],
                'avg_sinr': h['avg_sinr'], 'min_sinr': h['min_sinr'],
                'severity': h['severity'], 'severity_score': h['severity_score'],
                'measured_ratio': h['measured_ratio'], 'bounding_box': h['bounding_box']})
        return {
            'total_holes': len(self.holes),
            'total_cells_affected': total_cells,
            'total_area_m2': total_area,
            'by_severity': dict(by_severity),
            'regions': regions_summary,
            'detection_time': self.last_detection_time}

    def print_summary(self):
        summary = self.get_summary()
        print("\n" + "="*60)
        print("   COVERAGE HOLE DETECTION")
        print("="*60)
        if summary['total_holes'] == 0:
            print("  No coverage holes detected")
            print("="*60 + "\n")
            return
        print(f"  Total regions : {summary['total_holes']}")
        print(f"  Cells affected: {summary['total_cells_affected']}")
        print(f"  Area affected : {summary['total_area_m2']:.0f} m2")
        print("-"*60)
        print("  By severity:")
        icons = {'critical': '[CRIT]', 'severe': '[SEVR]', 'moderate': '[MODR]', 'mild': '[MILD]'}
        for sev in ['critical', 'severe', 'moderate', 'mild']:
            if sev in summary['by_severity']:
                d = summary['by_severity'][sev]
                print(f"    {icons[sev]} {sev:10s}: {d['count']} regions, "
                      f"{d['cells']} cells, {d['area_m2']:.0f} m2")
        print("-"*60)
        print("  Top coverage holes (prioritized):")
        for i, r in enumerate(summary['regions'][:10]):
            icon = icons.get(r['severity'], '[????]')
            print(f"    {i+1}. {icon} [{r['severity']}] "
                  f"center=({r['center']['x']:.0f}, {r['center']['y']:.0f}) "
                  f"area={r['area_m2']:.0f}m2 "
                  f"SINR={r['avg_sinr']:.1f}dB "
                  f"({r['num_cells']} cells)")
        print("="*60 + "\n")


# ===============================================================================
#   CONFIGURACION DEL ALGORITMO DE REPOSICIONAMIENTO: CENTROIDE PONDERADO POR SINR
# ===============================================================================
# El gemelo digital reubica cada gNodeB hacia el centroide de los UEs que sirve,
# ponderando cada UE por la inversa de su SINR, de modo que los usuarios peor
# servidos atraen mas la estacion hacia las zonas de cobertura degradada.

OPTIMIZER_NAME = "Centroide ponderado por SINR"

ENABLE_TOPOLOGY_OPTIMIZATION = True
OPTIMIZATION_INTERVAL = 10.0        # Segundos de simulacion entre optimizaciones
OPTIMIZATION_MIN_DISTANCE = 50.0    # Distancia minima (m) para enviar MOVE
OPTIMIZATION_SPEED = 20.0           # Velocidad de vuelo del dron (m/s)
OPTIMIZATION_MIN_UES = 1            # Minimo de UEs servidos para optimizar

# Constante de regularizacion para evitar division por cero al invertir el SINR.
# Se aplica sobre el SINR en escala lineal (no en dB).
SINR_WEIGHT_EPS = 1e-3


# ===============================================================================
#   GEMELO DIGITAL  (servidor TCP integrado en la clase)
# ===============================================================================

class DigitalTwin:
    QOS_COLORS = {
        'Excellent': '#00C853', 'Good': '#64DD17', 'Fair': '#FFD600',
        'Poor': '#FF6D00', 'No Service': '#D50000'}

    def __init__(self):
        self.G = nx.Graph()
        self.grid = CoverageGrid()
        self.hole_detector = CoverageHoleDetector(self.grid)
        self.ue_data = defaultdict(list)
        self.gnb_data = {}
        self.start_time = None
        self._known_gnbs = set()

        # Estado del reposicionamiento
        self.move_commands_log = []
        self.last_optimization_sim_time = 0

        # Estado del servidor TCP (integrado en la clase)
        self.conn = None
        self.addr = None
        self.server_running = True

        print("[DigitalTwin] Inicializado")

    # ---------------------------------------------------------------- utilidades
    @staticmethod
    def _sinr_to_qos(sinr):
        if sinr >= 20: return 'Excellent'
        if sinr >= 13: return 'Good'
        if sinr >= 0:  return 'Fair'
        if sinr >= -10: return 'Poor'
        return 'No Service'

    @staticmethod
    def _dist(x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    # ------------------------------------------------- ingesta de telemetria UE
    def handleUE_ReportMessage(self, message):
        try:
            data = json.loads(message)
            if data.get("type") != "UE_Report": return False
            ue_id = data.get("ue_id", -1)
            ue_index = data.get("ue_index", -1)
            ts = data.get("timestamp")
            pos = data.get("position", {})
            net = data.get("network", {})
            x, y, z = pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)
            sinr = net.get("sinr", -999)
            master = net.get("master_id", -1)
            qos = self._sinr_to_qos(sinr)

            self.grid.update_from_ue_measurement(x, y, sinr, master, ts)

            self.ue_data[ue_id].append({
                'timestamp': ts, 'ue_index': ue_index,
                'x': x, 'y': y, 'z': z, 'sinr': sinr, 'master_id': master})
            if self.start_time is None:
                self.start_time = ts

            records = self.ue_data[ue_id]
            vals = [r['sinr'] for r in records if r['sinr'] > -999]
            avg_sinr = sum(vals) / len(vals) if vals else -999
            min_sinr = min(vals) if vals else -999
            max_sinr = max(vals) if vals else -999
            handovers = sum(1 for i in range(1, len(records))
                            if records[i]['master_id'] != records[i-1]['master_id'])

            dist_to_gnb = -1.0
            gnb_node = f"gNB_{master}"
            if gnb_node in self.G:
                gx, gy = self.G.nodes[gnb_node]['pos_x'], self.G.nodes[gnb_node]['pos_y']
                dist_to_gnb = self._dist(x, y, gx, gy)

            ue_node = f"UE_{ue_index}"
            self.G.add_node(ue_node,
                            node_type='ue', ue_id=ue_id, ue_index=ue_index,
                            pos_x=float(x), pos_y=float(y), pos_z=float(z),
                            sinr=float(sinr),
                            avg_sinr=float(avg_sinr),
                            min_sinr=float(min_sinr),
                            max_sinr=float(max_sinr),
                            qos=qos, master_id=int(master),
                            dist_to_gnb=float(dist_to_gnb),
                            handovers=int(handovers),
                            measurements=int(len(records)),
                            last_update=float(ts) if ts else 0.0)

            old = [(u, v) for u, v in self.G.edges(ue_node)
                   if self.G.edges[u, v].get('link_type') == 'serving']
            self.G.remove_edges_from(old)
            if master >= 0 and gnb_node in self.G:
                self.G.add_edge(ue_node, gnb_node,
                                link_type='serving', sinr=float(sinr),
                                qos=qos, distance=float(dist_to_gnb))
            return True
        except Exception as e:
            print(f"[DT] Error UE: {e}")
            return False

    # ---------------------------------------------- ingesta de telemetria gNodeB
    def handleGNB_ReportMessage(self, message):
        try:
            data = json.loads(message)
            if data.get("type") != "GNB_Report": return False
            gnb_id = data.get("gnb_id")
            gnb_index = data.get("gnb_index")
            ts = data.get("timestamp")
            pos = data.get("position", {})
            connected = data.get("connected_ues", [])
            num_conn = data.get("num_connected", 0)
            x, y, z = pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)

            print(f"\n[Servidor] GNB_Report from gNodeB[{gnb_index}] (id={gnb_id})")
            print(f"  Time: {ts:.2f}s | Pos: ({x:.1f}, {y:.1f}, {z:.1f})")
            print(f"  Connected UEs: {num_conn} {connected if connected else ''}")

            if gnb_id not in self.gnb_data:
                self.gnb_data[gnb_id] = []
            self.gnb_data[gnb_id].append({
                'timestamp': ts, 'index': gnb_index,
                'x': x, 'y': y, 'z': z,
                'connected_ues': connected, 'num_connected': num_conn})

            gnb_node = f"gNB_{gnb_id}"
            self.G.add_node(gnb_node,
                            node_type='gnb', gnb_id=int(gnb_id),
                            gnb_index=int(gnb_index),
                            pos_x=float(x), pos_y=float(y), pos_z=float(z),
                            num_connected=int(num_conn),
                            connected_ues_str=json.dumps(connected),
                            total_reports=int(len(self.gnb_data[gnb_id])),
                            last_update=float(ts) if ts else 0.0)

            if gnb_id not in self._known_gnbs:
                for oid in self._known_gnbs:
                    on = f"gNB_{oid}"
                    if on in self.G:
                        ox = self.G.nodes[on]['pos_x']
                        oy = self.G.nodes[on]['pos_y']
                        self.G.add_edge(gnb_node, on, link_type='x2',
                                        distance=float(self._dist(x, y, ox, oy)))
                self._known_gnbs.add(gnb_id)
            return True
        except Exception as e:
            print(f"[DT] Error GNB_Report: {e}")
            return False

    def detect_coverage_holes(self):
        return self.hole_detector.detect()

    # ----------------------------------------------- envio de comandos a OMNeT++
    def send_move_batch(self, move_list):
        if self.conn is None:
            print("[DT] ERROR: No hay conexion TCP activa para enviar MOVE_BATCH")
            return False
        if not move_list:
            return False

        cmd = {"type": "MOVE_BATCH", "moves": move_list}
        json_str = json.dumps(cmd)
        try:
            self.conn.sendall(json_str.encode("utf-8"))
            for move in move_list:
                gnb_index = move['gnb_index']
                x, y, z = move['x'], move['y'], move['z']
                speed = move.get('speed', OPTIMIZATION_SPEED)
                distance = -1
                eta = -1
                for n, d in self.G.nodes(data=True):
                    if (d.get('node_type') == 'gnb' and
                            d.get('gnb_index') == gnb_index):
                        dx = x - d['pos_x']
                        dy = y - d['pos_y']
                        distance = math.sqrt(dx*dx + dy*dy)
                        eta = distance / speed if speed > 0 else 0
                        break
                self.move_commands_log.append({
                    "wall_time": time.time(),
                    "sim_time": self.grid.last_update_time,
                    "command": {"type": "MOVE", "gnb_index": gnb_index,
                                "x": x, "y": y, "z": z, "speed": speed},
                    "distance": distance, "eta": eta,
                    "reason": "topology_optimization"})

            print("\n" + "=" * 60)
            print("  >>> MOVE_BATCH ENVIADO A OMNET++ (OPTIMIZACION) <<<")
            print("=" * 60)
            for move in move_list:
                print(f"  gNodeB[{move['gnb_index']}] -> "
                      f"({move['x']:.1f}, {move['y']:.1f}, {move['z']:.1f}) "
                      f"@ {move.get('speed', OPTIMIZATION_SPEED)} m/s")
            print(f"  Sim time: {self.grid.last_update_time:.2f}s")
            print("=" * 60 + "\n")
            return True
        except Exception as e:
            print(f"[DT] ERROR enviando MOVE_BATCH: {e}")
            return False

    # ======================================================================
    #   ALGORITMO DE REPOSICIONAMIENTO: CENTROIDE PONDERADO POR SINR
    # ======================================================================
    @staticmethod
    def _inverse_sinr_weight(sinr_db):
        """
        Peso = 1 / max(SINR_lineal, eps).

        El SINR llega en dB y puede ser negativo o nulo, por lo que invertirlo
        directamente en dB no es valido. Se convierte primero a escala lineal
        (siempre positiva) y se invierte: cuanto peor es el SINR, mayor es el
        peso, de modo que los UEs mal cubiertos arrastran el centroide hacia
        las zonas degradadas.
        """
        sinr_lin = 10.0 ** (sinr_db / 10.0)
        return 1.0 / max(sinr_lin, SINR_WEIGHT_EPS)

    def compute_optimal_positions(self):
        """
        Para cada gNodeB calcula el centroide ponderado de los UEs que sirve,
        donde el peso de cada UE es inversamente proporcional a su SINR. Solo se
        genera comando si la distancia al centroide supera OPTIMIZATION_MIN_DISTANCE.
        """
        moves = []

        # UEs servidos por cada gNodeB (con su SINR para ponderar)
        gnb_serving_ues = defaultdict(list)
        for _, nd in self.G.nodes(data=True):
            if nd.get('node_type') != 'ue':
                continue
            master_id = nd.get('master_id', -1)
            if master_id < 0:
                continue
            gnb_serving_ues[master_id].append({
                'x': nd['pos_x'], 'y': nd['pos_y'], 'sinr': nd.get('sinr', -999)})

        for gnb_node_name in list(self.G.nodes):
            nd = self.G.nodes[gnb_node_name]
            if nd.get('node_type') != 'gnb':
                continue

            gnb_id = nd.get('gnb_id')
            gnb_index = nd.get('gnb_index')
            gnb_x, gnb_y, gnb_z = nd['pos_x'], nd['pos_y'], nd['pos_z']

            ues = gnb_serving_ues.get(gnb_id, [])
            if len(ues) < OPTIMIZATION_MIN_UES:
                continue

            weights = [self._inverse_sinr_weight(u['sinr']) for u in ues]
            w_sum = sum(weights)
            if w_sum <= 0:
                continue

            centroid_x = sum(w * u['x'] for w, u in zip(weights, ues)) / w_sum
            centroid_y = sum(w * u['y'] for w, u in zip(weights, ues)) / w_sum
            dist = self._dist(gnb_x, gnb_y, centroid_x, centroid_y)

            if dist >= OPTIMIZATION_MIN_DISTANCE:
                moves.append({'gnb_index': gnb_index,
                              'x': centroid_x, 'y': centroid_y, 'z': gnb_z,
                              'speed': OPTIMIZATION_SPEED,
                              'num_ues': len(ues), 'distance': dist})
                print(f"[W-CENTROID] gNodeB[{gnb_index}] (id={gnb_id}): "
                      f"pos=({gnb_x:.0f}, {gnb_y:.0f}) -> "
                      f"centroide_pond=({centroid_x:.0f}, {centroid_y:.0f}) "
                      f"dist={dist:.0f}m, {len(ues)} UEs")
        return moves

    def should_optimize(self):
        if not ENABLE_TOPOLOGY_OPTIMIZATION:
            return False
        sim_time = self.grid.last_update_time
        if sim_time is None:
            return False
        return (sim_time - self.last_optimization_sim_time) >= OPTIMIZATION_INTERVAL

    # ===========================================================================
    #   SERVIDOR TCP  (integrado en la clase DigitalTwin)
    # ===========================================================================

    @staticmethod
    def split_tcp_messages(raw_data):
        messages = []
        remaining = raw_data.strip()
        while remaining:
            remaining = remaining.strip()
            if not remaining: break
            if remaining.startswith('{'):
                depth = 0
                for i, ch in enumerate(remaining):
                    if ch == '{': depth += 1
                    elif ch == '}':
                        depth -= 1
                        if depth == 0:
                            messages.append(remaining[:i+1])
                            remaining = remaining[i+1:]
                            break
                else:
                    messages.append(remaining)
                    remaining = ""
            else:
                next_json = remaining.find('{')
                if next_json > 0:
                    remaining = remaining[next_json:]
                else:
                    break
        return messages

    def process_message(self, message):
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            if msg_type == "UE_Report":
                self.handleUE_ReportMessage(message)
                sinr = data.get("network", {}).get("sinr")
                ue_id = data.get("ue_id", "?")
                ue_index = data.get("ue_index", "?")
                if sinr is not None:
                    print(f"[Servidor] UE[{ue_index}] (id={ue_id}) SINR: {sinr:.2f} dB")
                return {"status": "ok"}
            elif msg_type == "GNB_Report":
                self.handleGNB_ReportMessage(message)
                return {"status": "ok"}
            else:
                print(f"[Servidor] Tipo desconocido: {msg_type}")
                return {"status": "ignored"}
        except json.JSONDecodeError as e:
            print(f"[Servidor] JSON invalido: {e}")
            return {"status": "error", "message": "Invalid JSON"}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def run_tcp_server(self):
        """
        Servidor TCP del gemelo digital. Recibe UE_Report/GNB_Report, actualiza el grafo y,
        cada OPTIMIZATION_INTERVAL segundos de simulacion, ejecuta el algoritmo de
        reposicionamiento y devuelve un MOVE_BATCH como respuesta.
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.settimeout(1.0)
            s.bind((HOST, PORT))
            s.listen(1)

            print("="*60)
            print(f"[Servidor] Escuchando en {HOST}:{PORT}")
            print(f"[Servidor] Session: {SESSION_TIMESTAMP}")
            print(f"[Servidor] Algoritmo de reposicionamiento: {OPTIMIZER_NAME}")
            if ENABLE_TOPOLOGY_OPTIMIZATION:
                print(f"[Servidor] Optimizacion: cada {OPTIMIZATION_INTERVAL}s sim, "
                      f"dist_min={OPTIMIZATION_MIN_DISTANCE}m, "
                      f"speed={OPTIMIZATION_SPEED} m/s")
            else:
                print("[Servidor] Optimizacion de topologia DESACTIVADA")
            print("="*60)

            while self.server_running:
                try:
                    self.conn, self.addr = s.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                print(f"[Servidor] Conexion desde {self.addr}")
                with self.conn:
                    self.conn.settimeout(1.0)
                    while self.server_running:
                        try:
                            data = self.conn.recv(8192)
                        except socket.timeout:
                            continue
                        except OSError:
                            break
                        if not data:
                            print("[Servidor] Cliente desconectado")
                            break

                        raw = data.decode("utf-8", errors="replace").strip()
                        messages = self.split_tcp_messages(raw)

                        response = None
                        for msg in messages:
                            msg = msg.strip()
                            if not msg: continue
                            response = self.process_message(msg)

                        if self.should_optimize():
                            moves = self.compute_optimal_positions()
                            if moves:
                                print("\n" + "*" * 60)
                                print("  [OPTIM] Ejecutando reposicionamiento")
                                print(f"  [OPTIM] Sim time: {self.grid.last_update_time:.2f}s")
                                print(f"  [OPTIM] Estaciones a mover: {len(moves)}")
                                print("*" * 60 + "\n")
                                move_cmds = [{'gnb_index': m['gnb_index'],
                                              'x': m['x'], 'y': m['y'], 'z': m['z'],
                                              'speed': m['speed']} for m in moves]
                                self.send_move_batch(move_cmds)
                            else:
                                print(f"[OPTIM] Sin movimientos necesarios "
                                      f"(t={self.grid.last_update_time:.1f}s)")
                                self._send_plain_response(response)
                            self.last_optimization_sim_time = self.grid.last_update_time
                        else:
                            self._send_plain_response(response)

    def _send_plain_response(self, response):
        if response:
            try:
                self.conn.sendall(json.dumps(response).encode("utf-8"))
            except Exception as e:
                print(f"[Servidor] Error enviando respuesta: {e}")
        else:
            try:
                self.conn.sendall(b"ACK")
            except Exception:
                pass

    def close(self):
        self.server_running = False
        if self.conn:
            try:
                self.conn.sendall(b"SERVER_CLOSED")
                self.conn.close()
            except Exception:
                pass

    # ----------------------------------------------------------------- reporting
    def save_graphml(self):
        Path(RESULTS_DIR).mkdir(exist_ok=True)
        fp = Path(RESULTS_DIR) / f"digital_twin_{SESSION_TIMESTAMP}.graphml"
        nx.write_graphml(self.G, str(fp))
        print(f"[DigitalTwin] GraphML saved to {fp}")

    def get_summary_report(self):
        stats = self.grid.get_coverage_statistics()
        qos = self.grid.get_qos_distribution()
        elapsed = None
        if self.start_time and self.grid.last_update_time:
            elapsed = self.grid.last_update_time - self.start_time
        ue_info = {}
        for ue_id, records in self.ue_data.items():
            if not records: continue
            latest = records[-1]
            sinr_values = [r['sinr'] for r in records if r['sinr'] > -999]
            ue_info[ue_id] = {
                'ue_index': latest['ue_index'],
                'position': (latest['x'], latest['y'], latest['z']),
                'current_sinr': latest['sinr'],
                'avg_sinr': sum(sinr_values) / len(sinr_values) if sinr_values else -999,
                'master_id': latest['master_id'],
                'total_measurements': len(records)}
        gnb_info = {}
        for gnb_id, records in self.gnb_data.items():
            if not records: continue
            latest = records[-1]
            gnb_info[gnb_id] = {
                'index': latest['index'],
                'position': (latest['x'], latest['y'], latest['z']),
                'connected_ues': latest['connected_ues'],
                'num_connected': latest['num_connected'],
                'total_records': len(records)}
        self.detect_coverage_holes()
        return {
            'timestamp': datetime.now().isoformat(),
            'session': SESSION_TIMESTAMP,
            'optimizer': OPTIMIZER_NAME,
            'simulation_time': {
                'start': self.start_time,
                'current': self.grid.last_update_time,
                'elapsed': elapsed},
            'ues': {
                'count': len(self.ue_data),
                'total_measurements': sum(len(m) for m in self.ue_data.values()),
                'details': ue_info},
            'gnbs': gnb_info,
            'coverage': stats,
            'qos': qos,
            'coverage_holes': self.hole_detector.get_summary(),
            'move_commands': self.move_commands_log}

    def print_summary(self):
        report = self.get_summary_report()
        print("\n" + "="*60)
        print("   COVERAGE ANALYSIS REPORT")
        print("="*60)
        if report['simulation_time']['elapsed']:
            print(f"Simulation Time: {report['simulation_time']['elapsed']:.2f} s")
        print("-"*60)
        print(f"Active UEs: {report['ues']['count']}  |  "
              f"Total Measurements: {report['ues']['total_measurements']}")
        if report['ues']['details']:
            for ue_id, info in report['ues']['details'].items():
                print(f"  UE[{info['ue_index']}] (id={ue_id}):")
                print(f"    Position : ({info['position'][0]:.1f}, "
                      f"{info['position'][1]:.1f}, {info['position'][2]:.1f})")
                print(f"    SINR     : {info['current_sinr']:.2f} dB "
                      f"(avg: {info['avg_sinr']:.2f} dB)")
                print(f"    Serving  : gNodeB macNodeId={info['master_id']}")
                print(f"    Samples  : {info['total_measurements']}")
        if report['gnbs']:
            print("-"*60)
            print("gNodeB Status:")
            for gnb_id, info in report['gnbs'].items():
                print(f"  gNodeB[{info['index']}] (macNodeId={gnb_id}):")
                print(f"    Position: ({info['position'][0]:.1f}, "
                      f"{info['position'][1]:.1f}, {info['position'][2]:.1f})")
                print(f"    Connected UEs: {info['num_connected']} "
                      f"{info['connected_ues']}")
        print("-"*60)
        print(f"COVERAGE (total area): {report['coverage']['coverage_percentage']:.2f}%")
        print(f"  Covered cells: {report['coverage']['covered_cells']} / "
              f"{report['coverage']['total_cells']}")
        print(f"  Measured cells: {report['coverage']['measured_cells']} "
              f"({report['coverage']['measurement_percentage']:.1f}%)")
        print(f"  Avg SINR (covered): {report['coverage']['avg_sinr_covered']:.2f} dB")
        print("-"*60)
        print("QoS Distribution (measured cells):")
        for level, data in report['qos'].items():
            bar = "#" * int(data['percentage'] / 5)
            print(f"  {level:15s}: {data['percentage']:6.2f}% "
                  f"({data['cells']:3d} cells) {bar}")
        print("-"*60)
        print("gNodeB Cell Distribution (measured cells):")
        for gnb_id, data in report['coverage']['gnb_distribution'].items():
            print(f"  gNodeB macNodeId={gnb_id}: "
                  f"{data['percentage']:6.2f}% ({data['cells']} cells)")
        if self.move_commands_log:
            print("-"*60)
            print(f"MOVE Commands sent: {len(self.move_commands_log)}")
            for i, entry in enumerate(self.move_commands_log):
                cmd = entry['command']
                dist = entry.get('distance', -1)
                eta = entry.get('eta', -1)
                reason = entry.get('reason', 'manual')
                print(f"  #{i+1}: gNodeB[{cmd['gnb_index']}] -> "
                      f"({cmd['x']:.1f}, {cmd['y']:.1f}, {cmd['z']:.1f}) "
                      f"@ {cmd.get('speed', '?')} m/s "
                      f"dist={dist:.0f}m ETA={eta:.1f}s "
                      f"sim_t={entry['sim_time']:.2f}s [{reason}]")
        self.hole_detector.print_summary()

    def save_report(self):
        filename = f"coverage_report_{SESSION_TIMESTAMP}.json"
        Path(RESULTS_DIR).mkdir(exist_ok=True)
        filepath = Path(RESULTS_DIR) / filename
        report = self.get_summary_report()
        with open(filepath, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        print(f"[CoverageAnalyzer] Report saved to {filepath}")
        return filepath


# ===============================================================================
#   PLOTS
# ===============================================================================

def plot_topology(dt):
    fig, ax = plt.subplots(figsize=(14, 12))
    sim_t = f" - t={dt.grid.last_update_time:.1f}s" if dt.grid.last_update_time else ""
    ax.set_title(f"Gemelo Digital 5G - Topologia{sim_t}", fontsize=14, fontweight='bold')
    G = dt.G
    QC = DigitalTwin.QOS_COLORS

    sinr_disp = np.where(dt.grid.sinr_map == -999, np.nan, dt.grid.sinr_map)
    im = ax.imshow(sinr_disp, cmap='RdYlGn', origin='upper',
                   extent=[0, AREA_SIZE_X, AREA_SIZE_Y, 0],
                   vmin=-20, vmax=30, alpha=0.2, aspect='auto')
    plt.colorbar(im, ax=ax, label='SINR (dB)', shrink=0.8)

    for u, v, d in G.edges(data=True):
        xu, yu = G.nodes[u]['pos_x'], G.nodes[u]['pos_y']
        xv, yv = G.nodes[v]['pos_x'], G.nodes[v]['pos_y']
        lt = d.get('link_type', '')
        if lt == 'x2':
            ax.plot([xu, xv], [yu, yv], color='#546E7A', ls='--', lw=1.5, alpha=0.4, zorder=2)
        elif lt == 'serving':
            qos_color = QC.get(d.get('qos', ''), '#999999')
            ax.plot([xu, xv], [yu, yv], color=qos_color, ls='-', lw=2.0, alpha=0.7, zorder=3)

    for n in G:
        nd = G.nodes[n]
        if nd.get('node_type') != 'gnb': continue
        x, y = nd['pos_x'], nd['pos_y']
        ax.scatter(x, y, s=600, c='#D32F2F', marker='s', zorder=10,
                   edgecolors='black', linewidth=2.0)
        ax.text(x, y, 'B', fontsize=11, ha='center', va='center',
                color='white', fontweight='bold', zorder=11)
        label = f"gNB[{nd['gnb_index']}]\n{nd['num_connected']} UEs\n({x:.0f}, {y:.0f})"
        ax.annotate(label, (x, y), fontsize=7, ha='center', va='top',
                    xytext=(0, -24), textcoords='offset points', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', fc='#FFCDD2', ec='#D32F2F', alpha=0.95),
                    zorder=12)

    for n in G:
        nd = G.nodes[n]
        if nd.get('node_type') != 'ue': continue
        x, y = nd['pos_x'], nd['pos_y']
        color = QC.get(nd.get('qos', ''), '#999999')
        ax.scatter(x, y, s=250, c=color, marker='o', zorder=8,
                   edgecolors='black', linewidth=1.2)
        ax.text(x, y, 'U', fontsize=8, ha='center', va='center',
                color='black', fontweight='bold', zorder=9)
        label = (f"UE_{nd['ue_index']}\n{nd['sinr']:.1f}dB ({nd['qos']})\n"
                 f"->gNB_{nd['master_id']}  HO:{nd['handovers']}")
        ax.annotate(label, (x, y), fontsize=6, ha='center', va='bottom',
                    xytext=(0, 16), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=color, alpha=0.9, lw=1.5),
                    zorder=10)

    legend = [
        Line2D([0], [0], marker='s', color='w', markerfacecolor='#D32F2F',
               markersize=14, markeredgecolor='black', label='gNodeB'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#00C853',
               markersize=10, markeredgecolor='black', label='UE (Excellent)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#64DD17',
               markersize=10, markeredgecolor='black', label='UE (Good)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#FFD600',
               markersize=10, markeredgecolor='black', label='UE (Fair)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#FF6D00',
               markersize=10, markeredgecolor='black', label='UE (Poor)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='#D50000',
               markersize=10, markeredgecolor='black', label='UE (No Service)'),
        Line2D([0], [0], color='#00C853', lw=2, label='Serving (buena)'),
        Line2D([0], [0], color='#D50000', lw=2, label='Serving (mala)'),
        Line2D([0], [0], color='#546E7A', lw=1.5, ls='--', label='X2'),
    ]
    ax.legend(handles=legend, bbox_to_anchor=(1.15, 1), loc='upper left',
              fontsize=8, framealpha=0.9, borderaxespad=0.)

    ax.set_xlim(-50, AREA_SIZE_X + 50)
    ax.set_ylim(AREA_SIZE_Y + 50, -50)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True, alpha=0.15, ls='--')

    st = dt.grid.get_coverage_statistics()
    ax.text(0.02, 0.02,
            f"Cobertura: {st['coverage_percentage']:.1f}%\n"
            f"SINR medio: {st['avg_sinr_covered']:.1f} dB\n"
            f"Medidas: {st['total_measurements']}",
            transform=ax.transAxes, fontsize=9, va='bottom',
            bbox=dict(boxstyle='round', fc='wheat', alpha=0.9))
    plt.tight_layout()
    plt.show(block=False)
    plt.pause(0.1)


def plot_coverage_grid(coverage_grid, title="Coverage Map", save_path=None):
    fig, ax = plt.subplots(figsize=(12, 10))
    display_map = np.full((coverage_grid.grid_rows, coverage_grid.grid_cols), -1)
    measured_mask = coverage_grid.measurement_count > 0
    display_map[measured_mask & (coverage_grid.coverage_map == 1)] = 1
    display_map[measured_mask & (coverage_grid.coverage_map == 0)] = 0
    cmap = ListedColormap(['lightgray', 'red', 'lightgreen'])
    bounds = [-1.5, -0.5, 0.5, 1.5]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
    im = ax.imshow(display_map, cmap=cmap, norm=norm, origin='upper',
                   extent=[0, AREA_SIZE_X, AREA_SIZE_Y, 0], alpha=0.8)
    cbar = plt.colorbar(im, ax=ax, ticks=[-1, 0, 1])
    cbar.ax.set_yticklabels(['Sin mediciones', 'Sin Cobertura', 'Con Cobertura'])
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    stats = coverage_grid.get_coverage_statistics()
    textstr = (f"Cobertura: {stats['coverage_percentage']:.1f}%\n"
               f"Celdas cubiertas: {stats['covered_cells']}/{stats['total_cells']}\n"
               f"SINR promedio: {stats['avg_sinr_covered']:.1f} dB")
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=11,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
    plt.tight_layout()
    if save_path:
        Path(PLOTS_DIR).mkdir(exist_ok=True)
        plt.savefig(Path(PLOTS_DIR) / save_path, dpi=300, bbox_inches='tight')
        print(f"[Visualization] Saved to {PLOTS_DIR}/{save_path}")
    plt.show(block=False)
    plt.pause(0.1)


def plot_sinr_heatmap(coverage_grid, title="SINR Heatmap", save_path=None):
    fig, ax = plt.subplots(figsize=(12, 10))
    sinr_map_clean = np.where(coverage_grid.sinr_map == -999, np.nan, coverage_grid.sinr_map)
    im = ax.imshow(sinr_map_clean, cmap='RdYlGn', origin='upper',
                   extent=[0, AREA_SIZE_X, AREA_SIZE_Y, 0], vmin=-20, vmax=30)
    plt.colorbar(im, ax=ax, label='SINR (dB)')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    if save_path:
        Path(PLOTS_DIR).mkdir(exist_ok=True)
        plt.savefig(Path(PLOTS_DIR) / save_path, dpi=300, bbox_inches='tight')
        print(f"[Visualization] Saved to {PLOTS_DIR}/{save_path}")
    plt.show(block=False)
    plt.pause(0.1)


def plot_coverage_holes(coverage_grid, hole_detector, title="Coverage Holes", save_path=None):
    fig, ax = plt.subplots(figsize=(14, 11))
    sinr_display = np.where(coverage_grid.sinr_map == -999, np.nan, coverage_grid.sinr_map)
    im = ax.imshow(sinr_display, cmap='RdYlGn', origin='upper',
                   extent=[0, coverage_grid.area_size_x, coverage_grid.area_size_y, 0],
                   vmin=-20, vmax=30, alpha=0.6)
    plt.colorbar(im, ax=ax, label='SINR (dB)', shrink=0.8)
    severity_colors = {'critical': '#FF0000', 'severe': '#FF6600',
                       'moderate': '#FFCC00', 'mild': '#99CC00'}
    for hole in hole_detector.holes:
        color = severity_colors.get(hole['severity'], '#888888')
        bb = hole['bounding_box']
        if hole['type'] == 'cluster':
            rect = plt.Rectangle((bb['x_min'], bb['y_min']),
                                 bb['x_max'] - bb['x_min'], bb['y_max'] - bb['y_min'],
                                 linewidth=2, edgecolor=color, facecolor=color, alpha=0.25)
            ax.add_patch(rect)
            ax.plot(hole['center']['x'], hole['center']['y'], 'x', color=color,
                    markersize=10, markeredgewidth=2)
            ax.annotate(f"{hole['severity']}\n{hole['area_m2']:.0f}m2\n{hole['avg_sinr']:.1f}dB",
                        (hole['center']['x'], hole['center']['y']), fontsize=7,
                        ha='center', va='bottom', color='white', fontweight='bold',
                        bbox=dict(boxstyle='round,pad=0.2', facecolor=color, alpha=0.8))
        else:
            ax.plot(hole['center']['x'], hole['center']['y'], 's', color=color,
                    markersize=6, alpha=0.7)
    legend_elements = [
        Patch(facecolor='#FF0000', alpha=0.5, label='Critical (SINR < -10 dB)'),
        Patch(facecolor='#FF6600', alpha=0.5, label='Severe (-10 <= SINR < 0 dB)'),
        Patch(facecolor='#FFCC00', alpha=0.5, label='Moderate (0 <= SINR < 13 dB)'),
        Patch(facecolor='#99CC00', alpha=0.5, label='Mild (13 <= SINR < 20 dB)')]
    ax.legend(handles=legend_elements, loc='lower right', fontsize=9)
    summary = hole_detector.get_summary()
    textstr = (f"Coverage Holes: {summary['total_holes']}\n"
               f"Cells affected: {summary['total_cells_affected']}\n"
               f"Area affected: {summary['total_area_m2']:.0f} m2")
    ax.text(0.02, 0.98, textstr, transform=ax.transAxes, fontsize=11,
            verticalalignment='top', color='white',
            bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
    ax.grid(True, alpha=0.2, linestyle='--')
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    plt.tight_layout()
    if save_path:
        Path(PLOTS_DIR).mkdir(exist_ok=True)
        plt.savefig(Path(PLOTS_DIR) / save_path, dpi=300, bbox_inches='tight')
        print(f"[Visualization] Saved to {PLOTS_DIR}/{save_path}")
    plt.show(block=False)
    plt.pause(0.1)


# ===============================================================================
#   HILO DE ANALISIS
# ===============================================================================

def analysis_thread(dt):
    last_analysis_time = 0
    while dt.server_running:
        time.sleep(1.0)
        now = time.time()
        if ENABLE_REALTIME_ANALYSIS and (now - last_analysis_time >= ANALYSIS_INTERVAL):
            if dt.grid.last_update_time is not None:
                dt.detect_coverage_holes()
                dt.print_summary()
            last_analysis_time = now


# ===============================================================================
#   MAIN
# ===============================================================================

def main():
    Path(RESULTS_DIR).mkdir(exist_ok=True)
    Path(PLOTS_DIR).mkdir(exist_ok=True)

    dt = DigitalTwin()

    # Servidor TCP (metodo de la clase) en hilo secundario
    tcp_thread = threading.Thread(target=dt.run_tcp_server, daemon=True)
    tcp_thread.start()

    # Analisis en hilo secundario
    an_thread = threading.Thread(target=analysis_thread, args=(dt,), daemon=True)
    an_thread.start()

    def on_sigint(sig, frame):
        print("\n[Servidor] Cerrando servidor...")
        dt.server_running = False

    signal.signal(signal.SIGINT, on_sigint)

    last_topology_time = 0
    print("[Main] Bucle principal activo. Ctrl+C para finalizar.\n")

    while dt.server_running:
        try:
            now = time.time()
            if ENABLE_REALTIME_TOPOLOGY and (now - last_topology_time >= TOPOLOGY_UPDATE_INTERVAL):
                if dt.grid.last_update_time is not None:
                    try:
                        plt.close('all')
                        plot_topology(dt)
                    except Exception as e:
                        print(f"[Plot] Error topologia: {e}")
                last_topology_time = now
            try:
                plt.pause(0.5)
            except Exception:
                time.sleep(0.5)
        except KeyboardInterrupt:
            dt.server_running = False
            break

    if ENABLE_FINAL_REPORT:
        print("\n" + "="*60)
        print("GENERANDO REPORTE FINAL")
        print("="*60)
        dt.detect_coverage_holes()
        dt.print_summary()
        dt.save_report()
        dt.save_graphml()
        print("\n[Servidor] Generando visualizaciones...")
        plt.close('all')
        plot_coverage_grid(dt.grid, "Mapa de Cobertura Final",
                           f"coverage_map_{SESSION_TIMESTAMP}.png")
        plot_sinr_heatmap(dt.grid, "Mapa SINR Final",
                          f"sinr_heatmap_{SESSION_TIMESTAMP}.png")
        plot_coverage_holes(dt.grid, dt.hole_detector, "Coverage Holes Detectados",
                            f"coverage_holes_{SESSION_TIMESTAMP}.png")
        print(f"\n[Servidor] Resultados en: {RESULTS_DIR} / {PLOTS_DIR}")
        print(f"[Servidor] Session: {SESSION_TIMESTAMP}")
        input("\nPresiona Enter para cerrar...")
        plt.close('all')

    dt.close()
    sys.exit(0)


if __name__ == "__main__":
    main()
