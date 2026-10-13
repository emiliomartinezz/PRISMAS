"""
dt_link.py — enlace del plugin SUMO-UAV-Py con el gemelo digital.

El gemelo escucha en un puerto aparte del que usa OMNeT++ (por defecto el
50001) y envia ordenes de reposicionamiento. El plugin se conecta como
cliente, lee sin bloquear en cada paso de simulacion y devuelve las ordenes
ya listas para aplicar.

Formato que manda el gemelo (una linea JSON por mensaje, terminada en \n):

    {"type": "MOVE_BATCH",
     "moves": [{"gnb_index": 1, "x": 562.7, "y": 689.0, "z": 60.0, "speed": 20.0}, ...]}

Las coordenadas vienen en el sistema de OMNeT++, que Veins obtiene a partir
del de SUMO invirtiendo el eje Y y sumando un margen. Aqui se deshace esa
transformacion antes de devolverlas.
"""

import socket
import errno

try:
    import ujson as json
except ImportError:
    import json


class DTLink:

    def __init__(self, host="127.0.0.1", port=50001, margin=250.0):
        self.host = host
        self.port = port
        self.margin = float(margin)
        self.sock = None
        self.connected = False
        self.buffer = b""

        # Limites de la red SUMO, para convertir coordenadas.
        # Se rellenan con set_net_boundary() una vez TraCI esta en marcha.
        self.net_xmin = 0.0
        self.net_ymax = 0.0

    # ------------------------------------------------------------------ setup

    def set_net_boundary(self, boundary):
        """boundary = traci.simulation.getNetBoundary() -> ((xmin,ymin),(xmax,ymax))"""
        (xmin, _ymin), (_xmax, ymax) = boundary
        self.net_xmin = float(xmin)
        self.net_ymax = float(ymax)
        print(f"[DTLink] Limites de la red: xmin={self.net_xmin:.2f} ymax={self.net_ymax:.2f}"
              f" (margin={self.margin})")

    def connect(self):
        """Conexion no bloqueante. Si el gemelo no esta escuchando, se avisa y
        la simulacion sigue sin ordenes."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setblocking(False)
            err = self.sock.connect_ex((self.host, self.port))
            if err in (0, errno.EINPROGRESS, errno.EWOULDBLOCK):
                print(f"[DTLink] Conectando con el gemelo en {self.host}:{self.port}")
                return True
            print(f"[DTLink] No se pudo conectar ({errno.errorcode.get(err, err)}). "
                  f"La simulacion seguira sin ordenes del gemelo.")
            self._drop()
            return False
        except Exception as e:
            print(f"[DTLink] Error creando el socket: {e}")
            self._drop()
            return False

    def _drop(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None
        self.connected = False

    # ------------------------------------------------------------- conversion

    def omnet_to_sumo(self, x_omnet, y_omnet):
        """Inversa de la transformacion de Veins:
              x_omnet = x_sumo - net_xmin + margin
              y_omnet = net_ymax - y_sumo + margin
        """
        x_sumo = x_omnet + self.net_xmin - self.margin
        y_sumo = self.net_ymax - y_omnet + self.margin
        return x_sumo, y_sumo

    # ----------------------------------------------------------------- lectura

    def poll(self):
        """Lee lo que haya llegado y devuelve una lista de ordenes ya
        convertidas: [{'uav_id': int, 'x': float, 'y': float, 'z': float,
        'speed': float}, ...]. Nunca bloquea."""
        if self.sock is None:
            return []

        # Confirmar que el connect() no bloqueante ha terminado
        if not self.connected:
            try:
                self.sock.getpeername()
                self.connected = True
                print("[DTLink] Conectado con el gemelo digital")
            except OSError:
                return []

        try:
            while True:
                chunk = self.sock.recv(8192)
                if not chunk:
                    print("[DTLink] El gemelo ha cerrado la conexion")
                    self._drop()
                    return []
                self.buffer += chunk
        except BlockingIOError:
            pass
        except OSError as e:
            print(f"[DTLink] Error leyendo del gemelo: {e}")
            self._drop()
            return []

        moves = []
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line.decode("utf-8"))
            except Exception as e:
                print(f"[DTLink] Mensaje ilegible descartado: {e}")
                continue
            if msg.get("type") != "MOVE_BATCH":
                continue
            for m in msg.get("moves", []):
                try:
                    x_sumo, y_sumo = self.omnet_to_sumo(float(m["x"]), float(m["y"]))
                    moves.append({
                        "uav_id": int(m["gnb_index"]),
                        "x": x_sumo,
                        "y": y_sumo,
                        "z": float(m.get("z", 0.0)),
                        "speed": float(m.get("speed", 0.0)),
                    })
                except (KeyError, ValueError, TypeError) as e:
                    print(f"[DTLink] Orden mal formada, descartada: {e}")
        return moves

    def close(self):
        self._drop()
