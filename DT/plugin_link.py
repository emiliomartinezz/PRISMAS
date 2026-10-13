"""
plugin_link.py — enlace del gemelo digital con el plugin SUMO-UAV-Py.

El gemelo ya tiene un servidor TCP en el 50000 por donde le llega la
telemetria desde OMNeT++. Este modulo abre un SEGUNDO puerto (50001) al que
se conecta el plugin, y es por ahi por donde salen ahora las ordenes de
reposicionamiento.

Se hace asi, y no reutilizando la conexion de OMNeT++, para que el lazo de
control no dependa de la simulacion de red: OMNeT++ observa, el plugin actua.

Se manda una linea JSON por mensaje, terminada en \n, para que el plugin
pueda separar mensajes de un flujo TCP sin ambiguedad.

Uso desde el gemelo:

    from plugin_link import PluginLink
    self.plugin = PluginLink()      # en __init__
    self.plugin.start()             # antes del bucle principal
    self.plugin.send_moves(moves)   # en lugar de self.conn.sendall(...)
    self.plugin.close()             # al terminar
"""

import json
import socket
import threading


class PluginLink:

    def __init__(self, host="0.0.0.0", port=50001):
        self.host = host
        self.port = port
        self.server = None
        self.conn = None
        self.addr = None
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ setup

    def start(self):
        """Abre el puerto y acepta conexiones del plugin en segundo plano.
        Si el plugin se desconecta, vuelve a quedarse esperando."""
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        self.server.settimeout(1.0)

        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()
        print(f"[PluginLink] Escuchando ordenes hacia el plugin en {self.host}:{self.port}")

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                conn, addr = self.server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            if self.conn is not None:
                try:
                    self.conn.close()
                except Exception:
                    pass
            self.conn = conn
            self.addr = addr
            print(f"[PluginLink] Plugin conectado desde {addr}")

    @property
    def is_connected(self):
        return self.conn is not None

    # ------------------------------------------------------------------ envio

    def send_moves(self, move_list):
        """move_list son los mismos diccionarios que se mandaban a OMNeT++:
        {'gnb_index': i, 'x': ..., 'y': ..., 'z': ..., 'speed': ...}"""
        if not move_list:
            return False
        if self.conn is None:
            print("[PluginLink] El plugin no esta conectado: no se envian ordenes")
            return False

        payload = json.dumps({"type": "MOVE_BATCH", "moves": move_list}) + "\n"
        try:
            self.conn.sendall(payload.encode("utf-8"))
            return True
        except OSError as e:
            print(f"[PluginLink] Error enviando al plugin: {e}")
            try:
                self.conn.close()
            except Exception:
                pass
            self.conn = None
            return False

    def close(self):
        self._stop.set()
        for s in (self.conn, self.server):
            if s is not None:
                try:
                    s.close()
                except Exception:
                    pass
        self.conn = None
        self.server = None
