"""
dt_link.py — enlace MQTT del plugin SUMO-UAV-Py con el gemelo digital.

Antes el plugin se conectaba por TCP al puerto 50001 del gemelo. Ahora se
suscribe en el broker MQTT al topic donde el gemelo publica las ordenes:

    prismas/dt/moves   (QoS 1)

    {"type": "MOVE_BATCH",
     "moves": [{"gnb_index": 1, "x": 562.7, "y": 689.0, "z": 60.0, "speed": 20.0}, ...]}
    {"type": "DEPLOY", "gnb_index": 2, "x": 900.0, "y": 400.0, "z": 0.0, "speed": 20.0}
    {"type": "RECALL", "gnb_index": 2}

poll() devuelve una lista de ordenes, en el orden en que llegaron, cada una
con la clave "cmd":

    {"cmd": "MOVE",   "uav_id", "x", "y", "z", "speed"}
    {"cmd": "DEPLOY", "uav_id", "x", "y", "z", "speed"}
    {"cmd": "RECALL", "uav_id"}

Las coordenadas vienen en el sistema de OMNeT++, que Veins obtiene a partir
del de SUMO invirtiendo el eje Y y sumando un margen. Aqui se deshace esa
transformacion antes de devolverlas.

Requiere: pip install paho-mqtt  (version 2.x)
"""

import os
import queue
import threading

try:
    import ujson as json
except ImportError:
    import json

import paho.mqtt.client as mqtt


class DTLink:

    def __init__(self, host="127.0.0.1", port=1883, margin=250.0, prefix="prismas"):
        self.host = host
        self.port = int(port)
        self.margin = float(margin)
        self.topic_moves = f"{prefix}/dt/moves"
        self.client_id = f"{prefix}-plugin-{os.getpid()}"

        self.client = None
        self._queue = queue.Queue()
        self._connected = threading.Event()

        # Limites de la red SUMO, para convertir coordenadas.
        # Se rellenan con set_net_boundary() una vez TraCI esta en marcha.
        self.net_xmin = 0.0
        self.net_ymax = 0.0

        if self.port in (50000, 50001):
            print(f"[DTLink] AVISO: el puerto {self.port} es el del antiguo enlace TCP. "
                  f"Pon en config.json \"DT Port\": 1883 (puerto del broker MQTT)")

    @property
    def connected(self):
        return self._connected.is_set()

    # ------------------------------------------------------------------ setup

    def set_net_boundary(self, boundary):
        """boundary = traci.simulation.getNetBoundary() -> ((xmin,ymin),(xmax,ymax))"""
        (xmin, _ymin), (_xmax, ymax) = boundary
        self.net_xmin = float(xmin)
        self.net_ymax = float(ymax)
        print(f"[DTLink] Limites de la red: xmin={self.net_xmin:.2f} ymax={self.net_ymax:.2f}"
              f" (margin={self.margin})")

    def connect(self):
        """Conexion en segundo plano. Si el broker no esta, paho reintenta solo
        y la simulacion sigue sin ordenes mientras tanto."""
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                      client_id=self.client_id)
            self.client.on_connect = self._on_connect
            self.client.on_disconnect = self._on_disconnect
            self.client.on_message = self._on_message
            self.client.reconnect_delay_set(min_delay=1, max_delay=5)
            # Credenciales SOLO por variables de entorno (mqtt.env, fuera de git)
            user = os.environ.get("PRISMAS_MQTT_USER", "")
            if user:
                self.client.username_pw_set(user, os.environ.get("PRISMAS_MQTT_PASS", ""))
            self.client.connect_async(self.host, self.port, keepalive=30)
            self.client.loop_start()
            auth = f"usuario '{user}'" if user else "sin credenciales"
            print(f"[DTLink] Conectando al broker MQTT {self.host}:{self.port} ({auth})")
            return True
        except Exception as e:
            print(f"[DTLink] Error creando el cliente MQTT: {e}. "
                  f"La simulacion seguira sin ordenes del gemelo.")
            self.client = None
            return False

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"[DTLink] El broker rechazo la conexion: {reason_code}")
            return
        client.subscribe(self.topic_moves, qos=1)
        self._connected.set()
        print(f"[DTLink] Conectado al broker. Esperando ordenes en '{self.topic_moves}'")

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        self._connected.clear()
        if reason_code.is_failure:
            print(f"[DTLink] Conexion con el broker perdida ({reason_code}); reintentando...")
        else:
            print("[DTLink] Desconectado del broker")

    def _on_message(self, client, userdata, msg):
        # Hilo de red de paho: solo se encola, el parseo se hace en poll()
        self._queue.put(msg.payload)

    # ------------------------------------------------------------- conversion

    def omnet_to_sumo(self, x_omnet, y_omnet):
        """Inversa de la transformacion de Veins:
              x_omnet = x_sumo - net_xmin + margin
              y_omnet = net_ymax - y_sumo + margin
        """
        x_sumo = x_omnet + self.net_xmin - self.margin
        y_sumo = self.net_ymax - y_omnet + self.margin
        return x_sumo, y_sumo

    def _target(self, cmd, m):
        x_sumo, y_sumo = self.omnet_to_sumo(float(m["x"]), float(m["y"]))
        return {
            "cmd": cmd,
            "uav_id": int(m["gnb_index"]),
            "x": x_sumo,
            "y": y_sumo,
            "z": float(m.get("z", 0.0)),
            "speed": float(m.get("speed", 0.0)),
        }

    # ----------------------------------------------------------------- lectura

    def parse_payload(self, payload):
        """Convierte un mensaje MQTT del gemelo en una lista de ordenes."""
        try:
            text = payload.decode("utf-8") if isinstance(payload, bytes) else payload
            msg = json.loads(text)
        except Exception as e:
            print(f"[DTLink] Mensaje ilegible descartado: {e}")
            return []

        out = []
        mtype = msg.get("type")
        try:
            if mtype == "MOVE_BATCH":
                for m in msg.get("moves", []):
                    try:
                        out.append(self._target("MOVE", m))
                    except (KeyError, ValueError, TypeError) as e:
                        print(f"[DTLink] Orden MOVE mal formada, descartada: {e}")
            elif mtype == "DEPLOY":
                out.append(self._target("DEPLOY", msg))
            elif mtype == "RECALL":
                out.append({"cmd": "RECALL", "uav_id": int(msg["gnb_index"])})
        except (KeyError, ValueError, TypeError) as e:
            print(f"[DTLink] Orden {mtype} mal formada, descartada: {e}")
        return out

    def poll(self):
        """Devuelve las ordenes llegadas desde la ultima llamada, ya convertidas
        a coordenadas SUMO. Nunca bloquea."""
        commands = []
        while True:
            try:
                payload = self._queue.get_nowait()
            except queue.Empty:
                break
            commands.extend(self.parse_payload(payload))
        return commands

    def close(self):
        if self.client is not None:
            try:
                self.client.disconnect()
                self.client.loop_stop()
            except Exception:
                pass
        self.client = None
        self._connected.clear()