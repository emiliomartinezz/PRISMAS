
"""
plugin_link.py — enlace MQTT del gemelo digital.
 
Sustituye a los dos sockets TCP que usaba el gemelo:
  - el servidor del puerto 50000 por el que llegaba la telemetria de OMNeT++
  - el servidor del puerto 50001 por el que salian las ordenes al plugin
 
Ahora el gemelo es un cliente mas del broker MQTT:
 
    prismas/omnet/ue/<ue_index>/report     UE_Report   (OMNeT++ -> gemelo, QoS 0)
    prismas/omnet/gnb/<gnb_index>/report   GNB_Report  (OMNeT++ -> gemelo, QoS 0)
    prismas/dt/moves                       MOVE_BATCH  (gemelo -> plugin,  QoS 1)
 
 
Los mensajes son los mismos JSON que antes, uno por mensaje MQTT (MQTT ya
respeta los limites de cada mensaje, no hace falta separarlos a mano).
 
Para usar el broker del servidor en lugar del local, cambiar MQTT_HOST abajo
o lanzar el gemelo con:  PRISMAS_MQTT_HOST=<ip> python3 digital_twin_xxx.py
 
Requiere: pip install paho-mqtt  (version 2.x)
"""
 
import json
import os
import queue
import threading
 
import paho.mqtt.client as mqtt
 
# ------------------------------------------------------------ configuracion
MQTT_HOST = os.environ.get("PRISMAS_MQTT_HOST", "138.4.1.40")
MQTT_PORT = int(os.environ.get("PRISMAS_MQTT_PORT", "41883"))
TOPIC_PREFIX = os.environ.get("PRISMAS_MQTT_PREFIX", "prismas")
# Credenciales: SOLO por variables de entorno (fichero mqtt.env, fuera de git).
# Si no estan definidas se conecta sin usuario (broker local).
MQTT_USER = os.environ.get("PRISMAS_MQTT_USER", "")
MQTT_PASS = os.environ.get("PRISMAS_MQTT_PASS", "")
 
 
class PluginLink:
 
    def __init__(self, host=MQTT_HOST, port=MQTT_PORT, prefix=TOPIC_PREFIX,
                 max_queue=20000):
        self.host = host
        self.port = int(port)
        self.topic_telemetry = f"{prefix}/omnet/#"
        self.topic_moves = f"{prefix}/dt/moves"
 
        # Los reports llegan en el hilo de red de paho; se dejan en una cola y
        # el hilo del gemelo los procesa (asi el grafo solo lo toca un hilo).
        self._queue = queue.Queue(maxsize=max_queue)
        self._dropped = 0
        self._connected = threading.Event()
 
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id=f"{prefix}-gemelo-{os.getpid()}")
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self.client.reconnect_delay_set(min_delay=1, max_delay=5)
        if MQTT_USER:
            self.client.username_pw_set(MQTT_USER, MQTT_PASS)
 
    # ------------------------------------------------------------------ setup
 
    def start(self):
        """Conecta con el broker en segundo plano. No bloquea: si el broker
        aun no esta, paho reintenta solo."""
        auth = f"usuario '{MQTT_USER}'" if MQTT_USER else "sin credenciales"
        print(f"[PluginLink] Conectando al broker MQTT {self.host}:{self.port} ({auth})")
        self.client.connect_async(self.host, self.port, keepalive=30)
        self.client.loop_start()
 
    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"[PluginLink] El broker rechazo la conexion: {reason_code}")
            return
        # Suscribirse aqui para que se rehaga en cada reconexion
        client.subscribe(self.topic_telemetry, qos=0)
        self._connected.set()
        print(f"[PluginLink] Conectado al broker. Telemetria en '{self.topic_telemetry}', "
              f"ordenes en '{self.topic_moves}'")
 
    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        self._connected.clear()
        if reason_code.is_failure:
            print(f"[PluginLink] Conexion con el broker perdida ({reason_code}); reintentando...")
        else:
            print("[PluginLink] Desconectado del broker")
 
    def _on_message(self, client, userdata, msg):
        try:
            self._queue.put_nowait(msg.payload.decode("utf-8", errors="replace"))
        except queue.Full:
            self._dropped += 1
            if self._dropped % 1000 == 1:
                print(f"[PluginLink] Cola de telemetria llena: {self._dropped} "
                      f"mensajes descartados (el gemelo no da abasto)")
 
    @property
    def is_connected(self):
        """True si hay conexion con el broker. Con MQTT el gemelo no sabe si el
        plugin esta escuchando: solo que el broker recibe las ordenes."""
        return self._connected.is_set()
 
    # ----------------------------------------------------------------- lectura
 
    def get_telemetry(self, timeout=1.0):
        """Devuelve los reports (str JSON) recibidos desde la ultima llamada.
        Espera como mucho `timeout` segundos a que llegue el primero."""
        messages = []
        try:
            messages.append(self._queue.get(timeout=timeout))
        except queue.Empty:
            return messages
        while True:
            try:
                messages.append(self._queue.get_nowait())
            except queue.Empty:
                return messages
 
    # ------------------------------------------------------------------ envio
 
    def send_moves(self, move_list):
        """move_list: [{'gnb_index': i, 'x': ..., 'y': ..., 'z': ..., 'speed': ...}]"""
        if not move_list:
            return False
        if not self.is_connected:
            print("[PluginLink] Sin conexion con el broker: no se envian ordenes")
            return False
        payload = json.dumps({"type": "MOVE_BATCH", "moves": move_list})
        info = self.client.publish(self.topic_moves, payload, qos=1)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            print(f"[PluginLink] Error publicando ordenes: {mqtt.error_string(info.rc)}")
            return False
        return True
 
    def close(self):
        try:
            self.client.disconnect()
            self.client.loop_stop()
        except Exception:
            pass
        self._connected.clear()
 
