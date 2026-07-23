# `dt_bridge.py` — Puente entre SUMO-UAV-Py y el Gemelo Digital

## 1. Propósito del módulo

`dt_bridge.py` es el **adaptador de comunicación** entre el simulador de red de drones (`main_.py`, basado en SUMO/TraCI) y el **Gemelo Digital** (los scripts `digital_twin_*.py`), que fueron desarrollados originalmente para trabajar con OMNeT++/Simu5G.

La idea de diseño es un reparto de responsabilidades claro:

- **El Gemelo Digital decide *dónde* deben estar los UAVs** (analiza cobertura/SINR y calcula la reubicación óptima).
- **SUMO-UAV-Py decide *cómo y si* pueden llegar ahí** (velocidad real, consumo de batería, retorno a base, relevo entre UAVs, viento).

Para que el Gemelo Digital funcione **sin modificarse**, `dt_bridge.py` traduce el estado de la simulación SUMO al mismo formato JSON que ya esperaba (mensajes `GNB_Report` para UAVs y `UE_Report` para vehículos/usuarios), y traduce las respuestas (`MOVE` / `MOVE_BATCH`) a comandos que `main_.py` puede aplicar.

El módulo contiene dos clases:

| Clase | Rol |
|---|---|
| `LinkBudgetEstimator` | Calcula la calidad de señal (SNR) entre un usuario en tierra y cada UAV |
| `DTBridge` | Cliente TCP que envía telemetría al Gemelo Digital y recibe órdenes de movimiento |

---

## 2. `LinkBudgetEstimator`: estimación de la señal

Modela la propagación radio con el **modelo log-distance**, el mismo que usa `coverage_optimizer.py`:

```
PL(d)  = PL(d0) + 10·n·log10(d / d0)
PL(d0) = 20·log10(4·π·d0·f / c)      (pérdida en espacio libre a d0 = 1 m)
Prx    = Ptx − PL(d)
N      = −174 dBm/Hz + 10·log10(B)
SNR    = Prx − N
```

Parámetros configurables en el constructor:

- `tx_power_dbm`: potencia de transmisión del UAV.
- `freq_ghz`: frecuencia portadora.
- `path_loss_exp` (`n`): exponente de pérdida de propagación.
- `bandwidth_mhz`: ancho de banda (afecta al ruido térmico).

**Métodos clave:**

- `snr(ue_xy, uav_xyz)` → calcula el SNR en dB entre un punto en tierra (altura fija `UE_HEIGHT = 1.5 m`) y un UAV en `(x, y, z)`, usando distancia 3D.
- `best_server(ue_xy, uav_map)` → recorre todos los UAVs **en vuelo** (`airborne=True`) y devuelve el `gnb_id` del que ofrece mejor señal junto con su SNR. Este es el criterio de "a qué UAV está conectado cada vehículo".

---

## 3. `DTBridge`: el cliente TCP

### 3.1 Construcción

```python
bridge = DTBridge.from_config(sim.config)
```

`from_config()` lee del `config.json` de SUMO-UAV-Py:

| Clave en `config.json` | Uso |
|---|---|
| `DT Host`, `DT Port` | Dirección del servidor del Gemelo Digital (por defecto `127.0.0.1:50000`) |
| `DT Report Interval (s)` | Cada cuánto tiempo de simulación se envía telemetría |
| `tx power dbm`, `Carrier frequency (GHz)`, `Path loss Exponent`, `Bandwidth (MHz)` | Parámetros del `LinkBudgetEstimator` |

### 3.2 Conexión: `connect()`

- Intenta conectar por socket TCP al Gemelo Digital, **reintentando** hasta `max_retries` veces (por defecto 30, con 1 s de espera entre intentos) — esto permite arrancar `main_.py` aunque el proceso `digital_twin_*.py` todavía no esté escuchando.
- Si nunca logra conectar, lanza `RuntimeError`.
- Al conectar, lanza un **hilo en segundo plano** (`_rx_loop`) dedicado exclusivamente a **recibir** mensajes del Gemelo Digital, para no bloquear el bucle principal de simulación.

### 3.3 Recepción de comandos (hilo receptor)

El Gemelo Digital puede enviar varios objetos JSON **concatenados** en un mismo paquete TCP (no hay delimitador explícito). Por eso `_split_json_messages()` implementa un mini-parser que:

1. Busca el inicio de un objeto (`{`).
2. Cuenta llaves abiertas/cerradas (`depth`) para encontrar dónde termina cada objeto JSON completo.
3. Si un mensaje llega cortado a mitad, lo deja en un buffer (`remaining`) para completarlo con el siguiente `recv()`.

Cada mensaje completo se pasa a `_handle_dt_message()`, que distingue:

- `"type": "MOVE"` → un único comando de movimiento → `_enqueue_move()`.
- `"type": "MOVE_BATCH"` → una lista de comandos (`moves` o `commands`) → se encola cada uno.
- Cualquier otro tipo (p. ej. `{"status": ...}`) se ignora silenciosamente.

`_enqueue_move()` valida y normaliza el comando a un diccionario con `gnb_index`, `x`, `y`, `z`, `speed` (con valores por defecto si faltan), y lo añade a una cola thread-safe (`_cmd_queue` protegida por `_cmd_lock`). Si el mensaje viene mal formado, se descarta con un aviso por consola en lugar de romper la ejecución.

`main_.py` recoge estos comandos, de forma no bloqueante, llamando a:

```python
for cmd in bridge.pop_commands():
    ...  # aplica cmd['x'], cmd['y'], cmd['z'], cmd['speed'] al UAV correspondiente
```

`pop_commands()` vacía y devuelve toda la cola acumulada desde la última llamada.

### 3.4 Envío de telemetría: `maybe_send_reports()`

Se llama en cada paso de simulación de `main_.py`, pero **solo envía datos si ha pasado `report_interval` segundos** desde el último envío (control de frecuencia para no saturar la conexión).

Recibe:

- `uav_snapshot`: `{uav_id (int): {'x', 'y', 'z', 'state', 'battery'}}`
- `vehicle_snapshot`: `{veh_id (str): (x, y)}`

Pasos internos:

1. **Construye `uav_map`**: convierte cada UAV en un "gNodeB" del Gemelo Digital, con la convención `gnb_id = uav_id + 1`. Marca `airborne=False` si el UAV está en tierra (`z <= 0`) o en estado `CHARGING`.
2. **Aplica un offset de coordenadas** (`OFFSET_X`, `OFFSET_Y`) para que el sistema de referencia de SUMO coincida con el que espera el Gemelo Digital.
3. **Por cada vehículo (UE)**:
   - Calcula el mejor UAV servidor y el SNR con `LinkBudgetEstimator.best_server()`.
   - Asigna un ID numérico estable al vehículo (`_ue_numeric_id`), ya que SUMO usa IDs de texto y el Gemelo Digital espera enteros.
   - Genera un mensaje `UE_Report` con posición, SINR estimado y el UAV que lo sirve (`master_id`).
4. **Por cada UAV (gNodeB)**:
   - Genera un mensaje `GNB_Report` con su posición, la lista de UEs que tiene conectados (`connected_ues`), y campos extra no usados por el Gemelo Digital actual pero disponibles para futuros algoritmos: `uav_state` y `battery`.
5. Concatena todos los mensajes JSON (sin separador, igual que el parser del receptor espera) y los envía por el socket con `sendall()`.
6. Si falla el envío (`OSError`), marca `connected = False` para que el resto del sistema sepa que se perdió la conexión.

### 3.5 Cierre: `close()`

Detiene el hilo receptor (`_stop.set()`), marca `connected = False` y cierra el socket de forma ordenada (`shutdown` + `close`), capturando errores si el socket ya estaba cerrado por el otro extremo.

---

## 4. Flujo de comunicación resumido

```
main_.py (UAVSimulation)
   │
   │ 1. bridge = DTBridge.from_config(config)
   │ 2. bridge.connect()                         ──► TCP connect a DT_HOST:DT_PORT
   │
   │ (bucle de simulación, cada paso)
   │ 3. bridge.maybe_send_reports(...)            ──► JSON: UE_Report + GNB_Report ──► Gemelo Digital
   │
   │                              (hilo _rx_loop, en paralelo)
   │ 4. Gemelo Digital  ──► JSON: MOVE / MOVE_BATCH ──►  _handle_dt_message() ──► cola interna
   │
   │ 5. for cmd in bridge.pop_commands(): aplicar movimiento al UAV vía traci.vehicle.moveToXY
   │
   │ 6. bridge.close()                            ──► cierre ordenado del socket
```

## 5. Puntos de diseño a destacar

- **Desacoplamiento por JSON**: el Gemelo Digital no necesita saber que ahora los datos vienen de SUMO en vez de OMNeT++; solo ve el mismo esquema de mensajes de siempre.
- **Tolerancia a fallos de red**: reintentos de conexión al arrancar, manejo de mensajes JSON fragmentados o concatenados, y detección de desconexión (`connected = False`) sin tirar abajo la simulación.
- **Concurrencia simple y explícita**: un único hilo dedicado a recibir datos (`_rx_loop`), con una cola protegida por lock (`_cmd_lock`) como único punto de sincronización con el hilo principal — evita bloquear el bucle de simulación esperando datos de red.
- **Separación de responsabilidades**: `LinkBudgetEstimator` solo sabe de física de propagación; `DTBridge` solo sabe de protocolo/red. Ninguna de las dos clases conoce la lógica de vuelo de los UAVs (batería, estados, etc.), que vive en `main_.py`.
