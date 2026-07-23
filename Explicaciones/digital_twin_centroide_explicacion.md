# `digital_twin_centroide.py` — Gemelo Digital con reposicionamiento por centroide simple

## 1. Propósito del módulo

Este archivo implementa el **Gemelo Digital** de la red 5G/6G: un servidor TCP independiente (escucha en `0.0.0.0:50000`) que recibe la telemetría que le envía `dt_bridge.py` (`UE_Report` de cada vehículo, `GNB_Report` de cada UAV), mantiene un **grafo de red en tiempo real** con `networkx`, analiza la cobertura/calidad de servicio, y decide **hacia dónde debe moverse cada gNodeB (UAV)** para mejorar la cobertura — devolviendo comandos `MOVE_BATCH`.

Por la cabecera del propio archivo (`Autor: Lucas Vilariño — TFM MUIT-UPM`), este script es el **gemelo digital original**, pensado en su día para recibir datos de OMNeT++/Simu5G. `dt_bridge.py` lo alimenta con datos de SUMO usando exactamente el mismo formato de mensajes, así que este archivo funciona **sin saber que su origen de datos ha cambiado**.

Existen tres variantes casi idénticas de este archivo (`_centroide`, `_centroide_ponderado`, `_kmeans`); **solo cambia el algoritmo de la sección "reposicionamiento"** (aquí: `compute_optimal_positions`). Todo lo demás —grid de cobertura, detección de agujeros de cobertura, servidor TCP, gráficas, informes— es común a las tres.

El nombre de este algoritmo, definido en el propio código, es `OPTIMIZER_NAME = "Centroide simple"`.

Estructura del archivo:

| Componente | Tipo | Rol |
|---|---|---|
| `CoverageGrid` | clase | Rejilla espacial que acumula las medidas de SINR y cobertura |
| `CoverageHoleDetector` | clase | Detecta y caracteriza zonas sin buena cobertura ("agujeros") |
| `DigitalTwin` | clase | Núcleo: grafo de red, servidor TCP, algoritmo de reposicionamiento, informes |
| `plot_topology` / `plot_coverage_grid` / `plot_sinr_heatmap` / `plot_coverage_holes` | funciones | Visualización con `matplotlib` |
| `analysis_thread`, `main()` | funciones | Hilos de ejecución y arranque del programa |

---

## 2. Configuración común (constantes globales)

```python
HOST, PORT = "0.0.0.0", 50000          # servidor TCP
RESULTS_DIR, PLOTS_DIR = "./coverage_results", "./coverage_plots"

AREA_SIZE_X, AREA_SIZE_Y = 1410, 1212  # tamaño del área analizada (m)
GRID_CELL_SIZE = 100                    # tamaño de celda de la rejilla (m)
SINR_THRESHOLD_COVERAGE = 20            # dB: por encima de esto, celda "cubierta"

QOS_LEVELS = {'Excellent': 20, 'Good': 13, 'Fair': 0, 'Poor': -10, 'No Service': -999}

ENABLE_REALTIME_ANALYSIS, ANALYSIS_INTERVAL = True, 10.0    # resumen por consola cada 10 s reales
ENABLE_REALTIME_TOPOLOGY, TOPOLOGY_UPDATE_INTERVAL = True, 30.0  # repinta el grafo cada 30 s reales
ENABLE_FINAL_REPORT = True

# Algoritmo de reposicionamiento
ENABLE_TOPOLOGY_OPTIMIZATION = True
OPTIMIZATION_INTERVAL = 10.0        # segundos de simulación entre optimizaciones
OPTIMIZATION_MIN_DISTANCE = 50.0    # m: si el centroide está más cerca que esto, no se mueve
OPTIMIZATION_SPEED = 20.0           # m/s sugeridos al UAV (main_.py los ignora y usa la velocidad real)
OPTIMIZATION_MIN_UES = 1            # mínimo de UEs servidos para considerar mover el gNodeB
```

Nótese que `AREA_SIZE_X/Y` de este script (1410×1212, herencia del escenario original de Simu5G) **no coincide** con los límites `xmax/ymax` del escenario de Madrid en `config.json` de SUAVPy (1600×1750) — al fusionar ambos proyectos, la rejilla de cobertura queda referida al área original y no se ha reescalado al nuevo mapa. No afecta al algoritmo de reposicionamiento (que trabaja con coordenadas absolutas), pero sí a qué parte del mapa cubre realmente la rejilla/heatmap.

---

## 3. `CoverageGrid`: la rejilla de cobertura

Discretiza el área en celdas de `GRID_CELL_SIZE` metros y acumula, por celda:

- `coverage_map`: 1 si la última medida en esa celda superó `SINR_THRESHOLD_COVERAGE`, 0 si no.
- `sinr_map`: SINR **suavizado con media móvil exponencial** (`alpha = 0.3`): `sinr_nuevo = 0.3·medida + 0.7·sinr_anterior`. Así una medida puntual mala no "ensucia" de golpe el mapa.
- `cell_id_map`: qué gNodeB (`master_id`) sirvió la última medida de esa celda.
- `measurement_count`: cuántas veces se ha medido cada celda (permite distinguir "sin cobertura" de "sin datos").

Métodos de consulta:

- `get_coverage_percentage()`: % de celdas con `coverage_map == 1` sobre el total.
- `get_coverage_statistics()`: paquete completo — celdas cubiertas/no cubiertas/medidas/no medidas, SINR medio/mín/máx **calculado solo sobre celdas medidas** (para que zonas degradadas también cuenten), y distribución de celdas por gNodeB servidor.
- `get_qos_distribution()`: reparte las celdas medidas en 5 niveles (`Excellent/Good/Fair/Poor/No Service`) según los cortes de `QOS_LEVELS`.

---

## 4. `CoverageHoleDetector`: detección de agujeros de cobertura

Identifica y agrupa zonas del mapa con mala señal, en tres pasos (`detect()`):

### 4.1 `_identify_hole_cells`: candidatas a "agujero"

Para cada celda de la rejilla:

- Si está **medida** y su SINR está por debajo de `COVERAGE_THRESHOLD` (20 dB) → es un hueco de tipo `"measured"`.
- Si **no** está medida pero al menos 3 de sus 8 vecinas sí lo están y al menos 2 de esas vecinas tienen mala señal → se **infiere** un hueco de tipo `"inferred"`, con el SINR promediado de las vecinas medidas (`_avg_neighbor_sinr`). Esto permite marcar zonas por las que ningún UE ha pasado todavía pero que muy probablemente tienen mala cobertura, a partir del contexto.

Cada celda-candidata se clasifica en severidad (`_classify_severity`): `critical` (< -10 dB), `severe` (< 0 dB), `moderate` (< 13 dB) o `mild` (resto).

### 4.2 Agrupación en regiones (`_cluster_holes`)

Agrupa celdas contiguas con **DBSCAN** (`sklearn`, `eps=2` celdas, `min_samples=2`) usando como coordenadas la fila/columna de cada celda. Si `scikit-learn` no está instalado, cae automáticamente en `_cluster_holes_simple`: un BFS manual con la misma idea (distancia Manhattan ≤ `eps` → mismo clúster), evitando que el análisis falle por una dependencia opcional.

### 4.3 Caracterización (`_characterize_regions`)

Cada clúster (o celda aislada, `cluster_id == -1`) se convierte en una entrada con centro, área en m², número de celdas, SINR medio/mínimo, severidad "peor caso" (`_worst_severity`, prioriza `critical > severe > moderate > mild`) y `bounding_box`. La lista final se ordena por severidad y luego por área, así los huecos más importantes aparecen primero en los informes.

`get_summary()` / `print_summary()` agregan estos huecos en un resumen legible (totales por severidad, top 10 huecos), usado tanto en el análisis periódico como en el informe final.

---

## 5. `DigitalTwin`: el núcleo del gemelo digital

### 5.1 Estado interno (`__init__`)

- `self.G`: un `networkx.Graph()` que es, a la vez, **la estructura de datos central** (fuente de verdad de posiciones y relaciones) y lo que alimenta las gráficas y el `GraphML` exportado.
- `self.grid` / `self.hole_detector`: instancias de las dos clases anteriores.
- `self.ue_data` / `self.gnb_data`: históricos completos (listas de medidas) por UE y por gNodeB — se usan para calcular promedios, handovers, etc.
- `self.move_commands_log`: histórico de todos los `MOVE`/`MOVE_BATCH` enviados, con distancia y ETA estimadas.
- `self.conn`, `self.addr`, `self.server_running`: estado del socket TCP (aquí el servidor **vive dentro de la propia clase**, al contrario que en `dt_bridge.py` donde el cliente es una clase aparte).

### 5.2 Ingesta de telemetría

**`handleUE_ReportMessage(message)`** — por cada `UE_Report` recibido:

1. Actualiza `self.grid` con la posición/SINR de ese UE (`update_from_ue_measurement`).
2. Añade la medida al histórico `self.ue_data[ue_id]` y recalcula SINR medio/mín/máx y número de **handovers** (cambios de `master_id` entre medidas consecutivas).
3. Crea o actualiza el nodo `UE_{ue_index}` en el grafo `self.G`, con toda esta información como atributos.
4. Elimina la arista `"serving"` anterior de ese UE (si tenía una) y crea la nueva hacia `gNB_{master_id}` — así el grafo siempre refleja **a qué gNodeB está conectado cada UE ahora mismo**, no un histórico de conexiones.

**`handleGNB_ReportMessage(message)`** — por cada `GNB_Report`:

1. Guarda la medida en `self.gnb_data[gnb_id]`.
2. Crea o actualiza el nodo `gNB_{gnb_id}` en el grafo, con su posición y la lista de UEs conectados.
3. Si es la **primera vez** que se ve ese `gnb_id`, añade una arista tipo `"x2"` hacia **todos** los demás gNodeBs ya conocidos (`self._known_gnbs`) — modelando el interfaz X2 real de 5G entre estaciones base, y formando progresivamente un grafo completo entre todos los UAVs.

> Recordatorio de convención (fijada en `dt_bridge.py`): `gnb_id = uav_id + 1` (identificador "de red", como el `master_id` de los UEs) y `gnb_index = uav_id` (el que se usa en los comandos `MOVE`, porque es el que `main_.py` necesita para saber qué vehículo mover).

### 5.3 El algoritmo: `compute_optimal_positions` (centroide simple)

Este es el corazón de la variante "centroide":

```python
# 1. Agrupa los UEs por el gNodeB que los sirve (master_id)
gnb_serving_ues[master_id].append({'x':..., 'y':...})

# 2. Para cada gNodeB con al menos OPTIMIZATION_MIN_UES UEs servidos:
centroid_x = media(x de sus UEs)
centroid_y = media(y de sus UEs)
dist = distancia(posición_actual_del_gNodeB, centroide)

# 3. Solo genera un MOVE si el centroide está a más de OPTIMIZATION_MIN_DISTANCE
if dist >= OPTIMIZATION_MIN_DISTANCE:
    moves.append({gnb_index, x: centroid_x, y: centroid_y, z: altura_actual, speed: OPTIMIZATION_SPEED})
```

Es un algoritmo **puramente reactivo y geométrico**: no pondera por calidad de señal ni por tráfico, simplemente persigue el centro de masas de la demanda que ya tiene asociada. La altura (`z`) nunca se modifica — solo se reposiciona en el plano XY. El filtro `OPTIMIZATION_MIN_DISTANCE` evita "temblores": pequeños desplazamientos del centroide medida a medida no generan micro-movimientos constantes del UAV.

`should_optimize()` controla la cadencia: solo se ejecuta el algoritmo si han pasado `OPTIMIZATION_INTERVAL` segundos **de tiempo de simulación** (no de tiempo real) desde la última vez.

### 5.4 Envío de comandos: `send_move_batch`

Empaqueta la lista de movimientos como `{"type": "MOVE_BATCH", "moves": [...]}`, lo envía por el socket TCP activo (`self.conn`), y **registra** cada movimiento en `move_commands_log` calculando distancia y ETA estimadas a partir de la posición actual conocida del gNodeB en el grafo — esto es solo para los informes/depuración, no afecta al UAV (que en `main_.py` calcula su propio ETA con su velocidad real).

### 5.5 Servidor TCP (`run_tcp_server`, integrado en la clase)

- Socket con **timeout de 1 segundo** tanto en `accept()` como en `recv()`: así el bucle puede comprobar periódicamente `self.server_running` y cerrarse limpiamente (por ejemplo, tras `Ctrl+C`) en vez de quedar bloqueado esperando datos para siempre.
- `split_tcp_messages` (equivalente al parser de `dt_bridge.py`): separa varios JSON concatenados en un mismo paquete TCP contando llaves (`{`/`}`) para encontrar dónde termina cada mensaje.
- Por cada mensaje completo, `process_message` distingue `UE_Report` / `GNB_Report` (delegando en los `handle...` correspondientes) y devuelve un pequeño diccionario de estado (`{"status": "ok"}`, etc.).
- **Tras procesar los mensajes de cada paquete recibido**, comprueba `should_optimize()`:
  - Si toca optimizar y hay movimientos que enviar → `send_move_batch` (la respuesta al cliente **es** el `MOVE_BATCH`).
  - Si toca optimizar pero no hace falta mover nada, o si no toca optimizar todavía → responde con el `status` del último mensaje procesado o, si no hay nada que decir, un simple `b"ACK"` (`_send_plain_response`).

`close()` marca `server_running = False` y, si hay una conexión abierta, le envía un aviso de texto plano `b"SERVER_CLOSED"` antes de cerrar el socket.

### 5.6 Informes (`get_summary_report`, `print_summary`, `save_report`, `save_graphml`)

- `get_summary_report()` construye un único diccionario con: tiempo de simulación transcurrido, detalle por UE (posición, SINR actual/medio, gNodeB servidor, nº de medidas), detalle por gNodeB (posición, UEs conectados), estadísticas de cobertura y QoS (vía `CoverageGrid`), resumen de agujeros de cobertura, e histórico de comandos `MOVE` enviados.
- `print_summary()` vuelca ese informe por consola de forma legible (con barras de texto `#####` para la distribución de QoS).
- `save_report()` guarda ese mismo informe como JSON en `./coverage_results/coverage_report_<timestamp>.json`.
- `save_graphml()` exporta el grafo `networkx` completo a `./coverage_results/digital_twin_<timestamp>.graphml`, abrible en herramientas como Gephi para inspeccionar la topología fuera de este script.

---

## 6. Visualización (`plot_topology`, `plot_coverage_grid`, `plot_sinr_heatmap`, `plot_coverage_holes`)

Cuatro funciones de módulo (no pertenecen a `DigitalTwin`), todas con `matplotlib`:

| Función | Qué dibuja |
|---|---|
| `plot_topology` | El grafo completo: mapa de SINR de fondo, gNodeBs como cuadrados rojos, UEs como círculos coloreados por nivel de QoS, aristas `serving` (color según QoS) y `x2` (líneas discontinuas grises entre gNodeBs) |
| `plot_coverage_grid` | Mapa binario cubierto/no cubierto/sin datos (3 colores) |
| `plot_sinr_heatmap` | Mapa de calor continuo del SINR suavizado |
| `plot_coverage_holes` | Igual que el heatmap, pero superponiendo los rectángulos de los agujeros de cobertura detectados, coloreados por severidad, con su leyenda |

Todas aceptan un `save_path` opcional para guardar un PNG en `./coverage_plots/` además de mostrar la figura en pantalla (`plt.show(block=False)`).

---

## 7. Hilos de ejecución y `main()`

```
main()
 ├─ crea DigitalTwin()
 ├─ hilo "tcp_thread"  → dt.run_tcp_server()      (servidor TCP :50000, daemon)
 ├─ hilo "an_thread"   → analysis_thread(dt)       (resumen por consola cada 10s reales, daemon)
 ├─ registra manejador de SIGINT (Ctrl+C → server_running = False)
 └─ bucle principal (hilo principal):
       cada TOPOLOGY_UPDATE_INTERVAL (30s reales) → plot_topology(dt)
       plt.pause(0.5)   # cede tiempo a la GUI de matplotlib
```

- `analysis_thread`: cada segundo comprueba si toca análisis (`ANALYSIS_INTERVAL`, 10 s reales); si toca, ejecuta `detect_coverage_holes()` + `print_summary()`.
- El **refresco del grafo** (`plot_topology`) se hace deliberadamente en el **hilo principal**, no en un hilo secundario: `matplotlib` (backend `TkAgg`) no es thread-safe para dibujar, así que todo el pintado en pantalla pasa por el bucle principal de `main()`, mientras el servidor TCP y el análisis corren en segundo plano.
- Al salir del bucle (por `server_running = False`, disparado por SIGINT o por el propio cliente si cerrara la conexión) y si `ENABLE_FINAL_REPORT`, se genera un informe final completo: último `print_summary`, `save_report` (JSON), `save_graphml`, y las tres gráficas finales guardadas en disco — y se espera un `Enter` del usuario antes de cerrar todas las ventanas y terminar (`dt.close()` + `sys.exit(0)`).

---

## 8. Relación con el resto del proyecto

| Módulo | Relación |
|---|---|
| `dt_bridge.py` | Es su **único cliente**: se conecta al puerto 50000, le envía `UE_Report`/`GNB_Report` y recibe de vuelta `MOVE`/`MOVE_BATCH` |
| `main_.py` | Nunca habla directamente con este archivo — todo pasa a través de `DTBridge`. Es quien decide *si* puede ejecutar el `MOVE` recibido (ver `apply_dt_commands` en `main_.py`) |
| `digital_twin_centroide_ponderado.py` / `digital_twin_kmeans.py` | Variantes hermanas: mismo `CoverageGrid`, `CoverageHoleDetector`, servidor TCP e informes; solo cambia `compute_optimal_positions` (y el valor de `OPTIMIZER_NAME`) |

Este script se ejecuta como **proceso independiente** (`python3 digital_twin_centroide.py`), normalmente antes de arrancar `main_.py`, ya que este último se conecta a él como cliente TCP con reintentos.

---

## 9. Flujo resumido

```
Cliente (dt_bridge.py)                    DigitalTwin (este archivo)
        │                                          │
        │──── TCP connect :50000 ────────────────▶│  run_tcp_server() acepta conexión
        │                                          │
        │──── UE_Report / GNB_Report (JSON) ─────▶│  split_tcp_messages → process_message
        │                                          │     handleUE_ReportMessage → grid + grafo
        │                                          │     handleGNB_ReportMessage → grafo (+ aristas x2)
        │                                          │
        │                                          │  should_optimize()?
        │                                          │     sí, hay movimientos → compute_optimal_positions()
        │                                          │                            (centroide de UEs por gNodeB)
        │◀─── MOVE_BATCH (JSON) ───────────────────│     send_move_batch()
        │                                          │     no hace falta / no toca → ACK o status
        │                                          │
        │                                          │  (en paralelo) analysis_thread → resumen periódico
        │                                          │  (hilo principal) plot_topology cada 30s
        │                                          │
        │                                          │  Ctrl+C → informe final (JSON + GraphML + PNGs)
```

---

## 10. Puntos de diseño a destacar

- **El grafo `networkx` es el estado, no solo una vista**: posiciones, relaciones de servicio y topología X2 se leen y escriben directamente sobre `self.G`; tanto las gráficas como el algoritmo de centroide y el `GraphML` exportado beben de la misma fuente.
- **Resiliencia del socket con timeouts cortos**: tanto `accept()` como `recv()` usan `timeout=1.0`, lo que permite comprobar `server_running` con frecuencia y cerrar el servidor de forma ordenada ante `SIGINT`, en vez de quedarse bloqueado.
- **Robustez ante datos incompletos**: `CoverageHoleDetector` no solo marca huecos donde hay medidas malas, sino que **infiere** huecos en celdas nunca visitadas si su vecindario sugiere mala cobertura — útil porque los UEs (vehículos) solo miden a lo largo de sus propias rutas, dejando zonas del mapa sin ningún dato directo.
- **Degradación elegante del clustering**: si `scikit-learn` no está instalado, el detector de huecos no falla, simplemente usa una implementación equivalente más simple (`_cluster_holes_simple`).
- **El algoritmo de esta variante es intencionadamente simple**: no pondera por SINR, tráfico ni densidad, solo por posición geométrica de los UEs servidos — es el caso base con el que comparar las otras dos variantes (centroide ponderado y k-means).
- **Separación estricta hilo de red / hilo de dibujo**: el servidor TCP y el análisis corren en hilos `daemon` en segundo plano; todo lo que toca `matplotlib` vive en el hilo principal, evitando problemas de thread-safety con el backend `TkAgg`.
