# `main_.py` — Núcleo de la simulación (SUAVPy / `UAVSimulation`)

## 1. Propósito del módulo

`main_.py` es el **orquestador central** del proyecto. Contiene una única clase, `UAVSimulation`, que:

- Lee la configuración (`config.json`) y calcula el modelo de autonomía del UAV.
- Genera las trayectorias temporales de cada UAV (interpolación de movimiento y rotación).
- Arranca y controla la simulación de **SUMO/TraCI** (standalone o co-simulación con OMNeT++).
- Ejecuta el **bucle principal de simulación**, paso a paso, gestionando batería, relevos entre UAVs, visualización en el GUI de SUMO y registro de datos.
- Actúa como **cliente** de `coverage_optimizer.Optimizer` (posición inicial óptima) y de `dt_bridge.DTBridge` (comunicación con el Gemelo Digital).
- Expone dos vías de control externo: una **GUI local** en Tkinter y un **servidor TCP** en el puerto 1024.

Es, con diferencia, el módulo más grande del repositorio (~1300 líneas) porque concentra toda la lógica de negocio; el resto de módulos (`_utils`, `coverage_optimizer`, `dt_bridge`) son colaboradores a los que `main_.py` delega cálculos concretos.

---

## 2. Punto de entrada (`if __name__ == "__main__"`)

```python
root = tk.Tk(); root.withdraw()              # ventana Tkinter oculta (necesaria para los diálogos)
sim = UAVSimulation('config.json')

if sim.otimization_mode:
    optimizer = Optimizer(sim.config)
    optimizer.run(config_path="config.json", output_path="config_optimized.json")
    sim = UAVSimulation('config_optimized.json')   # se recarga con las posiciones ya optimizadas
    sim.optimized_snr_grid = optimizer.last_snr_grid
    ...

simulation_thread = threading.Thread(target=start_simulation_thread, args=(sim, root))
simulation_thread.start()
root.mainloop()          # el bucle de Tkinter debe vivir en el hilo principal
simulation_thread.join()
```

Puntos clave:

- Si `"Optimization Mode"` está activo, **antes de simular** se ejecuta `coverage_optimizer.Optimizer`, que decide las posiciones/alturas iniciales de los UAVs y escribe `config_optimized.json`. La simulación se relanza leyendo ese fichero ya optimizado.
- La simulación corre en un **hilo separado** (`start_simulation_thread`) precisamente porque Tkinter exige que su `mainloop()` se ejecute en el hilo principal del proceso.
- `start_simulation_thread()` simplemente encadena `sim.start_sumo()` → `sim.run_simulation()`, capturando `traci.exceptions.FatalTraCIError` (SUMO cerrado por el usuario) y cualquier excepción inesperada, y siempre cierra TraCI y para el `mainloop()` de Tkinter al terminar (`tk_root.quit()`).

---

## 3. Configuración y modelo de UAV (`read_config`)

Se ejecuta en el `__init__` y traduce `config.json` a atributos de la instancia. Algunos de los más relevantes:

| Clave de `config.json` | Atributo | Efecto |
|---|---|---|
| `Number of UAVs` | `self.num_UAVs` | Tamaño de la flota "primaria" |
| `Relay Mode` | `self.relay_mode` | Si es `True`, **duplica la flota**: `total_uavs = 2 × num_UAVs`. La segunda mitad son UAVs de repuesto (backup), copia exacta de los datos de la primera mitad, que empiezan en estado `STANDBY` |
| `DT Mode` | `self.dt_mode` | Activa la integración con el Gemelo Digital vía `dt_bridge` |
| `Battery Mode` | `self.battery_mode` | Activa todo el modelo de descarga de batería / retorno a base |
| `Uav Model` | `self.UavModel` | `"Mavic 2e"`, `"Mini 3 pro"` o `"Manual"` — ver tabla siguiente |
| `Remote Server` | `self.server_option` | Activa el servidor TCP (puerto 1024); si está activo, las trayectorias del `config.json` se **descartan** y se sustituyen por un único punto `[[0,0,0,0,0]]` por UAV, a la espera de que un cliente externo las rellene en tiempo real |
| `Local GUI` | `self.local_gui` | Activa el hilo de diálogos Tkinter para introducir posiciones manualmente |
| `Movement` | `self.movement` | `"Continuous"` (interpola rotar→mover→rotar) o `"Discrete"` (usa los waypoints tal cual, sin dinámica) |
| `Uav Mode` | `self.UavMode` | `"Hovering"`, `"Spinning"` o `"Sampling"` — cómo se comporta el yaw/FOV entre waypoints |
| `Wind Mode` / `Wind Speed` / `Wind Direction` | `self.wind_mode`, etc. | Activa el efecto del viento en velocidad relativa y consumo de batería |
| `Optimization Mode` | `self.otimization_mode` | Ejecuta `Optimizer` antes de simular y pinta el mapa de calor de SNR si el GUI está activo |

**Modelos de UAV** (`calculate_uav_rates` deriva las tasas de descarga a partir de la autonomía en tres regímenes):

| Modelo | Velocidad crucero | Velocidad máx. | Autonomía hovering / crucero / máx. |
|---|---|---|---|
| `Mavic 2e` | 7.0 m/s | 13.8 m/s | 29 / 31 / 21.6 min |
| `Mini 3 pro` | 6.0 m/s | 16.0 m/s | 30 / 34 / 25 min |
| `Manual` | configurable | configurable | configurable (`config.json`) |

`hover_rate` siempre vale `1.0` (referencia); `cruise_rate = t_hover / t_cruise` (normalmente < 1, volar en crucero gasta menos por paso que hacer hovering) y `max_rate = t_hover / t_max` (> 1, volar a máxima velocidad gasta más). `calculate_discharge_rate(velocity)` interpola linealmente entre `hover_rate` y `cruise_rate` por debajo de `v_cruise`, y de forma **cúbica** entre `cruise_rate` y `max_rate` por encima (para modelar que el arrastre aerodinámico penaliza mucho más al acercarse a la velocidad máxima).

Si `relay_mode` está activo, `self.original_uav_data` guarda una copia profunda de las trayectorias originales de **ambas mitades** de la flota: sirve de "plan de referencia" al que se vuelve tras un relevo o una carga.

---

## 4. Generación de trayectorias (`uav_path_data`)

Convierte la lista de waypoints `[t, x, y, z, yaw]` de cada UAV (definida en `config.json` o reescrita dinámicamente) en **una posición por cada paso de simulación**.

- **Modo `Discrete`**: no interpola nada; usa los waypoints tal cual, indexados por el paso más cercano.
- **Modo `Continuous`** (por defecto), para cada tramo `waypoint[i] → waypoint[i+1]`:
  1. **Hover** en la posición actual hasta llegar al instante de inicio del tramo.
  2. **Rotar hacia el rumbo de desplazamiento** (`rotate_to_move_steps`, calculado por `Calculations.calculate_rotate_steps`).
  3. **Desplazarse** en línea recta interpolando posición paso a paso (`calculate_move_steps`).
  4. **Rotar hasta el yaw objetivo** del waypoint (`rotate_to_target_steps`) — se omite si `UavMode == 'Spinning'`, porque en ese modo el yaw gira continuamente y no tiene un "objetivo" fijo.
  5. Si no hay desplazamiento horizontal (solo cambia altura o yaw), se interpola solo altura y luego solo rotación.
  6. Al final de todos los waypoints, el UAV **hace hovering en la última posición** hasta el final de la simulación (relleno para que todas las listas tengan longitud `total_simulation_steps`).

El resultado (`uav_positions_list`, `time_list`, `uav_yaw_angles_list`, uno por UAV) es la fuente de verdad que `run_simulation` usa para mover los vehículos en TraCI paso a paso. **Cada vez que cambian los waypoints** (relevo, retorno a base, comando del Gemelo Digital, entrada manual…) se vuelve a llamar a `uav_path_data()` para regenerar estas listas.

---

## 5. Arranque de SUMO (`start_sumo`, `create_uav_vehicles`)

Dos modos, seleccionados por `self.omnet_mode`:

- **Standalone** (por defecto): lanza `sumo`/`sumo-gui` como subproceso vía `traci.start(...)`, con `--step-length`, `--delay` y el `.sumocfg` del proyecto.
- **Co-simulación con OMNeT++** (`OMNeT Mode: true`): SUMO ya fue lanzado por `veins_launchd_multiclient.py`; Python simplemente se conecta como segundo cliente TraCI (`traci.init(port=9998)`, reintentando hasta 60 veces) y se registra con `traci.setOrder(2)`.

En ambos casos, tras conectar:

1. `create_uav_vehicles()` añade un vehículo TraCI ficticio por cada UAV (`uav{id}`) sobre una ruta "dummy" de una sola arista (los UAVs no siguen carreteras: se reposicionan directamente con `moveToXY`), dibuja un polígono azul marcando su base de origen, y si `wind_mode` está activo añade un icono POI (`wind_arrow.png`) indicando la dirección del viento.
2. Si `dt_mode` está activo, se crea y conecta el puente: `self.dt_bridge = DTBridge.from_config(self.config); self.dt_bridge.connect()`.

---

## 6. El bucle principal: `run_simulation`

Es el método más largo del archivo. Por cada paso de simulación (`while step < total_simulation_steps and not self.stop_flag`) ocurre, en este orden:

### 6.1 Avance de SUMO e integración con el Gemelo Digital

```python
traci.simulationStep()
if self.dt_mode and self.dt_bridge:
    uav_snapshot      = {...}   # posición, estado y batería restante de cada UAV primario
    vehicle_snapshot  = {...}   # posición de cada vehículo TraCI que NO es un UAV
    self.dt_bridge.maybe_send_reports(sim_time, uav_snapshot, vehicle_snapshot)
    self.apply_dt_commands(step, uav_state)
```

`apply_dt_commands` (ver §7) es quien realmente traduce los `MOVE`/`MOVE_BATCH` recogidos por el bridge en nuevas trayectorias.

### 6.2 Máquina de estados de cada UAV

Cada UAV tiene un estado en el diccionario `uav_state`, con las transiciones:

```
MISSION ──(batería baja: return_threshold)──▶ RETURNING
RETURNING ──(llega a menos de 0.5 m de casa)──▶ CHARGING
CHARGING ──(pasan charging_time_steps)──▶ MISSION  (modo normal, retoma su ruta original)
                                       └─▶ STANDBY  (modo relevo: queda de reserva)
STANDBY ──(su compañero activa el relevo)──▶ MISSION
```

- **`STANDBY`**: el UAV se mantiene fijo en su base (`moveToXY` a `uav_home`) y no consume batería ni lógica adicional; solo existe en `relay_mode`.
- **`CHARGING`**: igual que `STANDBY` en cuanto a posición, pero cuenta pasos con `charging_counter` hasta `charging_time_steps = 200 / step_length`. Al completarse, la batería se resetea (`battery_life_steps = 0`) y se resetean los flags de aviso (`warning_sent`, `return_sent`, `stop_sent`).
  - Si **no** hay `relay_mode`: retoma su **trayectoria original** (`original_uav_data`) desde el punto en que la abandonó (`departure_time`), desplazando los tiempos para que encajen con el instante actual.
- **`RETURNING`**: sigue una trayectoria de dos puntos (posición actual → base) generada al activarse el retorno; en cuanto la distancia a casa es < 0.5 m pasa a `CHARGING`.
- Para el resto de estados (`MISSION`), la posición de cada paso sale de `uav_positions_list`/`time_list` (generadas por `uav_path_data`) y se aplica con `traci.vehicle.moveToXY`.

### 6.3 Modelo de batería y relevo (solo si `battery_mode`)

Para cada UAV en vuelo (`z > 0`) se calcula, comparando la posición actual con la anterior:

- `discharge_rate`: vía `calculate_discharge_rate(velocidad_actual)` (o una penalización extra si está "hovering contra el viento": `wind_extra = (wind_speed / v_cruise)² × 0.5`).
- `discharge_rate_home` / `time_home`: coste estimado de volver a casa desde la posición actual (usado para decidir **cuándo** iniciar el retorno, no para descargar batería real todavía).
- Si `wind_mode`, la velocidad relativa se recalcula con `apply_wind_effect` (resta vectorial viento/UAV) tanto para el movimiento actual como para el trayecto de vuelta a casa.

Con esto se definen tres umbrales sobre `battery_life_steps` (pasos de batería consumidos):

| Umbral | Fórmula (aprox.) | Efecto |
|---|---|---|
| Aviso | `battery_life_steps ≥ total − 500/step` | Notificación no bloqueante ("le queda X% de batería") |
| **Relevo** (`relay_threshold`) | `total − (coste_vuelta + reserva + colchón_relevo)` | Con `relay_mode`: activa al **compañero en `STANDBY`**, que pasa a `MISSION` y hereda la ruta pendiente del UAV actual (con un tramo previo desde su base hasta la posición donde lo releva) |
| **Retorno** (`return_threshold`) | `total − (coste_vuelta + reserva)` | El UAV pasa a `RETURNING` hacia su base |

`reserve_steps = 50 / step_length` actúa como colchón de seguridad en ambos umbrales. Como el umbral de **relevo** resta un colchón adicional (`relay_buffer_steps`), se alcanza **antes** que el de retorno: el compañero sale a cubrir la zona mientras el UAV original todavía está en `MISSION`, y solo más tarde ese UAV original pasa a `RETURNING` — así se evita un hueco de cobertura durante el cambio.

Como salvaguarda final, si `battery_life_steps` llega a **agotarse por completo** (`≥ battery_life_steps`) sin haber completado el ciclo de retorno/carga a tiempo, el UAV se retira de forma definitiva: en `relay_mode` queda en `STANDBY` permanente ("sin batería"); si no, se elimina del todo de la simulación (`traci.vehicle.remove`, aviso de "señal perdida"), y si no queda ningún UAV activo (`MISSION`/`RETURNING`), `self.stop_flag = True` termina la simulación.

### 6.4 Visualización en el GUI de SUMO

Si `GuiOption` está activo, por cada UAV en `MISSION` se dibuja (delegando en `self.calc`, instancia de `_utils.Calculations`):

- Un **polígono de campo de visión (FOV)** y su borde (`add_fov_polygon` / `update_fov_polygon` / `remove_fov_polygon`), calculado a partir de `fov_degrees` y la altitud actual (`fov_calculation`).
- Un **icono POI** representando el dron (`add_poi` / `update_poi` / `remove_poi`), con la imagen correspondiente a `UavModel`.
- En modo `UavMode == 'Sampling'`, el polígono de FOV se **retira mientras el UAV se mueve** y solo se vuelve a dibujar cuando está parado — simula una cámara que únicamente "muestrea" en puntos fijos, no en movimiento.
- Si `otimization_mode` está activo, `add_snr_grid_overlay_exact()` pinta una rejilla de polígonos coloreados (rojo→amarillo→verde vía `snr_to_color`) sobre el mapa, representando el SNR estimado por el optimizador en cada celda respecto al umbral de cobertura configurado.

### 6.5 Registro de datos

Cada paso escribe filas en el CSV de salida (por defecto `Outputs/uav_output.csv`): una fila con la posición/yaw del UAV, y una fila adicional por cada vehículo terrestre detectado dentro de su campo de visión (`Calculations.get_vehicles_in_fov`, usando las suscripciones TraCI de posición/velocidad). Esto es, en la práctica, el **dataset de cobertura** que genera la simulación.

### 6.6 Hilos secundarios e instrumentación

- Si `local_gui`: se lanza `get_user_input` en un hilo aparte (diálogos Tkinter).
- Si `server_option`: se lanza `start_server` en un hilo aparte (servidor TCP en el puerto 1024).
- Se mide el tiempo real (`time.perf_counter()`) de cada paso y, al terminar, se imprime un resumen (media, mediana, percentil 95, máximo) y se vuelca a `step_times_{normal|cosim}.csv` — pensado para comparar el rendimiento en modo standalone frente a co-simulación con OMNeT++.
- Al finalizar (por fin de pasos o `stop_flag`), se hace `join()` de los hilos secundarios y `traci.close()`.

---

## 7. `apply_dt_commands`: aplicar las órdenes del Gemelo Digital

Este método materializa la regla de diseño explicada en el propio docstring del código:

> *El Gemelo Digital decide DÓNDE; el modelo realista de `main_.py` decide CÓMO y SI puede.*

Por cada comando `MOVE` recogido de `self.dt_bridge.pop_commands()`:

1. Se descarta si el UAV no existe, o si **no está en estado `MISSION`** (un UAV `RETURNING`/`CHARGING`/`STANDBY` tiene prioridad: la autonomía manda sobre la sugerencia del gemelo, y este simplemente volverá a proponer algo en su siguiente ciclo).
2. Se calcula la posición/yaw actuales a partir de la trayectoria vigente.
3. Se construye una nueva trayectoria de dos puntos (actual → objetivo del DT), calculando el tiempo de llegada **con la velocidad real del UAV** (`self.uav_speed`), no con la velocidad que hubiera sugerido el Gemelo Digital.
4. Se sobrescribe tanto `self.uav_data` como `self.original_uav_data` (para que un relevo o una reanudación tras carga continúen hacia el último objetivo fijado por el DT, no hacia el plan original de `config.json`).
5. Al terminar de procesar todos los comandos, se regeneran las listas interpoladas con `self.uav_path_data()`.

---

## 8. Interfaces de control externas

### 8.1 Servidor TCP remoto (`start_server`, puerto 1024)

Protocolo de texto muy simple, pensado para un cliente externo (distinto del Gemelo Digital, que usa su propio canal en el puerto configurado por `DT Port`):

- Escucha en `localhost:1024`, acepta **una única conexión**.
- Cada mensaje recibido tiene el formato `"<uav_id>:<json con [t, x, y, z, yaw]>"`.
- Responde con un eco (`"Data received: ..."`) y llama a `self.update_uav_path(uav_id, details)`.
- Ante error de conexión o excepción, cierra el socket y termina el hilo.

### 8.2 GUI local (`get_user_input`, `get_uav_position_input`, `non_blocking_warning`)

- `get_uav_position_input()` abre una ventana Tkinter (tema oscuro) pidiendo un ID de UAV y una posición `(t, x, y, z, φ)`; un botón "Break" permite terminar la simulación desde la propia GUI.
- `get_user_input()` es el bucle que mantiene ese diálogo abierto mientras la simulación corre, validando el JSON introducido y delegando en `update_uav_path`.
- `non_blocking_warning(title, message)` muestra una ventana emergente que se autodestruye a los 5 segundos — se usa para todos los avisos de batería, carga y relevo, sin bloquear el bucle de simulación.

### 8.3 `update_uav_path`

Inserta una nueva orden de posición en `self.uav_data[uav_id]`:

- Con **GUI local**: busca el siguiente waypoint pendiente (`point[0] > time`) y **lo sustituye** por el nuevo punto (inserta y elimina el siguiente), es decir, redirige el plan inmediato del UAV.
- Con **servidor TCP** (u otro caso): simplemente **añade** el waypoint al final de la lista y reordena por tiempo.

En ambos casos se regeneran las trayectorias con `uav_path_data()`.

---

## 9. Relación con el resto del proyecto

| Módulo | Qué le pide `main_.py` |
|---|---|
| `_utils.Calculations` | Cinemática del UAV (ángulos de yaw, pasos de rotación/movimiento), gestión de los polígonos de FOV y POIs en el GUI de SUMO, y detección de vehículos dentro del campo de visión |
| `coverage_optimizer.Optimizer` | Posiciones/alturas iniciales óptimas de los UAVs (antes de simular) y la rejilla de SNR usada para el overlay visual |
| `dt_bridge.DTBridge` | Envío de telemetría (`maybe_send_reports`) y recepción de órdenes de reposicionamiento (`pop_commands`) desde el Gemelo Digital |
| `config.json` | Toda la parametrización: modelo de UAV, modos, umbrales, rutas de SUMO, host/puerto del DT, trayectorias iniciales (`uav_data`) |

---

## 10. Flujo resumido de un paso de simulación

```
run_simulation()  (bucle "while step < total_steps")
   │
   ├─ traci.simulationStep()
   │
   ├─ [DT Mode] snapshot UAVs + vehículos ──► dt_bridge.maybe_send_reports()
   │             dt_bridge.pop_commands()  ──► apply_dt_commands() ──► nuevas trayectorias
   │
   ├─ por cada UAV, según su estado:
   │     STANDBY   → fijo en base
   │     CHARGING  → fijo en base, cuenta pasos, al completar: MISSION o STANDBY
   │     RETURNING → sigue ruta hacia base, al llegar: CHARGING
   │     MISSION   → traci.vehicle.moveToXY(posición interpolada del paso actual)
   │
   ├─ [Battery Mode] calcula descarga, evalúa umbrales de aviso/relevo/retorno,
   │                  dispara relevo (compañero STANDBY → MISSION) o RETURNING
   │
   ├─ [GUI Option] actualiza polígonos FOV / iconos POI / overlay de SNR
   │
   └─ registra fila(s) en el CSV de salida (posición UAV + vehículos en su FOV)

(en paralelo, en hilos aparte: get_user_input (GUI) / start_server (TCP:1024) /
 hilo receptor interno de DTBridge)
```

---

## 11. Puntos de diseño a destacar

- **Separación de "plan" y "física"**: tanto el `config.json` como el Gemelo Digital solo proponen *dónde* debería estar un UAV; la conversión a una trayectoria realmente volable (con velocidad, rotación y tiempos de llegada coherentes con el modelo físico) siempre pasa por `uav_path_data()`.
- **Doble copia de las trayectorias** (`uav_data` vs `original_uav_data`): permite que un UAV se desvíe temporalmente (ir a cargar, ser relevado) y luego **recupere su plan de referencia** en vez de perderlo.
- **Máquina de estados explícita pero implementada con `if`/diccionarios**, no con una clase de estado formal — es sencilla de seguir pero implica que toda la lógica de transición vive dentro del bucle `run_simulation`, lo que explica su tamaño.
- **Relevo anticipado**: el umbral de relevo se calcula deliberadamente más estricto que el de retorno, para garantizar que el UAV de refuerzo llegue *antes* de que el original abandone su posición.
- **Concurrencia acotada**: además del hilo principal (simulación + TraCI), como mucho hay dos hilos más (`get_user_input`, `start_server`), más el hilo receptor interno de `DTBridge`; la comunicación entre ellos se hace vía atributos compartidos sencillos (`self.stop_flag`, `self.uav_data`) más que con colas o locks explícitos (salvo dentro de `DTBridge`).
- **Código con restos de depuración**: hay bloques largos de `print` comentados (posiciones de UAVs en pasos concretos) que son residuos de sesiones de debugging y no afectan al comportamiento; puedes ignorarlos con seguridad o limpiarlos si vas a refactorizar el archivo.
- **`self.mode` (`"Input Mode"`) no se usa realmente**: se lee de la configuración pero ningún otro método del archivo lo consulta — es, a día de hoy, un campo vestigial.
