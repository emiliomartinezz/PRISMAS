# Gemelo digital para la gestión de la topología en redes 5G/6G basadas en drones

Trabajo de Fin de Máster (MUIT — UPM) · Grupo GIROS · Proyecto PRISMAS

Este repositorio contiene la implementación de un **gemelo digital de red** que
observa, analiza y actúa sobre una red móvil 5G/6G simulada, reposicionando en
tiempo real las estaciones base (`gNodeB`) desplegadas sobre drones para
mejorar la cobertura de los usuarios.

El sistema se compone de dos planos independientes que se comunican mediante un
socket TCP:

- Un **simulador de red** en [OMNeT++](https://omnetpp.org/) + [Simu5G](https://simu5g.org/)
  (paquete `dronenetwork`), con módulos C++ propios para la telemetría y la
  movilidad controlada de las estaciones.
- Un **gemelo digital en Python** (`DT/`), que recibe esa telemetría, construye
  un grafo de la topología con [NetworkX](https://networkx.org/), detecta
  zonas de cobertura degradada y envía órdenes de reposicionamiento a los
  drones.



---

## Índice

- [Estructura del repositorio](#estructura-del-repositorio)
- [Requisitos previos](#requisitos-previos)
- [Instalación](#instalación)
  - [1. Simulador (OMNeT++ / INET / Simu5G)](#1-simulador-omnet-inet-simu5g)
  - [2. Gemelo digital (Python)](#2-gemelo-digital-python)
- [Ejecución](#ejecución)
- [Escenarios disponibles (`omnetpp.ini`)](#escenarios-disponibles-omnetppini)
- [Resultados generados](#resultados-generados)

---

## Estructura del repositorio

```
.
├── OMNET++/
│   ├── src/                        # Módulos C++ / NED del paquete "dronenetwork"
│   │   ├── package.ned             
│   │   ├── DroneNetwork.ned        # Red principal 
│   │   ├── MobileGnb.ned           # gNodeB móvil 
│   │   ├── MobileUE.ned            # UE 
│   │   ├── UESender.{h,cc,ned}         # Telemetría del UE
│   │   ├── GnbSender.{h,cc,ned}        # Telemetría del gNodeB 
│   │   ├── Receiver.{h,cc,ned}         # Concentrador en el servidor
│   │   ├── Tcpclient.{h,cc,ned}        # Cliente TCP real hacia el gemelo digital
│   │   ├── SupervisedMobility.{h,cc,ned} # Modelo de movilidad controlada externamente
│   │   └── DroneController.{h,cc,ned}    # Cola de órdenes MOVE / MOVE_BATCH (mutex)
│   └── simulations/
│       ├── package.ned             
│       ├── omnetpp.ini             # Fichero de configuración 
│       └── network_config.xml      # Configuración de direccionamiento IP
│
├── DT/                              # Gemelo digital (Python), un script por estrategia
│   ├── digital_twin_centroide.py            # Estrategia: centroide simple
│   ├── digital_twin_centroide_ponderado.py  # Estrategia: centroide ponderado por SINR
│   └── digital_twin_kmeans.py               # Estrategia: k-means 
│
└── .gitignore
```

---

## Requisitos previos


| [OMNeT++](https://omnetpp.org/) (serie 6.0.3) | Motor de simulación de eventos discretos |
| [INET Framework](https://inet.omnetpp.org/) | Modelos de red de base (movilidad, IP, aplicaciones) sobre los que se apoya Simu5G |
| [Simu5G](https://simu5g.org/) | Modelado de la red 5G NR |
| Se emplea  Python 3.10.12 | Ejecución del gemelo digital |



---

## Instalación

### 1. Simulador (OMNeT++ / INET / Simu5G)



1. Instala OMNeT++ siguiendo la
   [guía oficial de instalación](https://doc.omnetpp.org/omnetpp/InstallGuide.pdf).

2. Descarga e importa **INET** en tu workspace de OMNeT++
   (`File → Import → Existing Projects into Workspace`) y compílalo.

3. Descarga e importa **Simu5G** de la misma forma, referenciando el proyecto
   INET ya importado, y compílalo.

4. Crea un nuevo proyecto OMNeT++ (`File → New → OMNeT++ Project`) y copia
   dentro el contenido de `OMNET++/src/` y `OMNET++/simulations/` de este
   repositorio, conservando la estructura de carpetas.

5. En `Project → Properties → OMNeT++ → Project References`, marca **INET** y
   **Simu5G** como proyectos referenciados.

6. En `Project → Properties → OMNeT++ → Source Folders`, comprueba que tanto
   `src/` como `simulations/` están marcadas como carpetas fuente NED (cada una
   define su propio paquete: `dronenetwork` y `dronenetwork.simulations`
   respectivamente, según sus `package.ned`).

7. Compila el proyecto (`Project → Build Project`).

### 2. Gemelo digital (Python)

Se recomienda un entorno virtual:

```bash

sudo apt install python3-tk # backend TkAgg de matplotlib

cd DT
python3 -m venv .venv
source .venv/bin/activate        
pip install -r ../requirements.txt
```

Las dependencias (ver `requirements.txt`) son:

- `networkx` — construcción del grafo de topología (gNodeBs, UEs, enlaces)
- `numpy` — malla de cobertura y cálculo estadístico del SINR
- `matplotlib` — visualizaciones en tiempo real y mapas finales (backend `TkAgg`)
- `scikit-learn` — DBSCAN para la detección de huecos de cobertura (con
  fallback a un clustering propio si no está instalado) y KMeans para la
  estrategia de reposicionamiento `digital_twin_kmeans.py` (sin fallback: es
  obligatorio para ese script en concreto)

---

## Ejecución

El orden de arranque importa: el gemelo digital debe estar escuchando antes
de lanzar la simulación, porque el módulo `TcpClient` de OMNeT++ se conecta
activamente a `127.0.0.1:50000` al iniciar.

1. **Arranca el gemelo digital**, eligiendo el script correspondiente a la
   estrategia de reposicionamiento que quieras evaluar:

   ```bash
   cd DT
   source .venv/bin/activate
   python3 digital_twin_centroide.py             # centroide simple
   # o bien:
   python3 digital_twin_centroide_ponderado.py   # centroide ponderado por SINR
   # o bien:
   python3 digital_twin_kmeans.py                # k-means ponderado por SINR
   ```

   El script abre un servidor TCP en `0.0.0.0:50000` y queda a la espera de
   conexión.

2. **Arranca OMNeT++**, 
 ```bash
   cd omnetpp-6.0.3 
   source setenv
   cd bin
   ./omnetpp
   ```


3. **Lanza la simulación en OMNeT++**, ejecuta 'omnetpp.ini' indicando la `Config` que corresponda a la estrategia elegida.



4. Durante la simulación, el gemelo digital imprime en consola la telemetría
   recibida y genera periódicamente gráficas de la topología en tiempo real.
   Al finalizar (o al interrumpir con `Ctrl+C`), guarda el informe final y las
   visualizaciones.

> Para la correcta validación de las pruebas la `Config` elegida en OMNeT++ (movilidad de las estaciones) y el script de
> `DT/` que arranques deben corresponder a la misma estrategia: `Config
> Centroides` con `digital_twin_centroide.py`, `Config CentroidesPonderados`
> con `digital_twin_centroide_ponderado.py`, y `Config KMeans` con
> `digital_twin_kmeans.py`. El simulador no valida esta correspondencia por sí
> solo.

---

## Escenarios disponibles (`omnetpp.ini`)

Un único fichero, `OMNET++/simulations/omnetpp.ini`, define la topología común
(3 `gNodeB`, 6 `UE` en 3 clusters, canal `URBAN_MACROCELL` con shadowing y
fading Jakes, área de 2×2 km) y cuatro configuraciones (`Config`) sobre ella:

| `Estatico` | `StationaryMobility` — posiciones fijas, referencia de cobertura | (ninguno específico) |
| `Centroides` | `SupervisedMobility`, reposicionada por el gemelo digital | `digital_twin_centroide.py` |
| `CentroidesPonderados` | `SupervisedMobility`, reposicionada por el gemelo digital | `digital_twin_centroide_ponderado.py` |
| `KMeans` | `SupervisedMobility`, reposicionada por el gemelo digital | `digital_twin_kmeans.py` |

`network_config.xml` es un fichero de apoyo referenciado desde `omnetpp.ini`
para la configuración de direccionamiento IP (`Ipv4NetworkConfigurator`).

---

## Resultados generados

Cada ejecución del gemelo digital crea (si no existen) dos carpetas junto al
script,  ambas están excluidas del control de versiones en `.gitignore`:

- `DT/coverage_results/`: informe final en JSON
  (`coverage_report_<timestamp>.json`) con estadísticas de cobertura,
  distribución de QoS, huecos detectados e historial de órdenes de movimiento
  enviadas; y el grafo de topología en formato GraphML.
- `DT/coverage_plots/`: mapa de cobertura, mapa de SINR y mapa de huecos de
  cobertura detectados, en PNG.

---

