# Subfase 1C — RawMapDatabase y replay de deltas reales

## Estado

```text
realizado
```

---

## Objetivo tecnico

Crear en `orbslam3_multi` una base de datos raw para almacenar exactamente los datos que llegan desde los wrappers de ORB-SLAM3, sin transformarlos, corregirlos ni optimizarlos.

La subfase debe conseguir que el flujo sea:

```text
wrapper ORB-SLAM3
  -> orbslam3_server/global_map_server.cpp
  -> orbslam3_multi::RawMapDatabase
```

Cada `OrbMap` recibido por el servidor debe recibir un `arrival_id` creciente, asignado por el propio servidor en orden real de llegada. Ese `arrival_id` debe guardarse junto con el delta o snapshot recibido, de forma que después se pueda reproducir la misma secuencia sin volver a ejecutar Gazebo.

La base de datos debe mantener dos cosas:

1. **Estado raw actual**, organizado por `(drone_id, map_epoch)`, con `KeyFrames`, `MapPoints`, BoW, observaciones, covisibilidad, parent/children, loop edges locales si existen y poses locales originales de ORB-SLAM3.
2. **Journal ordenado de entradas**, con todos los deltas/snapshots recibidos, ordenados por `arrival_id`.

La clase `RawMapDatabase` debe poder guardar su estado actual y su journal en un path. Después, el servidor debe poder cargar ese archivo, leer el journal y reinyectar los deltas/snapshots en una base de datos nueva usando un timer, como si los wrappers los estuvieran enviando de nuevo.

Esta subfase permite repetir pruebas sobre datos reales de ORB-SLAM3 sin lanzar Gazebo muchas veces.

---

## Contexto obligatorio a leer

Antes de modificar nada, Codex debe leer:

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1A.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1B.md`
- historiales recientes de Fase 1, especialmente 1A y 1B si ya existen
- `codex/contexto/paquetes/orbslam3_server/`
- `codex/contexto/paquetes/orbslam3_multi/`
- `codex/contexto/paquetes/orbslam3_msgs/`
- `codex/contexto/paquetes/simulacion_dron/`
- ADRs relacionados con submapas, fiduciales, loops, fused landmarks y optimización, si existen

También debe consultar el servidor nuevo creado en 1B y el servidor antiguo congelado como referencia de migración.

---

## Diagnostico de partida

Después de 1B debe existir un servidor nuevo mínimo que:

- arranca desde el launch normal;
- recibe `OrbMap` de los wrappers;
- muestra en logs `drone_id`, `map_epoch`, `map_sequence`, número de `KeyFrames` y número de `MapPoints`;
- no publica todavía una nube global útil;
- no gestiona fiduciales, loops, fused landmarks ni optimización.

En 1B el objetivo era solo validar recepción. En 1C el problema es que esos datos todavía no se almacenan en un backend raw limpio ni se pueden guardar/reproducir sin Gazebo.

Marcadores esperados heredados de 1B:

```text
[F1B-SERVER-INIT]
[F1B-SERVER-PARAMS]
[F1B-SERVER-SUBSCRIBED]
[F1B-ORBMAP-RX]
[F1B-SERVER-STATS]
```

La subfase 1C debe añadir marcadores nuevos que demuestren:

```text
[F1C-RAWDB-INIT]
[F1C-RAWDB-DELTA-RX]
[F1C-RAWDB-INSERT-DELTA]
[F1C-RAWDB-INSERT-FULL]
[F1C-RAWDB-NEW-SUBMAP]
[F1C-RAWDB-STATS]
[F1C-RAWDB-SAVE]
[F1C-REPLAY-LOAD]
[F1C-REPLAY-DELTA]
[F1C-REPLAY-DONE]
```

---

## Archivos permitidos a modificar

### Código

- `orbslam3_server/src/global_map_server.cpp`
- `orbslam3_server/include/...` si el servidor nuevo tiene cabecera propia
- `orbslam3_server/launch/*.launch.py` solo si hace falta añadir parámetros de record/replay
- `orbslam3_server/CMakeLists.txt`
- `orbslam3_server/package.xml` solo si aparece una dependencia real nueva
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp` o nombre equivalente si se decide separar tipos
- `orbslam3_multi/include/orbslam3_multi/raw_map_serialization.hpp` si se necesita separar serialización
- `orbslam3_multi/src/raw_map_serialization.cpp` si se necesita separar serialización
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_multi/package.xml` solo si aparece una dependencia real nueva

### Configuración y pruebas

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml`
- archivos generados en `codex/archivos_auxiliares/` para guardar datasets raw/replay

### Documentación

- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/paquetes/orbslam3_server/`
- `codex/contexto/paquetes/orbslam3_multi/`
- `codex/contexto/paquetes/simulacion_dron/`
- `codex/pipeline/fase_1_sparse_global/historial/`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1C.md`, solo si durante ejecución hay que corregir criterio o documentar bloqueo real
- un nuevo `.md` de pruebas clave, por ejemplo:

```text
codex/contexto/pruebas_clave/fase_1C_rawdb_replay.md
```

o, si encaja mejor con la documentación existente:

```text
codex/contexto/paquetes/simulacion_dron/pruebas_clave_rawdb_replay.md
```

---

## Archivos prohibidos

No modificar en esta subfase:

- `ORB_SLAM3/`
- `orbslam3_ros2/`, salvo que el log demuestre que 1B estaba mal y el wrapper no publica datos; en ese caso detenerse y documentar bloqueo antes de tocarlo
- `orbslam3_msgs/`, salvo error de compilación estrictamente necesario y justificado
- lógica de fiduciales
- lógica de loops
- lógica de fused landmarks
- lógica de pose graph u optimización
- `global_map_server_antiguo.cpp`, salvo comentario/documentación mínima que indique que es legacy si 1B no lo hizo
- archivos dentro de `orbslam3_multi/legacy/`, salvo copiar referencias legacy si 1B no lo hizo y siempre sin añadirlas al build
- `build/`, `install/`, `log/`, salvo limpieza mínima de artefactos generados si bloquean compilación, según `AGENTS.md`

---

## Funciones, clases o nodos que hay que localizar

El `planificador_fase` debe localizar antes de implementar:

- nodo nuevo del servidor creado en 1B;
- callback que recibe `OrbMap` delta en el servidor nuevo;
- clientes o callbacks de `GetOrbMap` full snapshot si ya existen en el servidor nuevo;
- tipos de mensaje de `orbslam3_msgs::msg::OrbMap`, `OrbKeyFrame`, `OrbMapPoint`, `OrbObservation`, etc.;
- estructura de CMake de `orbslam3_multi`;
- estructura de CMake de `orbslam3_server`;
- formato de trayectorias usado por `scenario_runner_node`;
- topics reales de cada dron para `orbslam/orb_map_delta`;
- parámetros del launch que permiten activar/desactivar record/replay, si ya existe alguno.

No inventar nombres de clases si ya existe una clase equivalente en `orbslam3_multi`. Si existe una clase antigua que ya almacena parte de los datos, se puede consultar, pero la implementación nueva de esta subfase debe quedar en `RawMapDatabase` o archivo equivalente nuevo.

---

## Cambios requeridos

### 1. Crear tipos raw internos en `orbslam3_multi`

Crear tipos C++ puros para representar los datos recibidos desde `OrbMap` sin depender innecesariamente de ROS dentro de la base de datos.

Ejemplos de tipos esperados, ajustables por el implementador si encuentra una estructura mejor:

```cpp
struct RawSubmapId
{
    uint32_t drone_id;
    uint64_t map_epoch;
};

struct RawKeyFrameId
{
    uint32_t drone_id;
    uint64_t map_epoch;
    uint64_t local_kf_id;
};

struct RawMapPointId
{
    uint32_t drone_id;
    uint64_t map_epoch;
    uint64_t local_mp_id;
};
```

La identidad interna debe respetar siempre:

```text
submapa = (drone_id, map_epoch)
```

Condición de seguridad:

- no usar solo `drone_id` como identidad de submapa;
- no usar solo `local_kf_id` o `local_mp_id` como identidad global;
- no transformar poses a `world`.

Log de validación:

```text
[F1C-RAWDB-NEW-SUBMAP]
```

---

### 2. Crear `RawMapDatabase`

Crear la clase:

```cpp
class RawMapDatabase
```

Responsabilidades:

- guardar submapas por `(drone_id, map_epoch)`;
- guardar `KeyFrames` raw;
- guardar `MapPoints` raw;
- guardar BoW y FeatureVector si vienen del wrapper;
- guardar descriptores, keypoints y asociaciones KF-MP;
- guardar observaciones MP-KF;
- guardar covisibilidad, parent, children y loop edges locales si vienen en el mensaje;
- guardar poses locales originales de ORB-SLAM3;
- guardar un journal ordenado de deltas/snapshots recibidos;
- devolver estadísticas.

Métodos mínimos:

```cpp
InsertDelta(arrival_id, delta)
InsertFullSnapshot(arrival_id, snapshot)
GetSubmap(...)
GetKeyFrame(...)
GetMapPoint(...)
GetDatabaseStats()
GetSubmapIds()
GetKeyFrameIdsForSubmap(...)
GetMapPointIdsForSubmap(...)
HasKeyFrame(...)
HasMapPoint(...)
Clear()
SaveToPath(path)
```

Si por estilo del proyecto se necesitan nombres más específicos, se permite adaptarlos, pero deben quedar documentados.

Condición de seguridad:

- `RawMapDatabase` no debe corregir poses;
- `RawMapDatabase` no debe saber de fiduciales;
- `RawMapDatabase` no debe saber de loops globales;
- `RawMapDatabase` no debe optimizar;
- `RawMapDatabase` no debe publicar ROS.

Logs de validación:

```text
[F1C-RAWDB-INIT]
[F1C-RAWDB-INSERT-DELTA]
[F1C-RAWDB-INSERT-FULL]
[F1C-RAWDB-STATS]
```

---

### 3. Asignar `arrival_id` en el servidor

El servidor nuevo debe tener un contador monotónico local:

```text
rawdb_next_arrival_id
```

Cada vez que recibe un delta desde wrapper:

```text
arrival_id = rawdb_next_arrival_id++
RawMapDatabase::InsertDelta(arrival_id, delta)
```

Cada vez que recibe un full snapshot:

```text
arrival_id = rawdb_next_arrival_id++
RawMapDatabase::InsertFullSnapshot(arrival_id, snapshot)
```

El `arrival_id` representa el orden real de llegada al servidor. No debe sustituirse por `map_sequence`, `keyframe_id` ni timestamp.

Condición de seguridad:

- el orden de replay debe basarse en `arrival_id`;
- `keyframe_id` identifica el KF, no el orden global de entrada;
- `map_sequence` puede guardarse como metadata, pero no debe ser el orden global principal.

Logs de validación:

```text
[F1C-RAWDB-DELTA-RX] arrival_id=...
```

---

### 4. Guardar base de datos y journal en un path

`RawMapDatabase::SaveToPath(path)` debe guardar:

- estado actual de submapas/KFs/MPs;
- metadata de cámara si se conserva;
- journal completo de deltas/snapshots ordenado por `arrival_id`;
- estadísticas suficientes para validar la carga;
- versión de formato.

Formato recomendado:

- JSON/YAML si el tamaño es razonable y facilita depuración;
- binario si JSON/YAML resulta demasiado pesado.

La elección debe justificarse en historial y documentación.

Ruta recomendada para pruebas:

```text
codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record
codex/archivos_auxiliares/repeticiones/rawdb_prueba_2.record
codex/archivos_auxiliares/repeticiones/rawdb_prueba_3.record
```

Condición de seguridad:

- no guardar en `src/` datasets generados;
- no depender de rutas absolutas del equipo del usuario;
- si falla el guardado, loggear el error y marcar la prueba como parcial o fallida según impacto.

Log de validación:

```text
[F1C-RAWDB-SAVE]
```

---

### 5. Cargar dataset grabado desde el servidor y reproducir deltas

La carga del archivo y el replay deben estar en el servidor o en una clase auxiliar del servidor, no como responsabilidad principal de `RawMapDatabase`.

El servidor debe poder trabajar en dos modos:

```text
modo normal/record:
    recibe deltas desde wrappers
    los inserta en RawMapDatabase
    opcionalmente guarda al final o por timer

modo replay:
    carga un archivo grabado
    lee el journal ordenado por arrival_id
    crea/limpia una RawMapDatabase nueva
    con un timer inserta el siguiente delta/snapshot completo
```

Parámetros sugeridos:

```text
rawdb_record_enabled
rawdb_record_path
rawdb_replay_enabled
rawdb_replay_path
rawdb_replay_period_sec
```

En 1C el replay debe ser por delta/snapshot completo, no por KeyFrame individual.

Condición de seguridad:

- no dividir artificialmente un delta en KeyFrames en esta subfase;
- no mezclar replay con recepción real de wrappers salvo que se documente una política clara;
- si `rawdb_replay_enabled=true`, el servidor debe ignorar o no suscribirse a wrappers reales, o loggear claramente qué fuente tiene prioridad.

Logs de validación:

```text
[F1C-REPLAY-LOAD]
[F1C-REPLAY-DELTA]
[F1C-REPLAY-DONE]
```

---

### 6. Crear documentación de pruebas clave de raw database

Durante esta subfase se debe crear o actualizar un `.md` de pruebas clave donde queden descritas las trayectorias largas que generan datasets útiles para repetir fases posteriores sin Gazebo.

Archivo recomendado:

```text
codex/contexto/pruebas_clave/fase_1C_rawdb_replay.md
```

Si esa carpeta no existe y el proyecto ya tiene otra ubicación para pruebas, usar la ubicación existente y documentarlo en historial.

El `.md` debe explicar:

- nombre de cada prueba;
- objetivo del dataset;
- trayectoria resumida;
- archivo `tray_prueba_X.yaml` asociado;
- archivo rawdb generado;
- duración aproximada;
- qué evidencia de logs valida que se grabó correctamente;
- si la prueba es corta, media o larga.

---

## Cambios prohibidos

En esta subfase está prohibido:

- implementar `GlobalPoseStore`;
- calcular `world_T_local`;
- transformar KeyFrames o MapPoints al frame `world`;
- detectar si un KF está en fiducial;
- crear lógica de primera/segunda visita a fiducial;
- buscar loops por BoW;
- construir subnubes;
- hacer RANSAC;
- fusionar landmarks;
- construir pose graphs;
- optimizar;
- resolver la propagación de correcciones a KFs nuevos tras optimización;
- publicar una nube global nueva como criterio de esta fase;
- rediseñar mensajes `orbslam3_msgs`;
- modificar el wrapper para añadir datos nuevos;
- renombrar masivamente archivos activos de `orbslam3_multi`;
- añadir archivos legacy al build;
- convertir esta subfase en una limpieza general del servidor.

Nota para fases posteriores:

```text
RawMapDatabase conserva poses locales originales.
Las poses globales, anchors, poses optimizadas y propagación de correcciones a KFs nuevos deben resolverse en GlobalPoseStore y fases de optimización posteriores.
```

---

## Paquetes a compilar

Comando esperado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
```

Si se toca accidentalmente o por necesidad real `orbslam3_msgs`, compilar también:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

Si falla el build:

```bash
./codex/herramientas/reduce_build_log.sh
```

Después, diagnosticar el primer error real, corregir lo mínimo y repetir.

Si por limpieza de artefactos o por workspace inconsistente hay que reconstruir paquetes pesados para poder simular, hacerlo de uno en uno. Orden recomendado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs
./codex/herramientas/build_selected_packages.sh orbslam3
./codex/herramientas/build_selected_packages.sh orbslam3_multi
./codex/herramientas/build_selected_packages.sh orbslam3_server
./codex/herramientas/build_selected_packages.sh dron_individual
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

No agrupar `orbslam3`, `dron_individual` y `simulacion_dron` en un build masivo salvo que el usuario lo pida explícitamente.

---

## Pruebas Gazebo requeridas

En esta subfase hay dos tipos de prueba:

1. pruebas para grabar datasets raw reales desde Gazebo;
2. pruebas de replay sin depender de que Gazebo vuelva a producir los datos.

El implementador debe ejecutar al menos la prueba simple y una prueba de replay. Las pruebas largas pueden ejecutarse si el tiempo de simulación y estabilidad lo permiten. Si una prueba larga no se ejecuta por tiempo, debe quedar marcada como pendiente en historial y documentación, no inventada como conseguida.

---

### Prueba 1 — Simple fiducial ida/vuelta

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Objetivo:

- validar inserción de deltas reales;
- validar separación por dos drones;
- validar `arrival_id` creciente;
- guardar un dataset corto para replay.

Secuencia esperada:

1. esperar arranque de simulación;
2. dron 1 va al fiducial 2 con altura `z=1.0` y yaw `90°`;
3. dron 2 va al fiducial 2 con altura `z=1.3` y yaw `90°`;
4. dron 1 se mueve a `x=-8` sin cambiar altura ni yaw;
5. dron 2 se mueve a `x=-8` sin cambiar altura ni yaw;
6. dron 1 vuelve al fiducial 2;
7. dron 2 vuelve al fiducial 2;
8. esperar unos segundos;
9. terminar simulación.

Tiempos orientativos:

```text
cada movimiento: 20 s
```

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Archivo rawdb esperado:

```text
codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record
```

Observación esperada en RViz2:

```text
No aplica como criterio de éxito.
En 1C se valida por logs y por archivo rawdb generado.
```

---

### Prueba 2 — Dos drones rodean el edificio en sentidos opuestos

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Objetivo:

- generar un dataset más completo con varios KFs y MPs;
- cubrir ida a fiducial 2, recorrido por el edificio, fiducial 1 y vuelta a fiducial 2;
- obtener datos útiles para fases posteriores de fiduciales, loops y optimización.

Tiempos de movimiento:

```text
tx = 40 s
ty = 40 s
tz = 40 s
tyaw = 13 s
```

Trayectoria dron 1:

1. ir a fiducial 2 con `z=1.0` y yaw `90°`;
2. manteniendo altura y yaw, ir a `(-10, -10)`;
3. manteniendo altura, poner yaw `0°` e ir a `(-10, 10)`;
4. manteniendo altura, poner yaw `-90°` e ir al fiducial 1;
5. manteniendo altura y yaw, ir a `(10, 10)`;
6. manteniendo altura, poner yaw `-180°` e ir a `(10, -10)`;
7. manteniendo altura, poner yaw `90°` e ir al fiducial 2.

Trayectoria dron 2:

1. ir a fiducial 2 con `z=1.3` y yaw `90°`;
2. manteniendo altura y yaw, ir a `(10, -10)`;
3. manteniendo altura, poner yaw `180°` e ir a `(10, 10)`;
4. manteniendo altura, poner yaw `-90°` e ir al fiducial 1;
5. manteniendo altura, poner yaw `90°` e ir a `(-10, 10)`;
6. manteniendo altura, poner yaw `0°` e ir a `(-10, -10)`;
7. manteniendo altura, poner yaw `90°` e ir al fiducial 2.

Al final:

1. esperar unos segundos;
2. terminar simulación.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 30
```

Archivo rawdb esperado:

```text
codex/archivos_auxiliares/repeticiones/rawdb_prueba_2.record
```

Observación esperada en RViz2:

```text
No aplica como criterio de éxito en 1C.
Esta prueba solo debe validar grabación rawdb de un recorrido largo.
```

---

### Prueba 3 — Dos vueltas alrededor de la casa

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml
```

Objetivo:

- generar un dataset largo con recorrido circular alrededor del edificio;
- obtener datos útiles para futuros loops, subnubes y optimización;
- evitar repetir una simulación larga en fases posteriores.

Secuencia esperada:

1. dron 1 va al fiducial 2 con `z=1.0` y yaw `90°`;
2. dron 2 va al fiducial 2 con `z=1.3` y yaw `90°`;
3. ambos se colocan en `(0, -13)` manteniendo sus alturas y yaw `90°`;
4. dron 1 realiza una trayectoria circular alrededor de la casa con radio aproximado `13` en ambos ejes, duración `60 s` por vuelta;
5. dron 2 realiza la misma trayectoria circular de uno en uno, con altura `z=1.3`, duración `60 s` por vuelta;
6. repetir hasta que ambos drones den 2 vueltas en total según lo indicado por el usuario;
7. esperar unos segundos;
8. terminar simulación.

Condición especial:

Si el `scenario_runner_node` o el formato YAML existente no tiene una primitiva clara para trayectorias circulares, Codex no debe inventar una implementación compleja en esta subfase. Debe detenerse y preguntar al usuario cómo quiere codificar la trayectoria circular, o proponer una aproximación por waypoints solo si el formato existente ya lo permite claramente.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 3 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 30
```

Archivo rawdb esperado:

```text
codex/archivos_auxiliares/repeticiones/rawdb_prueba_3.record
```

Observación esperada en RViz2:

```text
No aplica como criterio de éxito en 1C.
Esta prueba solo debe validar grabación rawdb de una trayectoria larga.
```

---

### Prueba 4 — Replay de dataset grabado

YAML:

```text
No requiere Gazebo si el modo replay puede ejecutarse desde el servidor.
Si las herramientas actuales exigen Gazebo, usar el launch normal pero con rawdb_replay_enabled=true y sin depender de movimiento real.
```

Objetivo:

- cargar un `.record` generado por una prueba anterior;
- reinyectar deltas/snapshots en orden de `arrival_id` usando timer;
- validar que una base de datos nueva se reconstruye con estadísticas coherentes.

Parámetros esperados:

```text
rawdb_replay_enabled=true
rawdb_replay_path=codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record
rawdb_replay_period_sec=0.5
```

Logs esperados:

```text
[F1C-REPLAY-LOAD]
[F1C-REPLAY-DELTA]
[F1C-REPLAY-DONE]
[F1C-RAWDB-STATS]
```

Criterio específico:

El número de deltas reproducidos debe coincidir con el número de deltas guardados en el archivo. Las estadísticas finales deben ser coherentes con el dataset grabado.

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-RAWDB|F1C-REPLAY|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-RAWDB|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 3

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-RAWDB|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 4

```text
F1C-REPLAY|F1C-RAWDB|F1C-RAWDB-STATS|ERROR|FATAL|Segmentation fault|Killed
```

La reducción genera `prueba_X.reduced.log` y conserva `prueba_X.log` completo.

Si el reducido no muestra los marcadores necesarios, revisar el completo antes de repetir Gazebo o concluir que el código no emitió el marcador.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. `RawMapDatabase` existe en `orbslam3_multi` y se compila como parte del paquete;
3. el servidor nuevo asigna `arrival_id` creciente a cada delta/snapshot recibido;
4. el servidor llama a `InsertDelta(arrival_id, ...)` para deltas reales;
5. si se usa full snapshot, el servidor llama a `InsertFullSnapshot(arrival_id, ...)`;
6. la base de datos separa datos por `(drone_id, map_epoch)`;
7. los logs muestran inserción de al menos dos drones en la prueba simple;
8. aparece `F1C-RAWDB-STATS` con número de submapas, KFs y MPs;
9. `SaveToPath(path)` genera un archivo rawdb en `codex/archivos_auxiliares/`;
10. el journal guardado contiene deltas/snapshots ordenados por `arrival_id`;
11. el servidor puede cargar el archivo guardado y reproducir los deltas con un timer;
12. el replay muestra `F1C-REPLAY-LOAD`, varios `F1C-REPLAY-DELTA` y `F1C-REPLAY-DONE`;
13. no aparecen errores graves no explicados;
14. se actualiza el historial;
15. se actualiza la documentación de paquetes afectados;
16. se crea o actualiza el `.md` de pruebas clave con los datasets generados o pendientes.

En esta subfase no se exige observar nada nuevo en RViz2.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- el servidor no recibe `OrbMap`;
- no se asigna `arrival_id`;
- no se insertan deltas en `RawMapDatabase`;
- no se separa por `(drone_id, map_epoch)`;
- no se puede guardar ningún archivo rawdb;
- no se puede cargar/reproducir ningún dataset;
- falta un marcador obligatorio;
- aparece `Segmentation fault`, `Killed`, `FATAL` o error grave no explicado.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- se insertan deltas reales;
- se guarda el archivo rawdb;
- pero el replay falla o queda incompleto;
- o solo se valida un dron por problema de simulación;
- o no se puede ejecutar una prueba larga, pero la prueba simple y el replay funcionan.

La subfase debe marcarse como `BLOQUEADA` solo si:

- falta información crítica sobre el formato de trayectoria;
- la trayectoria circular de la prueba 3 no puede codificarse sin decisión del usuario;
- el servidor nuevo de 1B no existe o no arranca;
- hay una dependencia externa no resuelta;
- el mismo bloqueo se repite y no puede resolverse con cambios mínimos.

No marcar como `realizado` si el criterio de éxito no se cumple.

---

## Documentacion a actualizar

Actualizar obligatoriamente:

- historial de Fase 1 / subfase 1C;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- documentación específica nueva o actualizada de `RawMapDatabase`, por ejemplo:

```text
codex/contexto/paquetes/orbslam3_multi/raw_map_database.md
```

- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
- `codex/contexto/paquetes/orbslam3_server/launches.md` si se añaden parámetros de record/replay;
- documentación de pruebas clave, por ejemplo:

```text
codex/contexto/pruebas_clave/fase_1C_rawdb_replay.md
```

La documentación de `RawMapDatabase` debe explicar:

- que guarda datos raw originales de ORB-SLAM3;
- que no corrige poses;
- que no transforma a `world`;
- que separa por `(drone_id, map_epoch)`;
- qué métodos existen;
- cómo funciona `arrival_id`;
- qué contiene el journal;
- cómo se guarda con `SaveToPath(path)`;
- cómo el servidor usa el archivo para replay;
- qué logs validan el funcionamiento;
- qué queda explícitamente fuera de 1C.

La documentación del servidor debe explicar:

- que el servidor asigna `arrival_id`;
- que llama a `RawMapDatabase`;
- que puede grabar datasets;
- que puede reproducir datasets mediante timer;
- que en 1C sigue sin publicar nube global nueva ni gestionar fiduciales/loops.

El historial debe registrar:

- fecha;
- archivos modificados;
- paquetes compilados;
- resultado del build;
- pruebas ejecutadas;
- archivos rawdb generados;
- número de deltas guardados;
- número de deltas reproducidos;
- estadísticas finales de submapas/KFs/MPs;
- evidencia positiva;
- evidencia negativa o ausente;
- conclusión `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`;
- siguiente paso recomendado.

---

## Nota de diseño para fases posteriores

Esta subfase no resuelve anclaje ni optimización, pero deja fijada una regla crítica:

```text
RawMapDatabase conserva lo que dijo ORB-SLAM3.
Las poses globales, anchors, poses optimizadas y propagación de correcciones se implementarán fuera de RawMapDatabase.
```

Cuando en fases posteriores se optimicen KeyFrames, las poses corregidas no deben sobrescribir las poses locales originales guardadas en `RawMapDatabase`. Deberán almacenarse en una capa posterior como `GlobalPoseStore`.

Además, cuando existan KFs nuevos después de una optimización, no deberán colocarse solo con el anclaje rígido inicial. Esa propagación de correcciones queda fuera de 1C, pero debe tenerse en cuenta al diseñar `GlobalPoseStore` y las fases de optimización.
