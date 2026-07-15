# Subfase 1E — FiducialAnchorManager y anclaje inicial por fiducial simulado

## Estado

```text
realizado
```

`CONSEGUIDA` el 2026-07-08. Evidencia: build de `orbslam3_multi` y `orbslam3_server` con `BUILD-EXIT-CODE 0`, simulación Gazebo `prueba_1` con `SIM-DONE prueba=1 success=true`, nuevo `rawdb_prueba_1.record` versión 2 con `60` observaciones fiduciales, replay `prueba_2` con `SIM-DONE prueba=2 success=true`, `60` `[F1E-FID-REPLAY-OBS]`, `anchors_created=2`, `accepted=60`, `rejected=0` y `hard_fiducial_kfs=60`.

---

## Objetivo tecnico

Crear en `orbslam3_multi` la clase responsable de gestionar observaciones fiduciales y anclar submapas mediante una primera observacion absoluta simulada.

En esta subfase el servidor sigue siendo quien detecta, usando GT de Gazebo, que un dron esta dentro del radio de un fiducial. Sin embargo, el servidor no debe decidir internamente el anclaje del submapa. El servidor debe construir una observacion fiducial asociada a un `KeyFrame` concreto y entregarla al backend.

El backend debe:

- recibir una observacion fiducial ya construida por el servidor;
- consultar `RawMapDatabase` para obtener el submapa y la pose local ORB-SLAM3 del `KeyFrame`;
- consultar `GlobalPoseStore` para saber si el submapa ya esta anclado;
- si es la primera observacion fiducial valida del submapa, calcular `world_T_local`;
- llamar a `GlobalPoseStore` para anclar el submapa y asignar poses globales a sus `KeyFrames` existentes;
- marcar el `KeyFrame` como fiducial real/hard fiducial en la capa ligera de poses o en el registro de anchors;
- guardar eventos fiduciales asociados al `arrival_id` del delta para que las pruebas replay sin Gazebo puedan reproducir el mismo anclaje.

La subfase se verifica con build, simulacion, logs y, si es posible, observacion en RViz2 de que la nube/submapa anclado queda colocado de forma coherente respecto al fiducial.

---

## Contexto obligatorio a leer

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1A.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1B.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1C.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1D.md`
- historial reciente de Fase 1
- documentacion de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/simulacion_dron/`
- ADRs relacionados con:
  - fiduciales simulados;
  - separacion fiducial/loop;
  - `RawMapDatabase`;
  - `GlobalPoseStore`;
  - replay de deltas.

---

## Diagnostico de partida

Tras 1C y 1D debe existir:

- un servidor nuevo minimo que recibe `OrbMap` de los wrappers;
- `RawMapDatabase` con insercion por delta/full snapshot y `arrival_id` asignado por el servidor;
- guardado de base de datos raw y journal de deltas en un path;
- modo replay desde el servidor, reinyectando deltas completos en orden de llegada;
- `GlobalPoseStore`, inicialmente vacia, capaz de almacenar poses globales de `KeyFrames` cuando un submapa se ancla.

El problema actual es que todavia no existe una capa nueva y limpia que convierta una observacion fiducial simulada en un anclaje global del submapa.

Ademas, el fiducial actual es simulado con GT de Gazebo. Por tanto, la pose GT actual del dron no coincide necesariamente con el instante exacto del `KeyFrame` que llega en un delta. El servidor debe asociar razonablemente un `KeyFrame` con una pose GT/fiducial usando timestamp, buffer temporal o la mejor aproximacion disponible.

El dataset `.record` generado en `1C` solo contiene deltas/raw y no debe asumirse suficiente para validar `1E`. En esta subfase hay que generar al menos un nuevo dataset desde simulacion viva con observaciones fiduciales persistidas, de forma que el replay pueda reproducir que KeyFrame estuvo asociado a que fiducial y con que pose absoluta sin consultar GT en vivo.

La observacion fiducial no debe ser interpretada como loop. Debe ser una observacion absoluta.

Marcadores esperados heredados:

```text
[F1B-ORBMAP-RX]
[F1C-RAWDB-INSERT-DELTA]
[F1C-RAWDB-SAVE]
[F1C-REPLAY-DELTA]
[F1D-POSESTORE-ANCHOR-SET]
[F1D-POSESTORE-STATS]
```

Marcadores nuevos de esta subfase:

```text
[F1E-FID-CANDIDATE-GT]
[F1E-FID-KF-ASSOC]
[F1E-FID-OBS]
[F1E-FID-FIRST-ANCHOR]
[F1E-FID-WORLD-T-LOCAL]
[F1E-FID-KF-HARD]
[F1E-FID-JOURNAL-SAVE]
[F1E-FID-REPLAY-OBS]
```

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Se permite crear/modificar archivos nuevos del backend:

- `orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp`
- `orbslam3_multi/src/fiducial_anchor_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/include/orbslam3_multi/types.hpp` o archivo equivalente de tipos compartidos, si ya existe o si es necesario crearlo
- `orbslam3_multi/CMakeLists.txt`

### `orbslam3_server`

Se permite modificar el servidor nuevo y sus utilidades de replay/record:

- `orbslam3_server/src/global_map_server.cpp`
- `orbslam3_server/include/...` si existe cabecera del servidor nuevo
- archivos auxiliares nuevos de conversion o replay del servidor, si en subfases anteriores se crearon
- `orbslam3_server/launch/global_map_server.launch.py`
- `orbslam3_server/CMakeLists.txt`

### `simulacion_dron` y pruebas

Solo si es necesario para generar o ajustar la prueba:

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`
- configuracion de prueba ya prevista por el pipeline, si existe

### Documentacion

- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`
- `codex/contexto/paquetes/orbslam3_multi/fiducial_anchor_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`
- `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- `codex/contexto/paquetes/orbslam3_server/launches.md`
- historial de Fase 1/subfase 1E
- documentacion de datasets/replay creada en 1C, si existe

---

## Archivos prohibidos

- `ORB_SLAM3/` salvo permiso explicito del usuario.
- `orbslam3_ros2` salvo que se demuestre un bug de exportacion que bloquee la subfase.
- `orbslam3_msgs` salvo necesidad demostrada; no redisenar mensajes en esta subfase.
- `global_map_server_antiguo.cpp`, salvo lectura o comentarios/documentacion. No anadir funcionalidad nueva al servidor antiguo.
- Archivos dentro de `orbslam3_multi/legacy/`, salvo documentacion o creacion de copias congeladas no compiladas si ya estaba previsto.
- Rutas `build/`, `install/`, `log/` como codigo fuente.
- Implementaciones de loops, fusión de landmarks u optimización local/global.

---

## Funciones, clases o nodos que hay que localizar

El `planificador_fase` debe localizar por busqueda estatica antes de implementar:

- nodo nuevo del servidor creado en 1B;
- callback que recibe `OrbMap` delta en el servidor nuevo;
- mecanismo de `arrival_id` creado en 1C;
- codigo de guardado/replay de deltas creado en 1C;
- clase `RawMapDatabase` creada en 1C;
- clase `GlobalPoseStore` creada en 1D;
- estructuras de identificacion:
  - `drone_id`;
  - `map_epoch`;
  - `local_keyframe_id`;
  - `global_keyframe_id`;
  - `SubmapId` o equivalente;
- parametros existentes de fiduciales en el launch/servidor antiguo, si existen;
- fuente de GT en simulacion:
  - topic de GT por dron;
  - tipo de mensaje;
  - timestamp disponible;
- topics de entrada:
  - `/dron_X/orbslam/orb_map_delta` o equivalente;
  - `/dron_X/sensor/GT/pose` o equivalente;
- topic de nube/debug publicado por el servidor nuevo, si existe desde 1F o se usa uno temporal.

No inventar nombres definitivos si el codigo ya tiene equivalentes. Adaptar nombres nuevos a las convenciones existentes del paquete.

---

## Cambios requeridos

### 1. Crear `FiducialAnchorManager` en `orbslam3_multi`

Intencion:

Separar la logica de anclaje fiducial del servidor ROS.

Responsabilidad:

- recibir observaciones fiduciales ya asociadas a un `KeyFrame`;
- no leer GT directamente;
- no conocer Gazebo;
- no publicar ROS;
- consultar `RawMapDatabase` para obtener `local_T_kf` y submapa;
- consultar `GlobalPoseStore` para saber si el submapa esta anclado;
- si no esta anclado, calcular `world_T_local`;
- llamar a `GlobalPoseStore` para registrar el anclaje;
- marcar el `KeyFrame` como fiducial real/hard fiducial;
- devolver un resultado claro al servidor.

API inicial sugerida:

```cpp
struct FiducialObservation
{
    uint64_t arrival_id;
    uint32_t drone_id;
    uint64_t map_epoch;
    uint64_t local_keyframe_id;
    uint64_t global_keyframe_id;
    int fiducial_id;
    Sophus::SE3d world_T_kf_fiducial;
    double keyframe_stamp_sec;
    double gt_stamp_sec;
    double association_dt_sec;
    double distance_to_fiducial_m;
    std::string source;
};

struct FiducialAnchorResult
{
    bool observation_accepted;
    bool first_anchor_created;
    bool submap_already_anchored;
    bool keyframe_marked_hard_fiducial;
    SubmapId submap_id;
    Sophus::SE3d world_T_local;
    std::string reason;
};

FiducialAnchorResult RegisterFiducialObservation(
    const FiducialObservation& observation,
    const RawMapDatabase& raw_db,
    GlobalPoseStore& pose_store);
```

La firma exacta puede variar si existen tipos ya definidos, pero debe mantener la separacion conceptual.

Condicion de seguridad:

`FiducialAnchorManager` no debe llamar a APIs ROS ni leer topics.

Logs obligatorios:

```text
[F1E-FID-OBS]
[F1E-FID-FIRST-ANCHOR]
[F1E-FID-WORLD-T-LOCAL]
[F1E-FID-KF-HARD]
```

---

### 2. Asociar `KeyFrame` con pose GT/fiducial en el servidor

Intencion:

Construir una observacion fiducial simulada razonable mientras no exista un fiducial visual real.

El servidor debe:

1. mantener un buffer reciente de GT por dron;
2. al recibir un delta, detectar `KeyFrames` nuevos;
3. para cada `KeyFrame` nuevo, buscar la pose GT mas cercana a su timestamp o la mejor pose GT disponible;
4. comprobar si esa pose GT cae dentro del radio de algun fiducial configurado;
5. si cae dentro, construir `FiducialObservation`;
6. llamar a `FiducialAnchorManager::RegisterFiducialObservation(...)`.

La asociacion debe registrar:

- `kf_stamp`;
- `gt_stamp`;
- `dt` entre ambos;
- distancia al centro del fiducial;
- fiducial elegido;
- fuente de la observacion.

Fuentes recomendadas:

```text
SIM_GT_TEMPORAL_ASSOCIATION
SIM_GT_NEAREST_POSE
REPLAY_RECORDED_FIDUCIAL
REAL_FIDUCIAL_DETECTION
```

En esta subfase solo se implementan las fuentes simuladas y replay. `REAL_FIDUCIAL_DETECTION` puede quedar reservado.

Condicion de seguridad:

Si no hay GT cercano, si el timestamp no es fiable o si la distancia al fiducial supera el umbral, no se debe crear observacion fiducial.

Logs obligatorios:

```text
[F1E-FID-CANDIDATE-GT]
[F1E-FID-KF-ASSOC]
```

Ejemplo:

```text
[F1E-FID-KF-ASSOC] arrival_id=37 drone_id=1 epoch=0 kf=42 fid=2 kf_stamp=123.420 gt_stamp=123.398 dt=0.022 dist_to_fid=0.18 source=SIM_GT_TEMPORAL_ASSOCIATION
```

---

### 3. Calcular el anclaje inicial del submapa

Intencion:

Cuando un submapa recibe su primera observacion fiducial valida, obtener la transformacion rigida que coloca el frame local ORB-SLAM3 del submapa en `world`.

Formula:

```text
world_T_local = world_T_kf_fiducial * inverse(local_T_kf)
```

Donde:

- `world_T_kf_fiducial` es la pose asociada por el servidor usando GT/fiducial simulado;
- `local_T_kf` es la pose local original del `KeyFrame` en `RawMapDatabase`.

El anclaje debe registrarse en `GlobalPoseStore`.

Condicion de seguridad:

No modificar la pose local original en `RawMapDatabase`.

Logs obligatorios:

```text
[F1E-FID-FIRST-ANCHOR]
[F1E-FID-WORLD-T-LOCAL]
```

Ejemplo:

```text
[F1E-FID-FIRST-ANCHOR] fid=2 drone_id=1 epoch=0 kf=42 local_t=(0.31,0.02,0.00) gt_t=(5.00,2.00,1.00) world_T_local_t=(4.69,1.98,1.00) yaw=0.03
```

---

### 4. Registrar que un `KeyFrame` es fiducial real/hard fiducial

Intencion:

Distinguir entre:

```text
KeyFrame realmente observado en fiducial
```

y:

```text
KeyFrame normal perteneciente a un submapa anclado por fiducial
```

Solo el primero debe quedar marcado como fiducial real o hard fiducial.

Esto puede guardarse en `GlobalPoseStore`, en `FiducialAnchorManager` o en una estructura ligera compartida, pero no debe contaminar `RawMapDatabase` con poses corregidas.

Condicion de seguridad:

No marcar todos los KFs del submapa como hard fiducial. Solo el `KeyFrame` asociado a la observacion fiducial.

Log obligatorio:

```text
[F1E-FID-KF-HARD]
```

---

### 5. Ampliar el dataset guardado para incluir observaciones fiduciales

Intencion:

Permitir que los datasets guardados desde simulacion puedan reproducir tambien los eventos fiduciales en pruebas replay sin Gazebo.

Aunque `subfase_1C.md` no se modifica, esta subfase debe ampliar el formato o mecanismo de guardado creado en 1C para que incluya un journal adicional:

```text
fiducial_observation_journal
```

Cada entrada debe estar asociada al `arrival_id` del delta donde se detecto/asocio el fiducial.

Contenido minimo:

```text
arrival_id
drone_id
map_epoch
local_keyframe_id
global_keyframe_id
fiducial_id
world_T_kf_fiducial
kf_stamp
gt_stamp
association_dt_sec
distance_to_fiducial_m
source
```

Durante replay:

1. el servidor carga el dataset guardado;
2. reinyecta deltas por `arrival_id` en `RawMapDatabase`;
3. despues de insertar cada delta, consulta si existen observaciones fiduciales asociadas a ese `arrival_id`;
4. si existen, llama a `FiducialAnchorManager` con `source=REPLAY_RECORDED_FIDUCIAL`.

Condicion de seguridad:

El replay no debe usar GT en vivo. Debe usar exclusivamente observaciones fiduciales grabadas.

Logs obligatorios:

```text
[F1E-FID-JOURNAL-SAVE]
[F1E-FID-REPLAY-OBS]
```

Ejemplo:

```text
[F1E-FID-REPLAY-OBS] arrival_id=37 drone_id=1 epoch=0 kf=42 fid=2 source=REPLAY_RECORDED_FIDUCIAL
```

---

### 6. Estadisticas de anclaje y validacion

Intencion:

Al final de la simulacion o replay debe haber resumen claro.

Logs obligatorios:

```text
[F1E-FID-STATS]
[F1E-POSESTORE-STATS]
```

Ejemplo:

```text
[F1E-FID-STATS] observations=4 accepted=2 rejected=2 anchors_created=2 replay_observations=0 hard_fiducial_kfs=2
```

---

## Cambios prohibidos

- No implementar deteccion visual real de fiduciales.
- No tratar fiduciales como loops.
- No crear `LOCAL_LOOP_OPT_TASK` ni optimizacion local/global.
- No fusionar landmarks.
- No publicar fused map avanzado si no existia ya.
- No modificar poses locales de `RawMapDatabase`.
- No marcar todos los `KeyFrames` de un submapa como fiduciales/hard-fixed.
- No usar `FIDUCIAL_GLOBAL_DEBT` como prior fuerte.
- No cambiar `orbslam3_msgs` salvo bloqueo tecnico demostrado.
- No modificar el wrapper salvo bug confirmado.
- No limpiar legacy masivamente.
- No hacer que `FiducialAnchorManager` dependa de ROS, Gazebo o mensajes `geometry_msgs` directamente si puede evitarse.

---

## Paquetes a compilar

Comando esperado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron
```

Si durante la implementacion se confirma que `simulacion_dron` no se modifica, se puede compilar solo:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
```

Si falla el build:

```bash
./codex/herramientas/reduce_build_log.sh
```

El diagnosticador debe leer primero:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

---

## Pruebas Gazebo requeridas

### Prueba 1 — anclaje fiducial simulado en vivo y grabacion de journal fiducial

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. esperar arranque de Gazebo, wrappers y servidor nuevo;
2. mover `dron_1` al fiducial 2 con altura aproximada `z=1.0` y yaw `90°`;
3. esperar a que ORB-SLAM3 genere al menos un delta con `KeyFrame` cerca del fiducial 2;
4. mover `dron_2` encima de `dron_1`, con separacion vertical segura, para provocar observacion/covisibilidad cerca del mismo evento espacial;
5. esperar a que ORB-SLAM3 genere deltas de `dron_2`;
6. mover `dron_1` a la izquierda del fiducial 2, manteniendo altura y yaw razonables;
7. mover `dron_2` encima de `dron_1`;
8. esperar a que ambos wrappers publiquen nuevos deltas;
9. mover `dron_1` de vuelta al fiducial 2;
10. esperar a que ORB-SLAM3 genere al menos otro delta con `KeyFrame` cerca del fiducial 2;
11. mover `dron_2` encima de `dron_1`;
12. esperar unos segundos para que el servidor procese observaciones, anclajes y guardado del journal fiducial;
13. terminar la simulacion.

La trayectoria debe reproducir la intencion operacional:

```text
Dron1 a fiducial 2
Dron2 encima de Dron1
Dron1 a la izquierda
Dron2 encima de Dron1
Dron1 de vuelta al fiducial 2
Dron2 encima de Dron1
```

El `tray_prueba_1.yaml` debe activar o acompanar el modo record necesario para generar un `.record` nuevo que incluya tanto el journal raw de `1C` como el `fiducial_observation_journal` de `1E`. No basta con reutilizar sin ampliar `rawdb_prueba_1.record` de `1C`, porque ese dataset no contiene la asociacion `KeyFrame`-fiducial-pose requerida por esta subfase.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observacion esperada en RViz2:

```text
Si ya existe publicacion debug de poses/nube anclada, debe verse que el submapa o los KeyFrames anclados aparecen aproximadamente en la zona del fiducial.
Si todavia no existe publicacion visual suficiente, documentar que la validacion visual no aplica en 1E y basarse en logs.
```

Criterio especifico de la prueba:

- deben aparecer observaciones `[F1E-FID-KF-ASSOC]`;
- debe aparecer al menos un `[F1E-FID-FIRST-ANCHOR]`;
- debe aparecer `[F1E-FID-WORLD-T-LOCAL]`;
- debe aparecer `[F1E-FID-KF-HARD]`;
- debe guardarse el journal fiducial mediante `[F1E-FID-JOURNAL-SAVE]`;
- el dataset resultante debe contener, para cada observacion aceptada, `arrival_id`, `KeyFrame`, fiducial, pose absoluta asociada y metricas de asociacion temporal/distancia.

---

### Prueba 2 — replay con observaciones fiduciales grabadas

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Esta prueba no debe requerir movimiento real de drones si el modo replay se ejecuta sin Gazebo completo. Si la herramienta `run_simulation.sh` exige launch de Gazebo, se puede lanzar el sistema minimo y activar replay por parametros.

Secuencia esperada:

1. cargar el dataset guardado en Prueba 1, generado ya con `fiducial_observation_journal`;
2. crear una `RawMapDatabase` vacia;
3. reinyectar deltas por `arrival_id` mediante timer;
4. al llegar a un `arrival_id` con observacion fiducial registrada, llamar a `FiducialAnchorManager`;
5. comprobar que el anclaje se reproduce sin GT en vivo;
6. terminar replay.

Comando recomendado si se usa simulacion estandar:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si existe un launch/debug de replay sin Gazebo, `planificador_fase` puede usarlo y documentarlo en historial.

Observacion esperada en RViz2:

```text
No es obligatoria. El criterio principal de Prueba 2 son los logs de replay.
```

Criterio especifico de la prueba:

- debe aparecer `[F1E-FID-REPLAY-OBS]`;
- debe reaparecer `[F1E-FID-FIRST-ANCHOR]` o log equivalente de submapa ya anclado por replay;
- no debe usarse GT en vivo para generar observaciones durante replay;
- el numero de observaciones fiduciales replay debe coincidir con el journal del dataset cargado.
- si el dataset cargado no contiene `fiducial_observation_journal`, la prueba debe fallar o marcarse como no valida para `1E`; no debe inventar observaciones desde GT durante replay.

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-|F1D-|F1E-|FID|POSESTORE|RAWDB|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1E-FID-REPLAY|F1E-FID|F1D-POSESTORE|RAWDB|ERROR|FATAL|Segmentation fault|Killed
```

La reduccion genera:

```text
codex/archivos_auxiliares/logs/prueba_1.reduced.log
codex/archivos_auxiliares/logs/prueba_2.reduced.log
```

Debe conservarse:

```text
codex/archivos_auxiliares/logs/prueba_1.log
codex/archivos_auxiliares/logs/prueba_2.log
```

Si el reducido no muestra datos suficientes, revisar el log completo antes de repetir Gazebo o concluir que el codigo no emitio el marcador.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. las pruebas requeridas se ejecutan o se justifica tecnicamente cualquier sustitucion de Prueba 2 por un replay sin Gazebo;
3. `scenario_runner_node` se ejecuta en Prueba 1;
4. los goals requeridos de Prueba 1 terminan con `success=true`;
5. el servidor asocia al menos un `KeyFrame` con una observacion fiducial simulada;
6. aparece `[F1E-FID-KF-ASSOC]` con `kf_stamp`, `gt_stamp`, `dt` y distancia al fiducial;
7. `FiducialAnchorManager` recibe la observacion y aparece `[F1E-FID-OBS]`;
8. se calcula y loggea `world_T_local` mediante `[F1E-FID-WORLD-T-LOCAL]`;
9. `GlobalPoseStore` registra el anclaje del submapa;
10. solo el `KeyFrame` observado queda marcado como fiducial real/hard fiducial;
11. el dataset guardado incluye `fiducial_observation_journal` o mecanismo equivalente;
12. el replay reproduce al menos una observacion fiducial desde el journal mediante `[F1E-FID-REPLAY-OBS]`;
13. no aparecen errores graves no explicados;
14. la observacion en RViz2 se documenta si es posible; si no es observable todavia, se documenta explicitamente;
15. el historial y la documentacion de paquetes quedan actualizados.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- el servidor no recibe deltas;
- no se puede asociar ningun `KeyFrame` a fiducial en Prueba 1;
- falta `[F1E-FID-OBS]`;
- no se calcula `world_T_local`;
- `GlobalPoseStore` no registra ningun anclaje;
- el replay usa GT en vivo en lugar de observaciones grabadas;
- aparece un error grave no explicado;
- se modifica `RawMapDatabase` sobrescribiendo poses locales originales.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- Prueba 1 genera anclaje correcto;
- pero el journal fiducial o replay no queda completamente validado;
- o RViz2 no permite observar el resultado pero los logs principales son correctos;
- o solo un dron consigue observacion fiducial por falta de `KeyFrames` cerca del fiducial.

La subfase debe marcarse como `BLOQUEADA` solo si:

- falta informacion critica sobre topics de GT o fiduciales;
- el modo replay de 1C no existe o no puede usarse y no se puede adaptar con cambios minimos;
- ORB-SLAM3 no genera `KeyFrames` cerca del fiducial en ninguna prueba y no se puede ajustar la trayectoria sin cambiar el objetivo.

---

## Documentacion a actualizar

Actualizar siempre:

- historial de Fase 1/subfase 1E;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- `codex/contexto/paquetes/orbslam3_multi/fiducial_anchor_manager.md`;
- `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
- `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
- `codex/contexto/paquetes/orbslam3_server/launches.md` si se anaden parametros de replay/fiducial;
- documento de datasets/pruebas clave si se creo en 1C.

La documentacion debe explicar:

- que `FiducialAnchorManager` no detecta fiduciales por si mismo;
- que el servidor construye observaciones fiduciales usando GT simulado;
- como se asocia temporalmente `KeyFrame` con GT;
- que el dataset/replay incluye `fiducial_observation_journal`;
- que los fiduciales son observaciones absolutas y no loops;
- que solo los KFs realmente observados en fiducial se marcan como hard fiducial;
- que `RawMapDatabase` conserva poses locales originales;
- que el anclaje se registra en `GlobalPoseStore`.

No marcar la subfase como `realizado` si el criterio de exito no se cumple.
