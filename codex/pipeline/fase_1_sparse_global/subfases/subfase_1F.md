# Subfase 1F — GlobalMapBuilder y LandmarkScoreManager para publicar nube sparse con score

## Estado

```text
realizado
```

Resultado validado el 2026-07-08: `CONSEGUIDA`.

Evidencia principal:

- build separado de `orbslam3_multi` y `orbslam3_server` con `BUILD-EXIT-CODE 0`;
- prueba 1 live con Gazebo y `SIM-DONE prueba=1 success=true`;
- prueba 2 replay lento del `.record` a `1.0` s por delta y `SIM-DONE prueba=2 success=true`;
- hotfix posterior de `body_T_camera` validado con build y replay lento para RViz2;
- `[F1F-BODY-CAMERA-CONFIG] source=launch use_optical=true body_T_camera_t=(0.100,0.030,0.030) ...`;
- `[F1F-BODY-CAMERA-APPLY] ... body_t=(...) camera_t=(...)`;
- `[F1C-REPLAY-DONE] entries=318 journal=318 deltas=318 full=0 fiducial_observations=72 submaps=3 kfs=179 mps=22885`;
- `[F1F-GLOBALMAP-PUBLISH] topic=/global_sparse_cloud frame_id=world points_from_backend=22394 points_published=22394 ... score_field=true drone_id_field=true map_epoch_field=true`;
- no aparecieron `FATAL`, `Segmentation fault` ni `Killed`; en prueba live solo apareció el cierre conocido de Gazebo `exit code 255` después de `SIM-DONE success=true`.
- el dataset diferencial se conservo temporalmente como `codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record`, pero se borro al iniciar `1G` a peticion del usuario.

---

## Objetivo tecnico

Crear en `orbslam3_multi` la infraestructura minima para construir y publicar una nube sparse global en `world` usando solo datos ya anclados.

La subfase introduce dos responsabilidades nuevas:

1. `LandmarkScoreManager`: capa ligera que mantiene el score de cada `MapPoint`/landmark a partir de eventos semanticos.
2. `GlobalMapBuilder`: clase que construye los puntos globales publicables usando `RawMapDatabase`, `GlobalPoseStore` y `LandmarkScoreManager`.

El servidor no debe calcular el score. El servidor solo debe:

- pedir al backend los puntos globales con score;
- convertirlos a mensajes ROS;
- publicar la nube;
- registrar logs de validacion.

En esta subfase el score solo puede inicializarse o actualizarse usando informacion cruda exportada por ORB-SLAM3 a traves del wrapper, por ejemplo:

- `observations_count`;
- `found_ratio`;
- `is_bad`;
- existencia de descriptor valido;
- pertenencia a submapa anclado.

No se deben aplicar todavia cambios de score por fusion, loops, reobservacion, no reobservacion, optimizacion ni consistencia geometrica. Esos eventos deben quedar previstos en la API, pero no usados funcionalmente en 1F.

A partir de esta subfase debe poder verse una nube sparse en RViz2. La validacion automatica principal sigue viniendo del build, la simulacion y los logs; despues de la ejecucion, el usuario podra comentar si la nube visualizada en RViz2 es coherente o presenta problemas.

Nota de cierre `1F`: la primera visualizacion detecto que la nube se estaba anclando con la pose `world_T_body` como si fuera `world_T_camera`. El hotfix de `1F` mueve `body_T_camera` al backend activo y exige que las observaciones fiduciales calculen:

```text
world_T_camera = world_T_body * body_T_camera
world_T_local = world_T_camera * inverse(local_T_kf)
```

Por tanto, futuras pruebas de `1F` y regresiones visuales deben comprobar los marcadores `[F1F-BODY-CAMERA-CONFIG]` y `[F1F-BODY-CAMERA-APPLY]`. El antiguo `prueba_diff_anclaje.record` ya no esta disponible tras `1G`; si se quiere una regresion visual equivalente, hay que usar el record vigente o generar uno nuevo.

---

## Contexto obligatorio a leer

Antes de implementar, Codex debe leer:

- `AGENTS.md`;
- `codex/contexto/01_ESTADO_ACTUAL.md`;
- `codex/pipeline/PIPELINE_MAESTRO.md`;
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1A.md`;
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1B.md`;
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1C.md`;
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1D.md`;
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1E.md`;
- historial reciente de la fase 1;
- `codex/contexto/paquetes/orbslam3_server/`;
- `codex/contexto/paquetes/orbslam3_multi/`;
- ADRs relacionados con submapas, fiduciales, loops, fused map y separacion servidor/backend.

Si alguna de las subfases 1C, 1D o 1E no esta implementada o no esta validada, `planificador_fase` debe adaptar 1F a la interfaz real existente sin inventar APIs incompatibles.

---

## Diagnostico de partida

Tras 1C, 1D y 1E deberia existir:

- un servidor nuevo minimo que recibe datos del wrapper;
- `RawMapDatabase` con datos raw separados por `(drone_id, map_epoch)`;
- `arrival_id` para deltas/snapshots y persistencia/replay;
- `GlobalPoseStore` vacio inicialmente y con poses globales solo cuando se ancla un submapa;
- `FiducialAnchorManager` capaz de recibir una observacion fiducial simulada y anclar un submapa;
- journal de observaciones fiduciales para replay.

El problema actual es que, aunque el sistema ya puede almacenar datos raw y calcular poses globales para submapas anclados, todavia no existe una salida limpia para construir una nube sparse global publicable con score por punto desde el backend nuevo.

La subfase 1F debe demostrar que el flujo nuevo llega hasta:

```text
RawMapDatabase + GlobalPoseStore + LandmarkScoreManager
    -> GlobalMapBuilder
    -> servidor ROS
    -> PointCloud2 en world
    -> RViz2
```

Marcadores esperados de fases anteriores que pueden aparecer en el log:

```text
[F1C-RAWDB-INSERT-DELTA]
[F1C-RAWDB-STATS]
[F1C-REPLAY-DELTA]
[F1D-POSESTORE-ANCHOR-SET]
[F1D-POSESTORE-KF-WORLD-POSE]
[F1E-FID-OBS]
[F1E-FID-FIRST-ANCHOR]
[F1E-FID-WORLD-T-LOCAL]
```

Nuevos marcadores obligatorios de esta subfase:

```text
[F1F-SCORE-INIT]
[F1F-SCORE-UPDATE-ORBSLAM]
[F1F-SCORE-STATS]
[F1F-GLOBALMAP-BUILD]
[F1F-GLOBALMAP-POINT-STATS]
[F1F-GLOBALMAP-PUBLISH]
[F1F-BODY-CAMERA-CONFIG]
[F1F-BODY-CAMERA-APPLY]
```

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Crear o modificar, segun la estructura real del paquete:

- `orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp`
- `orbslam3_multi/src/landmark_score_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp`
- `orbslam3_multi/src/global_map_builder.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_sparse_point.hpp` si se decide separar tipos
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp` solo para exponer consultas necesarias
- `orbslam3_multi/src/raw_map_database.cpp` solo para exponer consultas necesarias o notificar altas/updates de MPs
- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp` solo para exponer consultas de poses globales/anclajes
- `orbslam3_multi/src/global_pose_store.cpp` solo para exponer consultas de poses globales/anclajes
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_multi/package.xml` solo si hiciera falta una dependencia real nueva

### `orbslam3_server`

Modificar solo lo necesario para integrar la salida del backend:

- `orbslam3_server/src/global_map_server.cpp`
- launch activo del servidor nuevo si se necesita declarar parametros de publicacion
- `orbslam3_server/CMakeLists.txt` solo si hace falta enlazar nuevos archivos/targets
- `orbslam3_server/package.xml` solo si hiciera falta una dependencia real nueva

### Archivos auxiliares y documentacion

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`, si se usa replay como segunda prueba
- documentacion de paquetes afectados en `codex/contexto/paquetes/`
- historial de fase/subfase
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real

---

## Archivos prohibidos

No modificar en esta subfase:

- `ORB_SLAM3/`;
- wrapper `orbslam3_ros2`, salvo que exista un bug critico demostrado que impida publicar datos raw ya existentes;
- `orbslam3_msgs`, salvo que sea estrictamente imposible publicar score en `PointCloud2` sin cambiar mensajes, en cuyo caso debe preguntarse antes;
- logica de loops;
- logica de fusion real de landmarks;
- optimizacion local/global;
- logica de segunda visita a fiducial;
- archivos legacy de `orbslam3_multi/legacy/` o `global_map_server_antiguo.cpp`, salvo para consulta estatica;
- paquetes no relacionados con Fase 1F.

No se debe reintroducir logica del servidor antiguo dentro del servidor nuevo. Si se necesita una utilidad antigua, debe migrarse al backend nuevo o documentarse como pendiente.

---

## Funciones, clases o nodos que hay que localizar

`planificador_fase` debe localizar con busqueda estatica antes de implementar:

- nodo ROS nuevo del servidor en `orbslam3_server/src/global_map_server.cpp`;
- callback que recibe `OrbMap` delta;
- ruta de replay si ya existe desde 1C;
- integracion actual con `RawMapDatabase`;
- integracion actual con `GlobalPoseStore`;
- integracion actual con `FiducialAnchorManager`;
- publisher actual o pendiente de la nube global;
- topic previsto para la nube sparse global;
- tipos internos usados por `RawMapDatabase` para `MapPoint`, `KeyFrame`, `SubmapId` y `GlobalMapPointId`.

Clases nuevas esperadas:

- `LandmarkScoreManager`
- `GlobalMapBuilder`

Tipos nuevos esperados, nombres ajustables si el codigo usa otra convencion:

- `GlobalSparsePoint`
- `ScoreEvent`
- `ScoreEventType`
- `GlobalMapBuildResult`
- `LandmarkScoreStats`

No inventar nombres incompatibles con la estructura real. Si ya existe una clase equivalente, extenderla solo si encaja con la responsabilidad definida aqui.

---

## Cambios requeridos

### 1. Crear `LandmarkScoreManager`

Intencion:

Crear una clase ligera que mantenga el score de cada punto/landmark y que reciba eventos semanticos desde otras clases del backend.

Responsabilidad:

```text
MapPoint/FusedLandmark id -> score actual
MapPoint/FusedLandmark id -> origen/estado basico del score
estadisticas de score
```

En 1F solo se deben aplicar eventos basados en informacion ORB-SLAM3 raw.

Eventos minimos:

```cpp
enum class ScoreEventType
{
    NewMapPoint,
    OrbSlamQualityUpdate,
    MarkedBad,

    // Eventos reservados para fases futuras.
    FusionConfirmed,
    Reobserved,
    NotReobserved,
    OptimizationAccepted,
    GeometryInconsistent
};
```

Condicion de seguridad:

Las clases externas no deben modificar scores directamente con valores arbitrarios. Deben emitir eventos semanticos al `LandmarkScoreManager`.

En 1F, si se definen eventos futuros, deben quedar sin uso funcional o con efecto neutro documentado.

Logs:

```text
[F1F-SCORE-INIT]
[F1F-SCORE-UPDATE-ORBSLAM]
[F1F-SCORE-MARKED-BAD]
[F1F-SCORE-STATS]
```

Ejemplo:

```text
[F1F-SCORE-INIT] mp=281474976710901 obs=4 found_ratio=0.750 is_bad=false score=0.620 reason=NewMapPoint
```

Ejemplo de estadisticas:

```text
[F1F-SCORE-STATS] tracked_points=3820 score_min=0.000 score_mean=0.581 score_max=0.940 bad_points=12
```

---

### 2. Inicializar/actualizar scores desde datos ORB-SLAM3 raw

Intencion:

Cuando `RawMapDatabase` inserte o actualice `MapPoints`, el backend debe poder informar a `LandmarkScoreManager`.

Forma permitida:

- `RawMapDatabase` devuelve en su `InsertDelta`/`InsertFullSnapshot` una lista de MPs nuevos/actualizados; o
- el servidor/backend consulta tras insertar que MPs cambiaron; o
- `LandmarkScoreManager` recorre los MPs nuevos/actualizados usando `RawMapDatabase`.

En cualquier caso, la politica de score vive en `LandmarkScoreManager`, no en el servidor.

Score inicial sugerido en 1F:

```text
score = funcion simple y acotada [0, 1] de:
- observations_count;
- found_ratio;
- is_bad;
- descriptor valido.
```

La formula exacta debe documentarse en el codigo y en la documentacion de paquete. Debe ser conservadora y depurable.

Condicion de seguridad:

No usar todavia fusion, loop, optimizacion, numero de drones, score global legacy ni ground truth para modificar score.

Logs:

```text
[F1F-SCORE-UPDATE-ORBSLAM]
```

---

### 3. Crear `GlobalMapBuilder`

Intencion:

Construir una salida de puntos sparse globales candidatos a publicacion, cada uno con su score.

Responsabilidad:

```text
consultar RawMapDatabase;
consultar GlobalPoseStore;
consultar LandmarkScoreManager;
tomar solo submapas anclados;
ignorar MPs bad o no publicables;
transformar posiciones locales a world;
devolver puntos con score.
```

Salida esperada:

```cpp
struct GlobalSparsePoint
{
    uint64_t global_mappoint_id;
    uint32_t drone_id;
    uint64_t map_epoch;

    double x;
    double y;
    double z;

    float score;

    uint32_t observations;
    bool from_anchored_submap;
    bool is_fused;
};
```

`is_fused` puede estar siempre en `false` en 1F si la fusion real aun no existe.

Condicion de seguridad:

`GlobalMapBuilder` no debe calcular score por su cuenta. Debe llamar a `LandmarkScoreManager`.

`GlobalMapBuilder` no debe crear anchors ni modificar poses. Debe usar `GlobalPoseStore`.

Logs:

```text
[F1F-GLOBALMAP-BUILD]
[F1F-GLOBALMAP-POINT-STATS]
[F1F-GLOBALMAP-SKIP-UNANCHORED]
```

Ejemplo:

```text
[F1F-GLOBALMAP-BUILD] anchored_submaps=1 skipped_unanchored=1 raw_points=3120 candidate_points=2890 returned_points=2890
```

Ejemplo:

```text
[F1F-GLOBALMAP-POINT-STATS] points=2890 score_min=0.120 score_mean=0.584 score_max=0.930 bad_skipped=14
```

---

### 4. Publicar nube sparse global con score desde el servidor nuevo

Intencion:

El servidor debe publicar la nube sparse global construida por `GlobalMapBuilder`.

Requisitos:

- frame esperado: `world` o el frame global definido por parametros;
- publicar solo puntos devueltos por el backend;
- incluir campo `score` en la nube si se usa `sensor_msgs/msg/PointCloud2`;
- incluir, si es viable sin complicar la fase, campos `drone_id` y `map_epoch`;
- loggear numero de puntos recibidos del backend y publicados.

Topic esperado:

`planificador_fase` debe localizar el topic legacy o el topic definido por el servidor nuevo. Si no existe, crear uno claro, por ejemplo:

```text
/global_sparse_cloud
```

El nombre final debe documentarse.

Parametro opcional:

```text
global_map_min_score_to_publish
```

Valor por defecto recomendado en 1F:

```text
0.0
```

En 1F se recomienda publicar todos los puntos candidatos con score. Si se aplica filtro, debe quedar registrado en log.

Logs:

```text
[F1F-GLOBALMAP-PUBLISH]
```

Ejemplo:

```text
[F1F-GLOBALMAP-PUBLISH] topic=/global_sparse_cloud frame_id=world points_from_backend=2890 points_published=2890 min_score_to_publish=0.000 score_field=true
```

---

### 5. Preparar validacion visual en RViz2

Intencion:

Desde esta subfase debe poder verse una nube sparse basica colocada en `world` para submapas anclados.

Se debe documentar:

- topic publicado;
- frame usado;
- si el campo `score` esta disponible;
- como visualizar la nube en RViz2;
- si RViz2 no permite colorear facilmente por score, al menos confirmar que el campo existe en `PointCloud2`.

Codex debe analizar logs y dejar la subfase lista para inspeccion visual. El usuario podra comentar despues si la nube se ve bien o no.

---

## Cambios prohibidos

No hacer en 1F:

- no implementar fusion real de landmarks;
- no subir score por fusion;
- no bajar score por no reobservacion;
- no subir score por optimizacion aceptada;
- no usar loops para modificar score;
- no calcular score en el servidor;
- no publicar union bruta de todos los `MapPoints` sin pasar por `GlobalMapBuilder`;
- no publicar submapas no anclados;
- no transformar poses modificando `RawMapDatabase`;
- no cambiar mensajes `orbslam3_msgs` si `PointCloud2` permite incluir score;
- no usar ground truth para score;
- no recuperar masivamente codigo legacy en `global_map_server.cpp`;
- no implementar subfases futuras 1G, 1H, 1I, etc.

---

## Paquetes a compilar

Comando esperado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
```

Si el cambio requiere reconstruir mensajes por una razon justificada, `implementador_fase` debe añadir `orbslam3_msgs` al build y registrarlo en el historial. En principio 1F no deberia modificar mensajes.

Si el build falla:

```bash
./codex/herramientas/reduce_build_log.sh
```

Despues, `diagnosticador_build` debe leer:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

Corregir solo el primer error real y repetir build.

---

## Pruebas Gazebo requeridas

### Prueba 1 — nube sparse anclada visible

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. esperar arranque de Gazebo, wrappers y servidor nuevo;
2. mover dron 1 al fiducial 2 con altura 1.0 y yaw 90 grados;
3. mover dron 2 al fiducial 2 con altura 1.3 y yaw 90 grados;
4. mover dron 1 en `x=-8` sin cambiar altura ni yaw;
5. mover dron 2 en `x=-8` sin cambiar altura ni yaw;
6. mover dron 1 de vuelta al fiducial 2;
7. mover dron 2 de vuelta al fiducial 2;
8. esperar unos segundos para que se publiquen deltas, fiduciales, poses globales y nube;
9. terminar simulacion.

Tiempos sugeridos:

```text
20 s por movimiento
```

Si la sintaxis exacta del YAML de trayectorias no esta clara, `implementador_fase` debe consultar ejemplos existentes en `codex/archivos_auxiliares/` o en paquetes de simulacion. Si no puede inferirla, debe preguntar antes de inventar un formato incompatible.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observacion esperada en RViz2:

```text
Debe aparecer una nube sparse global basica en frame world.
La nube debe provenir solo de submapas anclados.
La nube debe estar aproximadamente cerca del fiducial y trayectoria recorrida.
Los puntos deben tener score asociado aunque RViz2 no lo coloree automaticamente.
```

La observacion visual no sustituye a los logs. Si Codex no puede observar RViz2 directamente, debe reportar que la subfase queda lista para revision visual del usuario y documentar los topics/frames.

---

### Prueba 2 — replay de dataset grabado, si existe

Esta prueba se ejecuta si ya existe un dataset guardado por 1C/1E con:

- delta journal;
- observaciones fiduciales asociadas;
- datos suficientes para anclar al menos un submapa.

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Si el modo replay no usa Gazebo o no usa trayectorias, el archivo puede ser minimo o la subfase debe documentar el mecanismo real usado por `run_simulation.sh`/launch.

Secuencia esperada:

1. arrancar servidor nuevo en modo replay;
2. cargar dataset desde `codex/archivos_auxiliares/`;
3. reinyectar deltas en orden de `arrival_id` con timer;
4. reinyectar observaciones fiduciales registradas cuando corresponda;
5. construir y publicar nube sparse global con score;
6. terminar cuando el replay acabe.

Comando orientativo:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si el replay requiere otro launch o parametro, `planificador_fase` debe indicarlo y justificarlo en historial. El launch principal del proyecto sigue siendo `simulacion_dron multi_dron.launch.py` salvo necesidad tecnica real.

Observacion esperada en RViz2:

```text
La nube sparse publicada por replay debe ser coherente con la prueba grabada.
No debe ser necesario repetir toda la simulacion de Gazebo para validar cambios posteriores del builder/score.
```

Si no existe dataset compatible, la Prueba 2 puede marcarse como `PARCIAL` justificado, pero la Prueba 1 sigue siendo obligatoria.

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1C-RAWDB|F1D-POSESTORE|F1E-FID|F1F-SCORE|F1F-GLOBALMAP|POINTCLOUD|global_sparse_cloud|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1C-RAWDB|F1D-POSESTORE|F1E-FID|F1E-FID-REPLAY|F1F-SCORE|F1F-GLOBALMAP|POINTCLOUD|global_sparse_cloud|ERROR|FATAL|Segmentation fault|Killed
```

La reduccion genera:

```text
codex/archivos_auxiliares/logs/prueba_1.reduced.log
codex/archivos_auxiliares/logs/prueba_2.reduced.log
```

y conserva:

```text
codex/archivos_auxiliares/logs/prueba_1.log
codex/archivos_auxiliares/logs/prueba_2.log
```

Si el reducido no muestra datos suficientes, revisar el log completo antes de repetir Gazebo o concluir que el codigo no emitio el marcador.

Comandos:

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 1 \
  --patterns "SCENARIO-RUNNER|GOAL|RESULT|success|F1C-RAWDB|F1D-POSESTORE|F1E-FID|F1F-SCORE|F1F-GLOBALMAP|POINTCLOUD|global_sparse_cloud|ERROR|FATAL|Segmentation fault|Killed"
```

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 2 \
  --patterns "SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1C-RAWDB|F1D-POSESTORE|F1E-FID|F1E-FID-REPLAY|F1F-SCORE|F1F-GLOBALMAP|POINTCLOUD|global_sparse_cloud|ERROR|FATAL|Segmentation fault|Killed"
```

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. la Prueba 1 se ejecuta;
3. `scenario_runner_node` se ejecuta en la Prueba 1;
4. los goals de la Prueba 1 terminan con `success=true`, salvo fallo externo claramente documentado;
5. aparecen logs de datos raw/anclaje previos necesarios:
   - `[F1C-RAWDB-INSERT-DELTA]` o equivalente;
   - `[F1D-POSESTORE-ANCHOR-SET]` o equivalente;
   - `[F1E-FID-FIRST-ANCHOR]` o equivalente;
6. aparecen todos los marcadores tecnicos nuevos obligatorios:
   - `[F1F-SCORE-INIT]`;
   - `[F1F-SCORE-STATS]`;
   - `[F1F-GLOBALMAP-BUILD]`;
   - `[F1F-GLOBALMAP-POINT-STATS]`;
   - `[F1F-GLOBALMAP-PUBLISH]`;
7. el log de publicacion indica `points_published > 0`;
8. el log indica que la nube se publica en `world` o en el frame global correcto;
9. el log indica que existe campo `score` o que el score queda incluido de forma documentada;
10. no se publican submapas no anclados;
11. no aparecen errores graves no explicados;
12. la documentacion queda actualizada;
13. el historial registra que la subfase queda lista para inspeccion visual en RViz2.

La Prueba 2 con replay es muy recomendable. Si existe dataset compatible, tambien debe pasar. Si no existe dataset compatible, el historial debe indicarlo y la subfase puede seguir siendo `CONSEGUIDA` solo si la Prueba 1 demuestra la funcionalidad principal y queda documentado que replay se validara cuando haya dataset 1C/1E compatible.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- el servidor no arranca;
- la Prueba 1 no se ejecuta;
- no se reciben datos raw;
- no se ancla ningun submapa en la prueba requerida;
- no se construye nube global;
- no se publica ningun punto;
- falta el score de los puntos sin justificacion tecnica;
- se publican submapas no anclados;
- aparece `ERROR`, `FATAL`, `Segmentation fault` o `Killed` no explicado;
- se implementa score por fusion, loops u optimizacion en esta subfase.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- se reciben datos y se anclan submapas;
- se inicializan scores;
- pero no se consigue publicar la nube por un problema acotado de conversion ROS/PointCloud2;
- o se publica nube sin campo `score`, dejando claro que el backend si devuelve scores;
- o la Prueba 2 replay no puede ejecutarse por falta de dataset compatible, pero la Prueba 1 pasa.

La subfase debe marcarse como `BLOQUEADA` solo si:

- falta informacion critica de interfaces reales de 1C/1D/1E;
- el formato de replay/dataset impide probar 1F y no puede resolverse con cambios minimos;
- hay una dependencia externa no resuelta;
- el mismo bloqueo se repite y no puede resolverse sin redefinir subfases anteriores.

---

## Documentacion a actualizar

Actualizar siempre:

- historial de la fase/subfase;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
- documentacion del launch si cambia un parametro/topic.

Crear o actualizar, segun aplique:

```text
codex/contexto/paquetes/orbslam3_multi/landmark_score_manager.md
codex/contexto/paquetes/orbslam3_multi/global_map_builder.md
codex/contexto/paquetes/orbslam3_server/global_map_server.md
codex/contexto/paquetes/orbslam3_server/launches.md
```

La documentacion debe explicar:

- que `LandmarkScoreManager` mantiene scores por eventos semanticos;
- que en 1F solo se usan eventos derivados de datos raw de ORB-SLAM3;
- que fusion/loops/optimizacion no modifican score todavia;
- que `GlobalMapBuilder` devuelve puntos con score;
- que el servidor no calcula score;
- topic publicado;
- frame publicado;
- campos del `PointCloud2`;
- parametros nuevos como `global_map_min_score_to_publish`, si existen;
- logs de validacion;
- como observar la nube en RViz2;
- limitaciones conocidas.

El historial debe registrar:

- fecha;
- objetivo intentado;
- archivos modificados;
- paquetes compilados;
- resultado de build;
- pruebas ejecutadas;
- patrones usados para reducir logs;
- evidencia positiva;
- evidencia negativa o ausente;
- si RViz2 fue observado por el usuario o queda pendiente;
- conclusion `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`;
- siguiente paso recomendado.

No marcar la subfase como `realizado` si no se cumple el criterio de exito.

---

## Notas para fases posteriores

Esta subfase deja preparada la infraestructura de score, pero no implementa la politica completa.

Fases posteriores podran hacer que otras clases llamen a `LandmarkScoreManager` con eventos como:

```text
FusionConfirmed
Reobserved
NotReobserved
OptimizationAccepted
GeometryInconsistent
```

Regla permanente:

```text
Las clases que detectan eventos sobre puntos no deben modificar scores directamente.
Deben informar al LandmarkScoreManager de lo ocurrido.
LandmarkScoreManager decide como cambia el score.
```

En 1F, cualquier evento no basado en ORB-SLAM3 raw debe quedar sin uso funcional.
