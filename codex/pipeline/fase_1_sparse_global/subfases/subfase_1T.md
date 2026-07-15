# Subfase 1T — Contratos globales, convenciones e invariantes internos

## Estado

```text
sin hacer
```

---

## Objetivo tecnico

Cerrar la coherencia interna de toda la Fase 1 comprobando que todos los módulos nuevos usan las mismas convenciones de IDs, frames, transformadas, estados, eventos y ownership de datos.

Esta subfase no debe añadir nueva lógica SLAM. Debe validar y documentar contratos globales para evitar errores difíciles de depurar.

La idea central es:

```text
Antes de considerar cerrada la fase 1,
todas las piezas deben hablar el mismo idioma.
```

Debe quedar definido y verificado:

- cómo se identifican KeyFrames, MapPoints, submapas, fused tracks, tareas y eventos;
- qué frame usa cada pose;
- qué dirección tienen las transformadas;
- qué clase posee cada tipo de dato;
- qué clase puede modificar poses, scores o fused tracks;
- qué invariantes no pueden romperse nunca;
- cómo se comprueba automáticamente por logs y tests.

---

## Contexto obligatorio a leer

Antes de actuar, Codex debe leer:

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- todas las subfases nuevas de Fase 1:
  - `subfase_1A.md`
  - `subfase_1B.md`
  - `subfase_1C.md`
  - `subfase_1D.md`
  - `subfase_1E.md`
  - `subfase_1F.md`
  - `subfase_1G.md`
  - `subfase_1H.md`
  - `subfase_1I.md`
  - `subfase_1J.md`
  - `subfase_1K.md`
  - `subfase_1L.md`
  - `subfase_1N.md`
  - `subfase_1O.md`
  - `subfase_1P.md`
  - `subfase_1Q.md`
  - `subfase_1R.md`
  - `subfase_1S.md`
- documentación de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/orbslam3_msgs/`
  - `codex/contexto/paquetes/simulacion_dron/`
- ADRs relacionados con:
  - frames;
  - transformadas;
  - IDs;
  - bases de datos;
  - grafos;
  - optimización;
  - fusión;
  - scoring;
  - rollback;
  - replay.

---

## Diagnostico de partida

Tras 1A–1S, el sistema contiene muchas piezas nuevas:

```text
RawMapDatabase
GlobalPoseStore
FiducialAnchorManager
GlobalMapBuilder
LandmarkScoreManager
PoseGraphBuilder
OptimizationManager
LoopDetector
SubcloudLoopVerifier
LoopDecisionManager
FusionManager
FusedLandmarkManager
PostOptimizationKeyFrameQueue
```

Cada una usa IDs, poses, transformadas y estados. El riesgo principal es que haya incoherencias como:

```text
world_T_kf usado en una clase como Tcw y en otra como Twc;
candidate_T_query aplicado en dirección inversa;
correction_T_kf multiplicado por el lado equivocado;
un KF optimizado sobrescrito por pose raw;
un fused track publicando miembros raw duplicados;
un score modificado fuera de LandmarkScoreManager;
un rollback restaurando poses pero no scores;
un task_id no propagado al journal.
```

Esta subfase debe convertir esas reglas en contratos verificables.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Se pueden modificar o crear archivos de contratos, validación y debug:

- `orbslam3_multi/include/orbslam3_multi/types.hpp`
- `orbslam3_multi/include/orbslam3_multi/frame_conventions.hpp`
- `orbslam3_multi/include/orbslam3_multi/system_invariants.hpp`
- `orbslam3_multi/src/system_invariants.cpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp`
- `orbslam3_multi/include/orbslam3_multi/fused_landmark_manager.hpp`
- `orbslam3_multi/src/fused_landmark_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp`
- `orbslam3_multi/src/landmark_score_manager.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp`
- `orbslam3_multi/src/global_map_builder.cpp`
- `orbslam3_multi/CMakeLists.txt`

### `orbslam3_server`

Cambios mínimos para invocar checks de invariantes:

- `orbslam3_server/src/global_map_server.cpp`
- clases auxiliares del servidor nuevo
- launch/config si necesita activar `f1s_contract_checks_enabled`

### Documentación

- `codex/contexto/paquetes/orbslam3_multi/contracts.md`
- `codex/contexto/paquetes/orbslam3_multi/frame_conventions.md`
- `codex/contexto/paquetes/orbslam3_multi/invariants.md`
- documentación de clases afectadas
- historial de Fase 1 / subfase 1T

---

## Archivos prohibidos

No modificar:

- `ORB_SLAM3/`, salvo permiso explícito.
- `orbslam3_ros2` / wrapper, salvo bug crítico demostrado.
- `orbslam3_msgs`, salvo necesidad justificada.
- `global_map_server_antiguo.cpp`, salvo consulta.
- lógica algorítmica de BoW, RANSAC, fusión u optimización salvo checks no invasivos.
- `RawMapDatabase` para corregir poses.
- `GlobalPoseStore` para guardar datos raw.
- `build/`, `install/`, `log/`.

---

## Funciones, clases o nodos que hay que localizar

Localizar o crear:

- `GlobalKeyFrameId`
- `GlobalMapPointId`
- `SubmapId`
- `FusedLandmarkId`
- `OptimizationTaskId`
- `ArrivalId`
- `ScoreEventId`
- `RawMapDatabase`
- `GlobalPoseStore`
- `PoseGraphProblem`
- `FusedLandmarkManager`
- `LandmarkScoreManager`
- `GlobalMapBuilder`
- clase o función tipo `SystemInvariantChecker`

---

## Cambios requeridos

### 1. Definir contrato de IDs

Documentar y validar los identificadores globales:

```text
GlobalKeyFrameId:
    debe incluir drone_id, map_epoch y kf_id local o forma equivalente.

GlobalMapPointId:
    debe incluir drone_id, map_epoch y mappoint_id local o forma equivalente.

SubmapId:
    debe ser (drone_id, map_epoch).

FusedLandmarkId:
    único dentro de FusedLandmarkManager.

OptimizationTaskId:
    único por tarea.

ArrivalId:
    orden total de llegada de deltas/snapshots/replay.

ScoreEventId:
    único por evento de score.
```

Logs:

```text
[F1S-ID-CONTRACT]
[F1S-ID-CONSISTENCY-CHECK]
```

---

### 2. Definir contrato de frames y transformadas

Documentar y validar:

```text
world:
    frame global del servidor.

local/orb:
    frame local de ORB-SLAM3 por submapa.

world_T_local:
    transforma puntos del frame local del submapa a world.

local_T_kf o world_T_kf:
    declarar convención exacta.

Tcw/Twc:
    no usar sin conversión explícita y documentación.

correction_T_kf:
    transforma pose base anclada hacia pose corregida.

candidate_T_query:
    transforma query hacia candidate según RANSAC o convención documentada.
```

Debe existir una tabla en documentación:

```text
Nombre | Significado | Dirección | Clase que lo produce | Clase que lo consume
```

Logs:

```text
[F1S-FRAME-CONVENTION-CHECK]
[F1S-TRANSFORM-CONVENTION-CHECK]
```

---

### 3. Definir ownership de datos

Validar:

```text
RawMapDatabase:
    posee datos raw de ORB-SLAM3.
    no se corrige por optimización.

GlobalPoseStore:
    posee poses globales, anchors, correcciones y estados globales.

FusedLandmarkManager:
    posee tracks fused.
    no borra MapPoints raw.

LandmarkScoreManager:
    posee scores y score events.

GlobalMapBuilder:
    solo lee y publica.

PoseGraphProblem:
    temporal, no persistente.
```

Logs:

```text
[F1S-OWNERSHIP-CHECK]
```

---

### 4. Validar invariantes de RawMapDatabase

Invariantes:

```text
RawMapDatabase no cambia poses por optimización.
RawMapDatabase no borra MapPoints por fusión.
RawMapDatabase preserva llegada por arrival_id.
RawMapDatabase separa drone_id/map_epoch.
RawMapDatabase conserva datos suficientes para replay.
```

Logs:

```text
[F1S-INVARIANT-RAWDB]
```

---

### 5. Validar invariantes de GlobalPoseStore

Invariantes:

```text
solo GlobalPoseStore guarda poses globales corregidas;
KFs optimizados tienen estado explícito;
KFs propagados tienen estado explícito;
KFs rebasados tienen estado explícito;
correction_T_kf existe para KFs optimizados/propagados si procede;
full snapshot no pisa poses optimizadas;
rollback restaura poses y correcciones.
```

Logs:

```text
[F1S-INVARIANT-POSESTORE]
```

---

### 6. Validar invariantes de fusión y score

Invariantes:

```text
FusedLandmarkManager no borra MapPoints raw.
GlobalMapBuilder no publica miembros raw de tracks fused en nube principal.
LandmarkScoreManager es el único que modifica scores.
Score rollback revierte eventos de tareas rechazadas.
```

Logs:

```text
[F1S-INVARIANT-FUSION]
[F1S-INVARIANT-SCORE]
```

---

### 7. Validar invariantes de optimización

Invariantes:

```text
PoseGraphProblem es temporal.
OptimizationManager no decide ventana del grafo.
PoseGraphBuilder no aplica poses.
Apply solo modifica GlobalPoseStore.
Post-apply valida error real.
Rollback restaura estado.
Loop con error alto no fusiona antes de ACCEPT.
```

Logs:

```text
[F1S-INVARIANT-OPTIMIZATION]
```

---

### 8. Crear checker ejecutable

Crear método de comprobación:

```cpp
SystemInvariantReport CheckSystemInvariants(...);
```

o equivalente.

Debe poder ejecutarse:

- al arranque;
- tras insertar delta;
- tras full snapshot;
- tras anclaje;
- tras dry-run;
- tras apply;
- tras rollback;
- tras fusión;
- al final de replay.

Logs:

```text
[F1S-CONTRACT-CHECK]
[F1S-INVARIANT-PASS]
[F1S-INVARIANT-FAIL]
```

---

## Cambios prohibidos

- No modificar algoritmos para “hacer pasar” checks sin entender la causa.
- No desactivar invariantes críticas.
- No cambiar direcciones de transformadas sin actualizar toda la documentación.
- No añadir nuevos frames implícitos.
- No duplicar ownership de poses/scores/fused tracks.
- No usar GT para loops.
- No borrar legacy.

---

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron
```

---

## Pruebas Gazebo requeridas

### Prueba 1 — checks durante flujo normal

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia:

1. arrancar simulación;
2. insertar KFs;
3. anclar fiducial;
4. publicar nube;
5. ejecutar checks tras eventos clave.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

---

## Pruebas replay requeridas

### Prueba 2 — checks en replay con optimización/fusión

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Secuencia:

1. cargar dataset;
2. reproducir deltas;
3. ejecutar anclaje, loop/fusión u optimización si existen;
4. ejecutar invariantes al final.

---

## Patrones de reduccion de logs

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1S-|INVARIANT|CONTRACT|ERROR|FATAL|Segmentation fault|Killed
```

---

## Marcadores tecnicos obligatorios

```text
[F1S-CONTRACT-CHECK]
[F1S-ID-CONSISTENCY-CHECK]
[F1S-FRAME-CONVENTION-CHECK]
[F1S-OWNERSHIP-CHECK]
[F1S-INVARIANT-RAWDB]
[F1S-INVARIANT-POSESTORE]
[F1S-INVARIANT-FUSION]
[F1S-INVARIANT-SCORE]
[F1S-INVARIANT-OPTIMIZATION]
[F1S-INVARIANT-PASS]
```

No debe aparecer:

```text
[F1S-INVARIANT-FAIL]
```

salvo que la subfase se marque parcial/no conseguida con diagnóstico.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. compila;
2. existen contratos documentados de IDs, frames y ownership;
3. se ejecutan checks en live o replay;
4. no hay fallos de invariantes críticos;
5. las transformadas quedan documentadas con dirección;
6. la documentación queda actualizada;
7. no aparecen errores graves.

---

## Criterio de fallo o parcial

`NO CONSEGUIDA` si:

- no compila;
- hay invariantes críticas fallando sin resolver;
- la dirección de transformadas queda ambigua;
- no se documentan ownerships;
- se detecta mutación directa prohibida.

`PARCIAL` si:

- compila;
- los checks existen, pero alguno queda en warning documentado;
- falta una prueba live pero replay funciona.

---

## Documentacion a actualizar

Actualizar:

- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/paquetes/orbslam3_multi/contracts.md`
- `codex/contexto/paquetes/orbslam3_multi/frame_conventions.md`
- `codex/contexto/paquetes/orbslam3_multi/invariants.md`
- documentación de clases afectadas
- historial de Fase 1 / subfase 1T

No marcar como `realizado` si no se cumplen los criterios.
