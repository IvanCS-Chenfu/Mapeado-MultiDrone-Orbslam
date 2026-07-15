# 03 — Arquitectura actual y objetivo de Fase 1

## Resumen

El sistema construye un mapa sparse global multi-dron a partir de mapas locales ORB-SLAM3 ejecutados en cada dron.

La arquitectura actual ya congeló el servidor monolítico heredado en `orbslam3_server/src/global_map_server_antiguo.cpp`.

Desde `1B`, `orbslam3_server/src/global_map_server.cpp` es un servidor mínimo de recepción `OrbMap` con logs `[F1B-*]`. Desde `1C`, ese servidor inserta los deltas en `orbslam3_multi::RawMapDatabase`, guarda datasets raw y puede reproducirlos sin Gazebo. La planificación activa sigue migrando la lógica algorítmica hacia clases de `orbslam3_multi`, manteniendo `orbslam3_server` como adaptador ROS 2.

## Flujo de alto nivel

```text
Gazebo
  -> drones simulados
  -> cámaras estéreo
  -> wrapper ORB-SLAM3 por dron
  -> OrbMap delta / full snapshot / pose_local
  -> orbslam3_server minimo F1B/F1C
  -> logs de recepcion OrbMap
  -> RawMapDatabase en orbslam3_multi
  -> dataset raw / replay
  -> mapa sparse global fused + correcciones + debug en subfases posteriores
```

## `orbslam3_server`

En el estado técnico validado de `1C`, `orbslam3_server` actua como adaptador ROS 2 minimo:

- declara parametros basicos;
- se suscribe a `orbslam/orb_map_delta` por dron;
- loggea `drone_id`, `map_epoch`, `map_sequence`, KFs y MPs;
- asigna `arrival_id` creciente a cada `OrbMap` recibido;
- inserta deltas en `RawMapDatabase`;
- guarda `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`;
- puede cargar ese dataset y reproducir el journal con un timer;
- no publica mapa global ni correcciones.

En fases posteriores debe crecer de forma controlada para:

- declarar parámetros;
- suscribirse a `orbslam/orb_map_delta`;
- pedir `orbslam/get_full_map`;
- recibir `orbslam/pose_local`;
- leer GT solo para fiducial simulado/debug;
- convertir mensajes ROS a estructuras internas;
- llamar a `orbslam3_multi`;
- publicar nube global, `MapCorrection`, `CorrectedKeyFrameArray`, estado fiducial y topics debug.

No debe volver a concentrar de forma permanente:

- base de datos raw;
- BoW pesado;
- subnubes/RANSAC;
- decisión de loops;
- fusión y score;
- construcción de grafos;
- optimización;
- rollback.

## `orbslam3_multi`

Debe ser el backend algorítmico reutilizable, sin dependencia conceptual de ROS topics.

Componentes objetivo:

| Componente | Responsabilidad |
|---|---|
| `RawMapDatabase` | Guardar datos ORB-SLAM3 crudos, deltas, snapshots y replay. |
| `GlobalPoseStore` | Guardar anchors, poses globales, poses optimizadas, propagadas y correcciones heredables. |
| `FiducialAnchorManager` | Anclar submapas en primera visita y crear tareas por error en visitas posteriores. |
| `LoopDetector` | Generar candidatos BoW amplios sin confirmar loops. |
| `SubcloudLoopVerifier` | Verificar candidatos con subnubes, matching ORB, reducción espacial y RANSAC. |
| `LoopDecisionManager` | Decidir `REJECT`, `HOLD`, fusión u optimización por loop. |
| `FusionManager` / `FusedLandmarkManager` | Crear y actualizar tracks fused sin borrar raw. |
| `LandmarkScoreManager` | Centralizar score raw y fused mediante eventos semánticos. |
| `PoseGraphBuilder` | Construir grafos temporales para fiduciales y loops. |
| `OptimizationManager` | Dry-run, apply útil, validación, rollback e integración con `GlobalPoseStore`. |
| `GlobalMapBuilder` | Construir nube sparse global publicable desde poses globales y fused tracks. |

## Separación de datos

### Datos raw

`RawMapDatabase` conserva:

- submapas por `(drone_id, map_epoch)`;
- KeyFrames;
- MapPoints;
- poses locales;
- BoW/FeatureVector;
- descriptores;
- observaciones;
- covisibilidad;
- spanning tree;
- loop edges locales;
- `arrival_id`;
- deltas/full snapshots para replay.

Regla: no se modifica por optimización ni por fusión.

### Estado global

`GlobalPoseStore` conserva:

- `world_T_local`;
- anchors;
- estado de submapa;
- KFs realmente fiduciales/hard-fixed;
- poses globales optimizadas;
- poses propagadas;
- KFs rebasados;
- correcciones heredables.

Regla: si existe pose optimizada para un KF, esa pose prevalece frente a full snapshots posteriores. Si no existe, se deriva desde `world_T_local * local_T_kf`.

### Datos publicables

`GlobalMapBuilder` usa:

```text
RawMapDatabase + GlobalPoseStore + FusedLandmarkManager + LandmarkScoreManager
```

Debe evitar publicar duplicados raw cuando un punto pertenece a un fused track publicable.

## Fiduciales

El servidor detecta fiducial simulado con GT y entrega la observación al backend. El backend decide:

- primera visita: ancla submapa;
- visita posterior: mide error absoluto;
- error alto: crea `FiducialOptimizationTask`.

El fiducial no es un loop.

## Loops

El flujo objetivo es:

```text
LoopDetector
  -> candidatos BoW
SubcloudLoopVerifier
  -> query_subcloud / candidate_subcloud / matching / RANSAC
LoopDecisionManager
  -> reject / hold / fusion / loop optimization task
```

BoW no confirma loops. La confirmación exige evidencia geométrica.

## Optimización

`PoseGraphBuilder` construye grafos temporales. `OptimizationManager` ejecuta dry-run y solo aplica si es útil. El apply escribe en `GlobalPoseStore`, nunca en `RawMapDatabase`.

Después del apply se recalcula el error real. Si falla, rollback restaura poses, correcciones, scores y fused tracks afectados.

## Simulación

El launch oficial es:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

`run_simulation.sh` usa YAMLs:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_X.yaml
```

Los logs completos se guardan como `prueba_X.log` y los reducidos como `prueba_X.reduced.log`.

En `1C`, la simulacion validada usa `tray_prueba_1.yaml` para generar deltas reales y `tray_prueba_4.yaml` para replay sin Gazebo. No se espera nube global ni RViz2 util.

## Relación con fases futuras

Fase 1 debe producir:

- mapa sparse global fused;
- submapas coherentes;
- anchors;
- poses corregidas;
- corrected keyframes;
- landmarks globales con score;
- observabilidad suficiente para depurar.

Esto alimenta Fase 2, Fase 3 y Fase 5.
