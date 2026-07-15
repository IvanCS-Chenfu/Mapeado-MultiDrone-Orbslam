# `FiducialAnchorManager`

## Rol

`FiducialAnchorManager` es la capa de backend creada en `1E` dentro de `orbslam3_multi` y ampliada en `1H` para medir revisits fiduciales.

Su responsabilidad es convertir una observación fiducial ya asociada a un `KeyFrame` en un anclaje de submapa:

```text
FiducialObservation + RawMapDatabase + GlobalPoseStore
  -> world_T_camera
  -> world_T_local
  -> anchor de submapa
  -> poses world de KFs existentes
  -> KF marcado como hard fiducial
```

No lee topics, no conoce Gazebo, no usa ROS y no decide si el dron está dentro de un fiducial. Esa asociación la hace el servidor activo usando GT solo como fiducial simulado/debug.

Desde el hotfix de `1F`, la observación recibida desde servidor guarda la pose world del cuerpo del dron (`world_T_body_fiducial`). El manager pide a `GlobalPoseStore` aplicar `body_T_camera` para obtener `world_T_camera_fiducial` antes de calcular el anchor.

Desde `1H`, si el submapa ya esta anclado, una nueva observacion fiducial no recalcula el anchor. El manager obtiene la pose global estimada del KF desde `GlobalPoseStore`, la compara contra `world_T_camera_fiducial` y emite una decision:

```text
error bajo -> revisit OK
error alto -> FiducialOptimizationTask pendiente
```

La tarea no ejecuta optimizacion. Solo conserva evidencia para `PoseGraphBuilder`/`OptimizationManager` en subfases posteriores.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp
orbslam3_multi/src/fiducial_anchor_manager.cpp
orbslam3_multi/include/orbslam3_multi/fiducial_optimization_task.hpp
```

## Tipos principales

- `FiducialObservation`: observación absoluta ya asociada a `arrival_id`, `(drone_id, map_epoch)`, `local_keyframe_id`, `fiducial_id`, pose `world_T_body_fiducial`, timestamps y distancia al fiducial.
- `FiducialAnchorResult`: resultado de aceptar/rechazar la observación, si creó primer anchor, si el submapa ya estaba anclado, la pose `world_T_camera_fiducial` derivada, si el KF quedó marcado hard fiducial y, desde `1H`, datos de revisit, error, umbrales y tarea.
- `FiducialAnchorStats`: contadores de observaciones, aceptadas, rechazadas, anchors creados, observaciones replay, KFs hard fiducial y, desde `1H`, revisits OK, residuales altos, tareas creadas/deduplicadas y revisits sin pose.
- `FiducialRevisitConfig`: umbrales de `error_threshold_m`, `yaw_threshold_rad` y `rot_threshold_rad`.
- `FiducialOptimizationTask`: tarea ligera con KF, submapa, fiducial, poses estimada/target y residual medido.

## Método principal

```cpp
FiducialAnchorResult RegisterFiducialObservation(
  const FiducialObservation& observation,
  const RawMapDatabase& raw_db,
  GlobalPoseStore& pose_store);
```

Comportamiento:

- consulta el KF raw en `RawMapDatabase`;
- convierte la pose local raw del KF a `Sophus::SE3d`;
- calcula `world_T_camera_fiducial = world_T_body_fiducial * body_T_camera`;
- calcula `world_T_local = world_T_camera_fiducial * inverse(local_T_kf)`;
- si el submapa no está anclado, llama a `GlobalPoseStore::AnchorSubmap`;
- si el submapa ya está anclado, registra el KF nuevo si todavía no tenía pose world;
- en revisit, compara `estimated_world_T_kf` contra `world_T_camera_fiducial`;
- si el residual esta bajo umbral, confirma `revisit_ok`;
- si el residual esta alto, crea o deduplica una `FiducialOptimizationTask`;
- marca el KF como hard fiducial solo en primer anchor o revisit OK. Un residual alto no bloquea el KF como hard porque debe poder moverse en una optimizacion futura.

## Logs relacionados

Los emite `orbslam3_server/src/global_map_server.cpp` alrededor de la llamada al manager:

```text
[F1E-FID-OBS]
[F1E-FID-WORLD-T-LOCAL]
[F1E-FID-FIRST-ANCHOR]
[F1E-FID-KF-HARD]
[F1E-FID-STATS]
[F1F-BODY-CAMERA-APPLY]
[F1H-FID-REVISIT]
[F1H-FID-POSE-ERROR]
[F1H-FID-OK]
[F1H-FID-TASK-CREATED]
[F1H-FID-TASK-SKIP-DUPLICATE]
[F1H-FID-TASK-STATS]
```

## Validación 1E

El 2026-07-08 se validó con:

- build de `orbslam3_multi` y `orbslam3_server` con `BUILD-EXIT-CODE 0`;
- simulación Gazebo `prueba_1` con `SIM-DONE prueba=1 success=true`;
- nuevo `.record` versión 2 con `60` observaciones fiduciales;
- replay `prueba_2` sin GT vivo con `SIM-DONE prueba=2 success=true`;
- `[F1E-FID-STATS] reason=REPLAY_RECORDED_FIDUCIAL observations=60 accepted=60 rejected=0 anchors_created=2 replay_observations=60 hard_fiducial_kfs=60`.

## Validación hotfix 1F

El 2026-07-08 se validó que las observaciones fiduciales grabadas/reproducidas usan la pose del cuerpo como entrada y se convierten a cámara dentro del backend:

- build de `orbslam3_multi` y `orbslam3_server` con `BUILD-EXIT-CODE 0`;
- replay lento del dataset que entonces se llamaba `prueba_diff_anclaje.record` con `SIM-DONE prueba=2 success=true`; ese archivo se borro posteriormente al iniciar `1G` a peticion del usuario;
- logs `[F1F-BODY-CAMERA-CONFIG]` y `[F1F-BODY-CAMERA-APPLY]`;
- publicación final de `/global_sparse_cloud` con `22394` puntos.

## Validación 1H

El 2026-07-09 se validó:

- build de `orbslam3_multi` y `orbslam3_server` con `BUILD-EXIT-CODE 0`;
- live `prueba_1` con `SIM-DONE prueba=1 success=true`;
- replay `prueba_2` con `SIM-DONE prueba=2 success=true`;
- revisits live con `source=SIM_GT_TEMPORAL_ASSOCIATION`;
- revisits replay con `source=REPLAY_RECORDED_FIDUCIAL`;
- `[F1H-FID-POSE-ERROR]` muestra error traslacional/rotacional/yaw y umbrales;
- el dataset validado quedó bajo umbral y produjo `[F1H-FID-OK]`, no tareas;
- `[F1H-FID-TASK-STATS]` muestra `high_error=0` y `pending=0`;
- no se ejecutó optimización real.

## Restricciones

- No usar GT dentro de esta clase.
- No tratar fiduciales como loops.
- No fusionar landmarks ni optimizar poses en `1H`.
- No modificar poses locales raw.
- No mover KFs hard fiducial en fases posteriores salvo subfase explícita.
- No calcular `world_T_local` directamente desde `world_T_body`; siempre debe pasar por `GlobalPoseStore::TransformBodyPoseToCameraPose`.
- No sobrescribir `world_T_local` cuando una observacion fiducial pertenece a un submapa ya anclado; medir residual y crear tarea si toca.
