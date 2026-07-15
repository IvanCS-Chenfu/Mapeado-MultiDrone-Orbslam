# `GlobalPoseStore`

## Rol

`GlobalPoseStore` es la capa ligera creada en `1D` dentro de `orbslam3_multi`.

Su responsabilidad es responder:

```text
¿Que pose global tiene este KeyFrame segun el estado global actual?
```

No guarda datos ORB grandes. No guarda descriptores, BoW, keypoints, observaciones ni MapPoints completos. Cuando necesita una pose local o un listado de KFs consulta `RawMapDatabase`.

Desde `1E`, también conserva qué KeyFrames fueron marcados como fiduciales reales/hard fiducials por `FiducialAnchorManager`. Esa marca sirve para que optimizaciones futuras no traten esos KFs como variables normales.

Desde el hotfix de `1F`, también conserva la extrínseca fija `body_T_camera` usada para convertir observaciones fiduciales simuladas desde la pose world del cuerpo del dron a la pose world de la cámara/KF:

```text
world_T_camera = world_T_body * body_T_camera
```

Esto evita anclar submapas usando directamente el frame del cuerpo como si fuera el frame de cámara.

Desde `1G`, también puede reconciliar cambios raw procedentes de full snapshots. Si ORB-SLAM3 cambia la pose local de un KF, `GlobalPoseStore` recalcula las poses globales derivadas de anchor y conserva las poses optimizadas del servidor, actualizando la corrección interna que las relaciona con la nueva pose raw.

Desde `1J`, `GlobalPoseStore` actua como entrada de solo lectura para `OptimizationManager::RunDryRun`. El dry-run copia poses a memoria temporal y el servidor comprueba por estadisticas que `GlobalPoseStore` no cambia durante la prueba.

Desde `1K`, `GlobalPoseStore` es la unica capa persistente donde el servidor aplica poses optimizadas o propagadas. `RawMapDatabase` conserva la pose local raw; `GlobalPoseStore` conserva la decision global del servidor.

La protección transaccional de apply se desarrolló durante `1L`, pero pertenece
conceptualmente a `1K`: `GlobalPoseStore` guarda el estado global aplicado por
el servidor y puede restaurar backups si una validación post-apply rechaza una
optimización. `1L` solo diagnostica/valida ese estado.

Actualización `1K` posterior: los KFs creados después del último KF aplicado
heredan la corrección del extremo optimizado y quedan registrados también como
poses controladas por servidor. Así un full snapshot posterior no los rebasa al
anchor base del submapa.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp
orbslam3_multi/src/global_pose_store.cpp
```

## Estado inicial

`GlobalPoseStore` empieza vacío aunque `RawMapDatabase` ya tenga KFs y MPs.

Esto es intencionado: sin un anchor de submapa no existe una transformación válida:

```text
world_T_local
```

Por tanto no debe inventar:

```text
world_T_kf = world_T_local * local_T_kf
```

En `1D` solo se pobla mediante un modo debug sintético de replay. El primer anclaje real se espera en `1E` mediante `FiducialAnchorManager`.

## Responsabilidades

- Guardar anchors de submapa por `(drone_id, map_epoch)`.
- Crear poses `world_T_kf` para KFs existentes al anclar un submapa.
- Consultar si un submapa tiene anchor.
- Consultar si un KF tiene pose world.
- Registrar KFs nuevos de submapas ya anclados.
- Registrar poses optimizadas ya calculadas por una prueba o fase futura.
- Registrar poses propagadas tras optimización del servidor.
- Calcular y guardar una corrección simple heredable para KFs futuros del mismo submapa.
- Desde `1K`, tras un apply validable, guardar explícitamente la corrección
  heredable para KFs futuros desde el KF aplicado más nuevo. En candidatos
  parciales, esa entrada vive dentro del backup/rollback: si la validación
  falla se restaura; si se acepta, los KFs posteriores heredan
  `SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT`.
- Si un KF posterior ya está en el full snapshot o cambia su pose raw tras el
  apply, `ReconcileAfterRawIngestResult` reaplica la última corrección heredable
  del submapa y lo mantiene como server-controlled, no como simple pose derivada
  del anchor.
- Marcar y consultar KFs hard fiducial.
- Guardar y aplicar `body_T_camera` para convertir poses de cuerpo en poses de cámara antes de derivar anchors fiduciales.
- Reconciliar cambios de pose local raw detectados en full snapshots sin ceder la autoridad sobre poses globales optimizadas.
- Crear backups de apply para `1K` y restaurarlos/confirmarlos cuando `1L`
  decide rollback o commit.
- Exponer estadísticas ligeras para logs.

## Métodos principales

| Método | Responsabilidad |
|---|---|
| `AnchorSubmap` | Recibe `world_T_local`, consulta KFs en `RawMapDatabase` y crea poses `world_T_kf` para los KFs existentes. |
| `HasSubmapAnchor` | Indica si un submapa ya tiene anchor. |
| `HasWorldPose` | Indica si un KF ya tiene pose world registrada. |
| `GetWorldPose` | Devuelve la pose world existente; no inventa poses para KFs sin anchor. |
| `GetSubmapWorldTransform` | Devuelve `world_T_local` de un submapa anclado para consumidores de solo lectura como `GlobalMapBuilder`. |
| `RegisterNewKeyFrameIfAnchored` | Registra pose world para un KF nuevo si su submapa ya está anclado. |
| `SetOptimizedKeyFramePose` | Registra una pose optimizada ya calculada y su corrección por KF. |
| `SetPropagatedKeyFramePose` | Registra una pose propagada tras optimización y su corrección por KF. |
| `GetKeyFrameServerCorrection` | Devuelve la corrección del servidor asociada a un KF, usada por `GlobalMapBuilder`. |
| `GetSubmapLastServerCorrection` | Devuelve la última corrección heredable de un submapa para KFs futuros y para que consumidores eviten fallback rígido en submapas corregidos. |
| `SetSubmapLastServerCorrectionFromKeyFrame` | Marca explícitamente qué KF optimizado/propagado alimenta la corrección heredable para KFs futuros. |
| `CreateApplyBackup` | Guarda el estado previo de KFs afectados y correcciones de submapa antes del apply de `1K`. |
| `RestoreApplyBackup` | Restaura el backup de un `task_id` tras `REJECT_ROLLBACK`. |
| `ConfirmApply` | Descarta el backup de un `task_id` tras `ACCEPT` o `PARTIAL_KEEP_FOR_NEXT_PASS`. |
| `MarkHardFiducialKeyFrame` | Marca un KF como fiducial real/hard fiducial tras una observación aceptada. |
| `IsHardFiducialKeyFrame` | Consulta si un KF está protegido como hard fiducial. |
| `ConfigureBodyCameraTransform` | Configura `body_T_camera` desde parámetros del servidor/launch. |
| `GetBodyCameraTransform` | Devuelve la matriz `body_T_camera` vigente para debug y logs. |
| `TransformBodyPoseToCameraPose` | Calcula `world_T_camera = world_T_body * body_T_camera`. |
| `GetBodyCameraTransformConfig` | Devuelve la configuración de extrínseca activa. |
| `ReconcileAfterRawIngestResult` | Procesa cambios raw detectados por `RawMapDatabase`: recalcula KFs derivados de anchor y conserva KFs optimizados recalculando su corrección. |
| `GetPoseStoreStats` | Devuelve contadores de anchors, poses world, KFs optimizados y correcciones. |
| `GetSubmapPoseStats` | Resume estado global de un submapa concreto. |

## Extrínseca `body_T_camera`

La configuración activa por defecto replica los valores del launch legacy:

```text
body_T_camera_x=0.10
body_T_camera_y=0.03
body_T_camera_z=0.03
body_T_camera_roll_deg=0.0
body_T_camera_pitch_deg=-90.0
body_T_camera_yaw_deg=90.0
use_camera_optical_frame_convention=true
```

Con `use_camera_optical_frame_convention=true`, la rotación usa la convención óptica que ya estaba en `global_pose_corrector` legacy. Los valores RPY quedan documentados y parametrizados, pero la convención óptica es la que se aplica en la validación del hotfix `1F`.

## Política temporal de 1D

La política de 1D para KFs futuros tras optimización es deliberadamente simple:

```text
world_T_kf = correction_T_latest * (world_T_local * local_T_kf)
```

La corrección heredada solo se aplica dentro del mismo submapa.

En subfases posteriores podrá sustituirse por una estrategia más robusta:

- parent de ORB-SLAM3;
- KF covisible corregido más fuerte;
- nearest optimized KF;
- interpolación entre correcciones;
- deformation graph.

## Logs relacionados

Los logs los emite `orbslam3_server/src/global_map_server.cpp` al activar el modo debug de `1D`:

```text
[F1D-POSESTORE-INIT]
[F1D-POSESTORE-ANCHOR-REQUEST]
[F1D-POSESTORE-ANCHOR-SET]
[F1D-POSESTORE-ANCHOR-KF-POSE]
[F1D-POSESTORE-ANCHOR-SUMMARY]
[F1D-POSESTORE-OPT-POSE-SET]
[F1D-POSESTORE-CORRECTION-SET]
[F1D-POSESTORE-CORRECTION-STATS]
[F1D-POSESTORE-NEW-KF-POSE-SET]
[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]
[F1D-POSESTORE-STATS]
[F1E-FID-KF-HARD]
[F1E-FID-STATS]
[F1F-BODY-CAMERA-CONFIG]
[F1F-BODY-CAMERA-APPLY]
[F1G-POSESTORE-REBASE-ANCHOR]
[F1G-POSESTORE-KEEP-OPTIMIZED]
[F1G-SNAPSHOT-AFFECTS-OPTIMIZED-KFS]
[F1G-POSESTORE-RECONCILE-SUMMARY]
[F1K-POSESTORE-OPTIMIZED-POSE-SET]
[F1K-POSESTORE-PROPAGATED-POSE-SET]
[F1K-POSESTORE-CORRECTION-SET]
[F1K-POSESTORE-LAST-CORRECTION-SET]
[F1K-POSESTORE-NEW-KF-INHERIT-CORRECTION]
[F1K-POSESTORE-KEEP-SERVER-OPTIMIZED]
[F1K-POSESTORE-KEEP-FUTURE-INHERIT]
[F1K-POSESTORE-RECOMPUTE-CORRECTION-AFTER-RAW-CHANGE]
[F1L-POSESTORE-BACKUP-CREATED]
[F1L-POSESTORE-COMMIT-CONFIRMED]
[F1L-POSESTORE-ROLLBACK]
```

## Validación 1D

El 2026-07-08 se validó con replay sin Gazebo de:

```text
codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record
```

Evidencia principal:

- `BUILD-EXIT-CODE 0` en `orbslam3_multi`;
- `BUILD-EXIT-CODE 0` en `orbslam3_server`;
- `SIM-DONE prueba=1 success=true`;
- `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
- `[F1D-POSESTORE-INIT] anchors=0 world_poses=0 optimized_kfs=0 corrections=0`;
- `[F1D-POSESTORE-ANCHOR-SUMMARY] ... anchored_kfs=1`;
- `[F1D-POSESTORE-OPT-POSE-SET]`;
- `[F1D-POSESTORE-CORRECTION-SET]`;
- `4` eventos `[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]`;
- sin `ERROR`, `FATAL`, `Segmentation fault`, `Killed`, `std::bad_alloc` ni `No space left` durante la prueba.

## Validación 1E

El 2026-07-08 se validó el uso real con fiducial simulado:

- Gazebo live `prueba_1` generó observaciones fiduciales asociadas a KFs;
- replay `prueba_2` cargó `fiducial_observations=60` desde `.record` versión 2;
- `FiducialAnchorManager` aceptó `60` observaciones y creó `2` anchors;
- `GlobalPoseStore` terminó con `hard_fiducial_kfs=60` en `[F1E-FID-STATS]`;
- no se usó el modo debug `pose_store_debug_*` para validar `1E`.

## Validación hotfix 1F `body_T_camera`

El 2026-07-08 se corrigió la ruta de anclaje usada por `/global_sparse_cloud`:

- el `.record` conserva la pose world del cuerpo del dron;
- `FiducialAnchorManager` pide a `GlobalPoseStore` transformar esa pose a cámara;
- `world_T_local` se calcula desde `world_T_camera`, no desde `world_T_body`;
- el replay lento publicó `22394` puntos en `/global_sparse_cloud` tras aplicar la extrínseca;
- el dataset diferencial se conservó temporalmente como `codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record`; al iniciar `1G` se borró a petición del usuario para liberar espacio y sustituirlo por un record mejor con full snapshots.

## Validación 1G

El 2026-07-08 se validó reconciliación tras full snapshots:

- live `prueba_1` terminó con `SIM-DONE prueba=1 success=true`;
- `8` eventos `[F1G-POSESTORE-RECONCILE-SUMMARY]`;
- `48` eventos `[F1G-POSESTORE-REBASE-ANCHOR]`, demostrando recalculo de poses derivadas de anchor;
- `1` evento `[F1G-POSESTORE-KEEP-OPTIMIZED]`, demostrando que una pose optimizada debug se conserva ante cambio raw;
- replay `prueba_2` terminó con `SIM-DONE prueba=2 success=true`;
- replay reprodujo `12` full snapshots y volvió a emitir `12` resúmenes de reconciliación;
- no hubo `failed` en los resúmenes relevantes de reconciliación observados.

## Restricciones

- No modifica `RawMapDatabase`.
- No usa ground truth.
- No detecta fiduciales.
- No ejecuta optimización real.
- No publica nube global.
- No convierte anchors debug de `1D` en estado real de mapa.
- En `1E`, las marcas hard fiducial sí representan observaciones fiduciales reales simuladas; no deben moverse en optimizaciones futuras salvo fase explícita.
- En `1F`, `GetSubmapWorldTransform` permite publicar puntos en `world`, pero no autoriza a modificar anchors ni poses desde `GlobalMapBuilder`.
- `body_T_camera` solo corrige la relación cuerpo/cámara de observaciones fiduciales; no debe usarse como sustituto de optimización, fusión ni corrección de deriva entre submapas.
- En `1G`, un full snapshot no es una optimización: si cambia `local_T_kf`, los KFs anclados se recalculan y los KFs optimizados mantienen `optimized_world_T_kf`.
- En `1J`, `OptimizationManager` no debe llamar a `SetOptimizedKeyFramePose`; esa frontera queda para `1K`.
- En `1K`, `SetOptimizedKeyFramePose` y `SetPropagatedKeyFramePose` solo representan autoridad del servidor. No deben escribir en `RawMapDatabase`.
- En `1K`, un backup debe existir antes de aplicar un candidato que vaya a validarse post-apply.
- En `1K`, rollback restaura solo estado de `GlobalPoseStore`; no debe reescribir poses raw ni entries del journal.

## Validación 1K

El 2026-07-10 se validó:

- `prueba_3.log`: `17` `[F1K-POSESTORE-OPTIMIZED-POSE-SET]` y `40` `[F1K-POSESTORE-PROPAGATED-POSE-SET]`;
- `prueba_2.log`: `23` `[F1K-POSESTORE-OPTIMIZED-POSE-SET]` y `54` `[F1K-POSESTORE-PROPAGATED-POSE-SET]`;
- `prueba_3.log`: `309` `[F1K-POSESTORE-NEW-KF-INHERIT-CORRECTION]`;
- todos los applies terminaron con `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`;
- no se observaron hard fiducials movidos.

## Validación 1L

El 2026-07-10 se validó backup/commit/rollback:

- `prueba_1`: `8` backups y validaciones, `7` commits `ACCEPT` y `1` commit `PARTIAL_KEEP_FOR_NEXT_PASS`;
- `prueba_2`: `8` backups y validaciones, `4` commits `ACCEPT`, `1` commit `PARTIAL_KEEP_FOR_NEXT_PASS` y `3` rollbacks;
- `prueba_3`: `4` backups y validaciones, `3` commits `ACCEPT` y `1` commit `PARTIAL_KEEP_FOR_NEXT_PASS`;
- el rollback forzado de `prueba_2` mostro:
  ```text
  [F1L-POSESTORE-ROLLBACK] ok=true restored_world=80 removed_world=0 restored_optimized=0 removed_optimized=80 restored_submap_corrections=0 removed_submap_corrections=1
  ```
- no aparecieron `ROLLBACK_FAILED`, `RAWDB-POSE-OVERWRITE-BY-OPT` ni hard fiducials aceptados como movidos.
