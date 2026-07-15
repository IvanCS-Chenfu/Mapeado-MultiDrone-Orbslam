# Prueba clave — Fase 1F — `/global_sparse_cloud`

## Objetivo

Validar que `LandmarkScoreManager` y `GlobalMapBuilder` publican una primera nube sparse global en `world` usando solo submapas anclados, y que los anchors fiduciales aplican `body_T_camera` antes de transformar puntos al mundo.

## Fecha

2026-07-08

## Pruebas ejecutadas

### Prueba 1 — Gazebo live

```bash
./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20 --timeout-sec 900
```

Resultado:

```text
SIM-DONE prueba=1 success=true
SIM-EXIT-CODE 0
```

### Prueba 2 — replay lento validado en 1F

```bash
./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record rawdb_replay_period_sec:=1.0 fiducial_sim_enabled:=false pose_store_debug_enabled:=false global_map_publish_period_sec:=1.0" --post-scenario-wait-sec 20 --startup-wait-sec 2 --timeout-sec 540 --max-gazebo-retries 0
```

Resultado:

```text
SIM-DONE prueba=2 success=true
SIM-EXIT-CODE 0
F1C-REPLAY-DONE entries=318 journal=318 deltas=318 full=0 fiducial_observations=72 submaps=3 kfs=179 mps=22885
```

El replay de hotfix se ejecuto inicialmente con el path antiguo `rawdb_prueba_1.record`; al terminar se renombro ese dataset a `prueba_diff_anclaje.record` para conservarlo sin duplicar espacio. Ese archivo se borro despues al iniciar `1G` a peticion del usuario, por lo que el comando anterior queda como evidencia historica, no como comando vigente.

## Evidencia principal

```text
[F1F-SCORE-STATS] ... tracked_points=22885 score_min=0.238 score_mean=0.557 score_max=1.000
[F1F-BODY-CAMERA-CONFIG] source=launch use_optical=true body_T_camera_t=(0.100,0.030,0.030) rpy_deg=(0.000,-90.000,90.000)
[F1F-BODY-CAMERA-APPLY] ... body_t=(...) camera_t=(...)
[F1F-GLOBALMAP-BUILD] ... anchored_submaps=2 skipped_unanchored=1 raw_points=22394 candidate_points=22394 returned_points=22394
[F1F-GLOBALMAP-PUBLISH] ... topic=/global_sparse_cloud frame_id=world points_from_backend=22394 points_published=22394 min_score_to_publish=0.000 score_field=true drone_id_field=true map_epoch_field=true
```

## Observaciones

- La nube queda lista para inspección visual en RViz2.
- La nube se ancla usando `world_T_camera`, no `world_T_body` directo.
- El dataset diferencial `codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record` ya no esta disponible tras `1G`; para repetir una regresion visual de `body_T_camera` hay que usar el record vigente o generar uno nuevo.
- En `1F` no hay fusión real: `is_fused=false`.
- El submapa no anclado se salta correctamente.
- El cierre `gazebo ... exit code 255` en prueba live aparece después de `SIM-DONE success=true` y se considera ruido no bloqueante ya conocido.
