# Prueba clave Fase 1E — Fiducial record y replay

## Dataset generado

```text
codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record
```

El dataset anterior se borró antes de ejecutar `1E` para ahorrar espacio. El nuevo dataset se generó el 2026-07-08 y mide `398239640` bytes.

Contenido validado:

- `364` entradas de journal raw;
- `364` deltas;
- `3` submapas;
- `175` KFs;
- `21095` MPs;
- `60` observaciones fiduciales persistidas.

## Trayectoria live

Archivo:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Escenario:

```text
prueba_1e_fiducial_anchor_live_record
```

Secuencia:

- esperar arranque;
- mover `dron_1` al fiducial 2 en `(0, -9, 1)`;
- mover `dron_2` encima;
- mover `dron_1` a la izquierda;
- mover `dron_2` encima;
- devolver `dron_1` al fiducial 2;
- mover `dron_2` encima;
- esperar guardado.

Comando validado:

```bash
./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20 --timeout-sec 900
```

## Replay sin GT vivo

Archivo:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Escenario:

```text
prueba_1e_fiducial_anchor_replay
```

Comando validado:

```bash
./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.02 fiducial_sim_enabled:=false pose_store_debug_enabled:=false" --post-scenario-wait-sec 5 --startup-wait-sec 2 --timeout-sec 120 --max-gazebo-retries 0
```

## Evidencia

Grabación live:

- `SIM-DONE prueba=1 success=true`;
- `[F1E-FID-KF-ASSOC]` para KFs dentro del radio fiducial;
- `[F1E-FID-FIRST-ANCHOR]` para `2` submapas;
- `[F1E-FID-KF-HARD]`;
- `[F1E-FID-JOURNAL-SAVE]`;
- `[F1C-RAWDB-SAVE] reason=shutdown ... fiducial_observations=60 ...`.

Replay:

- `SIM-DONE prueba=2 success=true`;
- `[F1C-REPLAY-LOAD] ... fiducial_observations=60`;
- `60` eventos `[F1E-FID-REPLAY-OBS]`;
- `[F1E-FID-STATS] reason=REPLAY_RECORDED_FIDUCIAL observations=60 accepted=60 rejected=0 anchors_created=2 replay_observations=60 hard_fiducial_kfs=60`;
- `[F1C-REPLAY-DONE] entries=364 journal=364 deltas=364 full=0 fiducial_observations=60 submaps=3 kfs=175 mps=21095`.

## Patrones de reducción

Grabación:

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-|F1D-|F1E-|FID|POSESTORE|RAWDB|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
```

Replay:

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1E-FID-REPLAY|F1E-FID|F1D-POSESTORE|RAWDB|ERROR|FATAL|Segmentation fault|Killed
```

## Riesgos operativos

El record nuevo ocupa cerca de `380M`. Tras la validación de `1E` quedaron aproximadamente `397M` libres en `/home/chenfu/Gazebo`, así que antes de repetir Gazebo conviene liberar espacio de artefactos generados siguiendo `AGENTS.md`.
