# Prueba clave Fase 1C — Raw DB y replay

## Dataset generado

```text
codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record
```

Tamaño observado el 2026-07-06: aproximadamente `272M`.

Nota: el 2026-07-08, durante `1E`, este dataset fue borrado por petición del usuario para ahorrar espacio y sustituido por un `.record` versión 2 con observaciones fiduciales. La prueba histórica de `1C` queda documentada aquí, pero el archivo actual `rawdb_prueba_1.record` corresponde a `1E`.

El dataset se generó con `prueba_1` usando Gazebo y el launch oficial:

```bash
./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20 --timeout-sec 900
```

## Trayectoria de grabación

Archivo:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Escenario:

```text
prueba_1c_rawdb_record_simple
```

Secuencia:

- esperar arranque;
- mover `dron_1` al fiducial 2 con yaw `90` y altura `1.0`;
- mover `dron_2` encima de `dron_1` a altura `1.3`;
- mover `dron_1` a `[-6, -9, 1.0]`;
- mover `dron_2` encima;
- devolver `dron_1` al fiducial 2;
- mover `dron_2` encima;
- esperar post-escenario.

## Replay sin Gazebo

Archivo:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_4.yaml
```

Escenario:

```text
prueba_1c_rawdb_replay
```

Comando validado:

```bash
./codex/herramientas/run_simulation.sh --prueba 4 --launch "ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.05" --post-scenario-wait-sec 5 --startup-wait-sec 2 --timeout-sec 120 --max-gazebo-retries 0
```

## Evidencia

Grabación:

- `SIM-DONE prueba=1 success=true`;
- 6 goals con `success=true`;
- `284` `[F1C-RAWDB-DELTA-RX]`;
- `284` `[F1C-RAWDB-INSERT-DELTA]`;
- `3` `[F1C-RAWDB-NEW-SUBMAP]`;
- `[F1C-RAWDB-SAVE] reason=shutdown ... journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`.

Replay:

- `SIM-DONE prueba=4 success=true`;
- `[F1C-REPLAY-LOAD] ... success=true entries=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
- `284` `[F1C-REPLAY-DELTA]`;
- `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`.

## Patrones de reducción

Grabación:

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-RAWDB|F1C-REPLAY|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
```

Replay:

```text
SCENARIO-RUNNER|success|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed
```

## Riesgos operativos

El dataset raw ocupa bastante espacio. Tras la validación de 1C el disco quedó con unos `467M` libres. Antes de repetir pruebas largas conviene comprobar `df -h /home/chenfu/Gazebo` y, si hace falta, limpiar solo artefactos generados siguiendo `AGENTS.md`.
