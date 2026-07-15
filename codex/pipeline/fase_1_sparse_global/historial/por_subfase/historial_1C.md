# Historial 1C

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-06 12:46 — Subfase 1C

- objetivo intentado: crear `RawMapDatabase` en `orbslam3_multi`, conectar el servidor mínimo, asignar `arrival_id` a cada `OrbMap`, guardar estado raw/journal y validar replay sin Gazebo.
- cambios realizados:
  - `orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp`: tipos raw por `(drone_id, map_epoch)`, IDs de KF/MP y estadísticas.
  - `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`: interfaz de `RawMapDatabase`.
  - `orbslam3_multi/src/raw_map_database.cpp`: inserción de deltas/snapshots, journal ordenado, `SaveToPath` y `LoadFromPath`.
  - `orbslam3_multi/CMakeLists.txt`: añade `src/raw_map_database.cpp`.
  - `orbslam3_server/src/global_map_server.cpp`: inserta deltas en `RawMapDatabase`, emite logs `[F1C-*]`, guarda rawdb y reproduce journal por timer.
  - `orbslam3_server/CMakeLists.txt`: enlaza `global_map_server` con `orbslam3_multi`.
  - `orbslam3_server/launch/global_orb_map_server.launch.py`: añade parámetros `rawdb_record_enabled`, `rawdb_record_path`, `rawdb_replay_enabled`, `rawdb_replay_path` y `rawdb_replay_period_sec`.
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`: escenario `prueba_1c_rawdb_record_simple`.
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_4.yaml`: escenario `prueba_1c_rawdb_replay` con espera de 20 s.
- comandos de build:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh dron_individual
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ./codex/herramientas/build_selected_packages.sh orbslam3_msgs
  ```
- resultado de build: `OK`.
  - todos los builds finales terminaron con `BUILD-EXIT-CODE 0`;
  - se compiló en llamadas separadas para no bloquear el ordenador con paquetes pesados;
  - hubo un fallo intermedio de enlace en `orbslam3_server` por una librería `orbslam3_multi` incoherente tras la interrupción/apagado;
  - se ejecutó `reduce_build_log.sh`;
  - se limpió solo `build/install/orbslam3_multi` y se reconstruyó `orbslam3_multi`;
  - warning no bloqueante preexistente en `MultiDroneSystem.cpp`.
- pruebas Gazebo ejecutadas:
  - `prueba_1`: `OK`, grabación con Gazebo.
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20 --timeout-sec 900
    ```
  - `prueba_4`: `OK`, replay sin Gazebo usando solo `orbslam3_server`.
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 4 --launch "ros2 launch orbslam3_server global_orb_map_server.launch.py n_drones:=2 namespace_base:=dron use_sim_time:=false rawdb_record_enabled:=false rawdb_replay_enabled:=true rawdb_replay_path:=src/codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record rawdb_replay_period_sec:=0.05" --post-scenario-wait-sec 5 --startup-wait-sec 2 --timeout-sec 120 --max-gazebo-retries 0
    ```
- patrones de reducción usados:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-RAWDB|F1C-REPLAY|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 4:
    ```text
    SCENARIO-RUNNER|success|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed
    ```
- evidencia positiva:
  - `prueba_1`: `SIM-DONE prueba=1 success=true`;
  - `prueba_1`: 6 goals de `scenario_runner_node`, todos con `success=true`;
  - `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` generado con tamaño aproximado `272M`;
  - `prueba_1`: `284` `[F1C-RAWDB-DELTA-RX]`;
  - `prueba_1`: `284` `[F1C-RAWDB-INSERT-DELTA]`;
  - `prueba_1`: `3` `[F1C-RAWDB-NEW-SUBMAP]`;
  - `prueba_1`: `[F1C-RAWDB-SAVE] reason=shutdown ... journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
  - `prueba_4`: `SIM-DONE prueba=4 success=true`;
  - `prueba_4`: `[F1C-REPLAY-LOAD] ... success=true entries=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
  - `prueba_4`: `284` `[F1C-REPLAY-DELTA]`;
  - `prueba_4`: `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`.
- evidencia negativa o ausente:
  - no aparece `[F1C-RAWDB-INSERT-FULL]` porque la prueba validada solo ejercita deltas; los métodos de snapshot quedan implementados pero no ejercitados en esta ejecución;
  - no hay nube global, fiduciales, loops, fused landmarks ni optimización; queda fuera de `1C`;
  - el único `ERROR` observado es `gazebo ... exit code 255` durante cleanup posterior a `SIM-DONE success=true`;
  - no aparecen `FATAL`, `Segmentation fault`, `Killed`, `std::bad_alloc` ni `No space left`;
  - el dataset raw dejó el disco con margen bajo, unos `467M`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_1D.md` para crear `GlobalPoseStore` usando el dataset raw/replay generado por `1C`, vigilando espacio en disco antes de simulaciones largas.

