# Historial 1A

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-03 11:55 — Subfase 1A — Baseline servidor actual

- fase y subfase: Fase 1, `subfase_1A.md`.
- objetivo intentado: capturar una baseline verificable del servidor monolítico actual antes de iniciar la migración de `1B`, sin corregir lógica funcional.
- archivos modificados:
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1A.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1B.md`.
- código de paquetes modificado: ninguno.
- documentación de paquetes actualizada: no aplica, porque no se modificó código, launch, configuración ni scripts de paquetes.
- trayectoria usada:
  - `dron_1` a fiducial 2 en `(0.0, -9.0, 1.0)` con yaw `90` y tiempos `20 s`;
  - `dron_2` encima en `(0.0, -9.0, 1.3)` con yaw `90` y tiempos `20 s`;
  - `dron_1` a `(-6.0, -9.0, 1.0)` con yaw `90` y tiempos `20 s`;
  - `dron_2` encima en `(-6.0, -9.0, 1.3)` con yaw `90` y tiempos `20 s`;
  - `dron_1` vuelve a fiducial 2 en `(0.0, -9.0, 1.0)` con yaw `90` y tiempos `20 s`;
  - `dron_2` vuelve encima en `(0.0, -9.0, 1.3)` con yaw `90` y tiempos `20 s`;
  - espera final de `20 s`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_ros2 orbslam3_server dron_individual simulacion_dron lib_tray
  ```
- resultado de build: `OK`.
  - `BUILD-EXIT-CODE 0`;
  - `6 packages finished`: `orbslam3_msgs`, `orbslam3_multi`, `orbslam3_server`, `lib_tray`, `dron_individual`, `simulacion_dron`;
  - `colcon` avisó `ignoring unknown package 'orbslam3_ros2' in --packages-select`, porque el directorio `orbslam3_ros2` declara el paquete ROS como `orbslam3`.
- pruebas Gazebo ejecutadas:
  - primer intento de `prueba_1`: falló antes de ejecutar el escenario por `yaml-cpp: illegal map value` en `description`; se corrigió `tray_prueba_1.yaml` poniendo el texto entre comillas.
  - segundo intento de `prueba_1`:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20
    ```
    Resultado mecánico: `SIM-SCENARIO-EXIT-CODE 0`, `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|GOAL|RESULT|success|PIPE0-WRAPPER|WRAPPER|ORBMAP|GLOBAL-MAP|GLOBALMAP|SUBMAP|FID|FIDUCIAL|FUSED|LOOP|SUBCLOUD|LOCAL-OPT|LOCAL-POSE-GRAPH|POINTCLOUD|MAP-CORRECTION|CORRECTED|ERROR|FATAL|Segmentation fault|Killed
  ```
- logs generados:
  - `codex/archivos_auxiliares/logs/colcon_build.log`: `37` líneas;
  - `codex/archivos_auxiliares/logs/prueba_1.log`: `23715` líneas;
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`: `2151` líneas.
- evidencia positiva encontrada:
  - `scenario_runner_node` se ejecutó con el escenario `prueba_1a_baseline_fiducial_loop_secuencial`;
  - se enviaron 6 goals y los 6 terminaron con `success=true`;
  - aparecen `PIPE0-WRAPPER-TRACK` y `PIPE0-WRAPPER-DELTA-PUB` para `dron_1` y `dron_2`;
  - aparecen `PIPE0-SERVER-DELTA-RX`, `PIPE0-SERVER-MAP-IMPORT-END` y `HIST-EPOCH-SUBMAP-STORE`;
  - los submapas quedan identificados por `drone_1 epoch=0` y `drone_2 epoch=0`;
  - aparecen fiduciales anclados con `PIPE0-FID-OBS-ANCHORED` para `drone_1 epoch=0 fiducial_2` y `drone_2 epoch=0 fiducial_2`;
  - aparece `FIDUCIAL_DIRECT maturity=CONFIRMED`;
  - aparece `PIPE0-CORRECTION-PUB` para `drone_1 epoch=0` con `source=FIDUCIAL_DIRECT` y `absolute=true`;
  - aparece `POSE4B-KF-PUB drone_1 keyframes=55`;
  - aparecen `FUSED-SIMPLE-SUBMAP` para ambos drones, con `publicable=true`, `source=FIDUCIAL_DIRECT` y `maturity=CONFIRMED`;
  - aparece `HIST-EPOCH-FUSED-SUMMARY submaps_total=2 submaps_publicable=2` y candidatos fused totales en torno a `4233`;
  - aparecen muestras de loops/subclouds, por ejemplo `LOOP-FRUSTUM-SAMPLE` y `SUBCLOUD-LOCAL-OPT-TASK-SUMMARY`;
  - la baseline deja claro qué logs legacy conviene conservar temporalmente al migrar a `1B`.
- evidencia negativa o ausente:
  - no se observó RViz2 manualmente; no se inventa observación visual;
  - no hubo `LOCAL-OPT-APPLY-SUMMARY applied=true`;
  - los diagnósticos finales muestran `LOCAL-OPT-APPLY-TICK candidates=0 applied=0 optimized_kfs=0 optimized_submaps=0`;
  - hay tareas locales pendientes: `tasks_total=9`, `pending_regions=1`, `submaps_blocked=2`;
  - la fusión subcloud queda bloqueada por `OPTIMIZATION_PENDING_DRYRUN`;
  - `reduce_simulation_log.sh` emitió `grep: write error: Broken pipe`, pero generó `prueba_1.reduced.log`;
  - el único `ERROR` grave encontrado fue `gazebo ... exit code 255` durante el cierre, después de `SIM-DONE success=true`;
  - no aparecen `FATAL`, `Segmentation fault`, `Killed` ni `std::bad_alloc`.
- comportamiento a preservar al crear el nuevo servidor en `1B`:
  - recepción de deltas `OrbMap` multi-dron;
  - identidad de submapa `(drone_id, map_epoch)`;
  - logging suficiente de wrapper, servidor, submapas, fiduciales, correcciones y publicación fused;
  - publicación de correcciones y `corrected_keyframes` cuando hay anchor confirmado;
  - publicación fused solo para submapas publicables/anclados.
- problemas heredados que no deben confundirse con fallos nuevos:
  - servidor monolítico mezcla ROS, fiduciales, fused map, loops y optimización;
  - optimización local presente pero sin apply útil en esta baseline;
  - subcloud fusion puede quedar bloqueada por regiones pendientes;
  - el selector de build `orbslam3_ros2` no coincide con el nombre ROS real `orbslam3`.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_1B.md` para congelar el servidor legacy como referencia y crear el servidor nuevo mínimo que reciba `OrbMap` delta multi-dron con logs `F1B`.

