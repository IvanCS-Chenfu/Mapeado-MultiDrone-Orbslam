# Historial 1B

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-06 11:42 — Subfase 1B — Servidor nuevo mínimo y congelación del legacy

- fase y subfase: Fase 1, `subfase_1B.md`.
- objetivo intentado: sustituir el servidor monolítico activo por un servidor mínimo que arranca desde el launch oficial, se suscribe a `OrbMap` delta multi-dron y emite logs `[F1B-*]`, dejando el legacy congelado como referencia.
- archivos modificados:
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/src/global_map_server_antiguo.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `orbslam3_server/launch/global_orb_map_server_antiguo.launch.py`;
  - `orbslam3_server/CMakeLists.txt`;
  - `orbslam3_multi/legacy/README.md`;
  - `orbslam3_multi/legacy/*_antiguo.cpp`;
  - `orbslam3_multi/legacy/*_antiguo.hpp`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/03_ARQUITECTURA_ACTUAL.md`;
  - `codex/contexto/04_TOPICS_SERVICES_ACTIONS.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server_antiguo.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/simulacion_dron/simulacion_dron.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1B.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1C.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- cambios de código realizados:
  - `global_map_server.cpp` ahora define un `GlobalMapServer` mínimo autocontenido;
  - el servidor declara/lee `use_sim_time`, `world_frame`, `namespace_base`, `n_drones` y `f1b_stats_period_s`;
  - se crea un subscriber por dron a `/dron_X/orbslam/orb_map_delta`;
  - el callback `OnOrbMapDelta` loggea `drone_id`, `map_epoch`, `map_sequence`, `frame_id`, `map_frame`, KFs y MPs;
  - `PublishStatsLog` emite acumulados por dron y epochs vistos;
  - el servidor activo no incluye `global_map_server.hpp` heredado;
  - `CMakeLists.txt` deja de enlazar `global_map_server` con `ORB_SLAM3/g2o`.
- cambios legacy:
  - `global_map_server_antiguo.cpp` conserva el servidor monolítico anterior y no se compila;
  - `global_orb_map_server_antiguo.launch.py` conserva el launch anterior y no es el activo;
  - `orbslam3_multi/legacy/` contiene copias congeladas no compiladas de módulos actuales.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server simulacion_dron
  ```
- resultado de build:
  - primer intento: `BUILD-EXIT-CODE 2` por `No space left on device`;
  - se ejecutó `reduce_build_log.sh`;
  - se limpió de forma mínima `/home/chenfu/Gazebo/build/orbslam3_server` y `/home/chenfu/Gazebo/log/build_2026-07-06_11-35-41`;
  - segundo intento: `BUILD-EXIT-CODE 0`;
  - `4 packages finished`: `orbslam3_msgs`, `orbslam3_multi`, `orbslam3_server`, `simulacion_dron`;
  - warning no bloqueante: `global_pose_corrector.cpp` tiene variable `deg_to_rad` no usada.
- prueba Gazebo ejecutada:
  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20
  ```
- YAML usado:
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - escenario `prueba_1b_recepcion_orbmap_multidron`;
  - trayectoria multi-dron de 1A: fiducial 2, izquierda y vuelta, con `dron_2` a mayor altura.
- resultado de simulación:
  - `SIM-SCENARIO-EXIT-CODE 0`;
  - `SIM-DONE prueba=1 success=true`;
  - `SIM-EXIT-CODE 0`;
  - 6 goals enviados y 6 goals con `success=true`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|GOAL|RESULT|success|PIPE0-WRAPPER-DELTA-PUB|PIPE0-WRAPPER-FULL-SNAPSHOT|PIPE0-WRAPPER-TRACK|F1B-SERVER-INIT|F1B-SERVER-PARAMS|F1B-SERVER-SUBSCRIBED|F1B-ORBMAP-RX|F1B-ORBMAP-RX-KFS|F1B-ORBMAP-RX-MPS|F1B-SERVER-STATS|ERROR|FATAL|Segmentation fault|Killed
  ```
- logs generados:
  - `codex/archivos_auxiliares/logs/colcon_build.log`: `39` líneas;
  - `codex/archivos_auxiliares/logs/colcon_build.reduced.log`: generado para el primer build fallido por espacio;
  - `codex/archivos_auxiliares/logs/prueba_1.log`: `1906` líneas;
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`: `1788` líneas.
- evidencia positiva encontrada:
  - `[F1B-SERVER-INIT] node=global_orb_map_server mode=minimal_rx_only`;
  - `[F1B-SERVER-PARAMS] use_sim_time=true world_frame=world namespace_base=dron drones=2`;
  - `[F1B-SERVER-SUBSCRIBED]` para `drone_id=1 topic=/dron_1/orbslam/orb_map_delta`;
  - `[F1B-SERVER-SUBSCRIBED]` para `drone_id=2 topic=/dron_2/orbslam/orb_map_delta`;
  - `[F1B-ORBMAP-RX]` aparece para `drone_id=1` y `drone_id=2`;
  - conteo reducido: `174` recepciones de `drone_id=1` y `173` de `drone_id=2`;
  - `[F1B-ORBMAP-RX-KFS]` y `[F1B-ORBMAP-RX-MPS]` aparecen con conteos y rangos de IDs;
  - `[F1B-SERVER-STATS]` final muestra `rx_maps=329`, `rx_kfs=2030`, `rx_mps=482182`, `drones_seen=2`, `epochs_seen=3`;
  - `PIPE0-WRAPPER-TRACK` y `PIPE0-WRAPPER-DELTA-PUB` confirman wrappers activos.
- evidencia negativa o ausente:
  - no hay nube global, fiduciales, loops, fused landmarks ni optimización; es esperado en `1B`;
  - no se observó RViz2 porque no es criterio de éxito;
  - el único `ERROR` es `gazebo ... exit code 255` durante cleanup posterior a `SIM-DONE success=true`;
  - no aparecen `FATAL`, `Segmentation fault`, `Killed`, `std::bad_alloc` ni `No space left` durante la simulación;
  - el disco sigue con poco margen libre, unos `180M` tras la simulación, y conviene vigilarlo en la siguiente subfase.
- criterio de éxito:
  - build final `0`: cumplido;
  - prueba Gazebo ejecutada: cumplido;
  - `scenario_runner_node` y goals `success=true`: cumplido;
  - marcadores `[F1B-*]` obligatorios: cumplido;
  - recepción de al menos dos drones: cumplido;
  - legacy no compilado: cumplido;
  - documentación e historial actualizados: cumplido.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_1C.md` para crear `RawMapDatabase`, asignar `arrival_id`, guardar estado raw/journal y preparar replay sin Gazebo.

## 2026-07-06 — Subfase 1B reabierta — Comentarios y trazabilidad por subfase

- motivo: el usuario no quiere dar por concluida `1B` hasta que el código nuevo quede comentado con suficiente detalle y trazabilidad por subfase.
- decisión de estado:
  - la evidencia técnica de `1B` se conserva: build final `0`, simulación `success=true`, recepción `OrbMap` de dos drones y logs `[F1B-*]`;
  - `1B` vuelve a ser la subfase actual;
  - `1C` vuelve a quedar `sin hacer`;
  - la conclusión operativa pasa a `PARCIAL` hasta revisar comentarios/trazabilidad.
- regla añadida:
  - funciones nuevas o modificadas deben explicar objetivo, entradas relevantes, efecto y subfase;
  - callbacks ROS deben explicar topic procesado, estado actualizado y qué subfase los introdujo;
  - bucles y guards importantes deben explicar invariantes y motivo;
  - los comentarios deben usar etiquetas como `F1B`, `F1C`, etc. cuando correspondan a una subfase concreta.
- archivos modificados para reglas de agente/contexto:
  - `AGENTS.md`;
  - `.codex/agents/implementador_fase.toml`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/03_ARQUITECTURA_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1B.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1C.md`.
- archivos de paquete/documentación actualizados:
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`.
- archivos de código comentados:
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `orbslam3_server/CMakeLists.txt`.
- build/simulación:
  - no se reejecutan en esta entrada porque los cambios son comentarios y documentación; no modifican lógica, topics, parámetros ni dependencias.
- conclusión: `PARCIAL`.
- siguiente paso recomendado: revisar si los comentarios `F1B` son suficientes; si el usuario los acepta, repetir build/simulación si se desea una validación formal y marcar `1B` como `realizado`.

## 2026-07-06 — Subfase 1B cerrada tras revisión de comentarios

- decisión del usuario: aceptar la subfase `1B` como conseguida tras la revisión de comentarios y trazabilidad `F1B`.
- estado actualizado:
  - `subfase_1B.md`: `realizado`;
  - `subfase_1C.md`: `actual`;
  - `pipeline_fase_1.md`: `1B` pasa a `realizado` y `1C` pasa a `actual`;
  - contexto operativo actualizado para que un chat nuevo arranque en `1C`.
- evidencia conservada:
  - build final `BUILD-EXIT-CODE 0`;
  - simulación `SIM-DONE prueba=1 success=true`;
  - recepción `OrbMap` de `dron_1` y `dron_2`;
  - logs `[F1B-*]` obligatorios;
  - comentarios `F1B` añadidos en servidor, launch y CMake.
- build/simulación:
  - no se reejecutan en este cierre administrativo porque solo cambia estado documental; la evidencia técnica de `1B` ya existe.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: ejecutar `subfase_1C.md` para crear `RawMapDatabase`, asignar `arrival_id`, guardar journal/rawdb y validar replay.

