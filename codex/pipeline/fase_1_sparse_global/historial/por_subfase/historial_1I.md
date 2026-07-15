# Historial 1I

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## Nota de reclasificación 2026-07-14

Parte de la evidencia generada durante la sesión de `1L` pertenece realmente a
`1I`: selección métrica de vecinos fiduciales, muestreo por distancia acumulada,
vertices de esquina, splits de aristas largas y cobertura dura del grafo. Esa
evidencia sigue archivada en `historial_1L_2.md` a `historial_1L_4.md`, pero debe
leerse como evolución del contrato de `PoseGraphBuilder`.

## 2026-07-09 — Subfase 1I — PoseGraphBuilder temporal

- objetivo intentado:
  - crear `PoseGraphProblem` y `PoseGraphBuilder` en `orbslam3_multi`;
  - construir un grafo temporal desde `FiducialOptimizationTask`;
  - loggear vertices, fijos/variables, aristas, priors, pesos y `PropagationPlan`;
  - validar live y replay sin ejecutar solver ni aplicar poses.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - documentacion de contexto, paquetes, subfase e historial.
- cambios realizados:
  - `PoseGraphProblem` contiene vertices, aristas, priors, KFs fijos, KFs variables, KFs afectados no variables y `PropagationPlan`;
  - `PoseGraphBuilder::BuildForFiducialTask` construye el problema desde `FiducialOptimizationTask`, `RawMapDatabase` y `GlobalPoseStore`;
  - la ventana se limita al submapa `(drone_id, map_epoch)` de la tarea;
  - KFs hard fiducial entran fijos y KFs normales permanecen variables;
  - se crean aristas temporales y priors `FIDUCIAL_HARD`, `FIDUCIAL_TARGET` y `CURRENT_POSE_SOFT`;
  - `global_map_server` invoca el builder al detectar tareas pendientes de `FiducialAnchorManager`;
  - se anadieron parametros `pose_graph_*` y `f1i_debug_force_task_enabled`;
  - `tray_prueba_1.yaml` queda como la prueba tipica larga de rodeo con dos fiduciales;
  - `tray_prueba_2.yaml` queda como replay de espera con tarea debug equivalente habilitable.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_multi orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - `simulacion_dron`: `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas ejecutadas:
  - prueba `1`: live larga con `ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false`;
  - prueba `2`: replay con `ros2 launch orbslam3_server global_orb_map_server.launch.py ... rawdb_replay_enabled:=true ... f1i_debug_force_task_enabled:=true`.
- resultado de simulacion:
  - `prueba_1`: `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - `prueba_2`: `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE prueba=2 success=true`, `SIM-EXIT-CODE 0`.
- patrones previstos para reduccion:
  - prueba 1:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1I-|F1H-FID-TASK-CREATED|F1E-FID|F1D-POSESTORE|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba 2:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1I-|F1H-FID-TASK-CREATED|F1E-FID|F1D-POSESTORE|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed
    ```
- nota sobre logs:
  - por instruccion final del usuario, no se regeneraron `prueba_1.reduced.log` ni `prueba_2.reduced.log` en el cierre;
  - la evidencia se consulto directamente en `codex/archivos_auxiliares/logs/prueba_1.log` y `codex/archivos_auxiliares/logs/prueba_2.log`.
- evidencia positiva encontrada:
  - `prueba_1.log`: `109` lineas `[F1H-FID-TASK-CREATED]`;
  - `prueba_1.log`: `109` lineas `[F1I-GRAPH-BUILD-SUMMARY] ... success=true`;
  - `prueba_1.log`: `109` lineas `[F1I-GRAPH-PROPAGATION-PLAN]`;
  - `prueba_2.log`: `16` lineas `[F1H-FID-TASK-CREATED]` reales desde `REPLAY_RECORDED_FIDUCIAL`;
  - `prueba_2.log`: `17` lineas `[F1I-GRAPH-BUILD-SUMMARY] ... success=true`;
  - `prueba_2.log`: aparece `[F1I-DEBUG-TASK-CREATED] task_id=9000000001`;
  - grafos con `vertices=12`, `edges=11`, `priors=12` y propagacion no vacia;
  - no aparecieron `FATAL`, `Segmentation fault`, `Killed`, `OPT-APPLY`, `OPTIMIZATION-APPLIED` ni `SET_OPTIMIZED_POSE`.
- evidencia negativa o ausente:
  - no se ejecuta solver ni apply en esta subfase;
  - `prueba_1.log` contiene `gazebo ... exit code 255` despues de `SIM-DONE success=true`, durante cleanup; patron no bloqueante ya observado;
  - no se regeneraron logs reducidos por la instruccion de cierre.
- documentacion actualizada:
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1I.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- conclusion: `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_1J.md`: dry-run de optimizacion sobre `PoseGraphProblem`, sin aplicar cambios persistentes todavia.

## 2026-07-09 — Revalidacion Subfase 1I — cierre de evidencia reducida

- objetivo intentado:
  - comprobar el estado real de `subfase_1I.md` tras un cierre anterior incompleto por capacidad;
  - repetir el ciclo automatico de build, simulacion, reduccion y analisis definido en `AGENTS.md`;
  - conservar evidencia compacta en `prueba_1.reduced.log` y `prueba_2.reduced.log`.
- archivos modificados:
  - no se modifico codigo fuente;
  - no se modificaron YAMLs de prueba;
  - se actualizo documentacion de cierre:
    - `codex/contexto/07_ULTIMA_SESION.md`;
    - `codex/contexto/01_ESTADO_ACTUAL.md`;
    - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1I.md`;
    - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - los tres builds terminaron con `BUILD-EXIT-CODE 0`;
  - no hizo falta ejecutar `reduce_build_log.sh`.
- pruebas Gazebo ejecutadas:
  - prueba `1`: live larga con `ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false`;
  - prueba `2`: replay/debug con `ros2 launch orbslam3_server global_orb_map_server.launch.py ... rawdb_replay_enabled:=true ... f1i_debug_force_task_enabled:=true`;
  - ambas terminaron con `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE ... success=true` y `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  - se probo primero el patron amplio de la subfase; en `prueba_1.reduced.log` resulto demasiado verboso y oculto marcadores `F1I` relevantes por limite de reduccion;
  - se consulto el log completo como exige `AGENTS.md`;
  - se regeneraron los reducidos con patrones centrados en:
    ```text
    SCENARIO-RUNNER|SIM-DONE|SIM-EXIT-CODE|GOAL|RESULT|success|F1H-FID-TASK-CREATED|F1H-FID-TASK-STATS|F1I-|F1C-REPLAY|ERROR|FATAL|Segmentation fault|Killed|OPT-APPLY|OPTIMIZATION-APPLIED|SET_OPTIMIZED_POSE
    ```
- evidencia positiva encontrada:
  - `prueba_1.reduced.log`: `15` tareas `[F1H-FID-TASK-CREATED]`;
  - `prueba_1.reduced.log`: `15` grafos `[F1I-GRAPH-BUILD-SUMMARY] ... success=true`;
  - `prueba_1.reduced.log`: aparecen `F1I-TASK-RX`, `F1I-GRAPH-BUILD-START`, `F1I-GRAPH-WINDOW`, `F1I-GRAPH-VERTEX-SELECT`, `F1I-GRAPH-EDGES`, `F1I-GRAPH-PRIORS`, `F1I-GRAPH-WEIGHTS`, `F1I-GRAPH-PROPAGATION-PLAN` y `F1I-GRAPH-PROBLEM-CREATED`;
  - `prueba_2.reduced.log`: `16` tareas reales `[F1H-FID-TASK-CREATED]`;
  - `prueba_2.reduced.log`: `17` grafos `[F1I-GRAPH-BUILD-SUMMARY] ... success=true`, incluyendo la tarea debug `task_id=9000000001`;
  - la tarea debug construyo un grafo con `vertices=12`, `variable=10`, `fixed=2`, `hard_fiducial=2`, `edges=11`, `priors=12`, `affected_non_variable=5` y `propagation=5`.
- evidencia negativa o ausente:
  - no aparecieron eventos reales `OPT-APPLY`, `OPTIMIZATION-APPLIED` ni `SET_OPTIMIZED_POSE`, correcto para `1I`;
  - no aparecieron `FATAL`, `Segmentation fault`, `Killed` ni `std::bad_alloc`;
  - `prueba_1.reduced.log` conserva el `gazebo ... exit code 255` posterior a `SIM-DONE`, durante cleanup; se mantiene como no bloqueante.
- conclusion: `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_1J.md`: dry-run de optimizacion sobre `PoseGraphProblem`, sin aplicar todavia cambios persistentes.

## 2026-07-10 — Subfase 1I — Revalidacion con preservacion de anclajes fiduciales previos

- objetivo intentado:
  - corregir la construccion del `PoseGraphProblem` para que una tarea fiducial multi-anclaje no llegue con `fixed=0 hard_fiducial=0`;
  - incluir/proteger fiduciales previos del mismo submapa como frontera fija;
  - emitir logs explicitos de conectividad fiducial, ramas y preservacion de anclajes;
  - validar con prueba larga sin ejecutar dry-run/apply/post-apply.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1I.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1J.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- cambios realizados:
  - `PoseGraphProblem` anade `FiducialConnectivityEdge`, `FiducialConnectivityEdgeStatus` y `PoseGraphAnchorPreservationSummary`;
  - `PoseGraphBuilder` recopila KFs hard fiducial previos del mismo `(drone_id, map_epoch)`, selecciona anclajes frontera por rama temporal cercana y los inserta como vertices fijos;
  - la ventana se expande para incluir esos anclajes aunque queden fuera del path nominal;
  - los fiduciales dominados por un anclaje mas cercano en la misma rama se registran como `SUBDIVIDED_CONFIRMED`;
  - `global_map_server` declara/configura `pose_graph_fiducial_connectivity_enabled`, `pose_graph_branch_selection_enabled` y `pose_graph_subdivision_candidate_radius_m`;
  - `global_map_server` emite `[F1I-FID-CONNECTIVITY-BRANCHES]`, `[F1I-FID-CONNECTIVITY-EDGE]`, `[F1I-FID-CONNECTIVITY-SUBDIVISION]`, `[F1I-GRAPH-ANCHOR-PRESERVATION]` y `[F1I-GRAPH-PREVIOUS-FIDUCIAL-ANCHOR]`.
- YAMLs de prueba:
  - se reutilizo `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml` como prueba larga de rodeo;
  - se reutilizo `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml` como live equivalente a replay porque `rawdb_prueba_1.record` no estaba disponible.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - los tres builds terminaron con salida `0`;
  - no hizo falta ejecutar `reduce_build_log.sh`;
  - `orbslam3_multi` mostro solo un warning preexistente de funcion no usada en `optimization_manager.cpp`.
- pruebas Gazebo ejecutadas:
  - prueba `1` live larga:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=false f1k_apply_enabled:=false f1l_validation_enabled:=false" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
  - prueba `2` live equivalente a replay:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=false f1k_apply_enabled:=false f1l_validation_enabled:=false" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
- resultado de simulacion:
  - `prueba_1`: `14` goals `success=true`, `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`, `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - `prueba_2`: `14` goals `success=true`, `SCENARIO-RUNNER-DONE scenario='prueba_1l_rodeo_edificio_rollback_forzado' success=true`, `SIM-DONE prueba=2 success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER-GOAL-RESULT|SCENARIO-RUNNER-DONE|SIM-DONE|SIM-EXIT-CODE|F1H-FID-TASK-CREATED|F1I-GRAPH|F1I-FID-CONNECTIVITY|ERROR|FATAL|Segmentation fault|Killed|process has died
  ```
- logs reducidos:
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`;
  - `codex/archivos_auxiliares/logs/prueba_2.reduced.log`.
- evidencia positiva:
  - `prueba_1.log`: `33` `[F1H-FID-TASK-CREATED]`, `33` `[F1I-GRAPH-BUILD-SUMMARY]`, `33` `[F1I-GRAPH-ANCHOR-PRESERVATION]`;
  - `prueba_2.log`: `16` `[F1H-FID-TASK-CREATED]`, `16` `[F1I-GRAPH-BUILD-SUMMARY]`, `16` `[F1I-GRAPH-ANCHOR-PRESERVATION]`;
  - caso critico de `prueba_1`: `task_id=1` con `error_t=22.312883` ahora muestra `required=true satisfied=true previous_fiducial_fixed_count=1` y `fixed=1 hard_fiducial=1`;
  - caso critico equivalente de `prueba_2`: `task_id=1` muestra `required=true satisfied=true previous_fiducial_fixed_count=1` y `fixed=1 hard_fiducial=1`;
  - aparecen logs de conectividad con `DIRECT_UNCERTAIN`, `DIRECT_OBSERVED` y `SUBDIVIDED_CONFIRMED`;
  - no aparecen `OPT-APPLY`, `OPTIMIZATION-APPLIED` ni `SET_OPTIMIZED_POSE`, como corresponde a una validacion estricta de `1I`.
- evidencia negativa o notas:
  - no se ejecuto replay real porque no existia `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`;
  - el cierre conocido de Gazebo `exit code 255` aparece despues de `SIM-DONE success=true` y no invalida la prueba;
  - no se observo RViz2 manualmente durante esta ejecucion.
- conclusion:
  - `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_1J.md` para revalidar `OptimizationManager::RunDryRun` con los grafos multi-anclaje ya protegidos.
