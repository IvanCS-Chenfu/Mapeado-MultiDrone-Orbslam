# Historial 12R-D4

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-06-30 11:43 — Subfase 12R-D4

- objetivo intentado: desbloquear el apply de optimizacion local para que KFs normales de submapas anclados por fiducial puedan moverse, manteniendo fijos KFs realmente fiduciales/hard-fixed.
- cambios realizados:
  - `orbslam3_server/src/global_map_server.cpp`: guard D4 en `ApplyLocalOptimizationDryRunResultToServer`, log `LOCAL-OPT-APPLY-KF-ALLOW`, log `LOCAL-OPT-APPLY-PRECHECK accepted=true`, balance query/candidate en seleccion de variables.
  - `orbslam3_server/include/orbslam3_server/global_map_server.hpp`: parametros miembro `local_pose_graph_max_query_variables_` y `local_pose_graph_max_candidate_variables_`.
  - `orbslam3_server/launch/global_orb_map_server.launch.py`: caps query/candidate `8/6` y reduccion de logs masivos a `40` KFs con edge logs desactivados.
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`: escenario limpio/casi limpio.
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`: escenario con deriva lateral fuerte.
- comando de build:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
  ```
- resultado de build: `OK`.
  - primer intento fallo por artefacto generado en `build/orbslam3_msgs/ament_cmake_python/orbslam3_msgs/orbslam3_msgs`;
  - se ejecuto `reduce_build_log.sh`, se diagnostico symlink/directorio generado, se limpio solo ese artefacto y el segundo build paso.
- pruebas Gazebo ejecutadas:
  - `prueba_1`: `OK` mecanico (`SIM-SCENARIO-EXIT-CODE 0`, `SIM-DONE success=true`, `SIM-EXIT-CODE 0`), pero sin evidencia de apply util.
  - `prueba_2`: `OK` mecanico (`SIM-SCENARIO-EXIT-CODE 0`, `SIM-DONE success=true`, `SIM-EXIT-CODE 0`), pero sin evidencia de la ruta completa `LOCAL_LOOP_OPT_TASK -> solver -> apply`.
- patrones de reduccion usados:
  - prueba 1: `SCENARIO-RUNNER|LOCAL_LOOP_OPT_TASK|LOCAL-POSE-GRAPH|LOCAL-OPT|LOOP-SUBCLOUD|FUSION_EVENT|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc`
  - prueba 2: `SCENARIO-RUNNER|LOCAL_LOOP_OPT_TASK|LOCAL-POSE-GRAPH|LOCAL-OPT-APPLY|LOCAL-OPT-DRYRUN|LOCAL-OPT-WINDOW|LOOP-SUBCLOUD|SUBCLOUD-LOCAL-OPT-TASK-CREATE|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc`
- evidencia positiva:
  - el build seleccionado compilo `orbslam3_msgs`, `orbslam3_multi` y `orbslam3_server`;
  - el launch imprimio `LOCAL-POSE-GRAPH-SAFETY-CONFIG ... max_query_vars=8 max_candidate_vars=6`;
  - ambas simulaciones arrancaron `scenario_runner_node`, enviaron goals y reportaron `success=true`;
  - prueba 2 mostro actividad parcial de cola: `local_opt_task_ready=1`, `local_opt_windows=1`, `dryrun_results=1`.
- evidencia negativa o ausente:
  - no aparecieron eventos reales `LOCAL_LOOP_OPT_TASK` ni `SUBCLOUD-LOCAL-OPT-TASK-CREATE` en los logs reducidos, salvo las lineas de patron/configuracion;
  - no aparecieron `LOCAL-POSE-GRAPH-VAR-SELECT`, `LOCAL-POSE-GRAPH-SOLVER`, `LOCAL-OPT-APPLY-PRECHECK accepted`, `LOCAL-OPT-APPLY-KF-ALLOW` ni `LOCAL-OPT-APPLY-SUMMARY applied=true moved>0`;
  - no se pudo demostrar que KFs normales se movieran ni que hard fiducials no se movieran;
  - la reduccion de logs no permitio reconstruir con claridad la relacion entre loops, contadores de optimizacion y apply.
- conclusion: `NO CONSEGUIDA`.
- siguiente paso recomendado: no pasar a `12R-D5`; repetir `12R-D4` diagnosticando si la tarea parcial se procesa demasiado tarde, si falta un patron de reduccion para conservar los marcadores, o si la decision de subnube no cruza realmente a `LOCAL_LOOP_OPT_TASK`.

## 2026-06-30 12:00 — Aclaracion de usuario sobre 12R-D4

- objetivo intentado: registrar la observacion visual de RViz2 del usuario y corregir la interpretacion de las pruebas D4 antes de repetir simulaciones.
- cambios realizados:
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_12R-D4.md`: se anadio protocolo de repeticion N veces del mismo YAML, escenario base secuencial y obligacion de combinar logs con observacion RViz2.
  - `codex/contexto/01_ESTADO_ACTUAL.md`: se corrigio la interpretacion del estado y del cierre de Gazebo.
  - `codex/contexto/07_ULTIMA_SESION.md`: se anadio la aclaracion posterior.
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`: se documento que optimizacion implica loop/tarea derivada y que hay que distinguir fallo de codigo frente a fallo de reduccion.
  - `codex/contexto/paquetes/simulacion_dron/simulacion_dron.md` y `codex/contexto/paquetes/dron_individual/trayectorias.md`: se documento que la deriva/no deriva no se garantiza solo cambiando YAML.
- comando de build: no ejecutado; no se modifico codigo.
- resultado de build: `NO EJECUTADO`.
- pruebas Gazebo ejecutadas: `NO EJECUTADA` en esta actualizacion documental.
- patrones de reduccion recomendados para la siguiente ejecucion:
  - `SCENARIO-RUNNER|LOOP-SUBCLOUD|LOOP-SUBCLOUD-DECISION|LOOP-ERROR-UNIFIED|LOCAL_LOOP_OPT_TASK|decision=LOCAL_LOOP_OPT_TASK|SUBCLOUD-LOCAL-OPT-TASK-CREATE|OPT-TASK|OPT-TASK-QUEUE-STATE|local_opt_task_ready|local_opt_windows|dryrun_results|LOCAL-POSE-GRAPH|LOCAL-OPT-APPLY|LOCAL-OPT-DRYRUN|LOCAL-OPT-WINDOW|FUSION_EVENT|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc`
- evidencia positiva:
  - el usuario observo en RViz2 deriva importante y ruido/paredes duplicadas en ambas pruebas anteriores;
  - si RViz2 muestra deriva, la siguiente ejecucion debe buscar loops o explicar por que el log no los conserva;
  - `exit code 255` de Gazebo al final no debe considerarse error si ocurre tras `SIM-DONE success=true` durante cleanup.
- evidencia negativa o ausente:
  - en la ejecucion anterior no se podia decidir si faltaban loops por codigo o por reduccion del log, porque los logs ya estaban reducidos y faltaban marcadores de decision/cola suficientes;
  - la separacion anterior "prueba limpia" vs "prueba deriva" por YAML era conceptualmente debil.
- conclusion: `NO CONSEGUIDA`.
- siguiente paso recomendado: repetir `12R-D4` con un mismo escenario secuencial, tantas veces como haga falta, hasta obtener una ejecucion visualmente limpia y otra con deriva; tras cada prueba Codex debe dar conclusion preliminar por logs y esperar la observacion RViz2 del usuario antes de cerrar conclusion final.

## 2026-06-30 12:17 — Subfase 12R-D4 repetida con escenario secuencial

- objetivo intentado: repetir `12R-D4` con el mismo escenario secuencial en dos ejecuciones, ampliar patrones de reduccion y combinar logs con la observacion RViz2 del usuario.
- cambios realizados:
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`: sustituido por escenario secuencial recomendado.
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`: misma trayectoria secuencial que prueba 1, como segunda repeticion.
  - No se modifico codigo C++ ni launch en esta repeticion.
- comando de build:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
  ```
- resultado de build: `OK`.
  - `BUILD-EXIT-CODE 0`
  - `3 packages finished`: `orbslam3_msgs`, `orbslam3_multi`, `orbslam3_server`.
- pruebas Gazebo ejecutadas:
  - `prueba_1`: `OK` mecanico (`SIM-SCENARIO-EXIT-CODE 0`, `SIM-DONE success=true`, `SIM-EXIT-CODE 0`).
  - `prueba_2`: `OK` mecanico (`SIM-SCENARIO-EXIT-CODE 0`, `SIM-DONE success=true`, `SIM-EXIT-CODE 0`).
- patrones de reduccion usados:
  - prueba 1: `SCENARIO-RUNNER|LOOP-SUBCLOUD|LOOP-SUBCLOUD-DECISION|LOOP-ERROR-UNIFIED|LOCAL_LOOP_OPT_TASK|decision=LOCAL_LOOP_OPT_TASK|SUBCLOUD-LOCAL-OPT-TASK-CREATE|OPT-TASK|OPT-TASK-QUEUE-STATE|local_opt_task_ready|local_opt_windows|dryrun_results|LOCAL-POSE-GRAPH|LOCAL-OPT|FUSION_EVENT|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc`
  - prueba 2: `SCENARIO-RUNNER|LOOP-SUBCLOUD|LOOP-SUBCLOUD-DECISION|LOOP-ERROR-UNIFIED|LOCAL_LOOP_OPT_TASK|decision=LOCAL_LOOP_OPT_TASK|SUBCLOUD-LOCAL-OPT-TASK-CREATE|OPT-TASK|OPT-TASK-QUEUE-STATE|local_opt_task_ready|local_opt_windows|dryrun_results|LOCAL-POSE-GRAPH|LOCAL-OPT-APPLY|LOCAL-OPT-DRYRUN|LOCAL-OPT-WINDOW|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc`
- observacion RViz2 del usuario:
  - prueba 1: no habia deriva; deberia haber solo fusiones y ningun loop de optimizacion.
  - prueba 2: al volver `dron_1` al fiducial 2 aparecio deriva.
- evidencia positiva:
  - ambas simulaciones completaron `scenario_runner_node` con `success=true`;
  - el patron ampliado si conserva `LOOP-SUBCLOUD-*`, asi que la ausencia anterior de loops era parcialmente un problema de reduccion/patrones;
  - prueba 1 no genero `LOCAL_LOOP_OPT_TASK`, solver ni apply, coherente con la observacion de RViz2 sin deriva;
  - prueba 2 mostro muchos `LOOP-SUBCLOUD-*` y `LOOP-ERROR-UNIFIED`; hay evidencia de revisitas/fusiones/diagnosticos.
- evidencia negativa o ausente:
  - en prueba 2, pese a la deriva observada en RViz2, no aparecio ningun `decision=LOCAL_LOOP_OPT_TASK` real;
  - `SUBCLOUD-LOCAL-OPT-TASK-SUMMARY` termino con `local_loop_events=0 created=0`;
  - `OPT-TASK-QUEUE-STATE` termino con `local_loop=0`, `tasks=0`, `windows=0`, `dryrun_results=0`, `apply_results=0`;
  - no aparecieron `LOCAL-POSE-GRAPH-VAR-SELECT`, `LOCAL-POSE-GRAPH-SOLVER`, `LOCAL-OPT-APPLY-PRECHECK`, `LOCAL-OPT-APPLY-KF-ALLOW` ni `LOCAL-OPT-APPLY-SUMMARY applied=true moved>0`;
  - la subfase no ejercito el guard D4 de apply porque nunca se creo tarea local.
- conclusion: `NO CONSEGUIDA`.
- siguiente paso recomendado: antes de repetir el apply de D4 o pasar a `12R-D5`, diagnosticar por que una deriva visible en RViz2 se clasifica por subcloud como `FUSION_EVENT`/`HOLD_DIAGNOSTIC` y no como `LOCAL_LOOP_OPT_TASK`. Comparar `LOOP-ERROR-UNIFIED` alto contra `LOOP-SUBCLOUD-ERROR` bajo sin reactivar el criterio legacy como decision principal.

## 2026-06-30 12:45 — Cierre documental de 12R-D4

- objetivo intentado: dejar el proyecto preparado para otro chat de Codex,
  documentando el diagnostico fino de logs y el siguiente punto tecnico.
- cambios realizados:
  - `codex/contexto/07_ULTIMA_SESION.md`: se reescribio el resumen de cierre con
    objetivo, archivos, build, simulaciones, logs, conclusion y siguiente paso.
  - `codex/contexto/01_ESTADO_ACTUAL.md`: se anadio el analisis posterior de
    `LOOP-ERROR-UNIFIED` alto frente a `LOOP-SUBCLOUD-ERROR` bajo.
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_12R-D4.md`: se anadio
    la diferencia operativa entre unified/viewmap y subcloud decision.
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`: se documento
    que `ComputeLoopSubcloudError`/`DecideLoopActionFromSubcloudError` son la
    ruta de decision real y `LOOP-ERROR-UNIFIED` es diagnostico.
- comando de build: no ejecutado en este cierre documental.
- resultado de build: `NO EJECUTADO`.
  - ultimo build real de D4 sigue siendo `OK`:
    `orbslam3_msgs`, `orbslam3_multi`, `orbslam3_server`.
- pruebas Gazebo ejecutadas: no se ejecutaron nuevas pruebas en este cierre.
  - ultimas pruebas reales siguen siendo:
    - `prueba_1`: `OK` mecanico, `SIM-DONE success=true`;
    - `prueba_2`: `OK` mecanico, `SIM-DONE success=true`.
- patrones de reduccion usados: no se regeneraron logs en este cierre.
  - se revisaron directamente `codex/archivos_auxiliares/logs/prueba_1.log` y
    `codex/archivos_auxiliares/logs/prueba_2.log`.
- evidencia positiva:
  - prueba 1: RViz2 sin deriva segun usuario; `LOOP-SUBCLOUD-ERROR` bajo
    (`max error_t=0.2001`), decisiones de fusion y sin optimizacion;
  - prueba 2: RViz2 con deriva al volver `dron_1` al fiducial 2; aparece al
    final `LOOP-ERROR-UNIFIED` alto (`error_t=3.1834`,
    `error_rot_deg=25.892`, `usable_opt=true`), lo que confirma discrepancia
    fuerte en la ruta unified/viewmap;
  - la diferencia entre `LOOP-ERROR-UNIFIED` y `LOOP-SUBCLOUD-DECISION` quedo
    documentada para evitar diagnosticos ambiguos.
- evidencia negativa o ausente:
  - en prueba 2, la ruta moderna `LOOP-SUBCLOUD-ERROR` mantuvo errores bajos
    (`max error_t=0.2092`) y produjo fusiones;
  - no aparecio ningun `decision=LOCAL_LOOP_OPT_TASK` real;
  - no aparecieron `SUBCLOUD-LOCAL-OPT-TASK-CREATE`,
    `LOCAL-POSE-GRAPH-SOLVER` ni `LOCAL-OPT-APPLY-SUMMARY`;
  - por tanto no hay evidencia de optimizacion local ejecutada.
- conclusion: `NO CONSEGUIDA`.
- siguiente paso recomendado: no pasar a `12R-D5`. Revisar en
  `orbslam3_server/src/global_map_server.cpp` la conexion entre diagnostico
  unified/viewmap alto y decision moderna de subcloud, sin volver a usar
  `LOOP-VIEWMAP-ERROR` como criterio principal. La solucion debe basarse en
  evidencia geometrica real suficiente y terminar generando
  `LOCAL_LOOP_OPT_TASK`.

## 2026-06-30 13:02 — Subfase 12R-D4 continuacion unified/subcloud

- objetivo intentado: diagnosticar y corregir por que en `prueba_2` aparecia
  `LOOP-ERROR-UNIFIED` alto con `usable_opt=true`, pero la ruta moderna
  `LOOP-SUBCLOUD-DECISION` no generaba `LOCAL_LOOP_OPT_TASK`.
- archivos modificados:
  - `orbslam3_server/include/orbslam3_server/global_map_server.hpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_12R-D4.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- cambios realizados:
  - se anadio `CreateSubcloudLocalLoopEventFromUnifiedLoop`;
  - `RegisterUnifiedLoopRuntimeEvent` ahora puede promocionar un evento unified
    fuerte a `LoopSubcloudEvent decision=LOCAL_LOOP_OPT_TASK` si hay evidencia
    geometrica suficiente;
  - se anadieron parametros `subcloud_decision_unified_aux_*`;
  - se anadieron logs `SUBCLOUD-UNIFIED-AUX-EVENT` y
    `SUBCLOUD-UNIFIED-AUX-SKIP`;
  - no se reintrodujo `LOOP-VIEWMAP-ERROR` como criterio principal.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
  ```
- resultado de build: `OK`.
  - `BUILD-EXIT-CODE 0`.
  - `3 packages finished`: `orbslam3_msgs`, `orbslam3_multi`,
    `orbslam3_server`.
  - quedan warnings preexistentes en `global_map_server.cpp`
    (`anchored_by_loop`, parametro `context`, variables de publicacion fused);
    no bloquean la subfase.
- pruebas Gazebo ejecutadas:
  - `prueba_1`:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20
    ```
    Resultado mecanico: `SIM-SCENARIO-EXIT-CODE 0`,
    `SIM-DONE success=true`, `SIM-EXIT-CODE 0`.
  - `prueba_2`:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20
    ```
    Resultado mecanico: `SIM-SCENARIO-EXIT-CODE 0`,
    `SIM-DONE success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  - prueba 1: `SCENARIO-RUNNER|LOOP-SUBCLOUD|LOOP-SUBCLOUD-DECISION|LOOP-ERROR-UNIFIED|SUBCLOUD-UNIFIED-AUX|LOCAL_LOOP_OPT_TASK|decision=LOCAL_LOOP_OPT_TASK|SUBCLOUD-LOCAL-OPT-TASK-CREATE|OPT-TASK|OPT-TASK-QUEUE-STATE|local_opt_task_ready|local_opt_windows|dryrun_results|LOCAL-POSE-GRAPH|LOCAL-OPT|FUSION_EVENT|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc`
  - prueba 2: `SCENARIO-RUNNER|LOOP-SUBCLOUD|LOOP-SUBCLOUD-DECISION|LOOP-ERROR-UNIFIED|SUBCLOUD-UNIFIED-AUX|LOCAL_LOOP_OPT_TASK|decision=LOCAL_LOOP_OPT_TASK|SUBCLOUD-LOCAL-OPT-TASK-CREATE|OPT-TASK|OPT-TASK-QUEUE-STATE|local_opt_task_ready|local_opt_windows|dryrun_results|LOCAL-POSE-GRAPH|LOCAL-OPT-APPLY|LOCAL-OPT-DRYRUN|LOCAL-OPT-WINDOW|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc`
- evidencia positiva:
  - `prueba_1` genero `10` `SUBCLOUD-UNIFIED-AUX-EVENT` y `10`
    `SUBCLOUD-LOCAL-OPT-TASK-CREATE`;
  - `prueba_1` genero `2` `LOCAL-POSE-GRAPH-SOLVER`;
  - `prueba_1` aplico una optimizacion util:
    `LOCAL-OPT-APPLY-SUMMARY applied=true moved=11`;
  - `prueba_1` emitio `LOCAL-OPT-APPLY-KF-ALLOW`, validando el guard D4 para
    KFs normales de submapa anclado por fiducial;
  - `prueba_2` genero `11` `SUBCLOUD-UNIFIED-AUX-EVENT` y `11`
    `SUBCLOUD-LOCAL-OPT-TASK-CREATE`, asi que la transicion hacia
    `LOCAL_LOOP_OPT_TASK` ya ocurre en la ruta moderna;
  - `SUBCLOUD-UNIFIED-AUX-SKIP` confirma que el puente auxiliar rechaza
    candidatos unified cuando la geometria no es suficientemente fuerte.
- evidencia negativa o ausente:
  - `prueba_2` no genero `LOCAL-POSE-GRAPH-SOLVER`;
  - `prueba_2` no genero `LOCAL-OPT-APPLY-SUMMARY applied=true`;
  - el estado final de `prueba_2` quedo en `tasks=11`, `pending=11`,
    `tasks_with_window=4`, `tasks_window_failed=7`,
    `tasks_dryrun_failed=4`, `tasks_dryrun_useful=0` y `apply_results=0`;
  - por tanto la ejecucion con deriva no demuestra `moved>0`.
- conclusion: `PARCIAL`.
- siguiente paso recomendado: no pasar a `12R-D5`. Continuar `12R-D4`
  diagnosticando `BuildLocalOptimizationWindowFromTask` y
  `RunLocalOptimizationDryRunForWindow` para explicar por que las tareas
  modernas de `prueba_2` fallan antes de solver/apply util.

