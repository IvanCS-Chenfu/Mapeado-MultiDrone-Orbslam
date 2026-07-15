
- objetivo intentado:
  - responder a la inspeccion HTML/RViz2 donde solo se movian los controles
    cercanos al fiducial 1;
  - hacer que el solver reparta la correccion por la ventana, conserve mejor la
    posicion relativa local entre KFs y permita cambios mayores de yaw relativa.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - documentacion de estado, subfase, historial y paquete.
- cambios tecnicos:
  - se anadio una semilla elastica inicial hacia el prior fiducial objetivo;
  - se debilitaron los priors `CURRENT_POSE_SOFT`;
  - las aristas internas usan residual de traslacion relativa local escalado y
    yaw relativa con peso debil;
  - el solver aumenta iteraciones y paso maximo de yaw;
  - `ComputeCost` y `ComputeEdgeResidualStats` se alinean con la semantica de
    residual usada por el solver.
- paquetes compilados:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`.
- prueba Gazebo:
  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0 f1l_debug_animation_enabled:=true" --post-scenario-wait-sec 20 --startup-wait-sec 20 --timeout-sec 900 --max-gazebo-retries 1
  ```
- resultado de simulacion:
  - `SCENARIO-RUNNER-DONE scenario='prueba_1l_dron_2_antihorario_gt_debug' success=true`;
  - `SIM-DONE prueba=1 success=true`;
  - `SIM-EXIT-CODE 0`;
  - `4` goals enviados a `/dron_2/AccionTrayectoria`, todos con
    `success=true`.
- patrones de reduccion:
  ```text
  SCENARIO-RUNNER|SIM-|F1L-GT-|F1L-DEBUG-ANIMATION|F1H-FID-TASK-CREATED|F1I-GRAPH|F1I-FID-CONNECTIVITY|F1J-OPT|F1K-|F1L-|RAWDB-POSE-OVERWRITE-BY-OPT|HARD-FIDUCIAL-MOVED|APPLY-USEFUL-FALSE|ROLLBACK_FAILED|POST_APPLY_ACCEPT_WITH_ERROR_WORSE|ERROR|FATAL|Segmentation fault|Killed|process has died
  ```
- evidencia positiva:
  - task 1 acepto una correccion pequena:
    `before_t=0.393687`, `after_t=0.007719`,
    `improvement_ratio=0.980392`, `[F1L-POST-APPLY-ACCEPT]`;
  - task 2 creo grafo cubierto:
    `vertices=25`, `uncovered_long_segments=0`, `coverage_complete=true`;
  - task 2 llevo el fiducial objetivo casi a GT:
    `before_t=23.445255`, `after_t=0.002936`,
    `before_yaw=1.899575`, `after_yaw=0.000000`;
  - la vecindad del fiducial previo no se desanclo:
    `previous_fid_neighborhood_mean_before=0.104234`,
    `previous_fid_neighborhood_mean_after=0.104234`;
  - `RawMapDatabase` no se modifico y el rollback funciono:
    `[F1L-POSESTORE-ROLLBACK] ok=true`.
- evidencia negativa o ausente:
  - la ventana completa siguio lejos del GT debug:
    `mean_before=9.638503`, `mean_after=8.358120`,
    `max_after=23.627106`;
  - el candidato de task 2 se rechazo:
    `[F1L-POST-APPLY-REJECT] reason=deformable_edges_broken_without_safe_partial`;
  - la validacion interna detecto aristas deformables rotas:
    `internal_max_after=4.370994`, `num_edges_broken=5`,
    `deformable_edges_broken=5`;
  - la propagacion propuesta no fue coherente:
    `propagated_max_delta_t=17.060002`,
    `propagation_discontinuity_max_yaw=3.076036139`,
    `propagation_ok=false`;
  - no hay evidencia suficiente para declarar que el submapa se deforma de
    forma coherente entre fiducial 2 y fiducial 1.
- fallos graves:
  - no aparecieron eventos reales de `ROLLBACK_FAILED`,
    `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `RAWDB-POSE-OVERWRITE-BY-OPT`,
    `HARD-FIDUCIAL-MOVED`, `FATAL`, `Segmentation fault` ni `Killed`;
  - `gazebo ... exit code 255` aparece despues de `SIM-DONE` y se clasifica
    como cleanup no bloqueante.
- conclusion:
  - `PARCIAL`.
- siguiente paso recomendado:
  - mantener `1L` como subfase actual;
  - conservar la mejora milimetrica del fiducial objetivo, pero repartir mejor
    el campo de correccion;
  - probar mas controles o todos los KFs variables en la ventana del caso
    `dron_2` antihorario;
  - anadir regularizacion explicita entre controles y revisar la validacion de
    KFs propagados/omitidos antes de permitir commit.

## 2026-07-12 — Subfase 1L — solver 6D y distancias relativas preservadas

- objetivo intentado:
  - evitar que unos vertices se desplacen demasiado sobre otros;
  - permitir que la correccion se compre con giros relativos, sobre todo yaw,
    y tambien con roll/pitch pequenos, antes que con estiramiento de posiciones.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - documentacion de estado, subfase, historial e `orbslam3_multi`.
- cambios tecnicos:
  - el solver pasa de `x,y,z,yaw` a `x,y,z,roll,pitch,yaw`;
  - yaw mantiene paso maximo mayor y peso relativo muy debil en aristas;
  - roll/pitch tienen paso maximo pequeno y peso bajo;
  - las aristas internas penalizan fuerte el cambio de longitud/distancia
    relativa;
  - la direccion local de la traslacion queda como sesgo debil, no como guard
    principal;
  - `ComputeEdgeResidualStats` valida cambio de longitud relativa, que es la
    metrica directa para detectar separacion/acercamiento entre vertices.
- paquetes compilados:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`.
- prueba Gazebo:
  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0 f1l_debug_animation_enabled:=true" --post-scenario-wait-sec 20 --startup-wait-sec 20 --timeout-sec 900 --max-gazebo-retries 1
  ```
- resultado de simulacion:
  - `SCENARIO-RUNNER-DONE scenario='prueba_1l_dron_2_antihorario_gt_debug' success=true`;
  - `SIM-DONE prueba=1 success=true`;
  - `SIM-EXIT-CODE 0`;
  - `4` goals enviados a `/dron_2/AccionTrayectoria`, todos con
    `success=true`;
  - HTML generado: `codex/archivos_auxiliares/html/f1l_debug_animation_task_1.html`.
- patrones de reduccion:
  ```text
  SCENARIO-RUNNER|SIM-|F1L-GT-|F1L-DEBUG-ANIMATION|F1H-FID-TASK-CREATED|F1I-GRAPH|F1I-FID-CONNECTIVITY|F1J-OPT|F1K-|F1L-|RAWDB-POSE-OVERWRITE-BY-OPT|HARD-FIDUCIAL-MOVED|APPLY-USEFUL-FALSE|ROLLBACK_FAILED|POST_APPLY_ACCEPT_WITH_ERROR_WORSE|ERROR|FATAL|Segmentation fault|Killed|process has died
  ```
- evidencia positiva:
  - grafo cubierto:
    `vertices=25`, `max_edge_kf_gap=11`, `max_edge_length_m=2.856`,
    `uncovered_long_segments=0`, `coverage_complete=true`;
  - dry-run:
    `before_t=9.792646`, `after_t=0.009804`,
    `before_yaw=0.551786`, `after_yaw=0.000000`,
    `improvement_ratio=0.998999`;
  - ventana GT debug mejora:
    `mean_before=4.748133`, `mean_after=1.762549`,
    `max_before=10.032235`, `max_after=4.030853`;
  - ejemplos de KFs intermedios:
    `kf=120 error_t 9.265695 -> 3.549230`,
    `kf=140 error_t 9.922409 -> 3.061134`;
  - apply:
    `optimized_kfs=21`, `propagated_kfs=74`,
    `skipped_propagated_kfs=0`, `raw_db_modified=false`;
  - aristas internas:
    `internal_mean_after=0.004481`, `internal_max_after=0.027445`,
    `num_edges_broken=0`;
  - propagacion:
    `propagation_discontinuity_max_t=0.000000000`,
    `propagation_discontinuity_max_yaw=0.000000000`;
  - decision:
    `[F1L-POST-APPLY-ACCEPT] real_after_t=0.009804`.
- evidencia negativa o pendiente:
  - la ventana completa aun no queda cerrada:
    `max_after=4.030853`;
  - `worsened_kfs=11`;
  - queda pendiente inspeccion visual del HTML/RViz2 para confirmar que la
    cadena no solo pasa metrica interna sino que se ve coherente.
- fallos graves:
  - no aparecieron eventos reales de `ROLLBACK_FAILED`,
    `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `RAWDB-POSE-OVERWRITE-BY-OPT`,
    `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE`, `FATAL`,
    `Segmentation fault` ni `Killed`.
- conclusion:
  - `PARCIAL` con avance fuerte.
- siguiente paso recomendado:
  - inspeccionar `f1l_debug_animation_task_1.html`;
  - si visualmente la cadena ya se dobla bien, ajustar criterio/ventana GT;
  - si aun queda una zona deformada, aumentar controles o hacer variables los
    KFs de la zona responsable del `max_after`.

## 2026-07-12 — Subfase 1L — rigidez angular parabólica probada

- objetivo intentado:
  - probar la hipótesis de rigidez angular en U/parabólica;
  - hacer las aristas cercanas a ambos fiduciales más rígidas angularmente y
    dejar el centro más flexible;
  - evitar que los giros se concentren cerca del fiducial 1.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - documentación de estado, subfase, historial y paquete.
- cambios tecnicos:
  - se añadió `EdgeCenterAlphaInWindow(...)`;
  - se añadió `ParabolicAngularRigidity(...)`;
  - se multiplicó el coste/residual de yaw, roll y pitch relativos por esa
    rigidez angular;
  - tras probar dos parametrizaciones, el estado final queda neutralizado con
    `kAngularRigidityCenterMultiplier=1.0` y
    `kAngularRigidityEndpointExtraMultiplier=0.0`, equivalente al solver 6D
    uniforme validado.
- paquetes compilados:
  - variante fuerte:
    - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
    - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - variante suave:
    - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
    - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - estado final neutralizado:
    - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
    - `orbslam3_server`: `BUILD-EXIT-CODE 0`.
- prueba Gazebo:
  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0 f1l_debug_animation_enabled:=true" --post-scenario-wait-sec 20 --startup-wait-sec 20 --timeout-sec 900 --max-gazebo-retries 1
  ```
- variante fuerte:
  - parametros:
    `center=0.25`, `endpoint_extra=8.0`;
  - resultado:
    `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE success=true`,
    `SIM-EXIT-CODE 0`;
  - dry-run:
    `before_t=26.890869`, `after_t=0.201270`,
    `before_yaw=2.888383`, `after_yaw=0.000035`;
  - GT ventana:
    `mean_before=9.286605`, `mean_after=7.512610`,
    `max_after=22.978994`;
  - aristas internas:
    `internal_max_after=0.216025`, `num_edges_broken=0`;
  - resultado post-apply:
    `REJECT_ROLLBACK`, `reason=propagation_discontinuity`,
    `propagation_discontinuity_max_t=20.053579974`;
  - rollback:
    `[F1L-POSESTORE-ROLLBACK] ok=true`.
- variante suave:
  - parametros:
    `center=0.50`, `endpoint_extra=2.0`;
  - resultado:
    `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE success=true`,
    `SIM-EXIT-CODE 0`;
  - dry-run:
    `before_t=11.632946`, `after_t=0.001035`,
    `before_yaw=0.794660`, `after_yaw=0.002461`;
  - GT ventana:
    `mean_before=4.780997`, `mean_after=4.153484`,
    `max_after=11.441172`, `worsened_kfs=8`;
  - aristas internas:
    `internal_max_after=0.916624`, `num_edges_broken=0`;
  - resultado post-apply:
    `REJECT_ROLLBACK`, `reason=propagation_discontinuity`,
    `propagation_discontinuity_max_t=11.109036790`;
  - rollback:
    `[F1L-POSESTORE-ROLLBACK] ok=true`.
- comparación con mejor evidencia previa:
  - solver 6D uniforme:
    `ACCEPT`, `mean_after=1.762549`, `max_after=4.030853`,
    `internal_max_after=0.027445`, sin discontinuidad de propagación;
  - las dos parábolas fueron peores y no deben quedar activas.
- fallos graves:
  - no aparecieron `ROLLBACK_FAILED`,
    `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `RAWDB-POSE-OVERWRITE-BY-OPT`,
    `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE`, `FATAL`,
    `Segmentation fault` ni `Killed`;
  - los `gazebo ... exit code 255` aparecieron como arranque fallido reintentado
    o cleanup posterior a `SIM-DONE`, no como fallo final de simulación.
- conclusion:
  - `PARCIAL`;
  - la hipótesis parabólica es razonable pero estas parametrizaciones no son
    mejora;
  - el código final conserva el scaffold neutralizado para permitir un barrido
    futuro sin reescribir la función.
- siguiente paso recomendado:
  - no insistir con parábola por posición pura;
  - si se reintenta rigidez no uniforme, parametrizarla por launch/config y
    barrer valores en la misma ejecución de prueba o hacerla dependiente de
    curvatura/errores locales, no solo de distancia normalizada entre fiduciales.

## 2026-07-12 — Subfase 1L — dump del grafo y nodo offline

- objetivo intentado:
  - dejar documentada la hipótesis nueva: en zonas de mala textura la pose raw
    puede fallar también en distancia relativa, no solo en ángulos;
  - preparar una ruta para guardar el grafo exacto a optimizar y probar cambios
    del solver offline sin repetir la simulación larga;
  - ejecutar una simulación para generar el dump;
  - comprobar el nodo offline una vez sin modificar `orbslam3_multi` después de
    esa comprobación.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_problem_io.hpp`;
  - `orbslam3_multi/src/pose_graph_problem_io.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`;
  - `orbslam3_multi/CMakeLists.txt`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `simulacion_dron/launch/multi_dron.launch.py`;
  - documentación de subfase, estado, paquetes e historial.
- cambios técnicos:
  - `OptimizationManager` expone `RunDryRunGraphOnly` para ejecutar el solver
    sin `RawMapDatabase` ni `GlobalPoseStore`;
  - `pose_graph_problem_io` guarda/carga `PoseGraphProblem` en TSV versionado
    `F1L_POSE_GRAPH_DUMP\t1`;
  - `test_opt_graph_offline` carga el TSV y emite `[F1L-OFFLINE-LOAD]`,
    `[F1L-OFFLINE-DRYRUN]` y `[F1L-OFFLINE-METRICS]`;
  - `global_map_server` guarda `f1l_graph_task_<task_id>.tsv` si
    `f1l_graph_dump_enabled=true`;
  - `global_orb_map_server.launch.py` y `multi_dron.launch.py` exponen
    `f1l_graph_dump_enabled` y `f1l_graph_dump_output_dir`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server simulacion_dron
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - `simulacion_dron`: `BUILD-EXIT-CODE 0`;
  - solo quedaron warnings no bloqueantes ya conocidos en `optimization_manager.cpp`.
- prueba Gazebo:
  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0 f1l_debug_animation_enabled:=true f1l_graph_dump_enabled:=true f1l_graph_dump_output_dir:=src/codex/archivos_auxiliares" --post-scenario-wait-sec 20 --startup-wait-sec 20 --timeout-sec 900 --max-gazebo-retries 1
  ```
- resultado de simulación:
  - `SCENARIO-RUNNER-DONE scenario='prueba_1l_dron_2_antihorario_gt_debug' success=true`;
  - `SIM-DONE prueba=1 success=true`;
  - `SIM-EXIT-CODE 0`;
  - `4` goals enviados a `/dron_2/AccionTrayectoria`, todos `success=true`.
- dump generado:
  ```text
  codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv
  ```
  con `vertices=25`, `edges=24`, `priors=25`, `propagation=79`.
- prueba offline:
  ```bash
  source /home/chenfu/Gazebo/install/setup.bash
  ros2 run orbslam3_multi test_opt_graph_offline --graph /home/chenfu/Gazebo/src/codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv
  ```
- resultado offline:
  - carga OK;
  - dry-run OK;
  - `before_t=13.4123`;
  - `after_t=0.0143111`;
  - `improvement_ratio=0.998933`;
  - `iterations=35`;
  - `proposed_vertices=25`;
  - `proposed_propagated=0`, esperado porque la ruta offline no tiene
    `GlobalPoseStore`.
- patrones de reducción:
  ```text
  SCENARIO-RUNNER|F1L-GRAPH-DUMP|F1L-OFFLINE|F1J-OPT-GRAPH-RX|F1J-DRYRUN|F1K-APPLY|F1L-POST|F1L-GT|F1L-DECISION|success=true|ERROR|FATAL|Traceback|Exception
  ```
- evidencia live:
  - `[F1L-GRAPH-DUMP] ... success=true path=src/codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`;
  - `[F1L-GT-WINDOW-STATS] mean_before=5.739593 mean_after=4.676422 max_after=13.113617 worsened_kfs=4`;
  - `[F1L-POST-APPLY-ERROR] before_t=13.412348 real_after_t=0.014311 improvement_ratio=0.998933`;
  - `[F1L-POST-APPLY-INTERNAL-ERROR] ok=true internal_max_after=0.010045 num_edges_broken=0`;
  - `[F1L-POST-APPLY-PROPAGATION-CHECK] ok=false propagation_discontinuity_max_t=12.640211530`;
  - `[F1L-POST-APPLY-REJECT] reason=propagation_discontinuity decision=REJECT_ROLLBACK`.
- evidencia negativa o pendiente:
  - la infraestructura offline funciona, pero no valida apply/rollback ni KFs
    propagados;
  - la aplicación live vuelve a rechazar por discontinuidad de propagación;
  - la ventana GT completa sigue mal para cerrar `1L`.
- conclusión:
  - `PARCIAL`;
  - el nuevo nodo offline queda validado, pero la subfase `1L` no queda
    conseguida.
- siguiente paso recomendado:
  - usar `f1l_graph_task_1.tsv` para iterar offline pesos/solver;
  - probar relajación traslacional gradual en aristas con giro relativo alto,
    manteniendo rígidos ambos extremos fiduciales y sin usar GT como entrada.

## 2026-07-12 — Subfase 1L — soporte de aristas y HTML offline

- objetivo intentado:
  - probar la idea de dar más rigidez a aristas con más soporte de KeyFrames y
    menos rigidez a tramos con pocos KFs;
  - conservar la relajación traslacional en aristas con giro relativo alto;
  - ampliar el dump para incluir pose mapa y GT debug por KF de ventana;
  - generar HTML offline desde `test_opt_graph_offline`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/pose_graph_problem_io.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - documentación de subfase, estado, paquetes e historial.
- cambios técnicos:
  - `PoseGraphEdge` añade `intermediate_keyframe_count`,
    `support_keyframe_count`, `support_length_m`,
    `support_density_kfs_per_m` y `support_rigidity_multiplier`;
  - `PoseGraphBuilder` calcula densidad de KFs por metro y satura el
    multiplicador en `[0.65, 2.25]`;
  - `OptimizationManager` usa el peso de soporte y relaja la rigidez
    traslacional si la arista tiene giro relativo alto;
  - `PoseGraphProblem` añade `debug_keyframes`, solo para dumps offline;
  - `global_map_server` rellena `debug_keyframes` con pose mapa y GT si existe;
  - `test_opt_graph_offline` acepta `--html`, calcula propagación diagnóstica
    de KFs no vértice y emite métricas GT offline.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server simulacion_dron
  ```
- resultado de build:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - `simulacion_dron`: `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes conocidos en `optimization_manager.cpp` y uno nuevo
    no bloqueante por helper no usado en `test_opt_graph_offline.cpp`.
- prueba Gazebo:
  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0 f1l_debug_animation_enabled:=true f1l_graph_dump_enabled:=true f1l_graph_dump_output_dir:=src/codex/archivos_auxiliares" --post-scenario-wait-sec 20 --startup-wait-sec 20 --timeout-sec 900 --max-gazebo-retries 1
  ```
- resultado de simulación:
  - `SCENARIO-RUNNER-DONE scenario='prueba_1l_dron_2_antihorario_gt_debug' success=true`;
  - `SIM-DONE prueba=1 success=true`;
  - `SIM-EXIT-CODE 0`;
  - goals de `/dron_2/AccionTrayectoria` completados con `success=true`.
- dumps generados:
  - `codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`: caso pequeño,
    `debug_kfs=38`, `gt_debug_kfs=38`;
  - `codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv`: caso grande,
    `vertices=27`, `edges=26`, `propagation=103`, `debug_kfs=130`,
    `gt_debug_kfs=130`.
- evidencia live del caso grande `task_id=2`:
  - dry-run: `before_t=25.695117`, `after_t=0.223273`,
    `improvement_ratio=0.991311`;
  - GT ventana: `mean_before=7.593995`, `mean_after=6.621439`,
    `max_after=21.582332`, `worsened_kfs=3`;
  - aristas internas: `internal_max_after=0.664859`, `num_edges_broken=0`;
  - propagación: `propagation_discontinuity_max_t=18.136490008`;
  - decisión: `REJECT_ROLLBACK reason=propagation_discontinuity`.
- prueba offline:
  ```bash
  ros2 run orbslam3_multi test_opt_graph_offline \
    --graph /home/chenfu/Gazebo/src/codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv \
    --html /home/chenfu/Gazebo/src/codex/archivos_auxiliares/html/f1l_offline_graph_task_2.html
  ```
- resultado offline `task_id=2`:
  - soporte de aristas: `mean_multiplier=1.13084`, `min_multiplier=0.65`,
    `max_multiplier=2.25`;
  - dry-run: `before_t=25.6951`, `after_t=0.223273`,
    `improvement_ratio=0.991311`;
  - GT offline con propagación diagnóstica: `mean_before=7.59399`,
    `mean_after=3.73459`, `max_before=25.6951`, `max_after=10.0867`,
    `worsened_kfs=39`;
  - HTML generado: `codex/archivos_auxiliares/html/f1l_offline_graph_task_2.html`;
  - validación estructural HTML: `window_kfs=130`, `vertices=27`,
    `gt_valid=130`, `optimized_or_propagated=130`.
- patrones de reducción:
  ```text
  SCENARIO-RUNNER|F1L-GRAPH-DUMP|F1L-GRAPH-EDGE-SUPPORT|F1J-OPT-GRAPH-RX|F1J-OPT-DRYRUN|F1J-OPT-SOLVER|F1L-DEBUG-ANIMATION|F1L-GT-WINDOW-STATS|F1L-POST-APPLY|success=true|ERROR|FATAL|Traceback|Exception
  ```
- evidencia negativa o pendiente:
  - no se pudo crear captura headless del HTML porque Chrome/Firefox snap
    intentaron escribir en rutas de solo lectura; se validó el HTML por
    estructura/contenido;
  - el caso grande sigue fallando en live por discontinuidad de propagación;
  - el HTML offline muestra mejora fuerte de media GT, pero aún deja
    `max_after=10.0867` y muchos KFs empeorados.
- conclusión:
  - `PARCIAL`;
  - soporte por KFs, dump GT y HTML offline quedan implementados y validados;
  - `1L` sigue abierta por propagación/ventana completa.
- siguiente paso recomendado:
  - inspeccionar `f1l_offline_graph_task_2.html`;
  - ajustar la propagación de KFs no vértice y/o pesos para bajar `max_after` y
    reducir `worsened_kfs` antes de repetir Gazebo.

## 2026-07-12 23:06 — Subfase 1L — rigidez angular por esquina y soporte

- objetivo intentado:
  - probar que la deformacion angular se concentre en aristas centrales/poco
    fiables como `127->129`, y no en aristas soportadas/cercanas al fiducial
    objetivo como `150->161`.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`;
  - documentación de `orbslam3_multi`, `OptimizationManager`, historial e
    ultima sesión.
- cambio técnico:
  - se añadió una regularizacion del campo de correccion;
  - la traslacion se suaviza por soporte de KFs, sin perfil parabolico fuerte;
  - la parte angular combina soporte de KFs, rigidez extra cerca de fiduciales,
    giro relativo bruto y vertices `corner_yaw_vertex`;
  - `test_opt_graph_offline` emite `[F1L-OFFLINE-EDGE-DEFORMATION]` por arista.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos en `optimization_manager.cpp`.
- pruebas Gazebo/replay:
  - no se repitio Gazebo;
  - se uso el dump existente `codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv`;
  - comando offline:
    ```bash
    /home/chenfu/Gazebo/install/orbslam3_multi/lib/orbslam3_multi/test_opt_graph_offline \
      --graph /home/chenfu/Gazebo/src/codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv \
      --html /home/chenfu/Gazebo/src/codex/archivos_auxiliares/html/f1l_offline_graph_task_2_gradient.html
    ```
- patrones de reducción:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - replay offline OK: `success=true`, `iterations=11`;
  - objetivo fiducial: `before_t=25.6951`, `after_t=0.0277597`,
    `improvement_ratio=0.99892`;
  - arista central `127->129`: `delta_len=0.032908`,
    `delta_rel_yaw=0.949977`;
  - arista soportada/cercana a fiducial objetivo `150->161`:
    `delta_len=-0.00767068`, `delta_rel_yaw=0.0153493`;
  - HTML generado:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_gradient.html`.
- evidencia negativa o ausente:
  - GT offline diagnostico no supera la variante uniforme anterior:
    `mean_after=5.13446` frente a `3.73459` anterior;
  - `worsened_kfs=63`;
  - no valida apply real, rollback ni propagacion live.
- conclusión:
  - `PARCIAL`;
  - el reparto local de deformacion angular mejora, pero no se puede cerrar
    `1L` con esta evidencia.
- siguiente paso recomendado:
  - comparar visualmente `f1l_offline_graph_task_2_gradient.html`;
  - ajustar la propagacion de KFs no vertices y/o reequilibrar el prior
    fiducial para recuperar GT medio sin volver a concentrar giro en `150->161`;
  - solo repetir Gazebo cuando el replay offline mejore tambien `mean_after` y
    `worsened_kfs`.

## 2026-07-12 23:20 — Subfase 1L — bisagra XY para esquinas fiables

- objetivo intentado:
  - evitar que una esquina fiable cercana al fiducial objetivo invierta el
    sentido de giro tras optimizar, en particular `143->150->161`.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - documentación de paquete, historial e ultima sesión.
- cambios técnicos:
  - `OptimizationManager` añade un coste de bisagra sobre triples consecutivos
    `A-B-C` que preserva el giro firmado XY;
  - el peso de bisagra aumenta cerca de fiduciales y con soporte local alto;
  - `test_opt_graph_offline` emite
    `[F1L-OFFLINE-CORNER-DEFORMATION]` con `before_turn`, `after_turn`,
    `delta_turn` y `sign_flipped`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos en `optimization_manager.cpp`.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline final:
    ```bash
    /home/chenfu/Gazebo/install/orbslam3_multi/lib/orbslam3_multi/test_opt_graph_offline \
      --graph /home/chenfu/Gazebo/src/codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv \
      --html /home/chenfu/Gazebo/src/codex/archivos_auxiliares/html/f1l_offline_graph_task_2_hinge.html
    ```
- patrones usados para reducir logs:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - dry-run OK: `success=true`, `iterations=18`;
  - fiducial objetivo: `before_t=25.6951`, `after_t=0.0324856`,
    `improvement_ratio=0.998736`;
  - esquina `143->150->161`: `before_turn=1.42674`,
    `after_turn=1.42488`, `delta_turn=-0.00186582`,
    `sign_flipped=false`;
  - arista `150->161`: `delta_len=-0.0365503`,
    `delta_rel_yaw=-0.00700344`;
  - HTML generado:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_hinge.html`.
- evidencia negativa o ausente:
  - GT offline diagnóstico: `mean_after=5.47334`, `max_after=11.8263`,
    `worsened_kfs=59`;
  - sigue peor que la variante uniforme anterior en GT medio;
  - no valida apply real, rollback ni propagación live.
- conclusión:
  - `PARCIAL`;
  - la inversión de esquina fiable queda corregida localmente, pero no cierra
    `1L` por calidad global de ventana.
- siguiente paso recomendado:
  - mantener la bisagra para proteger esquinas fiables;
  - revisar la propagación/poses de KFs no vértice y el equilibrio con el prior
    fiducial para bajar `mean_after` y `worsened_kfs`;
  - no repetir Gazebo hasta que el replay offline supere también la métrica GT
    global.

## 2026-07-12 23:54 — Subfase 1L — métricas globales y arista pobre 127->129

- objetivo intentado:
  - corregir la interpretación de `before_rel_z` y permitir que la arista pobre
    `127->129` se ensanche cuando el mapa raw la deja demasiado corta.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/INDEX.md`.
- cambios técnicos:
  - `test_opt_graph_offline` añade métricas globales por arista:
    `before_map_xy_len`, `after_map_xy_len`, `before_map_delta_z`,
    `after_map_delta_z` y `gt_map_*` solo diagnóstico;
  - se confirma que `before_rel_z=-1.802` en `127->129` era componente local de
    `T_127^-1*T_129`, no altura global RViz2;
  - `OptimizationManager` añade relajación traslacional selectiva para aristas
    con poco soporte, zona central y proyección XY local/global anómala;
  - se probó y descartó usar coste de posición global completo: redujo la
    inversión en `161`, pero empeoró la ventana (`mean_after=5.87733`).
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos en `optimization_manager.cpp`.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline final:
    ```bash
    /home/chenfu/Gazebo/install/orbslam3_multi/lib/orbslam3_multi/test_opt_graph_offline \
      --graph /home/chenfu/Gazebo/src/codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv \
      --html /home/chenfu/Gazebo/src/codex/archivos_auxiliares/html/f1l_offline_graph_task_2_local_solver_sparse_projection.html
    ```
- patrones usados para reducir logs:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-EDGE-SUPPORT-EDGE|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - dry-run OK: `success=true`, `iterations=9`;
  - fiducial objetivo: `before_t=25.6951`, `after_t=0.0406869`;
  - `127->129` se clasifica con `sparse_translation_rigidity=0.238901`;
  - `127->129`: `before_map_xy_len=1.81843`,
    `after_map_xy_len=3.07182`, `gt_map_xy_len=4.90851`;
  - `127->129`: `before_map_delta_z=0.36023`,
    `after_map_delta_z=0.450046`; el gran `rel_z` era local;
  - HTML generado:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_local_solver_sparse_projection.html`.
- evidencia negativa o ausente:
  - GT offline diagnóstico: `mean_after=4.82182`, `max_after=10.2946`,
    `worsened_kfs=62`;
  - sigue peor que la mejor variante previa (`mean_after=4.43082` con
    split XY/Z local);
  - aparecen inversiones de esquina: `center_kf=150 sign_flipped=true` y
    `center_kf=161 sign_flipped=true`;
  - no valida apply real, rollback ni propagación live.
- conclusión:
  - `PARCIAL`;
  - el diagnóstico global y la válvula de arista pobre funcionan para
    `127->129`, pero el equilibrio solver/bisagra aún no es aceptable.
- siguiente paso recomendado:
  - mantener los logs `map_*`;
  - reequilibrar la relajación de arista pobre con una protección más fuerte de
    esquinas fiables cercanas a fiduciales, especialmente `150/161`;
  - no repetir Gazebo hasta que el replay offline mejore `mean_after` y no
    invierta esas bisagras.

## 2026-07-13 00:03 — Subfase 1L — control de Z global para no cerrar kf143

- objetivo intentado:
  - entender por qué las aristas alrededor de `kf=143` (`139->143` y
    `143->150`) formaban un ángulo cerrado aunque inicialmente eran casi rectas
    y con soporte razonable;
  - respetar la petición de generar HTMLs de replay offline como serie numerada
    `..._1.html`, `..._2.html`, `..._3.html`, `..._4.html`, sustituyendo los
    anteriores cuando se repita el mismo experimento.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md`.
- cambios técnicos:
  - se confirma que el cierre de `kf143` no venía de resolver en 2D: el solver
    seguía usando poses 6D, pero al preservar residuales relativos locales podía
    aceptar cambios de Z global que cerraban la proyección XY;
  - se añade residual de Z global por arista (`kEdgeGlobalVerticalResidualScale`)
    tanto en el vector de residuales como en `ComputeCost`;
  - se mantiene `kCornerHingeStraightMultiplier=0.25`;
  - se limita la relajación máxima de aristas pobres a
    `kSparseEdgeTranslationMaxRelax=0.55`;
  - se descarta reforzar más la bisagra recta fiable porque empeoró el GT medio
    y no resolvió `kf143`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos en `optimization_manager.cpp`.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - se ejecutaron cuatro replays offline sobre:
    `codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv`;
  - variante activa final:
    ```bash
    /home/chenfu/Gazebo/install/orbslam3_multi/lib/orbslam3_multi/test_opt_graph_offline \
      --graph /home/chenfu/Gazebo/src/codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv \
      --html /home/chenfu/Gazebo/src/codex/archivos_auxiliares/html/f1l_offline_graph_task_2_reliable_hinge_4.html
    ```
- patrones usados para reducir logs:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-EDGE-SUPPORT-EDGE|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - dry-run OK: `success=true`, `iterations=15`;
  - fiducial objetivo: `before_t=25.6951`, `after_t=0.0297886`,
    `improvement_ratio=0.998841`;
  - GT offline diagnóstico: `mean_after=3.28931`, `max_after=5.84293`;
  - `kf143`: `before_turn=-0.0106539`, `after_turn=-0.454941`,
    `sign_flipped=false`; mejora frente a la variante con relajación pobre
    anterior (`after_turn=-0.821971`);
  - `kf161`: `before_turn=0.227628`, `after_turn=0.134395`,
    `sign_flipped=false`;
  - aristas alrededor de `kf143` mantienen Z global razonable:
    `139->143 after_map_delta_z=0.302172`,
    `143->150 after_map_delta_z=0.179147`;
  - HTML final:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_reliable_hinge_4.html`.
- evidencia negativa o ausente:
  - `1L` no queda cerrada porque no se validó Gazebo/apply/rollback live;
  - `worsened_kfs=58`;
  - la arista pobre `127->129` ya no se ensancha hacia el GT diagnóstico en la
    variante final: `before_map_xy_len=1.81843`,
    `after_map_xy_len=1.53656`, `gt_map_xy_len=4.90851`;
  - los HTML conservados de esta serie son `_2`, `_3` y `_4`; para nuevas
    tandas se debe iniciar la serie numerada desde `_1` y sobrescribir artefactos
    previos del mismo experimento.
- conclusión:
  - `PARCIAL`;
  - el problema de cierre espurio en `kf143` se reduce bastante y la inversión
    de `kf161` queda corregida en replay offline, pero falta validación live y
    aún no se resuelve bien la zona pobre `127->129`.
- siguiente paso recomendado:
  - mantener el residual de Z global y la relajación pobre limitada;
  - estudiar una forma local y controlada de permitir expansión XY en
    `127->129` sin reintroducir cierres/inversiones en `143`, `150` o `161`;
  - repetir primero replay offline numerado; solo volver a Gazebo si el replay
    mejora `mean_after`, no invierte bisagras fiables y no empeora `127->129`.

