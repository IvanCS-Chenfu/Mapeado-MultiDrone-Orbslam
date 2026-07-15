# 01 — Estado actual

Versión corta para lectura inicial:

```text
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
```

Este archivo conserva el detalle histórico/técnico. No abrirlo completo por
defecto si el resumen basta.

## Estado global del proyecto

El proyecto busca construir una nube densa global de un entorno definido por YAML usando varios drones.

La fase activa es la primera gran fase: construir un mapa sparse global multi-dron coherente, estable, anclado, fusionado y útil para estimar poses globales y alimentar la futura reconstrucción densa.

## Fase activa

```text
Fase 1 — Mapa sparse global multi-dron
Planificación activa — subfases 1A a 1W
Subfase 1A — baseline conseguida el 2026-07-03
Subfase 1B — conseguida el 2026-07-06 tras validar build, simulación, logs y comentarios/trazabilidad por subfase
Subfase 1C — conseguida el 2026-07-06 tras validar RawMapDatabase, grabación raw y replay completo
Subfase 1D — conseguida el 2026-07-08 tras validar GlobalPoseStore con replay, anchor/corrección sintéticos y herencia de corrección
Subfase 1E — conseguida el 2026-07-08 tras validar FiducialAnchorManager, anclaje inicial por fiducial simulado, record v2 y replay fiducial
Subfase 1F — conseguida el 2026-07-08 tras validar LandmarkScoreManager, GlobalMapBuilder, publicación PointCloud2 en /global_sparse_cloud y hotfix body_T_camera
Subfase 1G — conseguida el 2026-07-08 tras validar full snapshots, reconciliación raw/global, record con snapshots y replay
Subfase 1H — conseguida el 2026-07-09 tras validar revisits fiduciales live/replay, medición de error absoluto y hotfix de publicación cacheada
Subfase 1I — conseguida de nuevo el 2026-07-10 tras preservar anclajes fiduciales previos dentro del grafo
Subfase 1J — conseguida de nuevo el 2026-07-10 tras revalidar dry-run con preservacion de anclajes previos
Subfase 1K — conseguida de nuevo el 2026-07-11 tras revalidar apply con preservacion de anclajes previos
Subfase 1L — PARCIAL el 2026-07-12; se implemento un solver minimo propio de pose graph en `OptimizationManager` y compila, pero RViz2 siguio mostrando que el dron antihorario no corrige bien al llegar al fiducial 1. El log de la prueba larga muestra casos como `28.738164 -> 0.017052 m` rechazados por `internal_edges_broken`, y casos posteriores de `~2.4 m -> ~0.004 m` rechazados por crecimiento interno. Tras el analisis, las aristas internas deben conservar la distancia relativa entre KFs; la yaw relativa debe poder variar bastante para deshacer esquinas falsas de ORB-SLAM3, y roll/pitch deben existir como correccion menor. En la ultima ejecucion completa se implemento GT debug por KeyFrame en `global_map_server`, activable con `f1l_gt_kf_debug_enabled:=true`, solo como metrica/log de simulacion. La prueba corta de `dron_2` antihorario genero el caso buscado: `error_t=19.776323 m` en fiducial 1, `PARTIAL_KEEP_FOR_NEXT_PASS` tras bajar a `0.013640 m`, y segunda pasada `ACCEPT` con `real_after_t=0.000019 m`. Aun asi, las metricas `[F1L-GT-WINDOW-STATS]` muestran que la ventana completa sigue mal: `mean_after` queda alrededor de `4.27 m` y `max_after` alrededor de `10.76 m`, con tareas posteriores que empeoran muchos KFs. Tras inspeccion RViz2, el usuario observa KFs separados tanto cerca del fiducial 1 como del fiducial 2; se sospecha fallo combinado de construccion del grafo y propagacion de KFs no vertices. Se anadio una animacion HTML opt-in por tarea (`f1l_debug_animation_enabled:=true`) para comparar mapa inicial, GT, vertices del grafo y propuesta optimizada. En una ejecucion del 2026-07-12 se reforzo `PoseGraphBuilder` con vertices por distancia acumulada, esquinas, vecindades fiduciales, limites de gap/longitud y `PropagationPlan` con `segment_alpha`; ademas `OptimizationManager` evita que un checkpoint parcial publique propagacion amplia o deje `last_correction_set=true`. La prueba aislada `dron_2` antihorario termino con build y simulacion OK, genero HTMLs `f1l_debug_animation_task_1..8.html` y mostro el guard funcionando (`propagated_kfs=0`, `skipped_propagated_kfs>0`, `last_correction_set=false`), pero todos los casos grandes del fiducial 1 se rechazaron como `deformable_edges_broken_without_safe_partial` aunque bajaban el residual fiducial de `26-29 m` a `0.03-0.04 m`; `[F1L-GT-WINDOW-STATS]` siguio con medias `8-10 m` y maximos `23-28 m`. En la ejecucion posterior se anadio `PoseGraphCoverageSummary`: el builder puede superar adaptativamente `pose_graph_max_vertices` nominal y rechaza antes del solver si quedan tramos largos sin control. La prueba `dron_2` antihorario volvio a terminar con build/simulacion OK; el caso grande bajo el fiducial objetivo de `23.602545 m` a `3.602548 m`, pero la ventana quedo en `mean_after=8.651571` y `max_after=22.521755`. Una prueba posterior con semilla elastica y traslacion relativa local llevo el fiducial 1 de `23.445255 m` a `0.002936 m`, pero fue rechazada por `deformable_edges_broken_without_safe_partial` y propagacion incoherente. La prueba con solver `x,y,z,roll,pitch,yaw`, preservacion fuerte de longitud/distancia relativa, direccion local debil y yaw casi libre fue la mejor: candidato aceptado con fiducial 1 `9.792646 m -> 0.009804 m`, ventana GT `mean_before=4.748133 -> mean_after=1.762549`, `max_before=10.032235 -> max_after=4.030853`, `internal_max_after=0.027445`, sin aristas rotas ni discontinuidad de propagacion. Despues se probo rigidez angular parabólica/U por posicion de arista, en variantes fuerte y suave; ambas empeoraron la ventana y fueron rechazadas por discontinuidad de propagacion. El scaffold queda en codigo con multiplicador neutro, equivalente al solver 6D uniforme. La iteracion offline posterior añade `f1l_graph_dump_enabled`, serializacion TSV de `PoseGraphProblem`, soporte de KFs por arista, `debug_kf` con pose mapa/GT y HTML desde `test_opt_graph_offline --html`. El dump grande actual `f1l_graph_task_2.tsv` reproduce offline `25.6951 m -> 0.2233 m`, con GT diagnostico `mean_after=3.7346`, `max_after=10.0867` y HTML `f1l_offline_graph_task_2.html`; el apply live aun se rechaza por `propagation_discontinuity`. `1L` queda `PARCIAL` y se volverá a comprobar en el futuro, sin absorber la deuda de covisibilidad/loops.
Subfase 1M — actual/sin hacer; `CovisibilityDatabase` confirmada para importar covisibilidad ORB-SLAM3, registrar loops geométricos confirmados y alimentar optimizaciones posteriores.
Subfase 1N — sin hacer; `LoopDetector` BoW. No iniciar `1N+` hasta completar `1M`, porque BoW debe consultar la base confirmada.
```

Las subfases antiguas `12R-D4`, `12R-D5`, `12R-E`, `12R-F`, `13`, `14` y `15` quedan como legacy/solo referencia. Contienen aprendizaje útil sobre el servidor monolítico anterior, pero no son la planificación activa.

Actualización 2026-07-14 de `1J/1K/1L`: el solver/HTML de KeyFrames puede ser bueno
aunque RViz2 muestre una nube arrastrada si `GlobalMapBuilder` publica
MapPoints mediante una corrección genérica de submapa. La ruta activa ahora
elige por MapPoint un KF publicable (`reference_keyframe_id` o primer observador
con pose en `GlobalPoseStore`), calcula `p_kf = raw_local_T_kf^-1 * p_raw` y
publica `p_world = final_world_T_kf * p_kf`. La prueba live `prueba_1` terminó
con `SIM-EXIT-CODE 0`, `F1L-POST-APPLY-ACCEPT`, fiducial objetivo
`11.640776 m -> 0`, propagación `ok=true` con `82` KFs y
`server_corrected_after=11359/19612`, no todos los puntos publicados. `1L`
sigue `PARCIAL` hasta confirmar visualmente RViz2.

Actualización posterior del mismo día: los KFs posteriores al apply ya no deben
volver a la corrección vieja del submapa. `OptimizationManager` marca como
herencia futura el KF aplicado con mayor `local_kf_id`, y `GlobalPoseStore`
guarda ese `from_keyframe` para que full snapshots de KFs anteriores no pisen
la corrección heredable. Ajuste final: `GlobalPoseStore` mantiene los KFs
posteriores como server-controlled frente a full snapshots, y
`GlobalMapBuilder` salta MapPoints sin KF publicable si el submapa ya tiene
corrección de servidor, evitando fallback rígido `world_T_local`. Validación
final: `task_id=2`, `from_kf=200`, `last_correction_set=true`, KFs `202..208`
con `SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT`, `fallback_submap_after=0`,
`SIM-EXIT-CODE 0`.

Actualización documental posterior: queda fijada la propiedad conceptual de las
subfases. `1J` es dry-run, dump/replay offline y HTML diagnóstico del grafo;
`1K` es apply en `GlobalPoseStore`, propagación, publicación y herencia de KFs
futuros; `1L` queda como diagnóstico post-apply con logs/RViz2/GT debug. Los
prefijos `f1l_*` en algunos parámetros/archivos son legacy y no cambian esa
propiedad.

## Qué ya existe

### Simulación y drones

Existe simulación Gazebo multi-dron con control de movimiento, cámaras estéreo simuladas, GT de Gazebo, launch multi-dron y escenario automático mediante YAMLs en `codex/archivos_auxiliares/trayectorias/tray_prueba_X.yaml`.

El launch oficial para pruebas automáticas es:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

### Wrapper ROS 2 de ORB-SLAM3

`orbslam3_ros2` ejecuta ORB-SLAM3 estéreo por dron, publica pose local, publica deltas `OrbMap` y ofrece `GetOrbMap` para snapshots completos.

Publica:

```text
orbslam/pose_local
orbslam/orb_map_delta
```

Ofrece:

```text
orbslam/get_full_map
```

El wrapper detecta `map_epoch`. La unidad correcta del sistema es:

```text
submapa = (drone_id, map_epoch)
```

El wrapper se considera estable y no debe tocarse salvo necesidad fuerte.

### Mensajes ROS

`orbslam3_msgs` contiene mensajes suficientes para exportar datos ORB-SLAM3 al servidor:

- `OrbMap`
- `OrbMapPoint`
- `OrbKeyFrame`
- `OrbKeyPoint`
- `OrbObservation`
- `OrbDescriptor`
- `MapCorrection`
- `CorrectedKeyFrame`
- `CorrectedKeyFrameArray`
- `FiducialLockStatus`
- `FiducialObservation`
- servicio `GetOrbMap`

El paquete se considera estable. No añadir score global al wrapper ni a los mensajes salvo fase explícita.

### Backend global actual

`orbslam3_multi` ya contiene las dos primeras piezas activas del backend nuevo:

- `RawMapDatabase`
- `GlobalPoseStore`

Las piezas antiguas `GlobalAtlas`, `GlobalKeyFrameDatabase`, `GlobalORBMatcher`, `GlobalGeometryVerifier`, `GlobalPoseGraphOptimizer`, `MultiDroneSystem`, `ImportedKeyFrame`, `ImportedMapPoint` y `GlobalPoseGraphTypes` se conservan solo en `orbslam3_multi/legacy/` con sufijo `_antiguo`. Ya no existen en `orbslam3_multi/src/` ni en `orbslam3_multi/include/`, ni se compilan dentro de la librería activa.

La nueva Fase 1 exige que este paquete evolucione hacia el backend real con clases pequeñas: `FiducialAnchorManager`, `LoopDetector`, `SubcloudLoopVerifier`, `LoopDecisionManager`, `FusionManager`, `FusedLandmarkManager`, `LandmarkScoreManager`, `PoseGraphBuilder`, `OptimizationManager` y `GlobalMapBuilder`.

### Servidor global actual

Antes de `1B`, `orbslam3_server/src/global_map_server.cpp` contenía mucha lógica acumulada: recepción de `OrbMap`, mirrors, snapshots, fiduciales simulados, anchors, BoW, subcloud matching, fused tracks, score, optimización local, apply, publicaciones y rutas legacy.

Desde `1B`, esa lógica acumulada queda en `orbslam3_server/src/global_map_server_antiguo.cpp` como referencia no activa. El archivo activo `orbslam3_server/src/global_map_server.cpp` es el servidor mínimo de recepción `OrbMap` con logs `[F1B-*]` y, desde `1C`, conecta esos deltas con `orbslam3_multi::RawMapDatabase`.

La nueva planificación no busca arreglar indefinidamente ese monolito. La ruta activa es:

1. capturar baseline del servidor actual en 1A;
2. crear servidor nuevo mínimo en 1B;
3. migrar responsabilidades de forma incremental hacia `orbslam3_multi`;
4. conservar legacy solo como referencia hasta que cada reemplazo compile y se valide.

Estado actual tras `1C`:

- `orbslam3_server/src/global_map_server.cpp` recibe `OrbMap` delta, asigna `arrival_id`, inserta en `RawMapDatabase`, guarda datasets raw y permite replay por timer;
- `orbslam3_server/src/global_map_server_antiguo.cpp` conserva el servidor monolítico anterior como referencia no compilada;
- `orbslam3_server/launch/global_orb_map_server.launch.py` lanza el servidor mínimo con parámetros `rawdb_record_*` y `rawdb_replay_*`;
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`, `raw_map_types.hpp` y `src/raw_map_database.cpp` contienen la base raw validada por submapa `(drone_id, map_epoch)` y journal;
- `orbslam3_multi/legacy/` contiene copias congeladas no compiladas de los módulos antiguos de backend, y la ruta activa de `orbslam3_multi` ya no instala esos headers antiguos.

Estado actual tras `1D`:

- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp` y `orbslam3_multi/src/global_pose_store.cpp` contienen `GlobalPoseStore`;
- `GlobalPoseStore` empieza vacío y no crea poses globales sin anchor;
- `GlobalPoseStore` guarda anchors, poses `world_T_kf`, poses optimizadas registradas y última corrección heredable por submapa;
- `RawMapDatabase` mantiene poses locales raw intactas y añade solo getters constantes `GetKeyFrame`/`GetMapPoint`;
- `orbslam3_server/src/global_map_server.cpp` puede activar un modo debug `pose_store_debug_*` para validar replay con anchor y corrección sintéticos;
- el modo debug de `1D` no es anclaje real y no debe confundirse con fiduciales; el primer anclaje real queda para `1E`.

Estado actual tras `1E`:

- `orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp` y `orbslam3_multi/src/fiducial_anchor_manager.cpp` contienen `FiducialAnchorManager`;
- `FiducialAnchorManager` no lee ROS ni Gazebo: recibe observaciones fiduciales ya asociadas a KeyFrames, consulta `RawMapDatabase`, calcula `world_T_local` y ancla submapas en `GlobalPoseStore`;
- `GlobalPoseStore` marca KeyFrames fiduciales reales como hard fiducials para que fases posteriores no los traten como KFs móviles normales;
- `RawMapDatabase` guarda `.record` versión 2 con journal raw y journal de observaciones fiduciales asociadas a `arrival_id`;
- `orbslam3_server/src/global_map_server.cpp` usa GT de Gazebo solo para simular la observación fiducial y asociarla temporalmente a KFs dentro del radio configurado;
- replay puede reproducir las observaciones fiduciales persistidas con `fiducial_sim_enabled:=false`, sin consultar GT en vivo.

Estado actual tras `1F`:

- `orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp` y `orbslam3_multi/src/landmark_score_manager.cpp` contienen `LandmarkScoreManager`;
- `LandmarkScoreManager` calcula score inicial acotado `[0,1]` desde datos raw de ORB-SLAM3: `observations_count`, `found_ratio`, `is_bad` y descriptor válido;
- `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp` y `orbslam3_multi/src/global_map_builder.cpp` contienen `GlobalMapBuilder`;
- `GlobalMapBuilder` consulta `RawMapDatabase`, `GlobalPoseStore` y `LandmarkScoreManager`, transforma MapPoints locales de submapas anclados a `world` y salta submapas no anclados;
- `GlobalPoseStore` contiene la extrínseca configurable `body_T_camera` y `FiducialAnchorManager` la aplica para convertir `world_T_body` en `world_T_camera` antes de calcular `world_T_local`;
- `orbslam3_server/src/global_map_server.cpp` publica `sensor_msgs/msg/PointCloud2` en `/global_sparse_cloud`, frame `world`, con campos `x`, `y`, `z`, `score`, `drone_id` y `map_epoch`;
- `orbslam3_server/launch/global_orb_map_server.launch.py` expone `body_T_camera_*` y `use_camera_optical_frame_convention`, replicando los valores del launch legacy para corregir la nube visualizada en RViz2;
- el dataset diferencial de anclaje de `1F` fue útil para validar visualmente `body_T_camera`, pero se borró al iniciar `1G` para liberar espacio y evitar conservar una prueba peor que el nuevo record con snapshots;
- en `1F` `is_fused=false` y todavía no hay fusión real de landmarks; la publicación es la primera salida sparse global anclada y puntuable para inspección en RViz2.

Estado actual tras `1G`:

- `orbslam3_server/src/global_map_server.cpp` crea clientes `orbslam3_msgs/srv/GetOrbMap` por dron y pide full snapshots al wrapper mediante `/dron_X/orbslam/get_full_map`;
- cada snapshot completo recibe un `arrival_id` monotónico igual que los deltas y se guarda en el journal de `RawMapDatabase`;
- `RawMapDatabase::InsertFullSnapshot` aplica una política conservadora `insert_update_no_absent_delete`: inserta/actualiza KFs y MPs recibidos, no borra ausentes todavía y detecta cambios de pose local raw en KFs existentes;
- `RawInsertResult` informa nuevos/actualizados/bad, cambios de pose raw y cambios grandes para logs y reconciliación;
- `GlobalPoseStore::ReconcileAfterRawIngestResult` recalcula poses globales derivadas de anchor para KFs no optimizados y conserva poses optimizadas del servidor, actualizando su corrección interna frente a la nueva pose raw;
- el replay reproduce entradas `FullSnapshot` con `[F1G-REPLAY-FULL-SNAPSHOT]` y ejecuta la misma reconciliación sin pedir servicios;
- se generó un nuevo `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` con `368` entradas, `356` deltas, `12` full snapshots, `74` observaciones fiduciales, `4` submapas, `225` KFs y `26165` MPs;
- no hay optimización real ni fusión real todavía; `1G` solo sincroniza snapshots y protege el estado global existente.

Estado actual tras `1H`:

- `orbslam3_multi/include/orbslam3_multi/fiducial_optimization_task.hpp` define `FiducialOptimizationTask`, una tarea ligera para conservar evidencia de residual fiducial alto sin ejecutar optimizacion;
- `FiducialAnchorManager` distingue primer anchor de revisit: en submapa no anclado conserva la ruta `1E`, y en submapa ya anclado mide residual contra la pose absoluta fiducial sin sobrescribir `world_T_local`;
- los revisits calculan `error_t_m`, `error_rot_rad` y `error_yaw_rad` y se clasifican con umbrales configurables;
- si el residual queda bajo umbral se emite `[F1H-FID-OK]`; si quedara alto, se crea/deduplica una `FiducialOptimizationTask` con `[F1H-FID-TASK-CREATED]`;
- `global_map_server` expone `fiducial_revisit_error_threshold_m`, `fiducial_revisit_yaw_threshold_rad` y `fiducial_revisit_rot_threshold_rad`;
- `/global_sparse_cloud` usa QoS `KeepLast(1)` y el timer publica solo la ultima nube cacheada con `[F1F-GLOBALMAP-REPUBLISH]`, evitando reconstrucciones de timer intercaladas;
- durante la validacion se detecto un parpadeo en RViz2 causado por un replay antiguo aun vivo: `ros2 topic info /global_sparse_cloud -v` mostro `Publisher count: 2`; tras cerrar ese proceso quedo `Publisher count: 1`;
- se genero un nuevo `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` de aproximadamente `586M`;
- no hay optimizacion real todavia; `1H` solo mide y prepara la entrada para `1I-1L`.

Estado actual tras `1I`:

- `orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp` define `PoseGraphProblem`, `PoseGraphVertex`, `PoseGraphEdge`, `PoseGraphPrior` y `PoseGraphPropagationPlanEntry`;
- `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp` y `orbslam3_multi/src/pose_graph_builder.cpp` contienen `PoseGraphBuilder`;
- `PoseGraphBuilder::BuildForFiducialTask` construye un grafo temporal desde una `FiducialOptimizationTask`, `RawMapDatabase` y `GlobalPoseStore`;
- el grafo se construye por submapa `(drone_id, map_epoch)`, usa poses globales ya existentes, respeta KFs hard fiducial como fijos y no fija KFs normales solo por estar anclados;
- el problema contiene vertices variables/fijos, aristas temporales, priors hard/target/soft, KFs afectados no variables y `PropagationPlan`;
- `orbslam3_server/src/global_map_server.cpp` invoca el builder cuando aparecen tareas pendientes de `FiducialAnchorManager` y emite logs `[F1I-*]`;
- `orbslam3_server/launch/global_orb_map_server.launch.py` expone parametros `pose_graph_*` y `f1i_debug_force_task_enabled`;
- la validacion live genero `109` tareas fiduciales y `109` grafos F1I correctos;
- la validacion replay genero `16` tareas fiduciales reales, `17` grafos F1I correctos y una tarea debug adicional;
- una revalidacion posterior del 2026-07-09 recompilo `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`, repitio las pruebas live/replay y regenero `prueba_1.reduced.log` y `prueba_2.reduced.log` con evidencia compacta: `15` grafos F1I en live y `17` grafos F1I en replay/debug;
- no se ejecuta solver, no se aplica optimizacion y no se modifican poses globales por el builder;
- reapertura 2026-07-10: una prueba larga posterior mostro que una tarea fiducial multi-anclaje podia construir un grafo con `fixed=0 hard_fiducial=0`; el caso `task_id=1` movio/propago KFs con `max_delta_t=23.886778` y visualmente parecio desanclar la zona del fiducial previo;
- cierre 2026-07-10: `PoseGraphBuilder` ya selecciona anclajes fiduciales previos del mismo submapa como frontera fija para tareas fiduciales multi-anclaje;
- las pruebas largas `prueba_1` y `prueba_2` validaron `[F1I-FID-CONNECTIVITY-BRANCHES] required=true satisfied=true previous_fiducial_fixed_count=1`, `[F1I-GRAPH-PREVIOUS-FIDUCIAL-ANCHOR] fixed=true hard_fiducial=true` y `[F1I-GRAPH-BUILD-SUMMARY] ... fixed=1 hard_fiducial=1`;
- `1I` queda conseguida de nuevo; la subfase actual pasa a `1J` para revalidar el dry-run sobre grafos con anclaje previo protegido.

Estado actual tras `1J`:

- `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp` define `OptimizationPoseProposal` y `OptimizationDryRunResult`;
- `orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp` y `orbslam3_multi/src/optimization_manager.cpp` contienen `OptimizationManager`;
- `OptimizationManager::RunDryRun` consume un `PoseGraphProblem`, `RawMapDatabase` y `GlobalPoseStore` de solo lectura, copia poses a memoria temporal, calcula coste before/after, propone poses de vertices y propagacion no variable, decide `useful` o `partial_candidate` y no aplica estado;
- `orbslam3_multi/include/orbslam3_multi/optimization_debug_exporter.hpp` y `orbslam3_multi/src/optimization_debug_exporter.cpp` contienen `OptimizationDebugExporter`;
- el exportador SVG es opcional: `f1j_export_debug_plot=false` por defecto, el servidor no lee CSV/SVG y el resultado normal viaja en memoria como `OptimizationDryRunResult`;
- `orbslam3_server/src/global_map_server.cpp` configura `OptimizationManager`, invoca el dry-run tras cada grafo `1I` y emite marcadores `[F1J-*]`;
- la revalidacion live previa produjo `107` dry-runs `success=true`: `83` `useful=true` y `24` `partial_candidate=true` con `reason=large_error_reduced_but_above_threshold`;
- la validacion replay previa produjo `17` dry-runs `success=true/useful=true`, incluyendo la tarea debug `task_id=9000000001`;
- la revalidacion posterior del 2026-07-10, tras reparar `1I`, produjo `24` dry-runs `success=true/useful=true` en `prueba_1` y `14` dry-runs `success=true/partial_candidate=true` en `prueba_2`;
- todos los dry-runs de la revalidacion muestran `[F1J-OPT-ANCHOR-PRESERVATION] ok=true required=true graph_satisfied=true previous_fiducial_fixed_count>=1 hard_fixed_moved=false max_previous_fiducial_delta_t=0`;
- la `prueba_2` reprodujo el caso fuerte con errores de fiducial `23-26 m`, reducidos a `2.09-2.31 m` y clasificados como `partial_candidate=true`, sin aplicar poses;
- no se aplican poses, no se actualiza `GlobalPoseStore`, no se modifica `RawMapDatabase`, no se publica `map_correction` ni `corrected_keyframes`;
- los casos con mejora grande pero error final por encima de `f1j_dryrun_max_final_error_t` quedan como `partial_candidate=true`, por ejemplo `12.038334 m -> 0.710850 m`, y no como `useful=true`;
- los casos parciales no se aplican en `1J` y no se consideran éxito normal: `1K` y `1L` deberán consumir esa clasificación con apply controlado, validación inmediata y rollback si procede.
- `1J` queda conseguida de nuevo; la siguiente revalidacion operativa fue `1K`.

Estado actual tras `1K`:

- `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp` define también `OptimizationApplyResult` y registros por KF aplicado;
- `OptimizationManager::ApplyUsefulResult` aplica únicamente dry-runs `useful=true` sobre `GlobalPoseStore`;
- `partial_candidate=true` con `useful=false` no se acepta ni se aplica en `1K`: el servidor emite `[F1K-OPT-PARTIAL-PENDING]` y deja deuda obligatoria para `1L`;
- `GlobalPoseStore` distingue KFs optimizados por servidor y KFs propagados, conserva correcciones por KF y última corrección heredable por submapa;
- `GlobalMapBuilder` publica MapPoints usando corrección del KF de referencia, de observaciones o la última corrección del submapa, y reporta `server_corrected_points`;
- `orbslam3_server/src/global_map_server.cpp` invoca apply tras dry-run útil, loggea precheck, KFs optimizados/propagados, raw intacto, reconstrucción y publicación tras apply;
- la prueba live larga `prueba_3` validó rodeo del edificio con ambos drones en sentidos opuestos: `14` goals enviados y `14` resultados `success=true`;
- la prueba replay/debug `prueba_2` validó apply reproducible desde `rawdb_prueba_1.record`;
- se observaron `5` applies útiles en live y `3` en replay, todos con `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`;
- se validó herencia posterior de corrección en live con `309` eventos `[F1K-POSESTORE-NEW-KF-INHERIT-CORRECTION]`;
- no hubo KFs pendientes llegados exactamente durante el apply (`pending_rebased=0`), porque el apply actual es síncrono y breve; el caso queda cubierto por herencia posterior y por validación específica de `1Q` si más adelante hace falta;
- no aparecieron `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE`, prechecks fallidos ni apply con `applied=false`;
- revalidacion 2026-07-11: `OptimizationManager::ApplyCandidateResult` y `global_map_server` bloquean apply si falta preservacion de anclaje previo;
- la revalidacion 2026-07-11 produjo `7` applies en `prueba_1` y `10` applies en `prueba_2`, todos con `[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK] ok=true`, `fixed_kfs>=1`, `hard_fixed_moved=false` y `[F1K-RAWDB-NOT-MODIFIED] ok=true`;
- en esa revalidacion no aparecieron `fixed_kfs=0`, `hard_fixed_moved=true`, `raw_db_modified=true`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE`, `FATAL`, `Segmentation fault` ni `Killed`;
- `1K` queda conseguida de nuevo y habilita la revalidacion de `1L`: validar el error real post-apply, crear backup/rollback y decidir `ACCEPT`, `PARTIAL_KEEP_FOR_NEXT_PASS` o `REJECT_ROLLBACK` con comprobacion explicita de preservacion de anclajes.

Estado actual tras `1L`:

- `OptimizationManager` define `PostApplyValidationResult`, métricas post-apply, decisión `PostApplyDecision` y sugerencias de reintento;
- `GlobalPoseStore` puede crear backup por `task_id`, restaurarlo con rollback y confirmarlo tras `ACCEPT` o `PARTIAL_KEEP_FOR_NEXT_PASS`;
- `OptimizationManager::ApplyCandidateResult` aplica candidatos útiles y, si `f1l_partial_apply_enabled=true`, también candidatos parciales bajo backup obligatorio;
- `OptimizationManager::ValidatePostApply` recalcula error fiducial real desde `GlobalPoseStore`, error interno de aristas, KFs fijos/hard fiducials, propagación y coherencia de publicación;
- `global_map_server` emite marcadores `[F1L-*]`, decide `ACCEPT`, `PARTIAL_KEEP_FOR_NEXT_PASS` o `REJECT_ROLLBACK`, confirma backup o restaura estado y reconstruye/publica la nube tras rollback;
- se validaron tres pruebas live con `rawdb_record_enabled:=false` para evitar llenar disco: prueba `1` accept/parcial, prueba `2` rollback forzado y rechazos naturales, prueba `3` accept/parcial;
- prueba `1`: `14` goals `success=true`, `7` decisiones `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`, `0` rollback;
- prueba `2`: `14` goals `success=true`, `4` decisiones `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`, `3` `REJECT_ROLLBACK`, incluyendo rollback forzado debug y rollback natural por `internal_edges_broken`;
- prueba `3`: `14` goals `success=true`, `3` decisiones `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`, `0` rollback;
- no aparecieron `ROLLBACK_FAILED`, `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `HARD-FIDUCIAL-MOVED_ACCEPTED`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `FATAL`, `Segmentation fault` ni `Killed`;
- los cierres `gazebo ... exit code 255` aparecen después de `SIM-DONE success=true` y siguen clasificados como cleanup no bloqueante;
- el dataset `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` se borró durante la ejecución de `1L` para liberar espacio tras fallos operativos de build/log por workspace/disco; por eso la prueba de replay recomendada se sustituyó por una tercera prueba live equivalente;
- no se observó RViz2 manualmente en la ejecución automática; la evidencia de logs confirma publicación y reconstrucción de `/global_sparse_cloud`, pero conviene inspección visual humana antes de juzgar calidad estética final.
- reapertura: una inspeccion RViz2 posterior a la prueba larga detecto un problema no cubierto por los criterios automaticos originales: en una optimizacion parcial del dron 2, la zona del fiducial objetivo mejoro, pero KFs del fiducial previo parecieron desplazarse. El log confirmo que ese grafo tenia `fixed=0 hard_fiducial=0`, por lo que el `fixed check` de `1L` paso de forma vacia;
- cierre 2026-07-11: `OptimizationManager::ValidatePostApply` incorpora `PostApplyAnchorPreservationCheck` y `global_map_server` emite `[F1L-ANCHOR-PRESERVATION-CHECK]`;
- la revalidacion ejecutó tres pruebas live con `rawdb_record_enabled:=false`, `f1j_dryrun_enabled:=true`, `f1k_apply_enabled:=true`, `f1l_validation_enabled:=true` y `f1l_partial_apply_enabled:=true`;
- las tres pruebas terminaron con `14` goals `success=true`, `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE success=true` y `SIM-EXIT-CODE 0`;
- `prueba_1`: `13` validaciones post-apply, `2` `ACCEPT`, `11` `REJECT_ROLLBACK`;
- `prueba_2`: `20` validaciones post-apply, `3` `ACCEPT`, `17` `REJECT_ROLLBACK`, incluyendo rechazo debug forzado y rechazos naturales por `internal_edges_broken`;
- `prueba_3`: `19` validaciones post-apply, `3` `ACCEPT`, `16` `REJECT_ROLLBACK`;
- todos los rechazos restauraron con `[F1L-POSESTORE-ROLLBACK] ok=true` y publicaron tras rollback;
- todos los checks de anclaje dieron `[F1L-ANCHOR-PRESERVATION-CHECK] ok=true required=true previous_fiducial_fixed_count=1 checked_branch_anchors=1 max_previous_fiducial_delta_t=0.000000000`;
- hubo `partial_candidate=true` tratados por `1L`, pero ninguno quedo como `PARTIAL_KEEP_FOR_NEXT_PASS`: se rechazaron con rollback cuando rompian aristas internas. La deuda de reconstruir grafo desde estado parcial sigue documentada para el caso en que aparezca una decision parcial segura;
- no aparecieron `F1L-ANCHOR-PRESERVATION-FAIL`, `ROLLBACK_FAILED`, `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED_ACCEPTED`, `FATAL`, `Segmentation fault` ni `Killed`;
- correccion de estado tras inspeccion RViz2 del usuario: `1L` queda `PARCIAL`, no `CONSEGUIDA`. El caso critico del dron antihorario al llegar al fiducial 1 se detecta y se intenta optimizar, pero la ruta no es todavia segura: con rollback se vuelve al estado absurdo y, al permitir parciales, una propagacion amplia puede desplazar zonas que estaban bien alrededor del fiducial previo.
- ajuste de plan para cerrar `1L`: no se debe volver al estado de `10-30 m` si una primera optimizacion baja mucho el error fiducial y preserva anclajes, pero tampoco se puede publicar una deformacion masiva del submapa. `1L` debe clasificar aristas internas como fuertes o deformables y ademas validar vecindades fiduciales y propagacion segura.
- definicion oficial para el siguiente intento: ventana entre fiduciales del mismo submapa, vertices representativos repartidos por toda la ventana, vecindades fiduciales rigidas, propagacion coherente/interpolada para KFs no variables y reconstruccion de grafo antes de una segunda pasada parcial.
- si `before_t >= fiducial_absurd_error_t` y el apply parcial baja mucho el error, no mueve hard fiducials, no modifica raw, no rompe aristas internas fuertes y no arrastra la vecindad del fiducial previo, debe guardarse como checkpoint parcial seguro, marcar `partial_pending=true`, reconstruir un grafo desde las poses parciales y reoptimizar hasta bajar del umbral. Si una pasada posterior falla, el rollback debe volver al ultimo checkpoint parcial seguro, no necesariamente al estado absurdo original.

## Arquitectura objetivo de Fase 1

```text
wrappers ORB-SLAM3
  -> OrbMap delta / full snapshot / pose_local
orbslam3_server
  -> adaptador ROS 2, conversión y publicación
orbslam3_multi
  -> RawMapDatabase
  -> GlobalPoseStore
  -> FiducialAnchorManager
  -> LoopDetector
  -> SubcloudLoopVerifier
  -> LoopDecisionManager
  -> FusionManager / FusedLandmarkManager
  -> LandmarkScoreManager
  -> PoseGraphBuilder
  -> OptimizationManager
  -> GlobalMapBuilder
orbslam3_server
  -> nube sparse global, correcciones, corrected keyframes y debug
```

## Subfases activas de Fase 1

La planificación activa completa está en `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md` y en los archivos `subfase_1A.md` a `subfase_1X.md`.

Resumen:

- `1A`: baseline del servidor antiguo y logs actuales.
- `1B`: servidor nuevo mínimo y migración segura.
- `1C`: `RawMapDatabase`, deltas/full snapshots y replay. Conseguida.
- `1D`: `GlobalPoseStore`. Conseguida.
- `1E`: `FiducialAnchorManager`. Conseguida.
- `1F`: nube global sparse con scores iniciales. Conseguida.
- `1G`: reconciliación con full snapshots. Conseguida.
- `1H`: segunda visita a fiducial y tarea de optimización. Conseguida.
- `1I`: grafo temporal. Conseguida de nuevo tras preservar fiduciales previos.
- `1J`: dry-run. Conseguida de nuevo tras revalidar preservacion de anclajes.
- `1K`: apply en `GlobalPoseStore`. Conseguida de nuevo tras revalidar `[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK] ok=true`, raw intacto y publicación tras apply.
- `1L`: validación post-apply, rollback y candidatos parciales. PARCIAL: el solver 6D ya acepta el caso `dron_2` antihorario con aristas internas coherentes, pero la ventana GT completa aún deja `max_after` alto y requiere cierre visual/técnico.
- `1M`: `CovisibilityDatabase` confirmada. Actual/sin hacer; no guarda candidatos ni campo `state`.
- `1N`: `LoopDetector` BoW. Sin hacer; no iniciar hasta completar `1M`.
- `1O`: subnubes, matching ORB, reducción espacial y RANSAC.
- `1P`: decisión de loop, fusión y scoring multi-dron.
- `1Q`: optimización por loop reutilizando 1I-1L.
- `1R`: reprocesado de KFs llegados durante optimización.
- `1S`: consolidación de scoring y fused tracks.
- `1T`: contratos, frames, IDs e invariantes.
- `1U`: observabilidad final en RViz2 y topics debug.
- `1V`: pruebas integrales end-to-end y regresión.
- `1W`: rendimiento, límites y robustez.
- `1X`: cierre documental, legacy y handoff de Fase 1.

## Estado de validación

La subfase `1A` se ejecutó el 2026-07-03 como baseline del servidor monolítico actual.

Evidencia principal:

- build ejecutado con `build_selected_packages.sh` y resultado `BUILD-EXIT-CODE 0`;
- `colcon` compiló `orbslam3_msgs`, `orbslam3_multi`, `orbslam3_server`, `lib_tray`, `dron_individual` y `simulacion_dron`;
- el paquete seleccionado `orbslam3_ros2` fue ignorado por `colcon` porque el nombre ROS declarado en su `package.xml` es `orbslam3`;
- simulación `prueba_1` ejecutada con `run_simulation.sh` y `SIM-DONE prueba=1 success=true`;
- `scenario_runner_node` envió 6 goals y los 6 terminaron con `success=true`;
- se generaron `codex/archivos_auxiliares/logs/prueba_1.log` y `codex/archivos_auxiliares/logs/prueba_1.reduced.log`;
- los logs muestran publicación de wrappers, recepción del servidor, submapas por `(drone_id, map_epoch)`, fiduciales, correcciones, candidatos fused, loops/subclouds y estado de optimización local.

Problemas o límites observados en la baseline:

- no se observó RViz2 manualmente;
- Gazebo emitió `process has died ... exit code 255` durante el cierre, después de `SIM-DONE success=true`;
- la optimización local no aplicó cambios útiles en esta prueba (`applied=0`, `optimized_kfs=0`, `optimized_submaps=0`);
- la fusión subcloud quedó bloqueada por una región pendiente de optimización (`OPTIMIZATION_PENDING_DRYRUN`);
- la ruta legacy sigue siendo monolítica y sirve solo como referencia para la migración.

Conclusión de `1A`: `CONSEGUIDA` para el objetivo de capturar y documentar baseline. El detalle queda en:

```text
codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md
```

La subfase `1B` se ejecutó el 2026-07-06 y queda `CONSEGUIDA`.

Evidencia principal:

- primer build falló por `No space left on device`; se redujo el log y se limpió solo `build/orbslam3_server` y el log generado de ese intento;
- build final con `BUILD-EXIT-CODE 0`;
- paquetes compilados: `orbslam3_msgs`, `orbslam3_multi`, `orbslam3_server`, `simulacion_dron`;
- simulación `prueba_1` con `SIM-DONE prueba=1 success=true`;
- 6 goals de `scenario_runner_node`, todos con `success=true`;
- logs `[F1B-SERVER-INIT]`, `[F1B-SERVER-PARAMS]`, `[F1B-SERVER-SUBSCRIBED]`, `[F1B-ORBMAP-RX]`, `[F1B-ORBMAP-RX-KFS]`, `[F1B-ORBMAP-RX-MPS]` y `[F1B-SERVER-STATS]`;
- recepción de `OrbMap` de `dron_1` y `dron_2`;
- sin `FATAL`, `Segmentation fault`, `Killed`, `std::bad_alloc` ni `No space left` durante la simulación.

Limitaciones esperadas de `1B`:

- no hay nube global;
- no hay fiduciales;
- no hay loops/subcloud/fusión;
- no hay optimización;
- no se observó RViz2 porque no es criterio de éxito en `1B`.

La subfase `1C` se ejecutó el 2026-07-06 y queda `CONSEGUIDA`.

Evidencia principal:

- build restaurado en paquetes separados para evitar bloqueo del ordenador: `orbslam3_msgs`, `orbslam3`, `orbslam3_multi`, `orbslam3_server`, `dron_individual` y `simulacion_dron`;
- se creó `RawMapDatabase` en `orbslam3_multi`;
- simulación `prueba_1` con `SIM-DONE prueba=1 success=true`;
- 6 goals de `scenario_runner_node`, todos con `success=true`;
- dataset `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` generado con tamaño aproximado `272M`;
- logs de grabación con `284` `[F1C-RAWDB-DELTA-RX]`, `284` `[F1C-RAWDB-INSERT-DELTA]`, `3` `[F1C-RAWDB-NEW-SUBMAP]` y guardado `[F1C-RAWDB-SAVE]`;
- replay `prueba_4` sin Gazebo con `SIM-DONE prueba=4 success=true`;
- `[F1C-REPLAY-LOAD] success=true entries=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
- `284` `[F1C-REPLAY-DELTA]` y `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`.

Limitaciones y riesgos:

- no hay nube global, fiduciales, loops, fused landmarks ni optimización; sigue siendo esperado hasta subfases posteriores;
- el único `ERROR` observado en simulación fue `gazebo exit code 255` durante cleanup posterior a `SIM-DONE success=true`;
- el dataset raw ocupa mucho espacio y el disco quedó con margen muy bajo, unos `467M`, por lo que conviene vigilar espacio antes de nuevas simulaciones largas.

La subfase `1D` se ejecutó el 2026-07-08 y queda `CONSEGUIDA`.

Evidencia principal:

- se creó `GlobalPoseStore` en `orbslam3_multi`;
- build en llamadas separadas: `orbslam3_multi` con `BUILD-EXIT-CODE 0` y `orbslam3_server` con `BUILD-EXIT-CODE 0`;
- el primer intento de build falló por `No space left on device`; se redujo el log, se limpió de forma mínima artefactos generados de `build/orbslam3_multi`, `build/orbslam3_server` y el log generado, y se eliminaron logs auxiliares antiguos de `prueba_2`;
- replay `prueba_1` sin Gazebo con `SIM-DONE prueba=1 success=true` y `SIM-EXIT-CODE 0`;
- `[F1D-POSESTORE-INIT] anchors=0 world_poses=0 optimized_kfs=0 corrections=0`;
- `[F1D-POSESTORE-ANCHOR-SUMMARY] ... anchored_kfs=1`;
- `[F1D-POSESTORE-OPT-POSE-SET]`;
- `[F1D-POSESTORE-CORRECTION-SET]`;
- `4` eventos `[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]`;
- `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
- no aparecen errores graves durante la prueba.

Limitaciones esperadas de `1D`:

- no hay anclaje real todavía;
- no hay fiduciales reales, loops, fused landmarks, optimización real ni nube global;
- las poses world observadas en logs provienen de un modo debug/sintético para validar la infraestructura;
- el disco sigue con poco margen libre, aproximadamente `569M` tras la validación y el cierre documental.

La subfase `1E` se ejecutó el 2026-07-08 y queda `CONSEGUIDA`.

Evidencia principal:

- se borró el `.record` anterior antes de la prueba viva para ahorrar espacio y generar un dataset nuevo;
- se creó `FiducialAnchorManager` y se integró con `GlobalPoseStore`;
- `RawMapDatabase` pasó a `.record` versión 2 con `fiducial_observation_journal`;
- build en llamadas separadas: `orbslam3_multi` con `BUILD-EXIT-CODE 0` y `orbslam3_server` con `BUILD-EXIT-CODE 0`;
- simulación Gazebo `prueba_1` con `SIM-DONE prueba=1 success=true`;
- dataset nuevo `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` generado con `364` deltas, `3` submapas, `175` KFs, `21095` MPs y `60` observaciones fiduciales;
- logs live con `[F1E-FID-KF-ASSOC]`, `[F1E-FID-OBS]`, `[F1E-FID-FIRST-ANCHOR]`, `[F1E-FID-WORLD-T-LOCAL]`, `[F1E-FID-KF-HARD]` y `[F1E-FID-JOURNAL-SAVE]`;
- replay `prueba_2` sin Gazebo con `SIM-DONE prueba=2 success=true`;
- `[F1C-REPLAY-LOAD] ... fiducial_observations=60`;
- `60` eventos `[F1E-FID-REPLAY-OBS]`;
- `[F1E-FID-STATS] ... observations=60 accepted=60 rejected=0 anchors_created=2 replay_observations=60 hard_fiducial_kfs=60`;
- `[F1C-REPLAY-DONE] entries=364 journal=364 deltas=364 full=0 fiducial_observations=60 submaps=3 kfs=175 mps=21095`.

Limitaciones esperadas de `1E`:

- la observación fiducial sigue siendo simulada con GT de Gazebo, permitido solo para fiducial/debug;
- todavía no se publica nube global visible en RViz2; eso empieza en `1F`;
- no hay segunda visita fiducial con optimización, loops, fused landmarks ni scoring;
- en la prueba Gazebo aparece `gazebo ... exit code 255` durante el cierre posterior a `SIM-DONE success=true`, patrón no bloqueante ya observado;
- el disco queda con margen muy bajo tras el nuevo `.record`, por lo que conviene liberar espacio antes de simulaciones largas.

## Siguiente objetivo

Ejecutar `1F` siguiendo su archivo:

```text
codex/pipeline/fase_1_sparse_global/subfases/subfase_1F.md
```

La meta de `1F` es crear `LandmarkScoreManager` y `GlobalMapBuilder` para publicar una nube sparse global inicial en `world` usando solo submapas ya anclados y score raw inicial.
