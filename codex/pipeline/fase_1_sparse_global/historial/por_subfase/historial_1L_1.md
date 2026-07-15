# Historial 1L

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-10 — Subfase 1L — Validación post-apply, rollback y parciales

- objetivo intentado:
  - validar cada optimización después de aplicarla realmente en `GlobalPoseStore`;
  - crear backup previo al apply y confirmar o restaurar según decisión;
  - recalcular error principal real, error interno de aristas, KFs fijos/hard fiducials, propagación y mapa global;
  - resolver `partial_candidate=true` sin dejar el mapa en el peor estado conocido;
  - emitir decisión explícita `ACCEPT`, `PARTIAL_KEEP_FOR_NEXT_PASS` o `REJECT_ROLLBACK`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `simulacion_dron/launch/multi_dron.launch.py`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - documentación de contexto, paquetes, subfase, pipeline e historial.
- cambios realizados:
  - nuevos tipos `PostApplyValidationResult`, `PostApplyDecision`, métricas post-apply y `RetrySuggestion`;
  - `GlobalPoseStore::CreateApplyBackup`, `RestoreApplyBackup` y `ConfirmApply`;
  - `OptimizationManager::ApplyCandidateResult` para aceptar parciales solo bajo control de `1L`;
  - `OptimizationManager::ValidatePostApply` para error real, error interno, fijos, propagación y sugerencia de reintento;
  - `global_map_server` crea backup, aplica, valida, confirma o rollbackea, reconstruye/publica nube y loggea `[F1L-*]`;
  - parámetros `f1l_validation_enabled`, `f1l_partial_apply_enabled`, `f1l_debug_force_reject_once`, `f1l_debug_force_reject_task_id`, `f1l_post_apply_internal_broken_edge_t` y `f1l_post_apply_internal_max_growth_t`;
  - `multi_dron.launch.py` reenvía los argumentos debug/control de `1L` al servidor.
- YAMLs de prueba:
  - se reutilizó `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - se actualizó `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml` como `prueba_1l_rodeo_edificio_rollback_forzado`;
  - se reutilizó `codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - los primeros intentos fallaron por problemas operativos de workspace/disco, no por error C++;
  - se ejecutó `./codex/herramientas/reduce_build_log.sh`;
  - el `colcon_build.reduced.log` conservado corresponde a un intento con `Read-only file system` al crear logs de `colcon`;
  - se liberó espacio borrando el auxiliar generado `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`;
  - build final de `orbslam3_multi` y `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - build final de `simulacion_dron`: `BUILD-EXIT-CODE 0`.
- pruebas ejecutadas:
  - prueba `1` live larga sin grabación raw:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false f1j_export_debug_plot:=false f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true" --post-scenario-wait-sec 35 --startup-wait-sec 20 --timeout-sec 1400 --max-gazebo-retries 1
    ```
  - prueba `2` live larga con rollback forzado:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false f1j_export_debug_plot:=false f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true f1l_debug_force_reject_once:=true" --post-scenario-wait-sec 35 --startup-wait-sec 20 --timeout-sec 1400 --max-gazebo-retries 1
    ```
  - prueba `3` live larga equivalente a replay recomendado por falta de dataset raw:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 3 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1g_debug_mark_optimized_kf:=false f1j_export_debug_plot:=false f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true" --post-scenario-wait-sec 35 --startup-wait-sec 20 --timeout-sec 1400 --max-gazebo-retries 1
    ```
- resultado de simulación:
  - `prueba_1`: `SCENARIO-RUNNER-DONE ... success=true`, `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`, `14` goals `success=true`;
  - `prueba_2`: `SCENARIO-RUNNER-DONE scenario='prueba_1l_rodeo_edificio_rollback_forzado' success=true`, `SIM-DONE prueba=2 success=true`, `SIM-EXIT-CODE 0`, `14` goals `success=true`;
  - `prueba_3`: `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`, `SIM-DONE prueba=3 success=true`, `SIM-EXIT-CODE 0`, `14` goals `success=true`.
- patrones usados para reducir logs:
  - prueba `1`:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1L-|F1K-OPT-APPLY|F1J-OPT|F1I-GRAPH|F1H-FID-TASK-CREATED|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba `2`:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1L-POST-APPLY-REJECT|F1L-POSESTORE-ROLLBACK|F1L-ROLLBACK-DIAGNOSTIC|F1L-RETRY-SUGGESTION|F1K-OPT-APPLY|ERROR|FATAL|Segmentation fault|Killed
    ```
  - prueba `3`:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1L-|F1C-REPLAY|F1K-OPT-APPLY|F1J-OPT|F1I-GRAPH|ERROR|FATAL|Segmentation fault|Killed
    ```
- logs reducidos:
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`;
  - `codex/archivos_auxiliares/logs/prueba_2.reduced.log`;
  - `codex/archivos_auxiliares/logs/prueba_3.reduced.log`.
- evidencia positiva:
  - `prueba_1`: `8` validaciones post-apply, `7` `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`, `0` rollback;
  - `prueba_2`: `8` validaciones post-apply, `4` `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`, `3` `REJECT_ROLLBACK`;
  - `prueba_3`: `4` validaciones post-apply, `3` `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`, `0` rollback;
  - todas las validaciones incluyen `[F1L-POST-APPLY-ERROR]`, `[F1L-POST-APPLY-INTERNAL-ERROR]`, `[F1L-POST-APPLY-FIXED-CHECK]`, `[F1L-POST-APPLY-PROPAGATION-CHECK]` y `[F1L-POST-APPLY-GLOBALMAP-CHECK]`;
  - `prueba_2` valida rollback forzado con `[F1L-POSESTORE-ROLLBACK] ok=true` y diagnóstico posterior;
  - `prueba_2` valida rechazos naturales por `internal_edges_broken` con rollback correcto;
  - los parciales se conservan como `PARTIAL_KEEP_FOR_NEXT_PASS` con `[F1L-RETRY-SUGGESTION] strategy=WIDER_WINDOW`;
  - no aparecen `ROLLBACK_FAILED`, `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `HARD-FIDUCIAL-MOVED_ACCEPTED`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `FATAL`, `Segmentation fault` ni `Killed`.
- evidencia negativa o notas:
  - la prueba replay recomendada no se ejecutó porque `rawdb_prueba_1.record` se borró para liberar espacio; se sustituyó por `prueba_3` live equivalente;
  - no se observó RViz2 manualmente;
  - el cierre conocido de Gazebo `exit code 255` aparece después de `SIM-DONE success=true` y no bloquea la validación.
- conclusión:
  - `CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_1N.md`: crear `LoopDetector` BoW sin geometría, sin RANSAC, sin fusión y sin optimización por loop.
## 2026-07-11 — Subfase 1L — Revalidacion post-apply/rollback con preservacion de anclajes

- objetivo intentado:
  - revalidar `1L` tras cerrar de nuevo `1I`, `1J` y `1K`;
  - comprobar error real post-apply, aristas internas, fijos/hard fiducials, propagacion y publicacion;
  - añadir una comprobacion explicita de preservacion de anclajes fiduciales previos en post-apply;
  - validar rollback ante rechazo debug y rechazos naturales.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/optimization_result.hpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1N.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- cambios realizados:
  - `PostApplyValidationResult` incorpora `anchor_preservation_ok` y `PostApplyAnchorPreservationCheck`;
  - `OptimizationManager::ValidatePostApply` compara vertices `previous_fiducial_anchor` contra su pose inicial de grafo;
  - si falta anclaje de rama, se mueve un fiducial previo o una subdivision no confirmada hace inseguro un parcial, la validacion falla;
  - `BuildRetrySuggestion` recomienda `REBUILD_GRAPH_FROM_PARTIAL_STATE` para parciales conservados y `DIFFERENT_VERTEX_SELECTION` si falla preservacion;
  - `global_map_server` emite `[F1L-ANCHOR-PRESERVATION-CHECK]`, `[F1L-ANCHOR-PRESERVATION-FAIL]` y `[F1L-PARTIAL-PENDING]`;
  - `tray_prueba_2.yaml` queda etiquetado como `prueba_1l_rodeo_edificio_rollback_forzado`;
  - `tray_prueba_3.yaml` queda etiquetado como `prueba_1l_rodeo_edificio_live_parciales`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ./codex/herramientas/build_selected_packages.sh simulacion_dron
  ```
- resultado de build:
  - los tres builds terminaron con salida `0`;
  - no hizo falta `reduce_build_log.sh`;
  - `orbslam3_multi` conserva un warning no bloqueante de funcion no usada en `optimization_manager.cpp`.
- pruebas Gazebo ejecutadas:
  - prueba `1`:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
  - prueba `2`:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 2 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true f1l_debug_force_reject_once:=true" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
  - prueba `3`:
    ```bash
    ./codex/herramientas/run_simulation.sh --prueba 3 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1600 --max-gazebo-retries 1
    ```
- resultado de simulacion:
  - las tres pruebas terminaron con `SCENARIO-RUNNER-DONE success=true`, `14` goals `success=true`, `SIM-DONE success=true` y `SIM-EXIT-CODE 0`;
  - en `prueba_1` y `prueba_2` aparece el cierre conocido de Gazebo `exit code 255` despues de `SIM-DONE success=true`, clasificado como cleanup no bloqueante.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER-GOAL-RESULT|SCENARIO-RUNNER-DONE|SIM-DONE|SIM-EXIT-CODE|F1H-FID-TASK-CREATED|F1I-GRAPH|F1I-FID-CONNECTIVITY|F1J-OPT|F1K-|F1L-|RAWDB-POSE-OVERWRITE-BY-OPT|HARD-FIDUCIAL-MOVED|APPLY-USEFUL-FALSE|ROLLBACK_FAILED|POST_APPLY_ACCEPT_WITH_ERROR_WORSE|ERROR|FATAL|Segmentation fault|Killed|process has died
  ```
- logs reducidos:
  - `codex/archivos_auxiliares/logs/prueba_1.reduced.log`;
  - `codex/archivos_auxiliares/logs/prueba_2.reduced.log`;
  - `codex/archivos_auxiliares/logs/prueba_3.reduced.log`.
- evidencia positiva:
  - `prueba_1`: `13` validaciones post-apply, `2` `ACCEPT`, `11` `REJECT_ROLLBACK`;
  - `prueba_2`: `20` validaciones post-apply, `3` `ACCEPT`, `17` `REJECT_ROLLBACK`, incluyendo rechazo debug forzado;
  - `prueba_3`: `19` validaciones post-apply, `3` `ACCEPT`, `16` `REJECT_ROLLBACK`;
  - todos los rechazos tienen `[F1L-POSESTORE-ROLLBACK] ok=true`, `[F1L-GLOBALMAP-REBUILD-AFTER-ROLLBACK]` y `[F1L-GLOBALMAP-PUBLISH-AFTER-ROLLBACK]`;
  - todos los checks de anclaje tienen `[F1L-ANCHOR-PRESERVATION-CHECK] ok=true required=true previous_fiducial_fixed_count=1 checked_branch_anchors=1 max_previous_fiducial_delta_t=0.000000000`;
  - hubo `partial_candidate=true` tratados por `1L`, por ejemplo reducciones grandes que luego se rechazaron por `internal_edges_broken` con rollback correcto.
- evidencia negativa o ausente:
  - no aparecio `PARTIAL_KEEP_FOR_NEXT_PASS` en esta tanda; la ruta queda implementada/documentada, pero no hubo un parcial que pasara todas las invariantes;
  - no aparecen `F1L-ANCHOR-PRESERVATION-FAIL`, `ROLLBACK_FAILED`, `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED_ACCEPTED`, `FATAL`, `Segmentation fault` ni `Killed`;
  - no se observo RViz2 manualmente durante esta ejecucion.
- conclusion:
  - `PARCIAL`.
- siguiente paso recomendado:
  - mantener la subfase actual en `1L` y corregir por que los parciales grandes del dron 2 en fiducial 1 terminan en `REJECT_ROLLBACK reason=internal_edges_broken`.

### Correccion posterior por inspeccion RViz2

Tras revisar visualmente RViz2, el usuario observo que no se puede dar `1L` como conseguida:

- cuando el dron 2 llega al fiducial 1 aparecen errores enormes, visibles tambien en logs con `error_t` entre `20 m` y `30 m`;
- el dry-run reduce esos errores a aproximadamente `1.8 m`-`3.3 m`, pero los clasifica como `partial_candidate=true` porque siguen por encima del umbral `0.35 m`;
- `1L` aplica esos parciales bajo backup, pero despues los rechaza con `reason=internal_edges_broken`;
- el rollback funciona, pero al restaurar tambien se pierde la mejora parcial, por lo que RViz2 queda con el mapa sin corregir.

Ejemplos:

```text
prueba_2 task_id=4: 20.122146 m -> 1.878937 m, REJECT_ROLLBACK reason=internal_edges_broken
prueba_1 task_id=13: 29.931767 m -> 3.321213 m, REJECT_ROLLBACK reason=internal_edges_broken
prueba_3 task_id=4: 26.624750 m -> 3.123809 m, REJECT_ROLLBACK reason=internal_edges_broken
```

Por tanto, el estado real de `1L` es `PARCIAL`, no `CONSEGUIDA`.

### Decision de diseño posterior

El usuario aclaro que no se puede dejar nunca un fiducial en errores absurdos (`10 m`, `20 m`, `30 m`) si una primera optimizacion ya puede reducirlo mucho, por ejemplo `30 m -> 4 m`. Ese resultado no es aceptable como final, pero es mucho mejor que conservar el estado absurdo.

Se acuerda cambiar el plan de `1L`:

- clasificar aristas internas como fuertes o deformables;
- permitir que aristas deformables se deformen mucho si proceden de zonas sospechosas de deriva, por ejemplo una pared recta poco texturizada que ORB-SLAM3 convirtio en una esquina falsa de casi `90 deg`;
- si un parcial reduce un error absurdo y preserva anclajes/hard fiducials/raw, conservarlo como `PARTIAL_KEEP_FOR_NEXT_PASS` aunque rompa aristas deformables;
- guardar un checkpoint parcial seguro;
- reconstruir un grafo nuevo desde las poses parciales;
- lanzar una segunda optimizacion para bajar el error por debajo del umbral;
- si la segunda pasada falla, volver al ultimo checkpoint parcial seguro antes que al estado absurdo original, salvo que ese checkpoint viole invariantes duras.

Marcadores futuros esperados:

```text
[F1L-PARTIAL-ABSURD-ERROR-POLICY]
[F1L-INTERNAL-EDGE-CLASSIFY]
[F1L-PARTIAL-CHECKPOINT]
[F1L-PARTIAL-PENDING] graph_rebuild_required=true
[F1I-GRAPH-REBUILD-FROM-PARTIAL]
```

### Analisis posterior — definicion canonica de optimizacion por fiducial

Tras una nueva prueba larga de `1L`, el usuario paro la simulacion porque RViz2 seguia mostrando movimientos incoherentes. El diagnostico cambia:

- el problema no es solo que los parciales se reviertan por `internal_edges_broken`;
- permitir un parcial tampoco basta si ese parcial desplaza zonas del submapa que estaban bien;
- el caso observado del dron antihorario al llegar al fiducial 1 mostraba un error inicial de aproximadamente `24 m` que bajaba a unos `2.3 m`, pero propagaba decenas de KFs con deltas de `14-25 m` y yaw de unos `2.07 rad`;
- visualmente, los KFs/puntos alrededor del fiducial 1 quedaban mejor, pero los KFs/puntos que antes estaban alrededor del fiducial 2 se desplazaban.

Decision de diseño:

- una optimizacion por fiducial debe construirse como ventana topologica entre fiduciales del mismo submapa;
- el `FiducialConnectivityGraph` registra aristas de submapa entre fiduciales, incluyendo `from_kf_id`, `to_kf_id`, direccion, submapa y estado de confianza;
- el grafo de optimizacion toma todos los KFs de esa ventana y selecciona `N` vertices representativos repartidos por toda la ventana;
- los vertices representan tambien vecindades de KFs no variables;
- las vecindades de fiduciales confirmados deben moverse cero o muy poco, no solo el KF hard fiducial exacto;
- la propagacion de KFs no variables debe ser coherente/interpolada con los vertices vecinos, no una transformacion rigida amplia del submapa;
- si el error baja mucho pero sigue fuera de umbral, se puede conservar un checkpoint parcial solo si no arrastra la vecindad del fiducial previo ni publica propagacion amplia incoherente;
- tras un parcial, se debe reconstruir el grafo desde las poses nuevas antes de lanzar otra optimizacion.

Documentacion actualizada:

- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1I.md`;
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1J.md`;
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1K.md`;
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1N.md`;
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
- `codex/contexto/01_ESTADO_ACTUAL.md`;
- `codex/contexto/07_ULTIMA_SESION.md`;
- documentacion de `orbslam3_multi` y `orbslam3_server`.

Conclusion: `1L` sigue `PARCIAL`. No iniciar `1N` hasta que el apply parcial por fiducial respete vecindades fiduciales y propagacion segura.

## 2026-07-11 22:15 — Subfase 1L — prueba aislada del dron antihorario

- objetivo intentado: aplicar la definicion canonica de ventana fiducial, proteger vecindades, propagar de forma interpolada y ejecutar segunda pasada desde checkpoint.
- codigo modificado: `pose_graph_problem.hpp`, `pose_graph_builder.hpp/.cpp`, `optimization_result.hpp`, `optimization_manager.cpp`, `global_map_server.cpp` y `global_orb_map_server.launch.py`.
- YAMLs: se mantuvieron `tray_prueba_1.yaml`/`tray_prueba_2.yaml`; `tray_prueba_3.yaml` se rehizo para mover solo `dron_2` antihorario.
- paquetes compilados: `orbslam3_multi`, `orbslam3_server`, `simulacion_dron`; builds finales con salida `0`.
- build intermedio: `orbslam3_server` fallo por `confirmed`/`retry_count` fuera de scope; `reduce_build_log.sh` identifico el primer error y el bloque se recoloco antes de repetir con exito.
- simulaciones: `prueba_1` completa de dos drones y `prueba_3` aislada; `prueba_2` rollback forzado no se repitio por el cambio de objetivo solicitado durante la ejecucion.
- patrones: `SCENARIO-RUNNER`, `F1H-FID-TASK-CREATED`, `F1I-GRAPH-*PARTIAL`, `F1J-OPT`, `F1K-OPT-APPLY`, `F1L-*`, errores graves y marcadores prohibidos.
- evidencia positiva aislada: `7` goals `success=true`; `28.176878 -> 1.408844 -> 0.070442 m`; `previous_fiducial_fixed_count=1`, `previous_fiducial_neighborhood_fixed_count=4` y todos sus deltas `0`; rollback observado `ok=true`.
- evidencia negativa: RViz2 mostro separacion de KFs/puntos en `prueba_1`; en `prueba_3` las primeras pasadas mueven `90-110` KFs y dejan `internal_max_after=49-60 m`, aunque la segunda pasada alcance umbral.
- conclusion: `PARCIAL`.
- siguiente paso recomendado: no ejecutar `1N`; implementar un solver real sobre edges/priors y bloquear la publicacion de checkpoints con geometria interna extrema.

## 2026-07-12 — Subfase 1L — aclaracion de diseño del trigger fiducial grande

- objetivo trabajado: documentar el comportamiento correcto cuando un submapa ya anclado llega a otro fiducial con error grande.
- archivos modificados: documentacion de subfases `1J`, `1K`, `1L`, pipeline de fase, estado actual, ultima sesion e informacion de paquetes `orbslam3_multi`/`orbslam3_server`.
- paquetes compilados: no aplica, solo documentacion.
- pruebas Gazebo ejecutadas: no aplica, solo analisis y documentacion.
- evidencia usada: pruebas anteriores de `1L` con `dron_2` aislado y logs donde el error fiducial baja (`28.176878 -> 1.408844 -> 0.070442 m`) pero aparecen `internal_max_after=49-60 m` y RViz2 muestra KFs/puntos separados.
- decision tecnica: el cierre de `1L` exige sustituir la heuristica por un solver real de pose graph. El grafo debe usar ventana entre fiduciales, vertices repartidos, esquinas/cambios de yaw, edges relativas, priors fiduciales y pesos suaves proporcionales a distancia/confianza respecto a fiduciales.
- regla importante: no fijar de forma binaria todo el entorno de un fiducial y dejar libre el resto; la fuerza debe ser gradual. Cerca de fiduciales confirmados cuesta mucho mover, lejos de fiduciales y en zonas con deriva se permite mas deformacion.
- conclusion: `PARCIAL`.
- siguiente paso recomendado: mantener `1L` como actual e implementar el solver real antes de iniciar `1N`.

## 2026-07-12 — Subfase 1L — solver minimo propio y nuevo criterio de aristas internas

- objetivo intentado: ejecutar `1L` sin insistir en `g2o` de ORB-SLAM3, usando
  una optimizacion minima propia si era la opcion mas segura.
- codigo modificado:
  - `orbslam3_multi/src/optimization_manager.cpp`.
- documentacion modificada:
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- paquetes compilados:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - `simulacion_dron`: `BUILD-EXIT-CODE 0` antes del ultimo ajuste;
  - builds finales tras el ultimo ajuste: `orbslam3_multi` y `orbslam3_server`
    con salida `0`.
- pruebas Gazebo ejecutadas:
  - `prueba_3` aislada de `dron_2`: `SCENARIO-RUNNER-DONE success=true`,
    `SIM-DONE success=true`, `SIM-EXIT-CODE 0`; no reprodujo el error grande
    del mismo submapa porque `dron_2` cambio de `map_epoch`;
  - `prueba_1` larga de dos drones: `SIM-DONE success=true`,
    `SIM-EXIT-CODE 0`;
  - el usuario pidio parar simulaciones tras observar en RViz2 que seguian
    ocurriendo optimizaciones malas; no se ejecuto otra prueba posterior al
    ultimo ajuste de aristas.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|SIM-|F1I-|F1J-|F1K-|F1L-|GOAL|RESULT|success|ERROR|FATAL|Segmentation fault|Killed
  ```
- evidencia positiva:
  - el solver minimo reduce errores moderados y aplica con checks sanos, por
    ejemplo `task_id=5`: `1.787994 -> 0.002748 m`,
    `F1L-POST-APPLY-ACCEPT`, `fixed_check ok=true`,
    `internal_max_after=1.374026`, `num_edges_broken=0`;
  - los builds finales pasan.
- evidencia negativa:
  - el caso critico del dron con deriva grande aun no queda resuelto:
    `task_id=6`: `28.738164 -> 0.017052 m`, pero
    `F1L-POST-APPLY-REJECT reason=internal_edges_broken`,
    `internal_max_after=8.526470`, `strong_edges_broken=1`;
  - tras la correccion parcial, aparecieron rechazos de `~2.4 m -> ~0.004 m`
    por crecimiento interno aunque `num_edges_broken=0`;
  - RViz2 mostro que el dron antihorario no optimiza correctamente al llegar al
    fiducial 1 y que el mapa hace movimientos malos.
- decision tecnica nueva:
  - las aristas internas deben conservar principalmente la distancia relativa
    entre KFs;
  - la yaw/rotacion relativa puede cambiar mucho en zonas de baja textura si
    ORB-SLAM3 genero una esquina falsa;
  - el solver y `ValidatePostApply` se ajustaron para penalizar longitud
    relativa y dar peso debil a yaw relativa.
- validacion pendiente:
  - el ultimo ajuste compila, pero no tiene todavia prueba Gazebo/RViz2 porque
    el usuario pidio parar simulaciones.
- conclusion: `PARCIAL`.
- siguiente paso recomendado:
  - mantener `1L` como actual;
  - antes de repetir una tanda larga, preparar una prueba corta/controlada que
    fuerce el caso antihorario con error grande y comprobar si el nuevo criterio
    de distancia relativa evita el rollback incorrecto y no separa KFs/puntos.

## 2026-07-12 — Subfase 1L — documentacion de metricas GT debug por KeyFrame

- objetivo intentado: dejar documentada la idea de usar GT solo como metrica de
  diagnostico para entender el error real de KFs antes/despues de las
  optimizaciones fiduciales grandes.
- cambios realizados:
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`: se anadio la
    propuesta `DebugGroundTruthKeyFrameStore`, logs `[F1L-GT-*]`, restricciones
    de uso y alternativas si la instrumentacion falla;
  - `codex/contexto/02_REGLAS_TECNICAS.md`: se aclaro que el GT por KF es
    valido solo como debug/metrica externa y queda prohibido como entrada del
    algoritmo;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`: se documento
    que el servidor puede asociar GT por timestamp a KFs y loggear metricas,
    manteniendolo separado de optimizacion/publicacion;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`: se aclaro
    que `OptimizationManager` no consume GT; solo puede ser evaluado contra
    metricas externas calculadas fuera;
  - `codex/contexto/01_ESTADO_ACTUAL.md` y
    `codex/contexto/07_ULTIMA_SESION.md`: se actualizo el estado de `1L` y el
    siguiente paso recomendado.
- comando de build: no ejecutado; actualizacion documental.
- resultado de build: `NO EJECUTADO`.
- pruebas Gazebo ejecutadas: `NO EJECUTADA`; el usuario habia pedido parar
  simulaciones y analizar.
- patrones de reduccion propuestos si se activa la metrica:
  ```text
  F1L-GT-|F1L-|F1K-OPT-APPLY|F1J-OPT|F1I-GRAPH|ERROR|FATAL|Segmentation fault|Killed
  ```
- evidencia positiva:
  - queda definido como medir `error_t/error_yaw` por KF, estadisticas de
    ventana y dano colateral en la vecindad del fiducial previo;
  - queda claro que el GT debug debe asociarse por `(drone_id,map_epoch,local_kf_id)`
    y timestamp, convirtiendo body a camera con `body_T_camera`;
  - quedan ideas de fallback para fallos de asociacion temporal, frames,
    cambios de epoch, outliers y ausencia de GT real.
- evidencia negativa o ausente:
  - no hay implementacion todavia;
  - no hay build ni simulacion nueva;
  - no hay evidencia `[F1L-GT-*]` real.
- conclusion: `PARCIAL`.
- siguiente paso recomendado: antes de repetir tandas largas de `1L`, implementar
  o simular una prueba corta de metricas GT por KF para saber si el solver esta
  corrigiendo la ventana completa o solo el fiducial objetivo a costa de dano
  colateral.

## 2026-07-12 — Subfase 1L — ejecucion con GT debug por KeyFrame y prueba corta antihoraria

- objetivo intentado: ejecutar `1L` con la instrumentacion GT debug por KF ya
  implementada, usando una sola prueba enfocada donde `dron_2` va en sentido
  antihorario desde el fiducial 2 al fiducial 1 con deriva grande, sin repetir
  tandas largas.
- codigo modificado:
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `simulacion_dron/launch/multi_dron.launch.py`.
- YAMLs modificados:
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`.
- documentacion modificada:
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/simulacion_dron/launches.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- paquetes compilados:
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - `simulacion_dron`: `BUILD-EXIT-CODE 0`.
- pruebas Gazebo ejecutadas:
  - `prueba_1` con launch:
    `ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0`;
  - resultado: `SCENARIO-RUNNER-DONE success=true`, `4` goals
    `success=true`, `SIM-DONE success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|SIM-|F1L-GT-|F1H-FID-TASK-CREATED|F1I-GRAPH|F1I-FID-CONNECTIVITY|F1J-OPT|F1K-|F1L-|RAWDB-POSE-OVERWRITE-BY-OPT|HARD-FIDUCIAL-MOVED|APPLY-USEFUL-FALSE|ROLLBACK_FAILED|POST_APPLY_ACCEPT_WITH_ERROR_WORSE|ERROR|FATAL|Segmentation fault|Killed|process has died
  ```
- evidencia positiva:
  - `[F1L-GT-DEBUG-CONFIG] enabled=true max_dt=1.000 source=GAZEBO_GT_DEBUG usage=metrics_only`;
  - aparecen `[F1L-GT-KF-ASSOC]`, `[F1L-GT-KF-ERROR-BEFORE]`,
    `[F1L-GT-KF-ERROR-AFTER]`, `[F1L-GT-WINDOW-STATS]` y
    `[F1L-GT-COLLATERAL-CHECK]`;
  - el caso grande se reproduce: `[F1H-FID-TASK-CREATED] ... fid=1 ...
    error_t=19.776323`;
  - se crea grafo: `[F1I-GRAPH-PROBLEM-CREATED] ... vertices=12 edges=11
    priors=12 variables=8 fixed=4 affected_non_variable=88`;
  - primera pasada: `[F1J-OPT-DRYRUN-DECISION] ... before_t=19.776323
    after_t=0.013640`, seguida de `[F1L-POST-APPLY-PARTIAL] ...
    decision=PARTIAL_KEEP_FOR_NEXT_PASS`;
  - segunda pasada desde grafo reconstruido: `[F1L-POST-APPLY-ACCEPT] ...
    real_after_t=0.000019`;
  - aparecen `7` accepts posteriores y `1` parcial;
  - no aparecen `ROLLBACK_FAILED`, `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`,
    `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED`, `FATAL`,
    `Segmentation fault` ni `Killed`;
  - el `process has died` de Gazebo aparece despues de `SIM-DONE success=true`
    y se considera cierre/cleanup no bloqueante.
- evidencia negativa:
  - las estadisticas GT de ventana no cumplen el objetivo:
    primera pasada `mean_before=7.179539 max_before=19.776323
    mean_after=4.274247 max_after=10.764699`;
  - segunda pasada `mean_before=4.274247 max_before=10.764699
    mean_after=4.272604 max_after=10.760135`;
  - muchos KFs intermedios quedan con `error_t` de `5-10 m` tras corregir el
    fiducial objetivo;
  - varias tareas posteriores aceptadas corrigen el KF fiducial por debajo del
    umbral, pero empeoran parte de la ventana (`worsened_kfs=36`, `35`, `44`).
- conclusion:
  - `PARCIAL`.
- siguiente paso recomendado:
  - mantener `1L` como actual;
  - usar los logs `[F1L-GT-WINDOW-STATS]` como criterio externo de diagnostico
    en simulacion;
  - modificar solver/propagacion para reducir el error de la ventana completa,
    no solo el residual del KF fiducial objetivo.

## 2026-07-12 — Subfase 1L — animacion HTML para diagnosticar grafo vs propagacion

- objetivo intentado: crear una visualizacion por tarea de optimizacion para
  entender por que RViz2 muestra KFs separados tanto cerca del fiducial 1 como
  cerca del fiducial 2.
- hipotesis del usuario: el fallo probablemente combina dos causas:
  - grafo mal construido;
  - propagacion incorrecta de KFs no vertices.
- codigo modificado:
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `simulacion_dron/launch/multi_dron.launch.py`.
- documentacion modificada:
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/paquetes/simulacion_dron/launches.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- cambios tecnicos:
  - nuevo parametro `f1l_debug_animation_enabled`;
  - nuevo parametro `f1l_debug_animation_output_dir`;
  - nueva funcion `ExportF1LDebugAnimation`;
  - por cada dry-run, si el parametro esta activo, se escribe:
    `codex/archivos_auxiliares/html/f1l_debug_animation_task_<task_id>.html`.
- contenido de la animacion:
  - frame `Inicial`: KFs del mapa en rojo y GT debug en negro;
  - frame `Grafo`: vertices agrandados, aristas del grafo, vertices mapa en
    morado/naranja y vertices GT en gris;
  - frame `Optimizado`: propuestas del dry-run en azul para propagados y verde
    para vertices.
- paquetes compilados:
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - `simulacion_dron`: `BUILD-EXIT-CODE 0`.
- pruebas Gazebo ejecutadas:
  - no ejecutadas en esta mini-iteracion; la herramienta queda compilada y lista
    para activarse en la proxima prueba corta.
- patrones de log nuevos:
  ```text
  F1L-DEBUG-ANIMATION-CONFIG
  F1L-DEBUG-ANIMATION-EXPORT
  ```
- evidencia positiva:
  - el build valida que el exportador HTML y los launch arguments compilan;
  - la herramienta separa visualmente grafo/vertices de propagacion no vertex.
- evidencia negativa o ausente:
  - aun no hay HTML generado por simulacion porque no se relanzo Gazebo;
  - aun no demuestra si el fallo real esta en grafo, propagacion o ambas.
- conclusion:
  - `PARCIAL`.
- siguiente paso recomendado:
  - ejecutar una sola prueba corta con
    `f1l_gt_kf_debug_enabled:=true f1l_debug_animation_enabled:=true` y abrir el
    HTML del `task_id` que corresponda al fiducial 1 del dron antihorario.

## 2026-07-12 — Subfase 1L — documentacion del refuerzo de grafo y propagacion

- objetivo intentado: aplicar en los MDs la decision tecnica acordada para
  corregir el error observado en RViz2: el fallo probable combina grafo mal
  construido y propagacion incorrecta de KFs no vertices.
- archivos modificados:
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1I.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- cambios documentados:
  - si la animacion de `1L` confirma vertices mal repartidos, reabrir de forma
    limitada `1I/PoseGraphBuilder`;
  - construir la ventana desde una arista fiducial real del mismo submapa;
  - seleccionar vertices por distancia acumulada de trayectoria, fiduciales,
    vecindades de ambos fiduciales y cambios fuertes de yaw;
  - subdividir aristas largas antes del solver;
  - rechazar grafos con mala cobertura mediante
    `[F1I-GRAPH-REJECT] reason=bad_window_coverage`;
  - exigir que el `PropagationPlan` guarde tramo/alpha para cada KF no vertex;
  - no aceptar una optimizacion si la animacion muestra KFs separados cerca de
    cualquiera de los dos fiduciales.
- paquetes compilados:
  - no aplica, solo documentacion.
- pruebas Gazebo:
  - no ejecutadas.
- conclusion:
  - `PARCIAL`.
- siguiente paso recomendado:
  - ejecutar una prueba corta con animacion activa para confirmar si el fallo
    esta en vertices del grafo, propagacion o ambos; despues implementar el
    refuerzo de `PoseGraphBuilder`.

## 2026-07-12 — Subfase 1L — refuerzo de grafo/propagacion y prueba antihoraria

- fase y subfase: Fase 1, subfase `1L`.
- objetivo intentado:
  - ejecutar `1L` con todo lo acordado tras las observaciones en RViz2;
  - corregir el fallo probable de grafo/propagacion en la optimizacion fiducial
    grande del dron antihorario;
  - comprobar con una prueba corta si el tramo entre fiducial 2 y fiducial 1
    mejora de forma coherente, no solo en el KF fiducial objetivo.
- archivos de codigo modificados:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp`;
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `simulacion_dron/launch/multi_dron.launch.py`.
- YAMLs de prueba:
  - se uso `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - escenario: solo `dron_2` se mueve en sentido antihorario desde fiducial 2,
    rodea por el lado de mayor deriva y llega al fiducial 1 con yaw `-90 deg`;
  - el resto queda quieto/no participa.
- cambios tecnicos implementados:
  - `PoseGraphBuilder` usa distancia acumulada de trayectoria para seleccionar
    vertices representativos;
  - se anaden vertices por vecindad del fiducial objetivo, vecindad del fiducial
    previo, cambios fuertes de yaw y subdivisiones de tramos largos;
  - `PoseGraphPropagationPlanEntry` guarda `segment_alpha`,
    `distance_from_a_m`, `segment_length_m` y `control_span_kf_gap`;
  - `OptimizationManager` usa `segment_alpha` en propagacion de KFs no vertices;
  - los checkpoints parciales de error absurdo omiten propagacion amplia,
    loggean `[F1L-PARTIAL-PROPAGATION-SKIP]` y no dejan
    `last_correction_set=true`;
  - `global_map_server` expone nuevos parametros del builder y loggea cobertura,
    esquinas y segmentos de propagacion.
- paquetes compilados:
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - `simulacion_dron`: `BUILD-EXIT-CODE 0`.
- estrategia de build:
  - se compilaron paquetes en llamadas separadas, manteniendo paquetes pesados
    acotados segun AGENTS.md.
- prueba Gazebo ejecutada:
  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false f1j_dryrun_enabled:=true f1k_apply_enabled:=true f1l_validation_enabled:=true f1l_partial_apply_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0 f1l_debug_animation_enabled:=true" --post-scenario-wait-sec 20 --startup-wait-sec 20 --timeout-sec 900 --max-gazebo-retries 1
  ```
- resultado de simulacion:
  - `SCENARIO-RUNNER-DONE success=true`;
  - `SIM-DONE prueba=1 success=true`;
  - `SIM-EXIT-CODE 0`;
  - `4` goals enviados a `/dron_2/AccionTrayectoria`, todos con
    `success=true`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|SIM-|F1L-GT-|F1L-DEBUG-ANIMATION|F1H-FID-TASK-CREATED|F1I-GRAPH|F1I-FID-CONNECTIVITY|F1J-OPT|F1K-|F1L-|RAWDB-POSE-OVERWRITE-BY-OPT|HARD-FIDUCIAL-MOVED|APPLY-USEFUL-FALSE|ROLLBACK_FAILED|POST_APPLY_ACCEPT_WITH_ERROR_WORSE|ERROR|FATAL|Segmentation fault|Killed|process has died
  ```
- evidencia positiva:
  - aparecen `[F1I-GRAPH-VERTEX-COVERAGE]`, `[F1I-GRAPH-CORNER-VERTEX]` y
    `[F1I-GRAPH-PROPAGATION-SEGMENT]`;
  - se generaron `f1l_debug_animation_task_1.html` a
    `f1l_debug_animation_task_8.html`;
  - los casos parciales grandes muestran
    `[F1L-PARTIAL-PROPAGATION-SKIP] skipped_propagated_kfs>0`;
  - `[F1K-OPT-APPLY-SUMMARY]` muestra `propagated_kfs=0`,
    `last_correction_set=false`, `raw_db_modified=false` y
    `hard_fixed_moved=false` en esos checkpoints;
  - no aparecen `ROLLBACK_FAILED`, `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`,
    `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED`, `FATAL`,
    `Segmentation fault` ni `Killed` como fallo grave de sistema.
- evidencia negativa:
  - los casos grandes del fiducial 1 siguen terminando en
    `[F1L-POST-APPLY-REJECT] reason=deformable_edges_broken_without_safe_partial`;
  - ejemplos: `before_t=26.843055 real_after_t=0.034834`, y tareas posteriores
    con `before_t` alrededor de `27-29 m` y `real_after_t` alrededor de
    `0.03-0.04 m`, rechazadas;
  - `[F1L-GT-WINDOW-STATS]` muestra que la ventana completa sigue lejos:
    task 1 `mean_before=9.281666`, `max_before=26.843055`,
    `mean_after=8.697974`, `max_after=23.345369`;
  - el log completo muestra que aun puede quedar algun tramo de propagacion con
    `segment_length_m` mayor que el limite deseado cuando `max_vertices=24` se
    agota.
- conclusion:
  - `PARCIAL`.
- razon de la conclusion:
  - la seguridad de parciales mejoro y ya no se publica una propagacion amplia
    peligrosa en checkpoints, pero no hay evidencia de que el mapa completo de
    la ventana fiducial-to-fiducial quede corregido en RViz2/logs;
  - no se puede marcar `CONSEGUIDA` porque el criterio exige mejora coherente de
    la ventana completa y no solo del residual fiducial objetivo.
- documentacion actualizada:
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/paquetes/simulacion_dron/launches.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- siguiente paso recomendado:
  - mantener `1L` como actual;
  - convertir la cobertura del grafo en invariante dura antes del solver:
    rechazar con `[F1I-GRAPH-REJECT] reason=bad_window_coverage` o aumentar
    vertices adaptativamente si quedan tramos largos/gaps excesivos;
  - despues reejecutar una sola prueba corta con debug GT/HTML y confirmar en
    RViz2 que los KFs de ambos fiduciales y los intermedios quedan coherentes.

## 2026-07-12 — Subfase 1L — cobertura dura y prueba `dron_2` antihoraria

- objetivo intentado:
  - ejecutar `1L` en la prueba aislada `dron_2` desde fiducial 2 hasta fiducial
    1 en sentido antihorario;
  - convertir la cobertura insuficiente del grafo en invariante dura antes del
    solver;
  - comparar GT de KeyFrames contra poses post-optimizacion sin usar GT para
    decidir runtime.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - documentacion de estado, subfase, historial y paquetes.
- YAMLs:
  - se reutilizo `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
  - no fue necesario modificarlo.
- cambios tecnicos:
  - se anadio `PoseGraphCoverageSummary` a `PoseGraphProblem`;
  - `PoseGraphBuilder` calcula cobertura de ventana, controles y segmentos;
  - el builder puede superar adaptativamente `max_vertices` nominal para cubrir
    la ventana;
  - si quedan tramos largos sin control o faltan vertices obligatorios, el grafo
    falla antes del solver con `bad_window_coverage`;
  - `global_map_server` loggea `[F1I-GRAPH-REJECT]` con metricas de cobertura.
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
  - task 1 creo grafo cubierto:
    `vertices=24`, `max_edge_kf_gap=12`, `max_edge_length_m=0.451`,
    `uncovered_long_segments=0`, `coverage_complete=true`;
  - task 1 acepto una correccion pequena:
    `[F1L-POST-APPLY-ACCEPT] real_after_t=0.001837`;
  - task 2 creo una primera pasada cubierta aunque supero el presupuesto
    nominal: `vertices=25`, `max_vertices=24`,
    `uncovered_long_segments=0`, `coverage_complete=true`;
  - task 2 redujo el fiducial objetivo de `23.602545 m` a `3.602548 m`;
  - la vecindad del fiducial previo no se desanclo:
    `previous_fid_neighborhood_mean_before=0.330278 mean_after=0.330278`;
  - la segunda pasada desde checkpoint parcial fue bloqueada antes del solver:
    `[F1I-GRAPH-REJECT] reason=bad_window_coverage ... uncovered_long_segments=3`;
  - no aparecieron eventos reales de `ROLLBACK_FAILED`,
    `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `RAWDB-POSE-OVERWRITE-BY-OPT`,
    `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE`, `FATAL`,
    `Segmentation fault` ni `Killed`.
- evidencia negativa o ausente:
  - el fiducial objetivo de task 2 quedo por encima del umbral:
    `real_after_t=3.602548 threshold_t=0.350000`;
  - la ventana completa siguio mal segun GT debug:
    `mean_before=9.100249`, `mean_after=8.651571`,
    `max_after=22.521755`;
  - no hay evidencia suficiente para declarar que el submapa se deforma de
    forma coherente entre fiducial 2 y fiducial 1.
- conclusion:
  - `PARCIAL`.
- siguiente paso recomendado:
  - mantener `1L` como subfase actual;
  - convertir `bad_window_coverage` en una peticion de refinamiento efectiva:
    anadir controles en los segmentos rechazados o elevar el presupuesto real
    hasta cubrir la ventana;
  - reejecutar la misma prueba `dron_2` antihoraria y exigir mejora clara de
    `[F1L-GT-WINDOW-STATS]`, no solo del KF fiducial objetivo.

## 2026-07-12 — Subfase 1L — semilla elastica y prueba `dron_2` antihoraria
