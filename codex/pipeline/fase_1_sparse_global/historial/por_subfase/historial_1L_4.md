## 2026-07-13 12:38 — Subfase 1L — soporte de KeyFrames reforzado

- objetivo intentado:
  - aumentar la diferencia de score entre zonas con muchos KeyFrames alrededor
    y zonas pobres;
  - proteger mejor corners con soporte, especialmente la curva alrededor de
    `kf=135`;
  - mantener todavía más fija la cola del fiducial 1.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - documentación de estado, paquete, subfase e historial.
- cambio de código:
  - aristas con soporte usan `support_rigidity_multiplier^0.5` en sus residuos
    SE(3);
  - bisagras/corners usan `support_rigidity_multiplier^1.2`;
  - `kCornerHingeResidualScale=3.0`;
  - se conserva cola rígida del target con rigidez `100.0`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline sobre `codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`;
  - HTML activo:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_support_mild_1.html`;
  - logs de variantes:
    `/tmp/f1l_so3_support_stronger_1.log`,
    `/tmp/f1l_so3_support_stronger_2.log`,
    `/tmp/f1l_so3_support_hinge_1.log`,
    `/tmp/f1l_so3_support_mild_1.log`.
- patrones de reducción:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `support_mild_1`: `success=true`, `useful=true`, `iterations=2`;
  - target fiducial en pose 6D exacta:
    `after_translation=0`, `after_rotation=0`;
  - cola final sigue casi bloqueada:
    - `159->160`: `delta_map_xy_len=0.00258464`;
    - `160->161`: `delta_map_xy_len=0.00560968`;
    - `161->162`: `delta_map_xy_len=0.0190939`;
  - `worsened_kfs=47`, uno menos que `target_tail_locked_2`.
- evidencia negativa o ausente:
  - `support_mild_1` no mejora a `target_tail_locked_2`:
    `mean_after=2.12554 -> 2.12846`,
    `max_after=4.722 -> 4.72425`;
  - `kf=135` apenas cambia:
    `after_turn=0.942368` en `target_tail_locked_2`,
    `after_turn=0.940466` en `support_mild_1`,
    lejos del `before_turn=1.48931`;
  - variantes más agresivas:
    - `support_stronger_1`: `mean_after=2.12348`, `max_after=4.72486`,
      pero `useful=false`;
    - `support_stronger_2`: `mean_after=2.14131`, `max_after=4.788`,
      `useful=false`;
    - `support_hinge_1`: `mean_after=2.20681`, `max_after=4.67594`,
      `useful=false`;
  - no hay validación Gazebo/apply live posterior.
- conclusión:
  - `PARCIAL`;
  - reforzar el soporte de KeyFrames está implementado y es estable en versión
    moderada, pero no corrige el corner de `kf=135`. El problema no parece
    resolverse solo con pesos.
- siguiente paso recomendado:
  - inspeccionar `support_mild_1` frente a `target_tail_locked_2`;
  - si la curva de `kf=135` sigue visualmente mal, añadir más vértices/control
    en ese tramo o una restricción geométrica explícita de corner/tramo fiable.

## 2026-07-13 13:02 — Subfase 1L — cola relativa del fiducial 1 y arista densa 63-77

- objetivo intentado:
  - explicar por qué `63->77` se deformaba pese a tener muchos KeyFrames;
  - fijar las aristas cercanas al fiducial 1 de forma equivalente al lado del
    fiducial previo;
  - mantener las zonas densas como bloques rígidos y desplazar la deformación a
    tramos pobres.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - documentación de estado, paquete, subfase e historial.
- cambio de código:
  - se probó primero bloquear aristas densas solo con pesos; el replay
    `rigid_dense_tail_4` quedó `useful=false`, con coste alto y `sign_flipped`
    en `kf=77`;
  - se sustituyó por un bloqueo relativo explícito de la cola del target:
    `BuildTargetTailPoseLocks` reconstruye poses esperadas hacia atrás desde el
    prior 6D del `FiducialTarget` y añade residuos fuertes de pose para las
    últimas `3` aristas.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline sobre `codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`;
  - HTML activo:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_target_tail_pose_lock_5.html`;
  - log:
    `/tmp/f1l_so3_target_tail_pose_lock_5.log`.
- patrones de reducción:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `success=true`, `useful=true`, `iterations=35`;
  - target fiducial en pose 6D exacta:
    `after_translation=0`, `after_rotation=0`;
  - GT diagnóstico mejora fuerte frente a `support_mild_1`:
    `mean_after=2.12846 -> 1.15554`,
    `max_after=4.72425 -> 2.57206`;
  - la arista densa `63->77` queda casi inmóvil:
    `delta_map_len=0.0111595`, `delta_rel_yaw=0.0000784569`;
  - la cola del fiducial 1 queda casi bloqueada en longitud 3D:
    - `159->160`: `delta_len=0.000164341`;
    - `160->161`: `delta_len=0.000252363`;
    - `161->162`: `delta_len=0.00079152`.
- evidencia negativa o ausente:
  - la deformación se concentra ahora en `110->112`:
    `delta_len=2.52799`;
  - `kf=110` aparece con `sign_flipped=true`;
  - aún no hay validación Gazebo/apply live.
- conclusión:
  - `PARCIAL`;
  - es la mejor variante offline reciente y resuelve las dos observaciones del
    usuario en el log, pero requiere inspección visual antes de simular.
- siguiente paso recomendado:
  - revisar `f1l_offline_graph_task_1_so3_target_tail_pose_lock_5.html`;
  - si la concentración `109..112` es aceptable, repetir Gazebo/apply;
  - si no, añadir subdivisión/control en la zona pobre `109..112`.

## 2026-07-13 13:15 — Subfase 1L — cola del target con posición relativa global

- objetivo intentado:
  - evitar que las aristas `159->160->161->162` roten visualmente respecto al
    fiducial 1;
  - no fijar poses absolutas de esos vértices, solo su posición relativa al
    `FiducialTarget`.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - documentación de estado, paquete, subfase e historial.
- cambio de código:
  - se eliminó el bloqueo de pose completa de vértices de cola;
  - se añadió `BuildTargetTailPositionLocks`, que conserva el offset global
    inicial de cada vértice de las últimas `3` aristas respecto al target;
  - escala activa: `kTargetTailRelativePositionLockScale=500.0`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline sobre `codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`;
  - HTML activo:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_target_tail_relative_position_lock_8.html`;
  - log:
    `/tmp/f1l_so3_target_tail_relative_position_lock_8.log`.
- patrones de reducción:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `success=true`, `useful=true`, `iterations=3`;
  - target fiducial en pose 6D exacta:
    `after_translation=0`, `after_rotation=0`;
  - `161->162` mejora visualmente frente a `_5`:
    `delta_map_xy_len=0.0232208` en `_8`;
  - `110` ya no aparece con `sign_flipped=true` en `_8`.
- evidencia negativa o ausente:
  - `_8` empeora GT frente a `_5`:
    `mean_after=1.15554 -> 2.18844`,
    `max_after=2.57206 -> 4.41718`;
  - `63->77` vuelve a deformarse más que en `_5`:
    `delta_map_len=0.7043`;
  - `_9` con escala `1500.0` queda descartada:
    `mean_after=2.58823`, `max_after=5.44372` y más deformación de cola;
  - no hay validación Gazebo/apply live.
- conclusión:
  - `PARCIAL`;
  - `_8` responde mejor a la petición visual de la cola del fiducial 1, pero
    `_5` sigue siendo mejor como solución global.
- siguiente paso recomendado:
  - comparar visualmente `_5` y `_8`;
  - si se prioriza rigidez visual cerca del fiducial 1, continuar desde `_8`;
  - si se prioriza error global, volver a `_5`.

## 2026-07-13 13:18 — Subfase 1L — restaurar `_5` como línea activa

- objetivo intentado:
  - aplicar la decisión del usuario: `_5` es bastante mejor que `_8`.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - documentación de estado, paquete, subfase e historial.
- cambio de código:
  - se elimina la línea activa basada en offset global de posición;
  - se restaura `BuildTargetTailPoseLocks`;
  - constantes activas:
    `kTargetTailPoseLockTranslationScale=180.0`,
    `kTargetTailPoseLockRotationScale=35.0`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline de verificación:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_target_tail_pose_lock_restored_10.html`;
  - log:
    `/tmp/f1l_so3_target_tail_pose_lock_restored_10.log`.
- patrones de reducción:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `success=true`, `useful=true`, `iterations=35`;
  - target fiducial en pose 6D exacta:
    `after_translation=0`, `after_rotation=0`;
  - GT diagnóstico restaurado:
    `mean_after=1.15554`, `max_after=2.57206`;
  - arista densa `63->77` casi inmóvil:
    `delta_map_len=0.0111595`;
  - cola final con longitud 3D casi fija:
    `159->160=0.000164341`,
    `160->161=0.000252363`,
    `161->162=0.00079152`.
- evidencia negativa o ausente:
  - el HTML conserva el defecto visual ya conocido en la última arista;
  - no hay validación Gazebo/apply live.
- conclusión:
  - `PARCIAL`;
  - `_5`/`restored_10` vuelve a ser la línea activa recomendada.
- siguiente paso recomendado:
  - usar `_5` como base;
  - si se quiere corregir el detalle visual de la última arista, probar otra
    parametrización distinta, no `_8/_9`.

## 2026-07-14 — Subfase 1L — vecindades fiduciales simétricas por locks relativos

- objetivo intentado:
  - aplicar el mismo tratamiento a ambos fiduciales;
  - no fijar en absoluto todos los vértices cercanos al fiducial previo;
  - conservar las aristas cercanas a cada fiducial mediante poses relativas
    fuertes respecto al propio fiducial.
- archivos modificados:
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md`.
- cambio de código:
  - `PoseGraphBuilder` deja fijo solo el `previous_fiducial_anchor`; los
    `previous_fiducial_neighborhood` pasan a variables etiquetadas;
  - `OptimizationManager` añade `IsHardFixedVertex` para separar hard fiducials
    de vecindades;
  - se añade `BuildPreviousAnchorTailPoseLocks`, que recorre hacia delante desde
    el ancla previa y reconstruye poses esperadas usando las aristas SE(3);
  - el coste y el vector de residuales aplican esos locks con las mismas escalas
    que `BuildTargetTailPoseLocks`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos (`dry_run`, `CollectAffectedKeyFrames`,
    `RotationError`).
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline sobre `codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`;
  - HTML:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_symmetric_fiducial_tail_lock_12.html`;
  - log:
    `codex/archivos_auxiliares/f1l_so3_symmetric_fiducial_tail_lock_12.log`.
- patrones de reducción:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `success=true`, `useful=true`, `iterations=35`;
  - target fiducial en pose 6D exacta:
    `after_translation=0`, `after_rotation=0`;
  - GT diagnóstico:
    `mean_after=1.20049`, `max_after=2.56876`;
  - vecindad del fiducial previo casi inmóvil en longitud:
    - `55->56`: `delta_len=-0.0000005`;
    - `56->57`: `delta_len=-0.000657`;
    - `57->58`: `delta_len=-0.000836`;
  - cola del fiducial 1 sigue casi bloqueada en longitud 3D:
    - `159->160`: `delta_len=0.000164`;
    - `160->161`: `delta_len=0.000252`;
    - `161->162`: `delta_len=0.000792`.
- evidencia negativa o ausente:
  - no mejora la media GT de la referencia `_5`/`restored_10`
    (`1.20049` frente a `1.15554`);
  - sigue habiendo deformaciones visibles en corners internos:
    `kf=110 sign_flipped=true`, `kf=160 delta_turn=-0.310429`,
    `kf=161 delta_turn=0.878307`;
  - no hay validación Gazebo/apply live.
- conclusión:
  - `PARCIAL`;
  - la aplicación simétrica funciona y queda reproducible como `_12`;
  - no sustituye todavía a `_5`/`restored_10` como mejor base global.
- siguiente paso recomendado:
  - inspeccionar visualmente `_12` frente a `_5`;
  - si interesa mantener la simetría por coherencia conceptual, mejorar la
    deformación de corners internos antes de repetir Gazebo;
  - no declarar `1L` conseguida sin replay superior y validación live.

## 2026-07-14 11:24 — Subfase 1L — vecinos con pose inducida desde ambos fiduciales

- objetivo intentado:
  - corregir `_12`, donde las vecindades aún se trataban como variables con
    residual fuerte;
  - fijar ambos fiduciales y anclar los vecinos cercanos a la pose inducida por
    su transformada relativa ORB-SLAM3 respecto al fiducial correspondiente.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md`.
- cambio de código:
  - nuevo `BuildFiducialNeighborhoodPoseLocks`;
  - `CollectVariableVertexIds` excluye los vecinos con pose inducida;
  - `PinFiducialNeighborhoodPoseLocks` pincha esas poses en `base_states` antes
    del solve;
  - la semilla del solver sigue usando el estado original para no perder la
    inicialización elástica;
  - `test_opt_graph_offline` marca en naranja los vecinos bloqueados por pose
    inducida para que el HTML muestre ambos extremos igual.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - build final sin stderr de compilador; solo warnings de entorno/underlay de
    `colcon`.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline sobre `codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`;
  - HTML:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_fiducial_neighborhood_pose_locks_13.html`;
  - log:
    `codex/archivos_auxiliares/f1l_fiducial_neighborhood_pose_locks_13.log`.
- patrones usados para reducir logs:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `success=true`, `useful=true`, `iterations=35`;
  - target fiducial en pose 6D exacta:
    `after_translation=0`, `after_rotation=0`;
  - GT diagnóstico mejora frente a `_5/restored_10` y `_12`:
    `mean_after=0.822023`, `max_after=1.78632`;
  - fiducial previo rígido por aristas:
    - `55->56`: `delta_len=4.44e-16`;
    - `56->57`: `delta_len=-6.66e-16`;
    - `57->58`: `delta_len=2.22e-16`;
  - fiducial objetivo rígido por aristas:
    - `159->160`: `delta_len=3.11e-15`;
    - `160->161`: `delta_len=8.88e-16`;
    - `161->162`: `delta_len=7.77e-16`;
  - HTML final marca `159`, `160`, `161` y `162` como bloque fiducial igual
    que el extremo del fiducial previo.
- evidencia negativa o ausente:
  - no hay validación Gazebo/apply live;
  - sigue apareciendo `kf=110 sign_flipped=true`;
  - la arista `110->112` concentra deformación grande:
    `delta_len=3.06004`.
- conclusión:
  - `PARCIAL`;
  - `_13` pasa a ser la mejor referencia offline actual y representa la idea
    pedida por el usuario.
- siguiente paso recomendado:
  - inspeccionar visualmente `_13`;
  - si el tramo central `109..112` es aceptable, repetir Gazebo/apply;
  - si no, añadir control/subdivisión o regla específica en esa zona antes de
    validar live.

## 2026-07-14 12:45 — Subfase 1L — publicación de MapPoints desde pose final del KF

- objetivo intentado:
  - hacer que RViz2 siga la deformación de KeyFrames que muestra el HTML;
  - evitar que la nube se mueva como una corrección rígida de submapa cuando el
    grafo optimizado está bien.
- archivos modificados:
  - `orbslam3_multi/src/global_map_builder.cpp`;
  - `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md`.
- cambio de código:
  - `GlobalMapBuilder` ya no publica aplicando una corrección elegida por
    MapPoint más fallback de última corrección de submapa;
  - para cada MapPoint elige un KF publicable: primero
    `reference_keyframe_id`, después observadores con pose en
    `GlobalPoseStore`;
  - calcula `p_kf = raw_local_T_kf^-1 * p_raw`;
  - publica `p_world = final_world_T_kf * p_kf`;
  - si no hay KF publicable, conserva la transformación anclada básica
    `world_T_local * p_raw`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build:
  - ambos comandos terminaron con `BUILD-EXIT-CODE 0`.
- pruebas Gazebo/replay:
  - Gazebo `prueba_1` con `f1l_gt_kf_debug_enabled`,
    `f1l_debug_animation_enabled` y `f1l_graph_dump_enabled`;
  - comando base:
    ```text
    ./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py f1l_gt_kf_debug_enabled:=true f1l_debug_animation_enabled:=true f1l_graph_dump_enabled:=true f1l_debug_animation_output_dir:=/home/chenfu/Gazebo/src/codex/archivos_auxiliares f1l_graph_dump_output_dir:=/home/chenfu/Gazebo/src/codex/archivos_auxiliares/repeticiones" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1400 --max-gazebo-retries 1
    ```
  - resultado: `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|SIM-|F1H-|F1I-|F1J-|F1K-|F1L-|F1F-GLOBALMAP|GT-WINDOW|FATAL|ERROR|Segmentation fault|Killed|process has died
  ```
- evidencia positiva:
  - HTML live:
    `codex/archivos_auxiliares/html/f1l_debug_animation_task_1.html`;
  - dump live:
    `codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`;
  - `[F1L-GRAPH-DUMP] success=true`, `vertices=25`, `propagation=82`,
    `debug_kfs=107`;
  - `[F1L-DEBUG-ANIMATION-EXPORT] success=true`,
    `optimized_or_propagated=107`;
  - `[F1L-POST-APPLY-ERROR] before_t=11.640776`,
    `real_after_t=0.000000`;
  - `[F1L-POST-APPLY-INTERNAL-ERROR] ok=true`,
    `internal_max_after=0.173890`;
  - `[F1L-POST-APPLY-PROPAGATION-CHECK] ok=true`,
    `propagated_count=82`, `skipped_propagated_count=0`,
    `propagation_discontinuity_max_t=0`;
  - `[F1L-POST-APPLY-GLOBALMAP-CHECK] ok=true`,
    `published_points_after=19612`, `server_corrected_after=11359`,
    `nan_points=0`;
  - `[F1L-POST-APPLY-ACCEPT] decision=ACCEPT`;
  - publicación final observada con `20786` puntos y `11358` corregidos, no
    todos los puntos del submapa.
- evidencia negativa o ausente:
  - la validación visual de RViz2 la debe confirmar el usuario;
  - `server_corrected_after` sigue siendo alto porque muchos MapPoints pertenecen
    a KFs optimizados/propagados, pero ya no coincide con todos los publicados;
  - Gazebo emitió `process has died exit code 255` durante cleanup tras
    `SIM-DONE`; el wrapper terminó con `SIM-EXIT-CODE 0`.
- conclusión:
  - `PARCIAL`;
  - la publicación ya usa poses finales de KFs y elimina el fallback global que
    podía arrastrar el submapa como bloque;
  - falta inspección visual en RViz2 antes de cerrar `1L`.
- siguiente paso recomendado:
  - observar RViz2 con esta simulación y comparar con
    `f1l_debug_animation_task_1.html`;
  - si aún hay puntos del fiducial previo desplazados, añadir logs acotados de
    `reference_kf`, `selected_kf` y `correction/source` por MapPoint en esa zona.

## 2026-07-14 13:05 — Subfase 1L — herencia de KFs futuros desde el extremo optimizado

- objetivo intentado:
  - corregir que los KeyFrames creados después de la optimización no siguieran
    la corrección nueva del extremo optimizado.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md`.
- cambio de código:
  - nuevo método `SetSubmapLastServerCorrectionFromKeyFrame`;
  - `ApplyCandidateResult` marca como herencia futura el KF aplicado con mayor
    `local_kf_id`;
  - `SubmapCorrection` conserva `from_keyframe`;
  - `RestoreApplyBackup` restaura también el KF fuente;
  - `ReconcileAfterRawIngestResult` no sustituye la corrección heredable con un
    KF optimizado anterior al KF fuente actual.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build:
  - ambos comandos terminaron con `BUILD-EXIT-CODE 0`;
  - warnings conocidos no bloqueantes en `optimization_manager.cpp`.
- pruebas Gazebo/replay:
  - se ejecutaron dos simulaciones tras el primer cambio;
  - la primera mostró mejora parcial: `kf=166` heredó
    `SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT`, pero `kf=167` volvió a
    `F1G_FULL_SNAPSHOT_RECONCILE`;
  - tras proteger `from_keyframe`, se repitió Gazebo `prueba_1`;
  - HTML final:
    `codex/archivos_auxiliares/html/f1l_debug_animation_task_2.html`;
  - dump final:
    `codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv`.
- patrones usados para reducir logs:
  ```text
  SCENARIO-RUNNER|SIM-|F1H-|F1I-|F1J-|F1K-|F1L-|F1F-GLOBALMAP|GT-WINDOW|FATAL|ERROR|Segmentation fault|Killed|process has died
  ```
- evidencia positiva:
  - simulación final: `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - caso fuerte final: `task_id=2`;
  - `[F1K-POSESTORE-LAST-CORRECTION-SET] task_id=2 ... from_kf=202`;
  - `[F1K-OPT-APPLY-SUMMARY] ... last_correction_set=true`;
  - KFs posteriores `204..213` emitieron
    `source=SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT`;
  - `[F1L-POST-APPLY-ERROR] before_t=27.653387`, `real_after_t=0.000000`;
  - `[F1L-POST-APPLY-INTERNAL-ERROR] ok=true`,
    `internal_max_after=0.564123`;
  - `[F1L-POST-APPLY-PROPAGATION-CHECK] ok=true`,
    `propagated_count=108`, `skipped_propagated_count=0`;
  - `[F1L-POST-APPLY-GLOBALMAP-CHECK] ok=true`,
    `published_points_after=19027`, `server_corrected_after=15214`;
  - `[F1L-POST-APPLY-ACCEPT] decision=ACCEPT`.
- evidencia negativa o ausente:
  - falta confirmación visual en RViz2 de continuidad de los KFs posteriores;
  - GT diagnóstico de ventana final no cierra perfecto:
    `mean_after=3.036654`, `max_after=6.890110`, `worsened_kfs=52`;
  - el criterio runtime sigue siendo el error del fiducial objetivo, no la media
    GT de ventana.
- conclusión:
  - `PARCIAL`;
  - corregida y validada por logs la herencia de KFs posteriores al apply.
- siguiente paso recomendado:
  - revisar RViz2: los KFs posteriores al fiducial 1 deberían continuar desde
    la corrección del extremo optimizado, no volver a la transformada previa;
  - si visualmente queda bien, el siguiente trabajo de `1L` es decidir si el
    error GT diagnóstico de la ventana es aceptable para cerrar o si requiere
    más refinamiento.

## 2026-07-14 13:40 — Subfase 1L — herencia persistente y sin fallback rígido de MapPoints

- objetivo intentado:
  - corregir los pocos MapPoints que seguían apareciendo tras la optimización
    con transformada antigua;
  - asegurar que los KFs posteriores al último KF optimizado sobreviven a full
    snapshots sin volver al anchor base del submapa;
  - seleccionar vecinos fiduciales por distancia métrica, no por un número fijo
    de vertices.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp`;
  - `orbslam3_multi/src/global_map_builder.cpp`;
  - `orbslam3_multi/src/global_pose_store.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - documentación de paquetes, subfase, resúmenes e historial.
- cambios de código:
  - `PoseGraphBuilder` usa `pose_graph_fiducial_neighborhood_radius_m=4.0`
    para seleccionar vecindad fiducial por distancia 3D de pose mapa;
  - `BuildTargetTailPoseLocks` y `BuildPreviousAnchorTailPoseLocks` usan esas
    vecindades métricas para pinchar poses inducidas desde cada fiducial;
  - `GlobalPoseStore::RegisterNewKeyFrameIfAnchored` guarda los KFs futuros
    heredados también en `optimized_keyframes_`;
  - `ReconcileAfterRawIngestResult` reaplica la última corrección heredable a
    KFs posteriores al `from_keyframe` y evita rebasarlos al anchor;
  - `GlobalMapBuilder` salta MapPoints sin KF publicable si el submapa tiene
    corrección de servidor, en vez de publicar con `world_T_local`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`.
- pruebas Gazebo/replay:
  - Gazebo `prueba_1` con `f1l_gt_kf_debug_enabled`,
    `f1l_debug_animation_enabled` y `f1l_graph_dump_enabled`;
  - resultado: `SIM-DONE prueba=1 success=true`, `SIM-EXIT-CODE 0`;
  - HTML final:
    `codex/archivos_auxiliares/html/f1l_debug_animation_task_2.html`;
  - dump final:
    `codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv`.
- patrones de reducción:
  ```text
  SCENARIO-RUNNER|SIM-|F1H-|F1I-|F1J-|F1K-|F1L-|F1F-GLOBALMAP|GT-WINDOW|FATAL|ERROR|Segmentation fault|Killed|process has died
  ```
- evidencia positiva:
  - `[F1L-FIDUCIAL-NEIGHBORHOOD-CONFIG] radius_m=4.000`;
  - `[F1L-GRAPH-DUMP] task_id=2 ... vertices=28 edges=27 propagation=98`;
  - `[F1L-DEBUG-ANIMATION-EXPORT] task_id=2 ... optimized_or_propagated=126`;
  - `[F1K-POSESTORE-LAST-CORRECTION-SET] task_id=2 ... from_kf=200`;
  - `[F1K-OPT-APPLY-SUMMARY] task_id=2 ... last_correction_set=true`;
  - KFs `202..208` emitieron
    `source=SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT`;
  - `[F1L-POST-APPLY-ERROR] before_t=26.641246 real_after_t=0.000000`;
  - `[F1L-POST-APPLY-PROPAGATION-CHECK] ok=true propagated_count=98 skipped_propagated_count=0`;
  - `[F1L-POST-APPLY-GLOBALMAP-CHECK] ok=true published_points_after=20815 server_corrected_after=14917`;
  - `[F1L-GLOBALMAP-KF-PROJECTION] keyframe_projected_after=20815 fallback_submap_after=0`;
  - `[F1L-POST-APPLY-ACCEPT] decision=ACCEPT`.
- evidencia negativa o ausente:
  - falta confirmación visual del usuario en RViz2;
  - GT diagnóstico de ventana queda `mean_after=2.646385`,
    `max_after=4.821657`, `worsened_kfs=67`;
  - esa media GT no decide runtime, pero sigue siendo deuda de calidad.
- conclusión:
  - `PARCIAL`;
  - la propagación posterior y la publicación sin fallback rígido quedan
    corregidas por logs;
  - no cerrar `1L` hasta confirmar visualmente RViz2 o definir el siguiente
    refinamiento.
- siguiente paso recomendado:
  - observar RViz2 con `f1l_debug_animation_task_2.html` al lado;
  - si quedan puntos sueltos, loggear `reference_keyframe_id`,
    observador elegido y source de pose por MapPoint en esa zona;
  - si RViz2 ya coincide con HTML, decidir si el error GT diagnóstico restante
    es aceptable para cerrar `1L` o si se requiere fiducial intermedio/loops.
