## 2026-07-13 00:10 — Subfase 1L — pruebas sparse_texture rechazadas

- objetivo intentado:
  - probar la hipótesis de que aristas con pocos KFs y baja textura deben pesar
    menos en distancia, para permitir que `127->129` se ensanche sin esperar a
    una nueva simulación.
- archivos modificados durante las pruebas:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`.
- estado final de código:
  - se revirtió a la mejor variante conocida;
  - valores activos finales:
    `kSparseEdgeTranslationMinMultiplier=0.08`,
    `kSparseEdgeTranslationMaxRelax=0.55`,
    `kEdgeGlobalVerticalResidualScale=8.0`,
    `kCornerHingeStraightMultiplier=0.25`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - todas las compilaciones terminaron con `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - se ejecutaron cuatro replays offline sobre:
    `codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv`;
  - HTMLs generados:
    - `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_sparse_texture_1.html`;
    - `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_sparse_texture_2.html`;
    - `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_sparse_texture_3.html`;
    - `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_sparse_texture_4.html`.
- patrones usados para reducir logs:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-EDGE-SUPPORT-EDGE|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - la hipótesis tiene efecto mecánico: al bajar más la rigidez, `127->129`
    sí se abre;
  - mejor apertura: variante `_1`, `after_map_xy_len=3.6081` frente a
    `before_map_xy_len=1.81843` y `gt_map_xy_len=4.90851`.
- evidencia negativa o ausente:
  - variante `_1`: `mean_after=4.25394`, `max_after=9.86848`,
    `kf143 after_turn=1.3705`, `kf161 after_turn=2.04977`;
  - variante `_2`: `mean_after=3.92155`, `max_after=7.9606`,
    `127->129 after_map_xy_len=3.28107`, pero `center_kf=150 sign_flipped=true`;
  - variante `_3`: `mean_after=5.00018`, `max_after=8.82942`,
    `127->129 after_map_xy_len=2.23748`, con flips en `129/139`;
  - variante `_4`: `mean_after=5.01575`, `max_after=8.88018`,
    `127->129 after_map_xy_len=2.19586`, con flips en `129/139`;
  - todas son peores que la variante activa `reliable_hinge_4`
    (`mean_after=3.28931`, `max_after=5.84293`, `kf161 sign_flipped=false`);
  - no se validó apply live.
- conclusión:
  - `PARCIAL`;
  - bajar peso de distancia por baja textura es necesario pero no suficiente:
    abre `127->129`, pero desplaza la deformación a bisagras fiables si no se
    localiza o se añade otra restricción.
- siguiente paso recomendado:
  - mantener el código en `reliable_hinge_4`;
  - diseñar una expansión local controlada para `127->129`, probablemente como
    variable/factor específico de arista pobre o como prior de escala local, y
    proteger explícitamente el tramo fiable `139->143->150->161`;
  - no repetir Gazebo hasta que el replay offline mejore `127->129` sin perder
    `mean_after` ni provocar flips.

## 2026-07-13 00:14 — Subfase 1L — reliable_straight rechazado

- objetivo intentado:
  - mantener recta la zona alrededor de `kf=161`, especialmente las aristas
    `150->161` y `161->174`, porque tienen muchos keyframes de soporte y no
    deberían formar un ángulo pronunciado.
- cambio probado:
  - refuerzo selectivo de bisagras casi rectas con mucho soporte en ambos lados:
    `support_keyframe_count >= 10`;
  - solo afectaba a rectas fiables, no a todas las rectas.
- archivos modificados durante la prueba:
  - `orbslam3_multi/src/optimization_manager.cpp`.
- estado final de código:
  - el cambio se revirtió;
  - queda activa la configuración `reliable_hinge_4`:
    `kCornerHingeStraightMultiplier=0.25`,
    `kSparseEdgeTranslationMaxRelax=0.55`,
    `kEdgeGlobalVerticalResidualScale=8.0`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - build de la prueba: `BUILD-EXIT-CODE 0`;
  - build final tras revertir: `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline:
    ```bash
    /home/chenfu/Gazebo/install/orbslam3_multi/lib/orbslam3_multi/test_opt_graph_offline \
      --graph /home/chenfu/Gazebo/src/codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv \
      --html /home/chenfu/Gazebo/src/codex/archivos_auxiliares/html/f1l_offline_graph_task_2_reliable_straight_1.html
    ```
- patrones usados para reducir logs:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - dry-run OK: `success=true`, `iterations=16`;
  - la prueba confirma que `support_keyframe_count` permite identificar la zona
    fiable alrededor de `kf161`.
- evidencia negativa o ausente:
  - GT diagnóstico empeora mucho: `mean_after=6.46072`, `max_after=11.0613`;
  - `kf161` sigue creando ángulo apreciable:
    `before_turn=0.227628`, `after_turn=0.787568`;
  - `kf143` queda peor: `after_turn=2.32395`;
  - `150->161` se comprime en XY global:
    `before_map_xy_len=1.44844`, `after_map_xy_len=0.000918934`;
  - no valida apply live.
- conclusión:
  - `PARCIAL`;
  - endurecer bisagras rectas fiables no resuelve el reparto de deformación; el
    solver necesita un mecanismo más local para absorber `127->129` sin mover
    `139->143->150->161`.
- siguiente paso recomendado:
  - mantener `reliable_hinge_4` como base;
  - formular la corrección de baja textura como factor/variable local en
    `127->129` y añadir guardas explícitas de longitud/rectitud para el tramo
    fiable alrededor de `161`, en vez de subir la bisagra globalmente.

## 2026-07-13 00:27 — Subfase 1L — edge_shape rechazado

- objetivo intentado:
  - aplicar la separación conceptual entre aristas fiables y pobres:
    proteger forma global en tramos con muchos KFs y permitir que `127->129`
    absorba más deformación por baja textura.
- cambios probados:
  - `edge_shape_1`: guard de vector XY global para aristas fiables y relajación
    sparse más fuerte;
  - `edge_shape_2`: mismo guard con peso mayor;
  - `edge_shape_3`: guard intermedio y stretch target suave para aristas pobres
    calculado solo desde incoherencia local/global, sin GT.
- archivos modificados durante la prueba:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`.
- estado final de código:
  - todos los cambios se revirtieron;
  - queda activa la configuración `reliable_hinge_4`:
    `kCornerHingeStraightMultiplier=0.25`,
    `kSparseEdgeTranslationMinMultiplier=0.08`,
    `kSparseEdgeTranslationMaxRelax=0.55`,
    `kEdgeGlobalVerticalResidualScale=8.0`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - builds de prueba: `BUILD-EXIT-CODE 0`;
  - build final tras revertir: `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replays offline generados:
    - `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_edge_shape_1.html`;
    - `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_edge_shape_2.html`;
    - `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_edge_shape_3.html`.
- patrones usados para reducir logs:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `edge_shape_1` abre `127->129`:
    `before_map_xy_len=1.81843`, `after_map_xy_len=3.04668`;
  - `edge_shape_2` protege mejor `kf161`:
    `before_turn=0.227628`, `after_turn=0.262516`;
  - las pruebas confirman que el problema se puede mover entre arista pobre y
    tramos fiables, pero no resolver con pesos simples.
- evidencia negativa o ausente:
  - `edge_shape_1`: `mean_after=6.23787`, `max_after=10.5922`,
    `kf143 after_turn=1.9297`, `kf161 after_turn=0.57229`;
  - `edge_shape_2`: `mean_after=6.49986`, `max_after=10.9196`,
    `127->129 after_map_xy_len=6.98018`, sobreexpansión;
  - `edge_shape_3`: `mean_after=6.9776`, `max_after=11.1103`,
    `127->129 after_map_xy_len=2.22276`, sigue peor que la base;
  - todas son peores que `reliable_hinge_4`
    (`mean_after=3.28931`, `max_after=5.84293`);
  - no valida apply live.
- conclusión:
  - `PARCIAL`;
  - la formulación actual por pesos/residuales adicionales no basta. Sin una
    parametrización separada, el solver mueve el error entre `127->129`,
    `143`, `150` y `161`.
- siguiente paso recomendado:
  - mantener `reliable_hinge_4`;
  - cambiar el modelo: añadir variable/factor de escala local acotado para
    aristas pobres o subtramos pobres, con constraints explícitos que mantengan
    longitud/dirección de tramos fiables; no seguir aumentando pesos globales.

## 2026-07-13 00:40 — Subfase 1L — piecewise/guards rechazados

- objetivo intentado:
  - probar una modificación más estructural tras observar que el tramo
    `131->150` y la zona cercana a fiduciales se deformaban de forma incoherente;
  - evitar que aristas fiables cerca de fiduciales absorban la corrección que
    debería concentrarse en la zona pobre `127->129`.
- cambios probados:
  - `piecewise_blocks_1`: rigidez fuerte del gradiente de traslación cerca de
    ambos fiduciales y relajación pobre más alta (`0.72`);
  - `piecewise_blocks_2`: misma rigidez de bloques, pero relajación pobre
    conservadora (`0.55`);
  - `global_xy_guard_1..2`: guardarraíl de longitud XY global para aristas con
    mucho soporte, primero fuerte y luego suave.
- archivos modificados durante la prueba:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp` solo para reflejar el valor
    de rigidez sparse en el log de la primera variante.
- estado final de código:
  - todos los cambios se revirtieron;
  - queda activa la base `reliable_hinge_4`:
    `kCornerHingeStraightMultiplier=0.25`,
    `kSparseEdgeTranslationMinMultiplier=0.08`,
    `kSparseEdgeTranslationMaxRelax=0.55`,
    `kEdgeGlobalVerticalResidualScale=8.0`;
  - replay final de control:
    `f1l_offline_graph_task_2_final_reliable_hinge_4.html`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - todas las compilaciones terminaron con `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replays offline sobre `codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv`;
  - HTMLs generados:
    - `f1l_offline_graph_task_2_piecewise_blocks_1.html`;
    - `f1l_offline_graph_task_2_piecewise_blocks_2.html`;
    - `f1l_offline_graph_task_2_global_xy_guard_1.html`;
    - `f1l_offline_graph_task_2_global_xy_guard_2.html`;
    - `f1l_offline_graph_task_2_final_reliable_hinge_4.html`.
- patrones usados para reducir logs:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `piecewise_blocks_1` confirma que bajar mucho la rigidez pobre sí abre
    `127->129`, pero de forma no controlada:
    `after_map_xy_len=7.66199`;
  - `global_xy_guard_2` abre moderadamente `127->129`
    (`after_map_xy_len=2.21313`) frente a la base (`1.53656`).
- evidencia negativa o ausente:
  - `piecewise_blocks_1`: `mean_after=6.86683`, `max_after=15.0919`,
    `150->161 after_map_xy_len=0.0040578`,
    `kf161 after_turn=2.0174`;
  - `piecewise_blocks_2`: `mean_after=7.14734`, `max_after=14.8727`,
    `150->161 after_map_xy_len=0.00616912`,
    `kf161 after_turn=1.79021`;
  - `global_xy_guard_1`: `mean_after=4.55855`, `max_after=8.16245`,
    `150->161 after_map_xy_len=0.309181`,
    `kf143 after_turn=2.27741`;
  - `global_xy_guard_2`: `mean_after=4.64556`, `max_after=8.94059`,
    flips en `kf139` y `kf150`;
  - todas son peores que `reliable_hinge_4`:
    `mean_after=3.28931`, `max_after=5.84293`,
    `kf150 sign_flipped=false`, `kf161 sign_flipped=false`;
  - no se validó apply live.
- conclusión:
  - `PARCIAL`;
  - los guards añadidos al mismo solver siguen moviendo el error entre
    `127->129`, `143`, `150` y `161`. No resuelven la causa estructural.
- siguiente paso recomendado:
  - mantener `reliable_hinge_4`;
  - no probar más pesos/guards sobre las mismas variables por vértice;
  - diseñar una parametrización nueva con variable explícita de escala/stretch
    para subtramos pobres y bloques rígidos reales para tramos fiables, o
    aumentar controles/observabilidad antes de optimizar.

## 2026-07-13 00:54 — Subfase 1L — fiducial 6D implementado, resultado parcial

- objetivo intentado:
  - hacer que el prior del fiducial objetivo obligue al KeyFrame fiducial a su
    pose absoluta completa, no solo posición+yaw;
  - añadir orientación visible en el HTML offline para no deducir hacia dónde
    mira un KF solo por la trayectoria XY.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `orbslam3_multi/src/test_opt_graph_offline.cpp`;
  - documentación de subfase/contexto/paquete.
- cambio de código:
  - `FiducialTarget` y `FiducialHard` usan ahora roll/pitch con peso completo
    (`PriorRollPitchResidualScale=1.0`) en vez de debilitarlo como prior suave;
  - la semilla elástica conserva roll/pitch parciales (`0.25`) para no arrancar
    plegando todo el grafo;
  - `test_opt_graph_offline` emite
    `[F1L-OFFLINE-TARGET-POSE-3D]` con error angular 3D before/after;
  - el HTML offline dibuja flechas de yaw en los vértices.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline sobre `codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv`;
  - HTML final con flechas:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_2_fiducial_6d_3.html`.
- patrones usados para reducir logs:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - el fiducial objetivo mejora también en orientación 3D:
    `before_rotation=2.92842`, `after_rotation=0.566355`;
  - traslación objetivo queda baja:
    `25.6951 m -> 0.0646345 m`;
  - `127->129` se abre algo:
    `before_map_xy_len=1.81843`, `after_map_xy_len=2.93945`.
- evidencia negativa o ausente:
  - la ventana global empeora frente a `reliable_hinge_4`:
    `mean_after=4.68414`, `max_after=7.94044`,
    `worsened_kfs=62`;
  - `150->161` se comprime demasiado:
    `before_map_xy_len=1.44844`, `after_map_xy_len=0.281648`;
  - aparecen flips en `kf129` y `kf139`;
  - no se validó apply live.
- conclusión:
  - `PARCIAL`;
  - la orientación del fiducial era una pieza necesaria y queda implementada,
    pero no basta: al forzar la pose 6D sin una observación intermedia, el
    solver traslada el conflicto a tramos fiables.
- siguiente paso recomendado:
  - usar el HTML con flechas para confirmar la incompatibilidad de orientación;
  - añadir una restricción local de observación fiducial/cámara o, de forma más
    robusta, colocar un fiducial intermedio en la zona pobre de la U y repetir
    la prueba.

## 2026-07-13 01:28 — Subfase 1L — solver SO(3), target fijado y replay offline

- objetivo intentado:
  - corregir el plegado donde los KFs anteriores al fiducial 1 aparecían al
    lado contrario aunque la pose 6D real del fiducial indicaba lo opuesto;
  - dejar de usar yaw inferido como restricción rotacional del solver.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md`.
- cambio de código:
  - los estados angulares del solver pasan de `roll/pitch/yaw` a
    `LogSO3(delta_R)`;
  - priors y aristas usan residual de rotación 3D completo;
  - `FiducialTarget` se excluye de variables y se fija a su prior 6D durante el
    solve;
  - se añade semilla por back-propagation desde el target usando las aristas
    SE(3) guardadas;
  - las edges que tocan `graph.target_keyframe_id` tienen rigidez local fuerte.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - primer intento falló por `RawKeyFrameId` sin `operator!=`;
  - se corrigió con `!(a == b)`;
  - build final `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replay offline sobre `codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`;
  - HTML final:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_pinned_target_rigid_edge_1.html`.
- patrones usados para reducir logs:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `success=true`, `iterations=3`;
  - fiducial target queda exactamente en su pose 6D:
    `before_translation=19.0658`, `after_translation=0`,
    `before_rotation=1.78423`, `after_rotation=0`;
  - GT diagnóstico mejora respecto al mapa inicial:
    `mean_before=7.09296`, `mean_after=2.22783`,
    `max_after=4.7393`;
  - la edge `161->162` ya no invierte yaw relativo:
    `delta_rel_yaw=0.000085648`;
  - `161->162` conserva bien XY global cerca del target:
    `after_map_xy_len=0.656313`, `gt_map_xy_len=0.627901`.
- evidencia negativa o ausente:
  - no hay validación Gazebo/apply live posterior;
  - persisten deformaciones internas y verticales:
    `161->162 delta_map_delta_z=0.311603`,
    `mean_after=2.22783` aún no cierra la ventana completa;
  - `worsened_kfs=47`.
- conclusión:
  - `PARCIAL`;
  - la causa del lado equivocado cerca del fiducial era compatible con usar yaw
    inferido y dejar negociar el target. El target 6D fijado y SO(3) completo
    corrigen esa parte, pero la subfase no queda conseguida sin prueba live ni
    reducción adicional de deformaciones internas.
- siguiente paso recomendado:
  - inspeccionar el HTML final;
  - si visualmente es aceptable, repetir Gazebo/apply con esta variante;
  - si no, reducir deformación Z/interna o probar fiducial intermedio en la U.

## 2026-07-13 01:42 — Subfase 1L — rigidez extendida cerca del fiducial 1

- objetivo intentado:
  - hacer más rígidas las aristas cercanas al fiducial 1 para conservar casi
    igual la geometría relativa local al vértice fiducial;
  - comparar offline sin repetir Gazebo usando el grafo guardado.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - documentación de estado, paquete, subfase e historial.
- cambio de código:
  - se mantiene el solver SO(3) con `FiducialTarget` fijado a pose 6D;
  - `TargetNeighborhoodEdgeRigidity` aplica una rampa desde
    `alpha=0.86` hasta `kTargetNeighborhoodEdgeRigidity=20.0`;
  - la edge que toca directamente el target usa
    `kTargetEndpointEdgeRigidity=25.0`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replays offline sobre `codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`;
  - HTMLs:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_target_near_rigid_1.html`
    y
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_target_near_rigid_2.html`.
- patrones de reducción:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `near_rigid_2`: `success=true`, `iterations=3`;
  - target fiducial: `after_translation=0`, `after_rotation=0`;
  - mejora frente a rigidizar solo la edge final:
    `mean_after=2.22783 -> 2.17031`,
    `max_after=4.7393 -> 4.43241`;
  - edge `161->162`: `after_map_xy_len=0.678069`,
    `gt_map_xy_len=0.627901`, `delta_rel_yaw=-0.00016249`.
- evidencia negativa o ausente:
  - no hay validación Gazebo/apply live posterior;
  - `mean_after=2.17031` sigue peor que el primer solver SO(3) no fijado
    estrictamente (`2.05936`), aunque aquel no dejaba el target 6D exacto;
  - persisten deformaciones internas/Z.
- conclusión:
  - `PARCIAL`;
  - la rigidez extendida cerca del fiducial 1 mejora el replay offline y queda
    activa, pero no basta para cerrar `1L` sin validación live.
- siguiente paso recomendado:
  - inspeccionar
    `f1l_offline_graph_task_1_so3_target_near_rigid_2.html`;
  - si visualmente conserva bien el tramo final, decidir entre repetir Gazebo
    con esta variante o añadir observabilidad/fiducial intermedio para la U.

## 2026-07-13 01:52 — Subfase 1L — perfil tipo U de rigidez en ambos extremos

- objetivo intentado:
  - implementar una rigidez SE(3) de aristas tipo U para que los vértices
    cercanos a ambos fiduciales cambien muy poco en relativa local;
  - mantener flexible el centro del tramo.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - documentación de estado, paquete, subfase e historial.
- cambio de código:
  - `TargetNeighborhoodEdgeRigidity` pasa a
    `EndpointNeighborhoodEdgeRigidity`;
  - edges directas a `graph.target_keyframe_id` o a
    `previous_fiducial_anchor` usan rigidez `25.0`;
  - aristas cercanas a cualquier extremo usan una rampa desde `alpha=0.86`
    hasta rigidez `20.0`;
  - la rigidez se aplica a residuos de traslación y rotación SE(3) de aristas.
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
  - HTML:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_endpoint_u_rigid_1.html`;
  - log:
    `/tmp/f1l_so3_endpoint_u_rigid_1.log`.
- patrones de reducción:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `success=true`, `iterations=2`;
  - target fiducial queda en pose 6D exacta:
    `after_translation=0`, `after_rotation=0`;
  - las primeras edges cercanas al fiducial previo quedan prácticamente
    inmóviles, por ejemplo `55->56` con `delta_len=0`,
    `delta_rel_yaw=0`.
- evidencia negativa o ausente:
  - empeora frente a `target_near_rigid_2`:
    `mean_after=2.17031 -> 2.21057`,
    `max_after=4.43241 -> 4.73642`;
  - aparece una inversión de esquina:
    `center_kf=77`, `sign_flipped=true`;
  - no hay validación Gazebo/apply live posterior.
- conclusión:
  - `PARCIAL`;
  - la idea tipo U es coherente con el objetivo, pero aplicada como rigidez
    fuerte de aristas en ambos extremos desplaza el error al tramo
    `63->77->88` y no mejora la mejor variante offline.
- siguiente paso recomendado:
  - inspeccionar el HTML `endpoint_u_rigid_1`;
  - si visualmente confirma el flip en `kf=77`, volver a la variante
    `target_near_rigid_2` o probar una U asimétrica/menos amplia cerca del
    fiducial previo.

## 2026-07-13 02:00 — Subfase 1L — cola rígida junto al fiducial 1

- objetivo intentado:
  - fijar las aristas cercanas al fiducial 1 como se hace cerca del fiducial 2,
    para que los vértices inmediatos mantengan casi intacta su relativa local
    respecto al KF fiducial;
  - recuperar la recta local perpendicular a la orientación del KF fiducial.
- archivos modificados:
  - `orbslam3_multi/src/optimization_manager.cpp`;
  - documentación de estado, paquete, subfase e historial.
- cambio de código:
  - se elimina la U amplia como variante activa;
  - las edges que tocan el target o cuyos dos extremos tienen
    `alpha >= 0.965` usan rigidez SE(3) `100.0`;
  - la rampa cerca del target se mantiene desde `alpha=0.86` hasta `20.0`;
  - la edge directa al fiducial previo usa `25.0`, sin extender la U hacia el
    interior para evitar el flip observado en `kf=77`.
- paquetes compilados:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ```
- resultado de build:
  - `BUILD-EXIT-CODE 0`;
  - warnings no bloqueantes ya conocidos.
- pruebas Gazebo/replay:
  - no se repitió Gazebo;
  - replays offline sobre `codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`;
  - HTMLs:
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_target_tail_locked_1.html`
    y
    `codex/archivos_auxiliares/html/f1l_offline_graph_task_1_so3_target_tail_locked_2.html`;
  - log activo:
    `/tmp/f1l_so3_target_tail_locked_2.log`.
- patrones de reducción:
  ```text
  F1L-OFFLINE-DRYRUN|F1L-OFFLINE-METRICS|F1L-OFFLINE-TARGET-POSE-3D|F1L-OFFLINE-EDGE-DEFORMATION|F1L-OFFLINE-CORNER-DEFORMATION|F1L-OFFLINE-GT-STATS|F1L-OFFLINE-HTML
  ```
- evidencia positiva:
  - `target_tail_locked_2`: `success=true`, `useful=true`, `iterations=2`;
  - target fiducial en pose 6D exacta:
    `after_translation=0`, `after_rotation=0`;
  - mejora la media GT frente a `target_near_rigid_2`:
    `mean_after=2.17031 -> 2.12554`;
  - las tres aristas finales quedan casi bloqueadas:
    - `159->160`: `delta_map_xy_len=0.00258296`;
    - `160->161`: `delta_map_xy_len=0.00561829`;
    - `161->162`: `delta_map_xy_len=0.0190759`;
  - no aparece `sign_flipped` en `kf=77`, `kf=160` ni `kf=161`.
- evidencia negativa o ausente:
  - el máximo GT empeora frente a `target_near_rigid_2`:
    `max_after=4.43241 -> 4.722`;
  - `target_tail_locked_1` con rigidez `250.0` dejaba la cola aún más bloqueada,
    pero `useful=false` por `cost_not_reduced`;
  - no hay validación Gazebo/apply live posterior.
- conclusión:
  - `PARCIAL`;
  - la cola rígida consigue el comportamiento local pedido cerca del fiducial 1
    y es mejor que la U amplia, pero hay que inspeccionar el HTML antes de
    adoptarla como base para Gazebo.
- siguiente paso recomendado:
  - revisar
    `f1l_offline_graph_task_1_so3_target_tail_locked_2.html`;
  - si visualmente se ve la recta local buscada, decidir si se acepta el peor
    `max_after` o se añade una restricción para corregir el punto de máximo.

