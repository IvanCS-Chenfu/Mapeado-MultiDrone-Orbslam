# 01 — Estado actual resumido

Este archivo es la versión corta de `01_ESTADO_ACTUAL.md`. Mantenerlo compacto.
Si hace falta evidencia histórica, abrir el historial por subfase.

## Fase y subfase

```text
Fase activa: Fase 1 — Mapa sparse global multi-dron
Subfase actual: 1M
Conclusión 1L: PARCIAL
Estado 1M: `CovisibilityDatabase`, a probar en simulación
Siguiente bloque funcional: 1N+, bloqueado hasta validar 1M
```

## Objetivo global

Construir una nube densa global de un entorno YAML usando varios drones. La Fase
1 solo cierra cuando exista un mapa sparse global coherente, anclado, optimizado,
fusionable y útil para poses globales sin ground truth.

## Lo ya conseguido

- `1A`: baseline del servidor antiguo.
- `1B`: servidor nuevo mínimo y legacy congelado.
- `1C`: `RawMapDatabase`, journal raw y replay.
- `1D`: `GlobalPoseStore`.
- `1E`: `FiducialAnchorManager` y anchors fiduciales simulados.
- `1F`: `LandmarkScoreManager`, `GlobalMapBuilder` y `/global_sparse_cloud`.
- `1G`: full snapshots y reconciliación raw/global.
- `1H`: revisits fiduciales y `FiducialOptimizationTask`.
- `1I`: `PoseGraphBuilder` temporal con preservación de anclajes previos.
- `1J`: dry-run de `OptimizationManager`, dump/replay offline y HTML del grafo.
- `1K`: apply seguro en `GlobalPoseStore`, propagación y publicación coherente.

## Estado real de `1M`

`1M` implementa `CovisibilityDatabase`, una base de relaciones confirmadas entre
KeyFrames. Importará covisibilidad ORB-SLAM3 intra-dron desde la base principal
y permitirá añadir loops confirmados por geometría, incluidos pares entre
drones distintos. No guarda candidatos BoW ni estados pendientes: si una arista
está dentro, se considera confirmada. Debe guardar soporte, fuente y pose
relativa medida/current para que las optimizaciones añadan restricciones sin
separar KFs covisibles.

La implementación queda pendiente de prueba en simulación/replay: debe
confirmarse que el wrapper rellena las conexiones nativas y que los logs
`[F1M-COVIS-*]` muestran importación y consultas coherentes antes de desbloquear
`1N+`.

## Estado real de `1L`

`1L` no está cerrada. Su propiedad vigente es diagnóstico post-apply:
logs/RViz2/GT debug. El dry-run y el HTML del grafo pertenecen a `1J`; el apply,
la propagación, la publicación y la herencia de KFs futuros pertenecen a `1K`.
La prueba `dron_2` antihoraria reproduce el caso crítico y ya permite comprobar
si lo aplicado por `1K` se ve correctamente.

Última evidencia de solver aceptado: `OptimizationManager` optimiza
`x,y,z,roll,pitch,yaw`, con yaw dominante y roll/pitch débiles. Las aristas
internas preservan fuerte la longitud/distancia relativa, dejan la dirección
local como sesgo débil y casi liberan yaw relativo. La prueba `dron_2`
antihoraria llegó a aceptar un candidato: fiducial 1
`9.792646 m -> 0.009804 m`, ventana GT `mean_after=1.762549`,
`max_after=4.030853`, e `internal_max_after=0.027445`.

Ejecuciones posteriores con dump offline de `1J`: se añadió guardado del grafo
con `f1l_graph_dump_enabled` y nodo `test_opt_graph_offline`. El prefijo `f1l_*`
queda como legacy de parámetros/archivos; funcionalmente es diagnóstico de
dry-run. El grafo grande actual es `f1l_graph_task_2.tsv`. La mejor variante activa sigue siendo
`reliable_hinge_4`: offline `25.6951 m -> 0.0297886 m`, GT diagnóstico
`mean_after=3.28931`, `max_after=5.84293`, HTML
`f1l_offline_graph_task_2_final_reliable_hinge_4.html`. Se rechazaron
`sparse_texture`, `reliable_straight_1`, `edge_shape_1..3`,
`piecewise_blocks_1..2` y `global_xy_guard_1..2`; abren o protegen piezas
aisladas pero empeoran la ventana completa. El apply live no se ha repetido tras
esa tanda offline concreta; después sí se repitió Gazebo para validar publicación y
herencia de KFs futuros. `1L` sigue `PARCIAL` por inspección visual/RViz2 y
decisión final pendientes.

Última iteración offline sobre el grafo nuevo `f1l_graph_task_1.tsv`: el solver
activo ya no usa yaw como restricción rotacional principal. Las variables
angulares son `LogSO3(delta_R)`, priors/aristas usan residual de rotación 3D y
el `FiducialTarget` se fija a su pose 6D. La prueba de rigidez tipo U amplia en
ambos extremos no mejoró la base (`mean_after=2.21057`, `max_after=4.73642`,
`sign_flipped=true` en `kf=77`). La variante activa bloquea de forma fuerte solo
la cola cercana al fiducial 1 (`alpha >= 0.965`, rigidez `100.0`) y conserva una
rampa suave desde `alpha=0.86`. Replay:
`f1l_offline_graph_task_1_so3_target_tail_locked_2.html`, target exacto,
`mean_after=2.12554`, `max_after=4.722`, `useful=true`. La iteración siguiente
reforzó el score por soporte de KeyFrames sin repetir Gazebo:
`f1l_offline_graph_task_1_so3_support_mild_1.html`, `useful=true`,
`mean_after=2.12846`, `max_after=4.72425`. No corrige de forma significativa la
curva alrededor de `kf=135`; las variantes más agresivas quedaron
`useful=false`.

Última prueba de densidad reforzada: se intentó hacer que las zonas con muchos
KeyFrames fueran mucho más rígidas y que la zona central pobre absorbiera más
deformación. Las variantes por pesos `support_dense_lock_1` y
`support_dense_hinge_2` quedaron `useful=false`; no seguir solo subiendo pesos.

Variante activa offline recomendada:
`f1l_offline_graph_task_1_so3_target_tail_pose_lock_5.html` (verificación
equivalente: `target_tail_pose_lock_restored_10`). Bloquea la cola del fiducial
1 reconstruyendo poses relativas desde el prior 6D del `FiducialTarget`.
Resultado: `success=true`, `useful=true`, target `t=0`, `rot=0`,
`mean_after=1.15554`, `max_after=2.57206`. La arista densa `63->77` queda casi
inmóvil (`delta_map_len=0.0111595`) y la cola final conserva longitud 3D casi
exacta. Las variantes `_8` y `_9` de posición relativa global quedan
descartadas como línea activa: mejoraban algún detalle visual de `161->162`,
pero empeoraban claramente el GT global. `1L` sigue `PARCIAL`.

Última aplicación probada: vecindades inducidas desde ambos fiduciales. El
fiducial previo queda en su pose anclada y el fiducial objetivo en su prior 6D;
los vecinos cercanos de ambos se pinchan antes del solve en la pose inducida
por la transformada relativa ORB-SLAM3 respecto a su fiducial. Replay:
`f1l_offline_graph_task_1_so3_fiducial_neighborhood_pose_locks_13.html`,
`success=true`, `useful=true`, target `t=0`, `rot=0`,
`mean_after=0.822023`, `max_after=1.78632`. Las aristas `55->56->57->58` y
`159->160->161->162` quedan con `delta_len` numéricamente cero. `_13` pasa a
ser la mejor referencia offline actual, aunque `1L` sigue `PARCIAL` por falta
de inspección visual/live y por un `sign_flipped=true` en `kf=110`.

Última prueba live de `1K` tras reconstruir MapPoints desde la pose final del KeyFrame
elegido y saltar fallback rígido en submapas corregidos:

```text
prueba_1 Gazebo: SIM-EXIT-CODE 0
fiducial objetivo: before_t=26.641246 -> real_after_t=0.000000
F1L-POST-APPLY-ACCEPT decision=ACCEPT
propagated_count=98 skipped_propagated_count=0
GlobalMapBuilder: server_corrected_after=14917 / published_points_after=20815
F1L-GLOBALMAP-KF-PROJECTION fallback_submap_after=0
HTML diagnóstico de 1J: codex/archivos_auxiliares/html/f1l_debug_animation_task_2.html
Dump: codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv
```

Ahora `GlobalMapBuilder` ya no publica MapPoints aplicando una última corrección
heredable de submapa. Para cada MapPoint prefiere su `reference_keyframe_id`, o
un observador con pose válida, calcula `p_kf = raw_local_T_kf^-1 * p_raw` y
publica `p_world = final_world_T_kf * p_kf`. Esto busca que RViz2 siga la misma
deformación de KeyFrames que el HTML. La deuda pasa a ser inspección visual en
RViz2 y, si aún hay puntos mal arrastrados, depurar la selección de KF por
MapPoint. Si el submapa ya tiene corrección de servidor y un punto no tiene KF
publicable, se salta en vez de salir con `world_T_local`.

Última validación adicional de herencia para KFs posteriores: `GlobalPoseStore`
guarda la corrección heredable desde el KF aplicado más nuevo y conserva el KF
fuente frente a reconciliaciones de full snapshot. Gazebo `prueba_1` terminó
con `SIM-EXIT-CODE 0`; el apply fuerte fue `task_id=2`, `from_kf=200`,
`last_correction_set=true`, y los KFs nuevos `202..208` heredaron
`source=SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT`. Si un full snapshot toca
KFs posteriores al `from_kf`, deben conservar esa corrección heredada y no
volver al anchor base del submapa.

## Siguiente trabajo mínimo

Primero completar `1M`. La revalidación de `1L` queda como trabajo futuro si se quiere cerrar del todo la ruta de
optimización fiducial. Para esa revalidación, respetar la propiedad 1J/1K/1L:

- inspeccionar el HTML diagnóstico (`1J`) y RViz2 tras el `ACCEPT`;
- confirmar visualmente que la publicación de MapPoints desde pose final de KF
  aplicada por `1K` coincide con el HTML y no arrastra el fiducial previo como
  bloque rígido;
- confirmar visualmente que los KFs posteriores al apply continúan desde la
  corrección heredada del KF optimizado más nuevo;
- si RViz2 sigue raro, loggear `reference_kf`/`selected_kf` por MapPoint en la
  zona conflictiva y tratarlo como ajuste de `1K`;
- si aún queda deformación global, añadir más controles/KFs variables en `1I` o
  ajustar dry-run en `1J`;
- no dejar activa la rigidez angular parabólica pura: se probó con dos
  intensidades y empeoró frente al solver 6D uniforme;
- usar el dump offline de `1J` para iterar pesos/solver sin repetir Gazebo;
- abrir `f1l_offline_graph_task_2_final_reliable_hinge_4.html` para revisar la
  mejor base offline vigente;
- no seguir probando solo pesos/guards sobre las mismas variables por vértice:
  hace falta nueva parametrización con escala/stretch explícito para subtramos
  pobres y bloques rígidos reales para tramos fiables, o más controles antes de
  optimizar;
- usar `f1l_offline_graph_task_1_so3_fiducial_neighborhood_pose_locks_13.html`
  como base offline actual;
- revisar el tramo central `109..112`, donde sigue apareciendo `kf=110`
  con `sign_flipped=true`;
- si la cola visual sigue molestando, resolverlo con otra parametrización, no
  con `_8/_9`;
- si se mantiene el solver SO(3) con target fijado, reducir deformaciones
  internas/Z antes de repetir Gazebo, o probar un fiducial intermedio en la U;
- no volver a usar aristas internas deformadas ni propagación diagnóstica como
  rollback automático mientras hard fixed, anchors y error fiducial estén OK.

## Archivos críticos si se implementa `1L`

```text
orbslam3_multi/include/orbslam3_multi/optimization_result.hpp
orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp
orbslam3_multi/src/optimization_manager.cpp
orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp
orbslam3_multi/src/global_pose_store.cpp
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
orbslam3_multi/src/global_map_builder.cpp
orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp
orbslam3_multi/include/orbslam3_multi/pose_graph_problem_io.hpp
orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp
orbslam3_multi/src/pose_graph_problem_io.cpp
orbslam3_multi/src/pose_graph_builder.cpp
orbslam3_multi/src/test_opt_graph_offline.cpp
orbslam3_server/src/global_map_server.cpp
orbslam3_server/launch/global_orb_map_server.launch.py
```

## Paquetes habituales de build para `1L`

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi
./codex/herramientas/build_selected_packages.sh orbslam3_server
```

Si se toca simulación/launch:

```bash
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

## Pruebas de `1L`

- error pequeño/moderado: esperado `ACCEPT_COMMIT`;
- deriva grave antihoraria: esperado refinamiento sin publicar parciales;
- dos drones: independencia por `(drone_id,map_epoch)`;
- rechazo controlado: `REJECT_NO_COMMIT` y estado confirmado intacto.

## Dónde ampliar contexto

- Contrato actual: `codex/pipeline/fase_1_sparse_global/subfases/subfase_1M.md`
- Contrato 1L: `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`
- Historial específico: `codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md`
- Estado largo: `codex/contexto/01_ESTADO_ACTUAL.md`
- Docs de paquete: `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`,
  `pose_graph_builder.md`, `global_pose_store.md`,
  `codex/contexto/paquetes/orbslam3_server/global_map_server.md`.
