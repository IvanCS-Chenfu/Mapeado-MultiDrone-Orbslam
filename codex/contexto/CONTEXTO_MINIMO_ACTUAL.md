# Contexto mínimo actual

Leer este archivo primero en chats nuevos. Si no basta, abrir solo los enlaces
necesarios.

## Estado

```text
Fase: 1 — mapa sparse global multi-dron
Subfase actual: 1M
Conclusión 1L: PARCIAL
Estado 1M: `CovisibilityDatabase`, sin hacer
Bloqueo: no iniciar 1N+ hasta completar 1M
```

Objetivo global: nube densa global con varios drones, sin ground truth para mapa
final ni pose final.

## Reglas críticas

- Responder y documentar en español.
- Editar solo dentro de `src/`.
- No tocar `ORB_SLAM3`, `orbslam3_ros2` ni `orbslam3_msgs` salvo necesidad clara.
- Usar `submapa = (drone_id, map_epoch)`.
- `RawMapDatabase` conserva raw; no se modifica por optimización.
- `GlobalPoseStore` conserva anchors, poses globales, optimizaciones y rollback.
- Fiduciales son observaciones absolutas, no loops.
- Ground truth solo para fiducial simulado, debug o métricas externas.
- `07_ULTIMA_SESION.md` se reemplaza completo; el detalle va al historial.

## Arquitectura activa

```text
orbslam3_ros2 -> orbslam3_server -> orbslam3_multi
```

`orbslam3_server`: ROS 2, conversión, publicación, GT solo debug/fiducial.  
`orbslam3_multi`: backend (`RawMapDatabase`, `GlobalPoseStore`,
`FiducialAnchorManager`, `PoseGraphBuilder`, `OptimizationManager`,
`GlobalMapBuilder`).

## Estado actual 1J/1K/1L

Propiedad funcional vigente:

```text
1J = dry-run + dump/replay offline + HTML diagnóstico del grafo
1K = apply del grafo en GlobalPoseStore + propagación + publicación
1L = diagnóstico post-apply con logs/sublogs/RViz2 opcional/GT debug
```

El caso `dron_2` antihorario ya puede llevar el KeyFrame fiducial objetivo a su
pose GT simulada de diagnóstico. La deuda actual de `1L` ya no es mejorar la
geometría local: es decidir documentalmente si la evidencia de logs basta para
cerrarla y dejar la falta de covisibilidad/loops a subfases posteriores.

Regla activa de aceptación:

```text
fiducial objetivo en umbral + hard fixed/anchors/NaN OK -> ACCEPT
aristas internas y propagación amplia -> diagnóstico, no rollback
```

El GT medio de ventana no decide runtime: solo diagnostica.

`1L` queda `PARCIAL` y se volverá a comprobar en el futuro. La subfase activa
es `1M`: crear `CovisibilityDatabase` confirmada. Esta base importará
covisibilidad ORB-SLAM3 intra-dron, recibirá loops geométricos confirmados y
será consultada por BoW/optimización para evitar trabajo repetido y no separar
KFs covisibles.

Última evidencia live de `1K`: tras cambiar `GlobalMapBuilder` para reconstruir
MapPoints desde la pose final del KF elegido, Gazebo `prueba_1` aceptó y
publicó:

```text
before_t=26.641246 -> real_after_t=0.000000
F1L-POST-APPLY-ACCEPT decision=ACCEPT
F1L-POST-APPLY-PROPAGATION-CHECK ok=true propagated_count=98 skipped=0
F1L-POST-APPLY-GLOBALMAP-CHECK ok=true server_corrected_after=14917/20815
F1L-GLOBALMAP-KF-PROJECTION fallback_submap_after=0
```

`GlobalMapBuilder` ya no usa la última corrección heredable del submapa para
publicar puntos sin KF local válido. Para cada MapPoint prefiere
`reference_keyframe_id`, o un observador con pose en `GlobalPoseStore`, calcula
`p_kf = raw_local_T_kf^-1 * p_raw` y publica
`p_world = final_world_T_kf * p_kf`. Si un submapa ya tiene corrección de
servidor y un punto no tiene KF publicable, lo salta en vez de publicarlo con
`world_T_local`.

Última corrección adicional: `GlobalPoseStore` fija la corrección heredable para
KFs futuros desde el KF aplicado más nuevo y registra esos KFs como
server-controlled para que full snapshots posteriores no los rebasen al anchor.
En la validación final:

```text
F1K-POSESTORE-LAST-CORRECTION-SET task_id=2 from_kf=200
F1K-OPT-APPLY-SUMMARY last_correction_set=true
KFs 202..208 -> source=SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT
```

Las reconciliaciones de full snapshot ya no deben pisar esa corrección heredable
con KFs optimizados anteriores.

El dry-run/HTML pertenece a `1J`, aunque algunos parámetros/archivos conserven
el prefijo legacy `f1l_*`. El dump offline `f1l_graph_dump_enabled` guarda
`f1l_graph_task_<id>.tsv` y `test_opt_graph_offline` reproduce el dry-run sin
Gazebo. El dump relevante actual es `f1l_graph_task_2.tsv`. Tras diagnosticar
que `before_rel_z` era componente local y no altura RViz2, la variante activa
limita la relajacion de aristas pobres a `0.55`, conserva bisagra XY suave y
añade residual de Z global por arista. Replay offline final:
`f1l_offline_graph_task_2_reliable_hinge_4.html`, dry-run `success=true`,
fiducial objetivo `25.6951 m -> 0.0297886 m`, GT diagnostico
`mean_after=3.28931`, `max_after=5.84293`, `worsened_kfs=58`; `kf143` mejora
de `after_turn=-0.821971` a `-0.454941` frente a la variante previa, y `kf161`
ya no invierte (`sign_flipped=false`). Después se probaron 4 variantes
`sparse_texture` para abrir `127->129` bajando más el peso de aristas pobres:
abren esa arista (`after_map_xy_len` hasta `3.6081`), pero empeoran bisagras
fiables y GT global; quedaron rechazadas. También se rechazó
`reliable_straight_1`: reforzar rectas con mucho soporte alrededor de `kf161`
empeoró `kf143`, comprimió `150->161` y subió `mean_after=6.46072`. El código
volvió a la variante `reliable_hinge_4`. Después se rechazaron `edge_shape_1..3`
(guards de vector XY fiable y stretch de arista pobre): abren o protegen piezas
aisladas, pero empeoran `mean_after` a `6.23787..6.9776`. También se
rechazaron `piecewise_blocks_1..2` (bloques por rigidez de gradiente cerca de
fiduciales) y `global_xy_guard_1..2` (guardarraíl de longitud XY global para
aristas fiables): abren o desplazan la deformación, pero rompen `150->161`,
`kf143/kf150/kf161` o empeoran `mean_after` a `4.55855..7.14734`. Conclusión
actual: no seguir con pesos/guards sobre el mismo solver; cambiar
parametrización con escala/stretch explícito para subtramos pobres y bloques
rígidos reales para tramos fiables. Esa rama offline queda como referencia
histórica; después se repitió Gazebo para validar publicación de MapPoints e
herencia de KFs futuros. `1L` sigue `PARCIAL` por decisión final pendiente,
pero ya no debe absorber la deuda de deformación local que corresponde a la
base futura de covisibilidad/loops.

Referencia visual actual: HTML live
`codex/archivos_auxiliares/html/f1l_debug_animation_task_1.html` y dump
`codex/archivos_auxiliares/repeticiones/f1l_graph_task_1.tsv`. Si se sigue refinando, mirar
primero la correspondencia HTML/RViz2 y la selección de KF por MapPoint antes de
volver a endurecer aristas internas.

## Leer según tarea

| Necesidad | Archivo |
|---|---|
| Pipeline corto | `codex/pipeline/fase_1_sparse_global/pipeline_fase_1_RESUMEN.md` |
| Contrato actual | `codex/pipeline/fase_1_sparse_global/subfases/subfase_1M.md` |
| Contrato 1L | `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md` |
| Historial 1L | `codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md` |
| Estado largo | `codex/contexto/01_ESTADO_ACTUAL.md` |
| Paquetes | `codex/contexto/05_MAPA_PAQUETES.md` |
| Logs | `codex/contexto/09_LOGS_Y_SUBLOGS.md` |

Para buscar paquete o componente rápidamente y evitar lecturas largas, usar:

```bash
python3 codex/herramientas/find_context.py <query>
```

## Build y simulación

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi
./codex/herramientas/build_selected_packages.sh orbslam3_server
./codex/herramientas/run_simulation.sh --prueba X --launch "ros2 launch simulacion_dron multi_dron.launch.py"
./codex/herramientas/split_simulation_log.sh --prueba X --fase 1L
```

Si se toca `simulacion_dron`, compilarlo aparte.

## Lectura barata

- Antes de abrir logs: usar `split_simulation_log.sh`, `rg`, `tail`, `wc`.
- Antes de abrir historiales largos: leer `historial/INDEX.md`.
- Antes de tocar paquetes: leer solo el `.md` del componente afectado.
