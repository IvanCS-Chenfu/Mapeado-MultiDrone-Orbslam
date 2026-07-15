# `PoseGraphProblem`

## Rol actual

`PoseGraphProblem` es la estructura temporal introducida en `1I` para describir un problema de optimizacion antes de ejecutar ningun solver.

No es un grafo global permanente. Se construye a partir de una tarea concreta, se loggea para validacion y desde `1J` lo consume `OptimizationManager` para dry-run.

## Archivo

```text
orbslam3_multi/include/orbslam3_multi/pose_graph_problem.hpp
```

## Estructuras

| Estructura | Uso |
|---|---|
| `PoseGraphProblem` | Contenedor temporal completo: tarea, submapa, vertices, aristas, priors, KFs afectados y `PropagationPlan`. |
| `PoseGraphVertex` | KF seleccionado como variable o fijo, con pose inicial `world_T_kf`, peso y motivo de seleccion. |
| `PoseGraphEdge` | Restriccion interna entre dos vertices. En `1I` se usan aristas temporales. |
| `PoseGraphPrior` | Prior de pose: `FIDUCIAL_HARD`, `FIDUCIAL_TARGET` o `CURRENT_POSE_SOFT`. |
| `PoseGraphPropagationPlanEntry` | KF afectado que no entra como vertice y debe heredar/interpolar correccion en fases futuras; desde la reejecucion de `1L` conserva tramo y `segment_alpha` explicitos. |
| `PoseGraphProblemSummary` | Conteos de vertices, aristas, priors, variables, fijos y propagacion. |
| `FiducialConnectivityEdge` | Evidencia topologica auxiliar entre fiduciales observados dentro del submapa actual. No es un edge de optimizacion. |
| `PoseGraphAnchorPreservationSummary` | Resumen de si la tarea requiere preservar anclajes fiduciales previos y si el grafo lo satisface. |
| `PoseGraphCoverageSummary` | Resumen de cobertura de vertices de control: KFs de ventana, controles, span maximo, tramos largos sin cubrir y motivo de aceptacion/rechazo. |
| `PoseGraphDebugKeyFramePose` | Bloque diagnostico opcional para dumps `1L`: pose mapa inicial y GT externo por KF de ventana. El solver debe ignorarlo. |

## Enums

```text
PoseGraphEdgeType
PoseGraphPriorType
PoseGraphPropagationMode
FiducialConnectivityEdgeStatus
```

Valores relevantes:

- `TEMPORAL_NEIGHBOR`
- `SOFT_CONSISTENCY`
- `FIDUCIAL_HARD`
- `FIDUCIAL_TARGET`
- `CURRENT_POSE_SOFT`
- `NEAREST_CONTROL_VERTEX`
- `PATH_SEGMENT`
- `INHERIT_LAST_CORRECTION`
- `DIRECT_OBSERVED`
- `DIRECT_UNCERTAIN`
- `SUBDIVISION_CANDIDATE`
- `SUBDIVIDED_CONFIRMED`
- `BYPASS_CONFIRMED`

## Preservacion de anclajes fiduciales previos

Desde la revalidacion de `1I` del 2026-07-10, `PoseGraphProblem` conserva metadatos para evitar que una tarea fiducial corrija el fiducial objetivo desplazando otro fiducial ya aceptado del mismo submapa.

Campos vigentes:

- `fiducial_connectivity_edges`: aristas auxiliares entre fiduciales vistas en el mismo submapa, con estado conservador de conectividad/subdivision.
- `anchor_preservation`: indica `required`, `satisfied`, numero de ramas independientes, anclajes frontera seleccionados, anclajes previos fijos y subdivisiones confirmadas/candidatas.

La condicion valida para una tarea multi-anclaje es:

```text
anchor_preservation.required=true
anchor_preservation.satisfied=true
previous_fiducial_fixed_count>=1
```

El grafo sigue siendo temporal: estos campos no crean estado global persistente ni autorizan por si solos optimizacion/apply.

## Restricciones

- No modifica `RawMapDatabase`.
- No modifica `GlobalPoseStore`.
- No representa estado persistente.
- No ejecuta solver.
- No aplica poses.
- No crea fused landmarks ni loops.
- Desde `1J`, puede ser consumido por `OptimizationManager`, pero el grafo sigue siendo temporal y no se almacena como estado global.

## Validacion 1I

Los logs de `global_map_server` deben mostrar:

```text
[F1I-GRAPH-PROBLEM-CREATED]
[F1I-GRAPH-BUILD-SUMMARY]
[F1I-FID-CONNECTIVITY-BRANCHES]
[F1I-FID-CONNECTIVITY-EDGE]
[F1I-FID-CONNECTIVITY-SUBDIVISION]
[F1I-GRAPH-ANCHOR-PRESERVATION]
[F1I-GRAPH-PREVIOUS-FIDUCIAL-ANCHOR]
```

En la validacion inicial del 2026-07-09:

- prueba live: `109` problemas creados con `success=true`;
- prueba replay: `17` problemas creados con `success=true`;
- hubo problemas con `vertices=12`, `edges=11`, `priors=12` y `propagation=49` en live.

En la revalidacion del 2026-07-10:

- `prueba_1` genero `33` tareas/grafos F1I;
- `prueba_2` genero `16` tareas/grafos F1I;
- los grafos multi-anclaje relevantes muestran `fixed=1 hard_fiducial=1`;
- se validaron marcadores `anchor_preservation satisfied=true` y `previous_fiducial_fixed_count=1`;
- no se ejecuto solver ni apply porque `f1j_dryrun_enabled=false`, `f1k_apply_enabled=false` y `f1l_validation_enabled=false`.

## Validacion 1J

El 2026-07-09 se valido que los `PoseGraphProblem` de `1I` alimentan `OptimizationManager`:

- prueba live: `19` resultados `[F1J-OPT-DRYRUN-RESULT] ... success=true`;
- prueba replay/debug: `17` resultados `[F1J-OPT-DRYRUN-RESULT] ... success=true`;
- los resultados muestran mejora de error/coste y `useful=true`;
- `[F1J-OPT-NO-STATE-MUTATION] ... ok=true` confirma que consumir el grafo no aplica estado.

## Campos vigentes de 1L

`PoseGraphVertex` incluye `is_anchor_neighborhood`. `PoseGraphAnchorPreservationSummary` incluye `previous_fiducial_neighborhood_fixed_count`.

`PoseGraphPropagationPlanEntry` incluye tambien:

```text
segment_alpha
distance_from_a_m
segment_length_m
control_span_kf_gap
```

Estos campos existen para que `OptimizationManager` propague KFs no vertices
por el tramo real entre dos vertices de control. No debe inferirse `alpha`
desde `local_kf_id`, porque una ventana con KFs irregulares o esquinas falsas
puede separar KFs en RViz2 aunque el residual fiducial objetivo baje.

`PoseGraphProblem` tambien conserva el linaje de una segunda pasada:

```text
rebuilt_from_partial_checkpoint
partial_parent_task_id
partial_retry_count
```

Estos campos permiten distinguir un grafo original de uno reconstruido desde poses parciales; no convierten el problema temporal en un grafo global persistente.

Desde la iteracion offline ejecutada durante `1L`, pero propiedad conceptual de
`1I/1J`, `PoseGraphEdge` incluye metadatos de soporte:

```text
intermediate_keyframe_count
support_keyframe_count
support_length_m
support_density_kfs_per_m
support_rigidity_multiplier
```

La idea es usar una señal interna de fiabilidad sin GT: una arista con muchos
KFs por metro tiene más evidencia local y se vuelve más rígida; una arista con
pocos KFs por metro se vuelve más deformable. El factor se satura y puede
combinarse con relajación por giro relativo alto.

El bloque `debug_keyframes` de `PoseGraphProblem` puede aparecer solo en dumps
generados por `global_map_server` con debug activado. Contiene:

```text
map_world_T_kf
has_gt
gt_world_T_kf
association_dt_sec
association_quality
```

Ese bloque sirve para HTML/logs offline y no puede alimentar pesos, selección de
vertices, priors ni decisión de aceptación.

Desde la reejecucion posterior, propiedad conceptual de `1I`,
`PoseGraphProblem` incluye tambien:

```text
coverage
```

`coverage` documenta si la ventana quedo realmente cubierta por controles. Si
el builder detecta tramos largos/gaps excesivos que no puede cubrir, devuelve
fallo antes del solver con `coverage.reason=bad_window_coverage`. Este rechazo
es una barrera de seguridad: no valida la optimizacion, solo evita optimizar una
ventana mal condicionada.
