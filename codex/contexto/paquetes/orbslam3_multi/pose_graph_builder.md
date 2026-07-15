# `PoseGraphBuilder`

## Rol actual

`PoseGraphBuilder` es la clase de `orbslam3_multi` introducida en `1I` para construir un `PoseGraphProblem` temporal desde una tarea de optimizacion.

En `1I` solo existe la ruta obligatoria:

```cpp
BuildForFiducialTask(...)
```

No optimiza y no aplica resultados. Desde `1J`, su salida es consumida por `OptimizationManager` para dry-run; el apply real queda para subfases posteriores.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp
orbslam3_multi/src/pose_graph_builder.cpp
```

## API principal

```cpp
struct PoseGraphBuilderConfig;
struct PoseGraphBuildResult;

class PoseGraphBuilder
{
public:
    void Configure(const PoseGraphBuilderConfig& config);
    const PoseGraphBuilderConfig& GetConfig() const;

    PoseGraphBuildResult BuildForFiducialTask(
        const FiducialOptimizationTask& task,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store) const;
};
```

## Entradas

- `FiducialOptimizationTask`: tarea creada por `FiducialAnchorManager` cuando un revisit fiducial mide residual alto.
- `RawMapDatabase`: KFs raw y submapas por `(drone_id, map_epoch)`.
- `GlobalPoseStore`: poses globales actuales, anchors y KFs hard fiducial.

## Politica inicial de 1I

- Construye la ventana solo dentro del submapa de la tarea.
- Usa solo KFs con pose global existente.
- Intenta limitar la ventana por KFs hard fiducial si `anchor_stop_enabled=true`.
- Selecciona siempre el KF objetivo, extremos de ventana, KFs hard fiducial y muestras temporales hasta `max_vertices`.
- Marca hard fiducials como fijos.
- Mantiene KFs normales como variables aunque el submapa este anclado.
- Crea aristas temporales entre vertices consecutivos.
- Crea priors hard, target y soft.
- Devuelve un `PropagationPlan` para KFs afectados no seleccionados como vertices.

## Politica vigente tras reapertura de 1I

Desde el 2026-07-10, `BuildForFiducialTask` protege explicitamente fiduciales previos del mismo submapa cuando el target corresponde a otro fiducial:

- recopila KFs hard fiducial del mismo `(drone_id, map_epoch)` excluyendo el target;
- selecciona como anclajes frontera los KFs hard fiducial mas cercanos por rama temporal antes/despues del target;
- expande la ventana si hace falta para incluir esos anclajes aunque queden fuera de `max_path_length`;
- inserta esos anclajes como vertices fijos con `selection_reason=previous_fiducial_anchor`;
- rellena `anchor_preservation` y `fiducial_connectivity_edges` para que servidor, dry-run y apply puedan validar la invariante;
- marca como `SUBDIVIDED_CONFIRMED` los fiduciales mas lejanos dominados por un anclaje cercano en la misma rama temporal.

La regla de seguridad es:

```text
Una tarea fiducial multi-anclaje no debe generar un grafo con fixed=0 hard_fiducial=0.
```

La implementacion es conservadora. Las aristas de conectividad ayudan a elegir fronteras y a loggear decisiones, pero no son aristas numericas del solver.

## Parametros

`PoseGraphBuilderConfig` contiene:

- `max_vertices`
- `min_vertices`
- `max_path_length`
- `anchor_stop_enabled`
- `include_temporal_edges`
- `temporal_edge_weight`
- `temporal_edge_weight_sparse`
- `fiducial_hard_weight`
- `fiducial_target_translation_weight`
- `fiducial_target_rotation_weight`
- `current_pose_soft_weight`
- `fiducial_connectivity_enabled`
- `branch_selection_enabled`
- `subdivision_candidate_radius_m`
- `max_temporal_edge_kf_gap`
- `max_temporal_edge_length_m`
- `corner_yaw_threshold_rad`

El servidor los expone como parametros `pose_graph_*`.

## Logs de validacion

El builder no depende de ROS. Los logs se emiten desde `orbslam3_server/src/global_map_server.cpp`:

```text
[F1I-TASK-RX]
[F1I-GRAPH-BUILD-START]
[F1I-GRAPH-WINDOW]
[F1I-GRAPH-VERTEX-SELECT]
[F1I-GRAPH-FIXED-KF]
[F1I-GRAPH-VARIABLE-KF]
[F1I-GRAPH-EDGES]
[F1I-GRAPH-PRIORS]
[F1I-GRAPH-WEIGHTS]
[F1I-GRAPH-PROPAGATION-PLAN]
[F1I-GRAPH-PROBLEM-CREATED]
[F1I-GRAPH-BUILD-SUMMARY]
[F1I-FID-CONNECTIVITY-BRANCHES]
[F1I-FID-CONNECTIVITY-EDGE]
[F1I-FID-CONNECTIVITY-SUBDIVISION]
[F1I-GRAPH-ANCHOR-PRESERVATION]
[F1I-GRAPH-PREVIOUS-FIDUCIAL-ANCHOR]
[F1I-GRAPH-VERTEX-COVERAGE]
[F1I-GRAPH-CORNER-VERTEX]
[F1I-GRAPH-LONG-EDGE-SPLIT]
[F1I-GRAPH-PROPAGATION-SEGMENT]
```

## Validacion 1I

El 2026-07-09 se valido inicialmente con:

- build `orbslam3_multi`, `orbslam3_server` y `simulacion_dron` con `BUILD-EXIT-CODE 0`;
- prueba live larga con `SIM-DONE prueba=1 success=true`;
- prueba replay con `SIM-DONE prueba=2 success=true`;
- live: `109` tareas fiduciales y `109` grafos construidos;
- replay: `16` tareas fiduciales reales y `17` grafos construidos;
- sin `OPT-APPLY`, `OPTIMIZATION-APPLIED` ni `SET_OPTIMIZED_POSE`.

El 2026-07-10 se revalido tras detectar el fallo `fixed=0 hard_fiducial=0` en una prueba larga posterior:

- build separado de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron` con salida correcta;
- `prueba_1` live larga: `33` tareas/grafos; el primer caso tenia `error_t=22.312883` y ahora genera `fixed=1 hard_fiducial=1`;
- `prueba_2` live equivalente a replay: `16` tareas/grafos; tambien genera `previous_fiducial_fixed_count=1`;
- ambas pruebas terminan con `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE success=true` y `SIM-EXIT-CODE 0`;
- `f1j_dryrun_enabled=false`, `f1k_apply_enabled=false` y `f1l_validation_enabled=false`, por lo que la prueba valida solo la construccion del grafo y no aplica poses.

## Pendiente operativo

- `1J`: revalidar dry-run del coste/resultado con grafos multi-anclaje.
- `1K`: revalidar apply usando evidencia de preservacion de anclajes.
- `1L`: revalidar post-apply/rollback y añadir check especifico de preservacion de anclajes.
- Fases posteriores: loops/subnubes podran reutilizar el mismo constructor o ampliarlo.

## Ajustes de grafo reclasificados como 1I

`PoseGraphBuilder` filtra como frontera fiducial solo observaciones de un fiducial distinto al objetivo. Ademas selecciona y fija una vecindad temporal configurable alrededor del anclaje previo mediante `fiducial_neighborhood_radius_kfs` (valor vigente `3`). Los vertices quedan marcados con `is_anchor_neighborhood` y `selection_reason=previous_fiducial_neighborhood`.

Esta proteccion se valido con `previous_fiducial_neighborhood_fixed_count=4` y deltas nulos. No resuelve por si sola la deformacion global: la prueba aislada mantiene `1L` parcial porque el solver consumidor puede generar residuales internos extremos sobre la ventana.

Actualización de `1I`: para probar el tratamiento simétrico de ambos fiduciales, el
builder ya no debe convertir toda la vecindad del fiducial previo en poses
absolutas fijas. Solo el `previous_fiducial_anchor` queda fijo. Los vértices
`previous_fiducial_neighborhood` siguen seleccionados y etiquetados, pero pasan
al solver como variables para que `OptimizationManager` los conserve mediante
locks relativos al ancla. La intención es que ambos extremos se comporten igual:
fiducial fijo/target 6D, vecindad rígida por aristas relativas, no por fijado
absoluto de cada vértice.

## Reejecucion durante 1L, propiedad conceptual 1I

Para diagnosticar el fallo visto en RViz2 se reforzo la seleccion de vertices.
Aunque se hizo durante la sesión de `1L`, pertenece al contrato de construcción
del grafo de `1I`:

- muestreo por distancia acumulada de trayectoria, no solo por indice;
- inclusion explicita de vecindad del fiducial objetivo y del fiducial previo;
- inclusion de cambios fuertes de yaw como `corner_yaw_vertex`;
- subdivisiones de tramos con gap o longitud excesiva;
- `PropagationPlan` con `segment_alpha`, distancia desde el control A,
  longitud del tramo y gap de KFs.

La prueba final de esa tanda mostro que los nuevos logs aparecen y que los
parciales ya no publican propagacion amplia. Esa segunda parte pertenece a
`1K/1L`; en `PoseGraphBuilder`, la deuda real era convertir la cobertura
insuficiente en condicion dura:

```text
si max_edge_kf_gap o segment_length_m exceden el limite tras seleccionar vertices,
rechazar el grafo antes del solver o aumentar adaptativamente max_vertices.
```

## Cobertura dura 1I

`PoseGraphBuilder` calcula ahora `PoseGraphCoverageSummary` y lo guarda en el
`PoseGraphProblem` cuando el grafo es valido. Durante la seleccion de controles
puede superar el limite nominal `max_vertices` si la ventana necesita mas
vertices para respetar los limites de gap/longitud.

Si aun asi quedan tramos largos sin cubrir, `BuildForFiducialTask` devuelve
`success=false` con:

```text
reason=bad_window_coverage
coverage.uncovered_long_segments>0
```

En la prueba `dron_2` antihoraria, la primera pasada quedo cubierta, pero la
segunda pasada desde checkpoint parcial fue rechazada por cobertura insuficiente.
El guard pertenece a `1I`; que `1L` lo use como evidencia para pedir
refinamiento o segunda pasada es diagnóstico post-apply.
