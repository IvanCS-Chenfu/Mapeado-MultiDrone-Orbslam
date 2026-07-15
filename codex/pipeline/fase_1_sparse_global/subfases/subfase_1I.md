# Subfase 1I — Grafo sparse adaptativo con controles, covisibilidad y plan de propagación

## Estado

```text
pendiente
```

Estado anterior: `realizado`.

Se reabre porque el builder actual ya crea ventanas y protege fiduciales previos, pero todavía puede representar una trayectoria larga con pocos controles, usar aristas demasiado largas y no aprovechar la covisibilidad ni las observaciones de MapPoints que ya existen en `RawMapDatabase`.

Esta revisión sustituye el criterio de “máximo 24 vértices y aristas temporales” por un **grafo sparse adaptativo**. No se optimizan todos los KeyFrames, pero todos los KeyFrames de la ventana quedan representados por controles, segmentos y evidencia de covisibilidad.

Observación nueva desde los HTML `f1l_debug_animation_task_*`: el grafo actual
permite que la corrección se concentre en los últimos controles cercanos al
fiducial objetivo. Eso no es una deformación válida del submapa. `1I` debe
construir un grafo donde la cadena completa entre fiducial previo y fiducial
objetivo tenga controles suficientes para repartir la corrección sin crear
tramos largos que funcionen como bisagras implícitas.

---

## Objetivo técnico

Construir, para una tarea fiducial vigente, un `PoseGraphProblem` temporal que:

1. delimite correctamente el tramo entre el fiducial previo aceptado y el fiducial objetivo;
2. preserve la geometría local exportada por ORB-SLAM3;
3. seleccione KeyFrames de control por anchors, geometría, covisibilidad, calidad y cobertura;
4. use un número reducido pero suficiente de variables;
5. añada aristas temporales y de covisibilidad con medidas relativas completas;
6. reserve evidencia no utilizada por el solver para validación independiente en `1L`;
7. describa cómo propagar la corrección a los KeyFrames no seleccionados;
8. rechace el grafo si el presupuesto de controles no permite representar bien la ventana.

Esta subfase no ejecuta el solver y no modifica `GlobalPoseStore`.

---

## Principio central

Los fiduciales aportan información absoluta. ORB-SLAM3 aporta la geometría local.

```text
fiduciales        -> dónde debe quedar el tramo en world
poses relativas   -> cómo debe conservarse la forma local
covisibilidad     -> qué partes deben mantenerse coherentes entre sí
```

No usar ground truth para seleccionar vértices, crear aristas, calcular pesos ni decidir cobertura.

---

## Cambios conceptuales respecto a la versión anterior

Eliminar como criterio final:

```text
un límite fijo pequeño de vértices garantiza rendimiento y corrección
```

Sustituirlo por:

```text
presupuesto inicial pequeño
+ selección informada
+ verificación de cobertura
+ refinamiento adaptativo solicitado por 1J/1L
```

Eliminar la idea de que una arista interna debe guardar únicamente longitud. Cada arista debe conservar una transformación relativa completa. La libertad diferente entre traslación y rotación se implementará mediante pesos anisotrópicos en `1J`, no eliminando información de la medida.

---

## Contexto obligatorio a leer

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `pipeline_fase_1.md`
- versiones revisadas de `1H`, `1J`, `1K` y `1L`
- documentación de:
  - `RawMapDatabase`
  - `GlobalPoseStore`
  - `PoseGraphProblem`
  - `PoseGraphBuilder`
  - `orbslam3_msgs`
- historial reciente de Fase 1

Debe verificarse en los mensajes existentes que ya están disponibles:

- `OrbKeyFrame.connected_keyframe_ids`;
- `OrbKeyFrame.connected_keyframe_weights`;
- `OrbKeyFrame.mappoint_ids`;
- `OrbMapPoint.observations`;
- `OrbMapPoint.reference_keyframe_id`;
- poses raw de KeyFrames.

No rediseñar `orbslam3_msgs`.

---

## Diagnóstico de partida

La implementación actual ya dispone de:

- `PoseGraphVertex`;
- `PoseGraphEdge`;
- `PoseGraphPrior`;
- `PoseGraphPropagationPlanEntry`;
- protección de hard fiducials previos;
- muestreo por distancia acumulada;
- selección de esquinas por yaw;
- subdivisión limitada por gap y longitud;
- `segment_alpha` para propagación.

Problemas pendientes:

1. La ventana todavía puede depender demasiado de `local_kf_id +/- max_path_length`.
2. `max_vertices=24` puede impedir subdividir todos los tramos largos.
3. El builder puede devolver `success=true` aunque queden aristas fuera de límites.
4. Las medidas se construyen a partir de poses world actuales, no explícitamente de la geometría raw de ORB-SLAM3.
5. No existen aristas de covisibilidad activas.
6. `FiducialIdForKeyFrame()` copia y recorre el journal completo repetidamente.
7. No hay aristas reservadas para validación independiente.
8. No existe una salida formal para solicitar refinamiento de segmentos concretos.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

- `include/orbslam3_multi/raw_map_database.hpp`
- `src/raw_map_database.cpp`
- `include/orbslam3_multi/pose_graph_problem.hpp`
- `include/orbslam3_multi/pose_graph_builder.hpp`
- `src/pose_graph_builder.cpp`
- nuevos ficheros pequeños de índice de covisibilidad, solo si mejoran claridad
- `CMakeLists.txt`
- `package.xml` solo por dependencias reales

### `orbslam3_server`

- `src/global_map_server.cpp`, solo para integración, logs y parámetros
- `launch/global_orb_map_server.launch.py`

### Tests y documentación

- `orbslam3_multi/test/`
- `codex/archivos_auxiliares/`
- documentación de paquetes
- historial de Fase 1

---

## Archivos prohibidos

No modificar:

- `ORB_SLAM3/`;
- `orbslam3_ros2`;
- `orbslam3_msgs`;
- solver de `1J` salvo adaptación mínima de compilación posterior;
- `GlobalPoseStore` persistente salvo consultas de solo lectura;
- código legacy.

---

## Estructuras requeridas

### 1. Extender `PoseGraphVertex`

Campos mínimos o equivalentes:

```text
keyframe_id
initial_world_T_kf
raw_local_T_kf
is_variable
is_fixed
is_hard_fiducial
path_distance_m
normalized_path_s
selection_reason_flags
support_count
orb_covisibility_strength
```

`normalized_path_s` se calcula con distancia acumulada sobre la trayectoria raw, no con ID ni distancia euclídea directa al fiducial.

### 2. Extender tipos de arista

Tipos mínimos:

```text
TEMPORAL_CHAIN
COVISIBILITY
```

No usar `SoftConsistency` como nombre genérico si oculta la procedencia real.

Cada `PoseGraphEdge` debe guardar:

```text
edge_id
from_keyframe_id
to_keyframe_id
edge_type
edge_role = SOLVER | VALIDATION_HOLDOUT
raw_relative_T_from_to
path_kf_count
path_length_m
shared_mappoint_count
orb_covisibility_weight
confidence_translation
confidence_rotation
source
```

La medida canónica es:

```text
raw_relative_T_from_to = raw_local_T_from.inverse() * raw_local_T_to
```

No debe derivarse de las poses world ya corregidas. Las poses world son el estado inicial del solver; la medida relativa procede de raw.

### 3. Extender `PoseGraphProblem`

Añadir o formalizar:

```text
ordered_window_keyframes
solver_edges
validation_holdout_edges
coverage_summary
covisibility_summary
refinement_generation
max_control_budget
```

### 4. Resumen de cobertura

Campos mínimos:

```text
window_keyframes
control_vertices
max_control_span_kfs
max_control_span_m
uncovered_long_segments
mandatory_vertices_missing
low_covisibility_boundaries_without_control
coverage_complete
```

---

## Cambios requeridos en `RawMapDatabase`

### 1. Índice fiducial por KeyFrame

Evitar copiar el journal completo para cada consulta.

API orientativa:

```cpp
std::optional<RecordedFiducialObservation>
GetLatestFiducialObservationForKeyFrame(const RawKeyFrameId&) const;
```

### 2. Índice de covisibilidad por submapa

Construir o mantener una estructura consultable a partir de:

- `connected_keyframe_ids/weights` de ORB-SLAM3;
- observaciones de MapPoints, para verificar o complementar soporte.

API orientativa:

```cpp
CovisibilitySupport GetCovisibilitySupport(
    const RawKeyFrameId& a,
    const RawKeyFrameId& b) const;

std::vector<CovisibilityNeighbor>
GetStrongCovisibilityNeighbors(const RawKeyFrameId& kf) const;
```

`CovisibilitySupport` debe poder incluir:

```text
orb_weight
shared_mappoint_count
normalized_shared_support
valid
```

No recalcular todos los pares O(N²) en cada tarea. Actualizar el índice al insertar deltas/snapshots o construir una caché por revisión del submapa.

### 3. Orden y consistencia temporal

La ventana se ordena por secuencia lógica de KeyFrames del mismo epoch. Verificar:

- IDs crecientes;
- timestamps finitos;
- ausencia de cruce de epoch;
- disponibilidad de pose raw.

Si hay una anomalía de orden, registrar y rechazar o dividir la ventana.

---

## Algoritmo de construcción

### Paso 1 — Revalidar la tarea

Antes de construir:

- misma identidad `(drone_id,map_epoch)`;
- target existente en raw y world;
- residual actual todavía alto;
- tarea no stale ni superseded.

### Paso 2 — Delimitar la ventana fiducial

Encontrar el hard fiducial aceptado relevante anterior al target dentro del mismo submapa y rama temporal.

Ventana canónica:

```text
KF del fiducial previo aceptado
    ... todos los KFs intermedios del mismo epoch ...
KF objetivo asociado al nuevo fiducial
```

Si existen varios hard fiducials dentro de la ventana, todos deben quedar representados y protegidos.

No cruzar `map_epoch`.

Si no existe un fiducial previo válido, rechazar la optimización fiducial de dos extremos o construir un modo explícito diferente. No inventar una frontera por ID sin dejarlo registrado.

### Paso 3 — Construir la cadena raw completa

Para todos los KFs de la ventana:

- obtener `raw_local_T_kf`;
- calcular transformaciones consecutivas;
- calcular distancia acumulada;
- calcular yaw/rotación relativa;
- consultar covisibilidad con vecinos;
- calcular soporte de MapPoints.

Esta cadena completa se usa para seleccionar controles y planificar propagación, aunque no todos los KFs sean variables.

### Paso 4 — Seleccionar controles obligatorios

Siempre incluir:

- fiducial previo;
- fiducial objetivo;
- hard fiducials intermedios;
- vecindad configurable antes/después de cada fiducial;
- primero y último de cualquier segmento dividido por una discontinuidad de datos.

El fiducial previo es fijo. El target es variable con prior absoluto fuerte en `1J`.

### Paso 5 — Seleccionar controles geométricos

Añadir KeyFrames en:

- cambios importantes de yaw o curvatura;
- antes y después de una esquina;
- inicio y final de tramos largos;
- máximos locales de cambio de orientación;
- gaps temporales o métricos grandes.

Un giro grande es un **punto que debe representarse**, no una prueba de que la arista sea errónea.

### Paso 6 — Seleccionar controles por calidad/covisibilidad

Añadir controles alrededor de:

- caída fuerte de covisibilidad;
- pocos MapPoints compartidos;
- pérdida de soporte entre KFs consecutivos;
- transiciones donde `connected_keyframe_weights` cambie bruscamente;
- zonas donde una arista agregada tendría soporte débil.

No debilitar automáticamente una arista solo por su ángulo.

### Paso 7 — Completar cobertura mediante refinamiento greedy

Con los obligatorios ya seleccionados:

1. ordenar controles por distancia de trayectoria;
2. encontrar el segmento peor representado;
3. medir:
   - gap de KFs;
   - longitud recorrida;
   - curvatura interna;
   - mínimo soporte de covisibilidad;
4. insertar el mejor KF intermedio;
5. repetir hasta que la cobertura sea válida o se alcance el presupuesto.

Parámetros iniciales orientativos:

```text
initial_control_budget: 24-32
hard_control_budget: 48-64
max_control_span_kfs
max_control_span_m
max_unrepresented_yaw_change_rad
```

Los valores finales deben ajustarse con build y replay. El `hard_control_budget` no autoriza a devolver un grafo mal cubierto: si no basta, dividir/rechazar.

Regla visual convertida en requisito:

```text
si la propuesta optimizada mueve principalmente los últimos controles de la
ventana y deja casi inmóvil el resto del tramo entre fiduciales, la cobertura de
`1I` se considera insuficiente aunque `max_span_kfs` cumpla.
```

El refinamiento debe insertar controles donde el campo de correcciones tendría
un salto grande, no solo donde el gap topológico sea grande. La salida debe
poder señalar segmentos concretos para que `1J/1L` pidan:

```text
refinement_reason=correction_concentrated_near_target
suggested_segment=<control_a,control_b>
suggested_kf=<kf_intermedio>
```

### Paso 8 — Construir aristas temporales

Entre controles consecutivos de la ventana:

- medida relativa completa desde raw;
- registrar cuántos KFs se agregaron;
- registrar longitud y curvatura del tramo;
- confianza menor cuanto mayor sea el tramo o menor su soporte;
- no borrar la componente angular de la medida.

La diferencia de libertad entre traslación y rotación se resolverá con matrices de información en `1J`.

### Paso 9 — Construir aristas de covisibilidad

Añadir conexiones no temporales cuando:

- ambos controles pertenecen al mismo submapa;
- existe soporte suficiente;
- la arista añade rigidez y no es duplicado trivial de la temporal;
- se mantiene un límite de vecinos por vértice para conservar sparsidad.

Puntuación inicial posible:

```text
normalized_support = shared_mappoints / sqrt(obs_a * obs_b)
```

Combinar, cuando existan, `connected_keyframe_weights` y MapPoints compartidos.

No crear un clique completo.

### Paso 10 — Reservar aristas de validación

Seleccionar un subconjunto de aristas fuertes de covisibilidad como:

```text
edge_role = VALIDATION_HOLDOUT
```

Reglas:

- no pueden ser necesarias para mantener conectado el grafo del solver;
- deben cubrir zonas diferentes de la ventana;
- no se incluyen en la función objetivo;
- `1L` las usará para comprobar si la solución generaliza.

Esto proporciona una validación geométrica más independiente que comparar la propuesta contra sí misma.

### Paso 11 — Construir `PropagationPlan`

Para cada KF no control entre controles `A` y `B`:

```text
affected_kf
control_vertex_a
control_vertex_b
segment_alpha por distancia acumulada
path_distance_from_a_m
segment_length_m
control_span_kf_gap
```

`segment_alpha`:

```text
alpha = distancia acumulada A->KF / distancia acumulada A->B
```

No usar `local_kf_id` como interpolador salvo fallback documentado.

### Paso 12 — Validar cobertura antes de devolver éxito

Rechazar si:

- falta uno de los dos extremos fiduciales;
- un hard fiducial dentro de la ventana no está representado;
- queda un segmento por encima de límites;
- hay un cambio angular interno importante sin controles alrededor;
- demasiados KFs dependen de un solo control;
- no se puede construir al menos una cadena conectada de solver;
- la ventana cruza epochs;
- el presupuesto máximo no permite cubrirla.

---

## Logs obligatorios

```text
[F1I-WINDOW-SELECTED] task_id=<id> start_fid_kf=<id> target_fid_kf=<id> window_kfs=<n> path_m=<m>
[F1I-COVISIBILITY-INDEX] submap=(d,e) pairs=<n> source_orb=<n> source_mp=<n>
[F1I-CONTROL-SELECT] kf=<id> reason=<ANCHOR|CORNER|LOW_COVIS|COVERAGE|REFINEMENT>
[F1I-COVERAGE-ITERATION] controls=<n> worst_span_kfs=<n> worst_span_m=<m> action=<ADD|DONE|REJECT>
[F1I-TEMPORAL-EDGE] edge=<id> span_kfs=<n> path_m=<m> confidence_t=<v> confidence_r=<v>
[F1I-COVIS-EDGE] edge=<id> shared_mps=<n> orb_weight=<n> role=<SOLVER|HOLDOUT>
[F1I-PROPAGATION-SEGMENT] a=<id> b=<id> affected=<n> length_m=<m>
[F1I-COVERAGE-SUMMARY] complete=<bool> controls=<n> max_span_kfs=<n> max_span_m=<m> holdout_edges=<n>
[F1I-GRAPH-REJECT] reason=<...>
[F1I-GRAPH-BUILD-SUMMARY] ...
```

Telemetría por arista en `DEBUG`, resumen en `INFO`, rechazos en `WARN`.

---

## Tests unitarios obligatorios

1. Ventana entre dos fiduciales del mismo epoch.
2. Rechazo al cruzar epochs.
3. Inclusión de todos los hard fiducials internos.
4. Selección de controles en una trayectoria con varias esquinas.
5. Inserción greedy hasta cumplir gap y distancia.
6. Rechazo si el hard budget no permite cobertura.
7. Construcción de covisibilidad desde `connected_keyframe_*`.
8. Verificación con observaciones de MapPoints.
9. Grafo sparse: límite de vecinos y ausencia de clique.
10. Holdout no desconecta el grafo del solver.
11. `segment_alpha` correcto por distancia recorrida.
12. Medida relativa obtenida de poses raw, no de world corregido.
13. Una misma entrada raw produce el mismo grafo en live y replay.

---

## Pruebas de integración

### Prueba 1 — replay del caso de deriva grave

Usar el `.record` que generó el HTML con giro grande.

Comprobar:

- controles antes, durante y después de los giros relevantes;
- ningún tramo representa una parte grande del edificio sin subdivisión;
- existen aristas de covisibilidad;
- todos los KFs quedan asignados a un segmento.

### Prueba 2 — deriva pequeña

Una ventana con error fiducial moderado debe construir un grafo más pequeño, sin llenar el hard budget innecesariamente.

### Prueba 3 — ventana larga

Demostrar que el builder:

- refina hasta cobertura;
- o rechaza/divide;
- nunca devuelve `success=true` con edges fuera de límites.

---

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi
./codex/herramientas/build_selected_packages.sh orbslam3_server
```

---

## Criterio de éxito

`1I` queda `CONSEGUIDA` si:

- la ventana está delimitada por fiduciales reales del mismo submapa;
- el grafo usa geometría raw y no poses world como medida;
- la cobertura es verificable y completa;
- no quedan aristas largas sin representar;
- existen controles adaptativos alrededor de cambios geométricos y de covisibilidad;
- se construyen aristas de covisibilidad sparse;
- existe holdout independiente;
- el plan de propagación cubre todos los KFs omitidos;
- el número de controles se mantiene acotado o el problema se rechaza/divide;
- no se usa GT.

---

## Criterio de fallo o parcial

`PARCIAL` si se construye el índice y el grafo, pero falta holdout o refinamiento.

`NO CONSEGUIDA` si:

- se acepta mala cobertura;
- se usan medidas derivadas del estado world ya deformado;
- se cruzan epochs;
- se ignoran hard fiducials;
- el builder depende de GT;
- se vuelve a un muestreo uniforme por ID como criterio principal.

---

## Documentación a actualizar

- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`
- `codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md`
- `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- `codex/contexto/paquetes/orbslam3_server/launches.md`
- historial de Fase 1

Mover evidencia histórica extensa al historial. La subfase debe describir el contrato vigente, no acumular diarios de ejecución.

---

## Entrega a la subfase 1J

`1I` entrega un `PoseGraphProblem` inmutable con:

```text
controles suficientes
poses iniciales world
poses raw locales
medidas relativas completas
pesos/confianzas separados
aristas temporales
aristas de covisibilidad
aristas holdout
priors fiduciales
plan de propagación
resumen de cobertura
```

`1J` no debe volver a consultar raw para redefinir el grafo ni cambiar silenciosamente su topología.
