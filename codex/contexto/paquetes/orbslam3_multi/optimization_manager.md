# `OptimizationManager`

## Rol actual

`OptimizationManager` es la clase de `orbslam3_multi` introducida en `1J` para ejecutar un dry-run sobre un `PoseGraphProblem` y ampliada en `1K` para aplicar resultados útiles en `GlobalPoseStore`. Los marcadores `F1L` validan/diagnostican el estado post-apply, pero la mutación persistente pertenece conceptualmente a `1K`.

Notas históricas reclasificadas 2026-07-12 a 2026-07-14:

Aunque muchas pruebas y artefactos se llamaron `f1l_*` porque se ejecutaron
durante la validación de `1L`, la propiedad conceptual queda así:

- `1J`: solver, pesos, dry-run, replay offline, dump TSV y HTML diagnóstico;
- `1K`: apply, propagación a KFs no vértice y corrección heredable;
- `1L`: lectura post-apply, comparación con logs/GT debug, decisión de aceptar,
  parcial o rollback.

```text
La infraestructura existe, pero 1L no esta cerrada. `RunDryRun` ya usa un
solver minimo propio de pose graph sobre variables `x,y,z,yaw`, priors y edges
locales, sin usar `g2o` de ORB-SLAM3. La primera revalidacion live aun mostro
fallos en RViz2: el dron antihorario no corrigio bien al llegar al fiducial 1
y algunas correcciones fueron rechazadas por crecimiento interno.

El ultimo ajuste tecnico convierte el solver a 6 variables por vertice:
`x,y,z,roll,pitch,yaw`. Yaw sigue siendo el giro dominante; roll/pitch entran
con peso pequeno. Las aristas internas preservan fuerte la longitud/distancia
relativa, mantienen la direccion de traslacion como sesgo debil y dejan yaw
relativa casi libre. Compila y en Gazebo acepta un candidato `dron_2`
antihorario: fiducial 1 `9.792646 m -> 0.009804 m`, ventana GT
`mean_after=1.762549`, `max_after=4.030853`, `internal_max_after=0.027445`.
La iteracion offline posterior reactivo la rigidez angular variable, pero ya no
usa solo posicion en la ventana: combina soporte de KFs, rigidez extra cerca de
fiduciales, giro relativo bruto de la arista y vertices `corner_yaw_vertex`.
Sobre `f1l_graph_task_2.tsv` desplaza la flexion angular desde la arista
soportada `150->161` hacia `127->129`, conservando cambios de longitud
pequenos. Aun no cierra `1L`: el GT medio offline queda peor que la variante
uniforme anterior y falta otra iteracion.

El ajuste siguiente añade una restricción de bisagra/corner sobre triples
consecutivos `A-B-C`: preserva el giro firmado XY en vértices fiables. El peso
sube cerca de fiduciales y con soporte local alto, y baja en el centro pobre de
la U. En `f1l_graph_task_2.tsv` evita que el corner `143->150->161` invierta el
signo de giro: `before_turn=1.42674`, `after_turn=1.42488`,
`sign_flipped=false`. Aun así, sigue siendo una mejora local: el GT offline
queda en `mean_after=5.47334`, peor que la variante uniforme anterior.

Iteración posterior: el replay offline distingue ahora métricas relativas
locales (`before_rel_z`, `before_xy_len`) de métricas globales de mapa
(`before_map_xy_len`, `before_map_delta_z`). `before_rel_z=-1.802` en
`127->129` no era altura global RViz2; la altura global inicial era ~`0.36 m`.
El problema real era distancia XY global corta: `before_map_xy_len=1.818`
frente a `gt_map_xy_len=4.908` solo diagnóstico. Se probó relajar la traslación
de aristas con poco soporte, zona central y proyección local/global anómala:
`127->129` pasó a `after_map_xy_len=3.071`, pero el replay quedó `PARCIAL`
(`mean_after=4.82182`, `max_after=10.2946`) por inversiones en esquinas
fiables `150`/`161`. No usar GT para pesos; mantener estas métricas globales
como diagnóstico y como señal interna solo si procede de poses mapa.

Iteración siguiente: el cierre artificial alrededor de `kf=143` se debía a que
las magnitudes relativas locales podían conservarse mientras la diferencia Z
global por arista crecía y la proyección XY se plegaba. El solver añade ahora
un residual de Z global por arista (`kEdgeGlobalVerticalResidualScale`) además
del residual local, y reduce la relajación máxima de arista pobre a `0.55`. En
replay offline `f1l_graph_task_2.tsv`, la variante
`f1l_offline_graph_task_2_reliable_hinge_4.html` mejora a
`mean_after=3.28931`, `max_after=5.84293`, `kf143 after_turn=-0.454941` y
`kf161 sign_flipped=false`. Sigue pendiente validación Gazebo/apply.

Pruebas posteriores rechazadas: `piecewise_blocks_1..2` aumentó la rigidez del
gradiente de traslación cerca de fiduciales, y `global_xy_guard_1..2` añadió un
guardarraíl de longitud XY global para aristas con soporte alto. Ambas líneas
fallaron sobre `f1l_graph_task_2.tsv`: abren o mueven la deformación de
`127->129`, pero rompen tramos fiables (`150->161`, `kf143`, `kf150`, `kf161`)
y empeoran el GT diagnóstico frente a `reliable_hinge_4`. El código activo
volvió a `reliable_hinge_4`; no seguir ajustando pesos/guards dentro de la
misma parametrización por vértice.

Modificación activa posterior: el solver dejó de usar yaw como restricción
rotacional principal. Los tres estados angulares son ahora el vector
`LogSO3(delta_R)` y priors/aristas penalizan rotación 3D completa. El
`FiducialTarget` se fija durante el solve a su pose 6D del prior; el resto del
grafo se deforma contra ese target. Para evitar mínimos locales donde la cadena
arranca al lado equivocado, el solver inicializa las variables con una
back-propagación desde el target usando las aristas SE(3) guardadas. La rigidez
local SE(3) vigente bloquea de forma fuerte la cola del fiducial 1: las edges
con ambos extremos en `alpha >= 0.965` o tocando el target usan rigidez `100.0`.
La rampa suave cerca del target se mantiene desde `alpha=0.86` hasta `20.0`; la
edge directa al fiducial previo usa `25.0`, sin extender una U amplia hacia el
interior. Además, el soporte de KeyFrames aumenta más el score: las aristas
usan `support_rigidity_multiplier^0.5` en residuos SE(3) y las bisagras/corners
usan `support_rigidity_multiplier^1.2` con `kCornerHingeResidualScale=3.0`.
`YawFromTransform` permanece como métrica/log heredado, no como coste rotacional
principal.

Replay offline vigente sobre `f1l_graph_task_1.tsv`:
`f1l_offline_graph_task_1_so3_target_tail_pose_lock_5.html`; replay de
verificación equivalente:
`f1l_offline_graph_task_1_so3_target_tail_pose_lock_restored_10.html`.
Evidencia: target `19.0658 m -> 0 m`, orientación 3D
`1.78423 rad -> 0 rad`, GT diagnóstico `mean_after=1.15554`,
`max_after=2.57206`, `useful=true`.

Cambio activo: además de rigidez de arista, el solver construye una cola rígida
relativa al `FiducialTarget`. Toma el prior 6D del target, recorre hacia atrás
las últimas `3` aristas temporales y añade residuos de pose para esos vértices
esperados. Así los KFs cercanos al fiducial 1 se mueven como bloque rígido con
el target. Las pruebas `_8/_9`, basadas en offset global de posición, quedan
descartadas como línea activa porque empeoran el GT global.

Aplicación simétrica corregida después: las vecindades de ambos fiduciales se
pinchan como poses inducidas antes del solve, no como simples residuales
fuertes. `BuildFiducialNeighborhoodPoseLocks` combina:

```text
fiducial previo anclado -> vecinos = T_anchor * T_anchor_vecino_raw
fiducial objetivo 6D   -> vecinos = T_target_nueva * T_target_vecino_raw
```

Esos vecinos se excluyen de `variable_ids` y `PinFiducialNeighborhoodPoseLocks`
actualiza sus `base_states` antes de construir residuales. Se siguen aplicando
como propuestas optimizadas en el dry-run, pero no participan como variables
libres. El objetivo es que ambos extremos mantengan rígida su geometría local
respecto al fiducial correspondiente.

Replay offline:
`f1l_offline_graph_task_1_so3_fiducial_neighborhood_pose_locks_13.html`.
Resultado: `success=true`, `useful=true`, target `t=0`, `rot=0`,
GT diagnóstico `mean_after=0.822023`, `max_after=1.78632`. Las aristas
`55->56`, `56->57`, `57->58`, `159->160`, `160->161` y `161->162` quedan con
`delta_len` numéricamente cero. Sigue pendiente inspección visual/live porque
`kf=110` mantiene `sign_flipped=true`.

Pruebas descartadas: `support_dense_lock_1` y `support_dense_hinge_2` intentaron
convertir zonas densas en rígidas solo subiendo pesos y/o bisagras; quedaron
`useful=false` o empeoraron el GT. El bloque de residuo directo de KeyFrames
intermedios permanece en código como experimento desactivado con
`kPropagatedKeyFrameShapeTranslationScale=0.0` y
`kPropagatedKeyFrameShapeRotationScale=0.0`.
```

Su pregunta tecnica es:

```text
Si aplicasemos esta optimizacion, bajarian el coste y el error de la tarea?
```

En `1J` no aplica resultados. Trabaja sobre copias temporales de poses, devuelve un `OptimizationDryRunResult` en memoria y puede alimentar dump/replay/HTML diagnóstico del grafo. En `1K`, si ese resultado es aceptable, `ApplyUsefulResult`/`ApplyCandidateResult` escriben las poses optimizadas/propagadas en `GlobalPoseStore` sin modificar `RawMapDatabase`, fijan la corrección heredable para KFs futuros y habilitan la publicación coherente. En `1L`, `ValidatePostApply` y los logs asociados diagnostican si el estado aplicado debe considerarse válido.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/optimization_manager.hpp
orbslam3_multi/src/optimization_manager.cpp
orbslam3_multi/include/orbslam3_multi/optimization_result.hpp
```

## API principal

```cpp
struct OptimizationManagerConfig;
struct OptimizationDryRunResult;
struct OptimizationApplyResult;
struct PostApplyValidationResult;

class OptimizationManager
{
public:
    void Configure(const OptimizationManagerConfig& config);
    const OptimizationManagerConfig& GetConfig() const;

    OptimizationDryRunResult RunDryRun(
        const PoseGraphProblem& graph,
        const RawMapDatabase& raw_db,
        const GlobalPoseStore& pose_store) const;

    OptimizationDryRunResult RunDryRunGraphOnly(
        const PoseGraphProblem& graph) const;

    OptimizationApplyResult ApplyUsefulResult(
        const OptimizationDryRunResult& dry_run,
        const PoseGraphProblem& graph,
        const RawMapDatabase& raw_db,
        GlobalPoseStore& pose_store) const;

    OptimizationApplyResult ApplyCandidateResult(
        const OptimizationDryRunResult& dry_run,
        const PoseGraphProblem& graph,
        const RawMapDatabase& raw_db,
        GlobalPoseStore& pose_store,
        bool allow_partial_candidate) const;

    PostApplyValidationResult ValidatePostApply(
        const OptimizationDryRunResult& dry_run,
        const PoseGraphProblem& graph,
        const OptimizationApplyResult& apply,
        const GlobalPoseStore& pose_store,
        bool force_reject) const;
};
```

`RunDryRunGraphOnly` es la ruta diagnóstica de `1J` para ejecutar el solver
offline sobre un `PoseGraphProblem` serializado. Reutiliza el mismo dry-run que
`RunDryRun`, pero no consulta `RawMapDatabase` ni `GlobalPoseStore` y no genera
poses persistentes. Sirve para iterar pesos/solver y generar HTML sin repetir
Gazebo; no sustituye la prueba live de apply/publicación de `1K`.

## Solver minimo vigente 1J

- `RunDryRun` resuelve un grafo minimo con variables `x`, `y`, `z` y vector
  `LogSO3(delta_R)` para vertices no fijos. No optimiza yaw como variable
  especial.
- Los priors fiduciales tiran hacia la pose absoluta esperada.
- Los priors fiduciales son 6D: posición y orientación completa. Solo los priors
  `CurrentPoseSoft` tienen escala suave reducida.
- El vértice `FiducialTarget` se excluye de variables y se fija a su prior 6D
  durante el solve; el resto del grafo se deforma contra esa pose.
- Los priors de pose actual se escalan suavemente segun distancia/confianza a
  fiduciales, para evitar fijar binariamente todo el entorno.
- Las edges internas penalizan fuerte el cambio de longitud/distancia relativa
  entre KFs. La direccion del vector de traslacion queda como sesgo debil para
  permitir que la cadena se doble.
- Además de la traslación relativa local, cada edge penaliza el cambio de Z
  global entre sus vértices. Esto evita que el solver preserve longitudes
  locales pero cierre la proyección XY metiendo diferencia vertical artificial,
  como ocurría alrededor de `kf=143`.
- Las edges internas penalizan rotación relativa con residual SO(3) completo.
  `YawFromTransform` queda para métricas heredadas; no es el coste principal.
- Las vecindades de fiduciales tienen protección dura por pose inducida: las
  últimas `3` aristas hacia el target y las primeras `3` aristas desde el ancla
  previa se reconstruyen desde la pose final del fiducial correspondiente y se
  excluyen de la optimización libre.
- Las aristas y bisagras usan soporte de KeyFrames como score de fiabilidad más
  marcado que antes. En este grafo no basta para recuperar el ángulo recto de
  `kf=135`; probablemente se necesitan más controles/vértices o una restricción
  geométrica explícita.
- Existe un bloque experimental para que los KeyFrames intermedios aporten
  rigidez de forma, pero las escalas están a `0.0`. La prueba offline con
  escalas no nulas empeoró coste y GT, así que no forma parte del solver activo.
- La rigidez angular del campo de correccion usa una señal combinada:
  soporte de KFs, perfil parabolico entre fiduciales, giro relativo bruto de la
  arista y vertices `corner_yaw_vertex`. La traslacion del campo de correccion
  no usa la misma parabola: se suaviza por soporte para evitar concentrar todo
  el estiramiento en el centro de la U.
- Los triples consecutivos de aristas tienen un coste de bisagra firmado en XY:
  se compara el giro `A->B->C` actual con el giro inicial. Este coste protege
  esquinas fiables cerca de fiduciales, como `143->150->161`, donde no debe
  cambiar el sentido izquierda/derecha aunque el fiducial objetivo tire fuerte.
- La hipótesis activa para aristas pobres es relajar tambien la traslacion
  relativa solo cuando haya señal interna suficiente: poco soporte, posición
  central y proyección XY local/global anómala. El caso objetivo es `127->129`.
  Debe aplicarse de forma gradual, sin usar GT, sin mover hard fiducials y
  manteniendo rigidez fuerte cerca de fiduciales.
- Las aristas usan soporte de KeyFrames como señal interna de fiabilidad:
  `support_density_kfs_per_m` aumenta la rigidez si hay muchos KFs por metro y
  la reduce si hay pocos. El factor queda saturado y no usa GT. Si una arista
  tiene giro relativo alto, el solver puede rebajar la rigidez traslacional para
  admitir que ORB-SLAM3 subestimó también la distancia en una zona pobre.
- `ValidatePostApply` usa la misma idea para `internal_error`: una traslacion
  relativa estirada/comprimida se registra como warning de calidad; desde la
  prueba live del 2026-07-14 ya no fuerza rollback si el fiducial objetivo queda
  en umbral y los invariantes duros pasan.

## Replay offline de grafo

Archivos nuevos:

```text
orbslam3_multi/include/orbslam3_multi/pose_graph_problem_io.hpp
orbslam3_multi/src/pose_graph_problem_io.cpp
orbslam3_multi/src/test_opt_graph_offline.cpp
```

`pose_graph_problem_io` guarda/carga `PoseGraphProblem` en TSV versionado
`F1L_POSE_GRAPH_DUMP\t1`. El ejecutable offline se lanza así:

```bash
ros2 run orbslam3_multi test_opt_graph_offline --graph /ruta/f1l_graph_task_1.tsv
```

Con HTML:

```bash
ros2 run orbslam3_multi test_opt_graph_offline \
  --graph /ruta/f1l_graph_task_1.tsv \
  --html /ruta/f1l_offline_graph_task_1.html
```

Salida esperada:

```text
[F1L-OFFLINE-LOAD]
[F1L-OFFLINE-DRYRUN]
[F1L-OFFLINE-METRICS]
[F1L-OFFLINE-EDGE-SUPPORT]
[F1L-OFFLINE-GT-STATS]
[F1L-OFFLINE-HTML]
```

El replay emite tambien `[F1L-OFFLINE-EDGE-DEFORMATION]` por arista, con
longitud/yaw relativos before/after, gradiente de correccion y métricas de mapa
global (`before_map_xy_len`, `after_map_xy_len`, `before_map_delta_z`,
`after_map_delta_z`). Para comparar casos como `127->129` frente a `150->161`,
usar las métricas `map_*`; `before_rel_z` es componente local de
`T_from^-1*T_to` y no equivale a la altura global visible en RViz2.

Para series de pruebas offline con HTML, usar nombres numerados por intento y
reutilizar/sobrescribir la serie anterior cuando se repita el mismo experimento:
`..._1.html`, `..._2.html`, `..._3.html`, `..._4.html`.

El replay emite `[F1L-OFFLINE-CORNER-DEFORMATION]` por triple `A-B-C`. Para
validar inversiones visuales de esquina, mirar `center_kf`, `before_turn`,
`after_turn`, `delta_turn` y `sign_flipped`.

Si el dump contiene `debug_keyframes`, el offline calcula una propagación
diagnóstica para KFs no vértice y puede comparar before/after contra GT externo
en logs y HTML. Sigue sin validar apply real, rollback ni publicación de nube.

Estado de validacion:

- build de `orbslam3_multi` y `orbslam3_server`: correcto;
- simulacion con GT debug posterior: ejecutada desde `orbslam3_server` con
  `f1l_gt_kf_debug_enabled:=true`; confirma que el fiducial objetivo puede
  bajar a error centimetrico/milimetrico, que las aristas internas quedan
  coherentes y que la ventana completa mejora mucho, aunque aun no basta para
  cerrar `1L` sin inspeccion visual y sin resolver el `max_after` alto.

## Metricas GT externas implementadas fuera del solver

`OptimizationManager` no debe consumir ground truth. Para diagnosticar `1L`,
`global_map_server` calcula metricas GT por KeyFrame antes y despues de llamar
al solver, pero esas metricas quedan fuera del problema de optimizacion.

Permitido:

- comparar `OptimizationDryRunResult` y `PostApplyValidationResult` contra logs
  externos `[F1L-GT-*]`;
- usar esas metricas para explicar una observacion de RViz2;
- documentar si el error medio/maximo real de la ventana baja o sube.

Prohibido:

- pasar poses GT a `RunDryRun`;
- usar GT para seleccionar vertices;
- usar GT para cambiar pesos de priors/edges;
- usar GT para decidir `useful`, `partial_candidate`, `ACCEPT`,
  `PARTIAL_KEEP_FOR_NEXT_PASS` o `REJECT_ROLLBACK`;
- escribir en `GlobalPoseStore` usando GT.

La regla practica es:

```text
El GT debug mira y acusa; no toca el volante.
```

Resultado observado en la primera prueba corta:

```text
task_id=1: 19.776323 -> 0.013640 -> 0.000019 m en el KF fiducial objetivo,
pero `[F1L-GT-WINDOW-STATS]` queda alrededor de `mean_after=4.27 m` y
`max_after=10.76 m`.
```

## Comportamiento 1J

- Recibe un `PoseGraphProblem` ya construido por `PoseGraphBuilder`.
- Comprueba precondiciones: vertices, variables, restricciones, poses finitas y pesos validos.
- Copia poses `world_T_kf` a estructuras temporales.
- Calcula coste inicial y coste final sobre la copia.
- Para tareas fiduciales, la ruta antigua repartia una correccion rigida hacia
  el prior objetivo; desde la reapertura de `1L`, la ruta activa usa el solver
  minimo descrito arriba.
- Respeta vertices fijos y KFs hard fiducial.
- Desde la revalidacion del 2026-07-10, consume `PoseGraphProblem::anchor_preservation` y expone si el dry-run preserva el fiducial previo seleccionado por `1I`.
- Calcula `before_error_t`, `after_error_t`, yaw before/after e `improvement_ratio`.
- Calcula propuestas de propagacion para KFs no variables afectados.
- Decide `useful` usando coste, reduccion de error, limites de delta y proteccion de fijos.
- Si no llega a `useful=true` pero reduce mucho el error y sigue por encima del umbral final, marca `partial_candidate=true` con `partial_reason=large_error_reduced_but_above_threshold`.
- No marca `useful=true` ni `partial_candidate=true` si `anchor_preservation_required=true` y la preservacion no queda demostrada con `previous_fiducial_fixed_count>0`, `graph_satisfied=true`, `hard_fixed_moved=false` y delta nulo del fiducial previo.

## Configuracion

`OptimizationManagerConfig` contiene:

- `dryrun_min_improvement_ratio`
- `dryrun_partial_min_improvement_ratio`
- `dryrun_max_allowed_delta_t`
- `dryrun_partial_max_allowed_delta_t`
- `dryrun_max_allowed_delta_yaw`
- `dryrun_max_final_error_t`
- `dryrun_require_cost_decrease`
- `solver_step_fraction`
- `post_apply_internal_broken_edge_t`
- `post_apply_internal_max_growth_t`

El servidor expone estos valores como parametros `f1j_*`.

## Resultado

`OptimizationDryRunResult` contiene:

- `success`, `useful`, `reason`, `decision_reason`;
- `partial_candidate`, `partial_reason`;
- `initial_cost`, `final_cost`;
- `before_error_t`, `after_error_t`;
- `before_error_yaw`, `after_error_yaw`;
- `improvement_ratio`;
- propuestas de poses de vertices y KFs propagados;
- estadisticas de deltas;
- bandera `hard_fixed_moved`;
- campos de preservacion de anclaje:
  - `anchor_preservation_required`;
  - `anchor_preservation_graph_satisfied`;
  - `anchor_preservation_ok`;
  - `previous_fiducial_fixed_count`;
  - `max_previous_fiducial_delta_t`;
  - `max_previous_fiducial_delta_yaw`;
- ruta de debug opcional si se exporta SVG.

El resultado viaja en memoria. No se serializa a CSV para que el servidor lo lea.

`OptimizationApplyResult` contiene, entre otros:

- `optimized_kfs`;
- `propagated_kfs`;
- `skipped_propagated_kfs`;
- `last_correction_set`;
- registros `optimized_records`;
- registros `propagated_records`;
- flags `raw_db_modified` y `global_pose_store_modified`.

`skipped_propagated_kfs` es importante para `1L`: indica que el dry-run proponia mover KFs no variables, pero el apply parcial no los escribio porque publicar esa propagacion antes de reconstruir grafo podria desplazar zonas buenas del submapa.

## Comportamiento 1K

- Comprueba que el `task_id` del dry-run y del grafo coinciden.
- Exige `dry_run.success=true` y `dry_run.useful=true`.
- Rechaza `partial_candidate=true/useful=false` con razon pendiente para `1L`.
- Desde la revalidacion del 2026-07-11, si `anchor_preservation_required=true`, exige `anchor_preservation_graph_satisfied=true`, `anchor_preservation_ok=true`, `previous_fiducial_fixed_count>0` y `fixed_kfs>0` antes de permitir cualquier escritura persistente.
- Revalida que coste/error bajan, que no hay NaN y que los deltas estan dentro de limites.
- Verifica que no se mueven KFs hard fiducial ni fijos.
- Aplica propuestas de vertices mediante `GlobalPoseStore::SetOptimizedKeyFramePose`.
- Aplica propuestas propagadas mediante `GlobalPoseStore::SetPropagatedKeyFramePose` solo para resultados normales aceptables; para parciales, la propagacion amplia puede omitirse y quedar contabilizada como `skipped_propagated_kfs`.
- Devuelve `OptimizationApplyResult` con contadores, deltas, registros por KF y flags `raw_db_modified=false`, `global_pose_store_modified=true`.

## Comportamiento post-apply 1L

- `ApplyCandidateResult` conserva el comportamiento seguro de `ApplyUsefulResult`, pero permite aplicar `partial_candidate=true/useful=false` solo si el servidor lo autoriza con `allow_partial_candidate=true`.
- Los parciales usan fuente diferenciada `SERVER_OPTIMIZATION_PARTIAL` para vertices realmente escritos.
- Desde el diagnostico RViz2 del 2026-07-11, los parciales no deben publicar una propagacion amplia de KFs no variables como salida normal. Si `dry_run.proposed_propagated_poses` contiene muchos KFs con deltas grandes, `ApplyCandidateResult` debe contabilizarlos como `skipped_propagated_kfs` y dejar que `1L` exija reconstruccion de grafo antes de una propagacion amplia. La escritura/propagación en sí pertenece a `1K`; aquí se documenta la política de validación posterior.
- `ValidatePostApply` recalcula el error real desde `GlobalPoseStore`, no desde `RawMapDatabase`.
- La validacion compara `before_error_t`, `predicted_after_error_t` y `real_after_error_t`.
- Recalcula residuos internos de aristas del `PoseGraphProblem` antes/despues.
  En la regla vigente, el residuo translacional interno representa cambio de
  longitud/distancia relativa entre KFs. La direccion local queda como sesgo del
  solver, no como guard principal, para que el mapa pueda doblarse angularmente.
- Clasifica aristas internas como fuertes o deformables para diagnóstico, pero
  `internal_edges_broken` ya no obliga a rollback por sí solo.
- Una arista deformable representa una restriccion ORB-SLAM3 sospechosa, por ejemplo una esquina falsa de casi `90 deg` creada en una pared poco texturizada o una arista `DIRECT_UNCERTAIN`/`SUBDIVISION_CANDIDATE` cerca de un residual fiducial absurdo.
- La yaw relativa puede cambiar mucho en esas aristas; lo que no debe romperse
  es la distancia relativa entre KFs consecutivos o covisibles. Roll/pitch
  pueden cambiar poco si ayuda a no comprar la correccion con traslacion.
- Comprueba que KFs fijos y hard fiducials no se movieron.
- Desde la revalidacion del 2026-07-11, comprueba explicitamente preservacion de anclajes fiduciales previos con `PostApplyAnchorPreservationCheck`.
- La preservacion post-apply exige que los vertices `previous_fiducial_anchor` sigan en su pose inicial del grafo y que las ramas/subdivisiones marcadas por `1I` no permitan aceptar una mejora comprada a costa de mover el fiducial anterior.
- Comprueba y loggea la propagación de KFs no vértice. Si queda discontinua,
  `propagation_ok=false` es diagnóstico de calidad y deuda de refinamiento, no
  condición de rollback mientras hard fixed/anchors y error fiducial estén OK.
- Devuelve `PostApplyDecision`:
  - `ACCEPT` si el error real queda en umbral y las invariantes duras pasan;
  - `PARTIAL_KEEP_FOR_NEXT_PASS` si el error baja mucho, sigue fuera de umbral, no rompe invariantes duras y solo deforma aristas internas deformables;
  - `REJECT_ROLLBACK` si se fuerza rechazo, no mejora, mueve protegidos, rompe
    anchors, aparece NaN o falla otra invariante dura.
- Añade `RetrySuggestion` para guiar la siguiente pasada, por ejemplo `REBUILD_GRAPH_FROM_PARTIAL_STATE`, `DIFFERENT_VERTEX_SELECTION`, `STRONGER_INTERNAL_EDGES`, `CHECK_FIDUCIAL_ASSOCIATION` o `BUG_FIXED_SELECTION`.

`1L` no implementa reintento automatico. Solo valida, decide y diagnostica.

## Restricciones

- No modifica `RawMapDatabase`.
- En `RunDryRun`, no modifica `GlobalPoseStore` ni llama a `SetOptimizedKeyFramePose`.
- En `ApplyUsefulResult`, modifica solo `GlobalPoseStore`.
- No publica correcciones ROS.
- No almacena un grafo global persistente.
- No usa ground truth como entrada de solver, decision o apply. Puede ser
  evaluado externamente por logs debug calculados fuera de esta clase.
- No usa CSV/SVG como canal normal de datos.
- En `1L`, no confirma una optimizacion que empeora el error real o rompe hard fiducials.
- En `1L`, no confirma ni conserva un parcial si falla `[F1L-ANCHOR-PRESERVATION-CHECK]`.
- En `1L`, un parcial conservado no equivale a mapa final cerrado: queda como deuda para segunda pasada.
- En `1L`, un error fiducial absurdo (`>=10 m`) no debe quedar sin corregir si
  una primera optimizacion lo reduce hasta umbral y preserva anclajes/hard
  fiducials, aunque una arista interna deformable o la propagación amplia queden
  marcadas como diagnóstico.
- En `1K`, tras escribir un candidato validable, debe fijarse
  `last_correction_set=true` desde el KF aplicado con mayor `local_kf_id` para
  que los KFs nuevos sigan el extremo optimizado, no la transformada previa del
  submapa.
- En `1K`, esa herencia futura debe sobrevivir a full snapshots: si aparece un
  KF posterior al `from_keyframe`, `GlobalPoseStore` lo registra con
  `SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT` y no permite que una
  reconciliación posterior lo rebaje al anchor base.
- En `1L`, `ValidatePostApply` diagnostica la propagación aplicada por `1K`; si
  falla, el estado debe restaurarse y quedar documentado.
- En `1L`, el rollback de un reintento posterior debe volver al ultimo checkpoint parcial seguro si existe, no necesariamente al estado absurdo original.

Reejecucion `1L` del 2026-07-12:

- `ApplyCandidateResult` aplica vertices y propagados bajo backup;
- `[F1K-OPT-APPLY-SUMMARY]` debe mostrar `last_correction_set=true` cuando hay
  KFs movidos, y `[F1K-POSESTORE-LAST-CORRECTION-SET]` debe apuntar al KF más
  nuevo aplicado;
- los KFs nuevos posteriores deben emitir
  `source=SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT` si heredan un apply
  parcial aceptado;
- si un full snapshot toca KFs posteriores al `from_keyframe`, deben mantenerse
  con la corrección heredada y no aparecer como `F1G_FULL_SNAPSHOT_RECONCILE`
  hacia el anchor base;
- el rollback sigue siendo la barrera de seguridad si la validación post-apply
  rechaza el candidato.

## Validacion 1J

El 2026-07-09 se valido con:

- build `orbslam3_multi`, `orbslam3_server` y `simulacion_dron` con `BUILD-EXIT-CODE 0`;
- prueba live `prueba_1`: `19` dry-runs `success=true/useful=true`;
- prueba replay `prueba_2`: `17` dry-runs `success=true/useful=true`, incluyendo `task_id=9000000001`;
- `prueba_1.reduced.log`: ejemplo `before_t=0.403252 after_t=0.020480 improvement_ratio=0.949214`;
- `prueba_2.reduced.log`: ejemplo `before_t=2.693890 after_t=0.163607 improvement_ratio=0.939267`;
- `[F1J-OPT-NO-STATE-MUTATION] ... ok=true` en las tareas validadas;
- sin `OPT-APPLY`, `OPTIMIZATION-APPLIED`, `SET_OPTIMIZED_POSE`, `MAP_CORRECTION_PUBLISH` ni `CORRECTED_KEYFRAMES_PUBLISH`.

Durante el desarrollo de `1J` el primer replay revelo un fallo del solver inicial: rotaba la ventana alrededor del origen y aumentaba `after_t`. Se corrigio calculando la correccion relativa desde la pose actual del KF objetivo hacia su target fiducial.

## Revalidacion partial_candidate 2026-07-09

Tras reabrir `1J`, se valido la clasificacion parcial:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`: `BUILD-EXIT-CODE 0`;
- prueba live: `107` dry-runs, `83` `useful=true` y `24` `partial_candidate=true`;
- prueba replay/debug: `17` dry-runs `useful=true`;
- ejemplo:
  ```text
  task_id=69 useful=false partial_candidate=true reason=large_error_reduced_but_above_threshold before_t=12.038334 after_t=0.710850 improvement_ratio=0.940951
  ```

Un `partial_candidate` no se aplica en `1J` y no equivale a `ACCEPT`; solo preserva la evidencia para que `1K/1L` puedan decidir con backup, validacion inmediata y rollback.

## Revalidacion 1J con anclajes previos 2026-07-10

Tras reparar `1I`, `OptimizationManager::RunDryRun` se revalido con grafos que incluyen un fiducial previo fijo. El resultado ahora registra explicitamente la preservacion de ese anclaje y bloquea decisiones utiles/parciales si la invariante falla.

Evidencia:

- build `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`: salida `0`;
- `prueba_1`: `24` dry-runs `success=true/useful=true`;
- `prueba_2`: `14` dry-runs `success=true/partial_candidate=true`;
- todos los dry-runs muestran `[F1J-OPT-ANCHOR-PRESERVATION] ok=true required=true graph_satisfied=true previous_fiducial_fixed_count>=1 hard_fixed_moved=false max_previous_fiducial_delta_t=0.000000000`;
- en `prueba_2`, errores `23-26 m` bajan a `2.09-2.31 m`, quedan `partial_candidate=true` y no se aplican en `1J`.

## Validacion 1K

El 2026-07-10 se valido el apply historico:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`: `BUILD-EXIT-CODE 0`;
- prueba live larga `prueba_3`: `5` applies `applied=true`;
- prueba replay/debug `prueba_2`: `3` applies `applied=true`;
- todos los applies tienen `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`;
- no aparecen `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE` ni prechecks fallidos;
- los `partial_candidate=true` quedan como `[F1K-OPT-PARTIAL-PENDING] ... required_next=F1L_DECISION`.

## Revalidacion 1K tras reparar 1I/1J

El 2026-07-11 se revalido el apply con preservacion de anclajes:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`: `BUILD-EXIT-CODE 0`;
- `prueba_1`: `22` tareas/grafos/dry-runs, `7` applies `applied=true`, `15` parciales pendientes;
- `prueba_2`: `44` tareas/grafos/dry-runs, `10` applies `applied=true`, `15` parciales pendientes;
- todos los applies tienen `[F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK] ... ok=true`, `previous_fiducial_fixed_count>=1`, `fixed_kfs>=1` y `hard_fixed_moved=false`;
- `prueba_1`: `27` poses optimizadas, `157` propagadas, `184` correcciones por KF y `402` herencias de correccion en KFs nuevos;
- `prueba_2`: `40` poses optimizadas, `439` propagadas, `479` correcciones por KF y `399` herencias de correccion en KFs nuevos;
- todos los applies tienen `[F1K-RAWDB-NOT-MODIFIED] ... ok=true` y publican con `[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY]`;
- no aparecen `fixed_kfs=0`, `hard_fixed_moved=true`, `raw_db_modified=true`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED`, `APPLY-USEFUL-FALSE`, `FATAL`, `Segmentation fault` ni `Killed`.

## Validacion 1L

El 2026-07-10 se valido:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`: `BUILD-EXIT-CODE 0`;
- prueba `1`: `8` validaciones post-apply, `7` `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`;
- prueba `2`: `8` validaciones post-apply, `4` `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`, `3` `REJECT_ROLLBACK`;
- prueba `3`: `4` validaciones post-apply, `3` `ACCEPT`, `1` `PARTIAL_KEEP_FOR_NEXT_PASS`;
- los rechazos de `prueba_2` restauran con `[F1L-POSESTORE-ROLLBACK] ok=true`;
- los parciales conservados emiten `[F1L-RETRY-SUGGESTION] strategy=WIDER_WINDOW`;
- no aparecen `ROLLBACK_FAILED`, `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `HARD-FIDUCIAL-MOVED_ACCEPTED`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `FATAL`, `Segmentation fault` ni `Killed`.

## Revalidacion 1L tras reparar 1I-1K

El 2026-07-11 se probo `1L` con preservacion explicita de anclajes. La infraestructura de backup/rollback funciono, pero el resultado quedo `PARCIAL` tras inspeccion RViz2 porque los errores grandes del dron 2 en fiducial 1 se revertian por `internal_edges_broken`:

- build final de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`: salida `0`;
- `PostApplyValidationResult` incluye `anchor_preservation_ok` y `PostApplyAnchorPreservationCheck`;
- `OptimizationManager::ValidatePostApply` rechaza si el anclaje previo se mueve, si falta un anclaje de rama o si una subdivisión no confirmada hace inseguro conservar un parcial;
- `prueba_1`: `13` validaciones, `2` `ACCEPT`, `11` `REJECT_ROLLBACK`;
- `prueba_2`: `20` validaciones, `3` `ACCEPT`, `17` `REJECT_ROLLBACK`;
- `prueba_3`: `19` validaciones, `3` `ACCEPT`, `16` `REJECT_ROLLBACK`;
- todas las validaciones tienen `[F1L-ANCHOR-PRESERVATION-CHECK] ok=true required=true previous_fiducial_fixed_count=1 checked_branch_anchors=1 max_previous_fiducial_delta_t=0.000000000`;
- todos los rechazos restauran con `[F1L-POSESTORE-ROLLBACK] ok=true`;
- hubo `partial_candidate=true` aplicados bajo backup por `1L`, pero ninguno quedo como `PARTIAL_KEEP_FOR_NEXT_PASS`; se rechazaron si rompian aristas internas;
- no aparecen `F1L-ANCHOR-PRESERVATION-FAIL`, `ROLLBACK_FAILED`, `POST_APPLY_ACCEPT_WITH_ERROR_WORSE`, `RAWDB-POSE-OVERWRITE-BY-OPT`, `HARD-FIDUCIAL-MOVED_ACCEPTED`, `FATAL`, `Segmentation fault` ni `Killed`;
- ejemplos del fallo pendiente:
  - `prueba_2 task_id=4`: `20.122146 m -> 1.878937 m`, luego `REJECT_ROLLBACK reason=internal_edges_broken`;
  - `prueba_1 task_id=13`: `29.931767 m -> 3.321213 m`, luego `REJECT_ROLLBACK reason=internal_edges_broken`;
  - `prueba_3 task_id=4`: `26.624750 m -> 3.123809 m`, luego `REJECT_ROLLBACK reason=internal_edges_broken`.

Conclusion vigente: `1L` no esta cerrada; hay que corregir la causa de `internal_edges_broken` para que una mejora parcial segura pueda conservarse o reoptimizarse.

## Politica pendiente para cerrar 1L

La siguiente iteracion de `OptimizationManager` debe implementar una politica de correccion progresiva:

- separar `fiducial_absurd_error_t` de `fiducial_accept_error_t`;
- clasificar aristas internas como `strong_internal_edge` o `deformable_internal_edge`;
- permitir `PARTIAL_KEEP_FOR_NEXT_PASS` si un error absurdo baja mucho y solo hay `deformable_edges_broken`;
- guardar checkpoint parcial seguro;
- devolver `RetrySuggestion strategy=REBUILD_GRAPH_FROM_PARTIAL_STATE`;
- no volver a `20-30 m` si una segunda pasada falla, salvo que el checkpoint parcial viole anclajes, hard fiducials, raw o aristas fuertes.

Marcadores esperados:

```text
[F1L-PARTIAL-ABSURD-ERROR-POLICY]
[F1L-INTERNAL-EDGE-CLASSIFY]
[F1L-PARTIAL-CHECKPOINT]
[F1L-PARTIAL-PENDING]
[F1I-GRAPH-REBUILD-FROM-PARTIAL]
```

## Resultado 1L de la prueba aislada

`RunDryRun` ya usa ambos controles de `PathSegment`. La propuesta vigente interpola por separado el desplazamiento y la orientacion para evitar rotar posiciones alrededor del origen `world`. `ApplyCandidateResult` puede publicar propuestas `SERVER_OPTIMIZATION_PARTIAL_PROPAGATED` sin guardar correccion heredable, y el servidor reconstruye el mismo task desde el checkpoint.

La ruta funcional se observo como `28.176878 -> 1.408844 -> 0.070442 m`. Sin embargo, la primera pasada produjo `internal_max_after=49-60 m` y movio `90-110` KFs. Por tanto este solver heuristico no esta validado visualmente ni geometricamente: debe sustituirse por una optimizacion que minimice realmente `PoseGraphProblem::edges` y `priors`, y no debe publicarse un checkpoint con geometria interna extrema.

## Especificacion del solver requerido

Cuando se active un error fiducial grande, `OptimizationManager` debe trabajar
con una ventana entre fiduciales del mismo submapa. Los vertices son KFs
representativos repartidos por la ventana: fiduciales, muestras por longitud de
trayectoria y esquinas/cambios de yaw. Las aristas son poses relativas entre
vertices. Los priors fiduciales y de vecindad deben tener pesos graduales:
cerca de fiduciales confirmados tiran fuerte, en el centro de la ventana tiran
menos y dejan corregir deriva.

El comportamiento esperado ya no es aplicar `ApplyScaledEndpointDelta` como
solucion principal. Esa funcion puede servir como inicializacion o fallback de
diagnostico, pero la solucion aceptable debe venir de minimizar el coste del
grafo en varias iteraciones y producir logs de convergencia, coste before/after
e invariantes internas.
