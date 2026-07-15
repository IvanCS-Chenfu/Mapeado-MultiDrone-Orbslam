# Subfase 12R-D4 — Desbloquear apply de optimización local

## Estado

`actual`

## Resultado de ejecucion 2026-06-30

Conclusion: `NO CONSEGUIDA`.

Se implemento y compilo el cambio de guard de apply, el balance query/candidate
y la reduccion de logs del launch. Se ejecutaron las dos pruebas Gazebo
requeridas y ambas reportaron `SIM-DONE success=true` y `SIM-EXIT-CODE 0`.

La subfase no se marca como realizada porque los logs reducidos no demostraron
la evidencia obligatoria:

```text
LOCAL_LOOP_OPT_TASK
LOCAL-POSE-GRAPH-VAR-SELECT
LOCAL-POSE-GRAPH-SOLVER
LOCAL-OPT-APPLY-PRECHECK accepted
LOCAL-OPT-APPLY-KF-ALLOW
LOCAL-OPT-APPLY-SUMMARY applied=true moved>0
```

La prueba 2 mostro actividad parcial al cierre:

```text
local_opt_task_ready=1
local_opt_windows=1
dryrun_results=1
```

pero sin marcadores de solver/apply. La siguiente ejecucion debe diagnosticar
si la simulacion se corta antes de procesar esa tarea, si falta patron de
reduccion para capturar los marcadores, o si la decision de subnube no cruza
realmente a `LOCAL_LOOP_OPT_TASK`.

### Aclaracion posterior del usuario sobre RViz2 y Gazebo

El usuario observo RViz2 durante ambas pruebas y vio deriva importante y bastante
ruido, con paredes duplicadas. Por tanto, visualmente habia condiciones donde
deberian aparecer varios loops o, como minimo, evidencia de deteccion/fusion de
zonas repetidas.

Correcciones de interpretacion:

- el `exit code 255` de Gazebo al final de las pruebas no debe tratarse como
  error grave si aparece despues de `SIM-DONE success=true` y durante cleanup;
  es compatible con el cierre forzado/controlado de Gazebo tras la simulacion;
- si aparece optimizacion local, debe existir una deteccion previa de loop o una
  tarea derivada de loop; no tiene sentido concluir "hay optimizacion pero no hay
  loops" sin investigar marcadores perdidos o reduccion incompleta;
- la ausencia de `LOCAL_LOOP_OPT_TASK` en el log reducido no basta por si sola
  para afirmar que el codigo no detecto loops si aparecen contadores como
  `local_opt_task_ready=1`, `local_opt_windows=1` o `dryrun_results=1`;
- en ese caso hay que distinguir explicitamente entre fallo real del codigo y
  fallo de reduccion/logging.

Para la siguiente ejecucion, si los logs reducidos no muestran loops pero hay
contadores de optimizacion, Codex debe responder a esta pregunta antes de tocar
codigo:

```text
¿No hay loops porque el codigo no los genera, o porque el patron de reduccion
no esta conservando los marcadores correctos?
```

La primera comprobacion debe ser ampliar patrones de reduccion con marcadores
de decision y cola:

```text
LOOP-SUBCLOUD-DECISION|LOOP-ERROR-UNIFIED|LOCAL_LOOP_OPT_TASK|decision=LOCAL_LOOP_OPT_TASK|SUBCLOUD-LOCAL-OPT-TASK-CREATE|OPT-TASK|OPT-TASK-QUEUE-STATE|local_opt_task_ready|local_opt_windows|dryrun_results|LOCAL-OPT-WINDOW|LOCAL-OPT-DRYRUN|LOCAL-POSE-GRAPH|LOCAL-OPT-APPLY
```

Si aun asi no aparecen loops ni decisiones hacia `LOCAL_LOOP_OPT_TASK`, entonces
hay que revisar el codigo de transicion desde `LOOP-SUBCLOUD-DECISION` hacia
tarea local. Si aparecen en el log completo/reducido ampliado, el problema era
la reduccion de logs o los patrones usados.

## Resultado de repeticion 2026-06-30 12:17

Conclusion: `NO CONSEGUIDA`.

Se repitio `12R-D4` con el mismo escenario secuencial en `tray_prueba_1.yaml`
y `tray_prueba_2.yaml`:

1. `dron_1` al fiducial 2 con yaw `90 deg`;
2. `dron_2` encima de `dron_1` a `0.3 m`;
3. `dron_1` a `-6 m` en `x`;
4. `dron_2` encima de `dron_1`;
5. `dron_1` vuelve al fiducial 2;
6. `dron_2` encima de `dron_1`.

Build:

```text
BUILD-EXIT-CODE 0
orbslam3_msgs, orbslam3_multi, orbslam3_server compilados
```

Simulaciones:

```text
prueba_1: SIM-SCENARIO-EXIT-CODE 0, SIM-DONE success=true, SIM-EXIT-CODE 0
prueba_2: SIM-SCENARIO-EXIT-CODE 0, SIM-DONE success=true, SIM-EXIT-CODE 0
```

Observacion RViz2 del usuario:

- prueba 1: no habia deriva; deberia haber tenido solo fusiones y ningun loop de optimizacion;
- prueba 2: cuando `dron_1` volvio al fiducial 2 aparecio deriva en RViz2.

Analisis de logs reducidos con patrones ampliados:

- prueba 1 tuvo eventos `LOOP-SUBCLOUD-*`, pero no `LOCAL_LOOP_OPT_TASK`, ni solver, ni apply. Estos eventos se interpretan como revisitas/fusiones/diagnostico, no como loop de optimizacion local;
- prueba 2 tuvo muchos mas eventos `LOOP-SUBCLOUD-*` y errores `LOOP-ERROR-UNIFIED`, incluyendo error alto tras el retorno hacia fiducial, pero las decisiones de subnube quedaron en `FUSION_EVENT`, `HOLD_DIAGNOSTIC` o `ANCHOR_CANDIDATE`;
- en prueba 2 no aparecio ningun `decision=LOCAL_LOOP_OPT_TASK` real, solo la linea de patrones;
- `SUBCLOUD-LOCAL-OPT-TASK-SUMMARY` termino con `local_loop_events=0 created=0`;
- `OPT-TASK-QUEUE-STATE` termino con `local_loop=0`, `tasks=0`, `windows=0`, `dryrun_results=0`, `apply_results=0`;
- no aparecieron `LOCAL-POSE-GRAPH-VAR-SELECT`, `LOCAL-POSE-GRAPH-SOLVER`, `LOCAL-OPT-APPLY-PRECHECK`, `LOCAL-OPT-APPLY-KF-ALLOW` ni `LOCAL-OPT-APPLY-SUMMARY`.

Diagnostico:

La ausencia de loops en la ejecucion anterior era en parte un problema de
reduccion/patrones: con los patrones ampliados si se ven `LOOP-SUBCLOUD-*`.
Pero el fallo actual ya no es la reduccion. El codigo detecta revisitas/fusiones
y calcula errores, pero no convierte la deriva observada en RViz2 en
`LOCAL_LOOP_OPT_TASK`. Por tanto, D4 no llega a ejercitar el guard de apply.

Siguiente paso minimo:

Antes de seguir con el apply de D4 o pasar a D5, revisar la decision de subnube
para casos donde RViz2 muestra deriva pero `LOOP-SUBCLOUD-DECISION` clasifica
como `FUSION_EVENT`/`HOLD_DIAGNOSTIC`. En particular, comparar
`LOOP-ERROR-UNIFIED` alto con `LOOP-SUBCLOUD-ERROR` bajo y decidir si falta una
regla de promocion a `LOCAL_LOOP_OPT_TASK` basada en evidencia geometrica real,
sin reactivar `LOOP-VIEWMAP-ERROR` como criterio principal.

## Cierre documental 2026-06-30 — diferencia unified/subcloud

Conclusion: `NO CONSEGUIDA`.

Analisis fino de los logs completos:

- `LOOP-SUBCLOUD-ERROR` y `LOOP-SUBCLOUD-DECISION` forman la ruta moderna que
  decide acciones del pipeline: `FUSION_EVENT`, `LOCAL_LOOP_OPT_TASK`,
  `HOLD_DIAGNOSTIC` o `ANCHOR_CANDIDATE`;
- `LOOP-ERROR-UNIFIED` mide una discrepancia entre la transformacion medida del
  loop y las poses globales/anchored de KFs. Es diagnostico de la ruta
  unified/viewmap y no demuestra que se haya creado una tarea local;
- prueba 1: RViz2 sin deriva; `LOOP-SUBCLOUD-ERROR` bajo
  (`max error_t=0.2001`), decisiones de fusion y sin optimizacion;
- prueba 2: RViz2 con deriva al retorno a fiducial; `LOOP-SUBCLOUD-ERROR` bajo
  (`max error_t=0.2092`) y fusiones, pero aparece un `LOOP-ERROR-UNIFIED` alto
  (`error_t=3.1834`, `error_rot_deg=25.892`, `usable_opt=true`);
- pese a ese unified alto, no hay `decision=LOCAL_LOOP_OPT_TASK`,
  `SUBCLOUD-LOCAL-OPT-TASK-CREATE`, `LOCAL-POSE-GRAPH-SOLVER` ni
  `LOCAL-OPT-APPLY-SUMMARY`;
- por tanto no hubo optimizacion local demostrada.

Implicacion para el siguiente chat:

No basta con encontrar `LOOP-ERROR-UNIFIED` alto. Hay que arreglar la transicion
para que una discrepancia fuerte y geometria real suficiente desemboquen en una
decision moderna `LOCAL_LOOP_OPT_TASK`, sin volver a usar `LOOP-VIEWMAP-ERROR`
como criterio principal legacy.

## Resultado de continuacion 2026-06-30 13:02 — puente unified auxiliar

Conclusion: `PARCIAL`.

Se corrigio el bloqueo concreto observado en `prueba_2`: un
`LOOP-ERROR-UNIFIED` alto con `usable_opt=true` ya puede generar un evento
moderno de subnube cuando la evidencia geometrica es fuerte.

Cambio aplicado:

- `CreateSubcloudLocalLoopEventFromUnifiedLoop` reconstruye pares 3D desde el
  evento unified, valida inliers/ratio/residuales y crea un
  `LoopSubcloudEvent` con `decision=LOCAL_LOOP_OPT_TASK`;
- `RegisterUnifiedLoopRuntimeEvent` registra aceptaciones con
  `SUBCLOUD-UNIFIED-AUX-EVENT` y rechazos con `SUBCLOUD-UNIFIED-AUX-SKIP`;
- los parametros `subcloud_decision_unified_aux_*` permiten ajustar umbrales
  sin convertir `LOOP-VIEWMAP-ERROR` en criterio principal;
- la tarea sigue entrando por la ruta moderna
  `SUBCLOUD-LOCAL-OPT-TASK-CREATE`.

Build:

```text
BUILD-EXIT-CODE 0
orbslam3_msgs, orbslam3_multi, orbslam3_server compilados
```

Simulaciones:

```text
prueba_1: SIM-SCENARIO-EXIT-CODE 0, SIM-DONE success=true, SIM-EXIT-CODE 0
prueba_2: SIM-SCENARIO-EXIT-CODE 0, SIM-DONE success=true, SIM-EXIT-CODE 0
```

Evidencia positiva:

- `prueba_1`: `10` eventos `SUBCLOUD-UNIFIED-AUX-EVENT`, `10` tareas
  `SUBCLOUD-LOCAL-OPT-TASK-CREATE`, `2` logs `LOCAL-POSE-GRAPH-SOLVER` y un
  `LOCAL-OPT-APPLY-SUMMARY applied=true moved=11`;
- `prueba_1`: aparece `LOCAL-OPT-APPLY-KF-ALLOW`, confirmando que un KF normal
  de submapa anclado por fiducial puede moverse si no es hard fiducial/hard-fixed;
- `prueba_2`: `11` eventos `SUBCLOUD-UNIFIED-AUX-EVENT` y `11` tareas
  `SUBCLOUD-LOCAL-OPT-TASK-CREATE`; por tanto la ausencia de
  `LOCAL_LOOP_OPT_TASK` ya no es el bloqueo principal;
- `SUBCLOUD-UNIFIED-AUX-SKIP` aparece en candidatos con unified alto pero
  geometria insuficiente, por ejemplo cuando `max_geom` supera `0.5 m`.

Evidencia negativa o ausente:

- `prueba_2` no emite `LOCAL-POSE-GRAPH-SOLVER`;
- `prueba_2` no emite `LOCAL-OPT-APPLY-SUMMARY applied=true`;
- el estado final de `prueba_2` queda en `tasks=11`, `pending=11`,
  `tasks_with_window=4`, `tasks_window_failed=7`,
  `tasks_dryrun_failed=4`, `tasks_dryrun_useful=0` y `apply_results=0`.

Implicacion:

La transicion unified/subcloud queda corregida, pero D4 no puede marcarse como
`CONSEGUIDA` porque la prueba con deriva no demuestra apply util. No pasar a
`12R-D5`.

Siguiente paso minimo:

Diagnosticar dentro de `12R-D4` por que las tareas modernas de `prueba_2`
fallan en construccion de ventana o dry-run:

- `BuildLocalOptimizationWindowFromTask` para `tasks_window_failed=7`;
- `RunLocalOptimizationDryRunForWindow` y los estados que producen
  `tasks_dryrun_failed=4` sin `LOCAL-POSE-GRAPH-SOLVER`;
- mantener la regla de no aplicar si el dry-run no mejora.

## Objetivo

Permitir que una optimización local útil calculada por el solver mueva realmente KeyFrames normales, aunque pertenezcan a un submapa anclado por fiducial.

Debe mantenerse la protección sobre:
- KeyFrames realmente fiduciales;
- KeyFrames hard-fixed;
- KeyFrames protegidos por constraint graph fuerte.

## Diagnóstico de partida

El solver local puede calcular una mejora útil, pero el apply puede terminar con:

```text
applied=false
moved=0
reason=fiducial_direct_without_constraint_support
```

El problema conceptual es que `pose_source` o `pose_reason` puede contener:

```text
FIDUCIAL_DIRECT:CONFIRMED
```

porque el submapa completo está anclado mediante fiducial. Eso **no** significa que cada KeyFrame esté físicamente dentro del fiducial ni que toda la trayectoria deba quedar congelada.

## Método recomendado primero

Modificar el guard en:

```text
orbslam3_server/src/global_map_server.cpp
ApplyLocalOptimizationDryRunResultToServer
```

La lógica deseada:

```text
si pose_source contiene FIDUCIAL_DIRECT
  y el KF NO tiene hard_fiducial_constraint
  y el KF NO tiene hard_fixed_by_constraint_graph
  y el role NO indica FIDUCIAL/FID
entonces permitir aplicar la optimización local
```

Añadir log acotado:

```text
LOCAL-OPT-APPLY-KF-ALLOW
```

Mantener skip para KeyFrames realmente fiduciales/hard.

## Balance query/candidate

Si el código actual lo permite sin gran refactor, añadir o ajustar balance de variables:

```text
max_query_variables = 8
max_candidate_variables = 6
min_query_variables = 2
min_candidate_variables = 2
```

El objetivo no es exactitud matemática, sino evitar que la selección de variables represente solo el lado candidato y deje mal representado el tramo query con deriva.

## Reducción de logs

Apagar o reducir logs masivos si existen los parámetros actuales:

```text
local_opt_constraint_graph_log_edges = false
subcloud_kf_covisibility_log_edges = false
subcloud_local_opt_window_max_kf_logs = 40
subcloud_local_opt_dryrun_max_kf_logs = 40
subcloud_local_opt_apply_max_kf_logs = 40
local_pose_graph_max_variable_candidate_logs = 12
fiducial_prior_stored_log_throttle_ms = 3000
```

## Preservar result_id

Si en `RunLocalOptimizationDryRunForWindow` aparece `LOCAL-POSE-GRAPH-SOLVER result=0` aunque luego el resultado guardado tenga otro id, preservar el `incoming_result_id` antes de resetear `result_out`.

Esto no es crítico funcionalmente, pero mejora diagnóstico.

## Cambios permitidos

- `orbslam3_server/src/global_map_server.cpp`
- headers de `orbslam3_server` si hacen falta
- launch/config del servidor para reducir logs
- YAMLs de prueba en `codex/archivos_auxiliares/trayectorias/tray_prueba_x.yaml`
- documentación de contexto tras compilar

## Cambios prohibidos

- No modificar ORB-SLAM3 interno.
- No rediseñar wrapper.
- No rediseñar mensajes.
- No implementar D5.
- No implementar optimización global/fiducial.
- No limpiar legacy masivamente.
- No usar ground truth para corregir mapa o pose.
- No subir `max_variables` agresivamente como solución.

## Build esperado

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

Si falla:

```bash
./codex/herramientas/reduce_build_log.sh
```

`diagnosticador_build` debe leer:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

Si falta contexto, debe consultar `codex/archivos_auxiliares/logs/colcon_build.log`
completo antes de pedir cambios de código.

## Pruebas requeridas

### Protocolo de pruebas D4 tras aclaracion RViz2

No asumir que una prueba "limpia" y una prueba "con deriva" se consiguen solo
cambiando parametros del YAML. Gazebo + ORB-SLAM3 pueden producir resultados
distintos con el mismo YAML por tracking, mapa local, timing y ruido.

Para D4 se debe usar un mismo escenario base secuencial y repetirlo N veces
hasta obtener:

- una ejecucion visualmente limpia o casi limpia: en RViz2 no se observa deriva
  importante ni paredes duplicadas; en logs deberian dominar fusiones o no haber
  loops destructivos;
- una ejecucion con deriva: en RViz2 se observa deriva/ruido/paredes duplicadas;
  en logs deben aparecer loops, decisiones de subnube o tareas de optimizacion.

Tras cada ejecucion, Codex debe dar una conclusion preliminar basada en logs.
Despues el usuario aportara su observacion de RViz2. La conclusion final de esa
prueba debe combinar ambas fuentes:

```text
conclusion_preliminar_logs + observacion_usuario_RViz2 -> conclusion_final
```

No cerrar `CONSEGUIDA`, `NO CONSEGUIDA` o `PARCIAL` de una prueba visualmente
ambigua sin incorporar la observacion del usuario cuando este la ofrezca.

### Escenario base recomendado

Todos los movimientos deben ser secuenciales: mover un dron solo despues de que
el otro haya terminado su movimiento.

Secuencia recomendada:

1. mover `dron_1` al fiducial 2 con yaw `90 deg`;
2. cuando termina, mover `dron_2` encima de `dron_1`, a `0.3 m` por encima;
3. cuando termina, mover `dron_1` `6 m` en `-x`;
4. cuando termina, mover `dron_2` encima de `dron_1`;
5. cuando termina, mover `dron_1` de nuevo al fiducial;
6. cuando termina, mover `dron_2` encima de `dron_1`.

Este mismo YAML debe poder repetirse N veces. La clasificacion de cada ejecucion
no viene del YAML, sino de logs + observacion RViz2.

### Ejecucion limpia o casi limpia

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia conceptual:
usar el escenario base secuencial y clasificar esta ejecucion como limpia solo
si logs y RViz2 lo apoyan.

Esperado:
- RViz2 estable;
- no apply destructivo;
- si hay `FUSION_EVENT`, correcto;
- si aparece `LOCAL_LOOP_OPT_TASK` pero el dry-run no mejora, no debe aplicar.

Patrones de log para reducir:

```text
SCENARIO-RUNNER|LOOP-SUBCLOUD|LOOP-SUBCLOUD-DECISION|LOOP-ERROR-UNIFIED|LOCAL_LOOP_OPT_TASK|decision=LOCAL_LOOP_OPT_TASK|SUBCLOUD-LOCAL-OPT-TASK-CREATE|OPT-TASK|OPT-TASK-QUEUE-STATE|local_opt_task_ready|local_opt_windows|dryrun_results|LOCAL-POSE-GRAPH|LOCAL-OPT|FUSION_EVENT|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc
```

### Ejecucion con deriva

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Secuencia conceptual:
usar el mismo escenario base secuencial. Repetir N veces si hace falta hasta
obtener una ejecucion donde RViz2 muestre deriva/ruido/paredes duplicadas y los
logs permitan estudiar loops u optimizacion.

Esperado en logs:
- loops visibles por `LOOP-SUBCLOUD`, `LOOP-SUBCLOUD-DECISION`,
  `LOOP-ERROR-UNIFIED` o `decision=LOCAL_LOOP_OPT_TASK`
- `LOCAL_LOOP_OPT_TASK` o contadores que expliquen su ausencia/presencia
- `LOCAL-POSE-GRAPH-VAR-SELECT`
- `LOCAL-POSE-GRAPH-SOLVER`
- `LOCAL-OPT-APPLY-PRECHECK accepted`
- `LOCAL-OPT-APPLY-KF-ALLOW`
- `LOCAL-OPT-APPLY-SUMMARY applied=true moved>0`

Esperado visual:
- movimiento coherente de zona corregida;
- no mover hard fiducials;
- no bloqueo del ordenador.

Patrones de log para reducir:

```text
SCENARIO-RUNNER|LOOP-SUBCLOUD|LOOP-SUBCLOUD-DECISION|LOOP-ERROR-UNIFIED|LOCAL_LOOP_OPT_TASK|decision=LOCAL_LOOP_OPT_TASK|SUBCLOUD-LOCAL-OPT-TASK-CREATE|OPT-TASK|OPT-TASK-QUEUE-STATE|local_opt_task_ready|local_opt_windows|dryrun_results|LOCAL-POSE-GRAPH|LOCAL-OPT-APPLY|LOCAL-OPT-DRYRUN|LOCAL-OPT-WINDOW|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc
```

## Criterio de éxito

La subfase se considera superada si:

1. compila;
2. en prueba limpia no hay apply destructivo;
3. en prueba con deriva aparece apply útil con `moved>0`;
4. aparecen logs `LOCAL-OPT-APPLY-KF-ALLOW` para KFs normales de submapa anclado;
5. los hard fiducials no se mueven;
6. no hay bloqueo por flood de logs;
7. el historial se actualiza con resultados.

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- el build no compila tras corregir el primer error real abordable;
- no se puede ejecutar alguna prueba requerida;
- la prueba con deriva observada en RViz2 no genera loops, decisiones de loop ni
  una explicacion clara en logs ampliados;
- aparece dry-run util pero no aparece apply aceptado;
- aparece apply aceptado pero `moved=0` por bloqueo incorrecto de KFs normales;
- se mueven KFs hard fiducial o hard-fixed;
- aparecen errores graves como `FATAL`, `Segmentation fault`, `Killed` o `std::bad_alloc` no explicados.

No contar como fallo de simulacion un `exit code 255` de Gazebo si aparece
despues de `SIM-DONE success=true` durante el cleanup controlado.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- la prueba limpia pasa;
- aparecen logs de seleccion/solver;
- pero la prueba con deriva no llega a demostrar `LOCAL-OPT-APPLY-SUMMARY applied=true moved>0`.

En caso de `NO CONSEGUIDA` o `PARCIAL`, no marcar esta subfase como `realizado`. El historial debe explicar exactamente que marcador faltó o qué condicion falló.

## Documentacion a actualizar

Al terminar una ejecucion real de 12R-D4, `curador_documentacion` debe actualizar:

- historial de Fase 1 o historial especifico de 12R-D4;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- documentacion de `orbslam3_server` si se modifica `global_map_server.cpp`;
- este archivo si el resultado obliga a cambiar el plan;
- la siguiente subfase solo si D4 queda `CONSEGUIDA` o si el fallo cambia explicitamente el orden de trabajo.
