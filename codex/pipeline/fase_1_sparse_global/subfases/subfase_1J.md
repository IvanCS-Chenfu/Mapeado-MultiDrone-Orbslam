# Subfase 1J — Dry-run robusto con transformaciones relativas completas y refinamiento adaptativo

## Estado

```text
pendiente
```

Estado anterior: `realizado`.

Propiedad funcional vigente:

```text
1J = dry-run + diagnóstico visual del grafo
```

La generación/visualización HTML del grafo pertenece conceptualmente a `1J`,
incluyendo dump/replay offline si se usa para inspeccionar el resultado del
dry-run. El HTML no debe depender de aplicar poses en `GlobalPoseStore`.

Se reabre porque el solver actual conserva principalmente longitudes de segmentos, utiliza una penalización débil de yaw y evalúa después un coste que no coincide exactamente con el residual optimizado. Esa formulación permite que el último KeyFrame llegue al fiducial deformando o estirando el resto de la ventana.

La nueva implementación debe corregir derivas pequeñas, graduales, mixtas y catastróficas sin asumir de antemano que existe una única bisagra ni que toda corrección debe repartirse suavemente.

Observación nueva desde HTML/RViz2: en el caso `dron_2` antihorario, la solución
actual mueve sobre todo los controles finales cerca del fiducial 1. El dry-run
de `1J` debe evitar ese mínimo local: si el target se corrige concentrando casi
toda la traslación en el último tramo, la salida correcta es
`needs_refinement=true` o `useful=false`, no candidato aceptable para `1K/1L`.

---

## Objetivo técnico

Ejecutar una optimización dry-run sobre el grafo de control creado por `1I` que:

1. lleve el target hacia el fiducial absoluto;
2. preserve fuertemente las traslaciones relativas locales exportadas por ORB-SLAM3;
3. permita más libertad en orientación que en traslación;
4. use covisibilidad para mantener coherencia local y entre tramos;
5. reparta una deriva gradual cuando sea la explicación más barata;
6. permita concentrar una corrección angular cuando una o pocas restricciones sean incompatibles;
7. no diseñe una bisagra fija ni debilite aristas solo por tener un ángulo grande;
8. use exactamente la misma función residual para solver, coste, logs y validación;
9. devuelva una propuesta completa en memoria sin modificar estado persistente;
10. solicite refinamiento de `1I` cuando el grafo de control sea insuficiente.

---

## Decisiones matemáticas obligatorias

### 1. Medida relativa completa

Para cada arista `i -> j`, la medida es:

```text
Z_ij = raw_local_T_i.inverse() * raw_local_T_j
```

El residual canónico es:

```text
r_ij = Log( Z_ij.inverse() * (T_i.inverse() * T_j) )
```

La implementación puede usar una versión SE(2.5) inicial, pero la estructura de datos no debe impedir SE(3) futuro.

### 2. Variables de la primera implementación

Para la simulación actual:

```text
variables activas: x, y, z, yaw
roll y pitch: fijos o con prior muy fuerte
```

Motivo:

- el dron puede cambiar altura;
- la deriva observada es principalmente de posición/yaw;
- abrir seis grados de libertad sin evidencia suficiente añade inestabilidad.

Debe quedar documentado cómo habilitar roll/pitch en una fase posterior si los datos lo justifican.

### 3. Pesos anisotrópicos

No usar un único `edge.weight` para todas las componentes.

Matriz conceptual:

```text
Omega_ij = diag(w_x, w_y, w_z, w_roll, w_pitch, w_yaw)
```

Regla inicial:

```text
w_translation >> w_yaw
w_z y w_roll/pitch altos en la simulación actual
```

Esto no significa que yaw sea libre. Significa que, ante una incompatibilidad global, debe resultar más barato corregir orientación que estirar la trayectoria.

Regla operativa para el caso fiducial:

```text
penalizar mucho la variación de traslación relativa local;
permitir bastante más variación de yaw relativa;
rechazar/refinar si la corrección traslacional queda concentrada al final de la ventana.
```

La trayectoria debe poder “girar” para encajar el fiducial nuevo, pero no debe
alargar o romper segmentos locales entre KFs consecutivos.

### 4. Traslación medida en el frame local

El término de traslación debe ser equivalente a:

```text
R_i.transpose() * (p_j - p_i) - t_ij_raw
```

No comparar solo posiciones globales ni conservar solo la longitud.

Esta formulación permite que un bloque posterior rote en world manteniendo su geometría local.

### 5. Misma función para solver y coste

Debe existir una única función o conjunto de funciones compartidas para:

- construir residual;
- construir Jacobiano;
- calcular `initial_cost`;
- calcular `final_cost`;
- producir métricas por arista;
- validar después en `1L`.

No se acepta:

```text
solver optimiza longitud
ComputeCost evalúa vector/rotación completos
```

### 6. Robustez sin bisagra predeterminada

Aplicar kernel robusto, inicialmente Huber, a aristas temporales y de covisibilidad.

Comportamiento deseado:

- error pequeño: coste cuadrático, corrección distribuida y suave;
- error grande: influencia acotada, una zona incompatible puede absorber más corrección;
- ninguna arista se apaga automáticamente;
- un giro real de 90 grados no se debilita solo por su ángulo.

No introducir `switchable constraints` en la primera implementación. Solo añadirlos si los tests demuestran que Huber y covisibilidad no bastan.

### 7. Regularización del campo de correcciones

Definir la corrección de cada control:

```text
C_i = T_i_optimized * T_i_initial.inverse()
```

Añadir un término robusto entre correcciones consecutivas:

```text
E_correction = sum rho_c( Log(C_i.inverse() * C_j) )
```

Este término favorece continuidad, pero el kernel robusto permite una variación localizada si es necesaria.

Añadir opcionalmente una regularización de segundo orden moderada sobre la secuencia de controles. Debe poder desactivarse y nunca dominar las medidas relativas.

### 8. Priors fiduciales

- Fiducial previo: hard fixed para eliminar gauge freedom y preservar el anchor aceptado.
- Fiducial objetivo: prior absoluto muy fuerte, no sobrescritura manual previa.
- Hard fiducials intermedios: fijos.
- Vecindades: se preservan mediante aristas y covisibilidad; no fijar rígidamente decenas de KFs salvo evidencia explícita.

### 9. Prior parabólico

No usarlo como componente principal.

Puede existir un prior `stay-close` débil y configurable con forma en U respecto a los dos anchors, desactivado por defecto durante la primera validación del nuevo solver.

La solución debe venir principalmente de:

- transformaciones relativas completas;
- covisibilidad;
- priors fiduciales;
- regularización robusta.

---

## Contexto obligatorio a leer

- `AGENTS.md`
- `01_ESTADO_ACTUAL.md`
- `PIPELINE_MAESTRO.md`
- `pipeline_fase_1.md`
- versiones revisadas de `1H`, `1I`, `1K`, `1L`
- documentación de:
  - `pose_graph_problem`
  - `pose_graph_builder`
  - `optimization_manager`
  - `global_pose_store`
  - `optimization_debug_exporter`
- código actual de `optimization_manager.cpp`
- historial reciente y HTMLs debug existentes

---

## Diagnóstico de partida

Problemas confirmados en el código actual:

1. El solver conserva principalmente distancia del segmento.
2. La dirección relativa puede cambiar sin coste suficiente.
3. El coste posterior no coincide con el residual del solver.
4. `solver_step_fraction` está declarado, pero no controla necesariamente el alpha inicial.
5. `success=true` puede devolverse sin convergencia real.
6. No se comprueba de forma explícita el estado de la factorización.
7. La solución puede mejorar el target y empeorar gran parte de la ventana.
8. Los resultados parciales se han aplicado antes de tener geometría válida.
9. No existe refinamiento formal del grafo a partir de segmentos mal representados.
10. La covisibilidad no entra todavía en el coste.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

- `include/orbslam3_multi/optimization_manager.hpp`
- `src/optimization_manager.cpp`
- `include/orbslam3_multi/optimization_result.hpp`
- `include/orbslam3_multi/pose_graph_problem.hpp`, solo para campos acordados con `1I`
- `include/orbslam3_multi/optimization_debug_exporter.hpp`
- `src/optimization_debug_exporter.cpp`
- nuevos ficheros matemáticos pequeños si evitan duplicación
- `test/`
- `CMakeLists.txt`

### `orbslam3_server`

- `src/global_map_server.cpp`
- `launch/global_orb_map_server.launch.py`

### Documentación y auxiliares

- documentación de paquetes
- HTML debug
- YAMLs de prueba
- historial de Fase 1

---

## Archivos prohibidos

No modificar:

- raw de ORB-SLAM3;
- `orbslam3_msgs`;
- wrapper;
- `GlobalPoseStore` persistente desde el dry-run;
- código legacy;
- GT para coste, selección, pesos o decisión runtime.

---

## Resultado y estados del solver

Sustituir el booleano ambiguo por un estado explícito o equivalente:

```text
CONVERGED
CONVERGED_SMALL_STEP
MAX_ITERATIONS
TIME_BUDGET_EXCEEDED
NO_PROGRESS
NUMERICAL_FAILURE
INVALID_PROBLEM
CANCELLED_SUPERSEDED
```

`success=true` solo si existe una propuesta finita y matemáticamente válida. `useful=true` exige además cumplir los criterios geométricos preliminares.

Campos nuevos mínimos en `OptimizationDryRunResult`:

```text
solver_status
converged
linear_solver_ok
line_search_accepted_steps
line_search_rejected_steps
elapsed_ms
needs_refinement
refinement_reason
refinement_segments
edge_metrics
holdout_not_evaluated_yet=true
```

---

## Función objetivo

La función conceptual mínima:

```text
E = E_fiducial
  + E_temporal
  + E_covisibility
  + E_correction_regularization
  + E_optional_stay_close
```

### `E_fiducial`

Priors del grafo proporcionados por `1I`.

### `E_temporal`

Residual relativo completo con pesos anisotrópicos y Huber.

### `E_covisibility`

Mismo residual geométrico, con confianza derivada del soporte de MapPoints/covisibilidad. Estas aristas deben ayudar a mantener paredes y tramos observados por varios KeyFrames.

### `E_correction_regularization`

Evita correcciones arbitrarias entre controles consecutivos, pero permite cambios localizados mediante robustez.

### `E_optional_stay_close`

Prior débil respecto a la pose inicial. Si se activa, su peso debe quedar en logs y no sustituir a la geometría.

---

## Construcción de pesos

### Temporal

Partir de parámetros base separados:

```text
temporal_translation_weight_xy
temporal_translation_weight_z
temporal_rotation_weight_yaw
temporal_rotation_weight_roll_pitch
```

Modificar por:

- `path_kf_count`;
- `path_length_m`;
- confianza suministrada por `1I`;
- soporte mínimo de covisibilidad del segmento.

Una arista agregada larga debe ser menos informativa que una arista corta.

### Covisibilidad

Usar:

```text
shared_mappoint_count
normalized_shared_support
orb_covisibility_weight
```

Aplicar saturación para que cientos de puntos no generen pesos infinitos.

### Ángulo

No multiplicar el peso por una función decreciente del ángulo como regla principal. El ángulo sirve para seleccionar controles y diagnosticar, no para asumir incertidumbre.

---

## Algoritmo de dry-run

### Paso 1 — Precheck

Rechazar si:

- cobertura incompleta;
- grafo desconectado;
- priors fiduciales ausentes;
- pose no finita;
- arista sin medida válida;
- tarea stale/superseded;
- hard fiducial marcado variable;
- no hay suficientes controles.

### Paso 2 — Copia inmutable

Copiar las poses iniciales y construir el vector de variables sin escribir en `GlobalPoseStore`.

### Paso 3 — Evaluar coste inicial

Usar exactamente los mismos residuals y kernels del solver.

Guardar métricas por componente:

```text
temporal_translation_cost
temporal_rotation_cost
covisibility_cost
fiducial_cost
correction_regularization_cost
```

### Paso 4 — Resolver

Requisitos mínimos:

- comprobar factorización;
- aplicar damping/Levenberg o estrategia equivalente;
- usar `solver_step_fraction` realmente o eliminar el parámetro;
- line search con estados claros;
- límite de iteraciones;
- límite de tiempo;
- cancelación si la tarea fue superseded;
- no bloquear callbacks ROS: el servidor debe ejecutar en worker.

### Paso 5 — Evaluar propuesta

Calcular:

- error fiducial antes/después;
- variación de traslación local por arista;
- variación de yaw por arista;
- coste por familia;
- número de aristas en zona robusta;
- corrección por control;
- gradiente de corrección por segmento;
- máximo delta world, solo como diagnóstico, no como rechazo automático.

Un KeyFrame lejano puede moverse mucho en world debido a una corrección angular válida.

### Paso 6 — Detectar necesidad de refinamiento

Solicitar refinamiento si:

- un segmento de control concentra demasiado residual;
- la corrección cambia mucho entre dos controles muy separados;
- una arista agregada entra profundamente en el kernel robusto;
- el target mejora, pero la geometría preliminar no cumple tolerancias;
- el budget inicial fue insuficiente.

Salida:

```text
needs_refinement=true
refinement_segments=[(control_a, control_b, reason, suggested_kf)]
```

El servidor devuelve la solicitud a `1I`, reconstruye con más controles y repite, hasta un máximo configurable.

### Paso 7 — Clasificar dry-run

Estados preliminares:

```text
CANDIDATE_FOR_VALIDATION
NEEDS_GRAPH_REFINEMENT
REJECT_DRYRUN
```

No usar `PARTIAL_KEEP_FOR_NEXT_PASS` como autorización para aplicar un estado geométricamente dudoso.

`1J` no decide commit. `1L` valida la propuesta completa.

---

## Propagación propuesta

`1J` puede calcular la corrección de los controles, pero la materialización de todos los KFs pertenece a `1K`.

Puede incluir una predicción de propagación para diagnosticar, usando el `PropagationPlan` de `1I`, pero:

- no escribir poses;
- no establecer `last_correction`;
- no usar nearest-control salvo tramo degenerado;
- no convertir la predicción en criterio suficiente de aceptación.

---

## Exportación HTML requerida

Esta sección es propiedad de `1J`. Si el código conserva nombres legacy
`f1l_debug_animation_*` o `f1l_offline_graph_*`, tratarlos como deuda de
renombrado/compatibilidad: funcionalmente siguen siendo herramientas de
dry-run/diagnóstico de `1J`, no de apply.

El HTML debe mostrar cuatro capas:

1. poses iniciales y GT debug opcional;
2. controles, aristas temporales, covisibilidad y holdout;
3. poses optimizadas de controles;
4. predicción de propagación.

Para cada arista mostrar al seleccionar o en panel:

```text
edge_id
type y role
span_kfs
path_length_m
shared_mappoints
relative translation before/after
translation residual
relative yaw before/after
yaw residual
weight translation
weight rotation
robust scale
robust influence
```

Colores separados para:

- temporal;
- covisibilidad solver;
- holdout;
- arista con residual alto;
- hard fiducial;
- target fiducial.

No usar GT dentro del solver. La capa GT es solo visual/debug.

---

## Logs obligatorios

```text
[F1J-PRECHECK] task_id=<id> ok=<bool> reason=<...>
[F1J-COST-BREAKDOWN-BEFORE] temporal_t=<v> temporal_r=<v> covis=<v> fid=<v> regularization=<v>
[F1J-SOLVER-ITER] iter=<n> cost=<v> step_norm=<v> alpha=<v> linear_ok=<bool>
[F1J-SOLVER-STATUS] status=<...> iterations=<n> elapsed_ms=<n>
[F1J-EDGE-METRIC] edge=<id> type=<...> residual_t=<m> residual_yaw=<rad> robust_weight=<v>
[F1J-CORRECTION-FIELD] max_step_t=<m> max_step_yaw=<rad> segment=<a,b>
[F1J-REFINEMENT-REQUEST] segment=<a,b> reason=<...> suggested_kf=<id>
[F1J-DRYRUN-DECISION] decision=<CANDIDATE_FOR_VALIDATION|NEEDS_GRAPH_REFINEMENT|REJECT_DRYRUN>
[F1J-NO-STATE-MUTATION] task_id=<id> ok=true
```

Iteraciones en `DEBUG`; resumen en `INFO`.

---

## Tests unitarios y sintéticos obligatorios

Crear problemas sin ROS ni Gazebo:

### Caso A — sin deriva

La solución debe permanecer prácticamente igual.

### Caso B — deriva de yaw gradual pequeña

La corrección debe repartirse entre varios controles sin estirar traslaciones.

### Caso C — deriva translacional gradual

La corrección debe repartirse suavemente y mantener covisibilidad.

### Caso D — salto angular localizado de 90°

La solución puede concentrar parte de la corrección en una zona, pero debe conservar traslaciones locales y no exigir una bisagra predefinida.

### Caso E — salto de 180°

Debe converger o rechazar de forma explícita; nunca devolver éxito silencioso con geometría estirada.

### Caso F — deriva mixta

Combinación de drift suave y salto localizado.

### Caso G — esquina real de 90° bien soportada

La covisibilidad fuerte debe impedir que el optimizador elimine una esquina correcta solo porque el ángulo sea grande.

### Caso H — grafo insuficiente

Debe devolver `NEEDS_GRAPH_REFINEMENT` con segmento concreto.

### Caso I — fallo numérico

Debe devolver `NUMERICAL_FAILURE`, sin propuestas parciales ni mutaciones.

---

## Pruebas de replay y Gazebo

### Replay 1 — dataset de deriva grave

Esperado:

- target mejora;
- variación de traslación local acotada;
- aristas de covisibilidad siguen coherentes;
- si faltan controles, se solicita refinamiento;
- no se autoriza apply desde `1J`.

### Replay 2 — error fiducial pequeño

Esperado:

- pocas iteraciones;
- correcciones pequeñas;
- sin refinamiento innecesario.

### Gazebo

Ejecutar solo después de pasar tests sintéticos y replay.

---

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi
./codex/herramientas/build_selected_packages.sh orbslam3_server
```

---

## Criterio de éxito

`1J` queda `CONSEGUIDA` si:

- solver y coste comparten residual;
- se preserva transformación relativa completa;
- los pesos de traslación y rotación son independientes;
- covisibilidad entra en el coste;
- Huber permite tratar outliers sin bisagra fija;
- los estados de convergencia son explícitos;
- no hay mutación persistente;
- los casos sintéticos A-I pasan;
- el caso de deriva grave no produce estiramiento como solución barata;
- el builder puede recibir solicitudes de refinamiento concretas;
- no se usa GT para optimizar.

---

## Criterio de fallo o parcial

`PARCIAL` si el residual está corregido, pero falta refinamiento o covisibilidad.

`NO CONSEGUIDA` si:

- se conserva solo longitud;
- solver y coste difieren;
- `success=true` aparece sin convergencia;
- una solución estirada se clasifica como candidata válida;
- se aplica una propuesta parcial;
- se usa GT para pesos o decisión.

---

## Documentación a actualizar

- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/optimization_debug_exporter.md`
- `codex/contexto/paquetes/orbslam3_multi/pose_graph_problem.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- `codex/contexto/paquetes/orbslam3_server/launches.md`
- historial de Fase 1

---

## Entrega a la subfase 1K

`1J` entrega una propuesta inmutable que contiene:

```text
poses optimizadas de controles
estado de convergencia
coste coherente
métricas por arista
campo de correcciones de controles
solicitud de refinamiento o candidatura a validación
```

No entrega un estado ya aplicado ni una corrección rígida única del submapa.
