# Subfase 1K — Candidato transaccional y propagación por campo de correcciones

## Estado

```text
pendiente
```

Estado anterior: `realizado`.

Propiedad funcional vigente:

```text
1K = aplicar el grafo/dry-run aceptable en GlobalPoseStore
```

Lo realizado al final de la sesión del 2026-07-14 pertenece a `1K`: escritura
de poses optimizadas/propagadas, corrección heredable para KFs futuros,
publicación de MapPoints desde poses finales de KFs y eliminación del fallback
rígido en submapas corregidos. `1L` solo debe diagnosticar si esa aplicación se
ve bien en logs/RViz2.

Se reabre porque la ruta actual escribe poses en `GlobalPoseStore` antes de conocer la decisión final de `1L`, depende de rollback posterior y puede reducir una deformación no rígida a una única `submap_last_correction`. También existe una ruta en la que un apply parcial puede fallar después de escribir algunos KeyFrames y perder el backup.

La nueva subfase no publica una corrección dudosa. Prepara un estado candidato privado, propaga la corrección a todos los KeyFrames afectados y lo entrega a `1L` para validación antes del commit atómico.

---

## Objetivo técnico

Transformar una propuesta dry-run válida de `1J` en un candidato completo que:

1. incluya controles y KeyFrames no variables;
2. interpole un campo de correcciones coherente por segmentos;
3. no use una única corrección rígida para toda una deformación;
4. no modifique `RawMapDatabase`;
5. no sea visible para publishers, timers ni callbacks;
6. pueda descartarse sin rollback porque todavía no fue comprometido;
7. pueda confirmarse mediante un commit corto y atómico después de `1L`;
8. gestione KeyFrames llegados durante la optimización sin mezclar estados.
9. publique MapPoints usando la pose final del KF asociado, no una corrección
   rígida genérica de submapa;
10. haga que los KFs posteriores al último KF aplicado hereden la corrección del
    extremo optimizado.

---

## Principio de seguridad

Sustituir:

```text
backup -> escribir GlobalPoseStore -> validar -> rollback si falla
```

por:

```text
snapshot -> construir candidato privado -> validar candidato -> commit atómico
```

El rollback queda como protección excepcional ante un fallo durante el commit, no como mecanismo normal de validación.

---

## Contexto obligatorio a leer

- `AGENTS.md`
- `01_ESTADO_ACTUAL.md`
- `PIPELINE_MAESTRO.md`
- `pipeline_fase_1.md`
- versiones revisadas de `1H`, `1I`, `1J`, `1L`
- documentación de:
  - `GlobalPoseStore`
  - `PoseGraphProblem`
  - `OptimizationManager`
  - `GlobalMapBuilder`
  - `global_map_server`
- código actual de apply, backup, rollback y publicación

---

## Diagnóstico de partida

Problemas que debe corregir la revisión:

1. `ApplyCandidateResult()` escribe secuencialmente en el store.
2. Si falla tras varias escrituras, puede quedar estado parcial.
3. El servidor puede llamar a `ConfirmApply()` cuando `apply.applied=false`, eliminando el backup en vez de restaurarlo.
4. La validación posterior compara a menudo la propuesta contra la misma pose recién escrita.
5. `submap_last_corrections_` termina con la corrección del último KF recorrido, no con una semántica geométrica válida.
6. Una deformación espacial variable se reduce a una transformación rígida por orden de iteración.
7. MapPoints pueden elegir la primera observación corregida, lo que depende del orden.
8. Los checkpoints parciales han llegado a publicarse aunque la geometría intermedia fuera mala.
9. Los contadores `propagated_kfs` mezclan planificados, evaluados y escritos.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

- `include/orbslam3_multi/global_pose_store.hpp`
- `src/global_pose_store.cpp`
- `include/orbslam3_multi/optimization_manager.hpp`
- `src/optimization_manager.cpp`
- `include/orbslam3_multi/optimization_result.hpp`
- `include/orbslam3_multi/pose_graph_problem.hpp`, solo para integración
- `include/orbslam3_multi/global_map_builder.hpp`
- `src/global_map_builder.cpp`
- nuevos ficheros pequeños para transacción/campo de correcciones
- `test/`
- `CMakeLists.txt`

### `orbslam3_server`

- `src/global_map_server.cpp`
- `launch/global_orb_map_server.launch.py`

### Documentación

- documentación de paquetes
- historial de Fase 1
- auxiliares de prueba

---

## Archivos prohibidos

No modificar:

- `RawMapDatabase` para guardar poses corregidas;
- `ORB_SLAM3`;
- wrapper/mensajes;
- código legacy;
- criterio final de aceptación, que pertenece a `1L`;
- generación/visualización HTML del grafo, que pertenece a `1J`;
- GT para propagación o commit.

---

## Estructuras nuevas o revisadas

### 1. `PoseCorrection`

Representa la corrección de un control:

```text
keyframe_id
before_world_T_kf
after_world_T_kf
world_correction_C = after * before.inverse()
```

La convención izquierda/derecha debe fijarse una vez y usarse en solver, propagación y MapPoints.

### 2. `CorrectionFieldSegment`

Para cada par de controles consecutivos:

```text
control_a
control_b
correction_a
correction_b
path_start_m
path_end_m
affected_keyframes
interpolation_mode
```

### 3. `OptimizationCandidateState`

Debe contener:

```text
task_id
source_store_revision
source_raw_arrival_id
control_pose_proposals
propagated_pose_proposals
correction_field
right_tail_correction opcional
planned_counts
evaluated_counts
candidate_valid
reason
```

No debe estar insertado en `GlobalPoseStore`.

### 4. `GlobalPoseTransaction`

API orientativa:

```cpp
GlobalPoseTransaction BeginTransaction(...);
bool StagePose(...);
bool StageCorrectionField(...);
TransactionCommitResult CommitTransaction(...);
void DiscardTransaction(...);
```

Una implementación mediante copia/snapshot inmutable también es válida. La propiedad obligatoria es que no haya escrituras visibles antes del commit.

### 5. Revisar `OptimizationApplyResult`

Separar:

```text
planned_optimized_kfs
planned_propagated_kfs
evaluated_optimized_kfs
evaluated_propagated_kfs
committed_optimized_kfs
committed_propagated_kfs
```

Solo `committed_*` puede llamarse aplicado.

---

## Propagación de KeyFrames no control

### 1. Interpolar la corrección, no la pose absoluta

Para controles `A` y `B`:

```text
C_A = T_A_after * T_A_before.inverse()
C_B = T_B_after * T_B_before.inverse()
```

Para un KF intermedio con `alpha` de `1I`:

```text
C_K = InterpolateCorrection(C_A, C_B, alpha)
T_K_after = C_K * T_K_before
```

No interpolar directamente `T_A_after` y `T_B_after`.

### 2. Interpolación mínima

Para la implementación SE(2.5):

```text
translation de corrección: interpolación lineal
yaw de corrección: interpolación angular con wrap
z: interpolación lineal
roll/pitch: conservar o interpolar solo si están habilitados
```

Para SE(3) futuro, usar Log/Exp de SE(3) o traslación + quaternion slerp con convención documentada.

### 3. Modos prohibidos dentro de la ventana

No usar como ruta normal:

```text
NEAREST_CONTROL_VERTEX
INHERIT_LAST_CORRECTION
```

Solo se permite nearest-control en un caso degenerado documentado donde exista un único control válido y `1L` pueda rechazarlo. Una ventana fiducial normal debe tener dos controles por segmento.

### 4. Segmentos alrededor de correcciones localizadas

Si la corrección cambia mucho entre `A` y `B`, el candidato no debe suavizarla ciegamente sobre un tramo largo. Debe:

- comprobar que `1I` colocó controles suficientes;
- solicitar refinamiento si el span es grande;
- interpolar únicamente dentro de segmentos ya validados como representativos.

### 5. KeyFrames fijos

Los hard fiducials y controles fijos se copian sin movimiento exacto dentro de tolerancia numérica.

---

## Campo de correcciones persistente

### Problema a evitar

No guardar:

```text
submap_last_correction = corrección del último KF iterado
```

para representar una optimización no rígida.

### Representación mínima

`GlobalPoseStore` debe conservar un campo por tramos o una estructura equivalente:

```text
submap_id
ordered_control_keyframes
correction per control
valid path intervals
right_tail_correction
source_task_id
revision
```

Uso:

- KFs dentro de la ventana: corrección interpolada por segmento;
- KFs anteriores al primer anchor: conservar estado previo;
- KFs futuros posteriores al target: pueden heredar explícitamente la `right_tail_correction`, que es la corrección del target aceptado, no una corrección escogida por orden;
- una nueva optimización sustituye/compone el campo de manera transaccional.

La composición de dos campos debe estar definida y testada. No sobrescribir una corrección anterior sin considerar la pose `before` sobre la que se calculó la nueva.

---

## Tratamiento de KeyFrames llegados durante el cálculo

Cada candidato se construye contra:

```text
source_store_revision
source_raw_arrival_id
```

Antes del commit:

### Caso A — nuevos KFs después del target

Pueden rebasarse con `right_tail_correction` si:

- pertenecen al mismo epoch;
- no modifican la ventana optimizada;
- sus poses raw son válidas.

### Caso B — nuevo/actualizado KF dentro de la ventana

La transacción se considera stale y debe volver a `1I/1J` o reconstruir el candidato. No mezclar un grafo antiguo con raw actualizado dentro de la ventana.

### Caso C — cambio de epoch

Cancelar la tarea para ese submapa. Nunca propagar entre epochs.

---

## Tratamiento de MapPoints

`RawMapDatabase` permanece intacta. `GlobalMapBuilder` transforma los MapPoints al construir la nube.

### Regla mínima

1. usar el KF de referencia si tiene corrección válida;
2. si no existe, combinar correcciones de observadores válidos;
3. no elegir arbitrariamente la primera observación;
4. si los observadores proponen correcciones muy diferentes, marcar el punto como ambiguo y:
   - omitirlo temporalmente; o
   - usar una combinación robusta documentada.

Combinación orientativa:

```text
peso por observador = soporte/covisibilidad/calidad
translation = media robusta
rotation = media angular o quaternion average
```

El objetivo es evitar paredes partidas por una elección dependiente del orden.

### Métricas

Registrar:

```text
mappoints_reference_correction
mappoints_blended_correction
mappoints_ambiguous_skipped
max_observer_correction_spread
```

---

## Algoritmo de candidato

### Paso 1 — Precheck

Requerir:

- dry-run `CANDIDATE_FOR_VALIDATION`;
- solver convergido;
- sin solicitud de refinamiento;
- grafo y tarea vigentes;
- todas las propuestas finitas;
- hard fiducials inmóviles;
- plan de propagación completo.

### Paso 2 — Abrir snapshot/transacción privada

Guardar revisiones y estado `before` de todos los KFs afectados.

### Paso 3 — Stage de controles

Añadir poses propuestas sin tocar el store visible.

### Paso 4 — Construir campo por segmentos

Ordenar controles por trayectoria y construir `CorrectionFieldSegment`.

### Paso 5 — Propagar KFs omitidos

Usar `segment_alpha` y la interpolación de correcciones.

### Paso 6 — Rebasar candidatos nuevos permitidos

Solo right-tail y bajo reglas de revisión.

### Paso 7 — Preparar vista candidata

Proporcionar a `1L` una interfaz de lectura:

```text
GetCandidateWorldPose(kf)
GetCandidateCorrection(kf)
GetCandidateGlobalMapPreview()
```

No publicar la preview en topics normales.

### Paso 8 — Entregar a `1L`

No commit todavía.

---

## Commit atómico

`1L` solicita commit únicamente tras `ACCEPT_COMMIT`.

El commit debe:

1. verificar de nuevo revisiones;
2. bloquear el store durante un intervalo corto;
3. aplicar todas las poses/campo o ninguna;
4. actualizar revisión;
5. liberar bloqueo;
6. permitir publicación posterior del estado confirmado.

Si una escritura falla:

- abortar antes de exponer el nuevo revision;
- restaurar internamente la transacción;
- nunca llamar a una función equivalente a `ConfirmApply()` sobre un apply fallido.

---

## Logs obligatorios

```text
[F1K-CANDIDATE-BEGIN] task_id=<id> store_revision=<r> raw_arrival=<a>
[F1K-CONTROL-STAGED] kf=<id> delta_t=<m> delta_yaw=<rad>
[F1K-CORRECTION-SEGMENT] a=<id> b=<id> affected=<n> max_gradient_t=<m> max_gradient_yaw=<rad>
[F1K-PROPAGATION-STAGED] kf=<id> a=<id> b=<id> alpha=<v> delta_t=<m> delta_yaw=<rad>
[F1K-RIGHT-TAIL] target_kf=<id> correction_t=<m> correction_yaw=<rad>
[F1K-CANDIDATE-SUMMARY] planned=<n> evaluated=<n> committed=0 visible=false
[F1K-CANDIDATE-STALE] reason=<...>
[F1K-COMMIT-BEGIN] task_id=<id> expected_revision=<r>
[F1K-COMMIT-SUMMARY] success=<bool> committed_optimized=<n> committed_propagated=<n> new_revision=<r>
[F1K-COMMIT-ABORT] reason=<...> partial_visible=false
[F1K-MAPPOINT-CORRECTION-STATS] ...
```

No usar `WARN` para cada KF normal.

---

## Tests unitarios obligatorios

1. Interpolación `alpha=0`, `0.5`, `1`.
2. Wrap correcto de yaw cerca de `+pi/-pi`.
3. Propagación conserva la pose `before` y aplica la corrección, no interpola poses absolutas.
4. Hard fiducial no se mueve.
5. Todos los KFs del plan quedan staged.
6. Un fallo a mitad del stage no cambia el store.
7. Un fallo a mitad del commit no deja estado visible parcial.
8. Revisión stale cancela commit.
9. KF nuevo posterior hereda right-tail explícito.
10. KF nuevo dentro de la ventana invalida candidato.
11. No se cruza epoch.
12. Campo no rígido no actualiza una única `last_correction` arbitraria.
13. MapPoint con referencia válida usa esa corrección.
14. MapPoint sin referencia combina observadores de forma independiente del orden.
15. MapPoint con spread excesivo se marca ambiguo.

---

## Pruebas de integración

### Prueba 1 — candidato sin publicación

Durante validación, comprobar que `/global_sparse_cloud` sigue mostrando el estado confirmado anterior.

### Prueba 2 — commit aceptado

Tras `1L ACCEPT_COMMIT`, todas las poses aparecen en la misma revisión y la nube se publica después.

### Prueba 3 — rechazo

Un candidato forzado a fallar se descarta sin modificar ninguna pose ni nube.

### Prueba 4 — llegada de KFs concurrentes

Generar nuevos KFs mientras se optimiza y comprobar los casos A/B anteriores.

---

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi
./codex/herramientas/build_selected_packages.sh orbslam3_server
```

---

## Criterio de éxito

`1K` queda `CONSEGUIDA` si:

- existe candidato privado completo;
- ningún publisher ve el candidato antes de aceptar;
- la propagación usa correcciones interpoladas por trayectoria;
- no se usa una única corrección arbitraria para la deformación;
- las revisiones evitan commits stale;
- el commit es todo-o-nada;
- los contadores distinguen planned/evaluated/committed;
- los MapPoints no dependen del orden de observaciones;
- los tests de fallo parcial demuestran `partial_visible=false`.

---

## Criterio de fallo o parcial

`PARCIAL` si el candidato es privado, pero el campo de correcciones o MapPoints siguen incompletos.

`NO CONSEGUIDA` si:

- se escriben poses antes de validar;
- un apply fallido elimina el backup sin restaurar;
- se publica un checkpoint parcial;
- se usa `last_correction` por orden de iteración;
- un commit puede dejar KeyFrames parcialmente escritos;
- se propaga entre epochs.

---

## Documentación a actualizar

- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`
- `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`
- `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- historial de Fase 1

---

## Entrega a la subfase 1L

`1K` entrega:

```text
candidato privado completo
vista de poses before/after
campo de correcciones
métricas de propagación
preview no publicada
revisiones para commit
```

`1L` decide si se confirma, se refina o se descarta.
