# Subfase 1N — `LoopDetector`: candidatos BoW sin geometría ni optimización

## Estado

```text
por implementar
```

---

Nota 2026-07-10:

```text
La implementación completa de 1N queda bloqueada hasta validar 1M. Se permite
adelantar únicamente la infraestructura sin BoW real mientras el contrato de
`OrbKeyFrame` no esté disponible en este checkout.
```

Nota 2026-07-11:

```text
`1L` queda `PARCIAL` tras inspeccion RViz2 y se revalidará en el futuro. La
deuda de covisibilidad/loops no se absorbe en `1L`; 1N no se considera realizada
hasta que 1M quede validada y el wrapper exponga BoW/FeatureVector verificables.
```

Antes de iniciar BoW/loops, la ruta comun de optimizacion debe preservar anclajes fiduciales previos y sus vecindades, no solo el KF hard fiducial exacto. Si no se corrige en `1I-1L`, los loops de `1Q` podrian reutilizar el mismo pipeline y producir desplazamientos globales del submapa.

---

## Objetivo tecnico

Crear la primera versión de detección de candidatos de loop basada en BoW.

La subfase 1N introduce en `orbslam3_multi` una clase de detección de candidatos visuales. Esta clase debe recibir el ID de un KeyFrame nuevo, consultar su BoW en `RawMapDatabase` y buscar KeyFrames candidatos en la base de datos.

En esta subfase **no se construyen subnubes**, **no se hace RANSAC**, **no se calcula error geométrico**, **no se fusionan landmarks** y **no se crea todavía una tarea de optimización por loop**.

Antes de devolver un candidato BoW hacia 1O, `LoopDetector` debe consultar
`CovisibilityDatabase`:

```text
si query_kf ↔ candidate_kf ya existe como covisibilidad confirmada:
    no enviar a verificación geométrica costosa;
    registrar que el par ya está confirmado;
si no existe:
    devolver LoopCandidate normal para 1O.
```

Esto evita repetir subnubes/RANSAC cuando ORB-SLAM3 ya confirmó covisibilidad
intra-dron o cuando un loop geométrico previo ya insertó la relación.

La lógica esperada es:

```text
RawMapDatabase inserta nuevo KeyFrame
        ↓
servidor/backend notifica a LoopDetector
        ↓
LoopDetector consulta BoW del nuevo KeyFrame
        ↓
LoopDetector compara contra KFs indexados en RawMapDatabase
        ↓
LoopDetector filtra/rankea candidatos
        ↓
LoopDetector consulta CovisibilityDatabase
        ↓
descarta/etiqueta pares ya confirmados
        ↓
devuelve LoopCandidate(s)
        ↓
logs verifican candidatos o ausencia de candidatos
```

Regla principal:

```text
BoW busca amplio.
Los filtros reducen.
La geometría confirmará en la fase siguiente.
```

La comparación BoW no debe limitarse solo a KeyFrames cercanos espacialmente, porque un loop importante puede estar lejos en la trayectoria, en otro submapa o en otro dron. La búsqueda debe ser amplia sobre los KeyFrames indexados y después aplicar filtros para obtener una lista corta y útil.

---

## Contexto obligatorio a leer

Antes de actuar, Codex debe leer:

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1A.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1B.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1C.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1D.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1E.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1F.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1G.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1H.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1I.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1J.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1K.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1M.md`
- historial reciente de Fase 1;
- documentación de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/orbslam3_msgs/`
  - `codex/contexto/paquetes/simulacion_dron/`
- ADRs relacionados con:
  - `RawMapDatabase`;
  - BoW/FeatureVector exportado por wrapper;
  - loops;
  - submapas `(drone_id, map_epoch)`;
  - separación fiducial/loop;
  - no usar GT para loops.

---

## Diagnostico de partida

Hasta esta subfase, el pipeline nuevo debe tener:

1. un servidor nuevo que recibe datos de wrappers;
2. `RawMapDatabase` con KeyFrames, MapPoints, BoW, FeatureVector, observaciones y poses locales;
3. `CovisibilityDatabase` con relaciones confirmadas intra-dron de ORB-SLAM3 y
   API de consulta;
4. journal/replay de deltas;
5. `GlobalPoseStore`;
6. anclaje por fiducial;
7. publicación básica de nube anclada con score;
8. ruta de optimización fiducial ya definida en 1H–1L.

El problema actual es que todavía no existe la ruta nueva de loops basada en BoW dentro de `orbslam3_multi`.

La subfase 1N debe crear esa primera pieza:

```text
nuevo KF → búsqueda BoW → candidatos visuales
```

No se debe confundir esta subfase con la validación geométrica. Un candidato BoW solo significa:

```text
este KeyFrame se parece visualmente a otro.
```

No significa:

```text
hay loop confirmado.
```

La geometría, las subnubes y RANSAC se implementarán en la subfase posterior.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Se pueden crear o modificar archivos relacionados con detección de candidatos BoW:

- `orbslam3_multi/include/orbslam3_multi/loop_detector.hpp`
- `orbslam3_multi/src/loop_detector.cpp`
- `orbslam3_multi/include/orbslam3_multi/bow_loop_candidate_detector.hpp` si se decide separar internamente
- `orbslam3_multi/src/bow_loop_candidate_detector.cpp` si se decide separar internamente
- `orbslam3_multi/include/orbslam3_multi/loop_candidate.hpp`
- `orbslam3_multi/include/orbslam3_multi/covisibility_database.hpp`
- `orbslam3_multi/src/covisibility_database.cpp` solo si falta API de consulta
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_multi/package.xml` solo si fuese necesario

Si ya existen clases legacy de BoW, pueden consultarse o adaptarse de forma controlada, pero sin reactivar rutas legacy peligrosas.

### `orbslam3_server`

Cambios mínimos para llamar a `LoopDetector` cuando entren KeyFrames nuevos:

- `orbslam3_server/src/global_map_server.cpp`
- clases auxiliares del servidor nuevo si existen
- launch nuevo del servidor si necesita parámetros debug

### `codex/archivos_auxiliares`

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`
- datasets/replay creados en 1C/1E si se usan para pruebas

### Documentación

- `codex/contexto/paquetes/orbslam3_multi/loop_detector.md`
- `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- historial de Fase 1 / subfase 1N

---

## Archivos prohibidos

No modificar:

- `ORB_SLAM3/`, salvo permiso explícito.
- `orbslam3_ros2` / wrapper, salvo bug crítico demostrado.
- `orbslam3_msgs`, salvo necesidad justificada y aprobada.
- `global_map_server_antiguo.cpp`, salvo consulta.
- archivos legacy de `orbslam3_multi/legacy/`, salvo consulta.
- lógica de subnubes/RANSAC.
- lógica de fusión de landmarks.
- lógica de optimización por loop.
- inserción de loops confirmados en `CovisibilityDatabase`; `1N` solo consulta.
- rutas de fiducial ya validadas salvo integración mínima necesaria.
- `RawMapDatabase` para corregir poses.
- `GlobalPoseStore` para mover poses por loops en esta subfase.
- `build/`, `install/`, `log/` como código fuente.

No borrar código legacy de forma masiva.

---

## Funciones, clases o nodos que hay que localizar

`planificador_fase` debe localizar o crear, según exista o no:

### En `orbslam3_multi`

- `RawMapDatabase`
- método para obtener KeyFrames nuevos desde última consulta;
- método para obtener BoW de un KeyFrame;
- método para iterar KeyFrames candidatos;
- `LoopDetector`
- `LoopCandidate`
- `LoopCandidateResult`
- `CovisibilityDatabase`
- índice BoW interno si existe o se crea;
- acceso a estado de submapa/anclaje si se usa en filtros;
- `GlobalPoseStore` para comprobar si candidato tiene pose global, si aplica.

### Consulta obligatoria a `CovisibilityDatabase`

Antes de devolver un candidato a 1O:

```text
confirmed = covis_db.HasConfirmedEdge(query_kf, candidate_kf)
```

Si `confirmed=true`, emitir un resultado de diagnóstico y no lanzar la
verificación geométrica:

```text
[F1N-BOW-SKIP-CONFIRMED-COVIS]
```

Ese skip no es rechazo del loop; significa que la relación ya está confirmada y
no hace falta gastar tiempo en `SubcloudLoopVerifier`.

### En `orbslam3_server`

- callback donde se inserta un delta en `RawMapDatabase`;
- lista de KeyFrames nuevos devueltos por `InsertDelta` / `InsertFullSnapshot`;
- punto donde notificar a `LoopDetector`;
- logs del servidor nuevo;
- parámetros de loop BoW.

### Topics/services relevantes

- `/dron_X/orbslam/orb_map_delta`
- `/dron_X/orbslam/get_full_map`
- replay de datasets guardados en 1C;
- topic de nube global de 1F, si se sigue publicando durante la prueba.

---

## Cambios requeridos

### 1. Crear estructura `LoopCandidate`

No devolver solo `true/false`. Un candidato debe contener información suficiente para fases posteriores y para logs.

Campos mínimos recomendados:

```text
query_kf_id
candidate_kf_id
query_submap_id
candidate_submap_id
bow_score
rank
same_drone
same_submap
kf_gap
candidate_has_world_pose
candidate_is_anchored
query_num_mappoints
candidate_num_mappoints
candidate_is_bad
rejection_reason opcional
```

Campos opcionales útiles:

```text
query_drone_id
candidate_drone_id
query_epoch
candidate_epoch
query_map_sequence
candidate_map_sequence
temporal_distance
spatial_distance_if_available
source = BOW
```

Logs:

```text
[F1N-LOOP-CANDIDATE]
```

---

### 2. Crear `LoopCandidateResult`

Debe representar la salida de procesar un KeyFrame nuevo.

Campos recomendados:

```text
query_kf_id
processed
reason
num_kfs_compared
num_candidates_raw
num_candidates_after_filter
candidates
best_candidate
```

Logs:

```text
[F1N-LOOP-CANDIDATE-SUMMARY]
```

---

### 3. Crear `LoopDetector`

Nueva clase en `orbslam3_multi`.

Responsabilidad inicial:

```text
Detectar candidatos visuales por BoW para un KeyFrame nuevo.
```

Métodos mínimos recomendados:

```cpp
LoopCandidateResult ProcessNewKeyFrame(
    GlobalKeyFrameId query_kf_id,
    const RawMapDatabase& raw_db,
    const GlobalPoseStore* pose_store_optional);
```

o, si se prefiere mantener estado interno:

```cpp
void AddOrUpdateKeyFrame(GlobalKeyFrameId kf_id, const RawMapDatabase& raw_db);
LoopCandidateResult QueryCandidates(GlobalKeyFrameId query_kf_id, const RawMapDatabase& raw_db);
```

La clase puede mantener un índice BoW interno ligero para no recorrer todo manualmente si ya existe infraestructura. Si no existe, en 1N puede implementarse búsqueda simple sobre los KFs indexados, siempre que sea suficiente para pruebas.

Condiciones de seguridad:

- no construir subnubes;
- no hacer RANSAC;
- no crear `LoopOptimizationTask`;
- no modificar poses;
- no fusionar landmarks.

Logs:

```text
[F1N-LOOP-KF-QUERY]
```

---

### 4. Búsqueda BoW amplia

La búsqueda BoW debe ser amplia sobre KeyFrames indexados en `RawMapDatabase`.

No se debe limitar a KeyFrames cercanos espacialmente.

Regla:

```text
Comparar el BoW del KeyFrame nuevo contra la base de KeyFrames indexados.
Después filtrar y rankear.
```

La búsqueda puede priorizar o filtrar candidatos con pose global disponible, pero no debe asumir que “cercano en world” es requisito para loop.

En 1N se recomienda empezar con candidatos de submapas anclados o candidatos con pose global disponible para facilitar fases posteriores. Debe quedar documentado si se excluyen submapas no anclados.

Logs:

```text
[F1N-LOOP-BOW-SEARCH]
```

Debe mostrar:

```text
query_kf_id
indexed_kfs
compared_kfs
raw_candidates
```

---

### 5. Filtros iniciales de candidatos

Aplicar filtros para evitar candidatos inútiles.

Filtros recomendados:

#### 5.1. BoW inexistente o vacío

Descartar si query o candidato no tiene BoW válido.

```text
reason=no_bow
```

#### 5.2. KeyFrame bad o incompleto

Descartar si candidato está marcado bad, no tiene suficientes MapPoints observados o no tiene descriptores suficientes.

```text
reason=bad_or_incomplete
```

#### 5.3. Mismo submapa demasiado cercano

No comparar con KFs demasiado recientes del mismo submapa:

```text
same_submap && abs(query_kf_id - candidate_kf_id) < min_kf_gap
```

o usar orden/arrival/map_sequence si `kf_id` no es fiable como gap.

```text
reason=too_recent_same_submap
```

#### 5.4. Candidato sin pose global

En la política inicial, se puede descartar o penalizar si no tiene pose global.

```text
reason=no_world_pose
```

Si se decide permitir candidatos no anclados para anchor por loop futuro, debe documentarse y no mezclarse con confirmación de loop.

#### 5.5. Limitar top-K

Devolver como máximo:

```text
loop_bow_max_candidates
```

#### 5.6. Diversidad por submapa

Evitar que todos los candidatos sean KFs consecutivos del mismo submapa.

Parámetros recomendados:

```text
loop_bow_max_candidates_per_submap
loop_bow_min_kf_gap_same_submap
```

Logs:

```text
[F1N-LOOP-CANDIDATE-FILTER]
```

---

### 6. Ranking de candidatos

El ranking inicial puede usar:

```text
bow_score
bonus si candidato tiene pose global
bonus si submapa está anclado
bonus si tiene suficientes MapPoints observados
penalización si es demasiado reciente
penalización si tiene pocos puntos
```

No debe usar geometría ni RANSAC.

Campos de ranking deben loggearse de forma resumida.

Logs:

```text
[F1N-LOOP-CANDIDATE-RANK]
```

---

### 7. Integración con `RawMapDatabase`

`RawMapDatabase` debe poder indicar qué KFs son nuevos tras un delta/full snapshot para que el servidor/backend llame a `LoopDetector`.

Ejemplo de flujo:

```text
InsertDelta(...)
    ↓
IngestResult.new_keyframe_ids
    ↓
for kf_id in new_keyframe_ids:
    LoopDetector.ProcessNewKeyFrame(kf_id, raw_db, pose_store)
```

Si `InsertFullSnapshot` trae KFs antiguos, no debe reconsultarlos todos sin control. Puede tener política:

```text
solo nuevos KFs
o
modo debug process_all_snapshot_kfs limitado
```

Logs:

```text
[F1N-LOOP-NEW-KF-DISPATCH]
```

---

### 8. Integración con servidor nuevo

El servidor debe llamar a `LoopDetector` cuando entren nuevos KFs.

En 1N el servidor solo debe loggear resultados. No debe actuar sobre ellos.

Logs del servidor:

```text
[F1N-SERVER-LOOP-DETECTOR-CALL]
[F1N-SERVER-LOOP-CANDIDATES-RX]
```

---

### 9. Soporte para replay

La detección BoW debe funcionar tanto con simulación live como con replay de deltas de 1C.

En replay, al reinsertar cada delta, se deben procesar los nuevos KFs en el mismo orden de llegada reproducido.

Logs:

```text
[F1N-LOOP-REPLAY-KF-QUERY]
```

---

### 10. Preparar interfaz para fase posterior de subnubes

Aunque 1N no construye subnubes, debe dejar la salida preparada para la fase siguiente.

La fase posterior recibirá:

```text
query_kf_id
candidate_kf_id
candidate_submap_id
bow_score
rank
```

y construirá:

```text
query subcloud
candidate/global subcloud
matching geométrico
RANSAC
LOOP-SUBCLOUD-ERROR
```

No implementar esta lógica en 1N.

---

## Cambios prohibidos

- No construir subnubes.
- No hacer matching geométrico.
- No ejecutar RANSAC.
- No calcular error de loop.
- No fusionar landmarks.
- No crear `LoopOptimizationTask`.
- No llamar a `PoseGraphBuilder` por loop.
- No llamar a `OptimizationManager` por loop.
- No mover poses.
- No modificar `GlobalPoseStore` salvo consultas.
- No usar GT para loops.
- No filtrar solo por cercanía espacial.
- No aceptar BoW como loop confirmado.
- No bloquear fiduciales ni rutas de optimización fiducial ya implementadas.

---

## Paquetes a compilar

Comando esperado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron
```

Si `simulacion_dron` no se modifica ni se requiere para build incremental, puede omitirse justificándolo en historial.

Si falla:

```bash
./codex/herramientas/reduce_build_log.sh
```

El diagnóstico debe leer:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

---

## Pruebas Gazebo requeridas

### Prueba 1 — candidatos BoW live multi-dron

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. arrancar simulación multi-dron;
2. mover drones por una trayectoria con revisitas o zonas visualmente similares;
3. insertar KeyFrames en `RawMapDatabase`;
4. llamar a `LoopDetector` por cada KF nuevo;
5. loggear candidatos BoW o ausencia de candidatos;
6. no construir subnubes ni optimizar.

Puede reutilizarse una de las pruebas clave largas de 1C si genera suficiente solape visual:

- dos drones recorriendo el edificio en sentidos opuestos;
- dos vueltas alrededor de la casa;
- prueba simple fiducial-izquierda-vuelta.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observación esperada en RViz2:

```text
No es obligatoria para declarar la subfase.
La nube de 1F puede seguir mostrándose.
Opcionalmente se pueden mostrar líneas debug entre KF query y candidatos BoW,
pero no es requisito.
```

---

## Pruebas replay requeridas

### Prueba 2 — candidatos BoW desde dataset guardado

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Secuencia esperada:

1. cargar dataset de deltas guardado en 1C;
2. reproducir deltas con timer;
3. para cada nuevo KF, llamar a `LoopDetector`;
4. loggear candidatos o ausencia de candidatos;
5. comprobar que el resultado es repetible sin ejecutar Gazebo largo.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si existe una prueba replay sin Gazebo más rápida y validada, `planificador_fase` puede usarla justificándolo.

---

## Prueba recomendada adicional

### Prueba 3 — filtro same-submap y diversidad

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml
```

Objetivo:

Validar que `LoopDetector` no devuelve solo KFs consecutivos del mismo submapa y que aplica filtros de gap/diversidad.

Puede hacerse con replay.

Logs esperados:

```text
[F1N-LOOP-CANDIDATE-FILTER] reason=too_recent_same_submap
[F1N-LOOP-CANDIDATE-SUMMARY]
```

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1C-RAWDB|F1F-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 3

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1N-LOOP-CANDIDATE-FILTER|F1N-LOOP-CANDIDATE-SUMMARY|too_recent_same_submap|ERROR|FATAL|Segmentation fault|Killed
```

La reducción genera:

```text
codex/archivos_auxiliares/logs/prueba_1.reduced.log
codex/archivos_auxiliares/logs/prueba_2.reduced.log
codex/archivos_auxiliares/logs/prueba_3.reduced.log
```

Si el reducido no muestra suficiente información, revisar el log completo antes de repetir Gazebo o concluir que falta un marcador.

---

## Marcadores tecnicos obligatorios

Deben aparecer:

```text
[F1N-LOOP-NEW-KF-DISPATCH]
[F1N-LOOP-KF-QUERY]
[F1N-LOOP-BOW-SEARCH]
[F1N-LOOP-CANDIDATE-SUMMARY]
```

Si hay candidatos:

```text
[F1N-LOOP-BOW-CANDIDATES]
[F1N-LOOP-CANDIDATE]
[F1N-LOOP-CANDIDATE-RANK]
```

Si no hay candidatos:

```text
[F1N-LOOP-NO-CANDIDATES]
```

Si se filtran candidatos:

```text
[F1N-LOOP-CANDIDATE-FILTER]
```

No debe aparecer:

```text
LOOP_CONFIRMED
LOOP_OPT_TASK_CREATED
SUBCLOUD_RANSAC
FUSION_EVENT
OPT-APPLY
```

salvo que sean logs legacy ajenos claramente documentados y no producidos por la ruta nueva.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. se ejecutan las pruebas requeridas o alternativas equivalentes justificadas;
3. `scenario_runner_node` se ejecuta en pruebas Gazebo que lo usen;
4. los goals requeridos terminan con `success=true` si la prueba tiene trayectoria;
5. `RawMapDatabase` entrega KeyFrames nuevos al flujo de detección;
6. `LoopDetector` procesa KeyFrames nuevos;
7. el log muestra búsqueda BoW amplia;
8. el log muestra candidatos BoW o ausencia justificada;
9. se aplican filtros de seguridad;
10. se devuelve una lista de `LoopCandidate` con información suficiente;
11. no se construyen subnubes;
12. no se hace RANSAC;
13. no se crea tarea de optimización por loop;
14. no se modifican poses;
15. no se usa GT para loops;
16. no aparecen errores graves no explicados;
17. la documentación queda actualizada;
18. el historial registra evidencia positiva y negativa.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- no se procesan KeyFrames nuevos;
- no hay búsqueda BoW;
- faltan marcadores obligatorios;
- se confirma loop sin geometría;
- se crea tarea de optimización por loop;
- se modifica `GlobalPoseStore`;
- se usa GT para loop;
- aparece un error grave no explicado.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- funciona en replay pero no en live;
- procesa KFs pero no encuentra candidatos en las pruebas;
- los candidatos aparecen pero faltan algunos campos de diagnóstico;
- los filtros funcionan parcialmente;
- la búsqueda es amplia pero todavía poco eficiente;
- no se logra prueba con revisita visual clara, pero la ruta está instrumentada.

La subfase debe marcarse como `BLOQUEADA` solo si:

- los wrappers no exportan BoW/FeatureVector de forma usable;
- `RawMapDatabase` no guarda BoW;
- no existe dataset ni simulación con KFs suficientes;
- falta información crítica y no se puede resolver con cambios mínimos.

---

## Documentacion a actualizar

Actualizar obligatoriamente:

- historial de Fase 1 / subfase 1N;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- crear o actualizar:
  - `codex/contexto/paquetes/orbslam3_multi/loop_detector.md`;
  - `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
- actualizar:
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`.

La documentación debe explicar:

- que 1N solo detecta candidatos BoW;
- que BoW no confirma loops;
- que la búsqueda es amplia y luego se filtra;
- qué filtros se aplican;
- qué contiene `LoopCandidate`;
- qué logs validan la subfase;
- qué queda pendiente para subnubes/RANSAC;
- que no se usa GT para loops.

No marcar como `realizado` si no se cumplen los criterios de éxito.

---

## Notas de diseño que deben respetarse en fases posteriores

1. BoW genera candidatos visuales, no loops confirmados.
2. La fase siguiente deberá construir subnubes query/candidate a partir de estos candidatos.
3. La comparación geométrica será la que decida si hay:
   - rechazo;
   - fusión;
   - tarea de optimización por loop.
4. La búsqueda BoW no debe limitarse a cercanía espacial.
5. La geometría y los scores podrán influir en fases posteriores, pero no deben mezclarse con la detección BoW básica de 1N.
