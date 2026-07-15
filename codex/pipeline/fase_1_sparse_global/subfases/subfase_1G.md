# Subfase 1G — Full snapshot y reconciliación segura con poses globales

## Estado

```text
realizado
```

---

## Objetivo tecnico

Reintroducir en el servidor nuevo la ruta de **snapshot completo** del wrapper ORB-SLAM3 y hacer que esos snapshots actualicen `RawMapDatabase` sin destruir el estado global ya mantenido por `GlobalPoseStore`.

La subfase debe conseguir que el servidor pueda:

1. pedir un mapa completo al wrapper mediante el servicio `orbslam/get_full_map`;
2. asignar un `arrival_id` tambien a cada snapshot completo recibido;
3. insertar el snapshot en `RawMapDatabase` mediante `InsertFullSnapshot(arrival_id, snapshot)`;
4. reconciliar KeyFrames y MapPoints ya existentes;
5. detectar KeyFrames antiguos cuya pose local ORB-SLAM3 haya cambiado;
6. avisar a `GlobalPoseStore` de esos cambios raw;
7. recalcular poses globales derivadas de anclaje para KFs no optimizados;
8. conservar poses globales optimizadas por el servidor si existieran;
9. recalcular la correccion interna de KFs optimizados respecto a la nueva pose raw;
10. guardar en los logs cualquier conflicto entre cambios raw de ORB-SLAM3 y poses globales del servidor.

La subfase **no** debe optimizar, fusionar landmarks ni resolver loops. Su objetivo es sincronizacion robusta y trazabilidad.

---

## Contexto obligatorio a leer

Antes de planificar o modificar codigo, Codex debe leer:

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
- historial reciente de Fase 1
- documentacion de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/orbslam3_msgs/`
- ADRs relacionados en `codex/contexto/decisiones/`

---

## Diagnostico de partida

Hasta esta subfase, el servidor nuevo ya deberia tener:

- recepcion de deltas `OrbMap` desde wrappers;
- `arrival_id` creciente asignado por el servidor;
- `RawMapDatabase` con estado raw por `(drone_id, map_epoch)`;
- journal de deltas para guardado/replay;
- `GlobalPoseStore` ligero para anchors, poses globales y correcciones;
- anclaje por fiducial simulado mediante `FiducialAnchorManager`;
- publicacion basica de nube sparse global con score mediante `GlobalMapBuilder` y `LandmarkScoreManager`.

El problema pendiente es que un wrapper ORB-SLAM3 puede modificar informacion antigua y el servidor nuevo todavia no tiene una ruta robusta para resincronizarse mediante snapshot completo.

Casos que debe cubrir esta subfase:

```text
1. El servidor arranca tarde y necesita recuperar el mapa completo.
2. Se pierde algun delta y el servidor pide snapshot para resincronizar.
3. ORB-SLAM3 local detecta un loop interno y modifica poses locales de KeyFrames antiguos.
4. ORB-SLAM3 marca MapPoints o KeyFrames como bad.
5. Un snapshot contiene KFs que ya existian en RawMapDatabase.
6. Algunos de esos KFs pueden tener ya pose global derivada de anchor en GlobalPoseStore.
7. En fases futuras, algunos KFs podrian tener pose global optimizada por el servidor.
```

Regla conceptual obligatoria:

```text
RawMapDatabase:
    ORB-SLAM3 tiene autoridad sobre datos raw/locales.

GlobalPoseStore:
    el servidor tiene autoridad sobre la colocacion global en world.
```

Por tanto, un full snapshot puede cambiar `local_T_kf` en `RawMapDatabase`, pero no debe pisar directamente una pose global optimizada en `GlobalPoseStore`.

---

## Archivos permitidos a modificar

Codex puede modificar, si existen o si deben crearse en esta subfase:

### `orbslam3_server`

- `orbslam3_server/src/global_map_server.cpp`
- `orbslam3_server/include/...` si el servidor nuevo usa headers propios
- `orbslam3_server/launch/global_map_server.launch.py`
- `orbslam3_server/CMakeLists.txt` solo si hace falta registrar nuevos archivos/targets
- `orbslam3_server/package.xml` solo si aparece una dependencia real nueva

### `orbslam3_multi`

- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/include/orbslam3_multi/multi_drone_system.hpp`
- `orbslam3_multi/src/multi_drone_system.cpp`
- nuevos archivos auxiliares internos si son necesarios para representar resultados de snapshot, por ejemplo:
  - `orbslam3_multi/include/orbslam3_multi/raw_map_ingest_result.hpp`
  - `orbslam3_multi/include/orbslam3_multi/raw_pose_update.hpp`

### Pruebas y auxiliares

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml`
- datasets generados en `codex/archivos_auxiliares/`, por ejemplo:
  - `rawdb_record_1G.json`
  - `rawdb_record_1G.bin`
  - `rawdb_replay_1G.json`

### Documentacion

- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`
- `codex/contexto/paquetes/orbslam3_server/launches.md`
- `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`
- `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`
- historial de Fase 1
- esta subfase o la siguiente solo si el resultado obliga a ajustar el plan

---

## Archivos prohibidos

No modificar en esta subfase:

- `ORB_SLAM3/` salvo permiso explicito del usuario;
- el wrapper `orbslam3_ros2` salvo bug confirmado que bloquee el servicio `get_full_map`;
- `orbslam3_msgs` salvo que falte una dependencia estrictamente necesaria para compilar lo ya definido;
- archivos legacy de `orbslam3_multi/legacy/` salvo para crear/copiar documentacion de referencia no compilada;
- `global_map_server_antiguo.cpp` salvo para consulta estatica;
- algoritmos de loop, fusion u optimizacion no pertenecientes a esta subfase;
- rutas de build/install/log manualmente salvo limpieza minima de artefactos generados que bloqueen build/simulacion.

---

## Funciones, clases o nodos que hay que localizar

`planificador_fase` debe localizar con busqueda estatica antes de implementar:

### Servidor

- nodo ROS nuevo del servidor global;
- callback actual de recepcion de `OrbMap` delta;
- contador actual `arrival_id` de deltas;
- estructura/conversor ROS `orbslam3_msgs::msg::OrbMap` -> estructura interna de `orbslam3_multi`;
- clientes existentes o legacy del servicio `orbslam/get_full_map`;
- parametros de drones/namespaces;
- timer o mecanismo para request de full snapshot;
- modo record/replay creado en 1C.

### Backend `orbslam3_multi`

- `RawMapDatabase`
- `RawMapDatabase::InsertDelta`
- `RawMapDatabase::InsertFullSnapshot`, si ya existe parcialmente
- `RawMapDatabase::SaveToPath`
- estructura que representa `delta_journal` / `snapshot_journal`
- `GlobalPoseStore`
- `GlobalPoseStore::SetSubmapAnchor`
- `GlobalPoseStore::SetOptimizedKeyFramePose`
- `GlobalPoseStore::OnNewKeyFrameInserted` o equivalente
- `GlobalPoseStore::GetWorldPose`
- `MultiDroneSystem` o fachada equivalente si ya se usa para coordinar backend

Si algun nombre no existe exactamente, `planificador_fase` debe localizar el equivalente y documentarlo antes de implementar.

### Topics / services relacionados

- topic delta de cada dron, normalmente:

```text
/dron_X/orbslam/orb_map_delta
```

- servicio snapshot completo de cada dron, normalmente:

```text
/dron_X/orbslam/get_full_map
```

- topic de nube global publicado en 1F;
- parametros de replay/record si existen.

---

## Cambios requeridos

### 1. Cliente de full snapshot en el servidor nuevo

**Intencion:** permitir que el servidor nuevo pida al wrapper un mapa completo.

**Archivo probable:** `orbslam3_server/src/global_map_server.cpp`.

**Cambio:** crear clientes ROS 2 para `orbslam/get_full_map` por dron/namespace configurado.

**Condicion de seguridad:** no bloquear el hilo principal esperando indefinidamente; usar llamada asincrona o timeout razonable.

**Logs obligatorios:**

```text
[F1G-FULL-SNAPSHOT-CLIENT-READY]
[F1G-FULL-SNAPSHOT-REQUEST]
[F1G-FULL-SNAPSHOT-RX]
[F1G-FULL-SNAPSHOT-TIMEOUT]
```

Ejemplo:

```text
[F1G-FULL-SNAPSHOT-REQUEST] drone_id=1 service=/dron_1/orbslam/get_full_map reason=startup_resync
```

---

### 2. `arrival_id` para snapshots completos

**Intencion:** mantener un unico orden de llegada para deltas y snapshots.

**Archivo probable:** `orbslam3_server/src/global_map_server.cpp`.

**Cambio:** cada respuesta de full snapshot debe recibir un `arrival_id` creciente igual que los deltas.

**Condicion de seguridad:** `arrival_id` debe ser monotono y unico dentro de la ejecucion/record.

**Logs obligatorios:**

```text
[F1G-FULL-SNAPSHOT-ARRIVAL]
```

Ejemplo:

```text
[F1G-FULL-SNAPSHOT-ARRIVAL] arrival_id=37 drone_id=1 epoch=0 seq=18 kfs=82 mps=6040
```

---

### 3. `RawMapDatabase::InsertFullSnapshot`

**Intencion:** insertar/reconciliar un mapa completo sin duplicar KFs/MPs y sin borrar indebidamente datos que aun no se sepa reconciliar.

**Archivo probable:**

- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`

**Cambio:** implementar o completar:

```cpp
FullSnapshotIngestResult InsertFullSnapshot(
    uint64_t arrival_id,
    const ImportedOrbMap& snapshot);
```

El resultado debe incluir al menos:

```text
arrival_id
submap_id
new_keyframes
updated_keyframes
new_mappoints
updated_mappoints
bad_keyframes
bad_mappoints
raw_pose_changed_keyframes
large_raw_pose_changed_keyframes
```

**Condicion de seguridad:** en esta subfase, si la reconciliacion fina de elementos ausentes es peligrosa, se debe usar politica conservadora:

```text
insertar/actualizar lo recibido;
marcar bad si el wrapper lo indica explicitamente;
no borrar elementos solo porque no aparezcan en el snapshot salvo que exista certeza clara.
```

**Logs obligatorios:**

```text
[F1G-RAWDB-INSERT-FULL]
[F1G-SNAPSHOT-RECONCILE]
[F1G-RAW-POSE-CHANGED]
```

Ejemplo:

```text
[F1G-RAWDB-INSERT-FULL] arrival_id=37 drone_id=1 epoch=0 new_kfs=0 updated_kfs=82 new_mps=24 updated_mps=6016 bad_kfs=0 bad_mps=12
```

Ejemplo de pose raw cambiada:

```text
[F1G-RAW-POSE-CHANGED] drone_id=1 epoch=0 kf=42 delta_t=0.37 delta_yaw=0.12 source=full_snapshot
```

---

### 4. Registrar snapshots en el journal guardable

**Intencion:** que los datasets guardados por 1C puedan reproducir tambien snapshots completos.

**Archivo probable:** `RawMapDatabase` y/o estructuras de record/replay del servidor.

**Cambio:** el journal debe distinguir eventos:

```text
type=delta
type=full_snapshot
```

Ambos deben conservar `arrival_id` y contenido suficiente para replay.

**Condicion de seguridad:** replay debe reinsertar eventos completos en el mismo orden de `arrival_id`.

**Logs obligatorios:**

```text
[F1G-JOURNAL-FULL-SNAPSHOT-APPEND]
[F1G-REPLAY-FULL-SNAPSHOT]
```

---

### 5. Reconciliacion con `GlobalPoseStore`

**Intencion:** si un full snapshot modifica poses locales de KFs ya conocidos, `GlobalPoseStore` debe actualizar su estado sin destruir poses globales optimizadas.

**Archivo probable:**

- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/src/multi_drone_system.cpp` si existe fachada.

**Cambio:** crear o completar un metodo equivalente a:

```cpp
PoseStoreReconcileResult ReconcileAfterRawPoseUpdates(
    const FullSnapshotIngestResult& ingest_result,
    const RawMapDatabase& raw_db);
```

Politica obligatoria:

```text
Si el KF no tiene pose global:
    no hacer nada.

Si el KF tiene pose global derivada solo de anchor:
    recalcular world_T_kf = world_T_local * local_T_kf_new.

Si el KF tiene pose optimizada por servidor:
    mantener optimized_world_T_kf.
    recalcular correction_T_kf = optimized_world_T_kf * inverse(raw_world_T_kf_new).

Si muchos KFs optimizados cambian de pose raw:
    marcar el submapa como raw_changed_after_server_optimization.
```

**Condicion de seguridad:** nunca borrar `optimized_world_T_kf` por el mero hecho de recibir un snapshot.

**Logs obligatorios:**

```text
[F1G-POSESTORE-REBASE-ANCHOR]
[F1G-POSESTORE-KEEP-OPTIMIZED]
[F1G-SNAPSHOT-AFFECTS-OPTIMIZED-KFS]
[F1G-POSESTORE-RECONCILE-SUMMARY]
```

Ejemplo para KF no optimizado:

```text
[F1G-POSESTORE-REBASE-ANCHOR] kf=41 reason=raw_pose_changed source=anchor_derived action=recompute_world_pose
```

Ejemplo para KF optimizado:

```text
[F1G-POSESTORE-KEEP-OPTIMIZED] kf=42 reason=server_optimized_pose_exists raw_pose_changed=true action=recompute_correction_keep_world_pose
```

---

### 6. Prueba debug de conflicto con KF optimizado

**Intencion:** como la optimizacion real aun no existe en 1G, debe haber un modo debug controlado para probar la politica de mantener poses optimizadas.

**Archivo probable:** `global_map_server.cpp`, `GlobalPoseStore` o test interno del backend.

**Cambio:** permitir, solo en modo debug de la subfase, marcar uno o varios KFs como optimizados antes de procesar un snapshot/replay.

Ejemplo conceptual:

```text
parametro debug_f1g_mark_first_anchor_kf_as_optimized=true
```

O metodo interno usado por el servidor durante la prueba:

```cpp
GlobalPoseStore::SetOptimizedKeyFramePose(kf_id, world_T_kf_debug, "F1G_DEBUG_OPTIMIZED_POSE");
```

**Condicion de seguridad:** este modo debug debe estar desactivado por defecto y documentado. No debe simular una optimizacion real ni usarse fuera de validacion de 1G.

**Logs obligatorios:**

```text
[F1G-DEBUG-OPTIMIZED-KF-SET]
```

---

### 7. Publicacion tras snapshot

**Intencion:** la nube publicada por 1F debe seguir existiendo y no duplicarse/desaparecer tras un full snapshot.

**Archivo probable:** `global_map_server.cpp` y `GlobalMapBuilder`.

**Cambio:** tras insertar snapshot y reconciliar poses, el servidor debe volver a construir/publicar la nube con el mecanismo de 1F.

**Condicion de seguridad:** la nube debe usar `GlobalPoseStore` para poses globales, no poses raw directamente.

**Logs obligatorios:**

```text
[F1G-GLOBALMAP-REBUILD-AFTER-SNAPSHOT]
```

---

## Cambios prohibidos

No hacer en 1G:

- no implementar optimizacion real;
- no detectar loops;
- no fusionar landmarks;
- no cambiar la politica de score salvo actualizaciones raw ya permitidas en 1F;
- no recalcular anchors por fiducial al recibir snapshot;
- no tratar full snapshot como una orden de mover el mapa global entero;
- no borrar poses optimizadas existentes;
- no modificar poses locales originales en `RawMapDatabase` fuera del contenido recibido de ORB-SLAM3;
- no dividir snapshots en KeyFrames artificiales durante replay;
- no redisenar mensajes ROS;
- no tocar `ORB_SLAM3`.

---

## Paquetes a compilar

Comando esperado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

Si falla el build:

```bash
./codex/herramientas/reduce_build_log.sh
```

`diagnosticador_build` debe leer:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

Si el error reducido no basta, puede consultar:

```text
codex/archivos_auxiliares/logs/colcon_build.log
```

---

## Pruebas Gazebo requeridas

### Prueba 1 — live snapshot tras anclaje basico

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. esperar arranque de Gazebo, wrappers y servidor;
2. mover dos drones al fiducial 2 con alturas distintas, como en pruebas clave anteriores;
3. esperar a que lleguen deltas y se cree al menos un submapa por dron;
4. permitir que 1E ancle al menos un submapa si el fiducial simulado esta disponible;
5. forzar o esperar una peticion de full snapshot por dron;
6. recibir snapshot completo;
7. insertar snapshot en `RawMapDatabase`;
8. reconciliar con `GlobalPoseStore`;
9. reconstruir/publicar nube global basica;
10. esperar 10-20 segundos y terminar.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observacion esperada en RViz2:

```text
La nube sparse global publicada en 1F no debe desaparecer ni duplicarse de forma evidente tras recibir full snapshot.
Si hay submapas anclados, deben seguir apareciendo cerca del fiducial.
```

La validacion principal sigue siendo por logs; RViz2 aporta observacion manual del usuario.

---

### Prueba 2 — replay con full snapshot y reconciliacion debug

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Esta prueba puede no mover drones si se ejecuta en modo replay. Debe usar un dataset guardado desde 1C/1F o generado en la Prueba 1 de 1G que contenga deltas y al menos un full snapshot.

Secuencia esperada:

1. arrancar servidor en modo replay;
2. cargar dataset desde `codex/archivos_auxiliares/`;
3. crear `RawMapDatabase` nueva vacia;
4. reinyectar eventos por `arrival_id`;
5. cuando llegue evento `full_snapshot`, llamar a `InsertFullSnapshot`;
6. activar modo debug que marque un KF como optimizado antes de procesar un snapshot, si hay datos suficientes;
7. comprobar que el snapshot no elimina la pose optimizada;
8. comprobar logs de `KEEP_OPTIMIZED` o, si no hubo KF optimizado debug, documentar que solo se valido la ruta anchor-derived.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si el modo replay no necesita Gazebo completo pero las herramientas del proyecto solo ejecutan simulacion mediante `run_simulation.sh`, se debe usar el launch minimo disponible que arranque servidor y clock simulado. Si no existe, usar el launch principal y documentar la limitacion.

Observacion esperada en RViz2:

```text
No obligatoria para Prueba 2. La validacion es por log.
```

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1G-FULL-SNAPSHOT|F1G-RAWDB|F1G-SNAPSHOT|F1G-RAW-POSE|F1G-POSESTORE|F1G-GLOBALMAP|F1F-GLOBALMAP|F1E-FID|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1G-REPLAY|F1G-FULL-SNAPSHOT|F1G-RAWDB|F1G-SNAPSHOT|F1G-DEBUG|F1G-POSESTORE|KEEP-OPTIMIZED|REBASE-ANCHOR|ERROR|FATAL|Segmentation fault|Killed
```

La reduccion genera:

```text
codex/archivos_auxiliares/logs/prueba_1.reduced.log
codex/archivos_auxiliares/logs/prueba_2.reduced.log
```

y conserva:

```text
codex/archivos_auxiliares/logs/prueba_1.log
codex/archivos_auxiliares/logs/prueba_2.log
```

Si el reducido no muestra marcadores suficientes, revisar el log completo antes de repetir Gazebo o concluir que el codigo no emitio el marcador.

---

## Marcadores tecnicos obligatorios

En al menos una prueba deben aparecer:

```text
[F1G-FULL-SNAPSHOT-CLIENT-READY]
[F1G-FULL-SNAPSHOT-REQUEST]
[F1G-FULL-SNAPSHOT-RX]
[F1G-FULL-SNAPSHOT-ARRIVAL]
[F1G-RAWDB-INSERT-FULL]
[F1G-SNAPSHOT-RECONCILE]
[F1G-POSESTORE-RECONCILE-SUMMARY]
```

Si el snapshot modifica poses raw, deben aparecer:

```text
[F1G-RAW-POSE-CHANGED]
```

Si no modifica poses raw, debe aparecer un resumen que lo diga explicitamente:

```text
[F1G-POSESTORE-RECONCILE-SUMMARY] raw_pose_changed=0
```

Si existe KF optimizado debug o real:

```text
[F1G-POSESTORE-KEEP-OPTIMIZED]
```

Si no existe KF optimizado en los datos, debe documentarse en historial y la subfase puede ser `CONSEGUIDA` si la ruta anchor-derived y snapshot completo quedan validadas. La prueba de `KEEP_OPTIMIZED` puede quedar como `PARCIAL` solo si el modo debug estaba requerido y no se ejecuto.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. la Prueba 1 se ejecuta;
3. `scenario_runner_node` se ejecuta en la prueba que lo use;
4. los goals requeridos terminan con `success=true`, salvo que la prueba sea replay sin trayectoria y este justificado;
5. el servidor crea clientes de full snapshot por dron;
6. el servidor solicita al menos un full snapshot;
7. el servidor recibe al menos un full snapshot;
8. cada snapshot recibido recibe un `arrival_id`;
9. `RawMapDatabase::InsertFullSnapshot` inserta/reconcilia sin duplicar masivamente KFs/MPs;
10. el resultado de snapshot identifica KFs/MPs nuevos/actualizados/bad;
11. si se detectan cambios de pose raw, aparecen logs `F1G-RAW-POSE-CHANGED`;
12. `GlobalPoseStore` recibe el resultado de reconciliacion;
13. KFs con pose global derivada de anchor se recalculan si cambia su pose raw;
14. KFs con pose optimizada, si existen o se simulan por debug, conservan `optimized_world_T_kf` y recalculan correccion;
15. la nube global basica de 1F no desaparece ni se duplica de forma evidente tras snapshot;
16. no aparecen errores graves no explicados;
17. el historial documenta resultado, evidencia positiva, evidencia negativa y conclusion.

---

## Criterio de fallo o parcial

Marcar `NO CONSEGUIDA` si:

- no compila;
- el servidor nuevo no arranca;
- no se crean clientes de full snapshot;
- no se puede solicitar snapshot a ningun wrapper;
- se recibe snapshot pero no se inserta en `RawMapDatabase`;
- falta `F1G-RAWDB-INSERT-FULL`;
- el snapshot pisa o borra indebidamente poses globales optimizadas conocidas;
- la nube publicada desaparece por error tras snapshot;
- aparece `Segmentation fault`, `FATAL`, `Killed` o error grave no explicado.

Marcar `PARCIAL` si:

- compila;
- se recibe snapshot y se inserta;
- pero no se pudo validar la reconciliacion con `GlobalPoseStore`;
- o no hubo datos suficientes para probar `KEEP_OPTIMIZED` y tampoco se ejecuto el modo debug previsto;
- o la prueba live funciona pero el replay con snapshots no queda validado.

Marcar `BLOQUEADA` solo si:

- el servicio `orbslam/get_full_map` no existe o no responde en el wrapper actual y no puede resolverse sin tocar subfases previas;
- el formato de datos guardado en 1C no contiene informacion suficiente para replay con snapshots y rehacerlo requiere decision del usuario;
- una dependencia externa impide ejecutar simulacion o servicio.

---

## Documentacion a actualizar

Actualizar siempre:

- historial de Fase 1/subfase 1G;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
- `codex/contexto/paquetes/orbslam3_server/launches.md` si se cambian parametros/launch;
- `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
- `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`.

La documentacion debe explicar:

```text
- que full snapshots actualizan RawMapDatabase;
- que los snapshots reciben arrival_id;
- que delta_journal/snapshot_journal se reproducen por arrival_id;
- que RawMapDatabase tiene autoridad sobre datos ORB locales;
- que GlobalPoseStore tiene autoridad sobre poses en world;
- que las poses optimizadas no se borran por snapshot;
- que si ORB-SLAM3 cambia una pose raw de un KF optimizado, se recalcula correction_T_kf;
- que si muchos KFs optimizados cambian, se marca el submapa para revision futura;
- que 1G no implementa optimizacion ni loops.
```

Si se crea un archivo nuevo especifico para resultados de ingest o pose updates, crear o actualizar su `.md` correspondiente en `codex/contexto/paquetes/orbslam3_multi/`.

No marcar esta subfase como `realizado` si el criterio de exito no se cumple.

---

## Restricciones tecnicas permanentes reforzadas por esta subfase

1. `RawMapDatabase` no guarda poses globales corregidas.
2. `GlobalPoseStore` no guarda descriptores, BoW ni MapPoints completos.
3. Un full snapshot no es una optimizacion.
4. Un full snapshot no debe mover el mapa global directamente.
5. ORB-SLAM3 puede cambiar poses locales antiguas; esos cambios son raw y deben registrarse.
6. El servidor no debe perder poses globales optimizadas por aceptar ciegamente un snapshot local.
7. Si hay conflicto entre raw local y pose global optimizada, se loggea y se conserva la autoridad de `GlobalPoseStore` sobre `world`.

---

## Resultado de ejecucion 2026-07-08

Conclusion:

```text
CONSEGUIDA
```

Evidencia principal:

- build separado de `orbslam3_multi` y `orbslam3_server` con `BUILD-EXIT-CODE 0`;
- prueba live `prueba_1` con `SIM-DONE prueba=1 success=true`;
- `scenario_runner_node` envio 5 goals y los 5 terminaron con `success=true`;
- el servidor creo clientes de full snapshot para `/dron_1/orbslam/get_full_map` y `/dron_2/orbslam/get_full_map`;
- `prueba_1.reduced.log` contiene 8 `[F1G-FULL-SNAPSHOT-RX]`, 8 `[F1G-RAWDB-INSERT-FULL]` y 8 `[F1G-POSESTORE-RECONCILE-SUMMARY]`;
- en live se observaron snapshots con insercion y actualizacion de KFs/MPs, por ejemplo `new_kfs=3 updated_kfs=24 raw_pose_changed=23` y `new_kfs=0 updated_kfs=27 raw_pose_changed=26`;
- la reconciliacion recalculo KFs derivados de anchor (`F1G-POSESTORE-REBASE-ANCHOR`) y preservo una pose optimizada debug (`F1G-POSESTORE-KEEP-OPTIMIZED`);
- el nuevo `rawdb_prueba_1.record` contiene `368` entradas, `356` deltas, `12` full snapshots, `74` observaciones fiduciales, `4` submapas, `225` KFs y `26165` MPs;
- replay `prueba_2` con `SIM-DONE prueba=2 success=true`;
- `prueba_2.reduced.log` contiene 12 `[F1G-REPLAY-FULL-SNAPSHOT]`, 12 `[F1G-RAWDB-INSERT-FULL]` y 12 `[F1G-POSESTORE-RECONCILE-SUMMARY]`;
- `[F1C-REPLAY-DONE] entries=368 journal=368 deltas=356 full=12 fiducial_observations=74 submaps=4 kfs=225 mps=26165`;
- no aparecieron `FATAL`, `Segmentation fault` ni `Killed`; el unico `ERROR` relevante fue el cierre de Gazebo con exit code `255` despues de `SIM-DONE`, patron no bloqueante ya conocido.

Notas:

- se borro el antiguo `prueba_diff_anclaje.record` solicitado por el usuario;
- se descarto una grabacion intermedia que contenia un snapshot vacio `drone_id=0`; el servidor ahora ignora snapshots vacios/no inicializados antes de asignarles estado util;
- `1G` no implementa optimizacion real, loops ni fused landmarks.
