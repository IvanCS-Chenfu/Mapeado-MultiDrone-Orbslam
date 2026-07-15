# Subfase 1D — `GlobalPoseStore` ligero para poses globales de KeyFrames

## Estado

```text
realizado
```

---

Realizada el 2026-07-08 con conclusión `CONSEGUIDA`.

Evidencia principal:

- `orbslam3_multi` y `orbslam3_server` compilaron con `BUILD-EXIT-CODE 0`;
- replay `prueba_1` sin Gazebo con `SIM-DONE prueba=1 success=true`;
- `GlobalPoseStore` empezó vacío:
  ```text
  [F1D-POSESTORE-INIT] anchors=0 world_poses=0 optimized_kfs=0 corrections=0
  ```
- anchor sintético debug:
  ```text
  [F1D-POSESTORE-ANCHOR-SUMMARY]
  ```
- pose optimizada sintética y corrección:
  ```text
  [F1D-POSESTORE-OPT-POSE-SET]
  [F1D-POSESTORE-CORRECTION-SET]
  ```
- KFs nuevos posteriores heredaron la última corrección del mismo submapa:
  ```text
  [F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]
  ```
- replay completo:
  ```text
  [F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884
  ```

El anchor de `1D` es sintético/debug. El primer anchor real queda para `1E`.

## Objetivo tecnico

Crear en `orbslam3_multi` una clase ligera llamada `GlobalPoseStore` encargada de guardar y consultar la pose global actual de los `KeyFrames` que ya pertenecen a submapas anclados o corregidos.

Esta subfase separa definitivamente dos capas:

```text
RawMapDatabase
    Datos crudos de ORB-SLAM3.
    Poses locales originales.
    KeyFrames, MapPoints, BoW, observaciones, covisibilidad, etc.

GlobalPoseStore
    Estado global ligero.
    Anchors de submapas.
    Pose world asociada a cada KeyFrame anclado/corregido.
    Correcciones simples heredables tras optimización.
```

`GlobalPoseStore` debe empezar vacía aunque `RawMapDatabase` ya tenga datos. Solo debe poblarse cuando una clase externa, ahora en modo prueba/controlado y en subfases posteriores mediante fiduciales u optimización, invoque métodos de anclaje o corrección.

La subfase debe ser verificable mediante build y logs. No se exige mapa global visible en RViz2 en esta subfase, aunque se permite publicar marcadores debug de poses si el servidor nuevo ya tiene infraestructura simple para ello.

---

## Contexto obligatorio a leer

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1A.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1B.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1C.md`
- historial reciente de Fase 1
- documentación de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/orbslam3_server/`
- ADRs relacionados con:
  - identidad `(drone_id, map_epoch)`;
  - separación servidor/backend;
  - fiduciales como observaciones absolutas;
  - no sobrescribir poses locales raw;
  - optimización local y futura propagación de correcciones.

---

## Diagnostico de partida

Tras la subfase 1C debe existir una `RawMapDatabase` en `orbslam3_multi` que reciba del servidor los datos crudos enviados por los wrappers ORB-SLAM3.

Esa base raw debe guardar información por:

```text
submapa = (drone_id, map_epoch)
```

pero todavía no debe existir una capa clara, ligera y separada que responda:

```text
¿Dónde está este KeyFrame en world según el sistema global actual?
```

El problema que se prepara para resolver, aunque no se optimiza todavía en esta subfase, es el siguiente:

1. Un submapa puede anclarse con una transformación rígida `world_T_local`.
2. Esa transformación permite calcular una pose global inicial para todos sus KFs:

   ```text
   world_T_kf = world_T_local * local_T_kf
   ```

3. Más adelante, una optimización podrá mover algunos KFs concretos.
4. Si luego llegan KFs nuevos del mismo submapa, no deben caer siempre en la trayectoria rígida antigua si ya existe una corrección optimizada reciente.
5. Por ello, `GlobalPoseStore` debe guardar anchors, poses globales y correcciones simples heredables para KFs futuros.

En esta subfase no se implementa la optimización real, pero sí se deja preparada la estructura para que otras clases de `orbslam3_multi` puedan:

- anclar submapas;
- registrar poses optimizadas;
- consultar poses globales actuales;
- aplicar una corrección simple a KFs nuevos de un submapa ya corregido.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Crear o modificar, según exista la estructura real del paquete:

- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/multi_drone_system.hpp`
- `orbslam3_multi/src/multi_drone_system.cpp`
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_multi/package.xml` solo si fuese estrictamente necesario

`raw_map_database.*` solo puede tocarse para añadir consultas mínimas que `GlobalPoseStore` necesite, por ejemplo:

- obtener IDs de KFs de un submapa;
- obtener pose local de un KF;
- obtener submapa de un KF;
- obtener parent/covisibilidad si ya existe y se usa para elegir corrección heredada.

### `orbslam3_server`

Modificar solo para integrar y probar `GlobalPoseStore` desde el servidor nuevo:

- `orbslam3_server/src/global_map_server.cpp`
- launch activo nuevo del servidor, si hace falta añadir parámetros debug/replay
- `orbslam3_server/CMakeLists.txt` solo si fuese necesario

### Archivos auxiliares y pruebas

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- dataset raw/replay generado en 1C, si existe:

  ```text
  codex/archivos_auxiliares/repeticiones/rawdb_record_*.json
  codex/archivos_auxiliares/repeticiones/rawdb_record_*.bin
  ```

- documentación relacionada en `codex/contexto/paquetes/`
- historial de la fase/subfase

---

## Archivos prohibidos

No modificar en esta subfase:

- `ORB_SLAM3/`
- `orbslam3_ros2/` wrapper
- `orbslam3_msgs/` mensajes y servicios
- lógica de detección real de fiduciales
- lógica real de loops/subnubes/RANSAC
- fusión de landmarks
- solver/optimización real
- publicación final de mapa global fused
- archivos dentro de `orbslam3_multi/legacy/` salvo para documentación o consulta
- `global_map_server_antiguo.cpp` salvo que se lea como referencia

No modificar manualmente:

- `build/`
- `install/`
- `log/`

---

## Funciones, clases o nodos que hay que localizar

`planificador_fase` debe localizar antes de implementar:

- clase nueva o punto de integración existente:

  ```text
  GlobalPoseStore
  ```

- base raw creada en 1C:

  ```text
  RawMapDatabase
  ```

- fachada/backend si existe:

  ```text
  MultiDroneSystem
  ```

- servidor nuevo creado en 1B:

  ```text
  global_map_server.cpp
  ```

- callback o función donde el servidor recibe `OrbMap` delta/snapshot y llama a `RawMapDatabase`.
- mecanismo de replay creado en 1C, si existe.
- tipos de identificador ya definidos para:

  ```text
  drone_id
  map_epoch
  SubmapId
  GlobalKeyFrameId
  GlobalMapPointId
  ```

No inventar nombres si ya existen tipos equivalentes. Reutilizar los tipos existentes en `orbslam3_multi` siempre que no rompa claridad ni build.

---

## Cambios requeridos

### 1. Crear `GlobalPoseStore` como clase ligera

Intención:

Crear una clase en `orbslam3_multi` que guarde únicamente estado global ligero.

Debe guardar como mínimo:

```text
submap_anchors:
    submap_id -> world_T_local, source, metadata minima

keyframe_world_poses:
    kf_id -> world_T_kf, source

optimized_keyframes:
    kf_id -> optimized_world_T_kf, correction_T_kf, source

submap_last_correction:
    submap_id -> correction_T_latest
```

Condición de seguridad:

No guardar en `GlobalPoseStore` datos grandes como descriptores, BoW, keypoints, MapPoints completos u observaciones. Si necesita esa información, debe consultarla en `RawMapDatabase`.

Logs requeridos:

```text
[F1D-POSESTORE-INIT]
[F1D-POSESTORE-STATS]
```

---

### 2. Añadir método para anclar un submapa

Método esperado, ajustando nombres a los tipos existentes:

```cpp
SetSubmapAnchor(submap_id, world_T_local, source)
```

o método equivalente:

```cpp
AnchorSubmap(submap_id, world_T_local, raw_db, source)
```

Comportamiento:

1. Recibir un `submap_id` y una transformación `world_T_local`.
2. Consultar a `RawMapDatabase` los IDs de todos los KFs existentes en ese submapa.
3. Para cada KF:

   ```text
   world_T_kf = world_T_local * local_T_kf
   ```

4. Guardar `world_T_kf` asociada al ID global del KF.
5. Marcar el submapa como anclado.

Condición de seguridad:

No escribir ni modificar la pose local original guardada en `RawMapDatabase`.

Logs requeridos:

```text
[F1D-POSESTORE-ANCHOR-REQUEST]
[F1D-POSESTORE-ANCHOR-SET]
[F1D-POSESTORE-ANCHOR-KF-POSE]
[F1D-POSESTORE-ANCHOR-SUMMARY]
```

Ejemplo de log:

```text
[F1D-POSESTORE-ANCHOR-SUMMARY] drone_id=1 epoch=0 source=DEBUG_TEST anchored_kfs=42 world_t=(2.000,0.000,0.000) yaw=1.570
```

---

### 3. Añadir consultas de pose global

Métodos esperados:

```cpp
HasSubmapAnchor(submap_id) const
HasWorldPose(kf_id) const
GetWorldPose(kf_id) const
GetPoseStoreStats() const
GetSubmapPoseStats(submap_id) const
```

Comportamiento:

- `HasWorldPose(kf_id)` debe ser `true` solo si el KF ya tiene pose global registrada.
- KFs de submapas no anclados no deben aparecer como publicables.
- `GetWorldPose(kf_id)` no debe inventar una pose global si no hay anchor o pose optimizada.

Logs requeridos:

```text
[F1D-POSESTORE-GET-POSE]
[F1D-POSESTORE-MISSING-POSE]
```

Los logs de consulta deben estar limitados o throttled para no saturar la simulación.

---

### 4. Registrar KFs nuevos de un submapa ya anclado

Método esperado:

```cpp
OnNewKeyFrameInserted(kf_id, raw_db)
```

o equivalente:

```cpp
RegisterNewKeyFrameIfAnchored(kf_id, raw_db)
```

Comportamiento:

1. Consultar en `RawMapDatabase` a qué submapa pertenece el KF.
2. Si el submapa no está anclado:

   ```text
   no crear pose global
   ```

3. Si el submapa está anclado y no hay corrección activa:

   ```text
   world_T_kf = world_T_local * local_T_kf
   ```

4. Si el submapa está anclado y tiene una corrección reciente por optimización:

   ```text
   raw_world_T_kf = world_T_local * local_T_kf
   world_T_kf = correction_T_latest * raw_world_T_kf
   ```

Esta política es intencionadamente simple. En subfases futuras podrá reemplazarse por parent/covisibilidad/interpolación.

Condición de seguridad:

La corrección heredada solo debe aplicarse dentro del mismo submapa.

Logs requeridos:

```text
[F1D-POSESTORE-NEW-KF-UNANCHORED]
[F1D-POSESTORE-NEW-KF-POSE-SET]
[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]
```

Ejemplo:

```text
[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT] drone_id=1 epoch=0 kf=57 correction_source=LAST_SUBMAP_CORRECTION dx=0.420 dy=-0.030 dyaw=0.080
```

---

### 5. Registrar poses optimizadas sin ejecutar optimización real

Método esperado:

```cpp
SetOptimizedKeyFramePose(kf_id, optimized_world_T_kf, raw_db, source)
```

Comportamiento:

1. Recibir la pose final optimizada en `world` de un KF.
2. Consultar `RawMapDatabase` para obtener:

   ```text
   local_T_kf
   submap_id
   ```

3. Obtener `world_T_local` del submapa.
4. Calcular la pose rígida base:

   ```text
   raw_world_T_kf = world_T_local * local_T_kf
   ```

5. Calcular la corrección:

   ```text
   correction_T_kf = optimized_world_T_kf * inverse(raw_world_T_kf)
   ```

6. Guardar:

   ```text
   optimized_world_T_kf
   correction_T_kf
   source
   ```

7. Actualizar una corrección simple heredable para el submapa:

   ```text
   submap_last_correction[submap_id] = correction_T_kf
   ```

Condición de seguridad:

En esta subfase la optimización real no existe. Este método se prueba con una corrección sintética/debug para comprobar que la estructura funciona.

Logs requeridos:

```text
[F1D-POSESTORE-OPT-POSE-SET]
[F1D-POSESTORE-CORRECTION-SET]
[F1D-POSESTORE-CORRECTION-STATS]
```

---

### 6. Integrar `GlobalPoseStore` con el servidor nuevo solo para prueba controlada

El servidor nuevo debe poder activar un modo debug de 1D para validar la clase con datos reales/replay de 1C.

Parámetros orientativos, adaptables a la estructura real del launch:

```text
pose_store_debug_enabled
pose_store_debug_anchor_after_deltas
pose_store_debug_anchor_drone_id
pose_store_debug_anchor_epoch
pose_store_debug_anchor_world_x
pose_store_debug_anchor_world_y
pose_store_debug_anchor_world_z
pose_store_debug_anchor_yaw
pose_store_debug_opt_enabled
pose_store_debug_opt_after_deltas
pose_store_debug_opt_kf_id
pose_store_debug_opt_dx
pose_store_debug_opt_dy
pose_store_debug_opt_dz
pose_store_debug_opt_dyaw
```

Si existen mecanismos mejores en el servidor nuevo, pueden usarse, pero deben quedar documentados.

Flujo de prueba esperado:

1. Cargar o recibir deltas en `RawMapDatabase`.
2. Tras cierto número de deltas o cuando exista al menos un submapa con KFs, invocar un anchor sintético:

   ```text
   SetSubmapAnchor(..., source=DEBUG_TEST)
   ```

3. Comprobar que se crean poses globales para los KFs existentes.
4. Aplicar una pose optimizada sintética a uno o varios KFs:

   ```text
   SetOptimizedKeyFramePose(..., source=DEBUG_TEST_OPT)
   ```

5. Insertar más deltas por replay.
6. Comprobar que los KFs nuevos del mismo submapa heredan la última corrección simple.

Logs requeridos del servidor:

```text
[F1D-SERVER-DEBUG-ANCHOR]
[F1D-SERVER-DEBUG-OPT]
[F1D-SERVER-POSESTORE-STATS]
```

---

### 7. Documentar la limitación de la corrección heredada simple

Debe quedar escrito en documentación e historial:

```text
La política de 1D para KFs futuros tras optimización es deliberadamente simple:
se aplica la última corrección conocida del mismo submapa.

En subfases posteriores podrá sustituirse por una estrategia más robusta:
- parent de ORB-SLAM3;
- KF covisible corregido más fuerte;
- nearest optimized KF;
- interpolación entre correcciones;
- deformation graph.
```

---

## Cambios prohibidos

No hacer en esta subfase:

- no detectar fiduciales reales;
- no usar ground truth para construir mapa;
- no calcular `world_T_local` desde fiducial real;
- no implementar `FiducialAnchorManager`;
- no implementar loops por BoW/subnube;
- no implementar fusión de landmarks;
- no ejecutar ni migrar el solver real de optimización;
- no publicar mapa global fused final;
- no modificar la semántica raw de `RawMapDatabase`;
- no sobrescribir poses locales originales de ORB-SLAM3;
- no renombrar ni limpiar legacy masivamente;
- no dividir deltas de replay por KeyFrame si 1C definió replay por delta completo;
- no mover lógica al servidor que deba vivir en `orbslam3_multi`, salvo parámetros/debug de integración.

---

## Paquetes a compilar

Comando esperado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
```

Si cambios reales en dependencias obligan a compilar más paquetes, `implementador_fase` puede añadirlos justificándolo en el historial.

Si falla el build:

```bash
./codex/herramientas/reduce_build_log.sh
```

Después, diagnosticar el primer error real y corregir lo mínimo.

---

## Pruebas requeridas

### Prueba 1 — Replay de RawMapDatabase con anchor y corrección sintéticos

Esta es la prueba principal de 1D.

Debe usar un dataset guardado en 1C. Si no existe ningún dataset compatible, Codex debe generar uno ejecutando una prueba corta de 1C antes de esta prueba, documentándolo en historial.

Dataset esperado, nombre orientativo:

```text
codex/archivos_auxiliares/repeticiones/rawdb_record_prueba_simple.json
```

o equivalente real generado por 1C.

Flujo esperado:

1. arrancar el servidor nuevo en modo replay 1C;
2. cargar el dataset raw con deltas ordenados por `arrival_id`;
3. crear una `RawMapDatabase` nueva vacía;
4. reinsertar deltas con timer;
5. tras varios deltas, ejecutar un anchor sintético sobre un submapa existente;
6. comprobar que `GlobalPoseStore` crea poses para KFs existentes del submapa;
7. aplicar una corrección/pose optimizada sintética sobre un KF;
8. continuar el replay;
9. comprobar que KFs nuevos del mismo submapa reciben pose global heredando la corrección simple;
10. terminar cuando se reproduzcan suficientes deltas para validar estadísticas.

Comando orientativo si el replay se integra en el launch normal:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si el modo replay no necesita Gazebo, la subfase debe documentar el comando exacto usado para ejecutar el nodo/launch de replay y conservar logs en:

```text
codex/archivos_auxiliares/logs/prueba_1.log
codex/archivos_auxiliares/logs/prueba_1.reduced.log
```

Observación esperada en RViz2:

```text
No obligatoria en 1D.
Opcionalmente, si se implementan markers debug, visualizar poses globales de KFs anclados/corregidos.
```

### Prueba 2 — Simulación corta de compatibilidad, solo si no existe dataset 1C

Esta prueba solo es necesaria si no hay dataset raw/replay generado en 1C.

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Secuencia mínima:

1. esperar arranque;
2. mover ambos drones hacia el fiducial 2 a alturas distintas;
3. mover un dron ligeramente en X;
4. mover el segundo dron de forma similar;
5. esperar unos segundos;
6. terminar.

El objetivo de esta prueba no es validar fiduciales, sino generar datos raw suficientes para que la Prueba 1 pueda ejecutarse.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1D-SERVER|F1D-POSESTORE|F1D-POSESTORE-ANCHOR|F1D-POSESTORE-OPT|F1D-POSESTORE-CORRECTION|F1D-POSESTORE-NEW-KF|F1D-POSESTORE-STATS|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1B-ORBMAP-RX|F1C-RAWDB|F1C-RAWDB-SAVE|F1D-POSESTORE|ERROR|FATAL|Segmentation fault|Killed
```

La reducción genera `prueba_X.reduced.log` y conserva `prueba_X.log` completo.

Si el reducido no muestra datos suficientes, revisar el log completo antes de repetir Gazebo o concluir que el código no emitió el marcador.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. `GlobalPoseStore` existe en `orbslam3_multi` y compila;
3. el servidor nuevo puede crear/usarla sin depender del servidor antiguo;
4. `GlobalPoseStore` empieza vacía;
5. al anclar un submapa sintético/debug:
   - se consulta `RawMapDatabase`;
   - se obtienen los KFs del submapa;
   - se calcula y guarda una pose `world_T_kf` para cada KF existente;
6. `RawMapDatabase` no sobrescribe poses locales originales;
7. al insertar KFs nuevos de un submapa anclado:
   - reciben pose global;
8. al registrar una pose optimizada sintética:
   - se guarda la pose optimizada;
   - se calcula una corrección;
   - se actualiza la corrección simple del submapa;
9. al insertar KFs nuevos posteriores a esa corrección:
   - heredan la última corrección del mismo submapa;
10. aparecen los marcadores obligatorios:

    ```text
    F1D-POSESTORE-ANCHOR-SUMMARY
    F1D-POSESTORE-OPT-POSE-SET
    F1D-POSESTORE-CORRECTION-SET
    F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT
    F1D-POSESTORE-STATS
    ```

11. no aparecen errores graves no explicados;
12. el historial queda actualizado.

No se exige que RViz2 muestre nube global en esta subfase.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- `GlobalPoseStore` no se integra en `orbslam3_multi`;
- no se puede anclar un submapa usando datos de `RawMapDatabase`;
- no se generan poses globales para KFs existentes del submapa anclado;
- se modifican poses raw en `RawMapDatabase`;
- faltan marcadores obligatorios;
- aparece un error grave no explicado;
- se implementan fiduciales/loops/optimización real en esta subfase rompiendo el alcance.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- `GlobalPoseStore` existe;
- el anclaje sintético crea poses;
- pero falla la corrección heredada para KFs nuevos o falta alguna estadística/log no crítica.

La subfase debe marcarse como `BLOQUEADA` solo si:

- no existe dataset replay 1C y Gazebo no puede ejecutarse para generarlo;
- la API real de `RawMapDatabase` no permite consultar KFs/poses locales y no puede ampliarse mínimamente;
- hay una dependencia externa no resuelta que impide validar la integración.

---

## Documentacion a actualizar

Actualizar obligatoriamente:

- historial de Fase 1/subfase 1D;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- crear o actualizar:

  ```text
  codex/contexto/paquetes/orbslam3_multi/global_pose_store.md
  ```

- documentación de `RawMapDatabase` si se añaden consultas nuevas:

  ```text
  codex/contexto/paquetes/orbslam3_multi/raw_map_database.md
  ```

- documentación de `orbslam3_server` si se añaden parámetros o modo debug/replay:

  ```text
  codex/contexto/paquetes/orbslam3_server/global_map_server.md
  codex/contexto/paquetes/orbslam3_server/launches.md
  ```

La documentación debe explicar claramente:

```text
GlobalPoseStore es una capa ligera.
No guarda datos ORB grandes.
Consulta RawMapDatabase cuando necesita KFs o poses locales.
Empieza vacía.
Se llena al anclar submapas o registrar poses optimizadas.
La política de corrección heredada de 1D es simple y temporal.
```

También debe quedar escrito que:

```text
El problema de KFs nuevos tras optimización se ha considerado desde 1D,
pero la política robusta basada en parent/covisibilidad/interpolación queda para subfases posteriores.
```

No marcar la subfase como `realizado` si no se cumple el criterio de éxito.
