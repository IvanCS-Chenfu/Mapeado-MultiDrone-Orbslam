# 04 — Topics, servicios y acciones

## Aviso

Este documento resume las conexiones conocidas. Si Codex modifica launches, namespaces o nombres de nodos/topics, debe actualizarlo.

Los nombres pueden ser relativos al namespace de cada dron. Revisar siempre los launch actuales antes de hacer cambios importantes.

## Convenciones de namespace

Los drones suelen usar namespaces tipo:

```text
/dron_1
/dron_2
...
```

La acción de trayectoria por dron sigue el patrón:

```text
/dron_X/AccionTrayectoria
```

## Wrapper ORB-SLAM3 por dron

### Entradas

| Topic relativo | Tipo | Descripción |
|---|---|---|
| `camera/left` | `sensor_msgs/msg/Image` | Imagen izquierda estéreo, normalmente remapeada desde la cámara izquierda del dron. |
| `camera/right` | `sensor_msgs/msg/Image` | Imagen derecha estéreo, normalmente remapeada desde la cámara derecha del dron. |

Ejemplo conceptual con namespace:

```text
/dron_1/camera/left
/dron_1/camera/right
```

En la simulación real estos topics pueden estar remapeados desde nombres tipo sensores Gazebo.

### Salidas

| Topic relativo | Tipo | Descripción |
|---|---|---|
| `orbslam/pose_local` | `geometry_msgs/msg/PoseStamped` | Pose local estimada por ORB-SLAM3. |
| `orbslam/orb_map_delta` | `orbslam3_msgs/msg/OrbMap` | Deltas del mapa ORB-SLAM3 local. |

Ejemplo:

```text
/dron_1/orbslam/pose_local
/dron_1/orbslam/orb_map_delta
```

### Servicio

| Servicio relativo | Tipo | Descripción |
|---|---|---|
| `orbslam/get_full_map` | `orbslam3_msgs/srv/GetOrbMap` | Snapshot completo del mapa local. |

Ejemplo:

```text
/dron_1/orbslam/get_full_map
```

## Servidor global

Estado activo desde `1G`: el servidor `global_orb_map_server` se suscribe a `OrbMap` delta, pide full snapshots mediante `GetOrbMap`, emite logs `[F1B-*]`, inserta en `RawMapDatabase`, guarda rawdb, puede reproducir datasets con logs `[F1C-*]`, procesa fiduciales simulados/persistidos aplicando `body_T_camera` antes de anclar, reconcilia snapshots con `GlobalPoseStore` y publica la nube sparse global en `/global_sparse_cloud`.

### Entradas principales

| Entrada | Tipo | Descripción |
|---|---|---|
| `/dron_X/orbslam/orb_map_delta` | `orbslam3_msgs/msg/OrbMap` | Deltas de mapa local por dron. |
| `/dron_X/orbslam/pose_local` | `geometry_msgs/msg/PoseStamped` | No consumido por el servidor mínimo de `1C`; reservado para fases posteriores. |
| `/dron_X/...GT...` | variable | GT de Gazebo consumido solo para fiducial simulado/debug; no se usa para loops, mapa global ni pose final. |

### Servicios usados

| Servicio | Tipo | Descripción |
|---|---|---|
| `/dron_X/orbslam/get_full_map` | `orbslam3_msgs/srv/GetOrbMap` | Usado desde `1G` por el servidor para pedir full snapshots, asignar `arrival_id`, guardar journal y reconciliar KFs/MPs. |

### Salidas principales

| Salida | Tipo | Descripción |
|---|---|---|
| `/global_sparse_cloud` | `sensor_msgs/msg/PointCloud2` | Nube sparse global publicada desde `1F`, frame `world`, con campos `x`, `y`, `z`, `score`, `drone_id` y `map_epoch`; los anchors fiduciales se calculan con `world_T_camera = world_T_body * body_T_camera`. |
| `/dron_X/map_correction` | `orbslam3_msgs/msg/MapCorrection` | Corrección mundo-local para el dron/submapa. |
| `/dron_X/corrected_keyframes` | `orbslam3_msgs/msg/CorrectedKeyFrameArray` | KeyFrames corregidos por optimización global/local. |
| estado fiducial | `orbslam3_msgs/msg/FiducialLockStatus` | Estado de fiduciales/anclaje. |

## Corrector de pose global

El ejecutable `global_pose_corrector` sigue existiendo, pero el launch activo de `1C` no lo lanza. Se reintroducirá cuando el servidor nuevo vuelva a publicar `MapCorrection` o `CorrectedKeyFrameArray`.

### Entradas

| Topic | Tipo | Descripción |
|---|---|---|
| `/dron_X/orbslam/pose_local` | `geometry_msgs/msg/PoseStamped` | Pose local de ORB-SLAM3. |
| `/dron_X/map_correction` | `orbslam3_msgs/msg/MapCorrection` | Transformación/corrección desde servidor. |
| `/dron_X/corrected_keyframes` | `orbslam3_msgs/msg/CorrectedKeyFrameArray` | KeyFrames corregidos. |

### Salidas

Publica poses globales corregidas de cámara/cuerpo del dron. Los nombres exactos deben comprobarse en código/launch.

## Acción de movimiento de drones

La acción está definida en `dron_individual`:

```text
dron_individual/action/TrayAction
```

Nombre de acción usado por la GUI y por el nodo automático:

```text
AccionTrayectoria
```

Con namespace:

```text
/dron_1/AccionTrayectoria
/dron_2/AccionTrayectoria
```

### Goal

```text
uint8 tipo_trayectoria
geometry_msgs/PoseStamped target_pose
float32 tx
float32 ty
float32 tz
float32 tyaw
bool absoluto_x
bool absoluto_y
bool absoluto_z
bool absoluto_yaw
```

### Result

```text
bool success
float32 t_total
```

### Feedback

```text
float32 t_act
std_msgs/Float64MultiArray x
std_msgs/Float64MultiArray y
std_msgs/Float64MultiArray z
std_msgs/Float64MultiArray yaw
```

## Nodo automático de escenarios

El nodo automático `scenario_runner_node` de `simulacion_dron` lee YAMLs tipo:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

y envía goals a `/dron_X/AccionTrayectoria`, esperando `success=true` antes de continuar según el modo del YAML.

En `1C`, `tray_prueba_4.yaml` se usa también con el mismo nodo para mantener vivo el servidor durante el replay; no envía movimiento real, solo espera.

## Logs útiles de movimiento automático

El nodo de escenario debe emitir logs con prefijo:

```text
SCENARIO-RUNNER
```

para poder filtrarlos en `reduce_simulation_log.sh`.

## Acción de trayectoria usada por pruebas automáticas

Cada dron expone la acción:

```text
/<dron_namespace>/AccionTrayectoria
```

Ejemplos:

```text
/dron_1/AccionTrayectoria
/dron_2/AccionTrayectoria
```

Tipo de acción:

```text
dron_individual/action/TrayAction
```

La GUI de control y el nodo automático scenario_runner_node deben usar esta misma acción.

El launch principal de simulación es:

```txt
ros2 launch simulacion_dron multi_dron.launch.py
```
