# `global_pose_corrector.cpp`

## Rol

Nodo por dron que transforma la pose local ORB-SLAM3 (`orbslam/pose_local`) en una pose global corregida usando información publicada por `global_map_server`.

## Nodo

`global_pose_corrector`

Normalmente se lanza dentro del namespace de cada dron:

```text
/dron_1/global_pose_corrector
/dron_2/global_pose_corrector
```

## Entradas

| Topic relativo | Tipo | Uso |
|---|---|---|
| `orbslam/pose_local` | `geometry_msgs/msg/PoseStamped` | Pose local ORB actual |
| `map_correction` | `orbslam3_msgs/msg/MapCorrection` | Transformación rígida `world_T_local_map` |
| `corrected_keyframes` | `orbslam3_msgs/msg/CorrectedKeyFrameArray` | Poses corregidas de KFs |
| `sensor/GT/pose` | `geometry_msgs/msg/PoseStamped` | Solo diagnóstico si `enable_gt_pose_diagnostics=true` |

## Salidas

| Topic relativo | Tipo | Descripción |
|---|---|---|
| `pose_global_corrected` | `geometry_msgs/msg/PoseStamped` | Pose global final, normalmente suavizada |
| `pose_global_corrected_raw` | `geometry_msgs/msg/PoseStamped` | Pose global sin suavizado |
| `pose_global_body_corrected` | `geometry_msgs/msg/PoseStamped` | Pose del cuerpo del dron, corregida |
| `pose_global_body_corrected_raw` | `geometry_msgs/msg/PoseStamped` | Pose del cuerpo sin suavizado |

## Modos de corrección

1. **`CORRECTED_KEYFRAME_RELATIVE`**  
   Usa el corrected KF cercano. Calcula:

   ```text
   T_world_current = T_world_kf_corrected * inv(T_local_kf) * T_local_current
   ```

2. **Fallback rígido**  
   Si no hay corrected KFs recientes o están lejos, usa:

   ```text
   T_world_camera = T_world_local * T_local_camera
   ```

3. **No publicar**  
   Si la corrección está demasiado vieja y los parámetros lo impiden.

## Suavizado

El nodo publica raw y smoothed. El suavizado:

- interpola transformaciones;
- limita saltos de traslación;
- limita saltos de yaw;
- resetea si hay salto bruto grande o mucho tiempo sin publicar.

## Extrínsecos cuerpo-cámara

Usa parámetros:

- `body_T_camera_x/y/z`;
- `body_T_camera_roll_deg/pitch_deg/yaw_deg`;
- `use_camera_optical_frame_convention`.

Sirve para convertir pose de cámara a pose de cuerpo.

## Catálogo de funciones actual

| Línea aprox. | Función | Qué hace / cuándo tocarla |
|---:|---|---|
| 3 | `GlobalPoseCorrector::GlobalPoseCorrector` | Constructor: inicializa parámetros, estructuras internas y dependencias de la clase. |
| 345 | `GlobalPoseCorrector::MapCorrectionCallback` | Callback ROS/timer: recibe eventos, mensajes o ticks y dispara la lógica asociada. |
| 407 | `GlobalPoseCorrector::CorrectedKeyFramesCallback` | Callback ROS/timer: recibe eventos, mensajes o ticks y dispara la lógica asociada. |
| 476 | `GlobalPoseCorrector::LocalPoseCallback` | Callback ROS/timer: recibe eventos, mensajes o ticks y dispara la lógica asociada. |
| 1255 | `GlobalPoseCorrector::PoseMsgToTf2` | Función auxiliar interna. Revisar implementación antes de modificarla. |
| 1269 | `GlobalPoseCorrector::TransformMsgToTf2` | Transforma poses/puntos entre frames. |
| 1283 | `GlobalPoseCorrector::Tf2ToPoseMsg` | Función auxiliar interna. Revisar implementación antes de modificarla. |
| 1305 | `GlobalPoseCorrector::IsFiniteTransform` | Predicado de validación o pertenencia. |
| 1326 | `GlobalPoseCorrector::IsFinitePose` | Predicado de validación o pertenencia. |
| 1346 | `GlobalPoseCorrector::TranslationDistance` | Predicado de validación o pertenencia. |
| 1362 | `GlobalPoseCorrector::YawFromTransform` | Transforma poses/puntos entre frames. |
| 1377 | `GlobalPoseCorrector::NormalizeAngleRad` | Normaliza ángulos, cuaterniones o matrices. |
| 1390 | `GlobalPoseCorrector::YawDifferenceDeg` | Extrae o compara yaw. |
| 1410 | `GlobalPoseCorrector::AgeSeconds` | Función auxiliar interna. Revisar implementación antes de modificarla. |
| 1421 | `GlobalPoseCorrector::ClampDouble` | Acota un valor numérico. |
| 1436 | `GlobalPoseCorrector::NormalizeQuaternion` | Normaliza ángulos, cuaterniones o matrices. |
| 1458 | `GlobalPoseCorrector::InterpolateTransform` | Transforma poses/puntos entre frames. |
| 1503 | `GlobalPoseCorrector::BuildBodyTCameraQuaternion` | Construye una estructura derivada a partir del estado del servidor/backend. |
| 1573 | `GlobalPoseCorrector::GroundTruthPoseCallback` | Callback ROS/timer: recibe eventos, mensajes o ticks y dispara la lógica asociada. |

## Cuándo tocar este archivo

- Fase 2: pose exacta de drones sin GT.
- Ajuste de suavizado.
- Corrección de convención cámara/cuerpo.
- Diagnóstico si RViz muestra pose global saltando o girada.

## Cuándo no tocarlo

- Para arreglar D4 normalmente no hace falta tocarlo.
- No usar GT para corregir pose final; GT solo diagnóstico.
