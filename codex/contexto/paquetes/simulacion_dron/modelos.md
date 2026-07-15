# Modelos, URDF/Xacro y plugins de `simulacion_dron`

## URDF/Xacro

### `urdf/dron.xacro`

Modelo base del dron:

- cuerpo central;
- brazos;
- motores;
- materiales;
- macros para 4, 6 u 8 brazos.

### `urdf/dron_plugins.xacro`

Modelo extendido con:

- plugins de motores;
- plugin de ground truth;
- cámara mono/estéreo;
- parámetros de sensores.

Usa parámetros procedentes de:

- `dron_individual/config/hardware.yaml`;
- `simulacion_dron/config/sim_dron.yaml`.

## Generación/spawn

### `src/generar_URDF/generador_URDF.cpp`

Nodo:

```text
generador_URDF
```

Rol:
- lee parámetros físicos y de simulación;
- ejecuta/genera descripción URDF/Xacro;
- llama al servicio `/spawn_entity` de Gazebo;
- spawnea el dron en el mundo.

## Plugins Gazebo

### `plugin_actuar_motores.cpp`

Plugin de Gazebo que se engancha al modelo y aplica fuerzas a los enlaces de motor.

Entradas ROS:

- `motor/arr_iz`
- `motor/ab_iz`
- `motor/ab_der`
- `motor/arr_der`
- o variantes para 6/8 motores según Xacro.

Uso:
- recibe fuerzas calculadas por `aplicar_fuerzas_dron`;
- aplica fuerzas y torques en Gazebo.

### `plugin_sensor_groundtrurh.cpp`

Plugin de Gazebo que publica estado real simulado del cuerpo.

Salidas:

| Topic | Tipo |
|---|---|
| `sensor/GT/pose` | `geometry_msgs/msg/PoseStamped` |
| `sensor/GT/vel` | `geometry_msgs/msg/TwistStamped` |
| `sensor/GT/acc` | `geometry_msgs/msg/AccelStamped` |

Uso permitido:
- control simulado;
- fiducial simulado;
- debug.

Uso prohibido:
- pose final sin GT;
- construcción de mapa final;
- fused score;
- dense mapping final.

## Cámara

Parámetros en `hardware.yaml`:

- `sensores.camara.porcentaje_estereo`;
- `sensores.camara.publish_rate`;
- `sensores.camara.mostrar_gazebo`.

Ojo:
- La calibración ORB-SLAM3 usada por el launch actual está en `dron_individual/config/orbslam/orbslam_stereo.yaml`.
- Debe ser coherente con la cámara generada por Xacro/Gazebo.
