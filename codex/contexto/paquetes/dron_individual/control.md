# Control del dron en `dron_individual`

## Archivos cubiertos

- `src/control_tray/gen_tray.cpp`
- `src/control_tray/control_calcular_fuerzas.cpp`
- `src/control_tray/aplicar_fuerzas_dron.cpp`
- `src/vision/control_dron.cpp` como control experimental de visión

## Flujo de control actual

```text
Cliente action / GUI / script
  ↓ goal TrayAction
/dron_X/AccionTrayectoria   (`gen_tray`)
  ↓ feedback: trayectoria deseada
control_calcular_fuerzas
  ↓ fuerza + torque
aplicar_fuerzas_dron
  ↓ fuerzas por motor
plugin_actuar_motores en Gazebo
  ↓ fuerzas físicas en enlaces motor
Gazebo
```

## `gen_tray.cpp`

Nodo:

```text
gen_tray
```

Action server:

```text
AccionTrayectoria
```

Entradas:

| Entrada | Tipo | Uso |
|---|---|---|
| `sensor/GT/pose` | `geometry_msgs/msg/PoseStamped` | Pose inicial para generar trayectoria |
| `sensor/GT/vel` | `geometry_msgs/msg/TwistStamped` | Velocidad inicial |
| `AccionTrayectoria` | `dron_individual/action/TrayAction` | Objetivo de trayectoria |

Salida:

- feedback de action con arrays `x`, `y`, `z`, `yaw`.

Funciones internas:

- `pose_actual_callback`: guarda una muestra de pose cuando no está bloqueada.
- `vel_actual_callback`: guarda una muestra de velocidad.
- `proyeccion`: reparte una velocidad máxima lineal entre X/Y/Z según dirección del movimiento.
- `pose2yaw`: extrae yaw de un `PoseStamped`.
- `normalizar_angulo`: normaliza ángulos a `[-pi, pi]`.
- `array_to_msg`: convierte `std::array<double,5>` a `Float64MultiArray`.
- `handle_goal`: acepta/rechaza goals. Rechaza `tipo_trayectoria > 2`.
- `handle_cancel`: acepta cancelaciones.
- `handle_accepted`: aborta goal anterior si existe y lanza `execute` en hilo separado.
- `execute`: función principal. Espera pose/vel inicial, crea trayectoria según tipo y publica feedback hasta terminar/cancelar.

Tipos de trayectoria:

1. `tipo_trayectoria=0`: `GenTrayPol3`.
2. `tipo_trayectoria=1`: `GenTrayVelTrap`.
3. `tipo_trayectoria=2`: `GenTrayElipse`.

Notas:
- Para objetivos relativos en X/Y se transforma el desplazamiento usando yaw actual.
- Para yaw relativo puede haber normalización si se sale de `[-pi, pi]`.
- Solo debe haber un goal activo; uno nuevo aborta el anterior.

## `control_calcular_fuerzas.cpp`

Nodo:

```text
control_calcular_fuerzas
```

Rol:
- recibe feedback de trayectoria y GT;
- calcula fuerza total y torque deseado con control PD;
- publica comandos agregados.

Entradas típicas:

| Topic | Tipo | Uso |
|---|---|---|
| `AccionTrayectoria/_action/feedback` | feedback action | Trayectoria deseada |
| `sensor/GT/pose` | `PoseStamped` | Pose real simulada |
| `sensor/GT/vel` | `TwistStamped` | Velocidad real simulada |

Salidas típicas:

| Topic | Tipo | Uso |
|---|---|---|
| `control/fuerza_total` | `std_msgs/msg/Float64` | Fuerza vertical total |
| `control/torque` | `geometry_msgs/msg/Vector3Stamped` | Torque deseado |

Funciones/conceptos:

- convierte pose a yaw;
- calcula error de posición/velocidad;
- aplica ganancias `kp`, `kv`, `kr`, `kw` de `tray_dron.yaml`;
- usa masa, inercia, gravedad y geometría del dron;
- calcula comandos agregados para `aplicar_fuerzas_dron`.

## `aplicar_fuerzas_dron.cpp`

Nodo:

```text
aplicar_fuerzas_dron
```

Rol:
- recibe fuerza total y torque;
- resuelve una matriz de mezcla para obtener fuerza en cada motor;
- publica fuerzas individuales.

Entradas:

| Topic | Tipo | Uso |
|---|---|---|
| `control/fuerza_total` | `std_msgs/msg/Float64` | Fuerza total |
| `control/torque` | `geometry_msgs/msg/Vector3Stamped` | Torque XYZ |

Salidas:

| Topic | Tipo |
|---|---|
| `motor/arr_iz` | `std_msgs/msg/Float64` |
| `motor/ab_iz` | `std_msgs/msg/Float64` |
| `motor/ab_der` | `std_msgs/msg/Float64` |
| `motor/arr_der` | `std_msgs/msg/Float64` |

Funciones/conceptos:

- construye matriz `A` de reparto fuerza/torque;
- si `A` es casi singular usa pseudoinversa;
- publica fuerzas por motor periódicamente.

## `control_dron.cpp`

Estado: experimental/visión.

Rol:
- cliente de `AccionTrayectoria`;
- recibe puntos de visión en `vision/keypoint_cercano`;
- publica byte de control en `vision/byte_control`;
- implementa comportamientos como encontrarse, colocarse frente a pared y obtener nube.

No es parte central del pipeline actual de sparse global multi-dron.

## Riesgos

- El control depende de GT en simulación.
- No usar estos topics de GT para estimación final de pose/mapa.
- Si se automatizan pruebas de Codex, el script debe llamar a `AccionTrayectoria` respetando namespaces.
