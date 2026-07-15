# Launches de `dron_individual`

## `launch/generar_dron.launch.py`

Lanza los nodos principales de un dron dentro del namespace que le ponga el launch superior.

Nodos lanzados:

| Ejecutable | Nombre | Rol |
|---|---|---|
| `gen_tray` | `gen_tray` | Action server de trayectoria |
| `control_calcular_fuerzas` | `control_calcular_fuerzas` | Control PD a fuerza/torque |
| `aplicar_fuerzas_dron` | `aplicar_fuerzas_dron` | Mezcla fuerza/torque a motores |

También incluye `orbslam_use.launch.py` si `activar_orbslam=true`.

Argumentos:

- `usar_veltrap`;
- `activar_orbslam`;
- `drone_id`;
- `drone_name`;
- `local_map_frame`;
- `use_sim_time`.

Lee defaults desde:

- `config/tray_dron.yaml`;
- `config/vision.yaml`.

## `launch/orbslam_use.launch.py`

Lanza ORB-SLAM3 mono o estéreo.

Argumentos:

- `vocab`;
- `yaml_mono`;
- `yaml_stereo`;
- `rectify`;
- `use_sim_time`;
- `usar_estereo`;
- `drone_id`;
- `drone_name`;
- `local_map_frame`.

Nodo estéreo actual:

```text
package='orbslam3'
executable='stereo'
name='orbslam3_stereo'
```

Remappings:

```text
camera/left  -> sensor/camara_izq/image_raw
camera/right -> sensor/camara_der/image_raw
```

Parámetros enviados:

- `use_sim_time`;
- `drone_id`;
- `drone_name`;
- `local_map_frame`;
- `use_corrected_keyframes=true`;
- `max_nearest_kf_distance_m=2.0`.

## Riesgos

- El paquete ejecutable del wrapper se llama `orbslam3`, no `orbslam3_ros2` en este launch.
- La calibración usada por este launch está en `dron_individual/config/orbslam/`, no en `simulacion_dron/config/orbslam/`.
- Si se cambian nombres de topics de cámara, actualizar remappings.
