# Paquetes y planificación legacy

## Paquetes legacy o de pruebas anteriores

| Paquete | Estado | Regla |
|---|---|---|
| `mi_tfg` | legacy/bajo interés | No modificar salvo petición explícita. |
| `ORB_SLAM3_MULTI` | legacy/bajo interés | No modificar salvo petición explícita. |

El pipeline activo usa:

- `dron_individual`
- `lib_tray`
- `simulacion_dron`
- `orbslam3_msgs`
- `orbslam3_ros2`
- `orbslam3_multi`
- `orbslam3_server`
- `ORB_SLAM3` como librería externa/base

## Subfases legacy de Fase 1

Las subfases residuales:

```text
subfase_12R-D4.md
subfase_12R-D5.md
subfase_12R-E.md
subfase_12R-F.md
subfase_13_fiducial_pose_window_error.md
subfase_14_optimizacion_global_fiducial.md
subfase_15_limpieza_legacy.md
```

no son planificación activa. Se conservan porque documentan problemas reales del servidor monolítico anterior:

- guards de fiduciales;
- deuda `FIDUCIAL_GLOBAL_DEBT`;
- subcloud loop detection;
- promoción de eventos a tareas;
- optimización local heredada;
- apply y rollback incompletos;
- criterios de logs que conviene no repetir mal.

La planificación activa está en `subfase_1A.md` a `subfase_1X.md`.

## Regla para Codex

No borrar legacy antes de que la ruta nueva compile, se valide y quede documentada. No extender rutas legacy salvo que una subfase activa lo exija explícitamente como puente temporal.
