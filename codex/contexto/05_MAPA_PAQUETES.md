# 05 — Mapa de paquetes

## Propósito

Índice rápido de paquetes y responsabilidades. La documentación detallada vive en:

```text
codex/contexto/paquetes/<paquete>/00_summary.md
```

Si se modifica un paquete, actualizar también su documentación.

Nota: para buscar un paquete concreto sin abrir muchos archivos, usar:

```bash
python3 codex/herramientas/find_context.py <query>
```

## Paquetes principales

| Paquete | Estado | Rol actual |
|---|---|---|
| `orbslam3_msgs` | estable | Contrato ROS 2 entre wrapper, servidor y corrector. No añadir score global ni rediseñar salvo fase explícita. |
| `orbslam3_ros2` | estable | Wrapper estéreo ORB-SLAM3. Publica pose local, `OrbMap` delta y `GetOrbMap`. |
| `orbslam3_multi` | activo/migración | Backend algorítmico objetivo de Fase 1. La ruta activa contiene `RawMapDatabase`; los módulos antiguos están solo en `orbslam3_multi/legacy/`. |
| `orbslam3_server` | activo/migración | Adaptador ROS 2 objetivo. El servidor legacy monolítico se conserva como referencia durante 1A-1B. |
| `dron_individual` | activo | Control por dron y acción `AccionTrayectoria`. |
| `simulacion_dron` | activo | Gazebo, launch multi-dron, mundo, plugins GT y nodo/flujo de escenarios. |
| `lib_tray` | activo | Generación de trayectorias. |
| `ORB_SLAM3` | externo/no tocar | Librería base ORB-SLAM3. Solo modificar con permiso explícito. |

## Separación conceptual de Fase 1

### Servidor ROS 2

```text
orbslam3_server
```

Debe tender a:

- recibir y publicar ROS 2;
- convertir mensajes;
- leer GT solo para fiducial simulado/debug;
- delegar lógica en `orbslam3_multi`.

### Backend algorítmico

```text
orbslam3_multi
```

Debe tender a contener:

- `RawMapDatabase`;
- `GlobalPoseStore`;
- `FiducialAnchorManager`;
- `LoopDetector`;
- `SubcloudLoopVerifier`;
- `LoopDecisionManager`;
- `FusionManager` / `FusedLandmarkManager`;
- `LandmarkScoreManager`;
- `PoseGraphBuilder`;
- `OptimizationManager`;
- `GlobalMapBuilder`.

### Dron/wrapper

```text
orbslam3_ros2
orbslam3_msgs
dron_individual
lib_tray
```

No rediseñar durante Fase 1 salvo necesidad justificada por una subfase.

### Simulación

```text
simulacion_dron
```

Proporciona Gazebo, GT permitido para fiduciales/debug y el launch oficial:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

## Paquetes legacy o de bajo interés

| Paquete | Regla |
|---|---|
| `mi_tfg` | No usar salvo petición explícita. |
| `ORB_SLAM3_MULTI` | No usar salvo petición explícita. |

Más detalles:

```text
codex/contexto/paquetes/paquetes_legacy.md
```

## Separación futura de paquetes

La Fase 4 separará físicamente o lógicamente:

```text
servidor/
dron/
simulacion/
```

Hasta entonces, todo sigue bajo `src/`, pero las responsabilidades de Fase 1 ya deben respetar esa separación conceptual.
