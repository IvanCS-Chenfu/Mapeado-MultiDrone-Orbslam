# 00 — Bootstrap mínimo para nuevo chat

Este archivo queda como respaldo corto. En chats nuevos leer primero:

```text
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
```

## Lectura recomendada

1. `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`
2. `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`
3. `codex/pipeline/fase_1_sparse_global/pipeline_fase_1_RESUMEN.md`
6. Subfase concreta si se va a implementar.
7. `historial/INDEX.md` y solo el historial por subfase necesario.
8. Docs del paquete/componente que se vaya a tocar.

## Estado operativo

- Fase activa: Fase 1 — mapa sparse global multi-dron.
- Subfase activa: `1M`, `CovisibilityDatabase` a probar en simulación.
- `1L` queda `PARCIAL` y se volverá a comprobar en el futuro.
- No iniciar `1N+` hasta validar `1M`.
- Objetivo global del proyecto: nube densa global sin usar ground truth para mapa
  final ni pose final.

## Invariantes que no se negocian

- `submapa = (drone_id, map_epoch)`.
- `RawMapDatabase` conserva raw ORB-SLAM3 y no se modifica por optimización.
- `GlobalPoseStore` conserva poses globales, anchors, optimizaciones y rollback.
- Fiduciales son observaciones absolutas, no loops.
- Ground truth solo para fiducial simulado, debug o métricas externas.
- Wrapper y mensajes son estables.
- `ORB_SLAM3` no se toca salvo permiso explícito.

## Ruta actual de la arquitectura

```text
orbslam3_ros2
  -> orbslam3_server
  -> orbslam3_multi
      RawMapDatabase
      GlobalPoseStore
      FiducialAnchorManager
      CovisibilityDatabase
      PoseGraphBuilder
      OptimizationManager
      GlobalMapBuilder
```

`orbslam3_server` debe ser adaptador ROS 2; `orbslam3_multi` debe concentrar la
lógica algorítmica.

## Herramientas

```bash
./codex/herramientas/build_selected_packages.sh <paquetes>
./codex/herramientas/reduce_build_log.sh
./codex/herramientas/run_simulation.sh --prueba X --launch "ros2 launch simulacion_dron multi_dron.launch.py"
./codex/herramientas/reduce_simulation_log.sh --prueba X --patterns "<patrones>"
```

Para logs grandes, crear sublogs específicos antes de leer el log completo.

## Dónde está el detalle

| Necesidad | Archivo |
|---|---|
| Estado completo | `codex/contexto/01_ESTADO_ACTUAL.md` |
| Estado corto | `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md` |
| Contexto mínimo | `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md` |
| Reglas técnicas | `codex/contexto/02_REGLAS_TECNICAS.md` |
| Mapa de código | `codex/contexto/06_MAPA_CODIGO.md` |
| Historial | `codex/pipeline/fase_1_sparse_global/historial/INDEX.md` |
| Política antitokens | `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md` |
