# Subfase 15 — Limpieza legacy

## Estado

`sin hacer`

## Dependencia

No empezar hasta que D4, D5, E, F, 13 y 14 estén estabilizadas.

## Objetivo

Eliminar o desactivar rutas legacy que contradicen el pipeline actual o que ya no deben decidir comportamiento.

## Rutas candidatas a limpiar

No limpiar sin revisar el código actual.

Candidatos mencionados:

```text
ComputeUnifiedLoopGraphError
ComputeLoopViewMapErrorDetailed
LoopViewMapErrorInfo
DecideUnifiedLoopDryRunAction
RequestLoopOptimizationRetryFromEvents
force_pose_graph_optimization_
optimización por new_loop_edges
optimización por new_fiducial_priors
relation_type-based thresholds si contradicen confianza real
logs masivos phase_* obsoletos
```

## Regla principal

No limpiar legacy antes de que el pipeline nuevo funcione.

La limpieza debe ser incremental:
1. localizar uso;
2. comprobar si sigue siendo necesario;
3. desactivar o eliminar;
4. compilar;
5. probar;
6. documentar.

## Cambios permitidos

- servidor global;
- configuración/launch;
- documentación;
- tests/scripts auxiliares si existen.

## Cambios prohibidos

- No rediseñar arquitectura funcional en esta fase.
- No cambiar wrapper/mensajes salvo que queden rutas muertas.
- No mezclar limpieza con nuevas features.

## Build esperado

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

## Pruebas requeridas

Repetir pruebas clave de fases anteriores:
- prueba limpia;
- prueba con loop local;
- prueba con fusión;
- prueba fiducial si ya existe.

Patrones de log:

```text
SCENARIO-RUNNER|LOCAL-OPT|LOOP-SUBCLOUD|FUSION_EVENT|FIDUCIAL|GLOBAL_FIDUCIAL_OPT_TASK|FUSED|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc
```

## Criterio de éxito

La subfase se considera superada si:
- se eliminaron rutas obsoletas sin regresión;
- el sistema compila;
- las pruebas principales siguen funcionando;
- los logs son más claros;
- la documentación refleja el nuevo estado.
