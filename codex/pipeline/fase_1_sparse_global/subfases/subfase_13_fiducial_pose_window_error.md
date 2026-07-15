# Subfase 13 — FIDUCIAL-POSE-WINDOW-ERROR

## Estado

`sin hacer`

## Dependencia

No empezar hasta estabilizar la optimización local por loops.

## Objetivo

Separar completamente el error fiducial del error de loop y calcular un error de ventana fiducial cuando un submapa ya anclado vuelve a observar o entrar en un fiducial.

## Problema que resuelve

Un submapa puede estar anclado por un fiducial, recorrer una zona con deriva acumulada y volver a otro fiducial o al mismo fiducial. En ese caso, el sistema debe medir si la trayectoria acumuló un error absoluto relevante.

Ese error no debe tratarse como loop local.

## Método recomendado primero

Cuando un submapa anclado entra en una ventana fiducial:

1. recopilar KeyFrames de la ventana;
2. calcular pose esperada en mundo por el fiducial;
3. comparar contra pose estimada actual;
4. obtener `FIDUCIAL-POSE-WINDOW-ERROR`;
5. clasificar:
   - error bajo: no optimizar;
   - error medio: hold con histéresis;
   - error alto: crear futura `GLOBAL_FIDUCIAL_OPT_TASK`.

## Reglas críticas

- `FIDUCIAL_GLOBAL_DEBT` no debe ser prior fuerte.
- No forzar `force_pose_graph_optimization_`.
- No mezclar con `LOOP-SUBCLOUD-ERROR`.
- No aplicar optimización global todavía si esta subfase solo mide error.

## Build esperado

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

## Pruebas requeridas

Prueba con dron anclado que vuelve al fiducial tras recorrido con deriva.

Patrones de log:

```text
SCENARIO-RUNNER|FIDUCIAL|FIDUCIAL-POSE-WINDOW-ERROR|FID-DEBT|GLOBAL_FIDUCIAL_OPT_TASK|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc
```

## Criterio de éxito

La subfase se considera superada si:
- se calcula error fiducial de ventana;
- se clasifica bajo/medio/alto;
- no se lanza optimización global por accidente;
- no se añade debt como prior fuerte;
- queda evidencia lista para Fase 14.
