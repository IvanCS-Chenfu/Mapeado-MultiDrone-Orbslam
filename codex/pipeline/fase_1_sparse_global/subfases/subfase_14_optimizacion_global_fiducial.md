# Subfase 14 — Optimización global/fiducial

## Estado

`sin hacer`

## Dependencia

No empezar hasta que la Subfase 13 mida correctamente el error fiducial y hasta que el loop local esté estabilizado.

## Objetivo

Resolver casos de deriva acumulada entre fiduciales mediante una optimización global/fiducial controlada.

## Problema que resuelve

Caso típico:

1. un submapa se ancla en un fiducial;
2. recorre una trayectoria larga;
3. acumula deriva;
4. vuelve a un fiducial;
5. la posición estimada no coincide con la pose absoluta esperada.

Esto requiere una optimización global/fiducial, no una optimización local de subnube.

## Método recomendado primero

Crear tareas explícitas:

```text
GLOBAL_FIDUCIAL_OPT_TASK
```

La optimización debe considerar:
- fiduciales confirmados;
- loops fuertes;
- edges locales;
- fused edges;
- pesos coherentes;
- regiones móviles y regiones hard.

## Reglas críticas

- No mover fiduciales confirmados incorrectamente.
- No mover todo rígidamente sin criterio.
- No confundir con optimización local por loop.
- No reactivar rutas legacy sin control.
- No usar ground truth salvo fiducial simulado/debug.

## Build esperado

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

## Pruebas requeridas

Prueba tipo U o recorrido largo:
1. anclar en fiducial;
2. recorrer zona con deriva;
3. volver a fiducial;
4. comprobar que la optimización global reduce error sin romper mapa local.

Patrones de log:

```text
SCENARIO-RUNNER|GLOBAL_FIDUCIAL_OPT_TASK|FIDUCIAL-POSE-WINDOW-ERROR|FID-DEBT|POSE-GRAPH|GLOBAL|LOCAL-OPT|FUSED|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc
```

## Criterio de éxito

La subfase se considera superada si:
- se crea y ejecuta tarea global/fiducial explícita;
- reduce error fiducial alto;
- no rompe loops locales;
- no genera dobles paredes nuevas;
- mantiene fiduciales confirmados coherentes.
