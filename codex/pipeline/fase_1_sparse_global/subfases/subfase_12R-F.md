# Subfase 12R-F — Gestión de KeyFrames pendientes durante optimización

## Estado

`sin hacer`

## Dependencia

No empezar hasta que D4, D5 y E estén razonablemente estabilizadas.

## Objetivo

Gestionar correctamente los KeyFrames que llegan mientras una optimización local está activa.

## Problema que resuelve

Durante una optimización local, pueden llegar nuevos KeyFrames del wrapper. Si se procesan como si el mapa estuviera estable, pueden:
- lanzar loops contra una geometría en transición;
- fusionarse contra landmarks no corregidos;
- crear edges incoherentes;
- publicar puntos provisionales como definitivos.

## Método recomendado primero

Cuando una región/submapa esté en optimización:

1. aceptar KeyFrames nuevos para no perder datos;
2. marcarlos como pendientes/provisionales;
3. no permitir que disparen loops/fusiones definitivas hasta que termine la optimización;
4. al terminar, reproyectar/revalidar esos KFs contra el mapa corregido;
5. publicarlos o fusionarlos solo si pasan validación.

## Cambios permitidos

- estructuras pending existentes;
- gating de loops/fusiones durante optimización;
- revalidación post-optimización;
- logs de pending KFs.

## Cambios prohibidos

- No descartar KFs sin motivo.
- No bloquear indefinidamente un dron por estar optimizando.
- No usar ground truth.
- No activar optimización global.

## Build esperado

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

## Pruebas requeridas

Usar una prueba con movimiento continuo durante optimización.

Patrones de log:

```text
SCENARIO-RUNNER|PENDING|LOCAL-OPT|LOCAL_LOOP_OPT_TASK|FUSION_EVENT|FUSED|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc
```

## Criterio de éxito

La subfase se considera superada si:
- no se pierden KFs nuevos;
- los KFs pendientes no contaminan loops/fusiones;
- tras la optimización se revalidan correctamente;
- el mapa no presenta saltos o duplicados por KFs llegados a mitad de corrección.
