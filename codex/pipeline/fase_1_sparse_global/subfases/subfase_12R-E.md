# Subfase 12R-E — Fusión y score post-optimización

## Estado

`sin hacer`

## Dependencia

No empezar hasta que `12R-D5` esté superada.

## Objetivo

Cuando una optimización local corrige una doble pared o reduce claramente el error de subnube, fusionar landmarks equivalentes y actualizar el score global de los fused tracks.

## Problema que resuelve

Aunque una corrección local mueva KeyFrames, pueden quedar:
- landmarks duplicados;
- tracks separados que representan el mismo punto físico;
- outliers con score alto;
- inliers sin refuerzo suficiente;
- dobles paredes parcialmente persistentes.

## Método recomendado primero

Usar los matches inliers del `LoopSubcloudEvent` que fue corregido y validado en D5.

Después del apply:
1. revalidar distancias de inliers;
2. crear o actualizar `FusedLandmarkTrack`;
3. fusionar tracks equivalentes;
4. subir score de inliers consistentes;
5. bajar score de outliers;
6. reforzar edges locales si el error bajó;
7. publicar una única posición global robusta por landmark físico.

## Cambios permitidos

- lógica de fused tracks en servidor;
- relación entre subcloud events y fused landmarks;
- score/histéresis si ya existe;
- logs de fusión post-optimización;
- documentación.

## Cambios prohibidos

- No rediseñar wrapper.
- No mover score global al wrapper.
- No usar ground truth para score.
- No fusionar sin validación post-apply.
- No hacer limpieza legacy todavía.

## Build esperado

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

## Pruebas requeridas

### Prueba 1 — caso con fusión clara

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Debe provocar observación común de una pared/zona con ambos drones.

### Prueba 2 — caso con deriva corregida

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Debe comprobar que tras aplicar optimización se consolidan landmarks.

Patrones de log:

```text
SCENARIO-RUNNER|FUSION_EVENT|FUSED|FUSED-SIMPLE-SUMMARY|LOCAL_OPT_APPLIED|LOCAL-OPT|LOOP-SUBCLOUD|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc
```

## Criterio de éxito

La subfase se considera superada si:
- compila;
- se fusionan landmarks equivalentes tras corrección validada;
- baja la duplicación visible;
- no se fusionan outliers claros;
- el score de fused tracks refleja evidencia positiva/negativa;
- la nube publicada es más limpia.
