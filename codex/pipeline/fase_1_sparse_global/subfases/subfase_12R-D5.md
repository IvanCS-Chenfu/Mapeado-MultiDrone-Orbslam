# Subfase 12R-D5 — Validar calidad real de la corrección local

## Estado

`sin hacer`

## Dependencia

No empezar hasta que `12R-D4` esté superada.

## Objetivo

Validar que la optimización local aplicada mejora realmente el error geométrico de la subnube, no solo el coste interno predicho por el solver.

D4 debe conseguir que se apliquen movimientos. D5 debe comprobar que esos movimientos son buenos.

## Problema que resuelve

Un dry-run puede predecir mejora, pero tras aplicar los cambios en el servidor podría ocurrir que:
- la nube publicada no mejore;
- el error real no baje;
- se corrija una zona pero se deteriore otra;
- se refuercen edges incorrectos;
- se fusione demasiado pronto.

## Método recomendado primero

Para cada `LOCAL_LOOP_OPT_TASK` aplicado:

1. guardar poses relevantes antes del apply;
2. aplicar optimización local;
3. recalcular `LOOP-SUBCLOUD-ERROR` usando el mismo `source_event`;
4. comparar error before/after:
   - error medio;
   - error traslacional;
   - error yaw;
   - residual de inliers;
   - número de matches/inliers;
5. aceptar como corrección buena solo si la mejora real supera umbrales.

## Acciones esperadas

Añadir o revisar lógica para:
- guardar snapshot de poses before;
- recalcular error después de aplicar;
- publicar o loguear `LOCAL-OPT-CONSTRAINT-AFTER-APPLY`;
- marcar evento como corregido si mejora;
- marcar `hold/fail` si no mejora;
- no reforzar edges si la mejora real no es suficiente.

## Cambios permitidos

- `orbslam3_server/src/global_map_server.cpp`
- estructuras auxiliares del servidor si ya existen
- logs de diagnóstico
- documentación

## Cambios prohibidos

- No implementar fusión post-optimización masiva todavía.
- No activar optimización global/fiducial.
- No limpiar legacy todavía.
- No usar ground truth para validar éxito interno.

## Build esperado

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

## Pruebas requeridas

### Prueba 1 — D4 limpia repetida

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Debe comprobar que no se aplica corrección si no hay mejora real necesaria.

### Prueba 2 — deriva con corrección

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Debe comprobar before/after.

Patrones de log:

```text
SCENARIO-RUNNER|LOCAL_LOOP_OPT_TASK|LOCAL-OPT-DRYRUN|LOCAL-OPT-APPLY|LOCAL-OPT-CONSTRAINT-AFTER-APPLY|LOOP-SUBCLOUD|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc
```

## Criterio de éxito

La subfase se considera superada si:
- compila;
- se calcula error real post-apply;
- el sistema distingue mejora real de mejora insuficiente;
- no se refuerzan edges si el error real no baja;
- se registran resultados claros en logs;
- el historial queda actualizado.
