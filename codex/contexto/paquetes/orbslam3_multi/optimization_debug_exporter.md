# `OptimizationDebugExporter`

## Rol actual

`OptimizationDebugExporter` es una utilidad opcional introducida en `1J` para exportar una vista 2D del dry-run.

No participa en el flujo normal de datos. El servidor no debe leer el SVG ni un CSV para consumir el resultado. La salida normal del dry-run es `OptimizationDryRunResult` en memoria.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/optimization_debug_exporter.hpp
orbslam3_multi/src/optimization_debug_exporter.cpp
```

## API principal

```cpp
OptimizationDebugExportResult ExportDryRun2DPlot(
    const PoseGraphProblem& graph,
    const OptimizationDryRunResult& result,
    const std::string& output_path) const;
```

## Activacion

El servidor solo lo invoca si:

```text
f1j_export_debug_plot=true
```

Por defecto queda:

```text
f1j_export_debug_plot=false
```

En el camino normal validado en `1J`, el log esperado es:

```text
[F1J-OPT-DEBUG-EXPORT] task_id=... enabled=false reason=disabled_by_default
```

## Salida

Cuando se activa, genera un SVG en:

```text
codex/archivos_auxiliares/f1j_dryrun_task_<task_id>.svg
```

El SVG representa en plano `x,y`:

- poses antiguas de vertices;
- poses propuestas de vertices;
- KFs afectados no variables;
- poses propagadas propuestas;
- aristas antes y despues del dry-run;
- resumen con `task_id`, tipo, error before/after, vertices, aristas y propagacion.

## Restricciones

- No escribir estado del mapa.
- No leer ni escribir `RawMapDatabase` o `GlobalPoseStore`.
- No sustituir logs de validacion.
- No convertir CSV/SVG en interfaz entre `orbslam3_multi` y `orbslam3_server`.

## Validacion 1J

Las pruebas de `1J` se ejecutaron con `f1j_export_debug_plot:=false` para confirmar que la exportacion no es obligatoria ni automatica.

Evidencia:

- `prueba_1.reduced.log` y `prueba_2.reduced.log` contienen `[F1J-OPT-DEBUG-EXPORT] ... enabled=false reason=disabled_by_default`;
- los dry-runs se validaron por logs y `OptimizationDryRunResult`;
- no se requirio ningun archivo SVG para que el servidor completase el flujo.
