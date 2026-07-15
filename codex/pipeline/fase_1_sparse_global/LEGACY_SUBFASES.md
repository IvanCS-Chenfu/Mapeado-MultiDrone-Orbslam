# Legacy de Fase 1 — Subfases antiguas

## Estado

```text
legacy
```

## Propósito

Este archivo marca las subfases residuales de Fase 1 que siguen presentes en `subfases/` pero no forman parte de la planificación activa.

No se editan los archivos `subfase_*.md` antiguos para evitar alterar su valor histórico. La marca de legacy vive aquí y en los documentos generales.

## Planificación activa

La planificación activa está en:

```text
subfases/subfase_1A.md
...
subfases/subfase_1X.md
```

## Archivos legacy

| Archivo | Motivo para conservarlo |
|---|---|
| `subfase_12R-D4.md` | Aprendizaje sobre apply local, fiduciales, subcloud/unified aux y tareas pendientes del servidor monolítico. |
| `subfase_12R-D5.md` | Continuación antigua de optimización local. |
| `subfase_12R-E.md` | Ruta antigua posterior de optimización/subcloud. |
| `subfase_12R-F.md` | Ruta antigua posterior de optimización/subcloud. |
| `subfase_13_fiducial_pose_window_error.md` | Plan antiguo de error fiducial. La nueva ruta está en 1H-1L. |
| `subfase_14_optimizacion_global_fiducial.md` | Plan antiguo de optimización global/fiducial. La nueva ruta usa `PoseGraphBuilder` y `OptimizationManager`. |
| `subfase_15_limpieza_legacy.md` | Plan antiguo de limpieza. La limpieza nueva debe cerrarse en 1X. |

## Regla operativa

Un chat nuevo de Codex no debe ejecutar estos archivos como fase activa. Puede consultarlos para:

- entender errores ya observados;
- no repetir decisiones peligrosas;
- recuperar patrones de log útiles;
- migrar funcionalidad al backend nuevo si una subfase 1A-1X lo pide.

No borrar estos archivos salvo fase explícita de limpieza con build, pruebas e historial.
