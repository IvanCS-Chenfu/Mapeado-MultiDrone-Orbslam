# Pipeline Fase 1 — resumen

Usar este archivo antes de abrir `pipeline_fase_1.md`.

## Estado

```text
Fase 1: actual
Subfase actual: 1N
Conclusión 1L: PARCIAL
1M: `CovisibilityDatabase`, a probar en simulación
1N: infraestructura provisional, por implementar; completar tras validar 1M/BoW
```

## Objetivo de Fase 1

Mapa sparse global multi-dron coherente, anclado, optimizable y preparado para
fusión, loops y futura nube densa.

## Tabla corta

| Subfase | Estado | Resultado |
|---|---|---|
| 1A | realizado | baseline servidor antiguo |
| 1B | realizado | servidor mínimo |
| 1C | realizado | `RawMapDatabase` y replay |
| 1D | realizado | `GlobalPoseStore` |
| 1E | realizado | anchors fiduciales |
| 1F | realizado | `/global_sparse_cloud` inicial |
| 1G | realizado | full snapshots y reconciliación |
| 1H | realizado | revisits fiduciales y tareas |
| 1I | realizado | `PoseGraphBuilder` con anclajes previos |
| 1J | realizado | dry-run, dump/replay offline y HTML diagnóstico del grafo |
| 1K | realizado/parcial | apply del grafo en `GlobalPoseStore`, propagación y publicación coherente |
| 1L | parcial | diagnóstico post-apply: logs/sublogs/RViz2 opcional/GT debug sin ser dueño del apply |
| 1M | actual/a probar en simulación | `CovisibilityDatabase` implementada; pendiente de validar importación ORB-SLAM3 y consultas |
| 1N | actual/por implementar | `LoopDetector` con despacho de KFs; falta contrato y búsqueda BoW real |
| 1O | sin hacer | subnubes/RANSAC |
| 1P | sin hacer | decisión de loop/fusión |
| 1Q | sin hacer | optimización por loop |
| 1R-1X | sin hacer | cierre de concurrencia, scoring, observabilidad, regresión y handoff |

## Regla de avance

No pasar a loops (`1N-1Q`) hasta que:

- `1J` muestre un dry-run/grafo coherente en HTML;
- `1K` aplique el grafo y publique MapPoints siguiendo las poses optimizadas;
- `1L` deje evidencia diagnóstica suficiente en logs/sublogs/GT debug, con
  RViz2 como confirmación visual cuando esté disponible;
- `1M` exista como `CovisibilityDatabase` confirmada y consultable por
  `LoopDetector`/`PoseGraphBuilder`.

## Archivos de detalle

```text
codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md
codex/pipeline/fase_1_sparse_global/subfases/subfase_1M.md
codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md
codex/pipeline/fase_1_sparse_global/historial/INDEX.md
codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md
```

## Paquetes frecuentes

```text
orbslam3_multi
orbslam3_server
simulacion_dron si cambia launch/YAML
```
