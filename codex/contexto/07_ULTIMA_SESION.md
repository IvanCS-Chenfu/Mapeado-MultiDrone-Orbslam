# 07 — Última sesión

## 2026-07-17 — Infraestructura provisional de 1N

- Se añadieron `LoopCandidate`, `LoopCandidateResult` y `LoopDetector`.
- `RawMapDatabase` expone los KFs nuevos de cada ingesta y `global_map_server`
  los despacha en live, full snapshot y replay.
- No se implementó una búsqueda BoW ficticia: faltan los campos BoW/
  `FeatureVector` de `OrbKeyFrame` en este checkout.
- Se normalizaron los marcadores planificados de 1N a `[F1N-LOOP-*]`.
- `1M` sigue pendiente de simulación y `1N` queda **por implementar** para
  completar desde VS Code con el workspace completo.
