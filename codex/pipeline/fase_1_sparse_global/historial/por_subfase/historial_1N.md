# Historial subfase 1N

## 2026-07-17 — Infraestructura provisional sin contrato BoW

- objetivo intentado:
  - adelantar el despacho de KFs nuevos y la frontera de `LoopDetector` sin
    inventar una comparación BoW que el checkout no puede verificar.
- archivos modificados:
  - `RawMapDatabase`, `LoopCandidate`, `LoopDetector` y `global_map_server`;
  - documentación de `1N` y de los paquetes afectados.
- cambios realizados:
  - `RawInsertResult` entrega `new_keyframe_ids`;
  - servidor despacha nuevos KFs en live, snapshots y replay;
  - detector devuelve `bow_data_unavailable_in_current_checkout` hasta disponer
    del contrato BoW/`FeatureVector` real;
  - marcadores normalizados de `F1M-LOOP-*` a `F1N-LOOP-*`.
- paquetes compilados:
  - no ejecutado: faltan `orbslam3_multi/CMakeLists.txt` y `orbslam3_msgs`.
- pruebas Gazebo/replay:
  - pendientes en VS Code/workspace completo.
- conclusión:
  - `1N` queda **por implementar**; no se producen candidatos ficticios ni se
    confirma ningún loop sin BoW/geometría.
- siguiente paso recomendado:
  - conectar los campos BoW reales de `OrbKeyFrame`, indexar/rankear y consultar
    `CovisibilityDatabase` antes de enviar candidatos a `1O`.
