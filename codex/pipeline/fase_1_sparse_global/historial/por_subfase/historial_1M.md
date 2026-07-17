# Historial subfase 1M

## 2026-07-14 — Reserva de nueva subfase 1M

- objetivo intentado:
  - reservar una subfase nueva entre `1L` y la antigua `1M`;
  - desplazar la planificación posterior para que las antiguas `1M-1W` pasen a
    `1N-1X`.
- archivos modificados:
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1M.md`;
  - documentos de pipeline, contexto e índices que apuntan a la subfase actual.
- paquetes compilados:
  - no aplica; cambio documental.
- pruebas Gazebo/replay:
  - no aplica; cambio documental.
- conclusión:
  - en ese momento `1M` queda temporalmente como hueco pendiente;
  - después se redefine como `CovisibilityDatabase` en la entrada siguiente.
- siguiente paso recomendado:
  - sustituido por la definición de `CovisibilityDatabase` documentada debajo.

## 2026-07-14 — Definición de `CovisibilityDatabase`

- objetivo intentado:
  - convertir `1M` en una subfase real para crear una base de covisibilidad
    confirmada;
  - documentar cómo la consumirán `1N`, `1O`, `1P` y `1Q`.
- archivos modificados:
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1M.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1N.md`;
  - subarchivos de `1O`, `1P` y `1Q`;
  - resúmenes e índices de contexto/pipeline.
- paquetes compilados:
  - no aplica; cambio documental.
- pruebas Gazebo/replay:
  - no aplica; cambio documental.
- conclusión:
  - `1M` queda como `CovisibilityDatabase`;
  - la base no guarda candidatos ni campo `state`;
  - solo entran relaciones confirmadas por ORB-SLAM3 nativo o por geometría del
    servidor;
  - las aristas guardan soporte, fuente y pose relativa medida/current.
- siguiente paso recomendado:
  - implementar `CovisibilityDatabase` antes de iniciar `1N`.

## 2026-07-16 — Implementación pendiente de validación en simulación

- objetivo intentado:
  - implementar la base confirmada de covisibilidad y conectarla a la ingesta
    raw y a `PoseGraphBuilder`.
- archivos modificados:
  - `orbslam3_multi/include/orbslam3_multi/covisibility_database.hpp`;
  - `orbslam3_multi/src/covisibility_database.cpp`;
  - `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`;
  - `orbslam3_multi/src/pose_graph_builder.cpp`;
  - `orbslam3_server/src/global_map_server.cpp`;
  - resúmenes y documentación de paquete de `1M`.
- cambios realizados:
  - importación de `connected_keyframe_ids/weights` con umbral configurable;
  - orden canónico de parejas, consultas de vecinos/ventana y conservación de
    `relative_pose_measured`;
  - integración de aristas `SoftConsistency` sin crear vértices nuevos;
  - logs de importación/resumen y reset de covisibilidad para replay.
- paquetes compilados:
  - no ejecutado: el checkout remoto no contiene `orbslam3_multi/CMakeLists.txt`
    ni el paquete `orbslam3_msgs` necesario para compilar fuera del workspace
    completo.
- pruebas Gazebo/replay:
  - pendiente: este entorno Codex Web no permite validar la simulación.
- conclusión:
  - `1M` queda **a probar en simulación**; no iniciar `1N` hasta obtener
    evidencia de importación y consultas con datos reales.
- siguiente paso recomendado:
  - compilar en el workspace completo y ejecutar replay/Gazebo verificando
    `[F1M-COVIS-IMPORT]`, `[F1M-COVIS-SUMMARY]` y aristas de ventana en
    `PoseGraphBuilder`.
