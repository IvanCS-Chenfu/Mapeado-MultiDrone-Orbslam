# 00_summary — orbslam3_multi

Resumen: Backend algorítmico principal de Fase 1. Contiene `RawMapDatabase`, `GlobalPoseStore`, `FiducialAnchorManager`, `GlobalMapBuilder` y `OptimizationManager`.

Reclasificación vigente H-L: `PoseGraphBuilder`/`PoseGraphProblem` son `1I`,
`OptimizationManager` solver/dry-run/replay/HTML es `1J`, apply/propagación y
publicación desde poses finales son `1K`, y `ValidatePostApply`/GT debug son
`1L`.

Interfaces clave:
- Consume: `orbslam3_msgs` (OrbMap del wrapper).
- Produce/expone: tipos para `GlobalMapBuilder` y APIs internas usadas por `orbslam3_server`.

Scripts / Ejecutables relevantes:
- `test_opt_graph_offline` (diagnóstico dry-run/HTML).

APIs clave: `RawMapDatabase`, `GlobalPoseStore`, `FiducialAnchorManager`, `OptimizationManager`, `GlobalMapBuilder`.

Config / Launch: no tiene launch; es librería C++ consumida por `orbslam3_server`.

Detalles y diseño en los MDs del paquete (raw_map_database.md, optimization_manager.md, ...).
