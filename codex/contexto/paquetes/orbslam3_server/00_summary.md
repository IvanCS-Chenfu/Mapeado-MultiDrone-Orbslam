# 00_summary — orbslam3_server

Resumen: Servidor global adaptador ROS 2: recibe `OrbMap` delta, inserta en `RawMapDatabase`, gestiona replay, y coordina optimización/validation pipelines.

Frontera H-L vigente: invoca construcción de grafo `1I`, dry-run/dump/HTML
`1J`, apply/publicación `1K` y diagnóstico post-apply `1L`. Algunos parámetros
`f1l_*` son legacy aunque pertenezcan conceptualmente a `1J` o `1K`.

Entradas: `/dron_X/orbslam/orb_map_delta` (OrbMap delta), clientes `GetOrbMap` para snapshots.

Ejecutables: `global_map_server` (servidor activo), `global_pose_corrector` (corrector heredado).

Parámetros importantes: `rawdb_record_enabled`, `rawdb_replay_enabled`, `body_T_camera_*`, `f1k_apply_enabled`, `f1l_validation_enabled`.

Artefactos auxiliares: logs en `codex/archivos_auxiliares/logs/`, HTML en
`codex/archivos_auxiliares/html/` y replay/dumps en
`codex/archivos_auxiliares/repeticiones/`.

Relación: consume `orbslam3_msgs`, usa `orbslam3_multi` para backend, publica `/global_sparse_cloud` desde `GlobalMapBuilder`.

Detalles en los MDs `launches.md`, `global_map_server.md` y otros del directorio.
