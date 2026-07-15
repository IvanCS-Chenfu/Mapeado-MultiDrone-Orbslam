# 00_summary — ORB_SLAM3

Resumen: Librería externa ORB-SLAM3 usada localmente en cada dron (stereo/mono). No modificar sin justificación.

Interfaces clave:
- Entradas: imágenes cámara (stereo/mono).
- Salidas extraídas por el wrapper: `pose_local`, `OrbMap` (deltas), snapshots.

Scripts / Ejecutables: ninguno propio (biblioteca C++ consumida por `orbslam3_ros2`).

APIs públicas relevantes: tipos `KeyFrame`, `MapPoint`, `BoW` internos (accedidos por wrapper).

Configuración / Launch: calibraciones en `codex/contexto/paquetes/dron_individual/config/orbslam/` y `vocabulary/ORBvoc.txt`.

Detalles completos en: otros MDs dentro del mismo directorio (no eliminados).
