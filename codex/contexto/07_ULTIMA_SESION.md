# 07 — Última sesión

## 2026-07-16 — Implementación de `CovisibilityDatabase`

- Se añadió `CovisibilityDatabase` en `orbslam3_multi` con pares canónicos de
  `RawKeyFrameId`, soporte, fuentes confirmadas y poses relativas medida/current.
- `global_map_server` importa conexiones nativas de ORB-SLAM3 tras cada delta o
  full snapshot, reinicia la base en replay y publica logs `[F1M-COVIS-*]`.
- `PoseGraphBuilder` puede añadir aristas `SoftConsistency` de covisibilidad sin
  crear vértices ni sustituir las aristas temporales existentes.
- Se documentó la API en `codex/contexto/paquetes/orbslam3_multi/` y el
  historial de `1M`.
- No se ejecutó build ni simulación: este checkout no incluye el CMake de
  `orbslam3_multi` ni `orbslam3_msgs`, y la validación Gazebo debe realizarse
  en el workspace completo desde VS Code.

Conclusión: `1M` queda **a probar en simulación**. No iniciar `1N+` hasta
verificar la importación de `connected_keyframe_ids/weights` y los logs de
covisibilidad con datos reales.
