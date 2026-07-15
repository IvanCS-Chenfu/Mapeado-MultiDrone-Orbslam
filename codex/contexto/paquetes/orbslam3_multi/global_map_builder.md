# `GlobalMapBuilder`

## Rol

`GlobalMapBuilder` es la capa creada en `1F` para construir puntos sparse globales publicables.

Consulta:

- `RawMapDatabase`;
- `GlobalPoseStore`;
- `LandmarkScoreManager`.

Devuelve puntos en frame `world` solo para submapas que tienen `world_T_local` registrado en `GlobalPoseStore`.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/global_sparse_point.hpp
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
orbslam3_multi/src/global_map_builder.cpp
```

## Salida

El tipo principal de salida es `GlobalSparsePoint`.

Campos relevantes:

- `global_mappoint_id`;
- `drone_id`;
- `map_epoch`;
- `local_mappoint_id`;
- `x`, `y`, `z` en `world`;
- `score`;
- `observations`;
- `from_anchored_submap`;
- `is_fused`.

En `1F`, `is_fused=false` porque todavía no existe fusión real de landmarks.

## Comportamiento

- Recorre submapas raw de `RawMapDatabase`.
- Pide `world_T_local` a `GlobalPoseStore`.
- Salta submapas no anclados.
- Salta MapPoints `is_bad`.
- Salta posiciones locales no finitas.
- Pide score a `LandmarkScoreManager`.
- Aplica `min_score_to_publish` si el servidor lo configura.
- Desde `1K`, publica cada MapPoint desde la pose final del KeyFrame elegido:
  - prefiere `reference_keyframe_id` si existe y tiene pose en
    `GlobalPoseStore`;
  - si no, usa el primer KF observador con pose válida;
  - calcula `p_kf = raw_local_T_kf^-1 * p_local_raw`;
  - publica `p_world = final_world_T_kf * p_kf`.
- Si no hay KF publicable:
  - en submapas sin corrección de servidor, conserva el fallback legacy
    `world_T_local * p_local_raw`;
  - en submapas ya corregidos por servidor, salta el punto y lo contabiliza en
    `invalid_pose_skipped`, para evitar puntos sueltos publicados con una
    transformada rígida antigua.
- Ya no usa la última corrección heredable del submapa para mover MapPoints
  sin KF local válido, porque eso puede arrastrar zonas ancladas como una
  transformación rígida de todo el submapa.
- Contabiliza `server_corrected_points`, `keyframe_projected_points` y
  `fallback_submap_points`.

Esta política pertenece conceptualmente al apply/publicación de `1K` y busca que
RViz2 siga la misma deformación de KeyFrames que se ve en el HTML diagnóstico
del grafo. Ese HTML pertenece a `1J`, aunque algunos artefactos conserven
prefijo legacy `f1l_*`.

## Logs

Los logs los emite el servidor al publicar el resultado:

```text
[F1F-GLOBALMAP-BUILD]
[F1F-GLOBALMAP-SKIP-UNANCHORED]
[F1F-GLOBALMAP-POINT-STATS]
[F1F-GLOBALMAP-PUBLISH]
```

Desde la validación post-apply, el servidor añade un resumen diagnóstico:

```text
[F1L-GLOBALMAP-KF-PROJECTION]
```

En una optimización aceptada de submapa corregido, el valor esperado es
`fallback_submap_after=0`: los puntos deben salir de poses finales de KFs o
saltarse si no tienen KF publicable.

Evidencia validada en `1F`:

```text
[F1F-GLOBALMAP-BUILD] ... anchored_submaps=2 skipped_unanchored=1 raw_points=22394 candidate_points=22394 returned_points=22394
[F1F-GLOBALMAP-PUBLISH] ... topic=/global_sparse_cloud frame_id=world points_published=22394 ... score_field=true drone_id_field=true map_epoch_field=true
```

Desde `1K`, `[F1F-GLOBALMAP-POINT-STATS]` incluye `server_corrected_points`. En la validacion del 2026-07-10:

- `prueba_3.log` muestra publicaciones con `server_corrected_points>0` tras apply;
- `prueba_2.log` muestra publicaciones con `server_corrected_points>0` en replay/debug;
- `[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY]` aparece tras cada apply validado.

## Restricciones

- No crear anchors.
- No modificar `RawMapDatabase`.
- No modificar `GlobalPoseStore`.
- No calcular score por cuenta propia.
- No publicar submapas sin anchor.
- No implementar fusión real en `1F`.
- Desde `1K`, reconstruir los puntos publicados desde el KF elegido y su pose
  final; no modificar raw, scores ni anchors.
- Desde `1K`, no publicar puntos de un submapa corregido mediante fallback de
  `world_T_local` si no se puede asociar un KF válido.
