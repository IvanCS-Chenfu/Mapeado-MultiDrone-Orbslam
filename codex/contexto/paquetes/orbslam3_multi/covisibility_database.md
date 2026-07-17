# `CovisibilityDatabase`

## Rol

`CovisibilityDatabase` es la capa de `1M` que conserva relaciones confirmadas
entre `KeyFrames`. No sustituye los datos locales de `RawMapDatabase` ni las
poses globales de `GlobalPoseStore`.

Una arista existe únicamente si procede de covisibilidad nativa de ORB-SLAM3 o
de un loop confirmado por geometría. No almacena candidatos BoW, rechazos ni
estados pendientes.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/covisibility_database.hpp
orbslam3_multi/src/covisibility_database.cpp
```

## Datos y API

`CovisibilityEdge` usa extremos `RawKeyFrameId`, por lo que conserva siempre
`(drone_id, map_epoch, local_kf_id)`. Los extremos se normalizan a orden
canónico para que `A-B` y `B-A` no dupliquen una relación.

La arista guarda soporte, fuente, `relative_pose_measured` inmutable,
`relative_pose_current` actualizable, `information_weight`, `arrival_id` y
revisión. Las APIs disponibles son:

- `ImportOrbslam3Native` para importar conexiones de `OrbKeyFrame` aplicando un
  umbral;
- `AddConfirmedLoopEdge` para una relación ya validada por geometría;
- `HasConfirmedEdge`, `GetEdge`, `GetNeighbors` y `GetEdgesForWindow` para
  consumidores de loops y optimización;
- `UpdateRelativePoseCurrent`, que no modifica la medición original;
- `Clear` y `GetStats` para replay y observabilidad.

`PoseGraphBuilder` recibe opcionalmente esta base y añade aristas
`SoftConsistency` solo entre controles que ya están en la ventana: no crea
vértices ni reemplaza las aristas temporales de `1I`.

## Integración y logs

`global_map_server` importa covisibilidad después de cada delta y full snapshot
y reinicia la base al iniciar replay. El parámetro
`f1m_covisibility_min_weight` fija el soporte mínimo, con valor predeterminado
`15.0`.

Los logs esperados son:

```text
[F1M-COVIS-IMPORT]
[F1M-COVIS-EDGE-ADD]
[F1M-COVIS-EDGE-UPDATE]
[F1M-COVIS-QUERY]
[F1M-COVIS-SUMMARY]
```

## Estado

Implementado y pendiente de prueba en simulación/replay. Antes de cerrar `1M`
hay que confirmar que `connected_keyframe_ids` y `connected_keyframe_weights`
llegan rellenos desde el wrapper, que el filtrado por umbral no elimina toda la
covisibilidad útil y que `PoseGraphBuilder` incorpora únicamente aristas de
ventanas válidas.
