# Subfase 1M — `CovisibilityDatabase`: covisibilidad confirmada para optimización

## Estado

```text
a probar en simulación
```

## Objetivo técnico

Crear una base de datos de covisibilidad confirmada en `orbslam3_multi`.

Esta base no guarda candidatos ni relaciones dudosas. Si una relación existe en
`CovisibilityDatabase`, se considera confirmada y usable como restricción:

```text
ORB-SLAM3_NATIVE intra-dron
SERVER_LOOP_GEOMETRIC inter/intra-dron validado por geometría
```

No usar campo `state`: los candidatos BoW, rechazos y cooldowns pertenecen al
detector/gestor de loops, no a esta base.

## Semántica

Una arista de covisibilidad representa:

```text
dos KeyFrames observan/representan la misma zona y no deben separarse libremente
```

No es un fiducial y no sustituye a `RawMapDatabase` ni a `GlobalPoseStore`.

`RawMapDatabase` conserva datos crudos ORB-SLAM3. `CovisibilityDatabase` es una
capa consultable por:

- `LoopDetector`, para evitar repetir trabajo si una relación ya está confirmada;
- `SubcloudLoopVerifier`/`LoopDecisionManager`, para registrar loops confirmados;
- `PoseGraphBuilder`, para añadir aristas extra en optimizaciones fiduciales y
  de loop.

## Modelo de datos recomendado

```text
CovisibilityEdge
  kf_a = (drone_id, map_epoch, keyframe_id)
  kf_b = (drone_id, map_epoch, keyframe_id)
  weight
  source = ORBSLAM3_NATIVE | SERVER_LOOP_GEOMETRIC
  relative_pose_measured
  relative_pose_current
  information_weight
  shared_mappoints_or_inliers
  created_arrival_id
  updated_revision
```

Reglas:

- `relative_pose_measured` conserva la medición original y no se sobrescribe.
- `relative_pose_current` puede actualizarse tras optimizaciones aceptadas.
- `weight` mide soporte visual/geometrico: MapPoints compartidos o inliers.
- `information_weight` traduce ese soporte a fuerza de arista para el grafo.
- La relación debe guardarse con orden canónico para evitar duplicados
  `A-B`/`B-A`.

## Fuentes

### ORB-SLAM3 nativo

ORB-SLAM3 calcula covisibilidad intra-mapa contando MapPoints compartidos entre
KeyFrames. En el mensaje `OrbKeyFrame` ya existen los campos:

```text
connected_keyframe_ids
connected_keyframe_weights
```

La subfase debe importar esas relaciones desde la base principal/raw si están
disponibles, aplicando un umbral mínimo para no guardar covisibilidad débil.

### Loops confirmados por servidor

Cuando un loop se confirme geométricamente en fases posteriores, el servidor
debe añadir una arista:

```text
source = SERVER_LOOP_GEOMETRIC
weight = inliers o soporte equivalente
relative_pose_measured = transformación verificada por RANSAC/subnube
```

No insertar nada por BoW solo.

## Cambios requeridos

1. Crear `CovisibilityDatabase` en `orbslam3_multi`.
2. Añadir tipos de ID basados en `(drone_id, map_epoch, keyframe_id)`.
3. Importar covisibilidad ORB-SLAM3 desde los KeyFrames raw.
4. Permitir consultas:
   ```text
   HasConfirmedEdge(a, b)
   GetEdge(a, b)
   GetNeighbors(kf, min_weight)
   GetEdgesForWindow(kfs, min_weight)
   ```
5. Permitir inserción de loops confirmados por geometría.
6. Exponer una forma de actualizar `relative_pose_current` y `updated_revision`
   tras una optimización aceptada.
7. Añadir logs compactos:
   ```text
   [F1M-COVIS-IMPORT]
   [F1M-COVIS-EDGE-ADD]
   [F1M-COVIS-EDGE-UPDATE]
   [F1M-COVIS-QUERY]
   [F1M-COVIS-SUMMARY]
   ```

## Archivos permitidos

- `orbslam3_multi/include/orbslam3_multi/covisibility_database.hpp`
- `orbslam3_multi/src/covisibility_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/pose_graph_builder.hpp`
- `orbslam3_multi/src/pose_graph_builder.cpp`
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_server/src/global_map_server.cpp` solo para integración mínima
- docs de paquete e historial de `1M`

## Archivos prohibidos

- No modificar `ORB_SLAM3/`.
- No modificar `orbslam3_ros2` ni `orbslam3_msgs` salvo que se demuestre que los
  campos existentes no se rellenan y el usuario lo apruebe.
- No usar ground truth.
- No guardar candidatos BoW, rechazos ni estados pendientes en esta base.
- No mover poses en `RawMapDatabase`.
- No fusionar MapPoints raw.

## Pruebas mínimas

- Unit/replay: importar covisibilidad ORB-SLAM3 desde KeyFrames con
  `connected_keyframe_ids/weights`.
- Unit/replay: consultar `HasConfirmedEdge(a,b)` con orden canónico.
- Unit/replay: insertar una arista `SERVER_LOOP_GEOMETRIC` sintética con pose
  relativa y recuperarla.
- Integración: `PoseGraphBuilder` puede pedir aristas de una ventana sin crear
  vértices nuevos obligatorios.

## Criterios de éxito

`CONSEGUIDA` solo si:

- compila `orbslam3_multi` y, si se toca integración, `orbslam3_server`;
- la base guarda solo relaciones confirmadas;
- ORB-SLAM3_NATIVE se importa con umbral mínimo;
- no hay campo `state`;
- `relative_pose_measured` no se sobrescribe;
- `relative_pose_current` puede actualizarse de forma controlada;
- `LoopDetector` y `PoseGraphBuilder` tienen API documentada para consultar la
  base en subfases posteriores;
- la documentación e historial quedan actualizados.

## Estado de implementación

La implementación está preparada para validación en simulación: importa la
covisibilidad nativa desde los `OrbKeyFrame` raw, expone las consultas e integra
las aristas confirmadas en ventanas de `PoseGraphBuilder`. Falta comprobar en
Gazebo/replay que `connected_keyframe_ids` y `connected_keyframe_weights`
llegan poblados desde el wrapper, que el umbral configurado produce aristas
útiles y que los marcadores `[F1M-COVIS-*]` son coherentes. No declarar
`CONSEGUIDA` hasta ejecutar esas pruebas.
