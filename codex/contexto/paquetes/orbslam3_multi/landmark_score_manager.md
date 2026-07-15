# `LandmarkScoreManager`

## Rol

`LandmarkScoreManager` es la capa de scoring creada en `1F` dentro de `orbslam3_multi`.

Su responsabilidad es centralizar el score de cada `MapPoint` raw sin que el servidor ni otras clases hagan modificaciones directas de tipo `score += X`.

En `1F` solo usa información exportada por ORB-SLAM3:

- `observations_count`;
- `found_ratio`;
- `is_bad`;
- descriptor válido.

No usa ground truth, fiduciales, loops, fusión, optimización ni número de drones para modificar el score.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp
orbslam3_multi/src/landmark_score_manager.cpp
```

## Política de score en 1F

La puntuación inicial está acotada en `[0, 1]`.

Si el punto está marcado como `is_bad`, el score queda en `0`.

Para puntos válidos:

```text
score =
  0.55 * min(observations_count / 8, 1)
+ 0.35 * clamp(found_ratio, 0, 1)
+ 0.10 * descriptor_valid
```

`descriptor_valid` vale `1` si algún byte del descriptor es distinto de `0`; si no, vale `0`.

## Eventos

Eventos activos en `1F`:

- `NewMapPoint`;
- `OrbSlamQualityUpdate`;
- `MarkedBad`.

Eventos reservados para fases futuras:

- `FusionConfirmed`;
- `Reobserved`;
- `NotReobserved`;
- `OptimizationAccepted`;
- `GeometryInconsistent`.

Los eventos futuros existen para fijar la API, pero no cambian funcionalmente el score en `1F`.

## Logs

Los logs los emite `orbslam3_server/src/global_map_server.cpp` al alimentar y consultar el manager:

```text
[F1F-SCORE-INIT]
[F1F-SCORE-UPDATE-ORBSLAM]
[F1F-SCORE-STATS]
```

Evidencia validada en `1F`:

```text
[F1F-SCORE-STATS] ... tracked_points=22885 score_min=0.238 score_mean=0.557 score_max=1.000 bad_points=0
```

## Restricciones

- No usar GT para score.
- No derivar score en el servidor.
- No aplicar todavía eventos de fusión, loops, no reobservación ni optimización.
- No borrar ni fusionar `MapPoints` raw.
