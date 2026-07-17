# `LoopDetector`

## Estado

Infraestructura inicial añadida en `1N`, pero la subfase sigue **por
implementar**. Este checkout no contiene el contrato de BoW/`FeatureVector` de
`OrbKeyFrame`, por lo que el detector no inventa scores ni candidatos.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/loop_candidate.hpp
orbslam3_multi/include/orbslam3_multi/loop_detector.hpp
orbslam3_multi/src/loop_detector.cpp
```

## API vigente

`ProcessNewKeyFrame` recibe un `RawKeyFrameId`, `RawMapDatabase`,
`GlobalPoseStore` opcional y `CovisibilityDatabase`. Comprueba la existencia del
KF y devuelve `bow_data_unavailable_in_current_checkout` mientras no se pueda
leer BoW de forma verificable.

La implementación posterior debe indexar/consultar BoW, aplicar filtros y usar
`HasConfirmedEdge` antes de devolver candidatos a `1O`. BoW nunca confirma un
loop y este componente no modifica poses, fusiona landmarks ni ejecuta RANSAC.
