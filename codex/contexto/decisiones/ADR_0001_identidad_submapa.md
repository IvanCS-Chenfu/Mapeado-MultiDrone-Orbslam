# ADR 0001 — Identidad de submapa mediante `(drone_id, map_epoch)`

## Estado

Aceptada.

## Contexto

En este proyecto, cada dron ejecuta ORB-SLAM3 localmente. ORB-SLAM3 puede perder tracking, resetear internamente, cambiar de mapa o iniciar un nuevo mapa local.

Por tanto, no es correcto identificar todo lo producido por un dron únicamente con `drone_id`.

Un mismo dron puede producir varios submapas distintos a lo largo de una ejecución.

## Decisión

La unidad correcta de identidad del sistema es:

```text
submapa = (drone_id, map_epoch)
```

Todo KeyFrame, MapPoint, landmark fusionado, loop, anchor, corrección, snapshot o estructura global que dependa del mapa local debe distinguir como mínimo:

```text
drone_id
map_epoch
id local del elemento
```

Ejemplos:

```text
GlobalKeyFrameId = (drone_id, map_epoch, keyframe_id_local)
GlobalMapPointId = (drone_id, map_epoch, mappoint_id_local)
```

## Motivo

ORB-SLAM3 puede generar nuevos mapas internos. Si el servidor global usa solo `drone_id`, podría mezclar KeyFrames o MapPoints de mapas diferentes.

Esto provocaría errores como:

- asociar MapPoints de épocas distintas;
- crear loops inválidos;
- fusionar landmarks que pertenecen a submapas diferentes;
- publicar una nube global incoherente;
- borrar o ocultar submapas históricos que ya estaban anclados;
- aplicar correcciones de pose al submapa equivocado.

## Consecuencias

El servidor global debe mantener submapas históricos aunque el dron empiece un nuevo `map_epoch`.

Un submapa anclado no debe desaparecer cuando el dron cambia de epoch.

Toda estructura de almacenamiento debe respetar esta separación.

## Reglas para Codex

Codex no debe implementar código que identifique un mapa local solo por `drone_id`.

Si modifica estructuras globales, debe comprobar si también debe incluir `map_epoch`.

Si encuentra código legacy que use solo `drone_id`, no debe cambiarlo masivamente sin una fase específica, pero debe evitar extender esa lógica.

## Ejemplos correctos

Correcto:

```cpp
auto submap_key = std::make_pair(drone_id, map_epoch);
```

Correcto:

```text
submap_mirror[drone_id][map_epoch]
```

Correcto:

```text
keyframe_global_id = drone_id + map_epoch + local_kf_id
```

## Ejemplos incorrectos

Incorrecto:

```text
El mapa actual del dron 2 representa todo lo que ha visto el dron 2.
```

Incorrecto:

```text
Si aparece un nuevo epoch, borrar el mapa anterior del dron.
```

Incorrecto:

```text
Fusionar KeyFrames solo porque tienen el mismo local_id y drone_id.
```

## Cuándo revisar esta decisión

Solo debe revisarse si se abandona ORB-SLAM3 local por dron o si se introduce un sistema completamente diferente de gestión de mapas que garantice identidades globales persistentes.
