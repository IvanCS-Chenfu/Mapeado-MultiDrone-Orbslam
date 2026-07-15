# ADR 0003 — Fused landmarks y score global en el servidor

## Estado

Aceptada.

## Contexto

Cada dron ejecuta ORB-SLAM3 localmente y produce MapPoints. Esos MapPoints son útiles, pero no deben tratarse directamente como la salida global final.

Diferentes drones pueden observar el mismo punto físico y generar MapPoints distintos.

El mismo dron también puede generar duplicados si hay resets, relocalizaciones o submapas distintos.

Si se publica simplemente la unión bruta de todos los MapPoints de ORB-SLAM3, aparecerán problemas como:

- dobles paredes;
- puntos duplicados;
- ruido persistente;
- puntos de trayectoria;
- landmarks inestables;
- mezcla de submapas;
- mapa global poco útil para fases posteriores.

## Decisión

El mapa sparse global publicado debe basarse en una capa propia del servidor:

```text
FusedLandmarkTrack
```

o estructura equivalente.

El servidor debe fusionar observaciones equivalentes y publicar una representación global filtrada, estable y con score.

El score global pertenece al fused landmark del servidor, no al MapPoint individual de ORB-SLAM3.

## Motivo

ORB-SLAM3 local no sabe que existen otros drones ni otros submapas remotos.

Por tanto, la responsabilidad de fusionar landmarks equivalentes entre drones/submapas pertenece al servidor global.

El servidor dispone de información global:

- submapa de origen;
- KeyFrames observadores;
- matches de loops;
- inliers geométricos;
- score acumulado;
- evidencia positiva;
- evidencia negativa;
- estabilidad temporal;
- observaciones cruzadas;
- estado de fusión.

## Consecuencias

El wrapper debe publicar información rica de ORB-SLAM3, pero no debe calcular el score global.

El servidor debe calcular y actualizar el score global.

El mapa publicado debe evitar ser una simple unión de MapPoints.

Los fused landmarks deben poder:

- acumular observaciones;
- fusionarse con otros tracks;
- subir score si reciben soporte consistente;
- bajar score si reciben evidencia negativa;
- no publicarse si son poco fiables;
- conservar información de origen.

## Información de ORB-SLAM3 que sí debe usarse

El servidor puede usar:

- posición local de MapPoints;
- descriptores ORB;
- observaciones KeyFrame-feature;
- número de observaciones;
- found ratio;
- normal;
- distancias invariantes;
- KeyFrame de referencia;
- asociaciones MapPoint-KeyFrame;
- covisibilidad;
- BoW;
- FeatureVector;
- loop edges locales.

## Información que no debe confundirse con score global

No son score global:

- `observations_count`;
- `found_ratio`;
- número de KeyFrames locales;
- calidad local de ORB-SLAM3;
- que un punto exista en un único submapa;
- que un punto aparezca en una nube candidata.

Esas métricas pueden contribuir al score, pero no sustituyen al score del servidor.

## Reglas para Codex

Codex no debe mover el score global al wrapper.

Codex no debe añadir campos de score global a MapPoints locales salvo decisión explícita.

Codex no debe publicar la nube global como unión bruta si existe capa fused disponible.

Si se modifica la fusión, debe revisar:

- creación de tracks;
- actualización de tracks;
- criterios de publicación;
- penalización de outliers;
- fusión tras loops;
- uso de poses optimizadas.

## Ejemplos correctos

Correcto:

```text
Un MapPoint observado por varios KeyFrames y validado como inlier en un loop aumenta el score de su fused track.
```

Correcto:

```text
Varios MapPoints equivalentes de distintos drones se fusionan en un único landmark global.
```

Correcto:

```text
La posición publicada es una media ponderada o estimación robusta del fused track.
```

## Ejemplos incorrectos

Incorrecto:

```text
Publicar todos los MapPoints de todos los drones directamente.
```

Incorrecto:

```text
Tratar `found_ratio` como score global final.
```

Incorrecto:

```text
Calcular score global dentro del wrapper de ORB-SLAM3.
```

Incorrecto:

```text
Usar ground truth para decidir qué fused landmark es bueno.
```

## Relación con fases del pipeline

Esta decisión pertenece principalmente a:

- Fase 1: mapa sparse global multi-dron;
- Fase 2: pose exacta de drones sin GT;
- Fase 5: nube densa global.

La calidad de la nube sparse fused condicionará la calidad de la pose global y de la nube densa.

## Cuándo revisar esta decisión

Solo debe revisarse si se sustituye completamente la arquitectura del servidor global por otro backend que ya mantenga landmarks globales equivalentes.
