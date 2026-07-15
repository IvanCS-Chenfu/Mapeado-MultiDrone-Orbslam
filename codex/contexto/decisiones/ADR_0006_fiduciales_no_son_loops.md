# ADR 0006 — Los fiduciales son observaciones absolutas, no loops

## Estado

Aceptada.

## Contexto

El sistema usa dos tipos de información geométrica importantes:

1. loops entre zonas/submapas;
2. fiduciales.

Un loop es una relación relativa entre dos partes del mapa.

Un fiducial es una observación absoluta en el mundo.

Mezclar ambos conceptos puede provocar optimizaciones incorrectas.

## Decisión

Los fiduciales no deben tratarse como loops.

Un fiducial debe modelarse como una observación absoluta o prior fuerte sobre una pose o ventana concreta.

Un loop debe modelarse como una restricción relativa entre KeyFrames, subnubes o submapas.

## Motivo

El error de loop y el error de fiducial tienen significados distintos.

Un loop responde a:

```text
¿Estas dos zonas del mapa representan el mismo lugar físico?
```

Un fiducial responde a:

```text
¿Esta zona observada del mapa coincide con una pose absoluta conocida?
```

Si se mezclan, el sistema puede:

- aplicar optimización local cuando necesita global;
- aplicar optimización global cuando solo necesita local;
- meter priors fuertes en momentos incorrectos;
- congelar trayectorias enteras;
- confundir deuda fiducial con loop closure;
- reactivar rutas legacy peligrosas.

## Consecuencias

Un loop local con error alto debe crear una tarea de optimización local.

Un fiducial con error alto debe generar evidencia para una futura tarea global/fiducial, no entrar como prior fuerte inmediato.

`FIDUCIAL_GLOBAL_DEBT` debe permanecer en cuarentena hasta la fase correspondiente.

## Reglas para Codex

Codex no debe usar el error fiducial como si fuera `LOOP-SUBCLOUD-ERROR`.

Codex no debe usar loops como sustituto de fiduciales.

Codex no debe añadir `FIDUCIAL_GLOBAL_DEBT` como prior fuerte salvo fase explícita.

Codex no debe reactivar `force_pose_graph_optimization_` para resolver deuda fiducial.

## En Fase 1 actual

La planificación activa `1A`-`1X` mantiene la separación conceptual:

- `1E` y `1H` gestionan observaciones fiduciales y tareas fiduciales;
- `1N`-`1Q` gestionan candidatos BoW, verificación por subnubes, decisión de loop
  y optimización por loop;
- `1I`-`1L` construyen, prueban, aplican, validan y revierten optimizaciones
  reutilizables para fiduciales y loops.

Las subfases `12R-*` quedan como legacy.

## Ejemplos correctos

Correcto:

```text
Un loop con subcloud error alto crea `LOCAL_LOOP_OPT_TASK`.
```

Correcto:

```text
Un fiducial con error alto queda registrado como deuda fiducial para fase futura.
```

Correcto:

```text
Un submapa sin fiducial puede heredar anclaje mediante loop con otro submapa ya anclado.
```

## Ejemplos incorrectos

Incorrecto:

```text
Tratar un fiducial como una loop edge normal.
```

Incorrecto:

```text
Optimizar globalmente solo porque apareció un loop local.
```

Incorrecto:

```text
Forzar pose graph legacy cada vez que aparece deuda fiducial.
```

## Cuándo revisar esta decisión

Debe revisarse al diseñar formalmente la fase de optimización global/fiducial, pero incluso entonces la separación conceptual debe mantenerse.
