# ADR 0005 — Wrapper ORB-SLAM3 y mensajes como interfaz estable

## Estado

Aceptada.

## Contexto

El wrapper ROS 2 de ORB-SLAM3 y el paquete `orbslam3_msgs` son la interfaz entre los drones y el servidor global.

El wrapper publica información local de ORB-SLAM3:

- pose local;
- KeyFrames;
- MapPoints;
- descriptores;
- BoW;
- FeatureVector;
- covisibilidad;
- observaciones;
- epoch del mapa;
- parámetros de cámara.

El servidor global depende de esa información para construir el mapa sparse global.

## Decisión

El wrapper y los mensajes actuales se consideran estables.

No deben rediseñarse durante fases de optimización del servidor salvo que exista una necesidad fuerte y explícita.

Las fases actuales deben priorizar corregir el servidor global antes de tocar la interfaz wrapper-servidor.

## Motivo

El wrapper ya publica la información necesaria para:

- reconstruir submapas locales;
- crear candidatos de loop;
- hacer matching por descriptores;
- usar BoW;
- reconstruir observaciones;
- crear fused landmarks;
- detectar cambios de epoch;
- pedir snapshots completos.

Rediseñar mensajes o wrapper durante una fase de optimización puede introducir errores nuevos y romper partes que ya funcionan.

## Consecuencias

Durante la Fase 1, las modificaciones normales deben concentrarse en:

- `orbslam3_server`;
- `orbslam3_multi`;
- configuración/launch;
- scripts de prueba;
- documentación.

El wrapper solo debe tocarse si:

- no publica un dato imprescindible;
- publica un dato erróneo;
- hay un bug confirmado en epoch;
- hay un bug confirmado en calibración;
- hay un error de compilación o integración.

## Reglas para Codex

Codex no debe modificar `orbslam3_msgs` o `orbslam3_ros2` como primera solución a un bug del servidor.

Codex debe justificar explícitamente cualquier cambio en mensajes o wrapper.

Si se modifica un mensaje ROS, Codex debe:

1. actualizar documentación;
2. compilar `orbslam3_msgs`;
3. compilar paquetes dependientes;
4. revisar serialización/deserialización;
5. revisar callbacks del servidor.

Si se modifica el wrapper, Codex debe:

1. revisar topics;
2. revisar servicios;
3. revisar `BuildOrbMap`;
4. revisar hashes de MapPoints/KeyFrames;
5. revisar publicación de `map_epoch`;
6. ejecutar prueba básica de publicación.

## Ejemplos correctos

Correcto:

```text
Corregir el guard de apply en el servidor sin tocar el wrapper.
```

Correcto:

```text
Añadir documentación sobre qué campos publica `OrbMap`.
```

Correcto:

```text
Tocar wrapper solo si falta un campo imprescindible para una fase futura.
```

## Ejemplos incorrectos

Incorrecto:

```text
Añadir score global al wrapper.
```

Incorrecto:

```text
Cambiar la estructura de `OrbMap` para evitar arreglar una lógica del servidor.
```

Incorrecto:

```text
Modificar ORB_SLAM3 interno sin diagnóstico claro.
```

## Cuándo revisar esta decisión

Debe revisarse si se entra en una fase donde realmente falten datos para Fase 2 o Fase 5, como observaciones necesarias para relocalización global o nube densa.
