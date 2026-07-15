# `global_map_server_antiguo.cpp`

## Rol

`orbslam3_server/src/global_map_server_antiguo.cpp` es la copia congelada creada en `1B` del servidor monolítico anterior.

No es el servidor activo.

No se compila desde `orbslam3_server/CMakeLists.txt`.

## Qué conserva

Conserva la lógica previa de:

- recepción de `OrbMap`;
- snapshots completos;
- mirrors activos e históricos;
- fiduciales simulados;
- anchors;
- BoW;
- subcloud loops;
- fused tracks;
- scoring heredado;
- optimización local;
- publicación de nubes, correcciones y corrected keyframes.

## Header heredado

El header:

```text
orbslam3_server/include/orbslam3_server/global_map_server.hpp
```

también pertenece al diseño anterior. En `1B`, el servidor activo no lo incluye.

## Reglas

- No añadir funcionalidad nueva aquí.
- No compilarlo.
- No usarlo desde código activo.
- Consultarlo solo como referencia durante la migración de Fase 1.

Si una subfase posterior necesita migrar una pieza, debe copiar el comportamiento mínimo necesario hacia componentes nuevos y validarlo con build, simulación y logs.
