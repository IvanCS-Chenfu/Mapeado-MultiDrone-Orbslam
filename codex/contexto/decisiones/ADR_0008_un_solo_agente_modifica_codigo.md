# ADR 0008 — Un solo agente debe modificar código en cada tarea

## Estado

Aceptada.

## Contexto

Codex puede usar subagentes para explorar, revisar, analizar logs o documentar.

Sin embargo, el proyecto contiene archivos grandes y delicados, especialmente:

```text
orbslam3_server/src/global_map_server.cpp
```

Si varios agentes modifican código a la vez, pueden:

- pisar cambios;
- aplicar soluciones incompatibles;
- tocar rutas distintas sin coordinación;
- generar diffs demasiado grandes;
- romper compilación;
- complicar el diagnóstico.

## Decisión

En cada tarea de implementación, solo un agente debe modificar código.

Los demás agentes deben ser de solo lectura o limitarse a documentación.

## Roles recomendados

### Agente principal

Coordina la tarea y da el informe final.

### Explorador de repo

Solo lectura.

Funciones:

- localizar archivos;
- localizar funciones;
- explicar dependencias;
- proponer plan.

### Implementador de fase

Puede modificar código.

Debe ser el único agente que edite C++, Python, launch, CMake o package.xml.

### Revisor ROS 2

Solo lectura.

Funciones:

- revisar posibles errores de compilación;
- revisar parámetros ROS;
- revisar launch;
- revisar riesgos de regresión.

### Analizador de logs

Solo lectura.

Funciones:

- analizar logs;
- extraer eventos importantes;
- concluir si la fase funcionó.

### Curador de documentación

Puede modificar documentación, pero no código.

Funciones:

- actualizar `.md`;
- actualizar historial;
- actualizar última sesión;
- actualizar mapas de código.

## Reglas para Codex

Antes de implementar:

1. usar agentes de lectura si hace falta;
2. consolidar plan;
3. elegir un único agente implementador;
4. aplicar cambios mínimos;
5. revisar;
6. compilar;
7. simular;
8. documentar.

## Permitido

Permitido:

```text
explorador_repo + revisor_ros2 + implementador_fase
```

si solo `implementador_fase` escribe código.

Permitido:

```text
implementador_fase modifica código
curador_docs modifica documentación
```

si no editan los mismos archivos.

## Prohibido

Prohibido:

```text
dos implementadores modificando global_map_server.cpp en paralelo
```

Prohibido:

```text
un agente refactorizando mientras otro arregla D4
```

Prohibido:

```text
un agente cambiando mensajes mientras otro cambia servidor sin coordinación
```

## Cuándo revisar esta decisión

Solo debe revisarse si se introduce un sistema de control de cambios más avanzado que permita merges automáticos fiables entre agentes.
