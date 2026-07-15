# Subfase 1A — Baseline del servidor actual antes de la migración

## Estado

```text
realizado
```

Baseline capturada el 2026-07-03 con conclusión `CONSEGUIDA`.

Evidencia principal:

- `BUILD-EXIT-CODE 0`;
- `SIM-DONE prueba=1 success=true`;
- `prueba_1.log` completo y `prueba_1.reduced.log` generado;
- historial actualizado en `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.

---

## Objetivo técnico

Capturar una fotografía verificable del comportamiento actual de la Fase 1 antes de empezar la migración del servidor monolítico hacia una arquitectura donde `orbslam3_server` sea un adaptador ROS 2 y `orbslam3_multi` contenga el backend algorítmico.

Esta subfase **no corrige el servidor actual** y **no implementa la arquitectura nueva**. Su objetivo es ejecutar la simulación con el código actual, generar logs completos/reducidos y documentar claramente:

- si los wrappers publican datos ORB-SLAM3;
- si el servidor recibe `OrbMap` delta o snapshots;
- si se crean o detectan submapas por `(drone_id, map_epoch)`;
- si aparece lógica de fiduciales;
- si aparece lógica de fused map, loops u optimización;
- si se publica alguna nube sparse global;
- qué errores, fallos, duplicaciones, dobles paredes o comportamientos incorrectos existen ahora;
- qué se observa en RViz2.

La subfase puede quedar `CONSEGUIDA` aunque el servidor actual tenga errores funcionales, siempre que esos errores queden documentados con evidencia de logs y, si aplica, observación visual.

---

## Contexto obligatorio a leer

Antes de ejecutar esta subfase, Codex debe leer:

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/02_REGLAS_TECNICAS.md`, si existe
- `codex/contexto/06_MAPA_CODIGO.md`, si existe
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`, si existe
- documentación de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/orbslam3_ros2/`
  - `codex/contexto/paquetes/orbslam3_msgs/`
  - `codex/contexto/paquetes/simulacion_dron/`
  - `codex/contexto/paquetes/dron_individual/`, si existe
- ADRs relacionados en `codex/contexto/decisiones/`, si existen

---

## Diagnóstico de partida

Se va a reiniciar la Fase 1 para reducir progresivamente `global_map_server.cpp` y mover la lógica algorítmica hacia clases de `orbslam3_multi`.

El servidor actual no se considera completamente correcto. Puede tener problemas en:

- publicación de mapa global;
- fiduciales;
- fused landmarks;
- loops;
- optimización local;
- dobles paredes;
- separación entre submapa anclado y KF realmente fiducial;
- mezcla de lógica ROS con lógica algorítmica.

Esta subfase debe dejar constancia de ese estado inicial. No debe intentar arreglarlo.

Marcadores relevantes que pueden existir actualmente y que deben buscarse si aparecen:

```text
PIPE0-WRAPPER-TRACK
PIPE0-WRAPPER-DELTA-PUB
PIPE0-WRAPPER-FULL-SNAPSHOT
PIPE0-WRAPPER-EPOCH-PUBLISH
WRAPPER-EPOCH
ORBMAP
GLOBAL-MAP
GLOBALMAP
SUBMAP
FID
FIDUCIAL
FUSED
LOOP
SUBCLOUD
LOCAL-OPT
LOCAL-POSE-GRAPH
POINTCLOUD
MAP-CORRECTION
CORRECTED-KEYFRAME
SCENARIO-RUNNER
ERROR
FATAL
Segmentation fault
Killed
```

Si alguno de esos marcadores no existe, no se debe concluir automáticamente que la funcionalidad no existe. Primero se debe revisar el log completo `prueba_1.log` antes de sacar conclusiones.

---

## Archivos permitidos a modificar

Por defecto, esta subfase **no modifica código fuente**.

Archivos permitidos:

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/logs/prueba_1.log` generado por herramienta
- `codex/archivos_auxiliares/logs/prueba_1.reduced.log` generado por herramienta
- `codex/archivos_auxiliares/logs/colcon_build.log` generado por herramienta
- `codex/archivos_auxiliares/logs/colcon_build.reduced.log` generado por herramienta, si falla build
- `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`

Excepción limitada:

Solo si tras ejecutar una primera simulación no aparece ninguna evidencia mínima de recepción/publicación por falta de logs, se permite añadir logs diagnósticos mínimos en:

- `orbslam3_server/src/global_map_server.cpp`
- `orbslam3_ros2/src/stereo-slam-node.cpp`, solo si fuera imprescindible para confirmar publicación del wrapper

Los logs añadidos deben tener prefijo:

```text
[F1A-BASELINE-...]
```

Si se añaden logs, no se debe corregir comportamiento funcional ni reestructurar código.

---

## Archivos prohibidos

No modificar en esta subfase:

- `orbslam3_server/src/global_map_server.cpp`, salvo la excepción limitada de logs mínimos
- `orbslam3_server/src/global_map_server_antiguo.cpp`, si ya existiera
- archivos de `orbslam3_multi`, salvo documentación si fuese imprescindible
- mensajes de `orbslam3_msgs`
- wrapper `orbslam3_ros2`, salvo la excepción limitada de logs mínimos
- `ORB_SLAM3/`
- `ORB_SLAM3_MULTI/`
- CMake/package.xml salvo que el build esté roto por un problema menor y evidente no relacionado con lógica
- cualquier archivo de subfases futuras
- cualquier código para crear `RawMapDatabase`, `GlobalPoseStore`, `FiducialAnchorManager`, `LoopDetector`, `FusionManager`, `PoseGraphBuilder` u `OptimizationManager`

No renombrar todavía:

```text
orbslam3_server/src/global_map_server.cpp
```

El renombrado a `global_map_server_antiguo.cpp` pertenece a la subfase 1B.

---

## Funciones, clases o nodos que hay que localizar

Codex debe localizar, sin modificar lógica:

- nodo servidor actual de `orbslam3_server`
- archivo actual `orbslam3_server/src/global_map_server.cpp`
- subscribers de `OrbMap` delta
- clientes del servicio `orbslam/get_full_map`, si existen
- publishers de nube global/fused map
- publishers de `map_correction` o `corrected_keyframes`, si existen
- lógica actual de fiduciales, si existe
- callbacks de GT simulado, si existen
- launch principal:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

- nodo de escenario usado por `run_simulation.sh`
- topics relevantes de wrapper:

```text
/dron_X/orbslam/orb_map_delta
/dron_X/orbslam/pose_local
/dron_X/orbslam/get_full_map
```

Los nombres exactos pueden variar según namespace. Codex debe confirmarlos mediante búsqueda estática o logs.

---

## Cambios requeridos

### Cambio 1 — Crear o reutilizar trayectoria de prueba baseline

Intención:

Ejecutar una simulación corta que obligue al menos a un dron a producir KeyFrames y, si el escenario lo permite, acercarse a un fiducial.

Archivo probable:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Condición de seguridad:

- Usar el formato real de trayectorias del proyecto.
- Si ya existe una trayectoria válida en historiales o pruebas anteriores, reutilizarla o adaptarla mínimamente.
- No crear una trayectoria compleja de loops todavía.

Log de validación:

```text
SCENARIO-RUNNER
GOAL
success=true
```

### Cambio 2 — Ejecutar build baseline

Intención:

Comprobar que el estado actual compila antes de capturar la simulación.

Condición de seguridad:

- No corregir lógica funcional.
- Si falla el build, reducir log y diagnosticar el primer error real.
- Solo corregir fallos triviales de build si impiden obtener baseline y no cambian comportamiento funcional.

Log de validación:

```text
colcon_build.log
colcon_build.reduced.log
```

### Cambio 3 — Ejecutar simulación baseline

Intención:

Obtener `prueba_1.log` completo del comportamiento actual.

Condición de seguridad:

- No repetir Gazebo innecesariamente si el log completo ya contiene evidencia suficiente.
- Si el reducido no muestra información suficiente, revisar primero el completo.

Log de validación:

```text
prueba_1.log
prueba_1.reduced.log
```

### Cambio 4 — Documentar comportamiento actual en historial

Intención:

Dejar constancia de lo que funciona y lo que falla antes de migrar.

Archivo:

```text
codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md
```

Debe registrar:

- fecha;
- subfase;
- archivos modificados;
- paquetes compilados;
- resultado de build;
- prueba Gazebo ejecutada;
- patrones de reducción usados;
- evidencia positiva;
- evidencia negativa;
- observación en RViz2, si se pudo observar;
- conclusión `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`;
- recomendación para subfase 1B.

---

## Cambios prohibidos

No hacer en esta subfase:

- no arreglar loops;
- no arreglar fiduciales;
- no crear `RawMapDatabase`;
- no crear `GlobalPoseStore`;
- no crear clases nuevas en `orbslam3_multi`;
- no mover lógica desde el servidor;
- no renombrar `global_map_server.cpp`;
- no borrar código legacy;
- no rediseñar mensajes;
- no cambiar el wrapper salvo logs mínimos imprescindibles;
- no usar ground truth para mapa, loops, fused score o nube final;
- no declarar la fase 1A fallida solo porque el mapa actual sea incorrecto, si el objetivo de baseline se cumplió.

---

## Paquetes a compilar

Comando recomendado:

```bash
./codex/herramientas/build_selected_packages.sh \
  orbslam3_msgs \
  orbslam3_multi \
  orbslam3_ros2 \
  orbslam3_server \
  dron_individual \
  simulacion_dron \
  lib_tray
```

Si alguno de esos paquetes no existe en el workspace real, Codex debe quitarlo justificándolo en el historial.

Si falla:

```bash
./codex/herramientas/reduce_build_log.sh
```

Después debe leerse:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

Si el reducido no basta, revisar:

```text
codex/archivos_auxiliares/logs/colcon_build.log
```

---

## Pruebas Gazebo requeridas

### Prueba 1 — Baseline servidor actual con llegada a fiducial si es posible

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. esperar arranque completo de Gazebo, wrappers y servidor;
2. mover al menos un dron lo suficiente para que ORB-SLAM3 genere KeyFrames;
3. si el escenario tiene fiducial accesible, mover ese dron hasta el radio del primer fiducial;
4. esperar varios segundos para que el wrapper publique deltas y el servidor procese;
5. si hay segundo dron disponible sin aumentar mucho la prueba, moverlo una trayectoria corta para confirmar multi-dron básico;
6. terminar el escenario con `success=true` si los goals fueron alcanzados.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observación esperada en RViz2, si se puede observar manualmente:

```text
- si aparece nube sparse global;
- si la nube está en world o parece descolocada;
- si aparecen puntos de uno o varios drones;
- si la nube cambia al llegar al fiducial;
- si aparecen dobles paredes evidentes;
- si hay saltos grandes de mapa;
- si desaparece la nube durante la simulación;
- si aparecen markers/frames de drones o fiduciales.
```

La observación RViz2 debe anotarse en historial. Si no se pudo abrir RViz2, anotarlo explícitamente y no inventar observaciones visuales.

---

## Patrones de reducción de logs

### Prueba 1

Usar:

```text
SCENARIO-RUNNER|GOAL|RESULT|success|PIPE0-WRAPPER|WRAPPER|ORBMAP|GLOBAL-MAP|GLOBALMAP|SUBMAP|FID|FIDUCIAL|FUSED|LOOP|SUBCLOUD|LOCAL-OPT|LOCAL-POSE-GRAPH|POINTCLOUD|MAP-CORRECTION|CORRECTED|ERROR|FATAL|Segmentation fault|Killed
```

Comando:

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 1 \
  --patterns "SCENARIO-RUNNER|GOAL|RESULT|success|PIPE0-WRAPPER|WRAPPER|ORBMAP|GLOBAL-MAP|GLOBALMAP|SUBMAP|FID|FIDUCIAL|FUSED|LOOP|SUBCLOUD|LOCAL-OPT|LOCAL-POSE-GRAPH|POINTCLOUD|MAP-CORRECTION|CORRECTED|ERROR|FATAL|Segmentation fault|Killed"
```

Si `prueba_1.reduced.log` queda demasiado grande, Codex puede crear una segunda reducción más específica, pero debe conservar `prueba_1.log` completo.

Si el reducido no contiene evidencia suficiente, revisar `prueba_1.log` completo antes de repetir simulación o concluir que falta un marcador.

---

## Criterio de éxito

La subfase se considera `CONSEGUIDA` si:

1. el build devuelve `0` o, si el build ya estaba validado y no hubo cambios de código, se justifica claramente por qué no fue necesario recompilar;
2. la simulación se ejecuta y genera `codex/archivos_auxiliares/logs/prueba_1.log`;
3. se genera `codex/archivos_auxiliares/logs/prueba_1.reduced.log`;
4. `scenario_runner_node` se ejecuta;
5. los goals de la trayectoria terminan con `success=true`, salvo que el objetivo explícito sea capturar un fallo de arranque;
6. el historial documenta al menos:
   - publicación del wrapper o ausencia de ella;
   - recepción del servidor o ausencia de ella;
   - evidencia de submapas/KFs/MPs o ausencia de ella;
   - evidencia de fiduciales o ausencia de ella;
   - evidencia de nube publicada o ausencia de ella;
   - errores graves encontrados o confirmación de que no aparecen;
7. queda clara la lista de comportamientos a preservar en la migración;
8. queda clara la lista de problemas heredados que no deben confundirse con fallos de la nueva arquitectura;
9. el historial queda actualizado.

Importante:

El servidor actual puede funcionar mal. Eso no impide marcar `CONSEGUIDA` si la baseline queda capturada con suficiente evidencia.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no se puede generar ningún log útil de simulación;
- el build falla y no se puede ejecutar la prueba;
- `run_simulation.sh` falla antes de lanzar el sistema y no deja evidencia diagnóstica suficiente;
- el historial no queda actualizado;
- no se puede saber qué ocurrió en el servidor actual.

La subfase debe marcarse como `PARCIAL` si:

- compila;
- la simulación arranca;
- hay logs útiles;
- pero no se consigue que el escenario llegue al fiducial o no se puede comprobar RViz2;
- aun así queda evidencia suficiente del flujo wrapper-servidor.

La subfase debe marcarse como `BLOQUEADA` si:

- falta una dependencia externa crítica;
- Gazebo no arranca por un problema de entorno no corregible en esta subfase;
- el workspace no contiene los paquetes necesarios;
- no existe ninguna trayectoria ejecutable y no se puede inferir el formato de `tray_prueba_1.yaml` desde el proyecto.

---

## Documentación a actualizar

Actualizar obligatoriamente:

```text
codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md
```

Añadir una entrada con:

```text
## Subfase 1A — Baseline servidor actual

Fecha:
Objetivo:
Archivos modificados:
Paquetes compilados:
Resultado build:
Prueba Gazebo:
Patrones de reducción:
Evidencia positiva:
Evidencia negativa:
Observación RViz2:
Conclusión:
Siguiente paso recomendado:
```

Actualizar también:

```text
codex/contexto/01_ESTADO_ACTUAL.md
```

Solo para reflejar que la Fase 1 reiniciada está en `1A` o, si se consigue, que la siguiente subfase será `1B`.

Si se modificó código para añadir logs mínimos, actualizar además la documentación correspondiente del paquete tocado:

```text
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/orbslam3_ros2/
```

Si no se modificó código, no es necesario actualizar documentación de paquetes, salvo que el historial revele una discrepancia importante entre documentación y comportamiento actual.

No marcar esta subfase como `realizado` en el pipeline hasta que exista entrada de historial con conclusión clara.

---

## Resultado esperado de la subfase

Al terminar esta subfase debe existir una baseline clara del servidor antiguo:

```text
codex/archivos_auxiliares/logs/prueba_1.log
codex/archivos_auxiliares/logs/prueba_1.reduced.log
codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md
```

La baseline debe responder, con evidencia, a estas preguntas:

1. ¿Publican los wrappers `OrbMap` delta?
2. ¿Recibe el servidor esos mapas?
3. ¿Se distinguen `drone_id` y `map_epoch`?
4. ¿Aparece algún fiducial?
5. ¿Se calcula alguna transformación o anclaje?
6. ¿Se publica alguna nube sparse global?
7. ¿Aparecen fused landmarks?
8. ¿Aparecen loops/subclouds?
9. ¿Aparecen optimizaciones locales?
10. ¿Qué falla actualmente?
11. ¿Qué se ve en RViz2?
12. ¿Qué comportamiento debe conservarse al crear el nuevo `global_map_server.cpp` en la subfase 1B?
