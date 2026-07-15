# Subfase 1B — Servidor nuevo mínimo y congelación del legacy

## Estado

```text
realizado
```

Servidor mínimo validado técnicamente el 2026-07-06 y cierre aceptado tras completar comentarios/trazabilidad de código por subfase.

Revisión posterior incorporada:

- el código nuevo quedó comentado con suficiente detalle en funciones, callbacks, bucles y bloques importantes;
- los comentarios indican qué parte pertenece a `F1B` para poder revisar o revertir esa subfase si más adelante se detecta un problema;
- esta trazabilidad queda comprobada y se conserva como requisito para subfases futuras.

Evidencia principal:

- `BUILD-EXIT-CODE 0`;
- `SIM-DONE prueba=1 success=true`;
- `[F1B-SERVER-INIT]`;
- `[F1B-SERVER-SUBSCRIBED]` para `dron_1` y `dron_2`;
- `[F1B-ORBMAP-RX]` para dos drones;
- `[F1B-SERVER-STATS]` con `drones_seen=2`.

Conclusión final: `CONSEGUIDA`.

---

## Objetivo técnico

Crear la primera versión limpia de `global_map_server.cpp` como adaptador ROS 2 mínimo, manteniendo el servidor anterior como referencia congelada para la migración.

Esta subfase inicia la reestructuración de Fase 1, pero **no implementa todavía** anclaje por fiduciales, fused map, loops, subnubes, optimización ni publicación de nube global. El objetivo verificable es más pequeño:

- conservar el servidor antiguo para consulta durante la migración;
- crear un servidor nuevo que arranque con el launch normal;
- crear o simplificar el launch activo del servidor si aplica;
- recibir `OrbMap` delta desde los wrappers de todos los drones configurados;
- mostrar en logs `drone_id`, `map_epoch`, `map_sequence`, número de `KeyFrames` y número de `MapPoints` recibidos;
- preparar una carpeta o zona `legacy` en `orbslam3_multi` con copias congeladas de archivos antiguos relevantes, **sin compilarlas**;
- documentar claramente qué archivos son activos y qué archivos son solo referencia histórica.

El resultado esperado es que el nuevo servidor demuestre esta cadena mínima:

```text
wrapper ORB-SLAM3 -> topic orb_map_delta -> nuevo global_map_server.cpp -> logs F1B
```

En esta subfase no se espera ver una nube útil en RViz2. La validación principal es por build, simulación y logs.

---

## Contexto obligatorio a leer

Antes de ejecutar esta subfase, Codex debe leer:

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/02_REGLAS_TECNICAS.md`, si existe
- `codex/contexto/06_MAPA_CODIGO.md`, si existe
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_1A.md`
- historial generado por la subfase 1A, especialmente:
  - marcadores encontrados;
  - launch usado;
  - topics reales de `OrbMap`;
  - nombres reales de drones;
  - trayectoria YAML usada;
  - errores heredados del servidor antiguo.
- documentación de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/simulacion_dron/`
  - `codex/contexto/paquetes/orbslam3_ros2/`, solo para confirmar topics del wrapper
  - `codex/contexto/paquetes/orbslam3_msgs/`, solo para confirmar campos de `OrbMap`
- ADRs relacionados en `codex/contexto/decisiones/`, si existen.

---

## Diagnóstico de partida

La subfase 1A debe haber dejado una fotografía del servidor actual. Ese servidor antiguo puede no funcionar correctamente, pero contiene lógica útil que se quiere consultar durante la migración.

Problemas de partida:

- `global_map_server.cpp` concentra demasiadas responsabilidades;
- la lógica ROS está mezclada con almacenamiento, fiduciales, loops, fused landmarks y optimización;
- la futura arquitectura necesita que `orbslam3_server` sea un adaptador ROS y que `orbslam3_multi` contenga el backend algorítmico;
- antes de crear `RawMapDatabase`, `GlobalPoseStore` y demás clases nuevas, hace falta un servidor mínimo que reciba datos de wrappers de forma fiable;
- los archivos antiguos de `orbslam3_multi` pueden ser útiles como referencia, pero no deben bloquear ni contaminar el nuevo diseño.

La subfase 1B debe evitar arreglar problemas funcionales del mapa. Su objetivo es crear una base limpia y verificable para las siguientes subfases.

Marcadores que deben existir tras esta subfase:

```text
[F1B-SERVER-INIT]
[F1B-SERVER-PARAMS]
[F1B-SERVER-SUBSCRIBED]
[F1B-ORBMAP-RX]
[F1B-ORBMAP-RX-KFS]
[F1B-ORBMAP-RX-MPS]
[F1B-SERVER-STATS]
```

Marcadores heredados útiles para comprobar que los wrappers siguen publicando:

```text
PIPE0-WRAPPER-DELTA-PUB
PIPE0-WRAPPER-FULL-SNAPSHOT
PIPE0-WRAPPER-TRACK
SCENARIO-RUNNER
```

---

## Archivos permitidos a modificar

### Código fuente de `orbslam3_server`

- `orbslam3_server/src/global_map_server.cpp`
- `orbslam3_server/src/global_map_server_antiguo.cpp`, solo para crear la copia congelada del servidor anterior
- `orbslam3_server/CMakeLists.txt`, solo si es necesario para que el ejecutable compile usando el nuevo servidor y no compile el antiguo
- `orbslam3_server/package.xml`, solo si el nuevo servidor mínimo necesita una dependencia ya usada realmente por el paquete

### Launches

El planificador debe localizar primero los launches reales del servidor y de simulación. Archivos probables:

- `orbslam3_server/launch/*.launch.py`
- `simulacion_dron/launch/multi_dron.launch.py`, solo si necesita seguir apuntando al launch activo normal del servidor

Se permite:

- congelar el launch antiguo del servidor como `*_antiguo.launch.py` si existe un launch específico de servidor;
- crear un nuevo launch activo con el nombre público normal;
- mantener el launch principal de simulación con el mismo nombre y el mínimo cambio necesario.

### Referencia legacy de `orbslam3_multi`

Se permite crear una carpeta de referencia, por ejemplo:

```text
orbslam3_multi/legacy/
```

Dentro de ella se pueden copiar archivos actuales relevantes de `orbslam3_multi` como referencia histórica, por ejemplo:

```text
orbslam3_multi/legacy/*_antiguo.hpp
orbslam3_multi/legacy/*_antiguo.cpp
orbslam3_multi/legacy/README.md
```

Condiciones obligatorias:

- esos archivos no deben añadirse a `CMakeLists.txt`;
- ningún archivo activo debe incluirlos;
- no deben contener funcionalidad nueva;
- solo sirven para observación durante la migración.

### Archivos auxiliares y documentación

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- `codex/archivos_auxiliares/logs/prueba_1.log`, generado por herramienta
- `codex/archivos_auxiliares/logs/prueba_1.reduced.log`, generado por herramienta
- `codex/archivos_auxiliares/logs/colcon_build.log`, generado por herramienta
- `codex/archivos_auxiliares/logs/colcon_build.reduced.log`, generado por herramienta si falla build
- `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- documentación de paquetes modificados en:
  - `codex/contexto/paquetes/orbslam3_server/`
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/simulacion_dron/`, solo si se modifica launch o YAML asociado

---

## Archivos prohibidos

No modificar en esta subfase:

- `ORB_SLAM3/`
- `orbslam3_ros2/` salvo que se demuestre que el topic documentado no coincide con el wrapper y sea imposible validar 1B sin un cambio mínimo. En principio está prohibido.
- `orbslam3_msgs/`, porque los mensajes se consideran estables.
- lógica de fiduciales, loops, fused map u optimización heredada más allá de moverla a archivo antiguo como referencia.
- archivos legacy de `orbslam3_multi` una vez copiados a `legacy/`; no se deben editar para implementar nada.
- `global_map_server_antiguo.cpp` después de crearlo, salvo para añadir un comentario inicial que indique que es legacy/no activo, si se considera útil.
- `build/`, `install/` y `log/` manualmente, salvo limpiezas mínimas permitidas por `AGENTS.md` si bloquean build/simulación.

No renombrar el paquete completo `orbslam3_multi` ni cambiar su `package.xml` para darle otro nombre.

---

## Funciones, clases o nodos que hay que localizar

El planificador debe localizar con búsqueda estática antes de implementar:

- archivo activo actual del servidor:
  - `global_map_server.cpp`
- ejecutable/nodo definido en `orbslam3_server/CMakeLists.txt`
- clase o función `main` actual del servidor, si existe
- nombre real del nodo ROS del servidor global
- topics reales de entrada de mapas ORB, especialmente los equivalentes a:

```text
/dron_X/orbslam/orb_map_delta
orbslam/orb_map_delta
```

- servicio de snapshot completo, aunque no es obligatorio usarlo todavía:

```text
/dron_X/orbslam/get_full_map
```

- launch del servidor global, si existe
- cómo `simulacion_dron/multi_dron.launch.py` lanza el servidor
- nombres reales de drones aceptados por `scenario_runner_node`
- formato real de `codex/archivos_auxiliares/trayectorias/tray_prueba_X.yaml`
- archivos actuales de `orbslam3_multi` que conviene congelar como referencia.

Nombres conceptuales del nuevo servidor mínimo:

- clase nueva recomendada, si se usa clase:

```text
GlobalMapServer
```

- callback conceptual:

```text
OnOrbMapDelta
```

No inventar nombres finales si el paquete ya usa convenciones claras. Si los nombres reales difieren, mantener la convención existente y documentarla.

---

## Cambios requeridos

### 1. Congelar el servidor antiguo

Intención:

- conservar el servidor anterior como referencia durante la migración.

Cambio:

```text
orbslam3_server/src/global_map_server.cpp
→
orbslam3_server/src/global_map_server_antiguo.cpp
```

Condiciones de seguridad:

- el archivo antiguo no debe ser el ejecutable activo;
- si `CMakeLists.txt` compila todos los `.cpp` por glob o incluye explícitamente el archivo, ajustar el CMake para que no compile el antiguo;
- no borrar lógica del archivo antiguo;
- no arreglar bugs dentro del archivo antiguo.

Log o validación:

- el build debe demostrar que el ejecutable activo compila con el nuevo `global_map_server.cpp`.

---

### 2. Crear un nuevo `global_map_server.cpp` mínimo

Intención:

- arrancar un servidor limpio que solo reciba datos y loggee lo recibido.

Responsabilidades del nuevo archivo:

- inicializar nodo ROS;
- declarar/leer parámetros mínimos necesarios para conocer drones, namespaces o topics;
- crear subscribers a `orb_map_delta` de cada dron;
- loggear cada mensaje recibido con datos básicos;
- mantener contadores simples de recepción;
- publicar estadísticas periódicas por log.

No debe implementar:

- fiduciales;
- anchoring;
- fused landmarks;
- loop detection;
- optimización;
- publicación de nube global real;
- correcciones de mapa.

Logs obligatorios:

```text
[F1B-SERVER-INIT]
[F1B-SERVER-PARAMS]
[F1B-SERVER-SUBSCRIBED]
[F1B-ORBMAP-RX]
[F1B-ORBMAP-RX-KFS]
[F1B-ORBMAP-RX-MPS]
[F1B-SERVER-STATS]
```

Ejemplos orientativos:

```text
[F1B-SERVER-INIT] node=global_map_server mode=minimal_rx_only
[F1B-SERVER-PARAMS] use_sim_time=true world_frame=world drones=2
[F1B-SERVER-SUBSCRIBED] drone_id=1 topic=/dron_1/orbslam/orb_map_delta
[F1B-ORBMAP-RX] drone_id=1 epoch=0 seq=14 frame_id=orb_map kfs=2 mps=136
[F1B-ORBMAP-RX-KFS] drone_id=1 epoch=0 seq=14 first_kf=0 last_kf=12 count=2
[F1B-ORBMAP-RX-MPS] drone_id=1 epoch=0 seq=14 first_mp=4 last_mp=982 count=136
[F1B-SERVER-STATS] rx_maps=42 rx_kfs=81 rx_mps=5240 drones_seen=2 epochs_seen=2
```

Condiciones de seguridad:

- si un mensaje llega vacío, loggear `kfs=0 mps=0` sin tratarlo como error grave;
- no asumir que `map_epoch` empieza siempre en cero;
- no asumir que solo hay un dron;
- no depender de RViz2 para validar.

---

### 3. Congelar launch antiguo y crear launch activo mínimo, si aplica

Intención:

- evitar que el launch activo siga pasando parámetros incompatibles con el nuevo servidor mínimo;
- conservar el launch antiguo para consulta.

Cambio permitido si existe launch específico del servidor:

```text
orbslam3_server/launch/<launch_servidor>.launch.py
→
orbslam3_server/launch/<launch_servidor>_antiguo.launch.py
```

Crear de nuevo el launch activo con el nombre público normal.

Condiciones de seguridad:

- el launch principal de simulación debe seguir siendo:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

- no crear un launch principal alternativo para la prueba;
- si `multi_dron.launch.py` incluye el launch del servidor, debe seguir apuntando al launch activo normal, no al antiguo;
- el launch antiguo no debe usarse en esta subfase salvo para consulta.

Validación:

- al ejecutar la simulación aparece `[F1B-SERVER-INIT]` del servidor nuevo.

---

### 4. Crear referencia legacy de `orbslam3_multi` sin compilarla

Intención:

- dejar congelados archivos antiguos de `orbslam3_multi` para observación durante la migración.

Cambio recomendado:

```text
orbslam3_multi/legacy/
orbslam3_multi/legacy/README.md
```

Copiar ahí archivos relevantes actuales, renombrados con sufijo `_antiguo`, si el planificador confirma que existen y son útiles.

Condiciones de seguridad:

- no tocar el nombre del paquete `orbslam3_multi`;
- no añadir `legacy/` al build;
- no incluir headers de `legacy/` desde código activo;
- no reescribir todavía clases activas de `orbslam3_multi`;
- si copiar todos los archivos genera ruido excesivo, copiar solo los módulos principales y documentar el criterio.

Contenido mínimo de `legacy/README.md`:

```text
Estos archivos son copias congeladas del diseño anterior de orbslam3_multi.
No forman parte del build.
No deben recibir funcionalidad nueva.
Solo sirven como referencia durante la migración de Fase 1.
```

Validación:

- el build de `orbslam3_multi` sigue usando los archivos activos normales;
- no aparecen símbolos duplicados por archivos legacy.

---

### 5. Crear o actualizar YAML de prueba

Intención:

- usar una trayectoria suficientemente larga para que ambos wrappers publiquen `OrbMap` y el servidor nuevo reciba datos de ambos drones.

La trayectoria debe ser la habitual indicada por el usuario:

1. dron 1 al fiducial;
2. dron 2 al fiducial pero encima;
3. dron 1 a la izquierda;
4. dron 2 a la izquierda pero encima;
5. dron 1 de vuelta al fiducial;
6. dron 2 de vuelta al fiducial pero encima.

Condiciones de seguridad:

- usar el formato real del `scenario_runner_node` existente;
- si existe un `tray_prueba_1.yaml` equivalente de 1A, reutilizarlo o adaptarlo mínimamente;
- no inventar nombres de drones si el proyecto usa `drone_0/drone_1` en vez de `drone_1/drone_2`;
- documentar en historial los nombres y coordenadas usadas.

---

### 6. Actualizar documentación de paquetes

Intención:

- que Codex entienda qué archivos son activos y cuáles son legacy.

Documentar en `codex/contexto/paquetes/orbslam3_server/`:

- nuevo `global_map_server.cpp` activo;
- `global_map_server_antiguo.cpp` como referencia congelada;
- launch activo nuevo;
- launch antiguo si existe;
- topics suscritos;
- logs `[F1B-*]`.

Documentar en `codex/contexto/paquetes/orbslam3_multi/`:

- que el paquete será el backend algorítmico de la nueva Fase 1;
- que en 1B todavía no se implementan clases nuevas funcionales;
- que `legacy/` contiene copias de consulta no compiladas;
- que las clases nuevas reales empiezan en 1C con `RawMapDatabase`.

---

## Cambios prohibidos

No hacer en 1B:

- no implementar `RawMapDatabase` todavía;
- no implementar `GlobalPoseStore` todavía;
- no implementar `FiducialAnchorManager` todavía;
- no implementar detección de fiduciales;
- no usar ground truth;
- no publicar nube global como criterio de éxito;
- no portar fused landmarks;
- no portar loops;
- no portar optimización;
- no modificar mensajes `orbslam3_msgs`;
- no modificar wrapper `orbslam3_ros2`;
- no renombrar el paquete `orbslam3_multi`;
- no compilar archivos de `orbslam3_multi/legacy/`;
- no borrar masivamente legacy;
- no arreglar problemas heredados del servidor antiguo;
- no crear una segunda arquitectura paralela fuera de `orbslam3_multi`.

Si aparece un fallo funcional heredado del servidor antiguo, documentarlo. Solo se corrigen errores que impidan el objetivo de 1B: compilar, arrancar y recibir `OrbMap`.

---

## Paquetes a compilar

Comando esperado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server simulacion_dron
```

Justificación:

- `orbslam3_msgs`: tipos usados por el nuevo servidor;
- `orbslam3_multi`: puede cambiar por `legacy/` o documentación de paquete; además debe seguir compilando tras la preparación;
- `orbslam3_server`: paquete principal modificado;
- `simulacion_dron`: compilar si se modifica o se comprueba integración de launch.

Si `simulacion_dron` no tiene cambios y el workspace permite compilar menos, `implementador_fase` puede compilar solo:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

Debe justificarlo en historial.

Si falla el build:

```bash
./codex/herramientas/reduce_build_log.sh
```

Después, diagnosticar el primer error real y corregir lo mínimo.

---

## Pruebas Gazebo requeridas

### Prueba 1 — recepción mínima multi-dron

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. esperar arranque de Gazebo, wrappers ORB-SLAM3, servidor nuevo y `scenario_runner_node`;
2. mover dron 1 al fiducial;
3. mover dron 2 al fiducial pero a una altura superior o posición equivalente segura;
4. mover dron 1 a la izquierda del fiducial/edificio;
5. mover dron 2 a la izquierda pero manteniendo separación vertical o espacial segura;
6. mover dron 1 de vuelta al fiducial;
7. mover dron 2 de vuelta al fiducial pero encima;
8. esperar unos segundos para que wrappers publiquen deltas finales;
9. cerrar simulación mediante la herramienta.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observación esperada en RViz2:

```text
No aplica como criterio de éxito.
```

En esta subfase no se espera ver nube sparse global útil. Si RViz2 se abre manualmente y no muestra mapa global, no es fallo. El criterio de validación es el log.

---

## Patrones de reducción de logs

### Prueba 1

Usar patrones específicos para no ocultar los marcadores importantes:

```text
SCENARIO-RUNNER|GOAL|RESULT|success|PIPE0-WRAPPER-DELTA-PUB|PIPE0-WRAPPER-FULL-SNAPSHOT|PIPE0-WRAPPER-TRACK|F1B-SERVER-INIT|F1B-SERVER-PARAMS|F1B-SERVER-SUBSCRIBED|F1B-ORBMAP-RX|F1B-ORBMAP-RX-KFS|F1B-ORBMAP-RX-MPS|F1B-SERVER-STATS|ERROR|FATAL|Segmentation fault|Killed
```

Comando:

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 1 \
  --patterns "SCENARIO-RUNNER|GOAL|RESULT|success|PIPE0-WRAPPER-DELTA-PUB|PIPE0-WRAPPER-FULL-SNAPSHOT|PIPE0-WRAPPER-TRACK|F1B-SERVER-INIT|F1B-SERVER-PARAMS|F1B-SERVER-SUBSCRIBED|F1B-ORBMAP-RX|F1B-ORBMAP-RX-KFS|F1B-ORBMAP-RX-MPS|F1B-SERVER-STATS|ERROR|FATAL|Segmentation fault|Killed"
```

La reducción genera:

```text
codex/archivos_auxiliares/logs/prueba_1.reduced.log
```

y conserva:

```text
codex/archivos_auxiliares/logs/prueba_1.log
```

Si el reducido no muestra suficientes datos, revisar el log completo antes de repetir Gazebo o concluir que el código no emitió el marcador.

---

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. la prueba Gazebo requerida se ejecuta;
3. `scenario_runner_node` se ejecuta;
4. los goals requeridos de la trayectoria terminan con `success=true`, salvo que el historial justifique una diferencia de nombres/acciones equivalente;
5. aparece `[F1B-SERVER-INIT]`;
6. aparece `[F1B-SERVER-PARAMS]`;
7. aparece al menos un `[F1B-SERVER-SUBSCRIBED]` por cada dron esperado o se documenta claramente el esquema de suscripción usado;
8. aparece `[F1B-ORBMAP-RX]` para al menos dos drones en la prueba multi-dron;
9. los logs `[F1B-ORBMAP-RX]` incluyen `drone_id`, `map_epoch`, `map_sequence`, número de `KeyFrames` y número de `MapPoints`;
10. aparece `[F1B-SERVER-STATS]` con contadores acumulados;
11. no aparecen `Segmentation fault`, `Killed`, `FATAL` ni errores graves no explicados;
12. `global_map_server_antiguo.cpp` queda como referencia y no como servidor activo;
13. si se crea `orbslam3_multi/legacy/`, esos archivos no se compilan;
14. la documentación e historial quedan actualizados.

No es necesario que se publique nube global ni que RViz2 muestre mapa.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- la simulación requerida no se ejecuta;
- el servidor nuevo no arranca;
- no aparece `[F1B-SERVER-INIT]`;
- no aparece ningún `[F1B-ORBMAP-RX]`;
- el servidor activo sigue siendo claramente el antiguo;
- aparecen errores graves no explicados;
- se rompen los wrappers o mensajes para conseguir la prueba.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- el servidor nuevo arranca;
- recibe datos de un dron pero no de todos;
- recibe `OrbMap` pero los logs no contienen todavía todos los campos requeridos;
- el launch nuevo funciona manualmente pero no integrado desde `multi_dron.launch.py`;
- la carpeta legacy de `orbslam3_multi` queda preparada pero falta documentación completa.

La subfase debe marcarse como `BLOQUEADA` solo si:

- la subfase 1A no dejó información suficiente de topics/launches y no se pueden localizar por búsqueda estática;
- el entorno Gazebo no arranca por una causa externa repetida;
- falta una dependencia externa no resoluble con cambios mínimos;
- no se puede determinar de forma segura qué archivo del servidor es el activo.

---

## Documentación a actualizar

Actualizar siempre:

- `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`

Actualizar documentación de paquetes modificados:

### `orbslam3_server`

Actualizar o crear archivos en:

```text
codex/contexto/paquetes/orbslam3_server/
```

Debe quedar documentado:

- `global_map_server.cpp` es el servidor activo nuevo y mínimo de 1B;
- `global_map_server_antiguo.cpp` es referencia congelada de migración;
- qué launch es activo;
- qué launch es antiguo, si existe;
- topics suscritos;
- parámetros mínimos leídos;
- logs `[F1B-*]`;
- limitación explícita: en 1B no hay nube global, fiduciales, loops ni optimización.

Archivos sugeridos:

```text
codex/contexto/paquetes/orbslam3_server/global_map_server.md
codex/contexto/paquetes/orbslam3_server/global_map_server_antiguo.md
codex/contexto/paquetes/orbslam3_server/launches.md
```

Si ya existen, actualizarlos en vez de duplicarlos.

### `orbslam3_multi`

Actualizar o crear:

```text
codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md
```

Debe quedar documentado:

- `orbslam3_multi` no se renombra como paquete;
- en la nueva Fase 1 será el backend algorítmico;
- en 1B solo se prepara referencia legacy si aplica;
- `orbslam3_multi/legacy/` contiene copias congeladas no compiladas;
- los archivos legacy no deben modificarse para añadir funcionalidad;
- las clases nuevas reales empezarán en 1C con `RawMapDatabase`.

Si se crea `orbslam3_multi/legacy/README.md`, mencionar su existencia.

### `simulacion_dron`

Actualizar documentación solo si se modifica el launch de simulación o la trayectoria:

```text
codex/contexto/paquetes/simulacion_dron/launches.md
codex/contexto/paquetes/simulacion_dron/trayectorias.md
```

Debe documentar:

- que la prueba 1B usa la trayectoria de dos drones al fiducial, izquierda y vuelta;
- que RViz2 no es criterio de éxito en 1B;
- que el objetivo de la prueba es generar `OrbMap` suficiente para validar recepción.

### Historial

El historial debe registrar:

- fecha;
- subfase `1B`;
- archivos renombrados/copias legacy creadas;
- nuevo archivo activo del servidor;
- launches cambiados;
- paquetes compilados;
- resultado de build;
- prueba Gazebo ejecutada;
- YAML usado;
- patrones de reducción;
- evidencia positiva:
  - `[F1B-SERVER-INIT]`;
  - `[F1B-SERVER-SUBSCRIBED]`;
  - `[F1B-ORBMAP-RX]` por dron;
  - `[F1B-SERVER-STATS]`;
- evidencia negativa o ausente;
- conclusión `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`;
- siguiente paso recomendado: subfase 1C `RawMapDatabase`.

No marcar como `realizado` si el criterio de éxito no se cumple.

Tampoco marcar como `realizado` si el código nuevo de `1B` no contiene comentarios suficientes para:

- explicar el objetivo de cada función nueva;
- describir qué procesa cada callback ROS;
- aclarar qué recorren los bucles o acumuladores importantes;
- señalar con `F1B` las partes introducidas por esta subfase.
