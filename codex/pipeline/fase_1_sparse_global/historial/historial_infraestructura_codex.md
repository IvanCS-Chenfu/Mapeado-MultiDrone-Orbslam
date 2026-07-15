# Historial de infraestructura Codex

## 2026-06-30 — Conservacion de logs completos y reducidos

### Objetivo

Evitar repetir builds o simulaciones Gazebo cuando una reduccion de log omite
marcadores importantes por patrones insuficientes.

### Cambio realizado

Antes de esta sesion, las herramientas de reduccion sobrescribian el log
original. Desde ahora:

- `reduce_build_log.sh` lee `codex/archivos_auxiliares/logs/colcon_build.log` y
  genera `codex/archivos_auxiliares/logs/colcon_build.reduced.log`;
- `reduce_simulation_log.sh` lee `codex/archivos_auxiliares/logs/prueba_X.log` y
  genera `codex/archivos_auxiliares/logs/prueba_X.reduced.log`;
- los logs completos no se sustituyen durante la reduccion.

### Procedimiento obligatorio

Los agentes deben analizar primero el log reducido porque es mas manejable.
Si el reducido no contiene marcadores suficientes, hay que revisar el log
completo antes de:

- repetir Gazebo;
- concluir que no hubo loops;
- concluir que una optimizacion no emitio marcadores;
- atribuir el fallo al codigo.

Esto es especialmente importante en `12R-D4`, donde la diferencia entre "no hay
loops" y "el patron de reduccion no capturo loops" cambia el diagnostico.

### Archivos modificados

- `codex/herramientas/reduce_build_log.sh`
- `codex/herramientas/reduce_simulation_log.sh`
- `.agents/skills/ejecutar-fase/SKILL.md`
- `.agents/skills/compilar-paquetes/SKILL.md`
- `.agents/skills/ejecutar-simulacion/SKILL.md`
- `AGENTS.md`
- `codex/herramientas/USO_HERRAMIENTAS.md`
- `codex/contexto/00_LEER_PRIMERO.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/02_REGLAS_TECNICAS.md`
- `codex/contexto/03_ARQUITECTURA_ACTUAL.md`
- `codex/contexto/06_MAPA_CODIGO.md`
- `codex/contexto/07_ULTIMA_SESION.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/PLANTILLA_SUBFASE_EJECUTABLE.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- `codex/pipeline/fase_1_sparse_global/subfases/subfase_12R-D4.md`

### Validacion

No se ejecuto build ROS ni Gazebo porque no se modificaron paquetes ROS.
La sintaxis de `reduce_build_log.sh` y `reduce_simulation_log.sh` se valido con
`bash -n`.

### Conclusion

Infraestructura actualizada. En futuras ejecuciones de fase, `*.log` significa
log completo y `*.reduced.log` significa log filtrado.

---

## 2026-06-29 — Prueba minima de infraestructura

### Objetivo

Validar el flujo mecanico de Codex sin implementar la subfase `12R-D4`:

- creacion de `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`;
- build con `build_selected_packages.sh`;
- reduccion de build log si falla;
- simulacion con `run_simulation.sh`;
- reduccion de `prueba_1.log`;
- analisis de log reducido;
- actualizacion de historial/contexto.

### Alcance

No se modifico la logica funcional de `12R-D4`.
No se modifico `ORB_SLAM3`.
No se redisenaron wrapper ni mensajes.
No se limpiaron rutas legacy.

### Archivos modificados

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`: escenario minimo con dos drones y cuatro goals de `AccionTrayectoria`.
- `codex/herramientas/build_selected_packages.sh`: ajuste minimo para cargar `install/setup.bash` sin abortar por variables no definidas bajo `set -u`.
- `codex/herramientas/run_simulation.sh`: mismo ajuste minimo para cargar `install/setup.bash`.
- `simulacion_dron/src/control_tray/gui_tray_multi.py`: se activo el bit ejecutable para que el launch pueda ejecutarlo desde el symlink de install.

### Build

Comando final:

```bash
./codex/herramientas/build_selected_packages.sh dron_individual simulacion_dron
```

Resultado final:

```text
BUILD-EXIT-CODE 0
2 packages finished: dron_individual, simulacion_dron
```

Incidencias:

- Primer fallo: `install/setup.bash` referenciaba `COLCON_TRACE` bajo `set -u`.
- Segundo fallo: el sandbox impedia a `colcon` crear `log/build_...` fuera de `src`.
- Tercer fallo: artefacto previo en `build/dron_individual/.../dron_individual` bloqueaba un symlink de `ament_cmake_python`.
- Cuarto fallo en simulacion inicial: `gui_tray_multi.py` no era ejecutable para ROS 2 launch.

Correcciones:

- Se desactivo `nounset` solo durante el `source install/setup.bash` en las herramientas.
- Se ejecuto el build real con permisos fuera del sandbox.
- Se elimino solo el artefacto generado que bloqueaba el symlink.
- Se activo el bit ejecutable de `gui_tray_multi.py`.

### Simulacion

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Resultado:

```text
SIM-SCENARIO-EXIT-CODE 0
SIM-DONE prueba=1 success=true
SIM-EXIT-CODE 0
```

Log reducido:

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 1 \
  --patterns "SCENARIO-RUNNER|AccionTrayectoria|GOAL|RESULT|success|ERROR|FATAL|Segmentation fault|Killed|ORB|SLAM|global_map_server|FUSED|FID|LOCAL"
```

Evidencia principal:

- `scenario_runner_node` arranco con `tray_prueba_1.yaml`.
- Se enviaron goals a `/dron_1/AccionTrayectoria` y `/dron_2/AccionTrayectoria`.
- Los 4 resultados de goal tuvieron `success=true`.
- La simulacion cerro con `SIM-EXIT-CODE 0`.

### Conclusion

La infraestructura basica queda validada para:

- crear YAML auxiliar;
- compilar paquetes seleccionados;
- reducir logs de build al fallar;
- lanzar simulacion con `run_simulation.sh`;
- reducir y analizar `prueba_1.log`;
- documentar resultados.

Esta prueba no valida ni completa `12R-D4`.

### Pendiente antes de 12R-D4

- Ejecutar la planificacion especifica de `12R-D4`.
- Preparar pruebas D4 realistas: prueba limpia y prueba con deriva.
- Reducir logs masivos de servidor si procede.
- Implementar y compilar solo los cambios especificos de D4.

---

## 2026-06-29 — Prueba de infraestructura con visita a fiducial

### Objetivo

Repetir la prueba real minima de infraestructura Codex usando una trayectoria mas representativa:

1. `dron_1` va al fiducial en `(0, -9, 1.0)` con yaw `90 deg`.
2. `dron_2` va al mismo punto XY con altura `1.3` y yaw `90 deg`.
3. `dron_1` va a `(-7, -9, 1.0)` con yaw `90 deg`.
4. `dron_2` va a `(-7, -9, 1.3)` con yaw `90 deg`.
5. `dron_1` vuelve al fiducial en `(0, -9, 1.0)` con yaw `90 deg`.
6. `dron_2` vuelve al fiducial en `(0, -9, 1.3)` con yaw `90 deg`.

Esta prueba sigue siendo de infraestructura. No implementa ni valida `12R-D4`.

### Archivos modificados

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`: se sustituyo la trayectoria anterior por la secuencia de ida/vuelta al fiducial y a la posicion comun.
- `codex/pipeline/fase_1_sparse_global/historial/historial_infraestructura_codex.md`: se anadio este registro.

No se modifico codigo funcional.
No se modifico `ORB_SLAM3`.
No se redisenaron wrapper ni mensajes.
No se limpiaron rutas legacy.

### Build

Comando:

```bash
./codex/herramientas/build_selected_packages.sh dron_individual simulacion_dron
```

Resultado:

```text
BUILD-EXIT-CODE 0
2 packages finished: dron_individual, simulacion_dron
```

No fue necesario ejecutar `reduce_build_log.sh` en esta repeticion porque el build termino correctamente.

### Simulacion

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Resultado general:

```text
SIM-SCENARIO-EXIT-CODE 0
SIM-DONE prueba=1 success=true
SIM-EXIT-CODE 0
```

Evidencia principal del log reducido:

- `scenario_runner_node` arranco con `prueba_infraestructura_codex_fiducial_ida_vuelta`.
- Se enviaron 6 goals a `AccionTrayectoria`.
- Los 6 goals devolvieron `success=true` con `t_total=10.000`.
- El nodo de escenario emitio `SCENARIO-RUNNER-DONE success=true`.
- La herramienta cerro con `SIM-EXIT-CODE 0`.

Incidencia observada:

- El log reducido contiene un `ERROR` de `gazebo` con `exit code 255`.
- A pesar de ese evento, los action servers respondieron, los 6 goals terminaron con `success=true` y `run_simulation.sh` finalizo con codigo `0`.
- Para pruebas funcionales de `12R-D4`, conviene vigilar si ese `ERROR` se repite y si afecta visualmente a RViz2/Gazebo.

### Evaluacion de agentes

Los roles/agentes definidos estan ayudando de forma util:

- `planificador_fase`: actuo bien al separar esta prueba de infraestructura de `12R-D4`, limitar paquetes a `dron_individual` y `simulacion_dron`, y definir una unica prueba observable.
- `implementador_fase`: actuo bien al modificar solo `tray_prueba_1.yaml` y no tocar codigo funcional.
- `diagnosticador_build`: no tuvo que intervenir en esta repeticion porque el build paso a la primera.
- `analizador_simulacion_logs`: actuo bien al confirmar runner, goals, resultados `success=true`, cierre de simulacion y al detectar el `ERROR` residual de `gazebo`.
- `curador_documentacion`: actuo bien al registrar la prueba en este historial sin marcar `12R-D4` como realizada.

Conclusion sobre los subagentes:

```text
Los subagentes estan actuando bien para esta infraestructura.
Ayudan a separar planificacion, ejecucion, diagnostico, analisis y documentacion.
La division de responsabilidades evita mezclar una prueba mecanica con avances funcionales de 12R-D4.
```

### Conclusion

La infraestructura queda validada tambien con una trayectoria mas cercana al caso de fiducial.
La prueba no demuestra que `12R-D4` este resuelta.

### Pendiente antes de 12R-D4

- Revisar si el `ERROR` de `gazebo` con exit code `255` es puntual o recurrente.
- Preparar las pruebas especificas de `12R-D4` con criterios de exito de optimizacion local.
- Implementar solo el guard de apply/fiducial cuando se ejecute la subfase real.
