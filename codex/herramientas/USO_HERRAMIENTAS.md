# Uso de herramientas de Codex

Este archivo explica cómo usar las herramientas de `src/codex/herramientas/` para automatizar compilación, simulación y reducción de logs durante una fase del pipeline.

La idea general es que los scripts hagan solo trabajo mecánico:

```text
compilar
reducir logs de build
lanzar simulaciones
reducir logs de simulación
```

El análisis real lo hacen los agentes:

```text
diagnosticador_build
analizador_simulacion_logs
curador_documentacion
```

---

## Uso dentro de una subfase automatica

Cuando el usuario pida ejecutar una subfase completa, las herramientas se usan dentro de este contrato:

```text
planificador_fase
  lee pipeline, subfase e historial
  define build, pruebas, YAMLs, patrones y criterio de exito

implementador_fase
  modifica codigo/YAMLs permitidos
  ejecuta build_selected_packages.sh

diagnosticador_build
  solo actua si el build falla y el log fue reducido

implementador_fase
  corrige el primer error real y recompila

implementador_fase
  ejecuta run_simulation.sh para cada prueba requerida
  ejecuta reduce_simulation_log.sh para cada prueba

analizador_simulacion_logs
  compara logs reducidos contra el criterio de exito

curador_documentacion
  registra resultado, evidencia y conclusion
```

Los scripts no deciden si una subfase esta conseguida. Una simulacion con `SIM-EXIT-CODE 0` solo indica que la herramienta termino correctamente. La subfase solo queda `CONSEGUIDA` si el log reducido contiene la evidencia tecnica definida por la subfase.

Conclusiones permitidas:

```text
CONSEGUIDA
NO CONSEGUIDA
PARCIAL
BLOQUEADA
```

Si faltan markers obligatorios o aparece un error grave, el historial debe decir `NO CONSEGUIDA` o `PARCIAL` aunque los scripts hayan devuelto `0`.

---

## Estructura esperada

Desde la raíz de `src/`:

```text
src/
  codex/
    herramientas/
      build_selected_packages.sh
      reduce_build_log.sh
      run_simulation.sh
      reduce_simulation_log.sh
      USO_HERRAMIENTAS.md

    archivos_auxiliares/
      logs/
        colcon_build.log
        colcon_build.reduced.log
        prueba_1.log
        prueba_1.reduced.log
        prueba_2.log
        prueba_2.reduced.log
      trayectorias/
        tray_prueba_1.yaml
        tray_prueba_2.yaml
      html/
        f1l_debug_animation_task_1.html
      repeticiones/
        rawdb_prueba_1.record
        f1l_graph_task_1.tsv
```

Los scripts se ejecutan normalmente desde `src/`:

```bash
./codex/herramientas/<script>.sh ...
```

## Permisos operativos preaprobados

El usuario autoriza a Codex a ejecutar directamente, sin pedir confirmación por chat:

1. `colcon build` a través de `build_selected_packages.sh`, aunque escriba en `build/`, `install/` y `log/`.
2. Limpiezas mínimas de artefactos generados dentro de `build/`, `install/` o `log/` si bloquean una compilación o simulación.
3. Simulaciones con `run_simulation.sh`, incluyendo generación de logs, cierre de Gazebo y reintentos automáticos.

Estas autorizaciones no permiten modificar código fuera de `src/`.
Si el sandbox externo exige escalado, Codex debe pedirlo directamente al ejecutar la herramienta.

---

## 1. `build_selected_packages.sh`

### Función

Compila los paquetes ROS 2 indicados por el agente o por el usuario.

El script guarda todo el log de compilación en:

```text
codex/archivos_auxiliares/logs/colcon_build.log
```

Si ya existía un `colcon_build.log`, se sobrescribe.

### Uso

```bash
./codex/herramientas/build_selected_packages.sh paquete_1 paquete_2 paquete_3
```

### Ejemplo habitual para Fase 1

```bash
./codex/herramientas/build_selected_packages.sh \
  orbslam3_msgs orbslam3_multi orbslam3_server
```

### Paquetes pesados

Si el build incluye paquetes pesados, no agruparlos todos en una sola llamada salvo necesidad clara.

Regla recomendada:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs
./codex/herramientas/build_selected_packages.sh orbslam3
./codex/herramientas/build_selected_packages.sh orbslam3_multi
./codex/herramientas/build_selected_packages.sh orbslam3_server
./codex/herramientas/build_selected_packages.sh dron_individual
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

Motivo: `orbslam3`, `dron_individual` y `simulacion_dron` pueden consumir bastante CPU, memoria y disco durante `colcon build`. Compilarlos uno a uno reduce el riesgo de bloqueo del ordenador. Si falla uno, reducir el log y diagnosticar ese paquete antes de seguir.

### Resultado

Si compila correctamente:

```text
exit code = 0
```

Si hay error de compilación:

```text
exit code != 0
```

En caso de error, se debe llamar a:

```bash
./codex/herramientas/reduce_build_log.sh
```

---

## 2. `reduce_build_log.sh`

### Función

Reduce el log de compilación para dejar solo los errores importantes.

Lee:

```text
codex/archivos_auxiliares/logs/colcon_build.log
```

Y escribe:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

El archivo `colcon_build.log` queda como log completo. Si el reducido no contiene
contexto suficiente, `diagnosticador_build` debe consultar el completo antes de
decidir que falta evidencia.

### Uso

```bash
./codex/herramientas/reduce_build_log.sh
```

### Cuándo usarlo

Solo si `build_selected_packages.sh` devuelve un código distinto de `0`.

### Flujo esperado

```text
build_selected_packages.sh falla
  ↓
reduce_build_log.sh genera colcon_build.reduced.log con errores clave
  ↓
diagnosticador_build lee colcon_build.reduced.log
  ↓
si falta contexto, diagnosticador_build consulta colcon_build.log completo
  ↓
diagnosticador_build indica qué corregir
  ↓
implementador_fase corrige
  ↓
build_selected_packages.sh se ejecuta de nuevo
```

### Qué debe hacer el agente después

El agente `diagnosticador_build` debe leer:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

y decirle a `implementador_fase`:

```text
archivo afectado
función afectada
primer error real
corrección mínima
qué no tocar
```

---

## 3. `run_simulation.sh`

### Función

Ejecuta una prueba de simulación.

El script:

```text
1. hace source de install/setup.bash;
2. lanza el launch principal de la simulación;
3. espera a que arranque ROS/Gazebo;
4. detecta si Gazebo murió al arrancar;
5. si Gazebo falla, mata restos de Gazebo y reintenta;
6. ejecuta el nodo scenario_runner_node con el YAML de trayectoria;
7. espera a que scenario_runner_node termine;
8. espera unos segundos extra tras la última acción;
9. cierra el grupo completo del launch con señales tipo Ctrl+C;
10. guarda el log completo de la prueba.
```

Para cerrar el launch, la herramienta intenta lanzarlo en un grupo de procesos propio con `setsid`.
Al terminar o fallar, envia `SIGINT` al grupo completo, espera, luego `SIGTERM` y finalmente `SIGKILL` si quedan procesos vivos.
Esto busca reproducir el comportamiento de pulsar `Ctrl+C` en una terminal y cerrar tambien los procesos hijos del launch.

### YAML esperado

Para la prueba `x`, el YAML debe estar en:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_x.yaml
```

Ejemplos:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

### Log generado

Para la prueba `x`, el log se guarda en:

```text
codex/archivos_auxiliares/logs/prueba_x.log
```

Si ya existía, se sobrescribe.

### Uso

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El script buscará automáticamente:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Y guardará el log en:

```text
codex/archivos_auxiliares/logs/prueba_1.log
```

### Reintentos automáticos si falla Gazebo

`run_simulation.sh` detecta fallos tempranos de Gazebo durante el arranque, por ejemplo:

```text
process has died ... gazebo
exit code 255
gzserver ... error
gzclient ... error
```

Cuando ocurre, la herramienta:

```text
1. cierra el grupo completo del launch activo;
2. ejecuta killall -9 gzserver gzclient gazebo;
3. espera unos segundos;
4. vuelve a lanzar la simulación.
```

Opciones:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20 \
  --max-gazebo-retries 2 \
  --gazebo-retry-wait-sec 5
```

Logs esperados si ocurre un retry:

```text
SIM-GAZEBO-DETECTED
SIM-RETRY
SIM-GAZEBO-KILL
SIM-RETRY-WAIT
SIM-ATTEMPT-START
```

### Ejemplo con dos pruebas

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20

./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Esto generará:

```text
codex/archivos_auxiliares/logs/prueba_1.log
codex/archivos_auxiliares/logs/prueba_2.log
```

---

## 4. `reduce_simulation_log.sh`

### Función

Reduce el log de una prueba de simulación para dejar solo las líneas importantes para validar la fase.

Lee:

```text
codex/archivos_auxiliares/logs/prueba_x.log
```

Y escribe:

```text
codex/archivos_auxiliares/logs/prueba_x.reduced.log
```

El archivo `prueba_x.log` queda como log completo. Si el reducido no contiene
marcadores suficientes, `analizador_simulacion_logs` debe buscar en el completo
antes de pedir repetir Gazebo o concluir que el código no emitió esos datos.

Si el reducido sigue siendo grande, no abrirlo entero por defecto. Crear sublogs
por marcador o tema según:

```text
codex/contexto/09_LOGS_Y_SUBLOGS.md
```

Comando:

```bash
./codex/herramientas/split_simulation_log.sh --prueba X --fase 1L
```

Ejemplos útiles:

```text
prueba_x.scenario.log
prueba_x.errors.log
prueba_x.F1L.log
prueba_x.gt_window.log
prueba_x.index.md
```

### Uso

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 1 \
  --patterns "SCENARIO-RUNNER|LOCAL-OPT|LOCAL-POSE-GRAPH|LOOP-SUBCLOUD|FID|FUSED|ERROR|FATAL"
```

### Ejemplo de reducción con patrones de Fase 1

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 1 \
  --patterns "SCENARIO-RUNNER|LOCAL_LOOP_OPT_TASK|LOCAL-POSE-GRAPH|LOCAL-OPT-APPLY|LOOP-SUBCLOUD|FID-DEBT|FUSED-SIMPLE-SUMMARY|ERROR|FATAL|Segmentation fault|Killed|std::bad_alloc"
```

### Cuándo usarlo

Después de ejecutar todas las simulaciones de una fase.

Ejemplo:

```text
run_simulation.sh prueba 1
run_simulation.sh prueba 2
  ↓
reduce_simulation_log.sh prueba 1
reduce_simulation_log.sh prueba 2
  ↓
analizador_simulacion_logs lee prueba_1.reduced.log y prueba_2.reduced.log
  ↓
si faltan marcadores, consulta prueba_1.log o prueba_2.log completos
```

---

## Flujo completo de una fase

Ejemplo general:

```text
planificador_fase lee pipeline, estado actual e historial
  ↓
implementador_fase modifica código y YAMLs de prueba
  ↓
build_selected_packages.sh compila paquetes seleccionados
  ↓
si build falla:
    reduce_build_log.sh
    diagnosticador_build lee colcon_build.reduced.log
    si falta contexto, consulta colcon_build.log
    implementador_fase corrige
    volver a build_selected_packages.sh
  ↓
si build pasa:
    run_simulation.sh prueba 1
    run_simulation.sh prueba 2
    ...
  ↓
reduce_simulation_log.sh prueba 1
reduce_simulation_log.sh prueba 2
  ↓
analizador_simulacion_logs valida resultados
  ↓
curador_documentacion actualiza documentación e historial
```

El historial debe registrar como minimo:
- fase/subfase;
- archivos modificados;
- paquetes compilados;
- resultado de build;
- pruebas ejecutadas;
- patrones de reduccion usados;
- evidencia positiva;
- evidencia negativa o ausente;
- conclusion exacta;
- siguiente paso recomendado.

---

## Ejemplo completo para una fase con dos pruebas

### 1. Compilar

```bash
./codex/herramientas/build_selected_packages.sh \
  orbslam3_msgs orbslam3_multi orbslam3_server
```

### 2. Si falla el build

```bash
./codex/herramientas/reduce_build_log.sh
```

Después, el agente `diagnosticador_build` lee:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

### 3. Si compila, ejecutar pruebas

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20

./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

### 4. Reducir logs de pruebas

```bash
./codex/herramientas/reduce_simulation_log.sh \
  --prueba 1 \
  --patterns "SCENARIO-RUNNER|LOCAL-OPT|LOCAL-POSE-GRAPH|LOOP-SUBCLOUD|FID|FUSED|ERROR|FATAL"

./codex/herramientas/reduce_simulation_log.sh \
  --prueba 2 \
  --patterns "SCENARIO-RUNNER|LOCAL-OPT|LOCAL-POSE-GRAPH|LOOP-SUBCLOUD|FID|FUSED|ERROR|FATAL"
```

### 5. Analizar

El agente `analizador_simulacion_logs` lee:

```text
codex/archivos_auxiliares/logs/prueba_1.reduced.log
codex/archivos_auxiliares/logs/prueba_2.reduced.log
```

Si faltan marcadores obligatorios o hay duda sobre patrones de `grep`, consulta
`prueba_1.log` o `prueba_2.log` completos antes de repetir simulaciones.

Y concluye si la fase ha funcionado.

---

## Reglas importantes

- `colcon_build.log` siempre se sobrescribe con el build completo.
- `colcon_build.reduced.log` se regenera desde `colcon_build.log`.
- `prueba_x.log` siempre se sobrescribe con la simulación completa.
- `prueba_x.reduced.log` se regenera desde `prueba_x.log`.
- No se guardan diagnósticos manuales en archivos separados por ahora.
- El análisis lo hacen los agentes, no los scripts.
- Los scripts solo ejecutan y reducen información.
- Los YAMLs de trayectoria deben llamarse `tray_prueba_x.yaml`.
- Los logs de simulación completos deben llamarse `prueba_x.log`.
- Los logs de simulación reducidos deben llamarse `prueba_x.reduced.log`.
