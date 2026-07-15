## Cambios prohibidos

- No descartar KFs llegados durante optimización.
- No rebasar con una corrección de una optimización rechazada.
- No procesar loops de KFs rebasados antes de que la optimización quede ACCEPT/PARTIAL definido.
- No usar GT para loops.
- No crear un pipeline especial de loops para estos KFs.
- No crear un optimizador especial para estos KFs.
- No fusionar directamente por estar en la cola; debe pasar por 1N–1P.
- No crear cascadas infinitas de optimización.
- No ignorar límites de cola/cooldown.
- No modificar `RawMapDatabase` por rebase u optimización.
- No borrar estados pendientes sin log.

---

## Paquetes a compilar

Comando esperado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron
```

Si `simulacion_dron` no se modifica ni se requiere para build incremental, puede omitirse justificándolo en historial.

Si falla:

```bash
./codex/herramientas/reduce_build_log.sh
```

El diagnóstico debe leer:

```text
codex/archivos_auxiliares/logs/colcon_build.reduced.log
```

---

## Pruebas Gazebo requeridas

### Prueba 1 — KF llega durante optimización aceptada

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. arrancar simulación multi-dron;
2. generar una tarea de optimización fiducial o loop;
3. durante la optimización, insertar al menos un KF nuevo del submapa afectado;
4. marcarlo como pending;
5. aceptar la optimización;
6. rebasar el KF pendiente;
7. encolarlo para loop check;
8. despacharlo por 1N–1P;
9. loggear resultado.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observación esperada en RViz2:

```text
Si el KF pendiente afecta a la nube, debe verse colocado de forma coherente
tras el rebase. La validación principal es por logs.
```

---

### Prueba 2 — optimización rechazada no reinyecta KFs

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Secuencia esperada:

1. generar optimización con rollback controlado;
2. insertar KF durante la optimización;
3. marcarlo como pending;
4. post-apply REJECT;
5. rollback;
6. no rebasar con corrección rechazada;
7. no disparar loop check post-optimización por esa corrección.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

---

## Pruebas replay requeridas

### Prueba 3 — cola post-optimización en replay

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml
```

Secuencia esperada:

1. cargar dataset con deltas;
2. activar modo replay;
3. simular optimización activa durante algunos deltas;
4. KFs nuevos van a pending;
5. aceptar optimización;
6. rebasar;
7. encolar;
8. despachar por 1N–1P;
9. comprobar determinismo de logs.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 3 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si existe una prueba replay sin Gazebo más rápida y validada, `planificador_fase` puede usarla justificándolo.

---

## Prueba recomendada adicional

### Prueba 4 — límite de cascada

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_4.yaml
```

Objetivo:

Comprobar que el reprocesado post-optimización no crea bucles infinitos.

Secuencia esperada:

1. forzar varios KFs rebasados;
2. procesar cola;
3. simular que algunos generan nuevas tareas de optimización;
4. alcanzar límite `post_opt_max_cascade_depth` o `post_opt_max_new_optimization_tasks_per_cycle`;
5. pausar o diferir cola con log.

Logs esperados:

```text
[F1Q-CASCADE-LIMIT]
[F1Q-QUEUE-PAUSED]
```

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1Q-|F1K-POSESTORE-REBASE|F1L-POST-APPLY-ACCEPT|F1M-LOOP|F1N-SUBCLOUD|F1O-LOOP|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1Q-OPT-REJECTED-NO-REBASE|F1Q-LOOP-CHECK-SKIP|F1L-POST-APPLY-REJECT|F1L-POSESTORE-ROLLBACK|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 3

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1Q-|F1Q-REPLAY|F1C-REPLAY|F1M-LOOP|F1N-SUBCLOUD|F1O-LOOP|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 4

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1Q-CASCADE-LIMIT|F1Q-QUEUE-PAUSED|F1Q-LOOP-CHECK-DISPATCH|ERROR|FATAL|Segmentation fault|Killed
```

La reducción genera:

```text
codex/archivos_auxiliares/logs/prueba_1.reduced.log
codex/archivos_auxiliares/logs/prueba_2.reduced.log
codex/archivos_auxiliares/logs/prueba_3.reduced.log
codex/archivos_auxiliares/logs/prueba_4.reduced.log
```

Si el reducido no muestra suficiente información, revisar el log completo antes de repetir Gazebo o concluir que falta el marcador.

---

