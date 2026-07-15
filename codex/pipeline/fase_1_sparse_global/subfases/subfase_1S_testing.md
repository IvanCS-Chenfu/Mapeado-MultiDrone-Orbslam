## Cambios prohibidos

- No modificar poses.
- No ejecutar optimización nueva.
- No crear detección BoW nueva.
- No crear RANSAC nuevo.
- No fusionar geométricamente fuera de `FusionManager`.
- No borrar MapPoints raw de `RawMapDatabase`.
- No permitir `score += X` fuera de `LandmarkScoreManager`.
- No penalizar todos los puntos sin match indiscriminadamente.
- No mantener scores/fused tracks de una tarea rechazada.
- No publicar miembros raw de fused tracks en la nube principal.
- No usar GT para score de loops.
- No ocultar ruido bajando score agresivamente sin logs.
- No romper rutas de fiducial/loop ya definidas.

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

### Prueba 1 — score inicial y publicación por score

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. arrancar simulación multi-dron;
2. insertar deltas con MapPoints nuevos;
3. inicializar scores desde datos raw;
4. publicar nube filtrada por score;
5. loggear estadísticas.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observación esperada en RViz2:

```text
La nube publicada debe contener puntos con score suficiente.
Si se habilita raw_debug, puede compararse nube raw vs nube filtrada/fused.
```

---

### Prueba 2 — fusión sube score y evita duplicados

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Secuencia esperada:

1. generar o reproducir caso `FUSION_CANDIDATE`;
2. fusionar inlier pairs con `FusionManager`;
3. emitir eventos `FusionConfirmed` y `FusedTrackConfirmed`;
4. actualizar scores;
5. publicar fused tracks;
6. comprobar que miembros raw no se duplican en nube principal.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

---

### Prueba 3 — penalización prudente de unmatched

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml
```

Secuencia esperada:

1. generar zona de solape confirmada;
2. identificar puntos esperados pero no asociados;
3. penalizar solo los que cumplan condiciones;
4. loggear skips para puntos no penalizables.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 3 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

---

## Pruebas replay requeridas

### Prueba 4 — rollback de scores

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_4.yaml
```

Secuencia esperada:

1. cargar dataset/replay;
2. crear fusión o score events asociados a un task;
3. forzar rechazo/rollback del task;
4. ejecutar rollback de score;
5. comprobar que scores y tracks quedan restaurados o desactivados;
6. publicar nube sin contaminación del task rechazado.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 4 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si existe prueba replay sin Gazebo más rápida, `planificador_fase` puede usarla justificándolo.

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1R-SCORE-ORBSLAM|F1R-SCORE-STATS|F1R-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1R-SCORE-FUSION|F1R-FUSED-SCORE|F1R-GLOBALMAP|F1O-FUSION|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 3

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1R-SCORE-UNMATCHED|F1R-SCORE-UPDATE|F1R-SCORE-STATS|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 4

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1R-SCORE-JOURNAL|F1R-SCORE-ROLLBACK|F1R-FUSED-TRACK-ROLLBACK|ERROR|FATAL|Segmentation fault|Killed
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

