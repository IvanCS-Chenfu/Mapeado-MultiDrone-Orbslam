## Cambios prohibidos

- No crear un nuevo sistema de optimización.
- No crear un segundo `PoseGraphBuilder` específico de loops.
- No crear un segundo `OptimizationManager`.
- No modificar `RawMapDatabase` por optimización.
- No ignorar `CovisibilityDatabase` al construir grafos de loop.
- No sobrescribir `relative_pose_measured`.
- No usar GT para loops.
- No fusionar antes de optimizar si el error de loop es alto.
- No fusionar si post-apply es `REJECT`.
- No fusionar por defecto si post-apply es `PARTIAL`.
- No mover hard fiducials.
- No aceptar un loop si el error real post-apply empeora.
- No publicar nube fused como aceptada si hubo rollback.
- No reintentar optimización automáticamente en bucle.
- No esconder fallos aumentando límites sin diagnóstico.

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

### Prueba 1 — loop optimization live aceptada

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. arrancar simulación multi-dron;
2. generar candidato BoW;
3. verificar subnubes;
4. obtener `LOOP_OPTIMIZATION_CANDIDATE`;
5. crear `LoopOptimizationTask`;
6. construir grafo de loop;
7. añadir restricciones desde `CovisibilityDatabase`;
8. ejecutar dry-run;
9. aplicar si useful;
10. validar post-apply;
11. si ACCEPT, actualizar `relative_pose_current` cuando proceda;
12. si ACCEPT, fusionar;
13. publicar mapa fused actualizado.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observación esperada en RViz2:

```text
Si la optimización se acepta, deberían reducirse duplicados/dobles paredes
en la zona del loop.
Los fiduciales o KFs hard-fixed no deben moverse de forma visible.
```

---

### Prueba 2 — loop optimization con rollback

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Objetivo:

Validar que una optimización por loop mala no se conserva ni fusiona.

Secuencia esperada:

1. generar o forzar un `LOOP_OPTIMIZATION_CANDIDATE`;
2. ejecutar grafo/dry-run/apply controlado;
3. forzar o producir fallo post-apply;
4. ejecutar rollback;
5. no fusionar landmarks;
6. publicar mapa restaurado;
7. emitir diagnóstico y retry suggestion.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

---

## Pruebas replay requeridas

### Prueba 3 — loop optimization desde dataset guardado

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml
```

Secuencia esperada:

1. cargar dataset de deltas;
2. reproducir KFs;
3. detectar candidato BoW;
4. verificar subnubes;
5. crear `LoopOptimizationTask`;
6. construir grafo usando `CovisibilityDatabase`;
7. ejecutar pipeline 1I–1L;
8. fusionar solo si ACCEPT.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 3 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si existe una prueba replay sin Gazebo más rápida y validada, `planificador_fase` puede usarla justificándolo.

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1P-|F1Q-COVIS|F1O-LOOP-OPT|F1N-SUBCLOUD|F1I-GRAPH|F1J-OPT|F1K-OPT-APPLY|F1L-POST-APPLY|F1O-FUSION|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1P-LOOP-POST-APPLY-REJECT|F1P-LOOP-ROLLBACK|F1P-LOOP-FUSION-SKIP|F1L-POSESTORE-ROLLBACK|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 3

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1P-|F1Q-COVIS|F1C-REPLAY|F1O-LOOP-OPT|F1N-SUBCLOUD|F1J-OPT|F1L-POST-APPLY|ERROR|FATAL|Segmentation fault|Killed
```

La reducción genera:

```text
codex/archivos_auxiliares/logs/prueba_1.reduced.log
codex/archivos_auxiliares/logs/prueba_2.reduced.log
codex/archivos_auxiliares/logs/prueba_3.reduced.log
```

Si el reducido no muestra suficiente información, revisar el log completo antes de repetir Gazebo o concluir que falta el marcador.

---
