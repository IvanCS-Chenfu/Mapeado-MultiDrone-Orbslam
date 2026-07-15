## Cambios prohibidos

- No borrar MapPoints originales de `RawMapDatabase`.
- No modificar poses en `GlobalPoseStore` por fusión.
- No fusionar si `LoopVerificationResult` es `REJECT`.
- No insertar covisibilidad si `LoopVerificationResult` es `REJECT` o `HOLD`.
- No insertar covisibilidad por BoW sin geometría.
- No fusionar antes de optimización cuando el error de loop es alto.
- No fusionar después de optimización si 1L rechaza o hace rollback.
- No bajar score de todos los puntos sin match indiscriminadamente.
- No hacer media float de descriptores ORB.
- No publicar miembros raw duplicados en la nube fused principal.
- No usar GT para loops.
- No ejecutar optimización fuera del pipeline 1I–1L.
- No ocultar fallos de RANSAC con fusión agresiva.
- No modificar rutas de fiducial salvo integración mínima.

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

### Prueba 1 — fusión por loop de error bajo

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. arrancar simulación multi-dron;
2. generar candidatos BoW;
3. verificar candidato con subnubes/RANSAC;
4. obtener `FUSION_CANDIDATE`;
5. insertar covisibilidad confirmada con pose relativa medida;
6. fusionar inlier pairs;
7. actualizar scores por eventos;
8. publicar nube fused;
9. comprobar que no se publican duplicados raw de tracks fusionados.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observación esperada en RViz2:

```text
La nube fused debería mostrar menos duplicados en la zona fusionada.
El usuario podrá revisar visualmente si aparecen dobles paredes o puntos duplicados.
La validación automática principal es por logs.
```

---

### Prueba 2 — loop de error alto crea tarea de optimización

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Secuencia esperada:

1. generar candidato BoW;
2. verificar subnubes;
3. obtener `LOOP_OPTIMIZATION_CANDIDATE`;
4. insertar covisibilidad confirmada como restricción para 1Q;
5. crear `LoopOptimizationTask`;
6. no fusionar todavía;
7. comprobar logs de fusión diferida.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si no se logra un loop real de error alto, se puede usar un caso replay/debug controlado y documentado.

---

## Pruebas replay requeridas

### Prueba 3 — fusión desde dataset guardado

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml
```

Secuencia esperada:

1. cargar dataset de deltas;
2. reproducir KFs;
3. generar candidatos BoW;
4. verificar subnubes;
5. insertar covisibilidad confirmada si hay `FUSION_CANDIDATE`;
6. fusionar si hay `FUSION_CANDIDATE`;
7. comprobar tracks fused y scores.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 3 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si existe prueba replay sin Gazebo más rápida, `planificador_fase` puede usarla justificándolo.

---

## Prueba recomendada adicional

### Prueba 4 — score de unmatched esperado

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_4.yaml
```

Objetivo:

Validar que no se penalizan todos los no asociados indiscriminadamente.

Logs esperados:

```text
[F1O-SCORE-UNMATCHED-CANDIDATE]
[F1O-SCORE-UNMATCHED-SKIP]
```

Debe comprobarse que solo se penalizan puntos dentro de solape confirmado.

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1O-|F1P-COVIS|F1N-SUBCLOUD|F1M-COVIS|F1F-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1O-LOOP-OPT-TASK-CREATED|F1O-FUSION-DEFERRED|F1P-COVIS|F1N-SUBCLOUD|F1I-GRAPH|F1J-OPT|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 3

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1O-FUSION|F1O-FUSED|F1O-SCORE|F1O-GLOBALMAP|F1C-REPLAY|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 4

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1O-SCORE-UNMATCHED|F1O-SCORE-UPDATE|F1O-SCORE-EVENT|ERROR|FATAL|Segmentation fault|Killed
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
