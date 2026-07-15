## Cambios prohibidos

- No llamar a `FusionManager`.
- No crear ni aplicar fusiones.
- No crear `LoopOptimizationTask` todavía.
- No llamar a `PoseGraphBuilder` por loop.
- No llamar a `OptimizationManager` por loop.
- No modificar `GlobalPoseStore`.
- No modificar `RawMapDatabase`.
- No usar GT para loops.
- No tratar BoW como loop confirmado.
- No insertar relaciones en `CovisibilityDatabase`; 1O solo produce evidencia.
- No usar solo los puntos del candidate_seed como candidate_subcloud final.
- No reducir candidate_subcloud con una caja basada en outliers sin fallback.
- No aceptar RANSAC degenerado.
- No romper rutas de fiducial/optimización fiducial existentes.

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

### Prueba 1 — subcloud verifier live multi-dron

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. arrancar simulación multi-dron;
2. mover drones por trayectoria con revisitas o zonas visualmente similares;
3. generar KFs y candidatos BoW con 1N;
4. comprobar que candidatos ya confirmados por `CovisibilityDatabase` se saltan;
5. verificar candidatos no confirmados con `SubcloudLoopVerifier`;
6. construir query_subcloud y candidate_subcloud;
7. hacer matching inicial, reducción espacial, matching refinado y RANSAC;
8. loggear decisión preliminar y pose relativa medida si hay confirmación;
9. no fusionar ni optimizar.

Puede reutilizarse una prueba clave de 1C/1N si produce suficientes candidatos visuales.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observación esperada en RViz2:

```text
No es obligatoria para declarar la subfase.
Opcionalmente se pueden publicar markers debug de subnubes/matches,
pero la validación principal es por logs.
```

---

## Pruebas replay requeridas

### Prueba 2 — subcloud verifier desde dataset guardado

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Secuencia esperada:

1. cargar dataset de deltas guardado;
2. reproducir KFs;
3. generar candidatos BoW;
4. saltar candidatos ya confirmados por `CovisibilityDatabase`;
5. verificar candidatos no confirmados con subnubes;
6. comprobar que los resultados son repetibles sin repetir simulación larga.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si existe una prueba replay sin Gazebo más rápida y validada, `planificador_fase` puede usarla justificándolo.

---

## Prueba recomendada adicional

### Prueba 3 — reducción de candidate_subcloud por matches

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml
```

Objetivo:

Validar específicamente que la candidate_subcloud inicial se reduce usando la zona espacial robusta de matches ORB.

Logs esperados:

```text
[F1N-SUBCLOUD-CANDIDATE-REDUCE-MATCH-BOX]
[F1N-SUBCLOUD-CANDIDATE-REDUCE-STATS]
[F1N-SUBCLOUD-MATCH-REFINED]
```

Debe verse:

```text
candidate_initial_points > candidate_reduced_points
fallback=false
```

o, si hay pocos matches:

```text
fallback=true
reason=not_enough_matches
```

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1O-SUBCLOUD-SKIP-CONFIRMED-COVIS|F1M-COVIS|F1C-RAWDB|F1F-GLOBALMAP|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1N-|F1O-SUBCLOUD-SKIP-CONFIRMED-COVIS|F1M-COVIS|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 3

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1N-SUBCLOUD-CANDIDATE-REDUCE|F1N-SUBCLOUD-MATCH-REFINED|fallback|ERROR|FATAL|Segmentation fault|Killed
```

La reducción genera:

```text
codex/archivos_auxiliares/logs/prueba_1.reduced.log
codex/archivos_auxiliares/logs/prueba_2.reduced.log
codex/archivos_auxiliares/logs/prueba_3.reduced.log
```

Si el reducido no muestra suficiente información, revisar el log completo antes de repetir Gazebo o concluir que falta el marcador.

---
