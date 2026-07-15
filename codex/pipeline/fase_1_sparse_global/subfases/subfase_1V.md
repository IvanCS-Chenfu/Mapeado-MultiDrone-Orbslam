# Subfase 1V — Pruebas integrales end-to-end y regresión de Fase 1

## Estado

```text
sin hacer
```

---

## Objetivo tecnico

Ejecutar una matriz completa de pruebas end-to-end para validar que toda la Fase 1 funciona de forma integrada.

Esta subfase no debe introducir lógica nueva. Debe comprobar que lo implementado en 1A–1U funciona junto:

```text
ingesta → raw db → pose store → fiduciales → publicación
→ loops → subnubes → fusión → optimización → rollback
→ KFs concurrentes → scoring → RViz/debug
```

---

## Contexto obligatorio a leer

Leer:

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `PIPELINE_MAESTRO.md`
- `pipeline_fase_1.md`
- subfases `1A` a `1U`
- historial de Fase 1
- documentación de paquetes:
  - `orbslam3_multi`
  - `orbslam3_server`
  - `simulacion_dron`

---

## Diagnostico de partida

La Fase 1 tiene muchas subfases unitarias. Falta una prueba integral que valide:

- que no hay regresiones entre subfases;
- que replay y live son coherentes;
- que rollback no contamina score/fusión;
- que full snapshot no pisa optimizaciones;
- que KFs concurrentes se reinyectan;
- que la nube publicada final es coherente;
- que los logs contienen métricas finales.

---

## Archivos permitidos a modificar

- scripts auxiliares de prueba en `codex/herramientas/`
- YAMLs en `codex/archivos_auxiliares/`
- launch/config de pruebas
- documentación e historial
- cambios mínimos de logging/metrics en `orbslam3_server` y `orbslam3_multi`

No modificar lógica principal salvo bug detectado y justificado.

---

## Archivos prohibidos

- `ORB_SLAM3/`
- wrapper salvo bug crítico
- cambios algorítmicos grandes
- refactors no necesarios
- borrar legacy

---

## Funciones, clases o nodos que hay que localizar

- `global_map_server`
- `scenario_runner_node`
- scripts `run_simulation.sh`
- scripts de reducción de logs
- replay de 1C
- publishers globales
- métricas de:
  - RawMapDatabase
  - GlobalPoseStore
  - LoopDetector
  - SubcloudLoopVerifier
  - FusionManager
  - OptimizationManager
  - LandmarkScoreManager
  - GlobalMapBuilder

---

## Cambios requeridos

### 1. Definir matriz de pruebas

Crear o consolidar una matriz:

```text
Test A: baseline simple.
Test B: dos drones al fiducial.
Test C: dos drones recorren edificio sentidos opuestos.
Test D: replay determinista.
Test E: full snapshot después de optimización.
Test F: loop error bajo → fusión.
Test G: loop error alto → optimización → fusión tras ACCEPT.
Test H: rollback de optimización mala.
Test I: KFs que llegan durante optimización.
Test J: score/fused publication.
```

Documentar qué pruebas son obligatorias y cuáles opcionales si no hay dataset.

---

### 2. Crear métricas finales unificadas

Al final de cada prueba emitir:

```text
[F1U-E2E-METRICS]
```

Con:

```text
raw_keyframes
global_keyframes
raw_mappoints
fused_tracks
score_events
loop_candidates
verified_loops
fusion_events
optimization_tasks
optimization_accept
optimization_partial
optimization_reject
rollbacks
pending_kfs
rebased_kfs
published_points
nan_points
runtime_sec
max_memory_mb si disponible
```

---

### 3. Validar replay determinista

Para replay:

```text
mismo dataset → mismos arrival_id → mismas métricas clave
```

Permitir pequeñas diferencias solo si están justificadas.

Logs:

```text
[F1U-REPLAY-DETERMINISM]
```

---

### 4. Validar no contaminación tras rollback

Tras rollback:

```text
poses restauradas
scores restaurados
fused tracks de task rechazado no activos
nube publicada restaurada
```

Logs:

```text
[F1U-ROLLBACK-CONSISTENCY]
```

---

### 5. Validar publicación final

Comprobar:

```text
nube publicada no vacía
sin NaNs
fused tracks sin duplicados raw principales
score stats razonables
```

Logs:

```text
[F1U-PUBLISH-CONSISTENCY]
```

---

### 6. Resumen final

Al terminar toda la matriz:

```text
[F1U-E2E-PASS]
```

o

```text
[F1U-E2E-FAIL]
```

Con lista de pruebas.

---

## Cambios prohibidos

- No ajustar algoritmos durante la prueba sin documentar.
- No ignorar errores graves.
- No marcar PASS si faltan pruebas obligatorias.
- No borrar logs completos.
- No usar solo RViz como validación automática.

---

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron
```

---

## Pruebas Gazebo requeridas

### Prueba 1 — simple fiducial

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_simple_fiducial.yaml
```

### Prueba 2 — dos drones trayectoria corta

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_dos_drones_corta.yaml
```

### Prueba 3 — recorrido edificio / loop

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_loop_edificio.yaml
```

Comando general:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba <N> \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

---

## Pruebas replay requeridas

### Prueba 4 — replay dataset clave

Usar dataset guardado en 1C/1N/1O.

### Prueba 5 — replay rollback/debug

Usar dataset o modo debug para forzar rechazo.

---

## Patrones de reduccion de logs

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1U-|F1[A-Z]-|ERROR|FATAL|Segmentation fault|Killed
```

---

## Marcadores tecnicos obligatorios

```text
[F1U-E2E-START]
[F1U-E2E-SCENARIO]
[F1U-E2E-METRICS]
[F1U-PUBLISH-CONSISTENCY]
[F1U-E2E-PASS]
```

Si replay:

```text
[F1U-REPLAY-DETERMINISM]
```

Si rollback:

```text
[F1U-ROLLBACK-CONSISTENCY]
```

---

## Criterio de exito

`CONSEGUIDA` si:

1. compila;
2. se ejecuta matriz mínima live/replay;
3. todas las pruebas obligatorias terminan con success;
4. no hay errores graves;
5. métricas finales coherentes;
6. rollback/fusión/optimización no contaminan estado;
7. nube final publicable;
8. documentación/historial actualizados.

---

## Criterio de fallo o parcial

`NO CONSEGUIDA` si:

- no compila;
- fallan pruebas obligatorias;
- hay NaNs o crashes;
- rollback contamina estado;
- replay no es reproducible sin explicación.

`PARCIAL` si:

- parte de la matriz pasa;
- faltan escenarios complejos por dataset;
- funciona replay pero no live o viceversa.

---

## Documentacion a actualizar

Actualizar:

- historial final de Fase 1;
- `codex/contexto/01_ESTADO_ACTUAL.md`;
- `codex/contexto/paquetes/*`;
- lista de datasets clave;
- tabla de pruebas y resultados.
