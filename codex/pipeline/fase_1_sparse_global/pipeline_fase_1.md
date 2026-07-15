# Pipeline Fase 1 — Mapa sparse global multi-dron

Resumen barato:

```text
codex/pipeline/fase_1_sparse_global/pipeline_fase_1_RESUMEN.md
```

Abrir este archivo completo solo si el resumen o la subfase concreta no bastan.

## Estado

```text
actual
```

Nota de reapertura 2026-07-10:

```text
La ruta 1I-1L se reabre por un fallo observado en RViz2 y confirmado en logs:
una optimizacion fiducial del dron 2 corrigio el fiducial objetivo, pero el grafo
llego con fixed=0 hard_fiducial=0 y pudo desplazar KFs del fiducial previo.
```

El 2026-07-11 se aislo `dron_2` antihorario. El pipeline ya fija el fiducial previo y tres KFs vecinos, interpola KFs no variables y reconstruye el mismo task desde el checkpoint; en logs llega de `28.18 m` a `1.41 m` y despues a `0.07 m`. Aun asi, la primera pasada genera residuales internos de hasta `49-60 m` sobre ventanas de mas de 100 KFs y RViz2 ha mostrado KFs/puntos separados. El 2026-07-12 se anadio cobertura dura del grafo: la segunda pasada se puede rechazar antes del solver con `[F1I-GRAPH-REJECT] reason=bad_window_coverage`. El 2026-07-14 `1L` queda como `PARCIAL`: la ruta fiducial se volvera a comprobar en el futuro, y la deuda de covisibilidad/loops no se absorbe en `1L`. La subfase activa pasa a ser `1M`: crear `CovisibilityDatabase` confirmada para importar covisibilidad ORB-SLAM3, registrar loops geométricos confirmados y alimentar optimizaciones posteriores.

## Fuente de verdad de subfases

La planificación activa de Fase 1 está compuesta por:

```text
subfase_1A.md
subfase_1B.md
...
subfase_1X.md
```

Cada archivo de subfase es fuente de verdad ejecutable para objetivo, cambios permitidos, paquetes a compilar, pruebas, patrones de log y criterios de éxito. No debe usarse como historial largo: el detalle de intentos, HTMLs concretos, logs y resultados descartados vive en `historial/por_subfase/historial_<ID>.md`.

Las subfases residuales `12R-*`, `13`, `14` y `15` son legacy/solo referencia. No forman parte del plan activo.

## Objetivo de la fase

Construir un mapa sparse global multi-dron coherente, estable, anclado, fusionado y optimizable. La Fase 1 debe producir una base suficiente para estimar poses globales sin GT en Fase 2 y para alimentar la futura nube densa.

El objetivo no es reiniciar el proyecto desde cero. El objetivo es migrar de forma controlada desde el servidor monolítico acumulado hacia una arquitectura donde:

- `orbslam3_server` sea adaptador ROS 2;
- `orbslam3_multi` concentre el backend algorítmico;
- los datos raw de ORB-SLAM3 queden separados de poses globales optimizadas;
- la fusión y scoring sean centralizados;
- los loops y fiduciales usen pipelines separados pero compartan optimización cuando toque;
- cada paso compile, se pruebe y quede documentado.

## Arquitectura objetivo

```text
orbslam3_ros2 / wrapper estéreo
    publica OrbMap delta
    ofrece GetOrbMap snapshot
    publica pose_local
        ↓
orbslam3_server
    recibe ROS
    pide snapshots
    lee GT solo para fiducial simulado/debug
    convierte ROS -> estructuras internas
    publica resultados
        ↓
orbslam3_multi
    RawMapDatabase
    GlobalPoseStore
    FiducialAnchorManager
    LoopDetector
    SubcloudLoopVerifier
    LoopDecisionManager
    FusionManager / FusedLandmarkManager
    LandmarkScoreManager
    PoseGraphBuilder
    OptimizationManager
    GlobalMapBuilder
```

## Responsabilidades clave

### `RawMapDatabase`

Guarda datos crudos de ORB-SLAM3:

- KeyFrames;
- MapPoints;
- BoW/FeatureVector;
- descriptores;
- observaciones;
- covisibilidad;
- poses locales;
- deltas;
- full snapshots;
- journal/replay.

No se modifica por optimización y no borra MapPoints por fusión.

### `GlobalPoseStore`

Guarda estado global:

- anchors por fiducial/loop;
- poses globales;
- poses optimizadas;
- poses propagadas;
- KFs rebasados;
- correcciones heredables;
- protección frente a full snapshots posteriores.

### `FiducialAnchorManager`

Recibe observaciones fiduciales construidas por el servidor. En primera visita ancla el submapa. En visitas posteriores mide error absoluto y crea tarea de optimización si hace falta.

### `PoseGraphBuilder`

Construye grafos temporales bajo demanda. No mantiene grafo global permanente. Sirve para fiduciales y loops.

### `OptimizationManager`

Ejecuta dry-run, aplica solo si es útil, escribe en `GlobalPoseStore`, valida post-apply y permite rollback.

### `LoopDetector`

Genera candidatos BoW. BoW no confirma loops. La búsqueda debe ser amplia, no solo por cercanía espacial.

### `SubcloudLoopVerifier`

Construye `query_subcloud` desde el KF nuevo y `candidate_subcloud` como zona global alrededor del candidate seed. Ejecuta matching ORB, reducción espacial, matching refinado y RANSAC.

Devuelve:

```text
REJECT
HOLD
FUSION_CANDIDATE
LOOP_OPTIMIZATION_CANDIDATE
```

### `LoopDecisionManager`

Decide qué hacer con el resultado geométrico. Error bajo y geometría buena activa fusión. Error alto y geometría buena crea `LoopOptimizationTask`. Si hay optimización, la fusión se hace solo tras validación post-apply aceptada.

### `FusionManager` / `FusedLandmarkManager`

Fusiona landmarks sin borrar MapPoints raw. Crea tracks fused y publica un solo punto fused en vez de duplicados raw. La posición fused usa media ponderada o estimación robusta; el descriptor fused debe ser representativo/medoid.

### `LandmarkScoreManager`

Centraliza todo el scoring. Nadie debe hacer `score += X` fuera de esta clase. Las demás clases emiten eventos semánticos y el manager permite rollback de eventos de score.

### `GlobalMapBuilder`

Publica la nube global usando `GlobalPoseStore`, `FusedLandmarkManager` y `LandmarkScoreManager`. No publica miembros raw duplicados si ya pertenecen a un fused track publicable.

## Tabla activa de subfases 1A-1X

| Subfase | Estado inicial | Objetivo corto | Entrada | Salida | Validación principal |
|---|---|---|---|---|---|
| 1A | `realizado` | Baseline del servidor antiguo y logs actuales. | Código actual, launch oficial y servidor monolítico. | Logs completos/reducidos, inventario de comportamiento y problemas. | `CONSEGUIDA` el 2026-07-03: build, simulación y logs documentan el estado real sin corregirlo. |
| 1B | `realizado` | Servidor nuevo mínimo y migración segura. | Baseline 1A y servidor legacy congelado. | `global_map_server` mínimo que recibe `OrbMap`, logs F1B y comentarios trazables por subfase. | `CONSEGUIDA` el 2026-07-06: build, simulación, logs y trazabilidad de comentarios validados. |
| 1C | `realizado` | `RawMapDatabase`, deltas/full snapshots y replay. | `OrbMap` recibidos por servidor nuevo. | Raw DB por `(drone_id, map_epoch)`, journal y replay. | `CONSEGUIDA` el 2026-07-06: 284 deltas grabados, dataset raw guardado y replay completo sin Gazebo. |
| 1D | `realizado` | `GlobalPoseStore`. | Raw DB con KFs y poses locales. | Capa de poses globales, anchors y consultas. | `CONSEGUIDA` el 2026-07-08: build, replay, anchor/corrección sintéticos y herencia de corrección validados. |
| 1E | `realizado` | `FiducialAnchorManager`. | Observación fiducial simulada asociada a KF. | Submapa anclado, KFs existentes con pose global derivada y `.record` v2 con observaciones fiduciales. | `CONSEGUIDA` el 2026-07-08: build, Gazebo live, record nuevo y replay con 60 observaciones fiduciales aceptadas. |
| 1F | `realizado` | Publicar nube global sparse con scores iniciales. | Raw DB + submapas anclados + scores raw iniciales. | `GlobalMapBuilder`, `LandmarkScoreManager` inicial y `/global_sparse_cloud`. | `CONSEGUIDA` el 2026-07-08: build, Gazebo live, replay lento y logs muestran `PointCloud2` en `world` con `points_published=22394`. |
| 1G | `realizado` | Reconciliación con full snapshots de ORB-SLAM3. | Deltas previos, snapshots y poses globales. | Raw DB reconciliada sin destruir poses optimizadas. | `CONSEGUIDA` el 2026-07-08: live y replay muestran 12 full snapshots grabados/reproducidos, KFs insertados/actualizados y reconciliación `anchor_rebased`/`optimized_kept`. |
| 1H | `realizado` | Segunda visita a fiducial y tarea de optimización. | Submapa ya anclado y nueva observación fiducial. | Error fiducial medido y `FiducialOptimizationTask` preparada si procede. | `CONSEGUIDA` el 2026-07-09: live y replay muestran `[F1H-FID-REVISIT]`, `[F1H-FID-POSE-ERROR]`, `[F1H-FID-OK]` y `[F1H-FID-TASK-STATS]`; no se optimiza todavia. |
| 1I | `realizado` | Construcción temporal de grafo con preservación de anclajes previos. | Tarea fiducial/loop y datos raw/globales. | `PoseGraphProblem` temporal, plan de propagación y fiduciales previos protegidos. | `CONSEGUIDA` el 2026-07-10: pruebas largas muestran `[F1I-GRAPH-ANCHOR-PRESERVATION] ... satisfied=true` y `[F1I-GRAPH-BUILD-SUMMARY] ... fixed=1 hard_fiducial=1`. |
| 1J | `realizado` | Dry-run, replay offline y HTML diagnóstico del grafo. | `PoseGraphProblem`. | `OptimizationDryRunResult` sin modificar estado persistente; HTML/debug del grafo para inspección. | El HTML del grafo pertenece conceptualmente a `1J`; no debe requerir apply. |
| 1K | `realizado/parcial` | Apply del grafo en `GlobalPoseStore`. | Dry-run útil o candidato aceptable de 1J. | Poses optimizadas/propagadas, correcciones heredables, publicación de nube desde poses finales de KFs y KFs futuros heredando el extremo optimizado. | La aplicación del grafo pertenece a `1K`; `RawMapDatabase` sigue intacto. |
| 1L | `parcial` | Diagnóstico post-apply. | Estado publicado por 1K. | Logs, GT debug, RViz2/HTML comparativo y decisión documental de si la optimización sirve. | `1L` no debe ser dueña del solver ni del apply; queda para revalidación futura. |
| 1M | `actual/sin hacer` | `CovisibilityDatabase` confirmada. | Raw DB con covisibilidad ORB-SLAM3 y loops geométricos futuros. | Base consultable por detector/optimizador, con pose relativa medida/current. | No guardar candidatos: solo relaciones confirmadas. |
| 1N | `sin hacer` | `LoopDetector` BoW. | KF nuevo con BoW en Raw DB + `CovisibilityDatabase`. | Candidatos BoW rankeados, saltando pares ya confirmados. | No iniciar hasta completar `1M`, porque BoW debe consultar la base confirmada. |
| 1O | `sin hacer` | Subnubes, matching ORB, reducción espacial y RANSAC. | Candidato BoW. | `LoopVerificationResult`. | Subnubes verifican/rechazan con geometría real. |
| 1P | `sin hacer` | Decisión de loop, fusión y scoring multi-dron. | `LoopVerificationResult`. | Fusión o `LoopOptimizationTask`. | Error bajo fusiona; error alto crea tarea sin fusionar aún. |
| 1Q | `sin hacer` | Optimización por loop reutilizando 1I-1L. | `LoopOptimizationTask`. | Corrección por loop validada o rollback. | Loop usa el mismo pipeline de optimización y fusiona solo tras accept. |
| 1R | `sin hacer` | Reprocesar KFs llegados durante optimización. | KFs pendientes de rebase. | KFs rebasados y reinyectados al pipeline de loops. | No se pierden KFs ni se reprocesan indefinidamente. |
| 1S | `sin hacer` | Consolidar scoring de landmarks y fused tracks. | Eventos raw/fusión/no match/rollback. | Política única de score en `LandmarkScoreManager`. | Nadie modifica score fuera del manager y rollback funciona. |
| 1T | `sin hacer` | Contratos globales, frames, IDs e invariantes. | Sistema 1A-1S. | Contratos documentados y verificaciones. | IDs, frames, ownership e invariantes quedan consistentes. |
| 1U | `sin hacer` | Observabilidad final en RViz2 y topics debug. | Pipeline funcional. | Topics/markers debug para KFs, loops, scores, grafos y rollback. | RViz2/logs permiten inspección completa. |
| 1V | `sin hacer` | Pruebas integrales end-to-end y regresión. | Sistema 1A-1U. | Matriz de pruebas y resultados. | End-to-end valida ingesta, fiduciales, loops, fusión, optimización y rollback. |
| 1W | `sin hacer` | Rendimiento, límites y robustez. | Resultados 1V. | Límites, degradación segura y debug desactivable. | Tiempos/memoria/colas quedan acotados. |
| 1X | `sin hacer` | Cierre documental, legacy y handoff. | Fase validada. | Documentación final, legacy marcado y traspaso a Fase 2. | Un chat nuevo entiende estado, comandos, límites y siguiente fase. |

## Orden de implementación

1. Ejecutar 1A para capturar baseline.
2. Ejecutar 1B para crear servidor nuevo mínimo y congelar legacy.
3. Implementar almacenamiento raw y pose store: 1C-1D.
4. Añadir fiduciales, publicación inicial y reconciliación de snapshots: 1E-1G.
5. Añadir optimización fiducial segura: 1H-1L. Propiedad vigente: `1J` calcula dry-run y HTML; `1K` aplica en `GlobalPoseStore` y publica; `1L` diagnostica con logs/RViz2/GT debug y queda `PARCIAL` para revalidación futura.
6. Crear `CovisibilityDatabase` confirmada en `1M`.
7. Añadir loops, subnubes, fusión y optimización por loop: 1N-1Q, solo despues de completar `1M`.
8. Cerrar concurrencia, scoring e invariantes: 1R-1T.
9. Cerrar observabilidad, regresión, rendimiento y handoff: 1U-1X.

## Legacy

Subfases residuales presentes:

```text
subfase_12R-D4.md
subfase_12R-D5.md
subfase_12R-E.md
subfase_12R-F.md
subfase_13_fiducial_pose_window_error.md
subfase_14_optimizacion_global_fiducial.md
subfase_15_limpieza_legacy.md
```

Regla: no ejecutarlas como plan activo. Consultarlas solo para entender decisiones previas, logs y problemas del servidor monolítico.

Índice específico:

```text
codex/pipeline/fase_1_sparse_global/LEGACY_SUBFASES.md
```

## Herramientas

Build habitual de Fase 1:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

Si la subfase o una reparación del workspace exige compilar paquetes pesados, no juntarlos todos en una única llamada. Compilar en orden y de uno en uno, por ejemplo:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs
./codex/herramientas/build_selected_packages.sh orbslam3
./codex/herramientas/build_selected_packages.sh orbslam3_multi
./codex/herramientas/build_selected_packages.sh orbslam3_server
./codex/herramientas/build_selected_packages.sh dron_individual
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

Esta regla evita bloquear el ordenador durante builds grandes y facilita diagnosticar el primer error real.

Si falla:

```bash
./codex/herramientas/reduce_build_log.sh
```

Simulación habitual:

```bash
./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py" --post-scenario-wait-sec 20
```

Reducción:

```bash
./codex/herramientas/reduce_simulation_log.sh --prueba 1 --patterns "<patrones>"
```

Los patrones exactos los define cada subfase.

## Criterio de fin de Fase 1

La Fase 1 termina cuando:

- el mapa sparse global fused es coherente;
- los submapas por `(drone_id, map_epoch)` se mantienen;
- fiduciales y loops están separados;
- los fiduciales anclan submapas y pueden crear optimización;
- loops buenos fusionan landmarks;
- loops con error alto optimizan por el pipeline común;
- `RawMapDatabase` conserva raw intacto;
- `GlobalPoseStore` gestiona poses globales y correcciones;
- rollback funciona;
- scoring está centralizado;
- la nube publicada no duplica raw ya fusionado;
- `orbslam3_server` queda reducido a ROS/adaptación/publicación;
- hay pruebas integrales y documentación suficiente para arrancar Fase 2.
