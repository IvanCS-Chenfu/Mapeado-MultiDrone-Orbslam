## Marcadores tecnicos obligatorios

Para recepción de tarea:

```text
[F1P-LOOP-OPT-TASK-RX]
```

Para grafo:

```text
[F1P-LOOP-GRAPH-BUILD]
[F1P-LOOP-GRAPH-SUMMARY]
[F1P-LOOP-CONSTRAINT]
[F1Q-COVIS-GRAPH-EDGES]
```

Para dry-run:

```text
[F1P-LOOP-DRYRUN-RESULT]
```

Para apply:

```text
[F1P-LOOP-APPLY]
```

Para post-apply:

```text
[F1P-LOOP-POST-APPLY-ERROR]
```

Debe aparecer exactamente una de estas decisiones:

```text
[F1P-LOOP-POST-APPLY-ACCEPT]
[F1P-LOOP-POST-APPLY-PARTIAL]
[F1P-LOOP-POST-APPLY-REJECT]
```

Si ACCEPT:

```text
[F1Q-COVIS-EDGE-REFINED]
[F1P-LOOP-FUSION-AFTER-ACCEPT]
[F1P-LOOP-GLOBALMAP-PUBLISH]
```

Si REJECT:

```text
[F1P-LOOP-ROLLBACK]
[F1P-LOOP-FUSION-SKIP]
[F1P-LOOP-ROLLBACK-DIAGNOSTIC]
[F1P-LOOP-RETRY-SUGGESTION]
```

No debe aparecer:

```text
NEW_LOOP_OPTIMIZER_CREATED
RAWDB-POSE-OVERWRITE-BY-LOOP
GT_USED_FOR_LOOP
FUSION_BEFORE_HIGH_ERROR_OPT
FUSION_AFTER_OPT_REJECT
HARD-FIDUCIAL-MOVED_ACCEPTED
```

salvo como error explícito detectado y explicado.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. se ejecutan las pruebas requeridas o alternativas equivalentes justificadas;
3. `scenario_runner_node` se ejecuta en pruebas Gazebo que lo usen;
4. los goals requeridos terminan con `success=true` si la prueba tiene trayectoria;
5. se crea o procesa al menos un `LoopOptimizationTask`;
6. `PoseGraphBuilder` construye un `PoseGraphProblem` para loop usando las clases existentes;
7. el grafo contiene query side, candidate side, loop constraint, fijos/variables y propagación;
8. el grafo consulta `CovisibilityDatabase` y añade o justifica aristas de covisibilidad confirmada;
9. las aristas de covisibilidad usan `relative_pose_current` y no sobrescriben `relative_pose_measured`;
10. `OptimizationManager` ejecuta dry-run para loop;
11. se calcula before/predicted-after del error de loop;
12. si `useful=true`, se aplica usando la ruta de 1K;
13. se valida post-apply usando la ruta de 1L;
14. si ACCEPT, se actualiza `relative_pose_current` de aristas usadas cuando proceda;
15. si ACCEPT, se fusionan landmarks después de aceptar;
16. si REJECT, se hace rollback y no se fusiona;
17. `RawMapDatabase` no cambia por optimización;
18. no se usa GT para loops;
19. no se crea un optimizador paralelo;
20. no aparecen errores graves no explicados;
21. la documentación queda actualizada;
22. el historial registra evidencia positiva y negativa.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- no se puede crear/procesar `LoopOptimizationTask`;
- se crea un optimizador paralelo;
- el grafo de loop no tiene constraint de loop;
- el grafo ignora `CovisibilityDatabase` sin justificación;
- se sobrescribe `relative_pose_measured`;
- dry-run no calcula error de loop;
- apply modifica `RawMapDatabase`;
- se fusiona antes de optimizar un loop de error alto;
- se fusiona después de rollback;
- se usa GT para loop;
- se mueve un hard fiducial y aun así se acepta;
- falta un marcador obligatorio;
- aparece un error grave no explicado.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- se crea grafo de loop pero no se logra dry-run útil;
- dry-run funciona pero el resultado se rechaza correctamente;
- funciona en replay pero no en live;
- se acepta optimización pero no se logra fusión posterior por falta de inlier pairs válidos;
- no se logra un caso real de error alto y se usa caso sintético/debug controlado;
- RViz2 muestra mejora dudosa aunque los logs sean coherentes.

La subfase debe marcarse como `BLOQUEADA` solo si:

- 1P no puede crear `LoopOptimizationTask`;
- 1O no produce inliers/transformación;
- 1I–1L no están implementadas;
- no existe dataset/simulación con loop de error alto;
- falta información crítica y no se puede resolver con cambios mínimos.

---

## Documentacion a actualizar

Actualizar obligatoriamente:

- historial de Fase 1 / subfase 1Q;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- actualizar:
  - `codex/contexto/paquetes/orbslam3_multi/loop_decision_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/pose_graph_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_multi/fusion_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/subcloud_loop_verifier.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`.

La documentación debe explicar:

- que 1Q reutiliza 1I–1L;
- que no existe optimizador específico nuevo para loops;
- cómo cambia el constraint respecto al fiducial;
- qué contiene `LoopOptimizationTask`;
- cómo se añaden restricciones desde `CovisibilityDatabase`;
- cuándo se actualiza `relative_pose_current`;
- cómo se mide error before/predicted/real para loop;
- cuándo se fusiona después de optimizar;
- cuándo se hace rollback;
- qué logs validan la subfase;
- que no se usa GT para loops;
- que `RawMapDatabase` no cambia por optimización.

No marcar como `realizado` si no se cumplen los criterios de éxito.

---

## Notas de diseño que deben respetarse en fases posteriores

1. Fiducial y loop usan el mismo pipeline general:
   ```text
   OptimizationTask → PoseGraphProblem → DryRun → Apply → PostApplyValidation
   ```

2. La diferencia entre fiducial y loop está en:
   - constraints;
   - pesos;
   - selección de KFs;
   - métrica de error.

3. Un loop con error alto no se fusiona hasta que la optimización sea aceptada.

4. Un rollback de loop implica no fusionar y restaurar el mapa.

5. Las aristas confirmadas de `CovisibilityDatabase` son restricciones para no
   separar zonas covisibles durante optimizaciones fiduciales o de loop.

6. Los KFs nuevos de submapas optimizados por loop deben heredar correcciones igual que en optimización fiducial.

7. Si la optimización por loop falla repetidamente, debe sugerirse recomputar subnubes, cambiar ventana/pesos o rechazar el candidato.
