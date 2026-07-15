## Marcadores tecnicos obligatorios

Para llegada durante optimización:

```text
[F1Q-KF-ARRIVED-DURING-OPT]
[F1Q-KF-MARK-PENDING]
```

Para optimización aceptada:

```text
[F1Q-OPT-FINISHED]
[F1Q-KF-REBASED]
[F1Q-KF-QUEUE-LOOP-CHECK]
```

Para dispatch:

```text
[F1Q-LOOP-CHECK-DISPATCH]
[F1Q-LOOP-CHECK-RESULT]
```

Para optimización rechazada:

```text
[F1Q-OPT-REJECTED-NO-REBASE]
[F1Q-LOOP-CHECK-SKIP]
```

Para límites:

```text
[F1Q-CASCADE-LIMIT]
```

si se alcanza límite.

No debe aparecer:

```text
POST_OPT_LOOP_WITH_REJECTED_CORRECTION
POST_OPT_GT_USED_FOR_LOOP
POST_OPT_DIRECT_FUSION_WITHOUT_1M_1N_1O
POST_OPT_INFINITE_CASCADE
RAWDB-POSE-OVERWRITE-BY-REBASE
```

salvo como error explícito detectado y explicado.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. se ejecutan las pruebas requeridas o alternativas equivalentes justificadas;
3. `scenario_runner_node` se ejecuta en pruebas Gazebo que lo usen;
4. los goals requeridos terminan con `success=true` si la prueba tiene trayectoria;
5. KFs llegados durante optimización se insertan en `RawMapDatabase`;
6. esos KFs se marcan como pendientes;
7. tras una optimización ACCEPT, se rebasan con corrección válida;
8. tras rebase, se encolan para loop check;
9. la cola despacha KFs al pipeline 1N–1P;
10. si se genera `LoopOptimizationTask`, se envía a 1Q;
11. si la optimización se rechaza, no se usa su corrección para rebase;
12. no se usa GT para loops;
13. no hay cascada infinita;
14. `RawMapDatabase` no cambia por rebase;
15. no aparecen errores graves no explicados;
16. la documentación queda actualizada;
17. el historial registra evidencia positiva y negativa.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- KFs llegados durante optimización se pierden;
- no se marcan pendientes;
- se rebasan con una optimización rechazada;
- no se encolan después de ACCEPT;
- no se despachan a 1N–1P;
- se fusiona directamente sin pasar por 1N–1P;
- se usa GT para loops;
- aparece una cascada sin límite;
- se modifica `RawMapDatabase` por rebase;
- falta un marcador obligatorio;
- aparece un error grave no explicado.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- funciona en replay pero no en live;
- marca pendientes y rebasea, pero no se logra candidato loop;
- despacha a 1N, pero 1O/1P rechazan por falta de evidencia;
- no se logra prueba con rollback, pero el caso ACCEPT funciona;
- los límites de cascada están implementados pero no se consiguen activar en prueba;
- la cola funciona pero falta export debug.

La subfase debe marcarse como `BLOQUEADA` solo si:

- 1K no implementa rebase de KFs pendientes;
- 1L no devuelve ACCEPT/REJECT usable;
- 1N–1P no están implementadas;
- no existe forma de simular KFs durante optimización;
- falta información crítica y no se puede resolver con cambios mínimos.

---

## Documentacion a actualizar

Actualizar obligatoriamente:

- historial de Fase 1 / subfase 1R;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- actualizar:
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
  - `codex/contexto/paquetes/orbslam3_multi/optimization_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/loop_detector.md`;
  - `codex/contexto/paquetes/orbslam3_multi/subcloud_loop_verifier.md`;
  - `codex/contexto/paquetes/orbslam3_multi/loop_decision_manager.md`;
- crear o actualizar:
  - `codex/contexto/paquetes/orbslam3_multi/post_optimization_kf_queue.md`;
- actualizar:
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`.

La documentación debe explicar:

- qué significa un KF pendiente durante optimización;
- cómo se rebasa después de ACCEPT;
- por qué se reinyecta a 1N–1P;
- qué pasa si la optimización es REJECT;
- qué pasa si es PARTIAL;
- cómo se evitan reprocesados duplicados;
- cómo se evitan cascadas infinitas;
- qué logs validan esta subfase.

No marcar como `realizado` si no se cumplen los criterios de éxito.

---

## Notas de diseño que deben respetarse en fases posteriores

1. Un KF rebasado tras optimización aceptada debe volver a pasar por loop detection.
2. La cola post-optimización debe usar el pipeline normal 1N–1P, no una ruta especial.
3. Si el reprocesado genera un loop de error alto, se debe usar 1Q.
4. Si una optimización se rechaza, no se usa su corrección para reinyectar KFs.
5. Deben existir límites de cascada para evitar bucles infinitos.
6. `RawMapDatabase` sigue siendo raw e inmutable por optimización/rebase.
7. Esta subfase cierra la coherencia temporal entre optimización y llegada continua de datos.
