## Marcadores tecnicos obligatorios

Para decisión:

```text
[F1O-LOOP-DECISION]
[F1O-LOOP-DECISION-SUMMARY]
[F1P-COVIS-EDGE-CONFIRMED]
```

Si la arista ya existía:

```text
[F1P-COVIS-EDGE-ALREADY-KNOWN]
```

Para fusión:

```text
[F1O-FUSION-EVENT]
[F1O-FUSION-PAIR]
[F1O-FUSED-TRACK-CREATE]
```

o, si se actualiza track existente:

```text
[F1O-FUSED-TRACK-UPDATE]
```

Para posición/descriptor fused:

```text
[F1O-FUSED-POSITION-UPDATE]
[F1O-FUSED-DESCRIPTOR-UPDATE]
```

Para scoring:

```text
[F1O-SCORE-EVENT]
[F1O-SCORE-UPDATE]
```

Para publicación:

```text
[F1O-GLOBALMAP-FUSED-BUILD]
[F1O-GLOBALMAP-FUSED-PUBLISH]
```

Si hay loop de error alto:

```text
[F1O-LOOP-OPT-TASK-CREATED]
[F1O-FUSION-DEFERRED-UNTIL-OPT-ACCEPT]
```

No debe aparecer:

```text
RAWDB-MAPPOINT-DELETED-BY-FUSION
FUSION_BEFORE_HIGH_ERROR_OPT
FUSION_AFTER_OPT_REJECT
GT_USED_FOR_LOOP
```

salvo como error explícito detectado y explicado.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. se ejecutan las pruebas requeridas o alternativas equivalentes justificadas;
3. `scenario_runner_node` se ejecuta en pruebas Gazebo que lo usen;
4. los goals requeridos terminan con `success=true` si la prueba tiene trayectoria;
5. `LoopDecisionManager` procesa `LoopVerificationResult`;
6. para `FUSION_CANDIDATE`, se crean o actualizan tracks fused;
7. se actualiza posición fused por media ponderada;
8. se asigna descriptor fused mediante política válida para ORB;
9. se actualizan scores mediante eventos semánticos;
10. no se penalizan indiscriminadamente todos los puntos sin match;
11. `GlobalMapBuilder` publica fused tracks preferentemente;
12. miembros raw de tracks fused no se publican duplicados en la nube principal;
13. para `FUSION_CANDIDATE`, se inserta covisibilidad confirmada;
14. para `LOOP_OPTIMIZATION_CANDIDATE`, se inserta covisibilidad confirmada,
    se crea `LoopOptimizationTask` y se difiere la fusión;
15. no se inserta covisibilidad en `REJECT/HOLD`;
16. no se fusiona si la optimización posterior es rechazada;
17. `RawMapDatabase` no borra ni fusiona MapPoints originales;
18. no se usa GT para loops;
19. no aparecen errores graves no explicados;
20. la documentación queda actualizada;
21. el historial registra evidencia positiva y negativa.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- no se procesa ningún `LoopVerificationResult`;
- no se crea ningún track fused en un caso `FUSION_CANDIDATE`;
- se fusiona antes de optimizar un caso de error alto;
- se fusiona después de un post-apply rechazado;
- se borran MapPoints originales de `RawMapDatabase`;
- se bajan scores indiscriminadamente de todos los no asociados;
- se hace media float de descriptores ORB;
- se publican duplicados raw y fused en la nube principal;
- se usa GT para loops;
- se inserta covisibilidad por BoW sin geometría;
- no se inserta covisibilidad tras un loop geométricamente confirmado;
- falta un marcador obligatorio;
- aparece un error grave no explicado.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- se crea `LoopOptimizationTask` pero no se logra caso real de fusión;
- se fusionan tracks pero falta política avanzada de descriptor;
- se actualizan scores de fusionados pero no se valida penalización de unmatched;
- funciona en replay pero no en live;
- la nube fused se publica pero RViz2 muestra duplicados que requieren ajuste;
- no se logra probar post-opt fusion porque no hay loop de error alto aceptado.

La subfase debe marcarse como `BLOQUEADA` solo si:

- 1O no produce `LoopVerificationResult`;
- no hay inlier pairs con MapPoint IDs;
- `RawMapDatabase` no guarda datos necesarios para fusionar;
- no existe dataset/simulación con candidatos suficientes;
- falta información crítica y no se puede resolver con cambios mínimos.

---

## Documentacion a actualizar

Actualizar obligatoriamente:

- historial de Fase 1 / subfase 1P;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- crear o actualizar:
  - `codex/contexto/paquetes/orbslam3_multi/loop_decision_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/fusion_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/fused_landmark_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/landmark_score_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`;
- actualizar:
  - `codex/contexto/paquetes/orbslam3_multi/subcloud_loop_verifier.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`.

La documentación debe explicar:

- diferencia entre `RawMapDatabase` y `FusedLandmarkManager`;
- que fusión no borra MapPoints raw;
- cómo se calcula posición fused;
- qué política de descriptor fused se usa;
- qué eventos de score existen;
- cuándo se sube score;
- cuándo se baja score;
- cuándo no se baja score;
- cómo publica `GlobalMapBuilder` fused tracks;
- cuándo se crea `LoopOptimizationTask`;
- cuándo se inserta covisibilidad confirmada y con qué pose relativa;
- que tras optimización por loop solo se fusiona si 1L acepta;
- que no se usa GT para loops.

No marcar como `realizado` si no se cumplen los criterios de éxito.

---

## Notas de diseño que deben respetarse en fases posteriores

1. Fusión de landmarks es una capa global encima de los MapPoints raw.
2. Los MapPoints originales de ORB-SLAM3 se conservan siempre en `RawMapDatabase`.
3. El score se modifica mediante eventos semánticos, no por sumas/restas dispersas.
4. Los puntos no asociados solo se penalizan si estaban dentro de un solape confirmado y razonablemente deberían haberse asociado.
5. Un loop de error alto se optimiza antes de fusionar.
6. Un loop geométricamente confirmado se registra en `CovisibilityDatabase`
   aunque la fusión quede diferida.
7. Tras optimización de loop, fusionar solo si la validación post-apply acepta.
8. `GlobalMapBuilder` debe evitar publicar duplicados raw de landmarks ya fusionados.
