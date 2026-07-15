## Marcadores tecnicos obligatorios

Eventos/política:

```text
[F1R-SCORE-POLICY]
[F1R-SCORE-EVENT]
[F1R-SCORE-UPDATE]
```

Raw ORB-SLAM3:

```text
[F1R-SCORE-ORBSLAM-RAW]
[F1R-SCORE-ORBSLAM-QUALITY-UPDATE]
```

Fusión:

```text
[F1R-SCORE-FUSION-CONFIRMED]
[F1R-FUSED-SCORE-UPDATE]
```

Unmatched:

```text
[F1R-SCORE-UNMATCHED-PENALIZE]
```

y, si aplica:

```text
[F1R-SCORE-UNMATCHED-SKIP]
```

Journal/rollback:

```text
[F1R-SCORE-JOURNAL-BEGIN]
[F1R-SCORE-JOURNAL-COMMIT]
```

y para rollback:

```text
[F1R-SCORE-JOURNAL-ROLLBACK]
[F1R-SCORE-ROLLBACK-FROM-OPT]
```

Publicación:

```text
[F1R-GLOBALMAP-SCORE-FILTER]
[F1R-GLOBALMAP-FUSED-SCORE-PUBLISH]
```

Stats:

```text
[F1R-SCORE-STATS]
```

No debe aparecer:

```text
SCORE_DIRECT_MUTATION_OUTSIDE_MANAGER
UNMATCHED_PENALIZE_ALL
REJECTED_TASK_SCORE_COMMITTED
FUSED_RAW_DUPLICATE_PUBLISHED_MAIN
GT_USED_FOR_LOOP_SCORE
```

salvo como error explícito detectado y explicado.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. se ejecutan las pruebas requeridas o alternativas equivalentes justificadas;
3. `scenario_runner_node` se ejecuta en pruebas Gazebo que lo usen;
4. los goals requeridos terminan con `success=true` si la prueba tiene trayectoria;
5. existe una lista central de eventos de score;
6. las actualizaciones de score pasan por `LandmarkScoreManager`;
7. datos raw de ORB-SLAM3 crean/actualizan scores;
8. fusión confirmada sube score mediante eventos;
9. fused tracks tienen score propio;
10. unmatched se penaliza solo con condiciones justificadas;
11. existe journal o mecanismo equivalente de rollback de score;
12. rollback de tarea revierte scores y tracks afectados;
13. `GlobalMapBuilder` filtra/publica según score;
14. miembros raw de fused tracks no se publican duplicados en nube principal;
15. logs muestran stats de score;
16. no se usa GT para score de loops;
17. no aparecen errores graves no explicados;
18. la documentación queda actualizada;
19. el historial registra evidencia positiva y negativa.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- hay modificaciones directas de score fuera de `LandmarkScoreManager`;
- no se inicializan scores raw;
- fusión no modifica scores;
- se penalizan todos los unmatched indiscriminadamente;
- rollback deja scores contaminados;
- se publican duplicados raw/fused en nube principal;
- se usa GT para scoring de loops;
- falta un marcador obligatorio;
- aparece un error grave no explicado.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- score raw y fused funcionan, pero rollback de scores queda incompleto;
- rollback funciona, pero no se logra probar unmatched;
- funciona en replay pero no en live;
- la publicación por score funciona pero RViz2 muestra demasiados o pocos puntos y requiere ajuste de umbrales;
- el journal existe pero no exporta CSV/debug;
- no se dispone de alguna métrica ORB-SLAM3 y se usa fallback documentado.

La subfase debe marcarse como `BLOQUEADA` solo si:

- `LandmarkScoreManager` no existe;
- `RawMapDatabase` no conserva datos mínimos de MapPoints;
- `FusionManager` / `FusedLandmarkManager` no existen;
- no hay forma de generar eventos de fusión ni replay;
- falta información crítica y no se puede resolver con cambios mínimos.

---

## Documentacion a actualizar

Actualizar obligatoriamente:

- historial de Fase 1 / subfase 1S;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- crear o actualizar:
  - `codex/contexto/paquetes/orbslam3_multi/landmark_score_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/landmark_score_events.md`;
  - `codex/contexto/paquetes/orbslam3_multi/fused_landmark_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/fusion_manager.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_map_builder.md`;
  - `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
- actualizar:
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`.

La documentación debe explicar:

- evento oficial de score;
- política numérica inicial;
- score raw de ORB-SLAM3;
- score de fused tracks;
- cuándo sube score;
- cuándo baja score;
- cuándo no baja score;
- rollback de score;
- publicación por score;
- umbrales configurables;
- logs de validación;
- limitaciones conocidas.

No marcar como `realizado` si no se cumplen los criterios de éxito.

---

## Notas de diseño que deben respetarse en fases posteriores

1. El score es una capa centralizada en `LandmarkScoreManager`.
2. Las clases externas emiten eventos, no modifican valores.
3. Un loop/fusión rechazada no debe dejar scores altos ni tracks activos.
4. La publicación principal no debe duplicar miembros raw de tracks fused.
5. Los puntos no asociados solo se penalizan en solape confirmado.
6. El score no sustituye a la geometría; solo afecta confianza/publicación/prioridad.
7. En fases futuras se podrá mejorar la política numérica sin cambiar la arquitectura de eventos.
