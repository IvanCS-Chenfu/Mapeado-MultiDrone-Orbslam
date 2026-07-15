
## Marcadores tecnicos obligatorios

Deben aparecer:

```text
[F1N-SUBCLOUD-VERIFY-START]
[F1N-SUBCLOUD-QUERY-BUILD]
[F1N-SUBCLOUD-CANDIDATE-SEED]
[F1N-SUBCLOUD-CANDIDATE-WINDOW]
[F1N-SUBCLOUD-CANDIDATE-BUILD]
[F1N-SUBCLOUD-MATCH]
[F1N-SUBCLOUD-CANDIDATE-REDUCE-STATS]
[F1N-SUBCLOUD-MATCH-REFINED]
[F1N-SUBCLOUD-RANSAC]
[F1N-SUBCLOUD-ERROR]
[F1N-SUBCLOUD-DECISION]
```

Si el par ya estaba confirmado en `CovisibilityDatabase`, puede sustituirse la
cadena costosa por:

```text
[F1O-SUBCLOUD-SKIP-CONFIRMED-COVIS]
```

No debe aparecer:

```text
FUSION_APPLY
FUSION_EVENT_APPLIED
LOOP_OPT_TASK_CREATED
OPT-APPLY
SERVER_LOOP_OPTIMIZATION_APPLY
```

salvo que sean logs legacy ajenos claramente documentados y no producidos por la ruta nueva.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. se ejecutan las pruebas requeridas o alternativas equivalentes justificadas;
3. `scenario_runner_node` se ejecuta en pruebas Gazebo que lo usen;
4. los goals requeridos terminan con `success=true` si la prueba tiene trayectoria;
5. `LoopDetector` proporciona candidatos BoW;
6. `SubcloudLoopVerifier` procesa al menos un candidato;
7. se construye query_subcloud desde puntos del KeyFrame nuevo;
8. se construye candidate_subcloud desde una ventana global alrededor del candidate_seed, no solo desde el candidate_seed;
9. se hace matching ORB inicial;
10. se reduce candidate_subcloud usando región robusta de matches o se justifica fallback;
11. se hace matching refinado;
12. se ejecuta RANSAC 3D-3D o se rechaza con razón clara si no hay datos suficientes;
13. se calcula error geométrico;
14. se devuelve decisión preliminar;
15. si el loop se confirma, el resultado conserva `relative_pose_measured` e
    inliers suficientes para que 1P inserte covisibilidad confirmada;
16. si el par ya está en `CovisibilityDatabase`, se salta RANSAC con log claro;
17. no se llama a fusión;
18. no se crea tarea de optimización por loop;
19. no se modifican poses;
20. no se usa GT para loops;
21. no aparecen errores graves no explicados;
22. la documentación queda actualizada;
23. el historial registra evidencia positiva y negativa.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- no se procesan candidatos BoW;
- la candidate_subcloud se construye solo con puntos del candidate_seed sin ventana;
- no se hace matching;
- no se hace RANSAC ni rechazo justificado;
- falta un marcador obligatorio;
- se llama a fusión u optimización;
- se modifica `GlobalPoseStore`;
- se usa GT para loops;
- se inserta covisibilidad confirmada en 1O en vez de dejarlo a 1P;
- aparece un error grave no explicado.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- funciona en replay pero no en live;
- se construyen subnubes pero no hay suficientes matches en las pruebas;
- RANSAC no encuentra inliers, pero el rechazo está bien instrumentado;
- la reducción espacial hace fallback por pocos matches;
- no se logra caso `FUSION_CANDIDATE` o `LOOP_OPTIMIZATION_CANDIDATE`, pero sí `REJECT/HOLD` con métricas claras;
- falta export debug pero los logs son suficientes.

La subfase debe marcarse como `BLOQUEADA` solo si:

- `RawMapDatabase` no guarda descriptores/observaciones suficientes;
- los wrappers no exportan datos necesarios;
- no existe dataset ni simulación con candidatos BoW;
- falta información crítica y no se puede resolver con cambios mínimos.

---

## Documentacion a actualizar

Actualizar obligatoriamente:

- historial de Fase 1 / subfase 1O;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- crear o actualizar:
  - `codex/contexto/paquetes/orbslam3_multi/subcloud_loop_verifier.md`;
  - `codex/contexto/paquetes/orbslam3_multi/loop_detector.md`;
  - `codex/contexto/paquetes/orbslam3_multi/raw_map_database.md`;
  - `codex/contexto/paquetes/orbslam3_multi/global_pose_store.md`;
- actualizar:
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`.

La documentación debe explicar:

- diferencia entre query_subcloud y candidate_subcloud;
- que candidate_seed_kf solo es semilla;
- cómo se crea la ventana candidate;
- cómo se filtran puntos;
- cómo se reduce candidate_subcloud usando matches ORB;
- cómo funciona RANSAC;
- qué métricas se calculan;
- qué decisiones preliminares existen;
- cómo se salta un par ya confirmado por `CovisibilityDatabase`;
- qué evidencia entrega 1O para que 1P inserte una arista confirmada;
- que 1O no fusiona ni optimiza;
- qué queda pendiente para la siguiente subfase.

No marcar como `realizado` si no se cumplen los criterios de éxito.

---

## Notas de diseño que deben respetarse en fases posteriores

1. BoW da candidatos; 1O confirma o rechaza geométricamente.
2. `FUSION_CANDIDATE` no implica fusionar en 1O; lo hará la siguiente fase.
3. `LOOP_OPTIMIZATION_CANDIDATE` no implica crear/apply de optimización en 1O; lo hará la siguiente fase.
4. La candidate_subcloud debe representar una zona global alrededor del candidato, no solo el candidate_seed.
5. La reducción por matches debe tener fallback para no descartar contexto por pocos matches.
6. No usar GT para loops.
