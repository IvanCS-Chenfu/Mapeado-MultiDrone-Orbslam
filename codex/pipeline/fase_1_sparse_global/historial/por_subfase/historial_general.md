# Historial general

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## Formato obligatorio de entrada

Cada ejecucion debe anadirse al final con este formato minimo:

- titulo: `YYYY-MM-DD HH:MM — Subfase <ID>`;
- objetivo intentado;
- cambios realizados;
- comando de build;
- resultado de build: `OK` o `ERROR`;
- pruebas Gazebo ejecutadas y estado de cada una: `OK`, `ERROR` o `NO EJECUTADA`;
- patrones de reduccion usados;
- evidencia positiva;
- evidencia negativa o ausente;
- conclusion: `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`;
- siguiente paso recomendado.

Si la conclusion no es `CONSEGUIDA`, la subfase no debe marcarse como `realizado` en el pipeline.

## 2026-07-02 — Actualización documental de planificación activa 1A-1X

- fase y subfase: Fase 1, actualización documental general. No es ejecución de
  una subfase funcional.
- objetivo intentado: dejar la documentación general en un estado en el que un
  chat nuevo de Codex entienda que la planificación activa de Fase 1 es
  `subfase_1A.md` a `subfase_1X.md`, y que `12R-*`, `13`, `14` y `15` son
  legacy/solo referencia.
- archivos modificados:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/02_REGLAS_TECNICAS.md`;
  - `codex/contexto/03_ARQUITECTURA_ACTUAL.md`;
  - `codex/contexto/04_TOPICS_SERVICES_ACTIONS.md`;
  - `codex/contexto/05_MAPA_PAQUETES.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/decisiones/ADR_0004_pipeline_total_proyecto.md`;
  - `codex/contexto/decisiones/ADR_0006_fiduciales_no_son_loops.md`;
  - `codex/pipeline/PIPELINE_MAESTRO.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/LEGACY_SUBFASES.md`;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
  - `codex/contexto/paquetes/orbslam3_server/orbslam3_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/global_map_server.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/paquetes/simulacion_dron/simulacion_dron.md`;
  - `codex/contexto/paquetes/dron_individual/trayectorias.md`;
  - `codex/contexto/paquetes/paquetes_legacy.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- por qué se modificaron:
  - documentos generales seguían apuntando a `12R-D4` como subfase actual;
  - `pipeline_fase_1.md` tenía parte de la arquitectura nueva, pero todavía
    listaba una planificación antigua 1A-1N y decía que los archivos se
    crearían después;
  - era necesario registrar `1A`-`1X` como planificación activa y explicar la
    separación `RawMapDatabase` / `GlobalPoseStore` / fused tracks / scoring;
  - tras aclaración del usuario, los documentos en `codex/contexto/paquetes/`
    se corrigieron para describir el código actual, no la arquitectura futura
    como si ya estuviera implementada.
- paquetes compilados: ninguno.
- resultado de build: `NO EJECUTADO`, porque solo se modificó documentación.
- pruebas Gazebo ejecutadas: ninguna.
- patrones usados para reducir logs: no aplica.
- evidencia positiva encontrada:
  - `PIPELINE_MAESTRO.md` apunta a la Fase 1 activa 1A-1X;
  - `pipeline_fase_1.md` lista `1A`-`1X` en orden con objetivo, entrada,
    salida y validación principal;
  - `LEGACY_SUBFASES.md` marca subfases residuales sin editar sus archivos;
  - `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md` indica que las
    clases nuevas como `RawMapDatabase` y `GlobalPoseStore` aún no existen en
    el código actual;
  - `codex/contexto/paquetes/orbslam3_server/*.md` documentan el servidor
    monolítico actual y separan claramente el plan futuro.
- evidencia negativa o ausente:
  - no se ejecutó build ni simulación;
  - no se valida ninguna subfase funcional;
  - no se marca ninguna subfase como `realizado`.
- dudas o inconsistencias encontradas:
  - la documentación de paquetes debe seguir describiendo código real vigente;
    el pipeline describe lo que se va a cambiar.
- conclusión: `CONSEGUIDA` para el objetivo documental de esta sesión.
- siguiente paso recomendado: ejecutar `subfase_1A.md` para capturar baseline
  real del servidor actual antes de iniciar la migración de `1B`.

## 2026-07-03 — Cierre de sesión y handoff para nuevo chat

- fase y subfase: Fase 1, cierre documental de sesión. No es ejecución de una
  subfase funcional.
- tema tratado: confirmar si los MDs actuales guían a un nuevo chat de Codex a
  ejecutar una subfase de Fase 1 de extremo a extremo.
- decisiones tomadas:
  - para Fase 1, la documentación actual debe llevar a Codex a leer contexto,
    leer la subfase, planificar, modificar solo lo permitido, compilar,
    corregir build si falla, simular si toca, reducir logs, analizar evidencia,
    concluir y actualizar historial/documentación;
  - `1A` sigue siendo el punto de entrada activo y es baseline, por lo que no
    debe modificar código salvo que `subfase_1A.md` lo pida;
  - los pipelines específicos de fases 2 a 6 quedan reservados para el usuario
    y no deben tocarse ni ejecutarse salvo petición explícita;
  - `codex/contexto/paquetes/` describe el código actual, mientras el pipeline
    describe la planificación futura.
- cambios documentales realizados:
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- código modificado: ninguno.
- paquetes compilados: ninguno.
- resultado de build: `NO EJECUTADO`, porque no hubo cambios de código.
- pruebas Gazebo ejecutadas: ninguna.
- patrones usados para reducir logs: no aplica.
- evidencia positiva encontrada:
  - el handoff deja explícito el prompt recomendado para el siguiente chat;
  - queda claro que no hubo avance técnico ni validación funcional;
  - queda claro que Fase 1 continúa por `subfase_1A.md`.
- evidencia negativa o ausente:
  - no se ejecutó build;
  - no se ejecutó simulación;
  - no se validó ninguna subfase real.
- conclusión: `CONSEGUIDA` para el cierre documental de sesión.
- próximo prompt recomendado:

  ```text
  Haz la subfase 1A de la Fase 1 siguiendo AGENTS.md y el pipeline.
  Compila, simula si la subfase lo pide, analiza logs y actualiza documentación e historial.
  No me pidas un plan previo salvo bloqueo crítico.
  ```

## 2026-07-08 12:23 — Limpieza posterior a 1C — `orbslam3_multi` sin duplicados activos

- objetivo intentado: eliminar de `orbslam3_multi/src/` y `orbslam3_multi/include/` los módulos antiguos que ya estaban congelados en `orbslam3_multi/legacy/`.
- cambios realizados:
  - `orbslam3_multi/CMakeLists.txt`: la librería activa queda reducida a `src/raw_map_database.cpp`;
  - eliminados de la ruta activa los archivos de `GlobalAtlas`, `GlobalKeyFrameDatabase`, `GlobalORBMatcher`, `GlobalGeometryVerifier`, `GlobalPoseGraphOptimizer`, `MultiDroneSystem`, `ImportedKeyFrame`, `ImportedMapPoint` y `GlobalPoseGraphTypes`;
  - `orbslam3_multi/legacy/*_antiguo.*`: includes internos actualizados para apuntar a otros `_antiguo`;
  - `orbslam3_server/src/global_map_server_antiguo.cpp` y `orbslam3_server/include/orbslam3_server/global_map_server.hpp`: referencias legacy ajustadas hacia `orbslam3_multi/legacy`;
  - documentación de contexto y paquete actualizada.
- comandos de build:
  ```bash
  ./codex/herramientas/build_selected_packages.sh orbslam3_multi
  ./codex/herramientas/build_selected_packages.sh orbslam3_server
  ```
- resultado de build: `OK`.
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`;
  - se limpió solo `build/install/orbslam3_multi` para eliminar headers antiguos instalados.
- pruebas Gazebo ejecutadas: `NO EJECUTADA`.
  - no se cambió comportamiento de simulación ni lógica runtime de 1C;
  - la validación necesaria era build de los paquetes afectados.
- patrones de reducción usados: no aplica; no hubo simulación ni fallo de build.
- evidencia positiva:
  - `orbslam3_multi/include/orbslam3_multi/` contiene solo `raw_map_database.hpp` y `raw_map_types.hpp`;
  - `orbslam3_multi/src/` contiene solo `raw_map_database.cpp`;
  - `install/orbslam3_multi/include/orbslam3_multi/` contiene solo los headers raw;
  - `orbslam3_server` compila sin depender de los headers antiguos activos.
- evidencia negativa o ausente:
  - no se ejecutó replay ni Gazebo porque el cambio fue limpieza estructural de duplicados;
  - el disco sigue con margen bajo, por lo que conviene vigilar espacio antes de simulaciones.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: continuar con `1D` creando `GlobalPoseStore` sin reintroducir los módulos antiguos desde `legacy`.

## 2026-07-08 — Cierre documental del chat tras 1A-1C

- objetivo intentado: dejar el proyecto preparado para que otro chat de Codex continúe desde `1D` sin depender de la conversación actual.
- cambios realizados:
  - `codex/contexto/07_ULTIMA_SESION.md`: reescrito como resumen completo de cierre;
  - se comprobó que `codex/contexto/01_ESTADO_ACTUAL.md` mantiene `1D` como subfase actual;
  - se comprobó que `pipeline_fase_1.md` marca `1A`, `1B` y `1C` como `realizado`, y `1D` como `actual`;
  - se comprobó que la documentación de `orbslam3_multi` y `orbslam3_server` describe el estado posterior a `1C` y a la limpieza de duplicados activos.
- comando de build: no ejecutado en este cierre documental.
  - último build real vigente:
    ```bash
    ./codex/herramientas/build_selected_packages.sh orbslam3_multi
    ./codex/herramientas/build_selected_packages.sh orbslam3_server
    ```
- resultado de build vigente: `OK`.
  - `orbslam3_multi`: `BUILD-EXIT-CODE 0`;
  - `orbslam3_server`: `BUILD-EXIT-CODE 0`.
- pruebas Gazebo ejecutadas: `NO EJECUTADA` en este cierre documental.
  - última evidencia de simulación vigente:
    - `prueba_1`: `SIM-DONE prueba=1 success=true`;
    - `prueba_4`: `SIM-DONE prueba=4 success=true`;
    - `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`.
- patrones de reducción usados: no se regeneraron logs en este cierre documental.
  - patrones vigentes de `1C`:
    ```text
    SCENARIO-RUNNER|GOAL|RESULT|success|F1B-|F1C-RAWDB|F1C-REPLAY|ORBMAP|WRAPPER|ERROR|FATAL|Segmentation fault|Killed
    SCENARIO-RUNNER|success|F1C-REPLAY|F1C-RAWDB|ERROR|FATAL|Segmentation fault|Killed
    ```
- evidencia positiva:
  - `1A`, `1B` y `1C` tienen build, simulación/logs e historial;
  - `orbslam3_multi` activo contiene solo `RawMapDatabase`;
  - los módulos antiguos quedan solo en `orbslam3_multi/legacy/`;
  - la documentación de paquetes afectados está alineada con el estado actual;
  - `1D` queda como siguiente subfase.
- evidencia negativa o ausente:
  - no se ejecutó una simulación nueva durante el cierre documental;
  - el disco queda con margen bajo y debe revisarse antes de simulaciones largas.
- conclusión: `CONSEGUIDA`.
- siguiente paso recomendado: iniciar el próximo chat ejecutando `subfase_1D.md` para crear `GlobalPoseStore` usando `RawMapDatabase` y el dataset `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record`.

## 2026-07-09 — Corrección de planificación — `partial_candidate` pertenece a 1J

- motivo:
  - el usuario aclaro que la subfase actual debe ser `1J`, no `1L`;
  - la clasificacion `partial_candidate` nace en el dry-run, por lo que debe definirse primero en `subfase_1J.md`;
  - `1K` y `1L` deben tener en cuenta esa clasificacion, pero siguen `sin hacer`.
- decision corregida:
  - `subfase_1J.md` queda como `actual`;
  - `subfase_1K.md` queda `sin hacer`;
  - `subfase_1L.md` queda `sin hacer`;
  - `1J` debe distinguir `useful=true`, `partial_candidate=true` y rechazo;
  - `partial_candidate=true` se reserva para casos como `30-32 m -> 4.2 m`, con `reason=large_error_reduced_but_above_threshold`;
  - `1K` no debe tratar `partial_candidate` como exito normal;
  - `1L` validara/rollbackeara esos estados cuando se implemente.
- archivos modificados:
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1J.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1K.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- conclusion:
  - ajuste documental realizado;
  - no se modifico codigo;
  - no se ejecuto build ni simulacion.

## 2026-07-10 — Reapertura 1I por preservacion de anclajes fiduciales previos

- objetivo intentado:
  - documentar el fallo observado en RViz2 tras la prueba larga posterior a `1L`;
  - decidir si el arreglo debe empezar en `1I`, `1J`, `1K` o `1L`;
  - actualizar los estados de subfase para no avanzar a `1N` con una invariante rota.
- archivos modificados:
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1I.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1J.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1K.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1N.md`.
- cambios realizados:
  - `1I` pasa a `actual`;
  - `1J`, `1K`, `1L` y `1N` pasan a `sin hacer` operativamente hasta revalidar la ruta;
  - se conserva la evidencia historica de que `1J-1L` fueron implementadas y validadas antes;
  - se documenta la nueva invariante: una optimizacion por fiducial no puede corregir el fiducial objetivo rompiendo un fiducial previo del mismo submapa.
- evidencia del fallo:
  ```text
  [F1I-GRAPH-BUILD-SUMMARY] task_id=1 ... vertices=12 variable=12 fixed=0 hard_fiducial=0 ... propagation=66
  [F1J-OPT-SOLVER-SUMMARY] task_id=1 ... moved_kfs=12 fixed_kfs=0 max_delta_t=23.886778 mean_delta_t=15.814376
  [F1K-OPT-APPLY-SUMMARY] task_id=1 applied=true ... optimized_kfs=12 propagated_kfs=66 fixed_kfs=0
  [F1L-POST-APPLY-FIXED-CHECK] task_id=1 ok=true hard_fixed_moved=false fixed_moved_count=0
  [F1L-POST-APPLY-PARTIAL] task_id=1 ... real_after_t=2.292275 threshold_t=0.350000 decision=PARTIAL_KEEP_FOR_NEXT_PASS
  ```
- interpretacion:
  - `1L` no movio hard-fixed, pero en esa tarea no habia hard-fixed dentro del grafo;
  - por tanto, el `fixed check` no bastaba para proteger el fiducial anterior;
  - el arreglo debe empezar en `PoseGraphBuilder` para incluir/proteger anclajes fiduciales previos.
- build:
  - no ejecutado; sesion documental.
- simulacion:
  - no ejecutada; se uso `codex/archivos_auxiliares/logs/prueba_3.log` existente.
- criterios nuevos de exito para reabrir/cerrar:
  - en `1I`, ver `[F1I-GRAPH-ANCHOR-PRESERVATION]`, `[F1I-GRAPH-PREVIOUS-FIDUCIAL-ANCHOR]` y `fixed>=1 hard_fiducial>=1` cuando la tarea cruce entre fiduciales del mismo submapa;
  - en `1J`, confirmar que el solver no propone ni clasifica como aplicable una solucion que desplaza el fiducial previo;
  - en `1K`, bloquear apply multi-anclaje si no llega evidencia de preservacion;
  - en `1L`, ver `[F1L-ANCHOR-PRESERVATION-CHECK] ok=true` o ejecutar `REJECT_ROLLBACK`.
- conclusion:
  - `PARCIAL`: la infraestructura 1J-1L existe, pero la ruta queda reabierta desde `1I` por falta de preservacion explicita de anclajes fiduciales previos.
- siguiente paso recomendado:
  - ejecutar `subfase_1I.md` y modificar solo lo necesario en `PoseGraphBuilder`/estado auxiliar para incluir fiduciales previos como frontera fija o restriccion equivalente.

## 2026-07-10 — Ajuste de diseño 1I: topologia fiducial por ramas independientes

- objetivo intentado:
  - incorporar al plan de `1I` la idea de usar un grafo topologico auxiliar de fiduciales para elegir fronteras de optimizacion;
  - evitar una regla ingenua de "usar todos los fiduciales" o "usar el grado bruto del nodo";
  - documentar criterios conservadores para aristas largas no optimas o potencialmente subdivisibles.
- archivos modificados:
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1I.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1J.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1K.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- cambios realizados:
  - se define `FiducialConnectivityGraph` como capa auxiliar, no como grafo de optimizacion permanente;
  - los nodos son fiduciales y las aristas son tramos de submapa entre fiduciales con `drone_id`, `map_epoch`, KFs extremos y path/rango;
  - las fronteras se eligen por ramas independientes: una X puede requerir varias ventanas, una J normalmente solo la frontera cercana;
  - aristas paralelas de varios drones sobre el mismo corredor no multiplican automaticamente la fuerza de una rama;
  - aristas largas `A -> C` solo se marcan como subdivisibles por `B` si la trayectoria pasa cerca de `B`, si hay orden temporal/covisibilidad que lo demuestre o si aparece `B` entre los KFs extremos;
  - se recomiendan estados `DIRECT_OBSERVED`, `DIRECT_UNCERTAIN`, `SUBDIVISION_CANDIDATE`, `SUBDIVIDED_CONFIRMED` y `BYPASS_CONFIRMED`.
- marcadores nuevos propuestos:
  ```text
  [F1I-FID-CONNECTIVITY-EDGE]
  [F1I-FID-CONNECTIVITY-BRANCHES]
  [F1I-FID-CONNECTIVITY-SUBDIVISION]
  [F1J-OPT-BRANCH-ANCHORS]
  [F1J-OPT-ANCHOR-WEIGHTS]
  [F1K-OPT-APPLY-ANCHOR-PRESERVATION-PRECHECK]
  [F1L-ANCHOR-PRESERVATION-CHECK]
  [F1L-ANCHOR-PRESERVATION-FAIL]
  ```
- build:
  - no ejecutado; sesion documental.
- simulacion:
  - no ejecutada; sesion documental.
- conclusion:
  - `PARCIAL`: queda documentado el diseño, pendiente de implementacion y validacion en la ejecucion de `1I`.
- siguiente paso recomendado:
  - ejecutar `subfase_1I.md` implementando la seleccion de fiduciales frontera por ramas independientes y la clasificacion conservadora de aristas largas.

## 2026-07-14 — Reorganización documental 1J/1K/1L y subfases cortas

- objetivo intentado:
  - dejar claro que los MDs en `subfases/` son contratos ejecutables, no
    historiales largos;
  - reasignar la propiedad conceptual de la ruta de optimización fiducial.
- archivos modificados:
  - `AGENTS.md`;
  - `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`;
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/paquetes/orbslam3_multi/*.md` relevantes;
  - `codex/contexto/paquetes/orbslam3_server/*.md` relevantes;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1_RESUMEN.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1J.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1K.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1L.md`.
- cambios realizados:
  - `1J` queda definido como dry-run, dump/replay offline y HTML diagnóstico
    del grafo;
  - `1K` queda definido como apply en `GlobalPoseStore`, propagación,
    publicación y herencia de KFs futuros;
  - `1L` queda definido como diagnóstico post-apply con logs/RViz2/GT debug;
  - `subfase_1L.md` se reemplaza por un contrato corto y deja el detalle largo
    en `historial_1L.md`;
  - se documenta que `f1l_debug_animation_*` y `f1l_graph_dump_*` son nombres
    legacy, aunque su función pertenece a `1J`.
- paquetes compilados:
  - no aplica; solo documentación.
- pruebas Gazebo/replay:
  - no ejecutadas; solo documentación.
- evidencia positiva:
  - `subfase_1L.md` baja de más de mil líneas a un contrato corto;
  - `AGENTS.md` y `08_POLITICA_TOKENS_DOCUMENTACION.md` fijan la regla para
    futuros chats.
- evidencia negativa o ausente:
  - no se renombraron parámetros/logs legacy `f1l_*` en código.
- conclusión:
  - `PARCIAL` documentalmente: la regla queda fijada, pero queda deuda opcional
    de renombrado de parámetros/logs si se quiere eliminar la compatibilidad
    legacy.
- siguiente paso recomendado:
  - futuros cambios de HTML/dry-run deben ir a `1J`;
  - futuros cambios de apply/publicación/herencia deben ir a `1K`;
  - `1L` debe limitarse a diagnóstico y documentación de evidencia.

## 2026-07-14 — Reindexado 1M-1X y 1L parcial

- objetivo intentado:
  - marcar `1L` como `PARCIAL`, pendiente de revalidación futura;
  - reservar una nueva subfase `1M` entre `1L` y la antigua `1M`;
  - desplazar las antiguas subfases `1M-1W` a `1N-1X`.
- archivos modificados:
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1M.md`;
  - `codex/pipeline/fase_1_sparse_global/subfases/subfase_1N.md` hasta
    `subfase_1X.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1_RESUMEN.md`;
  - puntos de entrada de contexto e índices de historial.
- paquetes compilados:
  - no aplica; cambio documental y de nombres de archivos.
- pruebas Gazebo/replay:
  - no aplica.
- conclusión:
  - `1M` queda como subfase actual y después se define como
    `CovisibilityDatabase`;
  - `1N+` no debe iniciarse hasta completar `1M`;
  - la antigua `1M` pasa a ser `1N`, y el cierre de Fase 1 pasa de `1W` a
    `1X`.
- siguiente paso recomendado:
  - implementar `CovisibilityDatabase` antes de ejecutar `1N`.
