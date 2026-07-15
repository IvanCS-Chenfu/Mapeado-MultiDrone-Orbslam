# Historial infraestructura

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-12 — Reorganización documental para reducir consumo de tokens

- objetivo intentado: reducir el coste de arranque de nuevos chats de Codex sin
  perder contexto histórico ni técnico.
- archivos modificados/creados:
  - `AGENTS.md`;
  - `codex/contexto/00_BOOTSTRAP_MINIMO.md`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`;
  - `codex/contexto/09_LOGS_Y_SUBLOGS.md`;
  - `.agents/skills/actualizar-documentacion/SKILL.md`;
  - `codex/herramientas/USO_HERRAMIENTAS.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/por_subfase/*.md`.
- archivos eliminados por redundantes tras integrar su información:
  - `codex/contexto/AGENTS_DETALLADO.md`;
  - `codex/contexto/07_ULTIMA_SESION_DETALLADA_2026-07-12.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/archivo/historial_fase_1_completo_2026-07-12.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_formato.md`.
- resultado de build: `NO EJECUTADO`; solo documentación.
- pruebas Gazebo: `NO EJECUTADAS`; solo documentación.
- evidencia positiva:
  - `AGENTS.md` queda como arranque compacto;
  - el estado corto vive en `01_ESTADO_ACTUAL_RESUMEN.md`;
  - el historial operativo queda dividido por subfase/tema, evitando el archivo
    monolítico;
  - `07_ULTIMA_SESION.md` queda definido como archivo reemplazable, no
    acumulativo;
  - la skill `actualizar-documentacion` exige mantener resúmenes, índices y
    sublogs para evitar repetir bloques largos.
- evidencia negativa o ausente:
  - no se modificaron scripts para generar sublogs automáticamente; por ahora la
    guía queda documentada en `09_LOGS_Y_SUBLOGS.md`.
- conclusión: `CONSEGUIDA` para la reorganización documental.
- siguiente paso recomendado:
  - en futuros cambios, leer primero `00_BOOTSTRAP_MINIMO.md` y
    `01_ESTADO_ACTUAL_RESUMEN.md`;
  - añadir historial nuevo solo en `por_subfase/historial_<ID>.md` y actualizar
    `INDEX.md`;
  - crear sublogs si un reducido sigue siendo grande.

## 2026-07-12 — Tres mejoras de arranque compacto y sublogs

- objetivo intentado: reducir otro salto de tokens en chats nuevos y análisis de
  logs.
- archivos creados/modificados:
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1_RESUMEN.md`;
  - `codex/herramientas/split_simulation_log.sh`;
  - `AGENTS.md`;
  - `codex/contexto/00_BOOTSTRAP_MINIMO.md`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`;
  - `codex/contexto/09_LOGS_Y_SUBLOGS.md`;
  - `.agents/skills/actualizar-documentacion/SKILL.md`;
  - `codex/herramientas/USO_HERRAMIENTAS.md`;
  - `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`.
- resultado de build: `NO EJECUTADO`; documentación y herramienta shell.
- pruebas Gazebo: `NO EJECUTADAS`.
- validación realizada:
  - `bash -n` para `split_simulation_log.sh`;
  - ejecución de `split_simulation_log.sh` sobre `prueba_1.log` existente.
- conclusión: `CONSEGUIDA` para la mejora documental/herramienta.

## 2026-07-03 — Aclaración documental sobre fases futuras y herramientas

- fase y subfase: Fase 1, actualización documental transversal. No es
  ejecución de una subfase funcional.
- objetivo intentado: hacer que un chat nuevo de Codex entienda cómo continuar
  el proyecto sin tocar los pipelines específicos de fases 2 a 6, que el
  usuario quiere completar más adelante.
- archivos modificados:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/pipeline/PIPELINE_MAESTRO.md`;
  - `codex/herramientas/USO_HERRAMIENTAS.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- archivos no modificados por instrucción del usuario:
  - `codex/pipeline/fase_2_poses_drones_sin_gt/`;
  - `codex/pipeline/fase_3_gui/`;
  - `codex/pipeline/fase_4_separacion_paquetes/`;
  - `codex/pipeline/fase_5_nube_densa/`;
  - `codex/pipeline/fase_6_mejoras/`.
- cambios realizados:
  - `00_LEER_PRIMERO.md` indica que los pipelines de fases 2 a 6 pueden estar
    vacíos o incompletos y quedan reservados para planificación futura del
    usuario;
  - `PIPELINE_MAESTRO.md` indica que esos pipelines específicos no son
    ejecutables hasta que el usuario los complete o pida trabajar en ellos;
  - `USO_HERRAMIENTAS.md` cambia encabezados de ejemplo `12R-D4` por ejemplos
    genéricos de Fase 1 para evitar confusión con planificación activa.
- paquetes compilados: ninguno.
- resultado de build: `NO EJECUTADO`, porque solo se modificó documentación.
- pruebas Gazebo ejecutadas: ninguna.
- patrones usados para reducir logs: no aplica.
- evidencia positiva encontrada:
  - la documentación transversal ya explica que Fase 1 sigue activa en `1A`-`1X`;
  - los ejemplos de herramientas ya no presentan `12R-D4` como fase de ejemplo
    activa;
  - los pipelines 2-6 quedan protegidos documentalmente para que Codex no los
    modifique sin petición explícita.
- evidencia negativa o ausente:
  - no se ejecutó build;
  - no se ejecutó simulación;
  - no se valida ninguna subfase funcional.
- conclusión: `CONSEGUIDA` para el objetivo documental de esta sesión.
- siguiente paso recomendado: continuar por `subfase_1A.md` cuando se quiera
  ejecutar trabajo funcional de Fase 1.
