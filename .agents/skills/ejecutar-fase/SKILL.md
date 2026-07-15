---
name: ejecutar-fase
description: Workflow simplificado para ejecutar una fase/subfase del pipeline: planificar, implementar, compilar, simular, analizar logs y documentar.
---

Usa esta skill cuando el usuario pida ejecutar una fase o subfase del pipeline.

Objetivo:
- convertir una orden directa del usuario en un ciclo completo de planificacion, implementacion, build, simulacion, analisis y documentacion;
- no dejar la fase en un estado ambiguo;
- concluir con evidencia si la subfase queda `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`.

Workflow:

1. Leer contexto:
   - `AGENTS.md`
   - `codex/contexto/01_ESTADO_ACTUAL.md`
   - `codex/pipeline/PIPELINE_MAESTRO.md`
   - pipeline de la fase actual
   - subfase actual
   - historial reciente
   - decisiones en `codex/contexto/decisiones/`
   - documentacion de paquetes afectados en `codex/contexto/paquetes/`

2. Usar `planificador_fase`:
   - validar si la fase está bien planteada;
   - definir archivos y funciones a tocar;
   - definir YAMLs de prueba `codex/archivos_auxiliares/tray_prueba_X.yaml`;
   - definir paquetes a compilar;
   - definir patrones para reducir logs de simulación.
   - definir criterio de exito y fallo verificable.

   Si la subfase no define archivos, pruebas, patrones o criterio de exito, buscar esa informacion en el pipeline de fase e historial. Si sigue faltando informacion critica, preguntar al usuario antes de modificar codigo.

3. Usar `implementador_fase`:
   - aplicar cambios mínimos;
   - crear/modificar YAMLs de prueba;
   - ejecutar build con `codex/herramientas/build_selected_packages.sh`.

4. Si el build falla:
   - ejecutar `codex/herramientas/reduce_build_log.sh`;
   - usar `diagnosticador_build` para leer `codex/archivos_auxiliares/colcon_build.reduced.log`;
   - si falta contexto, consultar `codex/archivos_auxiliares/colcon_build.log` completo;
   - devolver el diagnóstico a `implementador_fase`;
   - repetir build.

5. Si el build pasa:
   - ejecutar `codex/herramientas/run_simulation.sh` una vez por cada prueba definida;
   - generar logs completos `codex/archivos_auxiliares/prueba_1.log`, `prueba_2.log`, etc.;
   - ejecutar `codex/herramientas/reduce_simulation_log.sh` para generar `prueba_1.reduced.log`, `prueba_2.reduced.log`, etc.

6. Usar `analizador_simulacion_logs`:
   - leer primero los logs reducidos `prueba_X.reduced.log`;
   - si faltan marcadores obligatorios, consultar el `prueba_X.log` completo antes de repetir Gazebo o atribuir el fallo al código;
   - comprobar `SCENARIO-RUNNER`, envio de goals, resultados `success=true` y cierre de simulacion cuando aplique;
   - comprobar los marcadores tecnicos exigidos por la subfase;
   - comprobar ausencia de errores graves;
   - decidir si las pruebas pasaron;
   - concluir si la fase quedó `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`.

7. Usar `curador_documentacion`:
   - actualizar docs de paquetes modificados;
   - actualizar `codex/contexto/paquetes/` para que describa los archivos, funciones, nodos, topics/actions y logs actuales tras la modificacion;
   - actualizar historial;
   - actualizar estado actual;
   - ajustar subfase actual o siguiente si procede.

8. Responder al usuario:
   - fase/subfase trabajada;
   - archivos modificados;
   - paquetes compilados;
   - resultado del build;
   - pruebas ejecutadas;
   - resumen de logs;
   - conclusion exacta;
   - documentacion actualizada;
   - siguiente paso recomendado.

Reglas:
- Solo `implementador_fase` modifica código.
- Solo `curador_documentacion` modifica documentación.
- No crear archivos auxiliares extra salvo logs completos `colcon_build.log`/`prueba_X.log`, logs reducidos `colcon_build.reduced.log`/`prueba_X.reduced.log` y YAMLs de prueba, salvo que el usuario lo pida.
- No ejecutar fases futuras antes de cerrar la fase actual.
- No declarar una subfase como conseguida si no se cumple el criterio de exito escrito en la subfase.
- Si el build compila pero los logs no contienen la evidencia esperada, la conclusion debe ser `NO CONSEGUIDA` o `PARCIAL`.
- Si una prueba no se ejecuta, el historial debe decirlo explicitamente.
- Si RViz2 era parte de la validacion visual pero no fue observado por el usuario, documentarlo como no observado.
- Si se modifica un paquete, no cerrar la ejecucion sin revisar la documentacion correspondiente en `codex/contexto/paquetes/`.
