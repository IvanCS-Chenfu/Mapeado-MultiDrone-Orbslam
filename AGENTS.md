# AGENTS.md — Arranque compacto para Codex

Este archivo es el punto de entrada ligero del proyecto. No existe una copia
larga activa de `AGENTS.md`: si falta una regla, buscarla en los documentos
específicos de `codex/contexto/`, `codex/pipeline/` o `codex/contexto/decisiones/`.

<!-- CODEX_INDEX_START
codex_index:
  read_order:
    - codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
    - codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
    - codex/pipeline/PIPELINE_MAESTRO.md
    - codex/pipeline/fase_1_sparse_global/pipeline_fase_1_RESUMEN.md
  package_map: codex/contexto/05_MAPA_PAQUETES.md
  packages_dir: codex/contexto/paquetes/
  subfases_dir: codex/pipeline/fase_1_sparse_global/subfases/
  historial_dir: codex/pipeline/fase_1_sparse_global/historial/por_subfase/
  tools_dir: codex/herramientas/
  short_summary_name: 00_summary.md
  long_doc_tag: HISTORY: true
quick_commands:
  - "rg -n 'pattern' . --hidden --glob '!build'"
  - "./codex/herramientas/build_selected_packages.sh <paquetes>"
  - "./codex/herramientas/run_simulation.sh --prueba X --launch 'ros2 launch simulacion_dron multi_dron.launch.py'"
CODEX_INDEX_END -->

Regla general: leer primero los resúmenes e índices. Abrir documentos largos
solo cuando falte información concreta para implementar, compilar, simular o
documentar.

## Idioma

- Responder siempre en español.
- Documentar siempre en español.
- Mantener en inglés nombres de paquetes, archivos, clases, funciones,
  variables, topics, services, actions, parámetros ROS, logs y comandos.

## Alcance editable

- Modificar solo archivos dentro de `src/`.
- No modificar manualmente `build/`, `install/` ni `log/`.
- No tocar `ORB_SLAM3`, `orbslam3_ros2` ni `orbslam3_msgs` salvo necesidad
  explícita y justificada.
- No revertir cambios del usuario.

Permisos operativos preaprobados:

- ejecutar `./codex/herramientas/build_selected_packages.sh`;
- ejecutar `./codex/herramientas/run_simulation.sh`;
- limpiar artefactos generados mínimos dentro de `build/`, `install/` o `log`
  si bloquean build/simulación.

Si el sandbox exige escalado para esas herramientas, pedirlo directamente en la
llamada de herramienta.

## Lectura de contexto barata

Antes de actuar, leer en este orden:

```text
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_1_sparse_global/pipeline_fase_1_RESUMEN.md
```

Después, según la tarea:

- subfase actual: leer solo `subfase_<ID>.md`;
- historial: leer primero `historial/INDEX.md` y luego
  `historial/por_subfase/historial_<ID>.md`;
- paquetes: usar primero `find-context` o `codex/herramientas/find_context.py` para localizar `codex/contexto/paquetes/<paquete>/00_summary.md` y luego abrir solo los MDs necesarios;
- logs: usar primero `rg`, `tail`, conteos y sublogs específicos antes de abrir
  logs completos.

Abrir documentos largos como `01_ESTADO_ACTUAL.md` o historiales por subfase
solo si el resumen no basta.

## Estado técnico clave

- Objetivo global: nube densa global usando varios drones, sin ground truth para
  mapa final ni pose final.
- Fase activa: Fase 1 — mapa sparse global multi-dron.
- Subfase activa actual: consultar `01_ESTADO_ACTUAL_RESUMEN.md`.
- No iniciar fases posteriores mientras Fase 1 siga abierta salvo petición
  explícita.
- Las subfases `12R-*`, `13`, `14` y `15` son legacy.

Invariantes permanentes:

- `submapa = (drone_id, map_epoch)`;
- `RawMapDatabase` conserva datos ORB-SLAM3 crudos y no se modifica por
  optimización;
- `GlobalPoseStore` conserva anchors, poses globales, poses optimizadas,
  propagadas y rollback;
- los fiduciales son observaciones absolutas, no loops;
- el ground truth solo puede usarse para fiducial simulado, debug o métricas
  externas;
- la nube global publicada debe tender a fused tracks, no unión bruta;
- no aceptar optimizaciones que muevan hard fiducials o rompan invariantes.

## Flujo si el usuario pide ejecutar una subfase

Seguir el contrato de `codex/pipeline/fase_1_sparse_global/subfases/subfase_<ID>.md`.

1. Leer contexto mínimo, subfase, historial específico y docs de paquetes
   afectados.
2. Planificar archivos, paquetes, pruebas, patrones de log y criterios.
3. Modificar solo lo necesario.
4. Compilar paquetes seleccionados con builds pequeños si hay paquetes pesados.
5. Si falla build, ejecutar `reduce_build_log.sh`, diagnosticar primer error real
   y corregir.
6. Ejecutar pruebas Gazebo/replay definidas.
7. Reducir logs; crear sublogs específicos si el reducido sigue siendo grande.
8. Analizar evidencia contra el criterio de éxito.
9. Actualizar documentación compacta, docs de paquete e historial por subfase.
10. Responder con `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`.

No declarar `CONSEGUIDA` sin build, simulación/logs y documentación coherente.

## Documentación y tokens

Al modificar documentación:

- actualizar primero resúmenes e índices;
- no duplicar la misma narrativa en muchos archivos;
- poner evidencia larga en el historial de la subfase, no en todos los MDs;
- mantener `codex/pipeline/fase_1_sparse_global/subfases/subfase_<ID>.md`
  como contrato ejecutable corto: qué se debe hacer, límites, pruebas y
  criterios; no usarlo como diario de pruebas ni como historial técnico largo;
- mantener `01_ESTADO_ACTUAL_RESUMEN.md` corto;
- reemplazar `07_ULTIMA_SESION.md` en cada cierre; no añadir texto acumulado al
  final;
- guardar el detalle anterior solo en el historial por subfase;
- si un archivo supera mucho las 250 líneas, crear índice/resumen y mover detalle
  a archivo específico o de archivo;
- para historial nuevo, escribir en `historial/por_subfase/historial_<ID>.md` y
  actualizar `historial/INDEX.md`.

Reglas detalladas:

```text
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
```

## Herramientas principales

```bash
./codex/herramientas/build_selected_packages.sh <paquetes>
./codex/herramientas/reduce_build_log.sh
./codex/herramientas/run_simulation.sh --prueba X --launch "ros2 launch simulacion_dron multi_dron.launch.py"
./codex/herramientas/reduce_simulation_log.sh --prueba X --patterns "<patrones>"
./codex/herramientas/split_simulation_log.sh --prueba X --fase 1L
```

Usar `split_simulation_log.sh` antes de abrir logs completos.

## Documentación obligatoria tras código

Si se modifica código, launch, configuración o scripts de un paquete, actualizar
también la documentación correspondiente en:

```text
codex/contexto/paquetes/
```

La documentación de paquete debe describir el estado actual del código, no solo
añadir una nota histórica.

## Detalle bajo demanda

La documentación extensa queda distribuida como detalle. Usarla de forma
selectiva:

```text
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/fase_1_sparse_global/historial/por_subfase/
codex/contexto/paquetes/**/<componente>.md
```
