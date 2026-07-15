# Subfase 1L — Diagnóstico post-apply de optimización fiducial

## Estado

```text
parcial
```

`1L` no es dueña del solver ni del apply. Tampoco debe corregir poses,
MapPoints, propagación ni rollback como ruta normal: solo observa lo que ya
hizo `1K`, deja logs/sublogs diagnósticos y permite decidir documentalmente si
la evidencia basta para avanzar.

Propiedad funcional vigente:

```text
1J = dry-run + dump/replay offline + HTML diagnóstico del grafo
1K = apply del grafo en GlobalPoseStore + propagación + publicación
1L = diagnóstico post-apply con logs/sublogs/RViz2 opcional/GT debug
```

El detalle histórico de intentos, HTMLs concretos, variantes descartadas y
evidencia larga vive en:

```text
codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md
```

No volver a convertir este archivo en un diario de pruebas.

---

## Objetivo técnico

Comprobar mediante observabilidad que una optimización aplicada por `1K` quedó
bien reflejada en el sistema:

1. el KeyFrame del fiducial objetivo queda en su pose esperada;
2. anchors y hard fiducials permanecen protegidos;
3. los KFs optimizados/propagados continúan de forma coherente;
4. los KFs posteriores al último KF optimizado heredan la corrección del extremo;
5. los MapPoints publicados se reconstruyen desde poses finales de KFs y no
   desde un fallback rígido de submapa;
6. no hay NaN, crashes, rollback roto ni rebase incorrecto a anchors antiguos;
7. el GT debug, el HTML y RViz2 se usan solo como diagnóstico externo.

`1L` decide documentalmente si hay evidencia suficiente para pasar a `1N`.
No intenta demostrar que la geometría local ya sea perfecta; deformaciones por
falta de una base global de covisibilidad/loops quedan fuera de `1L`.
Si hay que cambiar solver, HTML o apply, la modificación pertenece a:

- `1J` si afecta a dry-run, dump, replay offline o HTML del grafo;
- `1K` si afecta a escritura en `GlobalPoseStore`, propagación, publicación o
  herencia de KFs futuros.

---

## Cambios permitidos

Permitidos en `1L`:

- añadir o ajustar logs diagnósticos post-apply;
- crear sublogs/reducciones;
- actualizar documentación de estado, paquetes e historial;
- ajustar YAMLs de prueba si solo cambia la observabilidad del escenario;
- añadir comprobaciones no mutantes sobre el estado ya aplicado por `1K`.
- registrar una conclusión `ACCEPT`, `REJECT` o `PARCIAL` como marcador
  diagnóstico, sin usarla para introducir cambios de estado nuevos.

Permitidos solo si se reclasifica explícitamente la tarea:

- cambios de solver/dry-run/HTML: mover a `1J`;
- cambios de apply, propagación, publicación o herencia futura: mover a `1K`;
- cambios de construcción de grafo/control points: mover a `1I`.

---

## Cambios prohibidos

En `1L` no se debe:

- modificar `RawMapDatabase`;
- modificar `GlobalPoseStore`, salvo que la tarea se reclasifique como `1K`;
- usar GT para tomar decisiones runtime;
- cambiar el solver del grafo;
- cambiar la política de apply en `GlobalPoseStore`;
- generar el HTML como si fuera propiedad de `1L`;
- mover KeyFrames, corregir MapPoints, fusionar landmarks o hacer loops;
- convertir `F1L-POST-APPLY-REJECT` en rollback normal dentro de `1L`;
- convertir un fallo visual de RViz2 en una heurística nueva dentro de `1L`;
- intentar resolver dobles paredes o deformación local causada por falta de
  covisibilidad; eso corresponde a la base de covisibilidad/loops de subfases
  posteriores;
- marcar la subfase como conseguida sin evidencia de build, simulación/logs y
  documentación.

---

## Archivos a revisar

Leer primero:

```text
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/pipeline/fase_1_sparse_global/historial/INDEX.md
codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_1L.md
codex/contexto/paquetes/orbslam3_multi/global_pose_store.md
codex/contexto/paquetes/orbslam3_multi/global_map_builder.md
codex/contexto/paquetes/orbslam3_multi/optimization_manager.md
codex/contexto/paquetes/orbslam3_server/global_map_server.md
```

Solo abrir código si falta comprobar una contradicción entre logs y docs.

---

## Paquetes a compilar

Si solo se modifica documentación, no compilar.

Si se toca logging del servidor:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_server
```

Si se toca una comprobación en `orbslam3_multi`:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
```

---

## Prueba Gazebo definida

Prueba principal:

```bash
./codex/herramientas/run_simulation.sh --prueba 1 --launch "ros2 launch simulacion_dron multi_dron.launch.py f1l_gt_kf_debug_enabled:=true f1l_debug_animation_enabled:=true f1l_graph_dump_enabled:=true f1l_debug_animation_output_dir:=/home/chenfu/Gazebo/src/codex/archivos_auxiliares/html f1l_graph_dump_output_dir:=/home/chenfu/Gazebo/src/codex/archivos_auxiliares/repeticiones" --post-scenario-wait-sec 60 --startup-wait-sec 20 --timeout-sec 1400 --max-gazebo-retries 1
```

La nomenclatura `f1l_debug_animation_*` queda como legacy de parámetros/logs.
Conceptualmente, la visualización HTML del grafo pertenece a `1J`; `1L` solo la
usa como evidencia cruzada cuando hace falta comparar logs/RViz2.

---

## Patrones de logs

Reducir con:

```text
SCENARIO-RUNNER|SIM-|F1H-|F1I-|F1J-|F1K-|F1L-|F1F-GLOBALMAP|GT-WINDOW|FATAL|ERROR|Segmentation fault|Killed|process has died
```

Sublogs recomendados:

```bash
./codex/herramientas/split_simulation_log.sh --prueba 1 --fase 1L
```

Marcadores mínimos a comprobar:

```text
F1L-POST-APPLY-ERROR
F1L-POST-APPLY-PROPAGATION-CHECK
F1L-POST-APPLY-GLOBALMAP-CHECK
F1L-GLOBALMAP-KF-PROJECTION
F1L-POST-APPLY-ACCEPT / F1L-POST-APPLY-REJECT
F1K-POSESTORE-LAST-CORRECTION-SET
F1K-POSESTORE-NEW-KF-INHERIT-CORRECTION
F1L-GT-WINDOW-STATS
```

---

## Criterios de éxito

`1L` queda conseguida si hay evidencia de:

- build correcto cuando se haya tocado código;
- simulación Gazebo terminada con `SIM-EXIT-CODE 0`;
- fiducial objetivo en umbral tras apply de `1K`;
- anchors/hard fiducials preservados;
- `propagated_count` coherente y `skipped_propagated_count=0` salvo razón
  documentada;
- KFs posteriores heredando `SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT`;
- `fallback_submap_after=0` en submapas corregidos;
- logs/sublogs que demuestren publicación coherente; RViz2 es confirmación
  visual recomendada cuando esté disponible, pero no debe forzar cambios en
  `1L` si los marcadores técnicos ya pasan;
- GT debug usado solo como métrica externa;
- documentación e historial actualizados.

`PARCIAL` si faltan marcadores técnicos, la simulación no es reproducible o se
detecta una contradicción entre logs y publicación.

`NO CONSEGUIDA` si el apply aceptado por `1K` no se refleja en publicación o
propagación, o si una inspección RViz2 disponible contradice los marcadores de
logs.

`BLOQUEADA` si falta una simulación reproducible o datos suficientes para
diagnosticar.

---

## Handoff actual

Última evidencia conocida:

```text
task_id=2
from_kf=200
fiducial objetivo: 26.641246 m -> 0.000000 m
propagated_count=98
fallback_submap_after=0
KFs 202..208 -> SERVER_OPTIMIZATION_PARTIAL_FUTURE_INHERIT
HTML: codex/archivos_auxiliares/html/f1l_debug_animation_task_2.html
Dump: codex/archivos_auxiliares/repeticiones/f1l_graph_task_2.tsv
```

Conclusión actual:

```text
PARCIAL hasta decisión final del usuario. La evidencia de logs ya permite
cerrarla si se acepta que la deformación local pendiente pertenece a
covisibilidad/loops, no a `1L`.
```
