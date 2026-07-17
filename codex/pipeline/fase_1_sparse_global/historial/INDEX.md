# Índice de historial — Fase 1

Leer este índice antes de abrir historiales largos. Abrir solo el archivo de la
subfase o tema que afecte a la tarea actual.

## Estado rápido

| Subfase/tema | Archivo | Estado útil |
|---|---|---|
| Legacy `12R-D4` | `por_subfase/historial_12R-D4.md` | Referencia histórica, no planificación activa. |
| General/planificación | `por_subfase/historial_general.md` | Cambios documentales generales y handoffs; incluye realineación 1J/1K/1L, regla de subfases cortas y reindexado 1M-1X. |
| Infraestructura | `por_subfase/historial_infraestructura.md` | Reglas/herramientas Codex; incluye reorganización antitokens 2026-07-12. |
| Pruebas típicas | `por_subfase/historial_pruebas_tipicas.md` | Trayectorias reutilizables. |
| `1A` | `por_subfase/historial_1A.md` | Baseline conseguida. |
| `1B` | `por_subfase/historial_1B.md` | Servidor mínimo conseguido. |
| `1C` | `por_subfase/historial_1C.md` | `RawMapDatabase` y replay conseguidos. |
| `1D` | `por_subfase/historial_1D.md` | `GlobalPoseStore` conseguido. |
| `1E` | `por_subfase/historial_1E.md` | `FiducialAnchorManager` conseguido. |
| `1F` | `por_subfase/historial_1F.md` | Nube sparse y `body_T_camera` conseguidos. |
| `1G` | `por_subfase/historial_1G.md` | Full snapshots/reconciliación conseguidos. |
| `1H` | `por_subfase/historial_1H.md` | Revisits fiduciales conseguidos. |
| `1I` | `por_subfase/historial_1I.md` | Grafo temporal revalidado con anclajes previos. |
| `1J` | `por_subfase/historial_1J.md` | Dry-run revalidado con preservación de anclajes. |
| `1K` | `por_subfase/historial_1K.md` (índice) | Apply seguro revalidado. Dividido en subhistoriales: `historial_1K_1.md` (2026-07-10), `historial_1K_2.md` (2026-07-11). |
| `1K/1L` | `por_subfase/historial_1K_1L.md` | Ajustes de planificación compartidos. |
| `1L` | `por_subfase/historial_1L.md` (índice) | PARCIAL; diagnóstico post-apply. Dividido en subhistoriales: `historial_1L_1.md` (2026-07-10 a 2026-07-12 00:54), `historial_1L_2.md` (2026-07-12 23:06 a 2026-07-13 00:14), `historial_1L_3.md` (2026-07-13 00:27 a 2026-07-13 13:18), `historial_1L_4.md` (2026-07-14). Propiedad vigente: HTML/dry-run en `1J`, apply/publicación/herencia en `1K`, validación/logs/RViz2/GT debug en `1L`. |
| `1M` | `por_subfase/historial_1M.md` | Actual/a probar en simulación; `CovisibilityDatabase` implementada para ORB-SLAM3 y loops geométricos. |
| `1N` | `por_subfase/historial_1N.md` | Actual/por implementar; infraestructura de despacho creada, falta BoW real. |

## Cómo añadir historial nuevo

1. Añadir la entrada al final de `por_subfase/historial_<ID>.md`.
2. Actualizar esta tabla si cambia estado o hay un aprendizaje clave.
3. No copiar la misma evidencia larga en `01_ESTADO_ACTUAL.md`,
   `07_ULTIMA_SESION.md` y docs de paquete.
4. No recrear un historial monolítico; el detalle vive en `por_subfase/`.

Formato mínimo de entrada:

```text
## YYYY-MM-DD HH:MM — Subfase <ID> — título corto

- objetivo intentado:
- archivos modificados:
- paquetes compilados:
- resultado de build:
- pruebas Gazebo/replay:
- patrones de reducción:
- evidencia positiva:
- evidencia negativa o ausente:
- conclusión:
- siguiente paso recomendado:
```
