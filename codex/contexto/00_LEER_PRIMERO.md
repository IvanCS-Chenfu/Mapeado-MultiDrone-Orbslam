# 00 — Leer primero

Este archivo queda como índice corto. Para arrancar un chat nuevo con bajo coste
de tokens, leer primero:

```text
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/00_BOOTSTRAP_MINIMO.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
```

La documentación extensa sigue existiendo, pero no debe abrirse por defecto.

## Fuente de verdad operativa

| Necesidad | Leer |
|---|---|
| Estado corto | `01_ESTADO_ACTUAL_RESUMEN.md` |
| Contexto mínimo | `CONTEXTO_MINIMO_ACTUAL.md` |
| Estado completo | `01_ESTADO_ACTUAL.md` |
| Reglas técnicas | `02_REGLAS_TECNICAS.md` |
| Arquitectura | `03_ARQUITECTURA_ACTUAL.md` |
| Topics/services/actions | `04_TOPICS_SERVICES_ACTIONS.md` |
| Paquetes | `05_MAPA_PAQUETES.md` y docs del paquete afectado |
| Código | `06_MAPA_CODIGO.md` |
| Última sesión | `07_ULTIMA_SESION.md` |
| Política antitokens | `08_POLITICA_TOKENS_DOCUMENTACION.md` |
| Logs/sublogs | `09_LOGS_Y_SUBLOGS.md` |
| Historial | `codex/pipeline/fase_1_sparse_global/historial/INDEX.md` |

## Fase activa

```text
Fase 1 — Mapa sparse global multi-dron
Subfase actual — 1M
Estado — sin hacer: `CovisibilityDatabase`
1L — PARCIAL; se revalidará en el futuro.
No iniciar 1N+ hasta completar 1M.
```

## Reglas rápidas

- No empezar desde cero.
- No usar subfases legacy `12R-*`, `13`, `14` o `15` como plan activo.
- No modificar `ORB_SLAM3` salvo permiso explícito.
- No rediseñar `orbslam3_msgs` ni `orbslam3_ros2` salvo necesidad fuerte.
- No usar ground truth para mapa final, loops, fusión, score, pose final ni nube
  densa.
- El GT solo puede usarse para fiducial simulado, debug o métricas externas.
- Identificar submapas como `(drone_id, map_epoch)`.
- Leer resúmenes e índices antes de abrir MDs grandes.
- **Subfases grandes** (`1O`, `1P`, `1Q`, `1R`, `1S`) se dividen en 4 subarchivos: 
  leer primero `subfase_<ID>.md` (índice breve), luego abrir el subarchivo temático 
  necesario (`especificacion`, `implementacion`, `testing` o `criterios`).

## Historial

Para trabajar en una subfase, usar:

```text
codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_<ID>.md
```

Actualizar siempre `historial/INDEX.md` con un resumen corto si cambia el estado.

## Después de validar una subfase

Actualizar solo lo necesario:

- `01_ESTADO_ACTUAL_RESUMEN.md` si cambia el punto de entrada;
- `01_ESTADO_ACTUAL.md` si cambia el estado completo;
- docs del paquete afectado;
- historial por subfase;
- `07_ULTIMA_SESION.md`;
- `pipeline_fase_1.md` o subfase solo si cambia el plan.

`07_ULTIMA_SESION.md` se reemplaza en cada cierre. No acumular sesiones antiguas
ahí: para conservar detalle anterior está el historial por subfase.

No duplicar la misma evidencia larga en varios archivos.
