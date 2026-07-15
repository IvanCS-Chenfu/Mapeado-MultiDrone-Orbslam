# 09 — Logs y sublogs para ahorrar tokens

Los logs completos se conservan, pero no deben ser la primera lectura.

## Flujo recomendado

1. Revisar resultado mecánico:

```bash
rg -n "SCENARIO-RUNNER-DONE|SIM-DONE|SIM-EXIT-CODE|ERROR|FATAL|Segmentation fault|Killed" codex/archivos_auxiliares/logs/prueba_X.log
```

2. Crear o revisar reducido estándar:

```bash
./codex/herramientas/reduce_simulation_log.sh --prueba X --patterns "<patrones>"
```

3. Si el reducido sigue grande, crear sublogs por tema.

Comando recomendado:

```bash
./codex/herramientas/split_simulation_log.sh --prueba X --fase 1L
```

## Sublogs sugeridos para Fase 1

```text
prueba_X.scenario.log       SCENARIO-RUNNER|SIM-
prueba_X.errors.log         [ERROR]|[FATAL]|Segmentation fault|Killed|process has died
prueba_X.F1H.log            F1H-
prueba_X.F1I.log            F1I-
prueba_X.F1J.log            F1J-
prueba_X.F1K.log            F1K-
prueba_X.F1L.log            F1L-
prueba_X.gt_window.log      F1L-GT-WINDOW-STATS|F1L-GT-COLLATERAL-CHECK
```

## Índice de prueba

La herramienta crea:

```text
codex/archivos_auxiliares/logs/prueba_X.index.md
```

Contenido recomendado:

```text
# Índice prueba X

- comando:
- escenario/YAML:
- resultado mecánico:
- goals enviados:
- goals success:
- errores graves, sin contar marcadores tipo `F1H-FID-POSE-ERROR`:
- marcadores clave y conteos:
- evidencia positiva:
- evidencia negativa:
- conclusión preliminar:
- log completo:
- sublogs:
```

## Reglas

- El log completo nunca se borra por crear sublogs.
- Un sublog no sustituye el análisis: solo reduce tokens.
- Si falta un marcador obligatorio en un sublog, buscar en el log completo antes
  de concluir que no existe.
- Los sublogs viven en `codex/archivos_auxiliares/logs/` y pueden regenerarse.
