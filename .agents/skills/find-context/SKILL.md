---
name: find-context
description: Localiza rápidamente archivos de contexto relevantes usando `CODEX_INDEX.yaml` y resúmenes cortos de paquete antes de que el agente abra documentos largos.
---

Usar cuando el agente no sepa dónde buscar documentación o contexto específico.

Objetivo:
- encontrar primero `00_summary.md` en `codex/contexto/paquetes/`.
- si no hay coincidencias, buscar en los archivos prioritarios de `CODEX_INDEX.yaml`.
- evitar abrir MDs largos salvo necesidad real.

Flujo:
1. Ejecutar `python3 codex/herramientas/find_context.py <query>`.
2. Si el resultado lista archivos, abrir solo esos archivos y no otros.
3. Si no hay coincidencias y se necesita más detalle, ejecutar con `--deep`.
4. Si el usuario pide un paquete específico, abrir primero `codex/contexto/paquetes/<paquete>/00_summary.md`.

Reglas:
- esta skill solo busca contexto, no modifica código ni documentación.
- si el resultado incluye menos de 3 coincidencias, el agente puede abrir esos archivos directamente.
- si no hay coincidencias y se usa `--deep`, priorizar archivos con `HISTORY: true` como última opción.
- no usar la búsqueda profunda por defecto para no gastar tokens en MDs largos.
