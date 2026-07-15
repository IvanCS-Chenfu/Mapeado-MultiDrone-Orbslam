# find_context.py

Esta herramienta busca contexto relevante usando `CODEX_INDEX.yaml` y los resúmenes cortos `00_summary.md`.

Uso:

```bash
python3 codex/herramientas/find_context.py <query>
```

Opciones:

- `--index`: ruta al índice YAML.
- `--deep`: buscar en todos los MDs si no hay coincidencias cortas.
- `--json`: salida JSON.
