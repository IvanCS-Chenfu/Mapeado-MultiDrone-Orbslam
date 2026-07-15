# `.agents`

Esta carpeta contiene **skills** de Codex, no agentes personalizados.

Los agentes personalizados están en:

```text
.codex/agents/
```

Las skills sirven para recordar workflows repetibles, por ejemplo:

- ejecutar una fase;
- compilar paquetes seleccionados;
- ejecutar simulaciones;
- actualizar documentación.

En este proyecto las herramientas reales viven en:

```text
codex/herramientas/
```
Entre ellas, `find_context.py` es la herramienta de búsqueda de contexto rápido para el agente.
Los archivos auxiliares temporales viven en:

```text
codex/archivos_auxiliares/
```
