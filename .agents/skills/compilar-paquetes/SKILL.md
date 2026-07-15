---
name: compilar-paquetes
description: Compila paquetes ROS 2 seleccionados por el agente usando build_selected_packages.sh y gestiona el log de build.
---

Usar cuando haya que compilar después de una modificación.

Comando base:

```bash
./codex/herramientas/build_selected_packages.sh <paquete_1> <paquete_2> ...
```

Reglas:

1. Los paquetes los decide `planificador_fase` o `implementador_fase`, no la herramienta.
2. El log completo se guarda siempre en:

```text
codex/archivos_auxiliares/colcon_build.log
```

3. Si el comando devuelve `0`, no reducir el log.
4. Si devuelve distinto de `0`, ejecutar:

```bash
./codex/herramientas/reduce_build_log.sh
```

5. Tras reducir, `diagnosticador_build` debe leer:

```text
codex/archivos_auxiliares/colcon_build.reduced.log
```

6. Si falta contexto en el reducido, `diagnosticador_build` debe consultar:

```text
codex/archivos_auxiliares/colcon_build.log
```

7. `implementador_fase` corrige solo lo que indique `diagnosticador_build`.
8. Repetir hasta compilar o hasta que haya un bloqueo claro.
