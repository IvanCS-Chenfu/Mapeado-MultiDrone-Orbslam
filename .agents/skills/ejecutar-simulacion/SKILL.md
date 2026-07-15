---
name: ejecutar-simulacion
description: Ejecuta las pruebas de simulación definidas por una fase usando run_simulation.sh y reduce los logs generados.
---

Usar después de que el build haya devuelto `0`.

Precondiciones:
- Existen YAMLs de prueba en `codex/archivos_auxiliares/tray_prueba_X.yaml`.
- La fase indica cuántas pruebas ejecutar.
- La fase o `planificador_fase` indica qué patrones buscar en cada log.

Comando base por prueba:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba <X> \
  --launch "ros2 launch <paquete> <launch>.launch.py" \
  --post-scenario-wait-sec <segundos>
```

La herramienta debe usar:

```text
codex/archivos_auxiliares/tray_prueba_X.yaml
```

La salida debe ser:

```text
codex/archivos_auxiliares/prueba_X.log
```

Ese archivo es el log completo de la prueba.

Después de cada prueba, reducir el log:

```bash
./codex/herramientas/reduce_simulation_log.sh --prueba <X> --patterns "PATRON1|PATRON2|ERROR|FATAL"
```

La reduccion debe generar:

```text
codex/archivos_auxiliares/prueba_X.reduced.log
```

Reglas:
- No analizar aquí si la prueba pasó. Eso lo hace `analizador_simulacion_logs`.
- No generar informes extra salvo que el usuario lo pida.
- Si una simulación falla por timeout o por action fallida, conservar tanto el log completo como el reducido para análisis.
- Si el reducido no contiene marcadores suficientes, consultar el completo antes de repetir la simulacion.
