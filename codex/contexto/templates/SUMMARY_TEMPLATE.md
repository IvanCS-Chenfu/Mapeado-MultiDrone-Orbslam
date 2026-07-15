# SUMMARY.md — Plantilla corta para paquetes

Resumen: Una frase (1-2 líneas) explicando el propósito del paquete.

Interfaces:
- `publishes`: lista de topics/msgs (una línea por entry)
- `subscribes`: lista de topics/msgs
- `services/actions`: lista de services/actions

Scripts:
- `script_name` : Una línea explicando qué hace y cómo ejecutarlo.

APIs clave:
- `ClassName::method` : una línea de propósito (mencionar si es pública y punto de entrada).

Config/Launch relevantes:
- ruta/archivo.launch.py : breve descripción.

Historial:
- Ver: codex/pipeline/fase_1_sparse_global/historial/por_subfase/

Nota: no incluir changelog ni historial; esos van en `historial/`.
