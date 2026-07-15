# GUI y gráficas en `simulacion_dron`

## `src/control_tray/gui_tray_multi.py`

GUI Tkinter para enviar goals `TrayAction` a varios drones.

Rol:
- seleccionar dron;
- configurar tipo de trayectoria;
- enviar objetivo a `/dron_X/AccionTrayectoria`.

Limitación:
- es interactiva; para pruebas automáticas conviene crear un script no interactivo que use la misma action.

## `src/graficar/graficar.py`

Nodo Python de visualización de arrays numéricos publicados en:

- `/numeric_array`
- `/labels_array`

## Ejecutables C++ de gráficas

- `graficar_GT.cpp`: publica valores GT para graficar.
- `graficar_tray.cpp`: publica referencias de trayectoria desde feedback de action.
- `graficar_GTvsTray.cpp`: publica errores entre GT y trayectoria.

## Estado

Útiles para debug de control, no para validar directamente el mapa global.

Para las fases de Codex, la validación principal debe venir de logs del servidor, nubes en RViz/GUI futura y métricas automáticas.
