# Trayectorias en `dron_individual`

## Relación con `lib_tray`

`dron_individual` no calcula directamente todas las ecuaciones de trayectoria. Usa `lib_tray`:

- `GenTrayPol3`
- `GenTrayVelTrap`
- `GenTrayElipse`

`gen_tray.cpp` actúa como action server ROS 2 que envuelve estas clases.

## Action `TrayAction`

Entrada:

- `tipo_trayectoria`:
  - `0`: polinomio cúbico;
  - `1`: velocidad trapezoidal;
  - `2`: elipse.
- `target_pose`.
- tiempos o parámetros `tx`, `ty`, `tz`, `tyaw`.
- flags `absoluto_x/y/z/yaw`.

Feedback:

Por cada eje `x`, `y`, `z`, `yaw` se publica un `Float64MultiArray` de 5 valores:

```text
[posición, velocidad, aceleración, jerk, ratio]
```

Resultado:

- `success`;
- `t_total`.

## Uso actual en simulación

La GUI `gui_tray_multi.py` del paquete `simulacion_dron` envía goals a:

```text
/dron_X/AccionTrayectoria
```

Los nodos de control consumen el feedback y mueven físicamente el dron en Gazebo.

## Recomendación para automatizar pruebas

Crear scripts que llamen a la action `TrayAction` directamente, en lugar de depender de la GUI. El script debe parametrizar:

- namespace del dron;
- tipo de trayectoria;
- objetivo XYZ/Yaw;
- flags absoluto/relativo;
- tiempos/velocidades.

Esto permitirá a Codex ejecutar escenarios reproducibles para cada fase.

## Protocolo secuencial legacy usado en `12R-D4`

Esta sección describe un escenario histórico útil para reproducir deriva en el
servidor monolítico. No es la planificación activa de Fase 1; las pruebas
activas deben venir del archivo `subfase_1X.md` correspondiente.

Para observar deriva y loops no basta con cambiar una vez el YAML. Debe poder
repetirse el mismo escenario N veces y clasificar cada ejecucion con logs + RViz2.

En ese escenario legacy, mover cada dron solo cuando el otro haya terminado su movimiento.
Escenario base recomendado:

1. mover `dron_1` al fiducial 2 con yaw `90 deg`;
2. mover `dron_2` encima de `dron_1`, a `0.3 m` por encima;
3. mover `dron_1` `6 m` en `-x`;
4. mover `dron_2` encima de `dron_1`;
5. mover `dron_1` de nuevo al fiducial;
6. mover `dron_2` encima de `dron_1`.

La clasificacion no debe depender del nombre `tray_prueba_1.yaml` o
`tray_prueba_2.yaml`, sino de la evidencia:

- ejecucion limpia: RViz2 sin deriva clara y logs sin loops destructivos;
- ejecucion con deriva: RViz2 con deriva/ruido/paredes duplicadas y logs con
  loops, decisiones de subnube o tareas/contadores de optimizacion.

Tras cada prueba, Codex debe dar una conclusion preliminar de logs y esperar la
observacion RViz2 del usuario para cerrar la conclusion final.
