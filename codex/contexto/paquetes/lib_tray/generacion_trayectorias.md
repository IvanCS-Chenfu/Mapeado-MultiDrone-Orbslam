# Generación de trayectorias en `lib_tray`

## Objetivo

Calcular trayectorias por eje para que `gen_tray` publique referencias suaves al controlador del dron.

Cada generador produce arrays de 5 valores por eje:

```text
[posición, velocidad, aceleración, jerk, ratio]
```

## `GenTrayPol3`

Archivo:

`src/generacion/gen_tray_pol3.cpp`

Rol:
- calcula trayectorias con polinomios cúbicos;
- usa condiciones iniciales/finales de posición y velocidad;
- permite tiempos finales por eje;
- permite objetivos absolutos o relativos según eje.

Uso típico:
- movimientos suaves de punto a punto con duración conocida.

## `GenTrayVelTrap`

Archivo:

`src/generacion/gen_tray_veltrap.cpp`

Rol:
- calcula trayectoria con perfil de velocidad trapezoidal;
- usa velocidad máxima y tiempo de aceleración;
- calcula fases de aceleración, velocidad constante y deceleración;
- exporta coeficientes para evaluación temporal.

Uso típico:
- movimientos donde se quiere limitar velocidad lineal/angular máxima.

## `GenTrayElipse`

Archivo:

`src/generacion/gen_tray_elipse.cpp`

Rol:
- genera trayectorias elípticas alrededor de un centro o punto inicial;
- útil para rodear zonas/edificios;
- calcula posición, velocidad y derivadas en función del tiempo.

Uso típico dentro del proyecto:
- mover drones alrededor de un edificio para mapearlo.

## Relación con Fase 1

La calidad de la trayectoria influye en:

- cantidad de KeyFrames generados;
- estabilidad de ORB-SLAM3;
- probabilidad de loops;
- cobertura del entorno.

## Reglas para modificar

- Mantener interfaz pública usada por `dron_individual`.
- Si se cambia formato de salida, actualizar `TrayAction` y control.
- Probar con los ejecutables de test y con una simulación corta.
