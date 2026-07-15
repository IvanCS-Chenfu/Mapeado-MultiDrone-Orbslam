# ADR 0007 — Automatización con Codex: código, build, simulación, logs y documentación

## Estado

Aceptada.

## Contexto

El proyecto es grande y tiene muchas piezas conectadas:

- ROS 2;
- Gazebo;
- ORB-SLAM3;
- servidor global;
- múltiples drones;
- mensajes personalizados;
- optimización;
- logs complejos;
- pruebas visuales.

Modificar código sin compilar, simular y documentar puede crear regresiones difíciles de detectar.

Además, se quiere que Codex pueda ejecutar fases de forma semiautomática o automática.

## Decisión

Toda modificación relevante de código debe seguir este flujo:

1. leer contexto;
2. leer fase/subfase;
3. explorar código actual;
4. modificar código;
5. compilar paquetes afectados;
6. corregir errores de compilación;
7. ejecutar simulación definida por la fase si está permitido;
8. analizar logs;
9. incorporar observación RViz si existe;
10. actualizar documentación;
11. actualizar historial;
12. generar informe final.

## Motivo

Este flujo evita que Codex:

- programe sin contexto;
- repita errores antiguos;
- avance fases sin validar;
- olvide compilar;
- ignore logs;
- deje documentación obsoleta;
- modifique archivos no relacionados.

## Documentación que debe actualizarse

Si cambia código, revisar:

```text
contexto/07_ULTIMA_SESION.md
contexto/paquetes/<paquete>/<paquete>.md
contexto/paquetes/<paquete>/<script>.md
contexto/pipeline/<fase>/historial/
contexto/informes/
```

Si cambia una decisión arquitectónica, actualizar también:

```text
contexto/decisiones/
```

## Build

Cada fase debe indicar qué paquetes compilar.

Ejemplo para cambios en servidor:

```bash
cd ..
colcon build --packages-select orbslam3_msgs orbslam3_multi orbslam3_server --symlink-install
```

Si se modifican mensajes, compilar primero `orbslam3_msgs`.

## Simulación

Cada fase debe indicar:

- escenario;
- movimientos de drones;
- logs esperados;
- logs prohibidos;
- criterio visual en RViz;
- criterio para pasar a la siguiente fase.

La simulación puede ser automática, pero debe poder pausarse si el usuario necesita observar RViz2 manualmente.

## Logs

Los logs deben resumirse.

No basta con decir “la simulación corrió”.

El informe debe indicar si aparecieron los logs esperados, por ejemplo:

```text
LOCAL_LOOP_OPT_TASK
LOCAL-POSE-GRAPH-SOLVER
LOCAL-OPT-APPLY-SUMMARY
FUSED-SIMPLE-SUMMARY
```

## Reglas para Codex

Codex debe terminar cada tarea con:

1. archivos modificados;
2. explicación de cambios;
3. comandos ejecutados;
4. resultado de build;
5. resultado de simulación;
6. resumen de logs;
7. estado de RViz si se observó;
8. conclusión;
9. siguiente paso.

Codex no debe marcar una fase como `realizado` si no hay evidencia suficiente.

Si no se ejecutó simulación, debe decirlo explícitamente.

Si no se observó RViz, debe decirlo explícitamente.

## Cuándo revisar esta decisión

Debe revisarse si se cambia el sistema de automatización, si se usan runners externos o si se separan físicamente los paquetes entre servidor/drones/simulación.
