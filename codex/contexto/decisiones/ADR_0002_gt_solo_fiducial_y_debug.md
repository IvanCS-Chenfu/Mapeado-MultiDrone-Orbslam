# ADR 0002 — Uso de ground truth solo para fiducial simulado y debug

## Estado

Aceptada.

## Contexto

El proyecto se desarrolla inicialmente en simulación Gazebo. Gazebo puede proporcionar ground truth de la pose real de cada dron.

Ese ground truth es útil para depurar y para simular observaciones de fiduciales, pero el objetivo final del proyecto es funcionar sin depender de ground truth.

El sistema final debe construir mapa y estimar poses usando sensores, ORB-SLAM3, landmarks, loops, fiduciales y optimización, no usando directamente la pose real de Gazebo.

## Decisión

El ground truth solo puede usarse para:

1. simular la detección de fiduciales;
2. generar métricas de debug;
3. evaluar errores externamente;
4. verificar visualmente si el sistema se comporta bien;
5. pruebas controladas que no alimenten directamente el resultado final.

El ground truth no puede usarse para:

1. construir el mapa sparse global;
2. construir la nube densa;
3. calcular el score de fused landmarks;
4. decidir si dos landmarks deben fusionarse;
5. calcular loops;
6. corregir directamente la pose de los drones;
7. publicar la pose final;
8. decidir qué puntos mapear o no mapear;
9. colocar puntos densos en el mundo;
10. sustituir la estimación de ORB-SLAM3 o del servidor.

## Motivo

Si se usa ground truth para construir el mapa o corregir las poses, la simulación dejaría de representar el problema real.

El objetivo del proyecto es que el sistema pueda trasladarse a drones reales. En drones reales no existirá ground truth perfecto.

## Fiduciales simulados

En simulación, un fiducial puede representarse como una región del mundo con pose conocida.

Ejemplo conceptual:

```text
si el dron entra en una región concreta
  entonces se considera que ha observado un fiducial
```

Esto no significa que el ground truth pueda corregir toda la trayectoria.

Solo significa que se simula una medida absoluta asociada al fiducial.

## Debug permitido

Se permite usar ground truth para imprimir diagnósticos como:

```text
error estimado contra GT
distancia al fiducial
dron dentro/fuera de región fiducial
comparación visual en RViz
```

Siempre que esos datos no alimenten directamente la nube global ni la pose final.

## Reglas para Codex

Si Codex modifica código que use GT, debe comprobar para qué se usa.

Codex debe rechazar o señalar como peligroso cualquier cambio que use GT para:

- fusionar landmarks;
- publicar mapa final;
- corregir poses globales;
- calcular nube densa;
- decidir loops.

Si se añade un nuevo uso de GT, debe estar documentado explícitamente como `debug` o `fiducial simulado`.

## Ejemplos correctos

Correcto:

```text
Usar GT para saber si el dron está dentro del radio de un fiducial simulado.
```

Correcto:

```text
Publicar un log de error contra GT para evaluar la simulación.
```

Correcto:

```text
Guardar una métrica de comparación, sin alimentar el optimizador.
```

## Ejemplos incorrectos

Incorrecto:

```text
Usar la pose GT del dron para publicar pose_global_corrected.
```

Incorrecto:

```text
Usar GT para colocar MapPoints en world.
```

Incorrecto:

```text
Usar GT para decidir que dos nubes sparse coinciden.
```

Incorrecto:

```text
Usar GT para construir la nube densa.
```

## Cuándo revisar esta decisión

Solo debe revisarse si el proyecto cambia de objetivo y pasa a ser únicamente una simulación con acceso garantizado a ground truth, lo cual no es el caso actual.
