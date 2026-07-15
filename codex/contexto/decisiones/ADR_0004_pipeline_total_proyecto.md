# ADR 0004 — Pipeline total del proyecto

## Estado

Aceptada.

## Contexto

El proyecto no termina al conseguir un mapa sparse global.

El objetivo final es obtener una nube densa global de un entorno definido por YAML usando varios drones, sin depender de ground truth para la pose final ni para la construcción del mapa.

La nube sparse global es una fase necesaria, pero no es el objetivo final.

## Decisión

El pipeline maestro del proyecto se divide en seis fases principales:

1. Conseguir un mapa sparse global multi-dron.
2. Obtener la pose exacta de cada dron sin ground truth usando la nube global.
3. Crear una GUI propia distinta de RViz2.
4. Separar paquetes entre servidor, drones reales y simulación.
5. Construir una nube densa global usando poses, mapa sparse y OpenCV C++.
6. Añadir mejoras opcionales y pruebas avanzadas.

## Fase 1 — Mapa sparse global multi-dron

Objetivo:

Construir una nube sparse global coherente usando varios drones y la información de ORB-SLAM3.

Debe evitar:

- dobles paredes;
- fusión incorrecta de submapas;
- pérdida de submapas históricos;
- uso indebido de ground truth;
- correcciones destructivas.

Resultado esperado:

- mapa sparse fused en frame `world`;
- submapas por `(drone_id, map_epoch)`;
- landmarks globales con score;
- loops validados;
- fiduciales tratados como observaciones absolutas;
- poses corregidas suficientemente fiables para avanzar a la Fase 2.

## Fase 2 — Pose exacta de cada dron sin ground truth

Objetivo:

Usar la nube sparse global y las observaciones actuales de cada dron para estimar su pose global sin ground truth.

Esto puede incluir:

- relocalización contra el mapa global;
- matching de features contra landmarks globales;
- PnP/RANSAC;
- refinamiento local;
- fusión con odometría local;
- validación contra la nube global.

No se permite usar GT como entrada del algoritmo.

## Fase 3 — GUI propia distinta de RViz2

Objetivo:

Crear una interfaz gráfica específica del proyecto para visualizar y controlar el sistema.

La GUI debería mostrar:

- puntos sparse nuevos;
- fused landmarks;
- poses de drones;
- trayectoria de cada dron;
- estado de submapas;
- estado de fiduciales;
- nube densa cuando exista;
- métricas de calidad;
- logs resumidos;
- estado de fase/prueba.

RViz2 puede seguir usándose para debug, pero no debe ser la GUI final del proyecto.

## Fase 4 — Separación de paquetes

Objetivo:

Separar los paquetes de `src/` en tres grupos lógicos:

1. paquetes del ordenador servidor;
2. paquetes que irían en cada dron real;
3. paquetes exclusivos de simulación.

Resultado esperado:

```text
servidor/
dron/
simulacion/
```

o una estructura equivalente.

Esta fase debe facilitar pasar de simulación a despliegue real.

## Fase 5 — Nube densa global

Objetivo:

Construir nubes densas usando:

- poses globales estimadas;
- cámaras de los drones;
- mapa sparse global;
- OpenCV C++;
- calibración estéreo;
- filtros geométricos;
- integración en frame `world`.

La nube densa debe colocar cada punto en su posición global correcta.

No se debe usar ground truth para colocar la nube densa.

## Fase 6 — Mejoras opcionales

Objetivo:

Mejorar realismo y robustez.

Ejemplos:

- usar nuevos mapas de Gazebo;
- probar reflejos;
- probar cambios de iluminación;
- detectar objetos dinámicos;
- evitar mapear puntos dinámicos;
- filtrar superficies problemáticas;
- añadir escenarios más complejos;
- comparar rendimiento con varios números de drones.

## Consecuencias

El archivo `PIPELINE_MAESTRO.md` debe reflejar estas seis fases.

Cada fase puede tener subfases.

La Fase 1 activa se planifica ahora como una secuencia `1A`-`1X`.

Las subfases antiguas `12R-*`, `13`, `14` y `15` se conservan como legacy/solo
referencia. No deben ejecutarse como planificación activa.

No debe confundirse el pipeline total con el pipeline interno de la Fase 1.

## Reglas para Codex

Codex debe consultar `PIPELINE_MAESTRO.md` antes de ejecutar una fase.

Codex no debe avanzar a fases posteriores si la fase actual no cumple sus criterios de éxito.

Codex debe documentar cada intento en el historial de fase correspondiente.

Codex no debe convertir una mejora opcional de Fase 6 en requisito de fases anteriores.

## Cuándo revisar esta decisión

Solo debe revisarse si cambia el objetivo final del proyecto.
