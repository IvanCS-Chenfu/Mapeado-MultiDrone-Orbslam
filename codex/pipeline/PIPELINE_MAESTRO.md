# PIPELINE_MAESTRO — Proyecto multi-dron ORB-SLAM3 + ROS 2 + Gazebo

## Objetivo final

El objetivo final no es solo construir una nube sparse común. El objetivo final es construir una nube densa global de un entorno definido por YAML, usando varios drones, sin ground truth para estimar la pose final ni para colocar puntos en el mapa final.

La nube sparse global es la base necesaria para:

- anclar submapas;
- evitar dobles paredes;
- estimar pose global de cada dron;
- servir como soporte geométrico para la nube densa;
- validar coherencia multi-dron antes de reconstrucción densa.

## Estados posibles

| Estado | Significado |
|---|---|
| `realizado` | Implementado, compilado, probado y documentado. |
| `actual` | Trabajo activo o siguiente punto de entrada. |
| `sin hacer` | Todavía no debe implementarse. |
| `bloqueado` | No puede avanzar por dependencia pendiente. |
| `opcional` | Mejora futura no imprescindible. |
| `legacy` | Plan antiguo o referencia histórica; no es planificación activa. |

## Reglas globales

1. No usar ground truth para mapa sparse, nube densa, pose final, score ni loops.
2. El GT solo puede usarse para fiducial simulado, debug o métricas externas.
3. La unidad correcta es `submapa = (drone_id, map_epoch)`.
4. El mapa publicado debe ser fused, no unión bruta de `MapPoints`.
5. Fiduciales son observaciones absolutas; no son loops.
6. BoW solo genera candidatos; la geometría de subnube confirma o rechaza loops.
7. `orbslam3_server` debe tender a adaptador ROS 2.
8. `orbslam3_multi` debe tender a backend algorítmico.
9. Las poses locales ORB-SLAM3 no se sobrescriben: `RawMapDatabase` conserva raw y `GlobalPoseStore` conserva global.
10. Una subfase solo se marca `realizado` si hay build, pruebas, logs e historial.

## Ejecución automática por Codex

Cuando el usuario pida ejecutar una subfase, Codex debe usar el archivo `subfase_*.md` como fuente de verdad y seguir:

1. leer contexto y pipeline;
2. planificar;
3. modificar solo lo permitido;
4. compilar paquetes definidos;
5. reducir y diagnosticar build si falla;
6. ejecutar pruebas Gazebo/replay definidas;
7. reducir logs;
8. analizar logs completos si faltan marcadores;
9. actualizar documentación e historial;
10. concluir `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`.

## Fases principales

| Fase | Estado | Nombre | Conclusión breve |
|---|---|---|---|
| 1 | `actual` | Mapa sparse global multi-dron | Plan activo 1A-1X: migración controlada a arquitectura raw/global/fused/optimizable. |
| 2 | `sin hacer` | Pose global de cada dron sin ground truth | Depende de mapa sparse global estable. |
| 3 | `sin hacer` | GUI propia distinta de RViz2 | RViz2 queda como debug durante fases iniciales. |
| 4 | `sin hacer` | Separación servidor/dron/simulación | Pendiente de arquitectura funcional estable. |
| 5 | `sin hacer` | Nube densa global con OpenCV C++ | Depende de poses globales fiables y mapa sparse. |
| 6 | `opcional` | Mejoras avanzadas y robustez | Nuevos mundos, reflejos, objetos dinámicos, estrés y evaluación. |

Los archivos específicos de pipeline de las fases 2 a 6 pueden estar vacíos o
incompletos. Son espacio reservado para planificación futura del usuario. Codex
debe tratarlos como no ejecutables hasta que el usuario los complete o pida
explícitamente trabajar en ellos.

## Fase 1 activa

Documento específico:

```text
codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md
```

Subfases activas:

```text
codex/pipeline/fase_1_sparse_global/subfases/subfase_1A.md
...
codex/pipeline/fase_1_sparse_global/subfases/subfase_1X.md
```

La planificación antigua `12R-*`, `13`, `14` y `15` queda como legacy. No borrarla salvo fase explícita de limpieza validada.

## Objetivo de Fase 1

Construir un mapa sparse global multi-dron coherente, estable, en frame `world`, con submapas por `(drone_id, map_epoch)`, anchors fiduciales, loops geométricamente verificados, fused landmarks, score centralizado, optimización segura y publicación depurada.

La arquitectura objetivo separa:

```text
RawMapDatabase
  datos ORB-SLAM3 crudos

GlobalPoseStore
  poses globales, anchors, optimización, propagación y rebase

FusedLandmarkManager / LandmarkScoreManager
  tracks fused, score y publicación sin duplicados raw

GlobalMapBuilder
  nube sparse global publicable
```

## Criterio de fin de Fase 1

La Fase 1 termina cuando:

- varios drones construyen un mapa sparse global común;
- no hay dobles paredes persistentes en escenarios normales;
- los submapas por `(drone_id, map_epoch)` se mantienen correctamente;
- fiduciales iniciales anclan submapas;
- segundas visitas a fiducial crean optimización si hay error real;
- loops se validan con subnubes/matching/RANSAC;
- loops buenos fusionan landmarks;
- loops con error alto crean optimización por el pipeline común;
- las poses optimizadas se guardan en `GlobalPoseStore`;
- rollback restaura poses, correcciones, scores y fused tracks afectados;
- la nube publicada usa fused tracks y scores centralizados;
- `global_map_server.cpp` deja de ser el centro algorítmico;
- existe observabilidad suficiente en logs/RViz2 para Fase 2.

## Fases posteriores

### Fase 2 — Pose global sin GT

Usar el mapa sparse global y observaciones actuales para estimar pose de cada dron sin ground truth.

No ejecutar ni completar su pipeline específico mientras Fase 1 siga activa, salvo petición explícita del usuario.

### Fase 3 — GUI propia

Crear una GUI específica del proyecto para mapa, drones, trayectoria, fiduciales, loops, optimización, nube densa y estado de pruebas.

No ejecutar ni completar su pipeline específico mientras Fase 1 siga activa, salvo petición explícita del usuario.

### Fase 4 — Separación de paquetes

Separar paquetes en servidor, dron real y simulación.

No ejecutar ni completar su pipeline específico mientras Fase 1 siga activa, salvo petición explícita del usuario.

### Fase 5 — Nube densa global

Construir nube densa con poses globales, cámaras estéreo, calibración, mapa sparse y OpenCV C++.

No ejecutar ni completar su pipeline específico mientras Fase 1 siga activa, salvo petición explícita del usuario.

### Fase 6 — Mejoras opcionales

Añadir robustez, mundos, reflejos, dinámicos y evaluación avanzada.

No ejecutar ni completar su pipeline específico mientras Fase 1 siga activa, salvo petición explícita del usuario.
