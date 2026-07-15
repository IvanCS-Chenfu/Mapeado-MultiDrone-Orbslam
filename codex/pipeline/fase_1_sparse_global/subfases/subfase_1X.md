# Subfase 1X — Cierre documental, limpieza legacy e handoff de Fase 1

## Estado

```text
sin hacer
```

---

## Objetivo tecnico

Cerrar formalmente la Fase 1 dejando el repositorio, la documentación y el pipeline en un estado claro para que otro chat de Codex pueda continuar sin reconstruir el contexto.

Esta subfase no debe introducir lógica nueva. Debe:

- actualizar documentación global;
- marcar qué archivos son activos y cuáles legacy;
- actualizar estado real del proyecto;
- registrar resultados de pruebas;
- dejar comandos de build/simulación/replay;
- listar limitaciones conocidas;
- preparar la transición a Fase 2.

---

## Contexto obligatorio a leer

Leer:

- `AGENTS.md`
- `PIPELINE_MAESTRO.md`
- `pipeline_fase_1.md`
- subfases `1A` a `1W`
- historial de Fase 1
- documentación de paquetes
- ADRs
- logs reducidos de pruebas finales 1V/1W
- archivos legacy antiguos a partir de `12R-D4`, solo para indexarlos como legacy si procede.

---

## Diagnostico de partida

Se ha creado una fase 1 extensa. Sin cierre documental, el proyecto puede quedar confuso:

- archivos legacy junto a archivos nuevos;
- launch antiguo y launch nuevo;
- servidor antiguo y servidor nuevo;
- subfases antiguas y nuevas;
- documentación parcial por paquete;
- datasets replay no catalogados;
- comandos dispersos;
- limitaciones no anotadas.

La subfase 1X debe dejar un handoff claro.

---

## Archivos permitidos a modificar

- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_1_sparse_global/pipeline_fase_1.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/paquetes/**`
- `codex/contexto/ADRs/**`
- `codex/archivos_auxiliares/README.md`
- READMEs de paquetes si existen
- historial de fase/subfase
- índice de legacy

Cambios de código solo si son comentarios/documentación no funcional o corrección menor detectada.

---

## Archivos prohibidos

- lógica nueva;
- refactors;
- borrar legacy sin instrucción explícita;
- cambiar mensajes;
- cambiar comportamiento del servidor;
- modificar ORB_SLAM3.

---

## Funciones, clases o nodos que hay que localizar

Localizar y documentar:

- servidor activo;
- servidor legacy;
- launch activo;
- launch legacy;
- paquetes modificados;
- clases principales:
  - `RawMapDatabase`
  - `GlobalPoseStore`
  - `FiducialAnchorManager`
  - `GlobalMapBuilder`
  - `LandmarkScoreManager`
  - `PoseGraphBuilder`
  - `OptimizationManager`
  - `LoopDetector`
  - `SubcloudLoopVerifier`
  - `LoopDecisionManager`
  - `FusionManager`
  - `FusedLandmarkManager`
  - `PostOptimizationKeyFrameQueue`
- topics/services activos;
- datasets replay.

---

## Cambios requeridos

### 1. Actualizar `PIPELINE_MAESTRO.md`

Debe reflejar el orden final:

```text
1A ... 1X
```

Debe indicar que las subfases antiguas posteriores a `12R-D4` quedan legacy/obsoletas si procede.

Logs/documento:

```text
[F1W-PIPELINE-MAESTRO-UPDATED]
```

---

### 2. Actualizar `pipeline_fase_1.md`

Debe resumir:

- objetivo de fase 1;
- arquitectura final;
- subfases;
- criterios de cierre;
- estado de validación;
- limitaciones conocidas.

---

### 3. Actualizar `01_ESTADO_ACTUAL.md`

Debe decir claramente:

```text
qué funciona;
qué se validó;
qué está parcial;
qué está bloqueado;
qué queda para fase 2.
```

---

### 4. Actualizar documentación de paquetes

Para cada paquete tocado:

```text
archivos activos
archivos legacy
clases/nodos
topics/services
parámetros
logs principales
pruebas
limitaciones
```

Paquetes mínimos:

```text
orbslam3_multi
orbslam3_server
orbslam3_msgs si se tocó
simulacion_dron
```

---

### 5. Crear índice de archivos legacy

Crear o actualizar:

```text
codex/contexto/legacy/FASE_1_LEGACY_INDEX.md
```

Debe listar:

- archivos antiguos renombrados;
- archivos que solo se usan como referencia;
- launches antiguos;
- subfases antiguas ignoradas;
- qué no debe modificar Codex salvo orden explícita.

---

### 6. Catalogar datasets y pruebas

Crear o actualizar:

```text
codex/archivos_auxiliares/README.md
```

Debe listar:

- YAMLs de pruebas clave;
- datasets replay;
- comandos para ejecutar;
- qué valida cada prueba;
- logs esperados.

---

### 7. Registrar resultados finales

Crear resumen:

```text
codex/pipeline/fase_1_sparse_global/RESULTADO_FINAL_FASE_1.md
```

Con:

```text
build status
pruebas ejecutadas
logs reducidos
métricas finales
observación RViz2
subfases conseguidas/parciales/bloqueadas
limitaciones
```

---

### 8. Preparar handoff a Fase 2

Añadir sección:

```text
Pendiente para Fase 2
```

Posibles temas:

```text
fiducial real no GT
mejorar optimizador
mejorar descriptor fused
mejorar score policy
densificación si aplica
multi-session real
robustez con más drones
```

---

## Cambios prohibidos

- No marcar como realizado algo no validado.
- No borrar legacy.
- No ocultar fallos.
- No inventar resultados de pruebas.
- No modificar lógica funcional.
- No crear nuevas fases sin documentarlas.

---

## Paquetes a compilar

No debería requerir build si solo documentación. Si se toca código menor:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron
```

---

## Pruebas requeridas

### Prueba 1 — revisión documental

No requiere Gazebo. Debe comprobar:

- existen documentos esperados;
- links/rutas son correctos;
- estado actual coincide con resultados.

### Prueba 2 — smoke test opcional

Ejecutar prueba simple para confirmar que documentación no rompió nada si hubo cambios de código.

---

## Patrones de reduccion de logs

```text
F1W-|DOCS|LEGACY|FINAL|ERROR|FATAL
```

---

## Marcadores tecnicos obligatorios

Si se implementan logs/scripts:

```text
[F1W-PIPELINE-MAESTRO-UPDATED]
[F1W-PIPELINE-FASE1-UPDATED]
[F1W-ESTADO-ACTUAL-UPDATED]
[F1W-PACKAGE-DOCS-UPDATED]
[F1W-LEGACY-INDEXED]
[F1W-DATASETS-INDEXED]
[F1W-FINAL-STATUS]
```

---

## Criterio de exito

`CONSEGUIDA` si:

1. documentación global actualizada;
2. pipeline maestro refleja 1A–1X;
3. estado actual refleja estado real;
4. paquetes documentados;
5. legacy indexado;
6. datasets/pruebas catalogados;
7. resultado final creado;
8. no se inventan resultados;
9. Fase 2 tiene handoff claro.

---

## Criterio de fallo o parcial

`NO CONSEGUIDA` si:

- falta documentación crítica;
- se marca realizado sin evidencia;
- legacy queda ambiguo;
- no hay resultado final.

`PARCIAL` si:

- documentación principal está, pero falta algún paquete;
- no hay smoke test por tiempo;
- resultados de pruebas quedan incompletos pero honestos.

---

## Documentacion a actualizar

Esta subfase es principalmente documentación. Actualizar:

- `PIPELINE_MAESTRO.md`
- `pipeline_fase_1.md`
- `01_ESTADO_ACTUAL.md`
- docs de paquetes
- índice legacy
- README de archivos auxiliares
- resultado final de Fase 1
- historial 1X
