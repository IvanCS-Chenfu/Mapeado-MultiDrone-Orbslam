---
name: actualizar-documentacion
description: Actualiza documentación de contexto, paquetes, historial y estado actual tras build y simulación, priorizando resúmenes e índices para ahorrar tokens.
---

Usar al final de una fase, tras una prueba fallida que deba quedar registrada o
cuando se reorganice documentación.

Agente recomendado:

- `curador_documentacion`

## Principio antitokens

Mantener dos capas:

```text
resumen corto -> detalle bajo demanda
```

No copiar la misma evidencia larga en varios MDs. Si un archivo crece demasiado,
crear o actualizar un resumen/índice y mover la evidencia detallada al historial
por subfase o a un archivo de detalle.

Leer antes de documentar:

```text
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/fase_1_sparse_global/historial/INDEX.md
```

Abrir archivos largos solo si falta información.

## Debe recibir

- resumen de cambios del implementador;
- resultado del build;
- análisis de logs;
- fase/subfase actual;
- historial específico reciente.

## Debe actualizar cuando aplique

- `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/07_ULTIMA_SESION.md`
- `codex/contexto/paquetes/<paquete>/<paquete>.md`
- `codex/contexto/paquetes/<paquete>/<componente>.md`
- `codex/pipeline/fase_*/historial/INDEX.md`
- `codex/pipeline/fase_*/historial/por_subfase/historial_<ID>.md`
- `codex/pipeline/fase_*/subfases/subfase_*.md`

No recrear un historial completo monolítico. El detalle debe vivir en el
historial por subfase o tema.

## Dividir historiales largos

Si un archivo `historial_<ID>.md` crece demasiado para leerse eficientemente
(por ejemplo, más de 1200-1500 líneas o varias entradas largas), dividirlo en
subhistoriales numerados:

- conservar `historial_<ID>.md` como índice breve y resumen de los subarchivos;
- crear `historial_<ID>_1.md`, `historial_<ID>_2.md`, ... con cortes cronológicos
  claros y entradas completas;
- el archivo principal debe explicar brevemente qué contiene cada subarchivo,
  incluyendo el rango de fechas y el tipo de contenido cubierto;
- usar esta regla para evitar archivos monolíticos y mantener la consulta rápida.

## Dividir especificaciones de subfase

Si un archivo `subfase_<ID>.md` crece demasiado (más de 1000 líneas), dividirlo
en 4 subarchivos temáticos:

- conservar `subfase_<ID>.md` como índice breve que lista los 4 subarchivos;
- crear `subfase_<ID>_especificacion.md`: Estado, Objetivo, Contexto, Diagnóstico, 
  Archivos permitidos/prohibidos (~180-200 líneas);
- crear `subfase_<ID>_implementacion.md`: Cambios requeridos/prohibidos, Funciones/clases 
  a localizar (~400-800 líneas, la más grande);
- crear `subfase_<ID>_testing.md`: Paquetes a compilar, Pruebas Gazebo/replay, Patrones 
  de reducción, Marcadores técnicos (~180-220 líneas);
- crear `subfase_<ID>_criterios.md`: Criterios de éxito/fallo, Documentación a actualizar, 
  Notas de diseño (~130-190 líneas);

El archivo principal debe ser muy breve (5-10 líneas) con título, descripción 
mínima y lista de subarchivos con descripción de contenido.

## Documentación obligatoria de paquetes

Si se modifica cualquier archivo de un paquete, actualizar la documentación
correspondiente en `codex/contexto/paquetes/`.

La actualización debe describir el estado actual del código tras la modificación,
no solo añadir una nota cronológica.

Revisar, según aplique:

- archivo general del paquete;
- archivo específico del componente tocado;
- lista de archivos relevantes;
- funciones, clases o nodos principales;
- topics, services, actions y parámetros afectados;
- logs o marcadores usados para validar;
- restricciones técnicas vigentes.

La evidencia larga de ejecución va al historial por subfase, no al `.md` del
paquete salvo que sea necesaria para entender el estado vigente.

## Historial por subfase

Ruta:

```text
codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_<ID>.md
```

Formato mínimo:

```text
## fecha/hora — Subfase <ID> — título corto

- objetivo intentado:
- archivos modificados:
- paquetes compilados:
- resultado de build:
- pruebas Gazebo/replay:
- patrones usados para reducir logs:
- evidencia positiva:
- evidencia negativa o ausente:
- conclusión: CONSEGUIDA / NO CONSEGUIDA / PARCIAL / BLOQUEADA
- siguiente paso recomendado:
```

Después de añadir una entrada:

- actualizar `historial/INDEX.md` con una línea corta;
- actualizar `01_ESTADO_ACTUAL_RESUMEN.md` si cambia la subfase o conclusión;
- reemplazar `07_ULTIMA_SESION.md` con un resumen breve de la última sesión.

No añadir sesiones antiguas debajo ni crear copias detalladas. El detalle
permanente debe quedar en el historial por subfase.

## Logs y sublogs

Si un log reducido sigue siendo grande, crear sublogs específicos antes de
analizarlo en detalle:

```bash
./codex/herramientas/split_simulation_log.sh --prueba X --fase 1L
```

Ver guía:

```text
codex/contexto/09_LOGS_Y_SUBLOGS.md
```

El log completo se conserva. Si falta un marcador obligatorio en el sublog,
buscarlo en el completo antes de concluir que no existe.

## Reglas

- No inventar resultados no comprobados.
- Si una prueba no se ejecutó, decirlo.
- Si RViz2 no se observó, decirlo.
- No borrar historial anterior de subfase; si un archivo se vuelve redundante,
  eliminarlo solo si su información ya está integrada en índices/historiales por
  subfase.
- No modificar código.
- No marcar una subfase como `realizado` si no hay evidencia suficiente.
- No duplicar grandes bloques entre estado, última sesión, paquete e historial.
