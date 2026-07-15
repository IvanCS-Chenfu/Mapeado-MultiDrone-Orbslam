# 08 — Política de documentación compacta y ahorro de tokens

Objetivo: que Codex conserve todo el contexto importante sin leer miles de
líneas en cada chat.

## Regla principal

Cada tema debe tener dos capas:

```text
resumen corto -> detalle bajo demanda
```

Los resúmenes orientan. Los detalles conservan evidencia.

## Orden de lectura para futuros chats

1. `AGENTS.md`
2. `CONTEXTO_MINIMO_ACTUAL.md`
3. índice o resumen relevante
4. detalle solo si falta información

No abrir `01_ESTADO_ACTUAL.md`, historiales de otras subfases o logs completos
por costumbre.

## Tamaños recomendados

| Tipo de archivo | Tamaño recomendado |
|---|---|
| Bootstrap/resumen | 80-180 líneas |
| Subfase ejecutable | 80-180 líneas |
| Índice | 50-150 líneas |
| Docs de componente | 150-300 líneas si es posible |
| Historial por subfase | Puede ser largo, pero se lee solo cuando toca |
| Archivo completo/legacy | Sin límite, pero no debe ser lectura obligatoria |

Si un `.md` crece demasiado, crear un índice/resumen y mover la evidencia larga a
un archivo específico.

## Historial

No acumular todo en un solo archivo.

Usar:

```text
codex/pipeline/fase_1_sparse_global/historial/INDEX.md
codex/pipeline/fase_1_sparse_global/historial/por_subfase/historial_<ID>.md
```

Una entrada nueva va al historial de su subfase. `INDEX.md` recibe solo una línea
o tabla corta.

## Archivos de subfase

Los archivos de:

```text
codex/pipeline/fase_1_sparse_global/subfases/subfase_<ID>.md
```

son contratos ejecutables, no historiales.

Subfases grandes (`1O`, `1P`, `1Q`, `1R`, `1S`) se dividen en 4 subarchivos:

- `subfase_<ID>.md`: índice breve (5-10 líneas) que lista los 4 subarchivos.
- `subfase_<ID>_especificacion.md`: Estado, Objetivo, Contexto, Diagnóstico, 
  Archivos permitidos/prohibidos (~180-200 líneas).
- `subfase_<ID>_implementacion.md`: Cambios requeridos/prohibidos, Funciones/clases 
  a localizar (~400-800 líneas).
- `subfase_<ID>_testing.md`: Paquetes a compilar, Pruebas, Patrones, Marcadores 
  (~180-220 líneas).
- `subfase_<ID>_criterios.md`: Criterios de éxito/fallo, Documentación a actualizar, 
  Notas de diseño (~130-190 líneas).

Leer siempre primero el índice principal. Abrir subarchivos solo según necesidad.

Subfases más cortas mantienen un solo archivo `subfase_<ID>.md` bajo 250 líneas.

Deben contener:

- objetivo de la subfase;
- propiedad funcional clara;
- archivos permitidos/prohibidos;
- paquetes a compilar;
- pruebas y logs obligatorios;
- criterios de éxito/fallo.

No deben contener:

- narrativas largas de intentos;
- listas extensas de HTMLs/logs históricos;
- resultados de muchas pruebas descartadas;
- discusiones exploratorias que ya quedaron en el historial.

Si un archivo de subfase supera mucho las 250 líneas, mover el detalle a
`historial/por_subfase/historial_<ID>.md` o a un MD de detalle referenciado, y
dejar en la subfase solo el contrato vigente.

## Estado actual

`01_ESTADO_ACTUAL_RESUMEN.md` debe responder rápido:

- fase actual;
- subfase actual;
- estado/conclusión;
- siguiente paso;
- archivos críticos;
- paquetes a compilar;
- pruebas definidas;
- enlaces a detalle.

`01_ESTADO_ACTUAL.md` puede conservar narrativa larga, pero no debe ser el primer
archivo que se abra.

## Última sesión

`07_ULTIMA_SESION.md` es un handoff efímero. En cada cierre:

1. reemplazar su contenido anterior;
2. dejar solo el resumen de la sesión más reciente;
3. guardar el detalle permanente en el historial por subfase o tema;
4. no crear copias `*_DETALLADA_*` salvo petición explícita.

## Paquetes

Cada documentación de paquete debe describir el estado vigente del código. La
evidencia cronológica larga pertenece al historial, no a todos los MDs de
paquete.

Cuando se toque un paquete:

1. actualizar el `.md` específico del componente;
2. si el cambio altera el mapa rápido, actualizar `05_MAPA_PAQUETAS.md`;
3. no duplicar una entrada larga de historial dentro del paquete.

## Uso de la herramienta de búsqueda rápida

Si el agente no sabe dónde buscar, usar antes:

```bash
python3 codex/herramientas/find_context.py <query>
```

Esto busca primero en `codex/contexto/paquetes/*/00_summary.md` y luego en las rutas clave de `CODEX_INDEX.yaml`.

No usar `--deep` salvo que el resumen corto no ofrezca ninguna coincidencia relevante.

## Logs

No abrir logs completos si antes se puede responder con:

```bash
rg -n "<patron>" archivo.log
tail -n 80 archivo.log
wc -l archivo.log
```

Si el reducido sigue siendo grande, crear sublogs:

```bash
./codex/herramientas/split_simulation_log.sh --prueba 1 --fase 1L
```

Esto genera sublogs e índice breve con conteos.

## Qué no hacer

- No copiar la misma evidencia larga en `01_ESTADO_ACTUAL.md`,
  `07_ULTIMA_SESION.md`, docs de paquete e historial.
- No obligar a leer todos los ADRs si el resumen basta.
- No usar logs completos como primera lectura.
- No mantener copias antiguas si la información ya está integrada en el
  historial por subfase o en un índice activo.

## Regla para cerrar una tarea documental

Al reorganizar documentación, comprobar:

```bash
wc -l AGENTS.md codex/contexto/CONTEXTO_MINIMO_ACTUAL.md codex/pipeline/fase_1_sparse_global/pipeline_fase_1_RESUMEN.md
wc -l codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md codex/pipeline/fase_1_sparse_global/historial/INDEX.md
```

El objetivo no es que todo sea corto, sino que el camino inicial sí lo sea.
