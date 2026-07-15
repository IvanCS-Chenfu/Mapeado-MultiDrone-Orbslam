# Plantilla de subfase ejecutable por Codex

Usar esta plantilla al crear o revisar cualquier `subfase_*.md`.

El objetivo es que otro chat de Codex pueda recibir la orden:

```text
Haz la subfase X de la fase Y y comprueba si funciona.
```

y tenga suficiente informacion para planificar, modificar, compilar, simular, analizar logs y documentar el resultado sin inventar criterios.

---

## Formato obligatorio de subfase

Cada `subfase_*.md` debe ser un documento ejecutivo, breve y libre de resultados de prueba.

No incluir en este archivo:

- resultados de build;
- conclusiones de simulación;
- historial de ejecuciones;
- notas largas de diagnóstico;
- archivos históricos o registros detallados.

La evidencia, los resultados y la historia deben escribirse en un archivo separado en:

```text
codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md
```

o en un archivo específico por subfase dentro de `codex/pipeline/fase_1_sparse_global/historial/`.

El contenido de una subfase se debe limitar a:

- Estado
- Objetivo técnico
- Contexto obligatorio
- Diagnóstico de partida breve
- Archivos permitidos
- Archivos prohibidos
- Funciones/clases/nodos a localizar
- Cambios requeridos
- Cambios prohibidos
- Paquetes a compilar
- Pruebas Gazebo requeridas
- Patrones de log
- Criterio de éxito
- Criterio de fallo o parcial
- Documentación a actualizar

---

## Estado

```text
actual | sin hacer | realizado | bloqueado | opcional
```

---

## Objetivo tecnico

Explicar que debe cambiar en el sistema y por que.

Debe ser verificable con build, simulacion y logs.

---

## Contexto obligatorio a leer

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- pipeline de la fase
- historial reciente de la fase/subfase
- documentacion de paquetes afectados
- ADRs relacionados

---

## Diagnostico de partida

Describir el problema actual con logs, sintomas o comportamiento observado.

Incluir marcadores exactos si existen:

```text
LOG-MARKER-EXAMPLE
reason=...
```

---

## Archivos permitidos a modificar

- `paquete/ruta/archivo.cpp`
- `paquete/ruta/archivo.hpp`
- `paquete/config/archivo.yaml`
- `codex/archivos_auxiliares/trayectorias/tray_prueba_X.yaml`
- documentacion relacionada

---

## Archivos prohibidos

- paquetes o rutas que no deben tocarse;
- wrapper/mensajes si no son objetivo;
- `ORB_SLAM3` salvo permiso explicito;
- legacy que no pertenezca a la subfase.

---

## Funciones, clases o nodos que hay que localizar

- `FuncionExacta`
- `ClaseExacta`
- `nodo_ros`
- topic/service/action relacionado

No inventar nombres: si no se conocen, indicar que `planificador_fase` debe localizarlos con busqueda estatica antes de implementar.

---

## Cambios requeridos

Lista concreta de cambios funcionales.

Cada cambio debe tener:
- intencion;
- archivo probable;
- condicion de seguridad;
- log nuevo o existente que permita validarlo.

---

## Cambios prohibidos

Lista de soluciones tentadoras que no deben hacerse.

Ejemplos:
- no aumentar limites como solucion principal;
- no usar ground truth;
- no redisenar mensajes;
- no implementar subfases futuras;
- no limpiar legacy masivamente.

---

## Paquetes a compilar

Comando esperado:

```bash
./codex/herramientas/build_selected_packages.sh paquete_1 paquete_2
```

Si hay dependencias adicionales reales, `implementador_fase` puede anadirlas justificandolo en historial.

---

## Pruebas Gazebo requeridas

### Prueba 1 — nombre corto

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
```

Secuencia esperada:

1. esperar arranque;
2. mover drones o activar acciones;
3. esperar resultado;
4. esperar al final.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Observacion esperada en RViz2, si aplica:

```text
...
```

### Prueba 2 — nombre corto

Repetir solo si la subfase necesita mas de una prueba.

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|MARCADOR_TECNICO|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|MARCADOR_TECNICO|ERROR|FATAL|Segmentation fault|Killed
```

Los patrones deben ser especificos. Evitar patrones demasiado amplios si pueden ocultar los markers importantes por exceso de lineas.
La reducción genera `prueba_X.reduced.log` y conserva `prueba_X.log` completo.
Si el reducido no muestra datos suficientes, revisar el completo antes de repetir
Gazebo o concluir que el código no emitió el marcador.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. todas las pruebas requeridas se ejecutan;
3. `scenario_runner_node` se ejecuta en cada prueba que lo use;
4. los goals requeridos terminan con `success=true`;
5. aparecen todos los marcadores tecnicos obligatorios;
6. no aparecen errores graves no explicados;
7. el comportamiento visual esperado se observa o se documenta que no fue observable;
8. el historial queda actualizado.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:
- no compila;
- una prueba requerida no se ejecuta;
- falta un marcador obligatorio;
- aparece un error grave;
- el resultado contradice el objetivo tecnico.

La subfase puede marcarse como `PARCIAL` si:
- compila;
- parte de las pruebas pasan;
- hay evidencia positiva, pero falta una condicion obligatoria.

La subfase debe marcarse como `BLOQUEADA` solo si:
- falta informacion critica;
- hay una dependencia externa no resuelta;
- el mismo bloqueo se repite y no puede resolverse con cambios minimos.

---

## Documentacion a actualizar

- historial de la fase/subfase;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- documentacion de paquetes modificados;
- subfase actual o siguiente si el plan cambia.

Para cada paquete modificado, actualizar `codex/contexto/paquetes/<paquete>/`.

La documentacion del paquete debe reflejar el codigo actual tras la modificacion:
- archivos relevantes;
- funciones/clases/nodos principales;
- topics, services, actions o parametros afectados;
- logs o markers de validacion;
- restricciones tecnicas que siguen vigentes;
- estado de validacion si el resultado fue `PARCIAL` o `NO CONSEGUIDA`.

Si existe un `.md` especifico del componente tocado, actualizarlo. Si no existe, actualizar el `.md` general del paquete o crear uno especifico cuando aporte trazabilidad real.

No marcar como `realizado` si el criterio de exito no se cumple.
