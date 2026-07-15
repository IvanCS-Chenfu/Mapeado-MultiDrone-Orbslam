# Pruebas tipicas de trayectoria

## Proposito

Este documento define trayectorias reutilizables que no pertenecen a una unica subfase. Sirven como pruebas de regresion para varias partes de la Fase 1: anclaje fiducial, revisits, snapshots, publicacion sparse, futura optimizacion y futura fusion.

Los archivos `codex/archivos_auxiliares/trayectorias/tray_prueba_X.yaml` son aliases temporales de ejecucion para `run_simulation.sh`. Cuando una trayectoria sea estable y util para varias subfases, debe conservarse tambien con un nombre semantico de prueba tipica.

Ejemplo de nombres estables:

```text
codex/archivos_auxiliares/trayectorias/prueba_tipica_anclaje_diferencial.yaml
codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
```

Regla practica:

- para ejecutar una subfase, copiar o adaptar una prueba tipica a `tray_prueba_1.yaml`, `tray_prueba_2.yaml`, etc.;
- no perder el nombre semantico de la prueba tipica si la trayectoria queda validada;
- documentar en historial que `tray_prueba_X.yaml` proviene de una prueba tipica.

## Prueba tipica 1 — anclaje diferencial y revisit simple

Nombre sugerido:

```text
prueba_tipica_anclaje_diferencial.yaml
```

Estado:

```text
Materializada en codex/archivos_auxiliares/trayectorias/prueba_tipica_anclaje_diferencial.yaml durante 1H.
```

Idea:

Validar el caso que se ha usado repetidamente en Fase 1 para comprobar anclaje, publicacion sparse, revisits sencillos y diferencias entre submapas.

Secuencia conceptual:

1. `drone_1` va al fiducial 2.
2. `drone_2` se coloca encima de `drone_1`, a distinta altura.
3. `drone_1` se mueve a la izquierda.
4. `drone_2` vuelve a colocarse encima de `drone_1`.
5. `drone_1` vuelve al fiducial 2.
6. `drone_2` vuelve a colocarse encima de `drone_1`.

Uso esperado:

- primera observacion fiducial para anclar;
- segunda observacion del mismo fiducial para medir revisit;
- comparar nubes de ambos drones en RViz2;
- generar `.record` pequeno o medio para replay rapido.

Limitaciones:

- no rodea todo el edificio;
- no fuerza necesariamente loops geometricos ricos;
- puede no cubrir fiducial 1.

## Prueba tipica 2 — rodeo del edificio con dos fiduciales

Nombre sugerido:

```text
prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
```

Idea:

Ambos drones rodean el edificio en sentidos contrarios, pasan por los fiduciales 2 y 1, y miran siempre hacia el edificio. Esta prueba debe ser util para validar revisits, drift acumulado, snapshots, futura optimizacion fiducial y futuros loops/fusion.

Tiempos de movimiento:

```text
tx = 40 s
ty = 40 s
tz = 40 s
tyaw = 13 s
```

Secuencia conceptual:

1. Ambos drones van a la vez al fiducial 2, con distinta altura como en las pruebas anteriores.
2. `drone_1` va a `(-10, -10)` con yaw `90 deg`.
3. `drone_2` va a `(10, -10)` con yaw `90 deg`.
4. `drone_1` va a `(-10, 10)` con yaw `0 deg`.
5. `drone_2` va a `(10, 10)` con yaw `180 deg`.
6. `drone_1` va al fiducial 1 con yaw `-90 deg`.
7. `drone_2` se coloca encima de `drone_1`.
8. `drone_1` va a `(10, 10)` con yaw `-90 deg`.
9. `drone_2` va a `(-10, 10)` con yaw `90 deg`.
10. `drone_1` va a `(10, -10)` con yaw `-180 deg`.
11. `drone_2` va a `(-10, -10)` con yaw `0 deg`.
12. Ambos drones vuelven al fiducial 2 con yaw `90 deg`.

Intencion geometrica:

- `drone_1` y `drone_2` recorren lados opuestos del edificio;
- los yaw se eligen para mirar hacia el edificio durante el rodeo;
- la visita a fiducial 1 debe crear una segunda referencia absoluta;
- la vuelta al fiducial 2 debe producir revisits con drift acumulado.

Precaucion importante:

La posicion exacta del fiducial 1 debe tomarse de la configuracion real de fiduciales del launch o del mapa. No inventar coordenadas si el fiducial 1 no esta configurado todavia. Si la subfase necesita esta prueba y el fiducial 1 aun no existe en configuracion, documentar la limitacion y usar solo la parte validable con fiducial 2.

Estado tras 1H:

```text
Materializada tras 1H como prueba de regresion larga en:
codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml
```

Nota de ejecucion:

- el launch activo del servidor global debe tener configurados los fiduciales legacy `ids=[1,2]`, `x=[0,0]`, `y=[9,-9]`, `z=[1,1]`, `yaw=[0,0]`, `radius=[2,2]`;
- por defecto conviene ejecutar esta prueba con `rawdb_record_enabled:=false` si queda poco disco, porque la trayectoria larga puede generar un `.record` grande;
- si se quiere conservar dataset para replay futuro, liberar espacio antes y usar un nombre semantico de record.

Validacion del 2026-07-09:

```bash
./codex/herramientas/run_simulation.sh --prueba 3 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false" --post-scenario-wait-sec 30 --startup-wait-sec 20 --timeout-sec 1200 --max-gazebo-retries 1
```

Resultado:

- `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`;
- `SIM-DONE prueba=3 success=true`;
- `SIM-EXIT-CODE 0`;
- se genero `codex/archivos_auxiliares/logs/prueba_3.log` y `codex/archivos_auxiliares/logs/prueba_3.reduced.log`;
- no se genero un `.record` nuevo porque se ejecuto con `rawdb_record_enabled:=false`.

Evidencia tecnica:

- el inicio en fiducial 2 produjo revisits `[F1H-FID-OK]` con error bajo;
- el paso por fiducial 1 produjo observaciones `[F1E-FID-KF-ASSOC] fid=1` y tareas `[F1H-FID-TASK-CREATED]`, por ejemplo `task_id=1 fid=1 drone_id=1 kf=211 error_t=0.445654` y `task_id=2 fid=1 drone_id=2 kf=158 error_t=22.743950`;
- la vuelta a fiducial 2 produjo revisits correctos en nuevos epochs, por ejemplo `drone_id=2 epoch=2 kf=245 error_t=0.001731` y `drone_id=1 epoch=3 kf=314 error_t=0.025037`;
- el estado final de tareas fiduciales llego a `total=41 pending=41 confirmed_ok=65 high_error=41 duplicates=0 no_pose=0 revisits=106`;
- no aparecieron `FATAL`, `Segmentation fault`, `Killed` ni `std::bad_alloc`;
- aparece `gazebo ... exit code 255` despues de `SIM-DONE`, durante cleanup, y no se considera fallo funcional.

Uso esperado:

- Fase 1H: provocar revisits fiduciales y medir residual absoluto.
- Fases 1I-1L: alimentar tareas de optimizacion fiducial.
- Fases 1N-1Q: generar candidatos de loop/subnube en lados opuestos.
- Fases 1V-1W: prueba integral de regresion con trayectoria mas larga.

## Relacion con `tray_prueba_X.yaml`

Las pruebas tipicas no sustituyen los YAML numerados. Los YAML numerados siguen siendo la entrada mecanica de `run_simulation.sh`.

Convencion recomendada:

```text
tray_prueba_1.yaml -> prueba live principal de la subfase actual
tray_prueba_2.yaml -> replay o variante rapida de la subfase actual
tray_prueba_4.yaml -> replay/espera legacy si una subfase lo conserva
```

Si una subfase necesita una prueba tipica:

1. conservar la prueba tipica con nombre semantico;
2. copiar su contenido a `tray_prueba_X.yaml` para ejecutar;
3. en el historial, indicar que `tray_prueba_X.yaml` deriva de la prueba tipica correspondiente.

## Criterio documental

Cuando una prueba tipica se modifique:

- actualizar este documento;
- actualizar `codex/contexto/paquetes/simulacion_dron/simulacion_dron.md` si cambia el uso del runner;
- registrar en el historial de la subfase que version se uso;
- no borrar una prueba tipica estable solo porque `tray_prueba_X.yaml` se reutilice para otra subfase.
