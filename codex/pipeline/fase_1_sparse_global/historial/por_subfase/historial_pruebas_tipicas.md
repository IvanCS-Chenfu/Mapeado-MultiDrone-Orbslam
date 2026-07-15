# Historial pruebas tipicas

> Extraido mecanicamente de `historial_fase_1.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-09 — Prueba tipica larga posterior a 1H — Rodeo del edificio con dos fiduciales

- objetivo:
  - ejecutar la prueba larga comentada por el usuario antes de avanzar a `1I`;
  - ambos drones rodean la casa en sentidos contrarios, pasan por fiducial 2, fiducial 1 y vuelven a fiducial 2 mirando hacia el edificio;
  - validar que la trayectoria queda disponible como prueba tipica y que los logs aportan evidencia para las subfases de optimizacion fiducial.
- archivos creados/modificados:
  - `codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
  - `codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml`;
  - `orbslam3_server/launch/global_orb_map_server.launch.py`;
  - `simulacion_dron/launch/multi_dron.launch.py`;
  - `codex/contexto/pruebas_clave/pruebas_tipicas.md`;
  - `codex/contexto/paquetes/simulacion_dron/simulacion_dron.md`;
  - `codex/contexto/paquetes/simulacion_dron/launches.md`;
  - `codex/contexto/paquetes/orbslam3_server/launches.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_1_sparse_global/historial/historial_fase_1.md`.
- cambios de launch:
  - `global_orb_map_server.launch.py` conserva configurados los fiduciales legacy `ids=[1,2]`, `x=[0,0]`, `y=[9,-9]`, `z=[1,1]`, `yaw=[0,0]`, `radius=[2,2]`;
  - `multi_dron.launch.py` permite pasar `rawdb_record_enabled` y `rawdb_record_path` al servidor global.
- compilacion:
  - no se ejecuto `build_selected_packages.sh` porque los cambios fueron launch/YAML/documentacion y los launch instalados son symlinks a `src`;
  - se valido sintaxis con `python3 -m py_compile simulacion_dron/launch/multi_dron.launch.py orbslam3_server/launch/global_orb_map_server.launch.py`;
  - se valido YAML con `yaml.safe_load`.
- simulacion ejecutada:

  ```bash
  ./codex/herramientas/run_simulation.sh --prueba 3 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false" --post-scenario-wait-sec 30 --startup-wait-sec 20 --timeout-sec 1200 --max-gazebo-retries 1
  ```

- resultado:
  - `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`;
  - `SIM-DONE prueba=3 success=true`;
  - `SIM-EXIT-CODE 0`;
  - se genero `codex/archivos_auxiliares/logs/prueba_3.log`;
  - se genero `codex/archivos_auxiliares/logs/prueba_3.reduced.log`;
  - no se genero `.record` nuevo porque se ejecuto con `rawdb_record_enabled:=false`.
- patrones de reduccion:

  ```text
  SCENARIO-RUNNER|SIM-DONE|SIM-EXIT-CODE|F1E-FID|F1H-FID|F1F-GLOBALMAP|F1G-FULL|ERROR|FATAL|Segmentation fault|Killed
  ```

- evidencia positiva:
  - inicio en fiducial 2 con revisits `[F1H-FID-OK]` y errores bajos;
  - paso por fiducial 1 con `[F1E-FID-KF-ASSOC] fid=1`;
  - creacion de tareas fiduciales con `[F1H-FID-TASK-CREATED]`, por ejemplo `task_id=1 fid=1 drone_id=1 kf=211 error_t=0.445654` y `task_id=2 fid=1 drone_id=2 kf=158 error_t=22.743950`;
  - vuelta al fiducial 2 con revisits `[F1H-FID-OK]` en epochs nuevos, por ejemplo `drone_id=2 epoch=2 kf=245 error_t=0.001731` y `drone_id=1 epoch=3 kf=314 error_t=0.025037`;
  - estadistica final: `total=41 pending=41 confirmed_ok=65 high_error=41 duplicates=0 no_pose=0 revisits=106`.
- evidencia negativa o ausente:
  - no aparecieron `FATAL`, `Segmentation fault`, `Killed` ni `std::bad_alloc`;
  - aparece `gazebo ... exit code 255` despues de `SIM-DONE`, durante cleanup, patron ya clasificado como no bloqueante;
  - el log reducido no conserva todas las lineas de `TASK_CREATED`, por lo que la evidencia detallada de fiducial 1 se consulto tambien en `prueba_3.log`.
- conclusion: `PRUEBA LARGA CONSEGUIDA`.
- siguiente paso recomendado:
  - ejecutar `subfase_1I.md`, usando `tray_prueba_3.yaml` como prueba de regresion larga cuando interese forzar tareas fiduciales reales.

