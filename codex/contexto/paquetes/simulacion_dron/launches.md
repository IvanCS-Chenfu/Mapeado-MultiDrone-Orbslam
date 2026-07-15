# Launches de `simulacion_dron`

## `launch/multi_dron.launch.py`

Launch principal del sistema simulado.

## Flujo

1. Lee `simulacion_dron/config/sim_dron.yaml`.
2. Extrae:
   - número de drones;
   - namespace base;
   - mundo activo.
3. Lanza Gazebo con el mundo seleccionado.
4. Lanza un nodo `clock`.
5. Lanza `gui_tray_multi.py`.
6. Para cada dron:
   - crea namespace `dron_i`;
   - lanza `generador_URDF`;
   - incluye `dron_individual/launch/generar_dron.launch.py`.
7. Incluye `orbslam3_server/launch/global_orb_map_server.launch.py`.

Desde la validacion posterior a `1H`, acepta estos argumentos para pasarlos al servidor global:

```text
rawdb_record_enabled
rawdb_record_path
f1g_debug_mark_optimized_kf
pose_graph_max_vertices
pose_graph_max_temporal_edge_kf_gap
pose_graph_max_temporal_edge_length_m
pose_graph_corner_yaw_threshold_rad
f1j_dryrun_enabled
f1j_export_debug_plot
f1k_apply_enabled
f1l_validation_enabled
f1l_partial_apply_enabled
f1l_debug_force_reject_once
f1l_debug_force_reject_task_id
f1l_gt_kf_debug_enabled
f1l_gt_kf_debug_max_dt_sec
f1l_debug_animation_enabled
f1l_debug_animation_output_dir
f1l_graph_dump_enabled
f1l_graph_dump_output_dir
```

Esto permite ejecutar pruebas largas de RViz/logs sin llenar disco con un `.record` nuevo y activar/desactivar dry-run, apply, validacion post-apply, parciales y rollback debug sin editar el launch.

Desde la revalidacion de `1L` del 2026-07-12, tambien permite activar una
prueba corta con metricas GT por KeyFrame:

```text
f1l_gt_kf_debug_enabled:=true
f1l_gt_kf_debug_max_dt_sec:=1.0
f1l_debug_animation_enabled:=true
```

Estas metricas solo se usan como diagnostico externo en simulacion. El servidor
no puede usarlas para optimizar, seleccionar vertices, ajustar pesos ni
publicar mapa.

Si `f1l_debug_animation_enabled:=true`, cada tarea de optimizacion genera un
HTML en `codex/archivos_auxiliares/` para comparar mapa inicial, GT, vertices
del grafo y propuesta optimizada. El launch no abre automaticamente el
navegador; solo pasa el parametro al servidor.

Si `f1l_graph_dump_enabled:=true`, cada tarea de optimizacion guarda el grafo en
`codex/archivos_auxiliares/repeticiones/f1l_graph_task_<task_id>.tsv`. Ese fichero se puede
reproducir sin Gazebo con:

```bash
ros2 run orbslam3_multi test_opt_graph_offline --graph codex/archivos_auxiliares/repeticiones/f1l_graph_task_<task_id>.tsv
```

El replay offline comprueba serializacion y dry-run del solver, pero no valida
apply, rollback ni publicacion.

En `1L`, la prueba de rollback forzado se ejecuto pasando:

```text
f1l_debug_force_reject_once:=true
```

junto con `f1l_validation_enabled:=true` y `f1l_partial_apply_enabled:=true`.

## Namespaces

Si `namespace_base=dron` y `dron.numero=2`:

```text
/dron_1
/dron_2
```

Cada dron contiene sus topics relativos:

```text
/dron_1/AccionTrayectoria
/dron_1/sensor/GT/pose
/dron_1/sensor/camara_izq/image_raw
/dron_1/orbslam/orb_map_delta
```

## GUI `gui_tray_multi.py`

Nodo Python que permite enviar goals `TrayAction` a cada dron.

Importante para Codex:
- para automatización conviene crear un script no interactivo que llame a la misma action;
- no depender de clicks manuales si se quiere CI/simulación automática.

## Relación con pruebas de fases

Los escenarios YAML de `codex/escenarios/` deberían llamar a la action:

```text
/dron_X/AccionTrayectoria
```

No modificar este launch para cada prueba concreta salvo que haga falta cambiar número de drones, mundo o servidor. Para pruebas largas, preferir pasar `rawdb_record_enabled:=false` por launch antes que borrar datasets manualmente.
