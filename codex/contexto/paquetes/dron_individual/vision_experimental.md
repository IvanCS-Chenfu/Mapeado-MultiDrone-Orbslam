# Scripts de visión experimental en `dron_individual`

## Estado

La carpeta `src/vision/` contiene scripts Python/C++ de pruebas anteriores de nube de puntos, profundidad, ICP, planos, TSDF y control basado en keypoints.

Ejemplos vistos:

- `vision.py`
- `nube_fuerza_bruta.py`
- `nube_total_planos.py`
- `test_pc_*.py`
- `guardar_cosas*/nube_puntos*.py`
- `guardar_cosas2/tsdf.py`
- `control_dron.cpp`

## Rol actual

No forman parte del pipeline activo de Fase 1 sparse global basado en ORB-SLAM3 multi-dron.

Pueden ser útiles más adelante para:

- Fase 5: nube densa global;
- pruebas de OpenCV;
- filtros de profundidad;
- experimentos de planos/ICP.

## Regla para Codex

No modificar estos scripts durante Fase 1 salvo que la tarea lo pida explícitamente.

Cuando empiece Fase 5, revisar estos scripts para rescatar ideas, pero no asumir que están limpios o integrados.
