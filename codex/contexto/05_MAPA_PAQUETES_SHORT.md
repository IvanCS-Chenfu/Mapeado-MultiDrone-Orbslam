# 05 — Mapa de paquetes (Resumen rápido)

Resumen: Índice rápido de paquetes y responsabilidades. Leer este archivo antes
de abrir `05_MAPA_PAQUETES.md` completo.

Dónde buscar:
- Contexto mínimo: `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`
- Política tokens: `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`
- Paquetes: `codex/contexto/paquetes/` (buscar `SUMMARY.md` dentro de cada paquete)
- Subfases: `codex/pipeline/fase_1_sparse_global/subfases/`
- Historial por subfase: `codex/pipeline/fase_1_sparse_global/historial/por_subfase/`

Líneas clave (una por paquete):
- `orbslam3_msgs`: Contrato ROS 2 entre wrapper, servidor y corrector.
- `orbslam3_ros2`: Wrapper estéreo ORB-SLAM3 (publica pose local, OrbMap delta).
- `orbslam3_multi`: Backend algorítmico (RawMapDatabase, GlobalPoseStore, fusion, optimización).
- `orbslam3_server`: Adaptador ROS 2 / servidor.
- `dron_individual`: Control por dron y acciones.
- `simulacion_dron`: Gazebo + launch multi_dron.launch.py.
- `lib_tray`: Generación de trayectorias.

Para detalles completos abrir `05_MAPA_PAQUETES.md`.
