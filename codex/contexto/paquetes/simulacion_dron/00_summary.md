# 00_summary — simulacion_dron

Resumen: Paquete que lanza Gazebo, spawnea drones, lanza `dron_individual` y `orbslam3_server`, y prepara escenarios automatizados para pruebas de subfase.

Launch principal: `ros2 launch simulacion_dron multi_dron.launch.py` (usado por `run_simulation.sh`).

Config: `config/sim_dron.yaml`, escenarios en `codex/archivos_auxiliares/trayectorias/*.yaml`.

Responsabilidades: GT para debug/fiducial simulado, creación de namespaces y cámaras simuladas.

Detalles en `launches.md`, `mundo_gazebo.md`, y `modelos.md`.
