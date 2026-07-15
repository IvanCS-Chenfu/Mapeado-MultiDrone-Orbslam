# 00_summary — orbslam3_ros2

Resumen: Wrapper ROS 2 para ORB-SLAM3; publica `pose_local`, `orb_map_delta` y ofrece `GetOrbMap`.

Interfaces:
- Publishes: `orbslam/pose_local`, `orbslam/orb_map_delta`.
- Services: `orbslam/get_full_map`.

Ejecutables/nodos: `StereoSlamNode`.

Parámetros relevantes: `drone_id`, `local_map_frame`, `delta_publish_period_frames`, `use_sim_time`.

Relación: alimentado por `simulacion_dron` (cámaras), usa `ORB_SLAM3`, define mensajes en `orbslam3_msgs` y es consumido por `orbslam3_server`.

Detalles en `stereo_slam_node.md` y demás MDs del directorio.
