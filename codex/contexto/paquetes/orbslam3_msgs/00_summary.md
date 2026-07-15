# 00_summary — orbslam3_msgs

Resumen: Paquete de mensajes ROS 2 que define `OrbMap`, `OrbMapPoint`, `OrbKeyFrame` y mensajes auxiliares; contrato entre wrapper, servidor y corrector.

Interfaces definidas (ejemplos): `OrbMap`, `OrbMapPoint`, `OrbKeyFrame`, `MapCorrection`, `CorrectedKeyFrameArray`, `FiducialObservation`.

Uso: publicadas por `orbslam3_ros2`, consumidas por `orbslam3_server` y `orbslam3_multi`.

Reglas: no cambiar nombres/campos sin actualizar wrapper, servidor y corrector.

Detalles en los MDs del paquete (definiciones de mensajes y servicios).
