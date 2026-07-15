# Historial 1L

Este índice de subhistoriales resume y organiza el historial de la subfase `1L`.
Las entradas largas se han dividido en varios archivos para facilitar la lectura y evitar archivos monolíticos.

## Reclasificación 2026-07-14

Durante la ejecución de `1H` a `1L` se hicieron pruebas técnicas con nombres
`f1l_*` que no pertenecen funcionalmente a `1L`. Para leer este historial:

- lo que habla de selección de vértices, vecindad fiducial, splits de aristas,
  cobertura del grafo y soporte por KFs pertenece a `1I`;
- lo que habla de solver, rigidez, pesos, `RunDryRunGraphOnly`, replay offline,
  dump TSV o HTML diagnóstico pertenece a `1J`, aunque el archivo se llame
  `f1l_graph_*` o `f1l_debug_animation_*`;
- lo que habla de escritura en `GlobalPoseStore`, poses optimizadas/propagadas,
  publicación de MapPoints desde KFs finales y corrección heredable pertenece a
  `1K`;
- `1L` queda limitado a validación y diagnóstico post-apply: logs, GT debug,
  checks de propagación/global map, decisión `ACCEPT`, `PARTIAL` o
  `REJECT_ROLLBACK`.

- `historial_1L_1.md`: 2026-07-10 a 2026-07-12 00:54 — validación post-apply, rollback, pruebas iniciales de cargo/propagación, métricas de grafos y primeras hipótesis de rigidez.
- `historial_1L_2.md`: 2026-07-12 23:06 a 2026-07-13 00:14 — exploraciones de rigidez angular, soporte de KeyFrames y diagnósticos offline con dumps y HTML.
- `historial_1L_3.md`: 2026-07-13 00:27 a 2026-07-13 13:18 — pruebas de fiduciales y aristas, diseño de target/cola, restauración de líneas activas y primeros enfoques de vecindades fiduciales.
- `historial_1L_4.md`: 2026-07-14 a 2026-07-14 13:40 — consolidación de vecindades fiduciales, herencia de MapPoints, publicación final y ajustes de poses inducidas.

Leer primero este resumen antes de abrir cualquiera de los subhistoriales.
