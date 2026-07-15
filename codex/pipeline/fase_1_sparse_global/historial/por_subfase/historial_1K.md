# Historial 1K

Este índice de subhistoriales resume y organiza el historial de la subfase `1K`.
Las entradas se han separado en archivos numerados para que el historial sea más fácil de leer y mantener.

## Nota de reclasificación 2026-07-14

Parte de lo probado durante `1L` pertenece realmente a `1K`: backup previo al
apply, escritura en `GlobalPoseStore`, poses propagadas, publicación de nube
tras apply, proyección de MapPoints desde KFs finales y corrección heredable para
KFs futuros. `1L` solo valida ese estado y decide si se conserva o se restaura.

- `historial_1K_1.md`: 2026-07-10 — Apply seguro en `GlobalPoseStore`, publicación de correcciones, raw intacto y primeros `partial_candidate=true` como deuda para `1L`.
- `historial_1K_2.md`: 2026-07-11 — Revalidación del apply con preservación de anclajes y checks adicionales de integridad antes de escribir en `GlobalPoseStore`.

Leer primero este resumen antes de abrir cualquiera de los subhistoriales.
