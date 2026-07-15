# 07 — Última sesión

Este archivo se reemplaza en cada cierre de sesión. No añadir sesiones nuevas
debajo de las anteriores. El detalle histórico debe vivir en:

```text
codex/pipeline/fase_1_sparse_global/historial/por_subfase/
```

## 2026-07-14 — Definición de 1M como CovisibilityDatabase

- Objetivo:
  - convertir `1M` en la subfase de `CovisibilityDatabase`;
  - documentar cómo `1N`, `1O`, `1P` y `1Q` deben consumir esa base.
- Regla vigente:
  ```text
  1M = base de covisibilidad confirmada
  1N = BoW consulta la base y salta pares ya confirmados
  1O = verifica geométricamente candidatos no confirmados y entrega evidencia
  1P = inserta covisibilidad confirmada tras geometría
  1Q = construye grafos usando aristas de CovisibilityDatabase
  ```
- Diseño acordado:
  - la base no guarda candidatos ni campo `state`;
  - si una arista está en la base, se considera confirmada;
  - fuentes iniciales:
    ```text
    ORBSLAM3_NATIVE
    SERVER_LOOP_GEOMETRIC
    ```
  - cada arista debe guardar soporte, fuente, `relative_pose_measured`,
    `relative_pose_current`, `information_weight` y revisión/arrival si aplica;
  - `relative_pose_measured` no se sobrescribe;
  - `relative_pose_current` puede actualizarse tras optimizaciones aceptadas.
- Cambios documentales:
  - `subfase_1M.md` deja de estar vacía y define el contrato técnico;
  - `subfase_1N.md` consulta la base antes de mandar candidatos a 1O;
  - `1O` salta pares ya confirmados y produce evidencia geométrica para 1P;
  - `1P` inserta aristas confirmadas en `FUSION_CANDIDATE` y
    `LOOP_OPTIMIZATION_CANDIDATE`;
  - `1Q` añade restricciones de covisibilidad al `PoseGraphProblem`;
  - resúmenes de contexto/pipeline apuntan a `1M` como subfase actual
    pendiente de implementar.
- Build/simulación:
  - no se ejecutaron; solo documentación.
- Conclusión:
  - `1M` está definida y sigue `sin hacer`;
  - `1N+` no debe iniciarse hasta implementar `CovisibilityDatabase`.
