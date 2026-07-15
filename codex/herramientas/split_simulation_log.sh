#!/usr/bin/env bash
set -u

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
AUX_DIR="${SRC_DIR}/codex/archivos_auxiliares"
LOG_DIR="${AUX_DIR}/logs"
PRUEBA=""
FASE="general"
INPUT=""
OUT_DIR="$LOG_DIR"

usage() {
  cat <<'USAGE'
Uso:
  split_simulation_log.sh --prueba X [--fase 1L] [--input archivo.log] [--out-dir dir]

Genera sublogs e índice breve:
  prueba_X.scenario.log
  prueba_X.errors.log
  prueba_X.F1H.log ... prueba_X.F1L.log
  prueba_X.gt_window.log
  prueba_X.index.md
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prueba) PRUEBA="${2:-}"; shift 2 ;;
    --fase) FASE="${2:-general}"; shift 2 ;;
    --input) INPUT="${2:-}"; shift 2 ;;
    --out-dir) OUT_DIR="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[SPLIT-LOG-ERROR] argumento desconocido: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ -z "$PRUEBA" && -z "$INPUT" ]]; then
  echo "[SPLIT-LOG-ERROR] falta --prueba o --input" >&2
  exit 2
fi

if [[ -z "$INPUT" ]]; then
  INPUT="${LOG_DIR}/prueba_${PRUEBA}.log"
  if [[ ! -f "$INPUT" && -f "${AUX_DIR}/prueba_${PRUEBA}.log" ]]; then
    INPUT="${AUX_DIR}/prueba_${PRUEBA}.log"
  fi
fi

if [[ -z "$PRUEBA" ]]; then
  base="$(basename "$INPUT")"
  PRUEBA="${base#prueba_}"
  PRUEBA="${PRUEBA%.log}"
fi

if [[ ! -f "$INPUT" ]]; then
  echo "[SPLIT-LOG-ERROR] no existe: $INPUT" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"
PREFIX="${OUT_DIR}/prueba_${PRUEBA}"

write_grep() {
  local name="$1"
  local pattern="$2"
  grep -E "$pattern" "$INPUT" > "${PREFIX}.${name}.log" || true
}

count_grep() {
  local pattern="$1"
  grep -Ec "$pattern" "$INPUT" || true
}

write_grep "scenario" "SCENARIO-RUNNER|SIM-"
SEVERE_PATTERN="\\[(ERROR|FATAL)\\]|Segmentation fault|Killed|std::bad_alloc|process has died|Traceback|Exception|No space left"
write_grep "errors" "$SEVERE_PATTERN"
write_grep "F1H" "F1H-"
write_grep "F1I" "F1I-"
write_grep "F1J" "F1J-"
write_grep "F1K" "F1K-"
write_grep "F1L" "F1L-"
write_grep "gt_window" "F1L-GT-WINDOW-STATS|F1L-GT-COLLATERAL-CHECK"

INDEX="${PREFIX}.index.md"
{
  echo "# Índice prueba ${PRUEBA}"
  echo
  echo "- log completo: \`${INPUT}\`"
  echo "- fase solicitada: \`${FASE}\`"
  echo "- líneas totales: $(wc -l < "$INPUT")"
  echo "- \`SCENARIO-RUNNER-DONE\`: $(count_grep 'SCENARIO-RUNNER-DONE')"
  echo "- \`SIM-DONE\`: $(count_grep 'SIM-DONE')"
  echo "- \`SIM-EXIT-CODE\`: $(count_grep 'SIM-EXIT-CODE')"
  echo "- errores graves: $(count_grep "$SEVERE_PATTERN")"
  echo "- \`F1H-\`: $(count_grep 'F1H-')"
  echo "- \`F1I-\`: $(count_grep 'F1I-')"
  echo "- \`F1J-\`: $(count_grep 'F1J-')"
  echo "- \`F1K-\`: $(count_grep 'F1K-')"
  echo "- \`F1L-\`: $(count_grep 'F1L-')"
  echo "- \`F1L-GT-WINDOW-STATS\`: $(count_grep 'F1L-GT-WINDOW-STATS')"
  echo
  echo "## Sublogs"
  echo
  for name in scenario errors F1H F1I F1J F1K F1L gt_window; do
    file="${PREFIX}.${name}.log"
    echo "- \`$(basename "$file")\`: $(wc -l < "$file") líneas"
  done
} > "$INDEX"

echo "[SPLIT-LOG-DONE] prueba=${PRUEBA} index=${INDEX}"
