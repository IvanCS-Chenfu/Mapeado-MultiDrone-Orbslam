#!/usr/bin/env bash
set -euo pipefail

# reduce_simulation_log.sh
# Reduce codex/archivos_auxiliares/logs/prueba_X.log y escribe
# codex/archivos_auxiliares/logs/prueba_X.reduced.log dejando intacto el log completo.
# Los patrones pueden venir como regex o desde un archivo.
#
# Uso con regex:
#   ./codex/herramientas/reduce_simulation_log.sh --prueba 1 --patterns "SCENARIO-RUNNER|LOCAL-OPT|ERROR"
#
# Uso con archivo:
#   ./codex/herramientas/reduce_simulation_log.sh --prueba 1 --patterns-file codex/archivos_auxiliares/logs/patrones_prueba_1.txt

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CODEX_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
AUX_DIR="$CODEX_DIR/archivos_auxiliares"
LOG_DIR="$AUX_DIR/logs"

PRUEBA=""
PATTERNS=""
PATTERNS_FILE=""
MAX_LINES_PER_PATTERN="120"
MAX_TAIL_LINES="120"

usage() {
  cat <<USAGE
Uso:
  $0 --prueba N (--patterns "REGEX" | --patterns-file FILE) [opciones]

Opciones:
  --max-lines-per-pattern N    Máximo de líneas por patrón si se usa archivo. Defecto: 120.
  --max-tail-lines N           Últimas líneas conservadas del log original. Defecto: 120.
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --prueba)
      PRUEBA="${2:-}"; shift 2 ;;
    --patterns)
      PATTERNS="${2:-}"; shift 2 ;;
    --patterns-file)
      PATTERNS_FILE="${2:-}"; shift 2 ;;
    --max-lines-per-pattern)
      MAX_LINES_PER_PATTERN="${2:-}"; shift 2 ;;
    --max-tail-lines)
      MAX_TAIL_LINES="${2:-}"; shift 2 ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "[REDUCE-SIM-ERROR] Argumento desconocido: $1" >&2
      usage
      exit 2 ;;
  esac
done

if [ -z "$PRUEBA" ]; then
  echo "[REDUCE-SIM-ERROR] Falta --prueba N" >&2
  usage
  exit 2
fi

mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/prueba_${PRUEBA}.log"
if [ ! -f "$LOG_FILE" ] && [ -f "$AUX_DIR/prueba_${PRUEBA}.log" ]; then
  LOG_FILE="$AUX_DIR/prueba_${PRUEBA}.log"
fi
REDUCED_LOG="$LOG_DIR/prueba_${PRUEBA}.reduced.log"
TMP_LOG="$LOG_DIR/prueba_${PRUEBA}.reduced.tmp"

if [ ! -f "$LOG_FILE" ]; then
  echo "[REDUCE-SIM-ERROR] No existe $LOG_FILE" >&2
  exit 2
fi

if [ -z "$PATTERNS" ] && [ -z "$PATTERNS_FILE" ]; then
  PATTERNS="SCENARIO-RUNNER|LOCAL-OPT|LOCAL-POSE-GRAPH|LOOP-SUBCLOUD|SUBCLOUD|FUSION|FUSED|FID|ERROR|FATAL|WARN|Segmentation fault|Killed|std::bad_alloc|SIM-"
fi

{
  echo "[REDUCED-SIMULATION-LOG] $(date '+%Y-%m-%d %H:%M:%S')"
  echo "[SOURCE] $LOG_FILE"
  echo "[OUTPUT] $REDUCED_LOG"
  echo "[PRUEBA] $PRUEBA"
  echo "[INFO] Este archivo ha sido reducido para validación de fase. El log completo se conserva en SOURCE."
  echo

  echo "===== CABECERA DE LA PRUEBA ====="
  grep -E '^\[(SIM-|PRUEBA|YAML_FILE|LAUNCH_CMD|SCENARIO_NODE)' "$LOG_FILE" || true
  echo

  if [ -n "$PATTERNS_FILE" ]; then
    echo "===== PATRONES DESDE ARCHIVO: $PATTERNS_FILE ====="
    if [ ! -f "$PATTERNS_FILE" ]; then
      echo "[REDUCE-SIM-ERROR] No existe patterns-file: $PATTERNS_FILE"
    else
      while IFS= read -r pattern || [ -n "$pattern" ]; do
        # Ignorar líneas vacías o comentarios.
        if [ -z "$pattern" ]; then
          continue
        fi
        case "$pattern" in
          \#*) continue ;;
        esac
        echo
        echo "----- PATTERN: $pattern -----"
        grep -Ein "$pattern" "$LOG_FILE" | head -n "$MAX_LINES_PER_PATTERN" || true
      done < "$PATTERNS_FILE"
    fi
  else
    echo "===== PATRONES REGEX ====="
    echo "$PATTERNS"
    echo
    grep -Ein "$PATTERNS" "$LOG_FILE" | head -n 2000 || true
  fi

  echo
  echo "===== ÚLTIMAS $MAX_TAIL_LINES LÍNEAS DEL LOG ORIGINAL ====="
  tail -n "$MAX_TAIL_LINES" "$LOG_FILE" || true
} > "$TMP_LOG"

mv "$TMP_LOG" "$REDUCED_LOG"

echo "[REDUCE-SIM-DONE] Log completo conservado en $LOG_FILE"
echo "[REDUCE-SIM-DONE] Log reducido en $REDUCED_LOG"
