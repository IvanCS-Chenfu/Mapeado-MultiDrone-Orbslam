#!/usr/bin/env bash
set -euo pipefail

# reduce_build_log.sh
# Reduce codex/archivos_auxiliares/logs/colcon_build.log y escribe
# codex/archivos_auxiliares/logs/colcon_build.reduced.log dejando intacto el log completo.
#
# Uso:
#   ./codex/herramientas/reduce_build_log.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CODEX_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
AUX_DIR="$CODEX_DIR/archivos_auxiliares"
LOG_DIR="$AUX_DIR/logs"
BUILD_LOG="$LOG_DIR/colcon_build.log"
if [ ! -f "$BUILD_LOG" ] && [ -f "$AUX_DIR/colcon_build.log" ]; then
  BUILD_LOG="$AUX_DIR/colcon_build.log"
fi
mkdir -p "$LOG_DIR"
REDUCED_LOG="$LOG_DIR/colcon_build.reduced.log"
TMP_LOG="$LOG_DIR/colcon_build.reduced.tmp"

if [ ! -f "$BUILD_LOG" ]; then
  echo "[REDUCE-BUILD-ERROR] No existe $BUILD_LOG"
  exit 2
fi

PATTERN='(^|[[:space:]])(error:|fatal error:|undefined reference|was not declared|is not a member of|no matching function|candidate:|static assertion failed|CMake Error|CMakeFiles|gmake\[[0-9]+\]: \*\*\*|make\[[0-9]+\]: \*\*\*|Failed|failed|stderr:|BUILD-EXIT-CODE|BUILD-CMD|PACKAGES|In file included from|required from here|note:|warning:)' 

{
  echo "[REDUCED-BUILD-LOG] $(date '+%Y-%m-%d %H:%M:%S')"
  echo "[SOURCE] $BUILD_LOG"
  echo "[OUTPUT] $REDUCED_LOG"
  echo "[INFO] Este archivo ha sido reducido para diagnóstico. El log completo se conserva en SOURCE."
  echo

  echo "===== CONTEXTO BUILD ====="
  grep -E '^\[(BUILD|PACKAGES|WS_DIR|SRC_DIR|BUILD-CMD|BUILD-EXIT-CODE)' "$BUILD_LOG" || true
  echo

  echo "===== ERRORES Y LÍNEAS RELEVANTES ====="
  grep -Ein "$PATTERN" "$BUILD_LOG" | tail -n 400 || true
  echo

  echo "===== ÚLTIMAS 120 LÍNEAS DEL BUILD ORIGINAL ====="
  tail -n 120 "$BUILD_LOG" || true
} > "$TMP_LOG"

mv "$TMP_LOG" "$REDUCED_LOG"

echo "[REDUCE-BUILD-DONE] Log completo conservado en $BUILD_LOG"
echo "[REDUCE-BUILD-DONE] Log reducido en $REDUCED_LOG"
