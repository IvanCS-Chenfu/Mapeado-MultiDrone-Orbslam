#!/usr/bin/env bash
set -uo pipefail

# build_selected_packages.sh
# Compila los paquetes ROS 2 indicados y guarda TODO el log en:
#   src/codex/archivos_auxiliares/logs/colcon_build.log
#
# Uso:
#   ./codex/herramientas/build_selected_packages.sh paquete1 paquete2 ...

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CODEX_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC_DIR="$(cd "$CODEX_DIR/.." && pwd)"
WS_DIR="$(cd "$SRC_DIR/.." && pwd)"
AUX_DIR="$CODEX_DIR/archivos_auxiliares"
LOG_DIR="$AUX_DIR/logs"
BUILD_LOG="$LOG_DIR/colcon_build.log"

mkdir -p "$LOG_DIR"
: > "$BUILD_LOG"

if [ "$#" -lt 1 ]; then
  {
    echo "[BUILD-ERROR] No se han indicado paquetes."
    echo "Uso: $0 paquete1 paquete2 ..."
  } | tee -a "$BUILD_LOG"
  exit 2
fi

{
  echo "[BUILD-START] $(date '+%Y-%m-%d %H:%M:%S')"
  echo "[BUILD-SCRIPT] $0"
  echo "[SRC_DIR] $SRC_DIR"
  echo "[WS_DIR] $WS_DIR"
  echo "[PACKAGES] $*"
  echo
} | tee -a "$BUILD_LOG"

cd "$WS_DIR" || {
  echo "[BUILD-ERROR] No se pudo entrar en WS_DIR=$WS_DIR" | tee -a "$BUILD_LOG"
  exit 2
}

# Cargar workspace si ya existe. No fallar si todavía no se ha compilado nunca.
if [ -f "$WS_DIR/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  # Los setup de colcon pueden leer variables no definidas como COLCON_TRACE;
  # desactivamos nounset solo durante el source para no abortar la herramienta.
  set +u
  source "$WS_DIR/install/setup.bash"
  set -u
  echo "[BUILD-INFO] Sourced $WS_DIR/install/setup.bash" | tee -a "$BUILD_LOG"
else
  echo "[BUILD-INFO] No existe $WS_DIR/install/setup.bash todavía. Se continúa sin source previo." | tee -a "$BUILD_LOG"
fi

echo "[BUILD-CMD] colcon build --packages-select $* --symlink-install" | tee -a "$BUILD_LOG"
echo | tee -a "$BUILD_LOG"

set +e
colcon build --packages-select "$@" --symlink-install 2>&1 | tee -a "$BUILD_LOG"
status=${PIPESTATUS[0]}
set -e

{
  echo
  echo "[BUILD-END] $(date '+%Y-%m-%d %H:%M:%S')"
  echo "[BUILD-EXIT-CODE] $status"
} | tee -a "$BUILD_LOG"

exit "$status"
