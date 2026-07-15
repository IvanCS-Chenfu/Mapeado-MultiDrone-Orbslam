#!/usr/bin/env bash
set -uo pipefail

# run_simulation.sh
# Ejecuta una prueba de simulación y guarda TODO el log en:
#   src/codex/archivos_auxiliares/logs/prueba_X.log
#
# El YAML de trayectoria se busca por defecto en:
#   src/codex/archivos_auxiliares/trayectorias/tray_prueba_X.yaml
#
# Uso mínimo:
#   ./codex/herramientas/run_simulation.sh \
#     --prueba 1 \
#     --launch "ros2 launch simulacion_dron <launch>.launch.py"
#
# Uso con opciones:
#   ./codex/herramientas/run_simulation.sh \
#     --prueba 1 \
#     --launch "ros2 launch simulacion_dron <launch>.launch.py" \
#     --post-scenario-wait-sec 20 \
#     --startup-wait-sec 15 \
#     --timeout-sec 900 \
#     --max-gazebo-retries 2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CODEX_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC_DIR="$(cd "$CODEX_DIR/.." && pwd)"
WS_DIR="$(cd "$SRC_DIR/.." && pwd)"
AUX_DIR="$CODEX_DIR/archivos_auxiliares"
LOG_DIR="$AUX_DIR/logs"
TRAY_DIR="$AUX_DIR/trayectorias"

PRUEBA=""
LAUNCH_CMD=""
YAML_FILE=""
POST_WAIT_SEC="20"
STARTUP_WAIT_SEC="15"
TIMEOUT_SEC="900"
MAX_GAZEBO_RETRIES="2"
GAZEBO_RETRY_WAIT_SEC="5"
SCENARIO_PACKAGE="simulacion_dron"
SCENARIO_EXECUTABLE="scenario_runner_node"
SCENARIO_PARAM_NAME="scenario_file"

usage() {
  cat <<USAGE
Uso:
  $0 --prueba N --launch "ros2 launch paquete launch.py" [opciones]

Opciones:
  --prueba N                         Número de prueba. Genera prueba_N.log y usa tray_prueba_N.yaml.
  --launch CMD                       Comando launch principal entre comillas.
  --yaml FILE                        YAML de trayectoria. Por defecto: codex/archivos_auxiliares/trayectorias/tray_prueba_N.yaml.
  --post-scenario-wait-sec SEC       Espera tras terminar scenario_runner_node. Defecto: 20.
  --startup-wait-sec SEC             Espera inicial tras lanzar simulación. Defecto: 15.
  --timeout-sec SEC                  Timeout para scenario_runner_node. Defecto: 900.
  --max-gazebo-retries N             Reintentos si Gazebo muere al arrancar. Defecto: 2.
  --gazebo-retry-wait-sec SEC        Espera tras matar Gazebo antes de reintentar. Defecto: 5.
  --scenario-package PKG             Paquete del nodo de escenarios. Defecto: simulacion_dron.
  --scenario-executable EXE          Ejecutable del nodo de escenarios. Defecto: scenario_runner_node.
  --scenario-param-name NAME         Parámetro ROS del YAML. Defecto: scenario_file.
USAGE
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --prueba)
      PRUEBA="${2:-}"; shift 2 ;;
    --launch)
      LAUNCH_CMD="${2:-}"; shift 2 ;;
    --yaml)
      YAML_FILE="${2:-}"; shift 2 ;;
    --post-scenario-wait-sec)
      POST_WAIT_SEC="${2:-}"; shift 2 ;;
    --startup-wait-sec)
      STARTUP_WAIT_SEC="${2:-}"; shift 2 ;;
    --timeout-sec)
      TIMEOUT_SEC="${2:-}"; shift 2 ;;
    --max-gazebo-retries)
      MAX_GAZEBO_RETRIES="${2:-}"; shift 2 ;;
    --gazebo-retry-wait-sec)
      GAZEBO_RETRY_WAIT_SEC="${2:-}"; shift 2 ;;
    --scenario-package)
      SCENARIO_PACKAGE="${2:-}"; shift 2 ;;
    --scenario-executable)
      SCENARIO_EXECUTABLE="${2:-}"; shift 2 ;;
    --scenario-param-name)
      SCENARIO_PARAM_NAME="${2:-}"; shift 2 ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "[SIM-ERROR] Argumento desconocido: $1" >&2
      usage
      exit 2 ;;
  esac
done

mkdir -p "$LOG_DIR" "$TRAY_DIR"

if [ -z "$PRUEBA" ]; then
  echo "[SIM-ERROR] Falta --prueba N" >&2
  usage
  exit 2
fi

if [ -z "$LAUNCH_CMD" ]; then
  echo "[SIM-ERROR] Falta --launch \"ros2 launch ...\"" >&2
  usage
  exit 2
fi

if [ -z "$YAML_FILE" ]; then
  YAML_FILE="$TRAY_DIR/tray_prueba_${PRUEBA}.yaml"
  if [ ! -f "$YAML_FILE" ] && [ -f "$AUX_DIR/tray_prueba_${PRUEBA}.yaml" ]; then
    YAML_FILE="$AUX_DIR/tray_prueba_${PRUEBA}.yaml"
  fi
fi

if [ ! -f "$YAML_FILE" ]; then
  echo "[SIM-ERROR] No existe el YAML de trayectoria: $YAML_FILE" >&2
  exit 2
fi

LOG_FILE="$LOG_DIR/prueba_${PRUEBA}.log"
: > "$LOG_FILE"

log() {
  echo "$@" | tee -a "$LOG_FILE"
}

LAUNCH_PID=""
LAUNCH_USES_PROCESS_GROUP=false
CURRENT_ATTEMPT=0
CURRENT_ATTEMPT_LOG_START_LINE=1

launch_is_alive() {
  if [ -z "${LAUNCH_PID:-}" ]; then
    return 1
  fi

  if [ "$LAUNCH_USES_PROCESS_GROUP" = true ]; then
    kill -0 "-$LAUNCH_PID" 2>/dev/null
  else
    kill -0 "$LAUNCH_PID" 2>/dev/null
  fi
}

signal_launch_tree() {
  local signal_name="$1"

  if [ -z "${LAUNCH_PID:-}" ]; then
    return 0
  fi

  if [ "$LAUNCH_USES_PROCESS_GROUP" = true ]; then
    kill "-$signal_name" "-$LAUNCH_PID" 2>/dev/null || true
  else
    kill "-$signal_name" "$LAUNCH_PID" 2>/dev/null || true
  fi
}

terminate_launch() {
  if launch_is_alive; then
    log "[SIM-CLEANUP] Enviando SIGINT launch PID=$LAUNCH_PID process_group=$LAUNCH_USES_PROCESS_GROUP"
    signal_launch_tree INT
    sleep 5

    if launch_is_alive; then
      log "[SIM-CLEANUP] Enviando SIGTERM launch PID=$LAUNCH_PID process_group=$LAUNCH_USES_PROCESS_GROUP"
      signal_launch_tree TERM
      sleep 3
    fi

    if launch_is_alive; then
      log "[SIM-CLEANUP] Forzando SIGKILL launch PID=$LAUNCH_PID process_group=$LAUNCH_USES_PROCESS_GROUP"
      signal_launch_tree KILL
      sleep 1
    fi
  fi

  LAUNCH_PID=""
  LAUNCH_USES_PROCESS_GROUP=false
}

kill_gazebo_processes() {
  log "[SIM-GAZEBO-KILL] killall -9 gzserver gzclient gazebo"
  killall -9 gzserver gzclient gazebo >> "$LOG_FILE" 2>&1 || true
}

gazebo_failed_during_startup() {
  if [ -n "${LAUNCH_PID:-}" ] &&
      ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log "[SIM-GAZEBO-DETECTED] reason=launch_process_exited_early attempt=$CURRENT_ATTEMPT pid=$LAUNCH_PID"
    return 0
  fi

  if tail -n +"$CURRENT_ATTEMPT_LOG_START_LINE" "$LOG_FILE" |
      grep -Ein "(gazebo|gzserver|gzclient).*process has died|process has died.*(gazebo|gzserver|gzclient)|exit code 255|gzserver: .*error|gzclient: .*error" >/dev/null 2>&1; then
    log "[SIM-GAZEBO-DETECTED] reason=log_pattern attempt=$CURRENT_ATTEMPT"
    return 0
  fi

  return 1
}

run_one_attempt() {
  CURRENT_ATTEMPT="$1"
  CURRENT_ATTEMPT_LOG_START_LINE=$(( $(wc -l < "$LOG_FILE") + 1 ))

  log "[SIM-ATTEMPT-START] attempt=$CURRENT_ATTEMPT max_gazebo_retries=$MAX_GAZEBO_RETRIES"

  log "[SIM-LAUNCH-START] $LAUNCH_CMD"
  # setsid crea un grupo de procesos propio. El cleanup puede enviar SIGINT
  # al grupo completo, que se parece mas a pulsar Ctrl+C en una terminal.
  if command -v setsid >/dev/null 2>&1; then
    setsid bash -lc "$LAUNCH_CMD" >> "$LOG_FILE" 2>&1 &
    LAUNCH_USES_PROCESS_GROUP=true
  else
    bash -lc "$LAUNCH_CMD" >> "$LOG_FILE" 2>&1 &
    LAUNCH_USES_PROCESS_GROUP=false
  fi
  LAUNCH_PID=$!
  log "[SIM-LAUNCH-PID] $LAUNCH_PID process_group=$LAUNCH_USES_PROCESS_GROUP"

  log "[SIM-WAIT-STARTUP] seconds=$STARTUP_WAIT_SEC"
  sleep "$STARTUP_WAIT_SEC"

  if gazebo_failed_during_startup; then
    log "[SIM-RETRY] reason=gazebo_died_early attempt=$CURRENT_ATTEMPT"
    terminate_launch
    kill_gazebo_processes
    log "[SIM-RETRY-WAIT] seconds=$GAZEBO_RETRY_WAIT_SEC"
    sleep "$GAZEBO_RETRY_WAIT_SEC"
    return 100
  fi

  SCENARIO_CMD=(
    ros2 run "$SCENARIO_PACKAGE" "$SCENARIO_EXECUTABLE"
    --ros-args
    -p "${SCENARIO_PARAM_NAME}:=${YAML_FILE}"
  )

  log "[SIM-SCENARIO-START] ${SCENARIO_CMD[*]}"
  set +e
  if command -v timeout >/dev/null 2>&1; then
    timeout "$TIMEOUT_SEC" "${SCENARIO_CMD[@]}" >> "$LOG_FILE" 2>&1
    scenario_status=$?
  else
    "${SCENARIO_CMD[@]}" >> "$LOG_FILE" 2>&1
    scenario_status=$?
  fi
  set -e
  log "[SIM-SCENARIO-EXIT-CODE] $scenario_status"

  log "[SIM-POST-SCENARIO-WAIT] seconds=$POST_WAIT_SEC"
  sleep "$POST_WAIT_SEC"

  if [ "$scenario_status" -ne 0 ]; then
    log "[SIM-ERROR] scenario_runner_node terminó con código $scenario_status"
    return "$scenario_status"
  fi

  log "[SIM-DONE] prueba=$PRUEBA success=true"
  return 0
}

cleanup() {
  local exit_code=$?
  terminate_launch
  log "[SIM-EXIT-CODE] $exit_code"
}
trap cleanup EXIT

{
  echo "[SIM-START] $(date '+%Y-%m-%d %H:%M:%S')"
  echo "[SRC_DIR] $SRC_DIR"
  echo "[WS_DIR] $WS_DIR"
  echo "[PRUEBA] $PRUEBA"
  echo "[LOG_FILE] $LOG_FILE"
  echo "[YAML_FILE] $YAML_FILE"
  echo "[LAUNCH_CMD] $LAUNCH_CMD"
  echo "[SCENARIO_NODE] ros2 run $SCENARIO_PACKAGE $SCENARIO_EXECUTABLE"
  echo "[STARTUP_WAIT_SEC] $STARTUP_WAIT_SEC"
  echo "[POST_SCENARIO_WAIT_SEC] $POST_WAIT_SEC"
  echo "[TIMEOUT_SEC] $TIMEOUT_SEC"
  echo "[MAX_GAZEBO_RETRIES] $MAX_GAZEBO_RETRIES"
  echo "[GAZEBO_RETRY_WAIT_SEC] $GAZEBO_RETRY_WAIT_SEC"
  echo
} | tee -a "$LOG_FILE"

cd "$WS_DIR" || {
  log "[SIM-ERROR] No se pudo entrar en WS_DIR=$WS_DIR"
  exit 2
}

if [ -f "$WS_DIR/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  # Los setup de colcon pueden leer variables no definidas como COLCON_TRACE;
  # desactivamos nounset solo durante el source para no abortar la prueba.
  set +u
  source "$WS_DIR/install/setup.bash"
  set -u
  log "[SIM-INFO] Sourced $WS_DIR/install/setup.bash"
else
  log "[SIM-ERROR] No existe $WS_DIR/install/setup.bash. Compila antes de simular."
  exit 2
fi

attempt=0
while [ "$attempt" -le "$MAX_GAZEBO_RETRIES" ]; do
  set +e
  run_one_attempt "$attempt"
  attempt_status=$?
  set -e

  if [ "$attempt_status" -eq 0 ]; then
    exit 0
  fi

  if [ "$attempt_status" -ne 100 ]; then
    exit "$attempt_status"
  fi

  attempt=$((attempt + 1))
done

log "[SIM-ERROR] Gazebo fallo durante el arranque tras $MAX_GAZEBO_RETRIES reintentos"
exit 1
