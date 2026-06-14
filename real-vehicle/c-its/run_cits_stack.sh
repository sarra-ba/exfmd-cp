#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISTRO=""
NODE_NAME="cam_listener"
BRIDGE_CONFIG=""
SESSION_NAME=""
ATTACH=true

usage() {
  cat <<'USAGE'
Usage: ./run_cits_stack.sh [options]

Starts a tmux session that launches:
  - domain_bridge on the host (Top Left Pane)
  - cam_listener node (Bottom Left Pane inside Devcontainer)
  - ldm_server: opens container bash, then types colcon build + source + ros2 run (Top Right Pane)
  - interactive container shell (Bottom Right Pane inside Devcontainer)

Options:
  -d, --distro <humble|jazzy>   ROS distro to start (if omitted, asks interactively)
  -n, --node <name>             v2x_apps node to run inside the container (default: cam_listener)
  -c, --bridge-config <path>    Bridge YAML config path (default: distro-specific bridge_config.yaml)
  -s, --session <name>          tmux session name (default: cits-<distro>)
      --no-attach               Create session without attaching
  -h, --help                    Show this help
USAGE
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
}

run_as_root() {
  if have_cmd sudo; then
    sudo "$@"
  else
    "$@"
  fi
}

install_tmux() {
  if have_cmd apt-get; then
    run_as_root apt-get update
    run_as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y tmux
  elif have_cmd brew; then
    brew install tmux
  elif have_cmd dnf; then
    run_as_root dnf install -y tmux
  elif have_cmd yum; then
    run_as_root yum install -y tmux
  elif have_cmd pacman; then
    run_as_root pacman -Sy --noconfirm tmux
  else
    echo "tmux is not installed and no supported package manager was found to install it automatically." >&2
    return 1
  fi
}

install_devcontainer_cli() {
  if have_cmd npm; then
    npm install -g @devcontainers/cli
  elif have_cmd apt-get; then
    run_as_root apt-get update
    run_as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y nodejs npm
    npm install -g @devcontainers/cli
  else
    echo "devcontainer is not installed and npm was not found to install it automatically." >&2
    return 1
  fi
}

ensure_command() {
  local cmd="$1"

  if have_cmd "$cmd"; then
    return 0
  fi

  case "$cmd" in
    tmux)
      install_tmux
      ;;
    devcontainer)
      install_devcontainer_cli
      ;;
    *)
      echo "Required command not found: $cmd" >&2
      return 1
      ;;
  esac

  if ! have_cmd "$cmd"; then
    echo "Installed $cmd, but it is still not available on PATH." >&2
    return 1
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--distro)
      DISTRO="${2:-}"
      shift 2
      ;;
    -n|--node)
      NODE_NAME="${2:-}"
      shift 2
      ;;
    -c|--bridge-config)
      BRIDGE_CONFIG="${2:-}"
      shift 2
      ;;
    -s|--session)
      SESSION_NAME="${2:-}"
      shift 2
      ;;
    --no-attach)
      ATTACH=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "$DISTRO" ]]; then
  echo "Choose ROS distro:"
  select selected in humble jazzy; do
    if [[ -n "${selected:-}" ]]; then
      DISTRO="$selected"
      break
    fi
  done
fi

if [[ "$DISTRO" != "humble" && "$DISTRO" != "jazzy" ]]; then
  echo "Invalid distro '$DISTRO'. Use humble or jazzy." >&2
  exit 1
fi

if [[ "$DISTRO" == "jazzy" ]]; then
  BRIDGE_DIR="$ROOT_DIR/domain_bridge_jazzy"
  APPS_DIR="$ROOT_DIR/ros_v2x_apps_jazzy"
else
  BRIDGE_DIR="$ROOT_DIR/domain_bridge_humble"
  APPS_DIR="$ROOT_DIR/ros_v2x_apps_humble"
fi

if [[ -z "${BRIDGE_DIR:-}" ]]; then
  echo "Could not find a bridge workspace for distro '$DISTRO'." >&2
  exit 1
fi

if [[ -z "${APPS_DIR:-}" ]]; then
  echo "Could not find a ros_v2x_apps workspace for distro '$DISTRO'." >&2
  exit 1
fi

if [[ -z "$SESSION_NAME" ]]; then
  SESSION_NAME="cits-${DISTRO}"
fi

if [[ -z "$BRIDGE_CONFIG" ]]; then
  BRIDGE_CONFIG="$BRIDGE_DIR/src/examples/carla_bridge_config.yaml"
fi

if [[ ! -f "$BRIDGE_CONFIG" ]]; then
  echo "Bridge config not found at $BRIDGE_CONFIG. Please provide --bridge-config." >&2
  exit 1
fi

echo "Building bridge workspace in $BRIDGE_DIR..."
if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon is not installed or not on PATH. Please install colcon and try again." >&2
  exit 1
fi
(cd "$BRIDGE_DIR" && colcon build --symlink-install) || {
  echo "colcon build failed for $BRIDGE_DIR" >&2
  exit 1
}

for cmd in tmux devcontainer docker; do
  ensure_command "$cmd"
done

# --- Single Container Workspace Handling ---

echo "Ensuring single devcontainer workspace instance is up..."
devcontainer up --workspace-folder "$APPS_DIR"

CONTAINER_ID=$(docker ps -q -f "label=devcontainer.local_folder=${APPS_DIR}" | head -n 1)

if [[ -z "$CONTAINER_ID" ]]; then
  echo "Error: Failed to find the running devcontainer ID via Docker labels." >&2
  exit 1
fi

CONTAINER_WORKSPACE_DIR="/home/cube/cube-its"
CONTAINER_DEV_WS_DIR="$CONTAINER_WORKSPACE_DIR/dev_ws"
ROS_SETUP_BASH="/opt/ros/humble/setup.bash"
ROS_DISTRO="$DISTRO"
CAM_MSG_PKG="etsi_its_cam_msgs"
CAM_MSG_APT_PKG="ros-${ROS_DISTRO}-etsi-its-messages"

container_exec() {
  local args=("$@")
  local opts=()
  local cmd=()
  while [[ ${#args[@]} -gt 0 ]]; do
    case "${args[0]}" in
      -w|--workdir)
        opts+=("${args[0]}" "${args[1]}")
        args=("${args[@]:2}")
        ;;
      -e|--env|-v|--volume)
        opts+=("${args[0]}" "${args[1]}")
        args=("${args[@]:2}")
        ;;
      --)
        args=("${args[@]:1}")
        break
        ;;
      -*)
        opts+=("${args[0]}")
        args=("${args[@]:1}")
        ;;
      *)
        cmd=("${args[@]}")
        break
        ;;
    esac
  done
  docker exec "${opts[@]}" "$CONTAINER_ID" "${cmd[@]}"
}

container_exec_root() {
  local args=("$@")
  local opts=()
  local cmd=()
  while [[ ${#args[@]} -gt 0 ]]; do
    case "${args[0]}" in
      -w|--workdir)
        opts+=("${args[0]}" "${args[1]}")
        args=("${args[@]:2}")
        ;;
      -e|--env|-v|--volume)
        opts+=("${args[0]}" "${args[1]}")
        args=("${args[@]:2}")
        ;;
      --)
        args=("${args[@]:1}")
        break
        ;;
      -*)
        opts+=("${args[0]}")
        args=("${args[@]:1}")
        ;;
      *)
        cmd=("${args[@]}")
        break
        ;;
    esac
  done
  docker exec -u root "${opts[@]}" "$CONTAINER_ID" "${cmd[@]}"
}

if ! container_exec -w "$CONTAINER_DEV_WS_DIR" bash -lc \
    "source $ROS_SETUP_BASH && ros2 pkg prefix $CAM_MSG_PKG >/dev/null 2>&1"; then
  echo "Installing missing ROS package '$CAM_MSG_PKG' inside the humble devcontainer..."
  if container_exec_root bash -lc "command -v apt-get >/dev/null 2>&1"; then
    container_exec_root bash -lc \
      "apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y $CAM_MSG_APT_PKG"
  else
    echo "Cannot install '$CAM_MSG_PKG': apt-get is not available in the container." >&2
    exit 1
  fi
fi

# --- Build command strings for each tmux pane ---

# Pane 0 (top-left): domain_bridge on the host
bridge_cmd="cd ${BRIDGE_DIR} && source install/setup.bash && ros2 run domain_bridge domain_bridge ${BRIDGE_CONFIG}"

# Pane 1 (bottom-left): cam_listener — open container bash, then type the run command
if [[ "$NODE_NAME" == "cam_listener" ]]; then
  CAM_LISTENER_CMD="ros2 run v2x_apps cam_listener"
else
  CAM_LISTENER_CMD="ros2 run v2x_apps ${NODE_NAME}"
fi
inner_shell="source ${ROS_SETUP_BASH} && source install/setup.bash && bash"
cam_enter_cmd="docker exec -it -w ${CONTAINER_DEV_WS_DIR} ${CONTAINER_ID} bash -lc '${inner_shell}'"

# Pane 3 (bottom-right): same plain interactive shell
shell_cmd="docker exec -it -w ${CONTAINER_DEV_WS_DIR} ${CONTAINER_ID} bash -lc '${inner_shell}'"

# --- Build the tmux session ---

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  tmux kill-session -t "$SESSION_NAME"
fi

tmux new-session -d -s "$SESSION_NAME" -n "cits-dashboard"
tmux split-window -h -t "$SESSION_NAME:cits-dashboard.0"
tmux split-window -v -t "$SESSION_NAME:cits-dashboard.1"
tmux split-window -v -t "$SESSION_NAME:cits-dashboard.0"

# Pane 0 — bridge (top-left)
tmux send-keys -t "$SESSION_NAME:cits-dashboard.0" "$bridge_cmd" C-m

# Pane 1 — cam_listener (bottom-left): open shell, then type run command
tmux send-keys -t "$SESSION_NAME:cits-dashboard.1" "$cam_enter_cmd" C-m
tmux send-keys -t "$SESSION_NAME:cits-dashboard.1" "source ${ROS_SETUP_BASH} && source install/setup.bash && ${CAM_LISTENER_CMD}" C-m

# Pane 2 — ldm_server (top-right):
#   Step 1: open an interactive bash inside the container (just like you'd do manually)
#   Step 2: type colcon build as a keystroke — waits for it to finish
#   Step 3: type source + ros2 run as the next keystroke — runs only after build completes
if [[ "$NODE_NAME" == "cam_listener" ]]; then
  tmux send-keys -t "$SESSION_NAME:cits-dashboard.2" "$shell_cmd" C-m
  tmux send-keys -t "$SESSION_NAME:cits-dashboard.2" "colcon build --symlink-install" C-m
  tmux send-keys -t "$SESSION_NAME:cits-dashboard.2" "source ${ROS_SETUP_BASH} && source install/setup.bash && ros2 run v2x_apps ldm_server" C-m
else
  tmux send-keys -t "$SESSION_NAME:cits-dashboard.2" "$shell_cmd" C-m
fi

# Pane 3 — interactive shell (bottom-right)
tmux send-keys -t "$SESSION_NAME:cits-dashboard.3" "$shell_cmd" C-m

echo "Started unified dashboard tmux session '$SESSION_NAME'."
echo "Bridge config: $BRIDGE_CONFIG"
echo "Node: $NODE_NAME"

if [[ "$ATTACH" == true ]]; then
  if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION_NAME"
  else
    tmux attach -t "$SESSION_NAME"
  fi
fi