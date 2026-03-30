#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
UBUNTU_CODENAME="noble"
AVAILABLE_CORES="$(nproc)"
NUM_JOBS="${NUM_JOBS:-$AVAILABLE_CORES}"
if (( AVAILABLE_CORES > 1 )); then
  DEFAULT_CPP_NUM_JOBS=$((AVAILABLE_CORES / 2))
else
  DEFAULT_CPP_NUM_JOBS=1
fi
CPP_NUM_JOBS="${CPP_NUM_JOBS:-$DEFAULT_CPP_NUM_JOBS}"
VENV_DIR="${REPO_ROOT}/.mpc_venv"
VBCONTROL_DIR="${HOME}/vbcontrol"
DEPS_DIR="/tmp/mors_mpc_deps"

log() {
  printf '[install] %s\n' "$*"
}

warn() {
  printf '[install][warn] %s\n' "$*" >&2
}

die() {
  printf '[install][error] %s\n' "$*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Required command is missing: $1"
}

check_ubuntu24() {
  [[ -r /etc/os-release ]] || die "Cannot read /etc/os-release"
  # shellcheck disable=SC1091
  source /etc/os-release

  [[ "${ID:-}" == "ubuntu" ]] || die "This installer supports Ubuntu only (found: ${ID:-unknown})"
  [[ "${VERSION_ID:-}" == 24.* ]] || die "Ubuntu 24.x is required (found: ${VERSION_ID:-unknown})"
  log "Ubuntu check: ${PRETTY_NAME}"
}

check_ros2_jazzy() {
  local distro_env="${ROS_DISTRO:-}"
  local distro_path=""
  local ros2_path=""
  local detected=""

  if command -v ros2 >/dev/null 2>&1; then
    ros2_path="$(readlink -f "$(command -v ros2)")"
    if [[ "$ros2_path" =~ /opt/ros/([^/]+)/bin/ros2$ ]]; then
      distro_path="${BASH_REMATCH[1]}"
    fi
  fi

  if [[ -n "$distro_env" ]]; then
    detected="${distro_env,,}"
    [[ "$detected" == "jazzy" ]] || die "ROS_DISTRO='${distro_env}' but ROS2 Jazzy is required."
  fi

  if [[ -n "$distro_path" ]]; then
    [[ "${distro_path,,}" == "jazzy" ]] || die "Active ros2 executable is from '${distro_path}', but Jazzy is required."
    detected="jazzy"
  fi

  if [[ -z "$detected" ]]; then
    [[ -d /opt/ros/jazzy ]] || die "ROS2 Jazzy was not found in /opt/ros/jazzy. Install ROS2 Jazzy manually, then rerun."
    detected="jazzy"
    warn "ROS2 Jazzy found in /opt/ros/jazzy, but current shell is not sourced."
  fi

  log "ROS2 check: ${detected}"
}

ensure_sudo() {
  require_cmd sudo
  sudo -v >/dev/null
}

configure_realtime_limits() {
  local target_user="${SUDO_USER:-${USER}}"
  local limits_file="/etc/security/limits.d/99-mors-realtime.conf"

  log "Configuring realtime limits for user: ${target_user}"
  sudo tee "$limits_file" >/dev/null <<EOF
${target_user} soft rtprio 95
${target_user} hard rtprio 95
${target_user} soft memlock unlimited
${target_user} hard memlock unlimited
${target_user} soft nice -10
${target_user} hard nice -10
EOF
}

install_base_apt_packages() {
  log "Installing base apt dependencies..."
  sudo apt-get update
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    git \
    curl \
    ca-certificates \
    gnupg \
    python3.12 \
    python3.12-venv \
    python3-pip \
    libeigen3-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    liblcm-dev \
    liblcm-bin \
    liblcm-java \
    python3-lcm \
    default-jdk \
    libglfw3 \
    libglew2.2 \
    libgl1 \
    libegl1 \
    libxrandr2 \
    libxi6 \
    libxinerama1 \
    libxcursor1 \
    libosmesa6 \
    libblas-dev \
    liblapack-dev
}

robotpkg_repo_configured() {
  grep -Rhsq "robotpkg.openrobots.org/packages/debian/pub" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null
}

robotpkg_packages_available() {
  apt-cache show robotpkg-pinocchio >/dev/null 2>&1 && apt-cache show robotpkg-eiquadprog >/dev/null 2>&1
}

configure_robotpkg_repo() {
  local keyring="/usr/share/keyrings/robotpkg-archive-keyring.gpg"
  local list_file="/etc/apt/sources.list.d/robotpkg.list"
  local tmp_key

  log "Configuring robotpkg repository..."

  tmp_key="$(mktemp)"
  curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | gpg --dearmor > "$tmp_key"
  sudo install -m 0644 "$tmp_key" "$keyring"
  rm -f "$tmp_key"

  printf 'deb [signed-by=%s] http://robotpkg.openrobots.org/packages/debian/pub %s robotpkg\n' \
    "$keyring" "$UBUNTU_CODENAME" | sudo tee "$list_file" >/dev/null

  sudo apt-get update
}

install_robotpkg_dependencies() {
  if ! robotpkg_packages_available; then
    robotpkg_repo_configured || configure_robotpkg_repo
    robotpkg_packages_available || die "robotpkg packages are unavailable even after repo setup."
  fi

  log "Installing robotpkg dependencies (Pinocchio/eiquadprog)..."
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y robotpkg-pinocchio robotpkg-eiquadprog
}

ensure_vbcontrol() {
  if [[ -e "$VBCONTROL_DIR" && ! -d "$VBCONTROL_DIR/.git" ]]; then
    die "Path exists but is not a git repo: $VBCONTROL_DIR"
  fi

  if [[ -d "$VBCONTROL_DIR/.git" ]]; then
    log "vbcontrol already exists: $VBCONTROL_DIR"
    return
  fi

  log "Cloning vbcontrol..."
  git clone --depth 1 https://github.com/voltbro/vbcontrol "$VBCONTROL_DIR"
}

osqp_installed() {
  ldconfig -p 2>/dev/null | grep -q "libosqp\\.so"
}

osqpeigen_installed() {
  find /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu -type f \
    -path "*/cmake/OsqpEigen/OsqpEigenConfig.cmake" -print -quit 2>/dev/null | grep -q .
}

build_osqp_stack() {
  if osqp_installed && osqpeigen_installed; then
    log "OSQP + OsqpEigen already installed."
    return
  fi

  log "Building OSQP + OsqpEigen from source..."
  mkdir -p "$DEPS_DIR"

  if [[ ! -d "$DEPS_DIR/osqp/.git" ]]; then
    git clone --depth 1 https://github.com/osqp/osqp.git "$DEPS_DIR/osqp"
  fi
  cmake -Wno-deprecated -S "$DEPS_DIR/osqp" -B "$DEPS_DIR/osqp/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX=/usr/local
  cmake --build "$DEPS_DIR/osqp/build" -j "$NUM_JOBS"
  sudo cmake --install "$DEPS_DIR/osqp/build"

  if [[ ! -d "$DEPS_DIR/osqp-eigen/.git" ]]; then
    git clone --depth 1 https://github.com/robotology/osqp-eigen.git "$DEPS_DIR/osqp-eigen"
  fi
  cmake -Wno-deprecated -S "$DEPS_DIR/osqp-eigen" -B "$DEPS_DIR/osqp-eigen/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/usr/local \
    -DCMAKE_INSTALL_PREFIX=/usr/local
  cmake --build "$DEPS_DIR/osqp-eigen/build" -j "$NUM_JOBS"
  sudo cmake --install "$DEPS_DIR/osqp-eigen/build"
  sudo ldconfig
}

setup_python_venv() {
  log "Setting up Python venv: $VENV_DIR"
  if [[ ! -x "$VENV_DIR/bin/python" ]]; then
    python3.12 -m venv --system-site-packages "$VENV_DIR"
  fi

  "$VENV_DIR/bin/python" -m pip install --upgrade pip "setuptools<80" wheel
  "$VENV_DIR/bin/python" -m pip install --upgrade numpy scipy pyyaml transforms3d mujoco

  if ! "$VENV_DIR/bin/python" -c "import lcm" >/dev/null 2>&1; then
    warn "Python module 'lcm' is not visible in the venv. Trying pip install..."
    "$VENV_DIR/bin/python" -m pip install --upgrade lcm || true
  fi

  "$VENV_DIR/bin/python" -c "import lcm" >/dev/null 2>&1 || die "Python module 'lcm' is unavailable in $VENV_DIR"
}

generate_lcm_types() {
  log "Generating LCM message bindings..."
  (
    cd "$REPO_ROOT/lcm_msgs"
    export LCM_JAR="${LCM_JAR:-/usr/share/java/lcm.jar}"
    bash ./lcm_gen.sh
  )
}

export_build_env() {
  export PATH="/opt/openrobots/bin:${PATH}"
  export PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
  export LD_LIBRARY_PATH="/opt/openrobots/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="/opt/openrobots/lib/python3.12/site-packages:${PYTHONPATH:-}"
  export CMAKE_PREFIX_PATH="/opt/openrobots:/usr/local:${CMAKE_PREFIX_PATH:-}"
  export CONFIGPATH="${REPO_ROOT}/config"
}

build_ros_workspace() {
  log "Building ROS2 workspace packages..."
  require_cmd colcon

  # shellcheck disable=SC1091
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u

  (
    cd "$REPO_ROOT/ros_ws"
    colcon build --symlink-install --packages-select mors_ros_msgs robot_mode_controller mors_keyboard_control --cmake-args -Wno-deprecated
  )
}

build_cpp_modules() {
  log "Building C++ modules (LocomotionController, MorsLogger)..."
  log "Using $CPP_NUM_JOBS of $AVAILABLE_CORES available CPU cores for C++ compilation."

  cmake -Wno-deprecated -S "$REPO_ROOT/LocomotionController" -B "$REPO_ROOT/LocomotionController/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$REPO_ROOT/LocomotionController/build" -j "$CPP_NUM_JOBS"

  cmake -Wno-deprecated -S "$REPO_ROOT/MorsLogger" -B "$REPO_ROOT/MorsLogger/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$REPO_ROOT/MorsLogger/build" -j "$CPP_NUM_JOBS"
}

update_bashrc() {
  local bashrc="${HOME}/.bashrc"
  local tmp
  local marker_start="# >>> mors_mpc deps >>>"
  local marker_end="# <<< mors_mpc deps <<<"

  touch "$bashrc"
  tmp="$(mktemp)"

  awk -v start="$marker_start" -v end="$marker_end" '
    $0 == start {skip=1; next}
    $0 == end {skip=0; next}
    !skip {print}
  ' "$bashrc" > "$tmp"

  cat >> "$tmp" <<EOF

$marker_start
export MORS_MPC_ROOT="$REPO_ROOT"

# Pinocchio
export PATH=/opt/openrobots/bin:\$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.12/site-packages:\$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH

export LCM_JAR=/usr/share/java/lcm.jar
export CLASSPATH="\$MORS_MPC_ROOT/lcm_msgs/lcm_types.jar\${CLASSPATH:+:\$CLASSPATH}"

export PYTHONPATH="\$MORS_MPC_ROOT/lcm_msgs/:\$PYTHONPATH"
export PYTHONPATH="\$HOME/ProgramFiles/LCM-Grapher/examples/scalar/:\$PYTHONPATH"
export PYTHONPATH="\$HOME/ProgramFiles/LCM-Grapher/examples/scalar/graphing_ex:\$PYTHONPATH"

export CONFIGPATH=\$MORS_MPC_ROOT/config
source /opt/ros/jazzy/setup.bash
source "$REPO_ROOT/ros_ws/install/setup.bash"
$marker_end
EOF

  mv "$tmp" "$bashrc"
  log "Updated $bashrc"
}

main() {
  check_ubuntu24
  check_ros2_jazzy
  ensure_sudo
  configure_realtime_limits

  install_base_apt_packages
  require_cmd git
  require_cmd cmake
  require_cmd python3.12
  require_cmd curl
  require_cmd gpg

  install_robotpkg_dependencies
  ensure_vbcontrol
  build_osqp_stack
  setup_python_venv
  generate_lcm_types

  export_build_env
  build_ros_workspace
  build_cpp_modules
  update_bashrc

  log "Installation complete."
  log "Run: source ~/.bashrc"
  log "Then start simulation with: ./start_controller.sh --sim"
}

main "$@"
