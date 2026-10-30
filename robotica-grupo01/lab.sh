#!/usr/bin/env bash
###############################################################################
# lab.sh - the host driver on macOS (also works on Linux / Git Bash).
#   ./lab.sh build | up | shell | down | gazebo on|off | doctor | vnc | help
###############################################################################
set -o pipefail
cd "$(dirname "$0")" || exit 1

SERVICE=lab

if [ -t 1 ]; then B=$'\033[1m'; R=$'\033[0m'; E=$'\033[31m'; else B=''; R=''; E=''; fi
say() { echo "${B}==>${R} $*"; }
die() { echo "${E}!!${R} $*" >&2; exit 1; }

# Docker Desktop for Mac puts its CLI here; a fresh install is not always on
# PATH until the next login.
[ -d /Applications/Docker.app/Contents/Resources/bin ] && PATH="${PATH}:/Applications/Docker.app/Contents/Resources/bin"
case "${1:-help}" in help|-h|--help|code) ;; *)
    command -v docker >/dev/null 2>&1 || die "Docker is not installed (or not on PATH) - install Docker Desktop for Mac"
    docker compose version >/dev/null 2>&1 || die "'docker compose' is not available"
    docker info >/dev/null 2>&1 || die "the Docker engine is not responding - open Docker Desktop and wait for the whale to settle" ;;
esac
[ -f .env ] || { cp .env.example .env; say ".env created from .env.example - set ROS_DOMAIN_ID in it"; }

dc() { echo "${B}\$${R} docker compose $*"; docker compose "$@"; }

running() {
    local id; id="$(docker compose ps -q "${SERVICE}" 2>/dev/null)"
    [ -n "${id}" ] && [ "$(docker inspect -f '{{.State.Running}}' "${id}" 2>/dev/null)" = "true" ]
}
# The entrypoint needs a few seconds to bring up the display; commands that
# arrive before it has written /tmp/.lab-display-mode see no DISPLAY at all.
wait_ready() {
    for _ in $(seq 1 40); do
        docker compose exec -T "${SERVICE}" test -f /tmp/.lab-display-mode 2>/dev/null && return 0
        sleep 0.5
    done
}
ensure_up() { running || { say "starting container"; dc up -d >/dev/null; }; wait_ready; }

set_env() {  # set_env KEY VALUE
    # [[:space:]] rather than \s: macOS (BSD) sed does not know \s.
    if grep -qE "^[[:space:]]*$1[[:space:]]*=" .env; then
        sed -i.bak -E "s|^[[:space:]]*$1[[:space:]]*=.*|$1=$2|" .env && rm -f .env.bak
    else
        echo "$1=$2" >> .env
    fi
}

case "${1:-help}" in
    build)
        say "building robotics-lab0:jazzy - 45-90 minutes, mostly downloads"
        shift; dc --progress plain build "$@" ;;
    rebuild)        shift; dc --progress plain build "$@" ;;
    rebuild-xarm)   dc --progress plain build --build-arg "XARM_CACHEBUST=$(date +%Y-%m-%d-%H%M%S)" ;;
    up|start)       dc up -d && wait_ready && say "running. Open a terminal:  ./lab.sh shell" \
                        && say "GUI (RViz/Gazebo):  ./lab.sh vnc   ->  http://localhost:6080/" ;;
    down|stop)      dc down ;;
    restart)        dc down; dc up -d && wait_ready ;;
    shell|sh|bash)  ensure_up; docker compose exec "${SERVICE}" bash ;;
    run|exec)       shift; ensure_up; docker compose exec "${SERVICE}" "$@" ;;
    doctor|check)   ensure_up; docker compose exec "${SERVICE}" lab doctor ;;
    vnc)
        ensure_up
        say "opening http://localhost:6080/  (only serves anything when DISPLAY_MODE=vnc)"
        open "http://localhost:6080/vnc.html?autoconnect=1&resize=scale" 2>/dev/null \
            || xdg-open "http://localhost:6080/" 2>/dev/null || true ;;
    code)
        command -v code >/dev/null 2>&1 || die "VS Code's 'code' command is not on PATH (VS Code -> Cmd+Shift+P -> 'Shell Command: Install')"
        say "opening VS Code. Use: Reopen in Container."
        code . ;;
    status|ps)      dc ps; echo; docker volume ls --filter name=robotics-lab0 ;;
    logs)           shift; dc logs --tail 200 "$@" ;;
    gazebo)
        case "${2:-status}" in
            on|1)  set_env LAB_GAZEBO 1; running && docker compose exec "${SERVICE}" lab gazebo on  || say "enabled in .env" ;;
            off|0) set_env LAB_GAZEBO 0; running && docker compose exec "${SERVICE}" lab gazebo off || say "disabled in .env" ;;
            *)     grep -E '^LAB_GAZEBO=' .env; running && docker compose exec "${SERVICE}" lab gazebo status || true ;;
        esac ;;
    nuke)
        echo "This deletes the container and its volumes (build artefacts, caches)."
        echo "./src and ./shared are NOT touched."
        read -r -p "Type 'yes' to continue: " a
        [ "${a}" = "yes" ] && dc down -v || echo "cancelled" ;;
    help|-h|--help)
        cat <<'HELP'
./lab.sh - Lab 0 container (UFACTORY Lite 6 / ROS 2 Jazzy / Gazebo Harmonic)

  build            build the image (once, 45-90 min)
  up               start the container
  shell            open a terminal inside it  (repeat per "new terminal")
  run <cmd...>     run one command inside, e.g.  ./lab.sh run lab test 2
  doctor           check the environment
  vnc              open the RViz/Gazebo desktop in your browser
  code             open VS Code here (then: Reopen in Container)
  gazebo on|off    allow / block Gazebo - takes effect immediately
  down             stop it (your files are kept)
  restart          stop, start, re-read .env
  status | logs    what is running / its output
  rebuild-xarm     fresh clone + build of xarm_ros2
  nuke             delete container + volumes (./src and ./shared survive)

  ./src    -> ~/dev_ws/src   your packages: edit on the host, build inside
  ./shared -> ~/shared       files in and out (the view_frames PDF, notes)
HELP
        ;;
    *) die "unknown command: $1  (try: ./lab.sh help)" ;;
esac
