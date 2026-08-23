#!/usr/bin/env bash
# Launch the X4 visualizer on macOS/Linux with the locally built Rozeta driver.
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"

fail() { printf 'error: %s\n' "$1" >&2; exit 1; }

case "$(uname -s)" in
    Darwin) library=librozeta.dylib ;;
    Linux)  library=librozeta.so ;;
    *) fail "run_demo.sh supports macOS and Linux; use scripts/run_demo.ps1 on Windows" ;;
esac

simulate=0
list_ports=0
forward_angle="-125"
field_of_view="70"
port=""
headless=0
frames=0
record=""
show_all=0
extra=()

usage() {
    cat <<'USAGE'
usage: run_demo.sh [--port DEVICE] [--simulate] [--list-ports] [--headless]
                   [--frames N] [--record FILE] [--forward-angle DEG]
                   [--field-of-view DEG] [--show-all] [-- EXTRA ARGS...]

Any option this wrapper does not consume is passed straight to
demo/x4_visualizer.py; run it with --help for the complete list.
USAGE
}

while [ $# -gt 0 ]; do
    case "$1" in
        --port) port="${2:?--port needs a device}"; shift 2 ;;
        --simulate) simulate=1; shift ;;
        --list-ports) list_ports=1; shift ;;
        --headless) headless=1; shift ;;
        --frames) frames="${2:?--frames needs a count}"; shift 2 ;;
        --record) record="${2:?--record needs a path}"; shift 2 ;;
        --forward-angle) forward_angle="${2:?--forward-angle needs degrees}"; shift 2 ;;
        --field-of-view) field_of_view="${2:?--field-of-view needs degrees}"; shift 2 ;;
        --show-all) show_all=1; shift ;;
        -h|--help) usage; exit 0 ;;
        --) shift; extra+=("$@"); break ;;
        *) extra+=("$1"); shift ;;
    esac
done

python_bin="$project_root/.venv/bin/python"
[ -x "$python_bin" ] || python_bin="${PYTHON:-python3}"
command -v "$python_bin" >/dev/null 2>&1 || [ -x "$python_bin" ] || fail "no Python found. Run ./scripts/setup.sh first."

rozeta_root="${ROZETA_DIR:-}"
if [ -z "$rozeta_root" ]; then
    for candidate in "$(dirname "$project_root")/rozeta-x4" "$(dirname "$project_root")/rozeta"; do
        if [ -f "$candidate/build-x4/$library" ]; then rozeta_root="$candidate"; break; fi
    done
fi
rozeta_library="${ROZETA_LIBRARY:-${rozeta_root:+$rozeta_root/build-x4/$library}}"

if [ "$simulate" -eq 0 ] && { [ -z "$rozeta_library" ] || [ ! -f "$rozeta_library" ]; }; then
    fail "the Rozeta X4 driver ($library) is not built. Run ./scripts/setup.sh first."
fi

# Built with if blocks on purpose: under `set -e` a false `[ ... ] && ...`
# line would end the script instead of skipping the flag.
args=("$project_root/demo/x4_visualizer.py")
if [ "$simulate" -eq 0 ]; then args+=(--rozeta-lib "$rozeta_library"); fi
if [ "$list_ports" -eq 1 ]; then args+=(--list-ports); fi
if [ -n "$port" ]; then args+=(--port "$port"); fi
if [ "$simulate" -eq 1 ]; then args+=(--simulate); fi
if [ "$headless" -eq 1 ]; then args+=(--headless); fi
case "$frames" in
    ''|*[!0-9]*) fail "--frames needs a non-negative whole number" ;;
    *) if [ "$frames" -gt 0 ]; then args+=(--frames "$frames"); fi ;;
esac
if [ -n "$record" ]; then args+=(--record "$record"); fi
args+=(--forward-angle "$forward_angle" --field-of-view "$field_of_view")
if [ "$show_all" -eq 1 ]; then args+=(--show-all); fi
if [ ${#extra[@]} -gt 0 ]; then args+=("${extra[@]}"); fi

exec "$python_bin" "${args[@]}"
