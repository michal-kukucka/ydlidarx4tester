#!/usr/bin/env bash
# One-time macOS/Linux setup: build Rozeta's native X4 driver and the Python venv.
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"

fail() { printf 'error: %s\n' "$1" >&2; exit 1; }

case "$(uname -s)" in
    Darwin) platform=macos; library=librozeta.dylib ;;
    Linux)  platform=linux; library=librozeta.so ;;
    *) fail "setup.sh supports macOS and Linux; use scripts/setup.ps1 on Windows" ;;
esac

command -v cmake >/dev/null 2>&1 || fail "cmake was not found. Install it with 'brew install cmake' (macOS) or your package manager."
if command -v ninja >/dev/null 2>&1; then
    generator=(-G Ninja)
else
    echo "Ninja was not found; falling back to the default CMake generator."
    generator=()
fi
command -v c++ >/dev/null 2>&1 || fail "no C++ compiler was found. On macOS run 'xcode-select --install'."

python_bin="${PYTHON:-python3}"
command -v "$python_bin" >/dev/null 2>&1 || fail "python3 was not found."
"$python_bin" -c 'import tkinter' 2>/dev/null || fail \
    "this Python has no tkinter, so the GUI cannot start. Install python.org's Python 3 or 'brew install python-tk'."

# Rozeta's X4 backend lives in the michal-kukucka/rozeta checkout. Accept an
# explicit ROZETA_DIR, otherwise prefer a sibling checkout that actually
# exports the X4 C ABI.
has_x4_abi() { [ -f "$1/src/c_api.cpp" ] && grep -q "rozeta_ydlidar_x4_create" "$1/src/c_api.cpp"; }

rozeta_root="${ROZETA_DIR:-}"
if [ -z "$rozeta_root" ]; then
    for candidate in "$(dirname "$project_root")/rozeta-x4" "$(dirname "$project_root")/rozeta"; do
        if has_x4_abi "$candidate"; then rozeta_root="$candidate"; break; fi
    done
fi
[ -n "$rozeta_root" ] || fail "no Rozeta checkout with the X4 C ABI was found beside this repository. Clone michal-kukucka/rozeta (branch main) next to it or set ROZETA_DIR."
[ -f "$rozeta_root/CMakeLists.txt" ] || fail "Rozeta source was not found at '$rozeta_root'."
has_x4_abi "$rozeta_root" || fail "'$rozeta_root' has no rozeta_ydlidar_x4 C ABI. Check out Rozeta's main branch (or a branch containing the YDLIDAR X4 backend)."

build_dir="$rozeta_root/build-x4"
echo "Configuring Rozeta's native YDLIDAR X4 driver ($platform)..."
# ${generator[@]+...} keeps macOS' bash 3.2 from treating an empty array as
# an unbound variable under `set -u`.
cmake -S "$rozeta_root" -B "$build_dir" ${generator[@]+"${generator[@]}"} \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DROZETA_BUILD_SHARED=ON \
    -DROZETA_BUILD_EXAMPLES=ON \
    -DROZETA_BUILD_TESTS=ON \
    -DROZETA_WITH_YDLIDAR=ON

echo "Building Rozeta and its X4 console smoke example..."
cmake --build "$build_dir" --parallel

rozeta_library="$build_dir/$library"
[ -f "$rozeta_library" ] || fail "build completed but $rozeta_library was not produced"

echo "Running Rozeta's C++ test suite..."
ctest --test-dir "$build_dir" --output-on-failure

venv_dir="$project_root/.venv"
venv_python="$venv_dir/bin/python"
if [ ! -x "$venv_python" ]; then
    echo "Creating the Python virtual environment..."
    "$python_bin" -m venv "$venv_dir"
fi
echo "Python demo uses only the standard library; no packages to download."

echo "Running Python demo adapter and simulation tests..."
ROZETA_LIBRARY="$rozeta_library" "$venv_python" -m unittest demo.test_x4_driver

cat <<SUMMARY

Setup complete. Start offline: ./scripts/run_demo.sh --simulate
Start hardware:              ./scripts/run_demo.sh --port /dev/cu.usbserial-0001
List serial ports:           ./scripts/run_demo.sh --list-ports
Rozeta library:              $rozeta_library
SUMMARY
