#!/usr/bin/env bash
# ===============================================================
#  Build script for Arduino-Project (PlatformIO)
#  Arquitectura Computacional — Access Control and Security
# ===============================================================
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PIO="${HOME}/.local/bin/pio"
VERBOSE=""

cd "$PROJECT_DIR"

# ---- Parse flags ----
for arg in "$@"; do
    case "$arg" in
        -v|--verbose) VERBOSE="-v" ;;
    esac
done

# ---- Detect command (first non-flag arg) ----
CMD="build"
for arg in "$@"; do
    case "$arg" in
        -v|--verbose) ;;
        *) CMD="$arg"; break ;;
    esac
done

if [ ! -x "$PIO" ]; then
    echo "PlatformIO is not installed. Run:"
    echo "  pip install platformio"
    exit 1
fi

case "$CMD" in
    build)
        echo "=== Compiling ==="
        "$PIO" run $VERBOSE
        ;;
    upload)
        echo "=== Compiling and uploading to board ==="
        "$PIO" run --target upload $VERBOSE
        ;;
    run)
        echo "=== Uploading and opening serial monitor ==="
        "$PIO" run --target upload $VERBOSE
        echo ""
        echo "=== Serial monitor (9600 baud) ==="
        "$PIO" device monitor --baud 9600
        ;;
    monitor)
        echo "=== Serial monitor (9600 baud) ==="
        echo "  (Ctrl+C to exit)"
        "$PIO" device monitor --baud 9600
        ;;
    clean)
        echo "=== Cleaning build ==="
        "$PIO" run --target clean $VERBOSE
        rm -f compile_commands.json
        ;;
    compiledb)
        echo "=== Regenerating compile_commands.json for LSP ==="
        "$PIO" run --target compiledb $VERBOSE
        ;;
    deps)
        echo "=== Installing/updating dependencies ==="
        "$PIO" pkg install $VERBOSE
        ;;
    size)
        echo "=== Firmware size ==="
        "$PIO" run --target size $VERBOSE
        ;;
    full)
        echo "=== Full clean + deps + build ==="
        "$PIO" run --target clean
        "$PIO" pkg install $VERBOSE
        "$PIO" run $VERBOSE
        ;;
    *)
        echo "Usage: $0 [command] [-v|--verbose]"
        echo ""
        echo "Commands:"
        echo "  build      Compile firmware (default)"
        echo "  upload     Compile and upload to board"
        echo "  run        Upload and open serial monitor"
        echo "  monitor    Open serial monitor (9600 baud)"
        echo "  clean      Clean build files"
        echo "  compiledb  Regenerate compile_commands.json"
        echo "  deps       Install/update dependencies"
        echo "  size       Show firmware size"
        echo "  full       clean + deps + build"
        echo ""
        echo "Flags:"
        echo "  -v, --verbose  Verbose PlatformIO output"
        ;;
esac
