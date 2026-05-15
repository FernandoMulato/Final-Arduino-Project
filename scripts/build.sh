#!/usr/bin/env bash
# ===============================================================
#  Script de compilacion para Arduino-Project (PlatformIO)
#  Arquitectura Computacional — Control de Acceso y Seguridad
# ===============================================================
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PIO="${HOME}/.local/bin/pio"
VERBOSE=""

cd "$PROJECT_DIR"

# ---- Parsear flags ----
for arg in "$@"; do
    case "$arg" in
        -v|--verbose) VERBOSE="-v" ;;
    esac
done

# ---- Detectar comando (primer arg no-flag) ----
CMD="build"
for arg in "$@"; do
    case "$arg" in
        -v|--verbose) ;;
        *) CMD="$arg"; break ;;
    esac
done

if [ ! -x "$PIO" ]; then
    echo "PlatformIO no esta instalado. Corre:"
    echo "  pip install platformio"
    exit 1
fi

case "$CMD" in
    build)
        echo "=== Compilando ==="
        "$PIO" run $VERBOSE
        ;;
    upload)
        echo "=== Compilando y subiendo a la placa ==="
        "$PIO" run --target upload $VERBOSE
        ;;
    run)
        echo "=== Subiendo y abriendo monitor serie ==="
        "$PIO" run --target upload $VERBOSE
        echo ""
        echo "=== Monitor serie (9600 baud) ==="
        "$PIO" device monitor --baud 9600
        ;;
    monitor)
        echo "=== Monitor serie (9600 baud) ==="
        echo "  (Ctrl+C para salir)"
        "$PIO" device monitor --baud 9600
        ;;
    clean)
        echo "=== Limpiando build ==="
        "$PIO" run --target clean $VERBOSE
        rm -f compile_commands.json
        ;;
    compiledb)
        echo "=== Regenerando compile_commands.json para el LSP ==="
        "$PIO" run --target compiledb $VERBOSE
        ;;
    deps)
        echo "=== Instalando/actualizando dependencias ==="
        "$PIO" pkg install $VERBOSE
        ;;
    size)
        echo "=== Tamano del firmware ==="
        "$PIO" run --target size $VERBOSE
        ;;
    full)
        echo "=== Limpieza total + dependencias + compilacion ==="
        "$PIO" run --target clean
        "$PIO" pkg install $VERBOSE
        "$PIO" run $VERBOSE
        ;;
    *)
        echo "Uso: $0 [comando] [-v|--verbose]"
        echo ""
        echo "Comandos:"
        echo "  build      Compilar el firmware (default)"
        echo "  upload     Compilar y subir a la placa"
        echo "  run        Subir y abrir monitor serie"
        echo "  monitor    Abrir monitor serie (9600 baud)"
        echo "  clean      Limpiar archivos de compilacion"
        echo "  compiledb  Regenerar compile_commands.json"
        echo "  deps       Instalar/actualizar dependencias"
        echo "  size       Mostrar tamano del firmware"
        echo "  full       clean + deps + build"
        echo ""
        echo "Flags:"
        echo "  -v, --verbose  Salida detallada de PlatformIO"
        ;;
esac
