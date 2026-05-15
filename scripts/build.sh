#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PIO="${HOME}/.local/bin/pio"

cd "$PROJECT_DIR"

if [ ! -x "$PIO" ]; then
    echo "PlatformIO no está instalado. Corré:"
    echo "  python3 -m pip install platformio --break-system-packages"
    exit 1
fi

case "${1:-build}" in
    build)
        echo "🔨 Compilando..."
        "$PIO" run
        ;;
    upload)
        echo "📤 Compilando y subiendo a la placa..."
        "$PIO" run --target upload
        ;;
    monitor)
        echo "🔌 Abriendo monitor serie (9600 baud)..."
        "$PIO" device monitor
        ;;
    clean)
        echo "🧹 Limpiando build..."
        "$PIO" run --target clean
        ;;
    compiledb)
        echo "📦 Regenerando compile_commands.json para el LSP..."
        "$PIO" run --target compiledb
        ;;
    deps)
        echo "📚 Instalando dependencias..."
        "$PIO" pkg install
        ;;
    size)
        echo "📏 Mostrando tamaño del firmware..."
        "$PIO" run --target size
        ;;
    *)
        echo "Uso: $0 [comando]"
        echo ""
        echo "Comandos disponibles:"
        echo "  build      Compilar el firmware (default)"
        echo "  upload     Compilar y subir a la placa"
        echo "  monitor    Abrir monitor serie"
        echo "  clean      Limpiar archivos de compilación"
        echo "  compiledb  Regenerar compile_commands.json para el LSP"
        echo "  deps       Instalar/actualizar dependencias"
        echo "  size       Mostrar tamaño del firmware"
        ;;
esac
