#!/usr/bin/env bash
set -euo pipefail

#PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_ROOT="$(pwd)"
DATA_DIR="$PROJECT_ROOT/data"

SLICE_PY="img2c_array_5.py"
MAP_PY="tmx_to_bin_map.py"
TILE_NAME_PNG="tilemap_packed.png"
TILE_NAME_H="tiles.h"
MAP_TMX="tilemap1.tmx"
MAP_MAP="tilemap1.map"


#echo "[build] Genero asset..."
#python3 "$PROJECT_ROOT/tools/$SLICE_PY" \
#  -f RGB565 "$DATA_DIR/$TILE_NAME_PNG" \
#  -o "$TILE_NAME_H" -f RGB565 --width 16 --height 16  # --single-array

echo "[build] Genero font..."
python3 "$PROJECT_ROOT/tools/$SLICE_PY" \
  "$DATA_DIR/$TILE_NAME_PNG" \
  -o "$TILE_NAME_H" -f PICOCALC_FONT --width 8 --height 10  --font-start-char 32 --font-end-char 91 --single-array

echo "[build] Genero map..."
python3 "$PROJECT_ROOT/tools/$MAP_PY" \
  "$DATA_DIR/$MAP_TMX" \
  "$DATA_DIR/$MAP_MAP"

