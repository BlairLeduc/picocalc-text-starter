# GFX_TILES Format Generator

## Overview

The `GFX_TILES` format generates C header files compatible with the PicoCalc graphics system (gfx.h/gfx.c). It converts images into tilesets that can be used directly with the tile-based rendering functions.

## Usage

```bash
python3 img2c_array_5.py -f GFX_TILES -w 16 -h 16 -o output.h input.png
```

### Parameters

- `-f GFX_TILES` - Specifies the GFX_TILES output format
- `-w WIDTH` - Tile width in pixels (default: 16)
- `-h HEIGHT` - Tile height in pixels (default: 16)
- `-o OUTPUT` - Output header file path
- `input` - Input image file (PNG/JPEG)

## Generated Output Format

The tool generates a C header file with:

1. **Header Guard** - Standard include guard
2. **Includes** - Required headers (stdint.h, gfx.h)
3. **C16 Macro** - RGB565 color conversion macro
4. **Tilesheet Array** - Constant uint16_t array containing all tiles
5. **Tile Count** - Constant with the number of tiles

### Example Output

```c
#ifndef _MYTILES_H_
#define _MYTILES_H_

#include <stdint.h>
#include "gfx.h"

/*
   ======================================================================
   TILESET (8 tiles, 16x16 RGB565)
   ----------------------------------------------------------------------
   Generated from mytiles.png
   ======================================================================
*/

#define C16(r,g,b)  (((r&0x1F)<<11)|((g&0x3F)<<5)|((b&0x1F)<<0))

const uint16_t mytiles_tilesheet[] = {

    /* === Tile 0 === */
    C16(28,50,63),C16(28,50,63),...,
    ...

    /* === Tile 1 === */
    C16(8,50,8),C16(8,50,8),...,
    ...
};

const uint16_t mytiles_tilesheet_count = 8;

#endif
```

## Image Requirements

- **Dimensions**: Image width/height should be multiples of tile size
- **Format**: PNG or JPEG with RGB/RGBA channels
- **Color Space**: RGB (converted to RGB565 automatically)

## Tile Layout

Tiles are extracted from the image in row-major order:
```
[Tile 0][Tile 1][Tile 2]...
[Tile N][Tile N+1]...
```

For example, a 64x32 image with 16x16 tiles produces:
- 4 tiles horizontally (64/16)
- 2 tiles vertically (32/16)
- Total: 8 tiles (indexed 0-7)

## Integration with PicoCalc

### 1. Include the Generated Header

```c
#include "mytiles.h"
```

### 2. Initialize Graphics System

```c
gfx_init(mytiles_tilesheet, mytiles_tilesheet_count);
```

### 3. Use Tiles in Your Code

```c
// Set tile at position (x=5, y=3) to tile index 2
gfx_set_tile(5, 3, 2);

// Clear entire screen to tile 0
gfx_clear_backmap(0);

// Present changes to LCD
gfx_present();
```

## RGB565 Color Format

The tool automatically converts images to RGB565 format:
- **Red**: 5 bits (0-31)
- **Green**: 6 bits (0-63)
- **Blue**: 5 bits (0-31)

Colors are packed as: `RRRRR GGGGGG BBBBB`

The C16(r,g,b) macro allows easy color manipulation in the generated code.

## Example Workflow

### Create Tileset Image

1. Create a PNG image with your tiles arranged in a grid
2. Each tile should be exactly WxH pixels
3. Example: 8 tiles of 16x16 = 128x16 or 64x32 or 32x64 image

### Generate Header File

```bash
python3 img2c_array_5.py -f GFX_TILES -w 16 -h 16 -o tiles.h mytileset.png
```

### Use in Your Project

```c
#include "gfx.h"
#include "tiles.h"

void setup_scene(void) {
    gfx_init(mytileset_tilesheet, mytileset_tilesheet_count);

    // Create a simple scene
    for (int x = 0; x < 20; x++) {
        gfx_set_tile(x, 19, 1);  // Ground tile
    }

    gfx_set_tile(5, 15, 2);  // Platform tile

    gfx_present();
}
```

## Compatibility

This format is fully compatible with:
- `gfx.h` / `gfx.c` - PicoCalc graphics system
- `tiles_sprites.h` - Example tileset format
- PicoCalc LCD display (320x320 RGB565)

## Notes

- Alpha channel is ignored (RGB565 has no alpha)
- For sprites with transparency, use `GFX_TRANSPARENT_COLOR` (0xFFFF)
- Maximum recommended tiles: limited by available RAM
- Tile size should match `GFX_TILE_W` and `GFX_TILE_H` (typically 16x16)
