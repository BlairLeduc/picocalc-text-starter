#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "drivers/lcd.h"

/*
  Graphics module for PicoCalc (Pico 2W)
  - Tileset: tiles are 16x16 pixels, RGB565 (uint16_t per pixel)
  - Screen: WIDTH x HEIGHT (from lcd.h), assumed divisible by 16
  - Double buffering at tilemap level: two tile-index maps are kept and compared on present()
  - Sprites: arbitrary w x h (<=16x16 recommended), use a transparent colour value
*/

/* Tile dimensions */
#define GFX_TILE_W 16
#define GFX_TILE_H 16

/* Derived */
#define GFX_TILES_X (WIDTH / GFX_TILE_W)
#define GFX_TILES_Y (HEIGHT / GFX_TILE_H)
#define GFX_TILEMAP_SIZE (GFX_TILES_X * GFX_TILES_Y)

/* Transparent colour (RGB565) used in sprite images */
#ifndef GFX_TRANSPARENT_COLOR
#define GFX_TRANSPARENT_COLOR 0xFFFF  /* default: white as transparent (can be overridden) */
#endif

/* Max number of tiles in the tilesheet (user can change before compilation) */
#ifndef GFX_MAX_TILES
#define GFX_MAX_TILES 256
#endif

/* Max number of sprites */
#ifndef GFX_MAX_SPRITES
#define GFX_MAX_SPRITES 16
#endif

/* Sprite handle type */
typedef int gfx_sprite_t; /* -1 == invalid */

/* Sprite structure (client fills image pointer in RGB565) */
typedef struct {
    bool active;
    int16_t x;    /* screen coords (can be negative for partial off-screen) */
    int16_t y;
    uint8_t w;    /* width in pixels */
    uint8_t h;    /* height in pixels */
    const uint16_t *image; /* pointer to w*h RGB565 pixel data; use GFX_TRANSPARENT_COLOR for transparent pixels */
    uint8_t z;    /* z-order (draw order) - lower drawn first */
    /* Previous position for erasing old sprite position */
    int16_t prev_x;
    int16_t prev_y;
    bool has_prev; /* true if we need to erase previous position */
} gfx_sprite_info_t;

/* Initialize gfx module
   - tilesheet_ptr may be NULL; use gfx_set_tilesheet() later
   - tiles_count number of tiles in tilesheet_ptr
*/
void gfx_init(const uint16_t *tilesheet_ptr, uint16_t tiles_count);

/* Set / replace tilesheet pointer (tiles are contiguous: tile0 16x16 pixels, tile1, ...) */
void gfx_set_tilesheet(const uint16_t *tilesheet_ptr, uint16_t tiles_count);

/* Set tile at tilemap coordinates tx,ty to tile index (0..tiles_count-1).
   If tile_index == UINT16_MAX => treat as "blank" (draw background colour).
*/
void gfx_set_tile(uint16_t tx, uint16_t ty, uint16_t tile_index);

/* Get current tile index at tx,ty (from back buffer / next frame buffer) */
uint16_t gfx_get_tile(uint16_t tx, uint16_t ty);

/* Clear the back tilemap to given tile_index (e.g. 0 or UINT16_MAX) */
void gfx_clear_backmap(uint16_t tile_index);

/* Mark full backmap dirty (force redraw on next present) */
void gfx_mark_all_dirty(void);

/* Swap buffers and present to LCD: only changed tiles are blitted; then sprites are composited and blitted.
   This implements the double-buffering behaviour.
*/
void gfx_present(void);

/* Sprites API */
gfx_sprite_t gfx_create_sprite(const uint16_t *image, uint8_t w, uint8_t h, int16_t x, int16_t y, uint8_t z);
bool gfx_destroy_sprite(gfx_sprite_t id);
bool gfx_move_sprite(gfx_sprite_t id, int16_t x, int16_t y);
bool gfx_set_sprite_z(gfx_sprite_t id, uint8_t z);
bool gfx_set_sprite_image(gfx_sprite_t id, const uint16_t *image, uint8_t w, uint8_t h);

/* Update single tile on screen immediately (bypass double-buffer tilemap compare) */
void gfx_force_draw_tile(uint16_t tx, uint16_t ty);

/* Convenience: draw a solid rectangle (colour RGB565) in tilemap units (tile coords) */
void gfx_fill_tiles_rect(uint16_t tx, uint16_t ty, uint16_t tw, uint16_t th, uint16_t tile_index);

/* VBlank synchronization control */
void gfx_set_vblank_sync(bool enabled);
bool gfx_get_vblank_sync(void);

/* Get constants */
static inline uint16_t gfx_tiles_x(void) { return GFX_TILES_X; }
static inline uint16_t gfx_tiles_y(void) { return GFX_TILES_Y; }

