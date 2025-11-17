#include "gfx.h"
#include "drivers/lcd.h"
#include <string.h>
#include <stdlib.h>
#include "pico/time.h"

/* Graphics system with single persistent framebuffer
   - tilesheet_ptr: pointer to tile images (16x16 pixels each, RGB565)
   - tiles_count: number of tiles in tilesheet
   - framebuffer: single persistent buffer holding the complete screen image (WIDTH x HEIGHT pixels)
   - tilemap: tracks which tiles are where
   - Sprites are composited directly into framebuffer

   Key principle: framebuffer is created once and reused.
   - On init: framebuffer is created and filled with tiles
   - On sprite move: erase old sprite position (redraw tiles), draw new sprite position, send to display
   - On tile change: redraw affected tiles into framebuffer
   - Full redraw only when explicitly requested
*/

#define GFX_BACKGROUND_COLOR 0x0000  // black

static const uint16_t background = GFX_BACKGROUND_COLOR;
static const uint16_t *tilesheet = NULL;
static uint16_t tiles_count = 0;

/* Single persistent framebuffer - holds complete screen image */
static uint16_t framebuffer[WIDTH * HEIGHT] __attribute__((aligned(4)));

/* Tilemap - tracks current tile layout */
static uint16_t tilemap[GFX_TILEMAP_SIZE];

/* Framebuffer needs full rebuild */
static bool framebuffer_dirty = true;

/* VBlank synchronization */
#define VBLANK_PERIOD_US 16667  // 60Hz = ~16.67ms per frame
static absolute_time_t next_vblank_time;
static bool vblank_sync_enabled = true;

/* Sprite storage */
static gfx_sprite_info_t sprites[GFX_MAX_SPRITES];

/* Helper: index in tilemap from tx,ty */
static inline uint32_t _tile_index(uint16_t tx, uint16_t ty) {
    return (uint32_t)ty * GFX_TILES_X + tx;
}

/* Helper: framebuffer index from screen x,y */
static inline uint32_t _fb_index(uint16_t x, uint16_t y) {
    return (uint32_t)y * WIDTH + x;
}

/* Clip helper */
static inline int16_t _clamp_i16(int16_t v, int16_t a, int16_t b) {
    if (v < a) return a;
    if (v > b) return b;
    return v;
}

/* Draw a tile into the framebuffer at pixel position (screen_x, screen_y) */
static void _draw_tile_to_framebuffer(uint16_t tile_index, uint16_t screen_x, uint16_t screen_y) {
    /* Draw tile pixels into framebuffer */
    if (tile_index == UINT16_MAX) {
        /* Blank tile - fill with background color */
        for (uint16_t y = 0; y < GFX_TILE_H; y++) {
            for (uint16_t x = 0; x < GFX_TILE_W; x++) {
                uint16_t fx = screen_x + x;
                uint16_t fy = screen_y + y;
                if (fx < WIDTH && fy < HEIGHT) {
                    framebuffer[_fb_index(fx, fy)] = background;
                }
            }
        }
        return;
    }

    if (!tilesheet || tile_index >= tiles_count) {
        /* Invalid tile - fill with background */
        for (uint16_t y = 0; y < GFX_TILE_H; y++) {
            for (uint16_t x = 0; x < GFX_TILE_W; x++) {
                uint16_t fx = screen_x + x;
                uint16_t fy = screen_y + y;
                if (fx < WIDTH && fy < HEIGHT) {
                    framebuffer[_fb_index(fx, fy)] = background;
                }
            }
        }
        return;
    }

    /* Copy tile pixels to framebuffer */
    const uint16_t *tile_pixels = tilesheet + (size_t)tile_index * (GFX_TILE_W * GFX_TILE_H);
    for (uint16_t y = 0; y < GFX_TILE_H; y++) {
        for (uint16_t x = 0; x < GFX_TILE_W; x++) {
            uint16_t fx = screen_x + x;
            uint16_t fy = screen_y + y;
            if (fx < WIDTH && fy < HEIGHT) {
                framebuffer[_fb_index(fx, fy)] = tile_pixels[y * GFX_TILE_W + x];
            }
        }
    }
}

/* Erase sprite from framebuffer by redrawing tiles underneath */
static void _erase_sprite_from_framebuffer(int16_t x, int16_t y, uint8_t w, uint8_t h) {
    if (w == 0 || h == 0) return;

    /* Calculate which tiles are affected */
    int16_t tile_x_start = x / GFX_TILE_W;
    int16_t tile_y_start = y / GFX_TILE_H;
    int16_t tile_x_end = (x + w - 1) / GFX_TILE_W;
    int16_t tile_y_end = (y + h - 1) / GFX_TILE_H;

    /* Clamp to valid tile range */
    if (tile_x_start < 0) tile_x_start = 0;
    if (tile_y_start < 0) tile_y_start = 0;
    if (tile_x_end >= GFX_TILES_X) tile_x_end = GFX_TILES_X - 1;
    if (tile_y_end >= GFX_TILES_Y) tile_y_end = GFX_TILES_Y - 1;

    /* Redraw affected tiles into framebuffer */
    for (int16_t ty = tile_y_start; ty <= tile_y_end; ty++) {
        for (int16_t tx = tile_x_start; tx <= tile_x_end; tx++) {
            uint32_t idx = _tile_index(tx, ty);
            uint16_t tile_idx = tilemap[idx];
            uint16_t sx = tx * GFX_TILE_W;
            uint16_t sy = ty * GFX_TILE_H;
            _draw_tile_to_framebuffer(tile_idx, sx, sy);
        }
    }
}

/* Draw sprite into framebuffer with transparency */
static void _draw_sprite_to_framebuffer(const gfx_sprite_info_t *s) {
    if (!s->active || !s->image || s->w == 0 || s->h == 0) return;

    /* Draw sprite pixels into framebuffer */
    for (uint16_t yy = 0; yy < s->h; yy++) {
        for (uint16_t xx = 0; xx < s->w; xx++) {
            int scr_x = s->x + xx;
            int scr_y = s->y + yy;

            /* Clip to screen bounds */
            if (scr_x < 0 || scr_y < 0 || scr_x >= (int)WIDTH || scr_y >= (int)HEIGHT) {
                continue;
            }

            uint32_t sprite_idx = (uint32_t)yy * s->w + xx;
            uint16_t pixel = s->image[sprite_idx];

            /* Only draw non-transparent pixels */
            if (pixel != GFX_TRANSPARENT_COLOR) {
                framebuffer[_fb_index(scr_x, scr_y)] = pixel;
            }
        }
    }
}

/* Rebuild entire framebuffer from tiles */
static void _rebuild_framebuffer(void) {
    /* Draw all tiles into framebuffer */
    for (uint16_t ty = 0; ty < GFX_TILES_Y; ty++) {
        for (uint16_t tx = 0; tx < GFX_TILES_X; tx++) {
            uint32_t idx = _tile_index(tx, ty);
            uint16_t tile_idx = tilemap[idx];
            uint16_t sx = tx * GFX_TILE_W;
            uint16_t sy = ty * GFX_TILE_H;
            _draw_tile_to_framebuffer(tile_idx, sx, sy);
        }
    }

    framebuffer_dirty = false;
}

/* Initialize gfx */
void gfx_init(const uint16_t *tilesheet_ptr, uint16_t tcount) {
    tilesheet = tilesheet_ptr;
    tiles_count = tcount;

    /* Clear framebuffer - essential to remove any previous LCD content */
    memset(framebuffer, 0, sizeof(framebuffer));

    /* Init tilemap to "blank" sentinel (UINT16_MAX) */
    for (uint32_t i = 0; i < GFX_TILEMAP_SIZE; i++) {
        tilemap[i] = UINT16_MAX;
    }

    /* Clear sprites */
    for (int i = 0; i < GFX_MAX_SPRITES; i++) {
        sprites[i].active = false;
        sprites[i].image = NULL;
        sprites[i].has_prev = false;
    }

    /* Initialize vblank timer */
    next_vblank_time = make_timeout_time_us(VBLANK_PERIOD_US);

    /* Mark framebuffer as needing rebuild */
    framebuffer_dirty = true;
}

/* Set / replace tilesheet pointer */
void gfx_set_tilesheet(const uint16_t *tilesheet_ptr, uint16_t tcount) {
    tilesheet = tilesheet_ptr;
    tiles_count = tcount;
    framebuffer_dirty = true;
}

/* Set tile at tilemap coordinates */
void gfx_set_tile(uint16_t tx, uint16_t ty, uint16_t tile_index) {
    if (tx >= GFX_TILES_X || ty >= GFX_TILES_Y) return;

    uint32_t idx = _tile_index(tx, ty);
    if (tilemap[idx] != tile_index) {
        tilemap[idx] = tile_index;

        /* Redraw this tile into framebuffer immediately if framebuffer is valid */
        if (!framebuffer_dirty) {
            uint16_t sx = tx * GFX_TILE_W;
            uint16_t sy = ty * GFX_TILE_H;
            _draw_tile_to_framebuffer(tile_index, sx, sy);
        }
    }
}

/* Get current tile index at tx,ty */
uint16_t gfx_get_tile(uint16_t tx, uint16_t ty) {
    if (tx >= GFX_TILES_X || ty >= GFX_TILES_Y) return UINT16_MAX;
    return tilemap[_tile_index(tx, ty)];
}

/* Clear the tilemap to given tile_index */
void gfx_clear_backmap(uint16_t tile_index) {
    for (uint32_t i = 0; i < GFX_TILEMAP_SIZE; i++) {
        tilemap[i] = tile_index;
    }
    framebuffer_dirty = true;
}

/* Mark framebuffer for full rebuild */
void gfx_mark_all_dirty(void) {
    framebuffer_dirty = true;
}

/* Force draw single tile (updates framebuffer and sends to display) */
void gfx_force_draw_tile(uint16_t tx, uint16_t ty) {
    if (tx >= GFX_TILES_X || ty >= GFX_TILES_Y) return;

    uint16_t tile_index = tilemap[_tile_index(tx, ty)];
    uint16_t screen_x = tx * GFX_TILE_W;
    uint16_t screen_y = ty * GFX_TILE_H;

    /* Update framebuffer */
    _draw_tile_to_framebuffer(tile_index, screen_x, screen_y);

    /* Send to display */
    lcd_blit(&framebuffer[_fb_index(screen_x, screen_y)], screen_x, screen_y, GFX_TILE_W, GFX_TILE_H);
}

/* Present: update framebuffer and send to display */
void gfx_present(void) {
    /* VBlank synchronization: wait until next vblank period 
    if (vblank_sync_enabled) {
        sleep_until(next_vblank_time);
        next_vblank_time = delayed_by_us(next_vblank_time, VBLANK_PERIOD_US);
    }
    */

    /* Rebuild framebuffer if needed (full redraw) */
    if (framebuffer_dirty) {
        _rebuild_framebuffer();
    }

    /* Erase previous sprite positions by redrawing tiles */
    for (int i = 0; i < GFX_MAX_SPRITES; i++) {
        if (sprites[i].active && sprites[i].has_prev) {
            _erase_sprite_from_framebuffer(sprites[i].prev_x, sprites[i].prev_y, sprites[i].w, sprites[i].h);
        }
    }

    /* Sort sprites by z-order */
    int active_ids[GFX_MAX_SPRITES];
    int active_count = 0;
    for (int i = 0; i < GFX_MAX_SPRITES; i++) {
        if (sprites[i].active && sprites[i].image && sprites[i].w > 0 && sprites[i].h > 0) {
            active_ids[active_count++] = i;
        }
    }

    /* Simple insertion sort by z */
    for (int i = 1; i < active_count; i++) {
        int key = active_ids[i];
        int j = i - 1;
        while (j >= 0 && sprites[active_ids[j]].z > sprites[key].z) {
            active_ids[j + 1] = active_ids[j];
            j--;
        }
        active_ids[j + 1] = key;
    }

    /* Draw sprites into framebuffer */
    for (int idx_i = 0; idx_i < active_count; idx_i++) {
        int si = active_ids[idx_i];
        gfx_sprite_info_t *s = &sprites[si];

        /* Skip if entirely off-screen */
        if (s->x + s->w <= 0 || s->y + s->h <= 0 || s->x >= (int)WIDTH || s->y >= (int)HEIGHT) {
            continue;
        }

        /* Draw sprite to framebuffer */
        _draw_sprite_to_framebuffer(s);

        /* Save position for next frame */
        s->prev_x = s->x;
        s->prev_y = s->y;
        s->has_prev = true;
    }

    /* Send entire framebuffer to display */
    lcd_blit(framebuffer, 0, 0, WIDTH, HEIGHT);
}

/* Sprite API implementations */
gfx_sprite_t gfx_create_sprite(const uint16_t *image, uint8_t w, uint8_t h, int16_t x, int16_t y, uint8_t z) {
    for (int i = 0; i < GFX_MAX_SPRITES; i++) {
        if (!sprites[i].active) {
            sprites[i].active = true;
            sprites[i].image = image;
            sprites[i].w = w;
            sprites[i].h = h;
            sprites[i].x = x;
            sprites[i].y = y;
            sprites[i].z = z;
            sprites[i].has_prev = false;
            return i;
        }
    }
    return -1;
}

bool gfx_destroy_sprite(gfx_sprite_t id) {
    if (id < 0 || id >= GFX_MAX_SPRITES) return false;

    /* Erase sprite from framebuffer before destroying */
    if (sprites[id].active && sprites[id].has_prev) {
        _erase_sprite_from_framebuffer(sprites[id].prev_x, sprites[id].prev_y, sprites[id].w, sprites[id].h);
    }

    sprites[id].active = false;
    sprites[id].image = NULL;
    sprites[id].has_prev = false;
    return true;
}

bool gfx_move_sprite(gfx_sprite_t id, int16_t x, int16_t y) {
    if (id < 0 || id >= GFX_MAX_SPRITES) return false;
    if (!sprites[id].active) return false;
    sprites[id].x = x;
    sprites[id].y = y;
    return true;
}

bool gfx_set_sprite_z(gfx_sprite_t id, uint8_t z) {
    if (id < 0 || id >= GFX_MAX_SPRITES) return false;
    if (!sprites[id].active) return false;
    sprites[id].z = z;
    return true;
}

bool gfx_set_sprite_image(gfx_sprite_t id, const uint16_t *image, uint8_t w, uint8_t h) {
    if (id < 0 || id >= GFX_MAX_SPRITES) return false;
    if (!sprites[id].active) return false;
    sprites[id].image = image;
    sprites[id].w = w;
    sprites[id].h = h;
    return true;
}

/* Fill rectangle in tile units */
void gfx_fill_tiles_rect(uint16_t tx, uint16_t ty, uint16_t tw, uint16_t th, uint16_t tile_index) {
    for (uint16_t yy = ty; yy < ty + th && yy < GFX_TILES_Y; yy++) {
        for (uint16_t xx = tx; xx < tx + tw && xx < GFX_TILES_X; xx++) {
            gfx_set_tile(xx, yy, tile_index);
        }
    }
}

/* VBlank synchronization control */
void gfx_set_vblank_sync(bool enabled) {
    vblank_sync_enabled = enabled;
    if (enabled) {
        next_vblank_time = make_timeout_time_us(VBLANK_PERIOD_US);
    }
}

bool gfx_get_vblank_sync(void) {
    return vblank_sync_enabled;
}
