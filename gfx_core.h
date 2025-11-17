#pragma once

#include <stdint.h>
#include <stdbool.h>

// Graphics commands that can be sent to core 1
typedef enum {
    GFX_CMD_INIT,           // Initialize graphics on core 1
    GFX_CMD_PRESENT,        // Present/render the current frame
    GFX_CMD_SET_TILE,       // Set a tile in the tilemap
    GFX_CMD_CLEAR,          // Clear the tilemap
    GFX_CMD_CREATE_SPRITE,  // Create a sprite
    GFX_CMD_MOVE_SPRITE,    // Move a sprite
    GFX_CMD_DESTROY_SPRITE, // Destroy a sprite
    GFX_CMD_DRAW_SPRITE,    // Draw sprites
    GFX_CMD_START_RENDERING,// Start continuous rendering
    GFX_CMD_STOP_RENDERING, // Stop continuous rendering
    GFX_CMD_SHUTDOWN,       // Shutdown graphics core
} gfx_cmd_type_t;

// Graphics command structure
typedef struct {
    gfx_cmd_type_t type;
    union {
        struct {
            const uint16_t *tilesheet;
            uint16_t tiles_count;
        } init;
        struct {
            uint16_t x, y;
            uint16_t tile_index;
        } set_tile;
        struct {
            uint16_t bg_tile;
        } clear;
        struct {
            const uint16_t *image;
            uint8_t w, h;
            int16_t x, y;
            uint8_t z;
            int *result_id;  // pointer to store sprite ID result
        } create_sprite;
        struct {
            int sprite_id;
            int16_t x, y;
        } move_sprite;
        struct {
            int sprite_id;
        } destroy_sprite;
    } data;
} gfx_command_t;

// Initialize the graphics core system
void gfx_core_init(void);

// Send a command to the graphics core (blocking - waits for completion)
bool gfx_core_send_command(const gfx_command_t *cmd);

// Wait for graphics core to finish processing (blocking)
void gfx_core_wait_idle(void);

// Check if graphics core is busy
bool gfx_core_is_busy(void);

// High-level API wrappers (call these instead of gfx_* functions)
void gfx_core_gfx_init(const uint16_t *tilesheet_ptr, uint16_t tiles_count);
void gfx_core_gfx_present(void);
void gfx_core_gfx_set_tile(uint16_t x, uint16_t y, uint16_t tile_index);
void gfx_core_gfx_clear_backmap(uint16_t bg_tile);
int gfx_core_gfx_create_sprite(const uint16_t *image, uint8_t w, uint8_t h, int16_t x, int16_t y, uint8_t z);
void gfx_core_gfx_move_sprite(int sprite_id, int16_t x, int16_t y);
void gfx_core_gfx_destroy_sprite(int sprite_id);
void gfx_core_start_rendering(void);
void gfx_core_stop_rendering(void);
