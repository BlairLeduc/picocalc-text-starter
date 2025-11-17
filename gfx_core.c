//
// Graphics Core - Multicore graphics rendering on Core 1
//
// This module runs all graphics operations on Core 1 with continuous rendering.
// Core 1 renders at a fixed frame rate while Core 0 sends commands asynchronously.
//

#include "gfx_core.h"
#include "gfx.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/time.h"
#include "pico/util/queue.h"
#include <string.h>

// Rendering control
static volatile bool gfx_core_running = false;
static volatile bool gfx_core_rendering_enabled = false;
static volatile bool gfx_core_initialized = false;
static mutex_t gfx_mutex;

// Command queue (safer than FIFO, with larger buffer)
#define CMD_QUEUE_SIZE 16
static queue_t cmd_queue;

// Frame timing (60 FPS)
#define FRAME_TIME_US 16667  // ~60Hz

// Core 1 main loop - continuous rendering
static void gfx_core1_main(void) {
    gfx_core_running = true;
    absolute_time_t next_frame_time = get_absolute_time();

    while (gfx_core_running) {
        // Process any pending commands from queue (non-blocking)
        gfx_command_t cmd;
        while (queue_try_remove(&cmd_queue, &cmd)) {

            // Execute command on core 1
            switch (cmd.type) {
                case GFX_CMD_INIT:
                    gfx_init(cmd.data.init.tilesheet, cmd.data.init.tiles_count);
                    // Signal initialization complete
                    gfx_core_initialized = true;
                    // Don't enable rendering yet - wait for scene setup
                    break;

                case GFX_CMD_START_RENDERING:
                    gfx_core_rendering_enabled = true;
                    break;

                case GFX_CMD_STOP_RENDERING:
                    gfx_core_rendering_enabled = false;
                    break;

                case GFX_CMD_SET_TILE:
                    gfx_set_tile(cmd.data.set_tile.x,
                                cmd.data.set_tile.y,
                                cmd.data.set_tile.tile_index);
                    break;

                case GFX_CMD_CLEAR:
                    gfx_clear_backmap(cmd.data.clear.bg_tile);
                    break;

                case GFX_CMD_CREATE_SPRITE: {
                    int sprite_id = gfx_create_sprite(
                        cmd.data.create_sprite.image,
                        cmd.data.create_sprite.w,
                        cmd.data.create_sprite.h,
                        cmd.data.create_sprite.x,
                        cmd.data.create_sprite.y,
                        cmd.data.create_sprite.z
                    );
                    // Store result if pointer provided
                    if (cmd.data.create_sprite.result_id) {
                        *cmd.data.create_sprite.result_id = sprite_id;
                    }
                    break;
                }

                case GFX_CMD_MOVE_SPRITE:
                    gfx_move_sprite(cmd.data.move_sprite.sprite_id,
                                   cmd.data.move_sprite.x,
                                   cmd.data.move_sprite.y);
                    break;

                case GFX_CMD_DESTROY_SPRITE:
                    gfx_destroy_sprite(cmd.data.destroy_sprite.sprite_id);
                    // Don't disable rendering here - use STOP_RENDERING command explicitly
                    break;

                case GFX_CMD_SHUTDOWN:
                    gfx_core_running = false;
                    return;

                default:
                    break;
            }
        }

        // Continuous rendering loop (60 FPS)
        if (gfx_core_rendering_enabled) {
            // Wait for next frame time
            sleep_until(next_frame_time);
            next_frame_time = delayed_by_us(next_frame_time, FRAME_TIME_US);

            // Render frame
            gfx_present();
        } else {
            // If rendering disabled, yield CPU briefly to allow command processing
            tight_loop_contents();
        }
    }
}

// Initialize the graphics core system
void gfx_core_init(void) {
    mutex_init(&gfx_mutex);
    gfx_core_initialized = false;

    // Initialize command queue (thread-safe, larger buffer than FIFO)
    queue_init(&cmd_queue, sizeof(gfx_command_t), CMD_QUEUE_SIZE);

    // Launch core 1 with graphics loop
    multicore_launch_core1(gfx_core1_main);

    // Wait a bit for core 1 to start
    sleep_ms(10);
}

// Send a command to the graphics core (uses queue - thread-safe)
bool gfx_core_send_command(const gfx_command_t *cmd) {
    if (!gfx_core_running) {
        return false;
    }

    // Add command to queue (thread-safe, no need for manual mutex)
    // For movement commands, use try_add to avoid blocking
    // With throttling on Core 0 side (60 FPS), queue should never fill
    if (cmd->type == GFX_CMD_MOVE_SPRITE) {
        // Non-blocking for movement
        // With proper throttling, this should always succeed
        if (!queue_try_add(&cmd_queue, cmd)) {
            // Queue full - this indicates Core 1 is overloaded
            // Skip this update, next one will catch up
            return false;
        }
        return true;
    } else {
        // Blocking for critical commands (must not be lost)
        queue_add_blocking(&cmd_queue, cmd);
        return true;
    }
}

//
// High-level API wrappers
//

void gfx_core_gfx_init(const uint16_t *tilesheet_ptr, uint16_t tiles_count) {
    gfx_core_initialized = false;

    gfx_command_t cmd = {
        .type = GFX_CMD_INIT,
        .data.init = {
            .tilesheet = tilesheet_ptr,
            .tiles_count = tiles_count
        }
    };
    gfx_core_send_command(&cmd);

    // Wait for Core 1 to complete initialization (critical for proper setup)
    int timeout = 0;
    while (!gfx_core_initialized && timeout < 1000) {
        sleep_us(100); // Wait 100us at a time
        timeout++;
    }
}

void gfx_core_gfx_present(void) {
    // With continuous rendering, present is automatic - this is now a no-op
    // The Core 1 renders continuously at 60 FPS
}

void gfx_core_gfx_set_tile(uint16_t x, uint16_t y, uint16_t tile_index) {
    gfx_command_t cmd = {
        .type = GFX_CMD_SET_TILE,
        .data.set_tile = {
            .x = x,
            .y = y,
            .tile_index = tile_index
        }
    };
    gfx_core_send_command(&cmd);
}

void gfx_core_gfx_clear_backmap(uint16_t bg_tile) {
    gfx_command_t cmd = {
        .type = GFX_CMD_CLEAR,
        .data.clear = {
            .bg_tile = bg_tile
        }
    };
    gfx_core_send_command(&cmd);
}

int gfx_core_gfx_create_sprite(const uint16_t *image, uint8_t w, uint8_t h, int16_t x, int16_t y, uint8_t z) {
    volatile int sprite_id = -1;
    gfx_command_t cmd = {
        .type = GFX_CMD_CREATE_SPRITE,
        .data.create_sprite = {
            .image = image,
            .w = w,
            .h = h,
            .x = x,
            .y = y,
            .z = z,
            .result_id = (int*)&sprite_id
        }
    };
    gfx_core_send_command(&cmd);

    // Wait for Core 1 to write the sprite ID (should happen within one frame)
    int timeout = 0;
    while (sprite_id == -1 && timeout < 100) {
        sleep_us(100); // Wait 100us at a time
        timeout++;
    }

    return sprite_id;
}

void gfx_core_gfx_move_sprite(int sprite_id, int16_t x, int16_t y) {
    gfx_command_t cmd = {
        .type = GFX_CMD_MOVE_SPRITE,
        .data.move_sprite = {
            .sprite_id = sprite_id,
            .x = x,
            .y = y
        }
    };
    gfx_core_send_command(&cmd);
}

void gfx_core_gfx_destroy_sprite(int sprite_id) {
    gfx_command_t cmd = {
        .type = GFX_CMD_DESTROY_SPRITE,
        .data.destroy_sprite = {
            .sprite_id = sprite_id
        }
    };
    gfx_core_send_command(&cmd);
}

void gfx_core_start_rendering(void) {
    gfx_command_t cmd = {
        .type = GFX_CMD_START_RENDERING
    };
    gfx_core_send_command(&cmd);
}

void gfx_core_stop_rendering(void) {
    gfx_command_t cmd = {
        .type = GFX_CMD_STOP_RENDERING
    };
    gfx_core_send_command(&cmd);

    // Wait until Core 1 has actually stopped rendering
    // This is important to ensure sprite destruction happens cleanly
    int timeout = 0;
    while (gfx_core_rendering_enabled && timeout < 1000) {
        sleep_us(100); // Wait 100us at a time
        timeout++;
    }
}
