#pragma once

#include <stdint.h>
#include "gfx.h"

/*
   ======================================================================
   TILESET (8 tiles, 16x16 RGB565)
   ----------------------------------------------------------------------
   Tile 0: blue sky with gradient
   Tile 1: grass with spots
   Tile 2: brown dirt
   Tile 3: red bricks
   Tile 4: gray floor (tiles)
   Tile 5: stone wall (irregular)
   Tile 6: water (light/dark blue waves)
   Tile 7: light yellow sand
   ======================================================================
*/

#define C16(r,g,b)  (((r&0x1F)<<11)|((g&0x3F)<<5)|((b&0x1F)<<0))

#define SKY_LIGHT  C16(28,50,63)
#define SKY_DARK   C16(16,35,55)

#define GRASS_LIGHT C16(8,50,8)
#define GRASS_DARK  C16(4,35,4)

#define DIRT_LIGHT C16(18,25,5)
#define DIRT_DARK  C16(10,15,3)

#define BRICK_RED  C16(31,0,0)
#define BRICK_DARK C16(20,0,0)
#define BRICK_MORT C16(25,25,25)

#define TILE_GRAY1 C16(20,20,20)
#define TILE_GRAY2 C16(25,25,25)

#define STONE_DARK C16(10,10,10)
#define STONE_LIGHT C16(22,22,22)

#define WATER_DARK C16(0,0,20)
#define WATER_LIGHT C16(0,30,45)

#define SAND_LIGHT C16(28,28,8)
#define SAND_DARK  C16(24,24,5)

/*
   ======================================================================
   SPRITE (16x16)
   ----------------------------------------------------------------------
   "Hero" symbol: red body with black border and blue visor
   transparent = white (GFX_TRANSPARENT_COLOR)
   ======================================================================
*/

#define T GFX_TRANSPARENT_COLOR
#define K C16(0,0,0)     /* black outline */
#define R C16(31,0,0)    /* red body */
#define B C16(0,0,31)    /* blue visor */
#define Y C16(31,31,0)   /* yellow decoration */

const uint16_t sprite1_pixels[16 * 16] = {
    T,T,T,K,K,K,K,K,K,K,K,K,T,T,T,T,
    T,K,K,R,R,R,R,R,R,R,R,R,K,K,T,T,
    K,R,R,R,R,R,R,R,R,R,R,R,R,R,K,T,
    K,R,R,Y,Y,R,R,R,R,R,Y,Y,R,R,K,T,
    K,R,R,R,R,R,R,R,R,R,R,R,R,R,K,T,
    K,R,R,R,R,R,R,R,R,R,R,R,R,R,K,T,
    K,R,R,R,B,B,B,B,B,B,B,B,R,R,K,T,
    K,R,R,R,B,B,B,B,B,B,B,B,R,R,K,T,
    K,R,R,R,R,B,B,B,B,B,B,R,R,R,K,T,
    K,R,R,R,R,R,R,R,R,R,R,R,R,R,K,T,
    K,R,R,R,R,R,R,R,R,R,R,R,R,R,K,T,
    K,R,R,R,R,R,R,R,R,R,R,R,R,R,K,T,
    K,R,R,R,R,R,R,R,R,R,R,R,R,R,K,T,
    K,R,R,R,R,R,R,R,R,R,R,R,R,R,K,T,
    T,K,K,R,R,R,R,R,R,R,R,R,K,K,T,T,
    T,T,T,K,K,K,K,K,K,K,K,K,T,T,T,T
};
