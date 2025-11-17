#!/usr/bin/env python3
"""
img2c_array.py
Convert image into C arrays (uint32_t/uint16_t) in RGBA8888, RGBA2222 or RGB565
Generates a .h header with per-tile arrays or a single array, optional palette init_colors().
Supports AgonLight-specific options: byte order, clamp alpha, 6-bit palette.
Can also generate Picocalc-compatible bitmap fonts from glyph sheets.
"""
import argparse
import os
import math
from collections import defaultdict
from PIL import Image

def to_rgba8888(pixel, byte_order="RGBA", clamp_alpha=False):
    r, g, b, a = pixel
    if clamp_alpha:
        a = 255 if a > 127 else 0
    if byte_order.upper() == "RGBA":
        return (r << 24) | (g << 16) | (b << 8) | a
    elif byte_order.upper() == "ABGR":
        return (a << 24) | (b << 16) | (g << 8) | r
    else:
        raise ValueError("Invalid byte_order")

def to_rgba2222(pixel, byte_order="RGBA", clamp_alpha=False):
    r, g, b, a = pixel
    if clamp_alpha:
        a = 255 if a > 127 else 0
    r2 = (r >> 6) & 0x03
    g2 = (g >> 6) & 0x03
    b2 = (b >> 6) & 0x03
    a2 = (a >> 6) & 0x03
    return (r2 << 6) | (g2 << 4) | (b2 << 2) | a2

def to_rgb565(pixel, byte_order="RGB", clamp_alpha=False):
    """
    Converte un pixel RGBA in formato RGB565 (16 bit).
    R: 5 bit (bits 15-11)
    G: 6 bit (bits 10-5)
    B: 5 bit (bits 4-0)
    
    byte_order può essere "RGB" (big-endian) o "BGR" (little-endian)
    """
    r, g, b, a = pixel
    # Converti da 8 bit a 5/6 bit
    r5 = (r >> 3) & 0x1F  # 5 bit
    g6 = (g >> 2) & 0x3F  # 6 bit
    b5 = (b >> 3) & 0x1F  # 5 bit
    
    if byte_order.upper() == "RGB":
        # Standard RGB565: RRRRRGGGGGGBBBBB
        return (r5 << 11) | (g6 << 5) | b5
    elif byte_order.upper() == "BGR":
        # BGR565: BBBBBGGGGGGRRRRR
        return (b5 << 11) | (g6 << 5) | r5
    else:
        raise ValueError("Invalid byte_order for RGB565 (use RGB or BGR)")

def extract_palette_adaptive(img_rgba, max_colors=None, clamp_alpha=False):
    if max_colors is None:
        seen = {}
        for px in img_rgba.getdata():
            if clamp_alpha:
                px = (px[0], px[1], px[2], 255 if px[3] > 127 else 0)
            seen[px] = seen.get(px, 0) + 1
        return [(r, g, b, a, count) for (r,g,b,a), count in sorted(seen.items(), key=lambda kv: -kv[1])]

    rgb_img = img_rgba.convert("RGB")
    pal_img = rgb_img.convert("P", palette=Image.ADAPTIVE, colors=max_colors)
    palette = pal_img.getpalette()
    counts = pal_img.getcolors(maxcolors=pal_img.width * pal_img.height + 1)
    if counts is None:
        idxs = list(pal_img.getdata())
        freq = defaultdict(int)
        for i in idxs:
            freq[i] += 1
        counts = [(c, idx) for idx, c in freq.items()]

    index_to_rgb = {idx: (palette[idx*3], palette[idx*3+1], palette[idx*3+2]) for idx in range(len(palette)//3)}

    idx_list = list(pal_img.getdata())
    rgba_list = list(img_rgba.getdata())
    alpha_acc = defaultdict(int)
    alpha_count = defaultdict(int)
    for idx, (r,g,b,a) in zip(idx_list, rgba_list):
        if clamp_alpha:
            a = 255 if a > 127 else 0
        alpha_acc[idx] += a
        alpha_count[idx] += 1

    entries = []
    for count, idx in counts:
        rgb = index_to_rgb.get(idx, (0,0,0))
        avg_a = int(round(alpha_acc[idx]/alpha_count[idx])) if alpha_count[idx] else 255
        entries.append((rgb[0], rgb[1], rgb[2], avg_a, count))

    entries.sort(key=lambda e: -e[4])
    return entries[:max_colors]

def glyph_to_bitmap(img, x, y, glyph_width, glyph_height, threshold=128):
    """
    Estrae un singolo glyph dall'immagine e lo converte in bitmap.
    Per larghezze <= 8: ogni byte rappresenta una riga (MSB = pixel più a sinistra)
    Per larghezze > 8: ogni riga usa più bytes (es: 16 bit = 2 bytes per riga)
    
    Returns: list of bytes (uno o più per ogni riga del glyph)
    """
    bitmap = []
    bytes_per_row = (glyph_width + 7) // 8  # Arrotonda per eccesso
    
    for row in range(glyph_height):
        row_bytes = []
        for byte_idx in range(bytes_per_row):
            byte_val = 0
            start_col = byte_idx * 8
            end_col = min(start_col + 8, glyph_width)
            
            for col in range(start_col, end_col):
                px = img.getpixel((x + col, y + row))
                # Converti in grayscale se necessario
                if isinstance(px, tuple):
                    gray = (px[0] + px[1] + px[2]) // 3 if len(px) >= 3 else px[0]
                else:
                    gray = px
                # Pixel acceso se sopra la soglia
                if gray > threshold:
                    bit_position = 7 - (col - start_col)
                    byte_val |= (1 << bit_position)
            row_bytes.append(byte_val)
        bitmap.extend(row_bytes)
    return bitmap

def generate_picocalc_font(img, glyph_width, glyph_height, start_char=32, end_char=127, threshold=128):
    """
    Genera un font in formato Picocalc da un'immagine con glyph disposti in griglia.
    
    Args:
        img: Immagine PIL contenente i glyphs
        glyph_width: Larghezza di ogni glyph in pixel
        glyph_height: Altezza di ogni glyph in pixel (GLYPH_HEIGHT)
        start_char: Primo carattere ASCII da generare (default 32 = space)
        end_char: Ultimo carattere ASCII da generare (default 127)
        threshold: Soglia per considerare un pixel acceso (0-255)
    
    Returns: dictionary con font data
    """
    img_w, img_h = img.size
    glyphs_per_row = img_w // glyph_width
    glyphs_per_col = img_h // glyph_height
    
    num_chars = end_char - start_char + 1
    max_chars_in_image = glyphs_per_row * glyphs_per_col
    
    # Verifica che l'immagine sia abbastanza grande
    if max_chars_in_image < num_chars:
        raise ValueError(
            f"Image too small! Need space for {num_chars} characters, "
            f"but image only fits {max_chars_in_image} glyphs.\n"
            f"Image size: {img_w}x{img_h} pixels\n"
            f"Glyph size: {glyph_width}x{glyph_height} pixels\n"
            f"Glyphs per row: {glyphs_per_row}, rows: {glyphs_per_col}\n"
            f"Required image size (minimum): {glyph_width * glyphs_per_row}x{glyph_height * ((num_chars + glyphs_per_row - 1) // glyphs_per_row)} pixels"
        )
    
    font_data = {
        'width': glyph_width,
        'height': glyph_height,
        'start_char': start_char,
        'end_char': end_char,
        'glyphs': []
    }
    
    print(f"[INFO] Image size: {img_w}x{img_h} pixels")
    print(f"[INFO] Glyph size: {glyph_width}x{glyph_height} pixels")
    print(f"[INFO] Glyphs per row: {glyphs_per_row}, total rows: {glyphs_per_col}")
    print(f"[INFO] Extracting {num_chars} characters (ASCII {start_char}-{end_char})")
    
    for char_idx in range(num_chars):
        # Calcola posizione del glyph nell'immagine
        glyph_x = (char_idx % glyphs_per_row) * glyph_width
        glyph_y = (char_idx // glyphs_per_row) * glyph_height
        
        # Verifica che il glyph sia nei limiti
        if glyph_x + glyph_width > img_w or glyph_y + glyph_height > img_h:
            raise ValueError(
                f"Glyph {char_idx} (ASCII {start_char + char_idx}) out of bounds!\n"
                f"Position: ({glyph_x}, {glyph_y}), "
                f"Size: {glyph_width}x{glyph_height}, "
                f"Image: {img_w}x{img_h}"
            )
        
        # Estrai bitmap del glyph
        bitmap = glyph_to_bitmap(img, glyph_x, glyph_y, glyph_width, glyph_height, threshold)
        font_data['glyphs'].append(bitmap)
    
    return font_data

def write_picocalc_font(f, basename, font_data):
    """Scrive il font in formato Picocalc nel file header."""
    width = font_data['width']
    height = font_data['height']
    start_char = font_data['start_char']
    end_char = font_data['end_char']
    glyphs = font_data['glyphs']
    
    bytes_per_row = (width + 7) // 8
    bytes_per_glyph = bytes_per_row * height
    
    f.write(f"// Font: {width}x{height} pixels, ASCII {start_char}-{end_char}\n")
    f.write(f"// Total characters: {len(glyphs)}\n")
    f.write(f"// Bytes per row: {bytes_per_row}, Bytes per glyph: {bytes_per_glyph}\n\n")
    
    f.write(f"const font_t {basename}_font = {{\n")
    f.write(f"    .width = {width},\n")
    f.write(f"    .glyphs = {{\n")
    
    # Scrivi i bitmap di tutti i glyphs
    for char_idx, bitmap in enumerate(glyphs):
        char_code = start_char + char_idx
        char_repr = chr(char_code) if 32 <= char_code <= 126 else '?'
        f.write(f"        // Character '{char_repr}' (ASCII {char_code})\n")
        
        # Scrivi il bitmap del glyph
        for row_idx in range(height):
            f.write("        ")
            start_idx = row_idx * bytes_per_row
            end_idx = start_idx + bytes_per_row
            row_bytes = bitmap[start_idx:end_idx]
            
            for i, byte_val in enumerate(row_bytes):
                f.write(f"0x{byte_val:02X}")
                # Aggiungi virgola se non è l'ultimo byte del font
                is_last_byte = (char_idx == len(glyphs) - 1 and 
                               row_idx == height - 1 and 
                               i == len(row_bytes) - 1)
                if not is_last_byte:
                    f.write(", ")
            
            # Aggiungi commento visivo per font <= 8 pixel di larghezza
            if bytes_per_row == 1 and row_bytes:
                binary = format(row_bytes[0], '08b').replace('0', '.').replace('1', '#')
                f.write(f"  // {binary[:width]}")
            
            f.write("\n")
    
    f.write("    }\n")
    f.write("};\n\n")
    
    f.write(f"#define {basename.upper()}_FONT_WIDTH {width}\n")
    f.write(f"#define {basename.upper()}_FONT_HEIGHT {height}\n")
    f.write(f"#define {basename.upper()}_FONT_BYTES_PER_ROW {bytes_per_row}\n")
    f.write(f"#define {basename.upper()}_FONT_START_CHAR {start_char}\n")
    f.write(f"#define {basename.upper()}_FONT_END_CHAR {end_char}\n\n")

def main():
    parser = argparse.ArgumentParser(description="Convert image to C RGBA arrays (AgonLight compatible)", add_help=False)
    parser.add_argument("--aspect-ratio", type=float, default=1.0,
                   help="Compensa l'aspect ratio dei pixel (es: 2.67 per AgonLight 2)")
    parser.add_argument("input", help="Input PNG/JPEG image")
    parser.add_argument("-o","--output", help="Output .h file", default="output.h")
    parser.add_argument("-f","--format", choices=["RGBA8888","RGBA2222","RGB565","PICOCALC_FONT","GFX_TILES"], default="RGBA8888")
    parser.add_argument("-w","--width", type=int, default=16)
    parser.add_argument("-h","--height", type=int, default=16)
    parser.add_argument("--single-array", action="store_true")
    parser.add_argument("--with-palette", action="store_true")
    parser.add_argument("--max-colors", type=int, default=None)
    parser.add_argument("--byte-order", default=None,
                        help="Byte order: RGBA/ABGR for RGBA8888/RGBA2222, RGB/BGR for RGB565 (auto-detected if not specified)")
    parser.add_argument("--clamp-alpha", action="store_true")
    parser.add_argument("--initcolor-6bit", action="store_true",
                        help="Scale palette r,g,b to 0..63 for VDP")
    
    # Opzioni specifiche per PICOCALC_FONT
    parser.add_argument("--font-start-char", type=int, default=32,
                        help="[FONT] First ASCII character to generate (default: 32 = space)")
    parser.add_argument("--font-end-char", type=int, default=127,
                        help="[FONT] Last ASCII character to generate (default: 127)")
    parser.add_argument("--font-threshold", type=int, default=128,
                        help="[FONT] Threshold for pixel on/off (0-255, default: 128)")
    
    parser.add_argument("--help", action="help", help="Show this help and exit")

    args = parser.parse_args()
    
    # Auto-detect byte order if not specified
    if args.byte_order is None:
        if args.format == "RGB565":
            args.byte_order = "RGB"
        else:
            args.byte_order = "RGBA"

    img = Image.open(args.input).convert("RGBA")
    basename = os.path.splitext(os.path.basename(args.input))[0]
    header_guard = f"_{basename.upper()}_H_"
    
    # Gestione speciale per PICOCALC_FONT
    if args.format == "PICOCALC_FONT":
        # Converti in grayscale per il font
        img_gray = img.convert('L')

        font_data = generate_picocalc_font(
            img_gray,
            args.width,
            args.height,
            args.font_start_char,
            args.font_end_char,
            args.font_threshold
        )

        with open(args.output, "w") as f:
            f.write(f"#ifndef {header_guard}\n#define {header_guard}\n\n")
            f.write(f"#include <stdint.h>\n")
            f.write(f'#include "font.h"\n\n')
            f.write(f"// Generated from {args.input}\n")
            f.write(f"// Glyph size: {args.width}x{args.height} pixels\n")
            f.write(f"// Threshold: {args.font_threshold}\n\n")

            write_picocalc_font(f, basename, font_data)

            f.write("#endif\n")

        print(f"[OK] Generated Picocalc font: {args.output}")
        print(f"[INFO] Glyph size: {args.width}x{args.height}")
        print(f"[INFO] Characters: {args.font_start_char}-{args.font_end_char} ({args.font_end_char - args.font_start_char + 1} total)")
        print(f"[INFO] Threshold: {args.font_threshold}")
        return

    # Gestione speciale per GFX_TILES (compatibile con tiles_sprites.h)
    if args.format == "GFX_TILES":
        img_w, img_h = img.size
        tiles_x = math.ceil(img_w/args.width)
        tiles_y = math.ceil(img_h/args.height)
        total_tiles = tiles_x * tiles_y

        with open(args.output, "w") as f:
            f.write(f"#ifndef {header_guard}\n#define {header_guard}\n\n")
            f.write(f"#include <stdint.h>\n")
            f.write(f'#include "gfx.h"\n\n')
            f.write(f"/*\n")
            f.write(f"   ======================================================================\n")
            f.write(f"   TILESET ({total_tiles} tiles, {args.width}x{args.height} RGB565)\n")
            f.write(f"   ----------------------------------------------------------------------\n")
            f.write(f"   Generated from {args.input}\n")
            f.write(f"   ======================================================================\n")
            f.write(f"*/\n\n")
            f.write(f"#define C16(r,g,b)  (((r&0x1F)<<11)|((g&0x3F)<<5)|((b&0x1F)<<0))\n\n")
            f.write(f"static const uint16_t {basename}_tilesheet[] = {{\n")

            tile_index = 0
            for ty in range(tiles_y):
                for tx in range(tiles_x):
                    f.write(f"\n    /* === Tile {tile_index} === */\n")

                    # Estrai la tile
                    tile = img.crop((tx*args.width, ty*args.height,
                                    tx*args.width+args.width, ty*args.height+args.height))
                    pixels = list(tile.getdata())

                    # Scrivi i pixel in formato RGB565
                    for row in range(args.height):
                        f.write("    ")
                        for col in range(args.width):
                            px = pixels[row * args.width + col]
                            rgb565 = to_rgb565(px, "RGB", False)

                            # Calcola i componenti RGB a 5/6 bit
                            r5 = (rgb565 >> 11) & 0x1F
                            g6 = (rgb565 >> 5) & 0x3F
                            b5 = rgb565 & 0x1F

                            f.write(f"C16({r5},{g6},{b5})")

                            # Aggiungi virgola se non è l'ultimo pixel
                            is_last_pixel = (tile_index == total_tiles - 1 and
                                           row == args.height - 1 and
                                           col == args.width - 1)
                            if not is_last_pixel:
                                f.write(",")
                        f.write("\n")

                    tile_index += 1

            f.write("};\n\n")
            f.write(f"static const uint16_t {basename}_tilesheet_count = {total_tiles};\n\n")
            f.write("#endif\n")

        print(f"[OK] Generated GFX tileset: {args.output}")
        print(f"[INFO] Tile size: {args.width}x{args.height} pixels")
        print(f"[INFO] Total tiles: {total_tiles} ({tiles_x}x{tiles_y})")
        print(f"[INFO] Format: RGB565 (compatible with tiles_sprites.h)")
        return
    
    new_width = args.width 
    if args.aspect_ratio != 1.0:
        new_width = int(img.width * args.aspect_ratio)
        img = img.resize((new_width, img.height), Image.NEAREST)
    img_w, img_h = img.size
    tiles_x = math.ceil(img_w/new_width)
    tiles_y = math.ceil(img_h/args.height)
    total_tiles = tiles_x*tiles_y

    # Seleziona il convertitore appropriato
    if args.format == "RGBA8888":
        converter = lambda px: to_rgba8888(px, args.byte_order, args.clamp_alpha)
        data_type = "uint32_t"
        hex_format = "08X"
    elif args.format == "RGBA2222":
        converter = lambda px: to_rgba2222(px, args.byte_order, args.clamp_alpha)
        data_type = "uint32_t"
        hex_format = "08X"
    else:  # RGB565
        converter = lambda px: to_rgb565(px, args.byte_order, args.clamp_alpha)
        data_type = "uint16_t"
        hex_format = "04X"

    basename = os.path.splitext(os.path.basename(args.input))[0]
    header_guard = f"_{basename.upper()}_H_"

    palette_entries = None
    if args.with_palette:
        palette_entries = extract_palette_adaptive(img, args.max_colors, args.clamp_alpha)

    with open(args.output, "w") as f:
        f.write(f"#ifndef {header_guard}\n#define {header_guard}\n\n#include <stdint.h>\n\n")
        f.write(f"// Generated from {args.input}, format {args.format}, tile {args.width}x{args.height}\n")
        f.write(f"// Byte order: {args.byte_order}\n\n")

        if args.single_array:
            all_pixels=[]
            for ty in range(tiles_y):
                for tx in range(tiles_x):
                    tile=img.crop((tx*args.width, ty*args.height, tx*args.width+args.width, ty*args.height+args.height))
                    all_pixels.extend(list(tile.getdata()))
            f.write(f"{data_type} {basename}_data[]={{\n")
            for i,px in enumerate(all_pixels):
                val = converter(px)
                end = ", " if i<len(all_pixels)-1 else ""
                f.write(f"0x{val:0{hex_format[1:]}}{end}")
                if (i+1)%args.width==0: f.write("\n")
            f.write("};\n")
            f.write(f"int {basename}_data_len={len(all_pixels)};\n\n")
            f.write(f"#define {basename.upper()}_NUM_TILES {total_tiles}\n\n")
        else:
            tile_index=0
            for ty in range(tiles_y):
                for tx in range(tiles_x):
                    tile=img.crop((tx*args.width, ty*args.height, tx*args.width+args.width, ty*args.height+args.height))
                    pixels=list(tile.getdata())
                    f.write(f"{data_type} {basename}_tile_{tile_index}[]={{\n")
                    for i,px in enumerate(pixels):
                        val=converter(px)
                        end=", " if i<len(pixels)-1 else ""
                        f.write(f"0x{val:0{hex_format[1:]}}{end}")
                        if (i+1)%args.width==0: f.write("\n")
                    f.write("};\n")
                    f.write(f"int {basename}_tile_{tile_index}_len={len(pixels)};\n\n")
                    tile_index+=1
            f.write(f"#define {basename.upper()}_NUM_TILES {tile_index}\n\n")

        if args.with_palette:
            f.write("void init_colors(){\n")
            for i,(r,g,b,a,count) in enumerate(palette_entries):
                if args.initcolor_6bit:
                    r=(r*63)//255
                    g=(g*63)//255
                    b=(b*63)//255
                f.write(f"    init_color({i},{r},{g},{b}); // alpha_avg={a} count={count}\n")
            f.write("}\n\n")
            f.write(f"// Palette entries: {len(palette_entries)} (max_colors={args.max_colors})\n\n")

        f.write("#endif\n")
    print(f"[OK] Generated {args.output} with format={args.format}, byte_order={args.byte_order}, clamp_alpha={args.clamp_alpha}")
    print(f"[INFO] Total Tiles entries: {total_tiles}")
    if args.with_palette:
        print(f"[INFO] Palette entries: {len(palette_entries)}, 6bit={args.initcolor_6bit}")

if __name__=="__main__":
    main()
