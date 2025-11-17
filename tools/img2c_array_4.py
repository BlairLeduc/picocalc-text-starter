#!/usr/bin/env python3
"""
img2c_array.py
Convert image into C arrays (uint32_t/uint16_t) in RGBA8888, RGBA2222 or RGB565
Generates a .h header with per-tile arrays or a single array, optional palette init_colors().
Supports AgonLight-specific options: byte order, clamp alpha, 6-bit palette.
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

def main():
    parser = argparse.ArgumentParser(description="Convert image to C RGBA arrays (AgonLight compatible)", add_help=False)
    parser.add_argument("--aspect-ratio", type=float, default=1.0,
                   help="Compensa l'aspect ratio dei pixel (es: 2.67 per AgonLight 2)")
    parser.add_argument("input", help="Input PNG/JPEG image")
    parser.add_argument("-o","--output", help="Output .h file", default="output.h")
    parser.add_argument("-f","--format", choices=["RGBA8888","RGBA2222","RGB565"], default="RGBA8888")
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
    parser.add_argument("--help", action="help", help="Show this help and exit")

    args = parser.parse_args()
    
    # Auto-detect byte order if not specified
    if args.byte_order is None:
        if args.format == "RGB565":
            args.byte_order = "RGB"
        else:
            args.byte_order = "RGBA"

    img = Image.open(args.input).convert("RGBA")
    
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
