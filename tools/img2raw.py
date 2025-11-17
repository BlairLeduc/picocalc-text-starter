#!/usr/bin/env python3
"""
Image converter to RAW RGB565 format for PicoCalc

Converts PNG, JPG, BMP, etc. images to RAW RGB565 format
compatible with PicoCalc 'showimg' command.

RAW file format:
  - Bytes 0-1: Width (16-bit little-endian)
  - Bytes 2-3: Height (16-bit little-endian)
  - Bytes 4+:  Pixel data in RGB565 format (2 bytes per pixel)

Usage:
  python3 img2raw.py input.png output.raw [--width WIDTH] [--height HEIGHT]

Example:
  python3 img2raw.py photo.jpg photo.raw --width 320 --height 240
"""

import sys
import struct
import argparse
from PIL import Image

def rgb888_to_rgb565(r, g, b):
    """Convert RGB888 (8-bit per channel) to RGB565 (16-bit total)"""
    r5 = (r >> 3) & 0x1F  # 5 bits for red
    g6 = (g >> 2) & 0x3F  # 6 bits for green
    b5 = (b >> 3) & 0x1F  # 5 bits for blue
    return (r5 << 11) | (g6 << 5) | b5

def convert_image(input_path, output_path, max_width=None, max_height=None):
    """Convert an image to RAW RGB565 format"""
    try:
        # Open image
        img = Image.open(input_path)
        print(f"Original image: {img.size[0]}x{img.size[1]} pixels, mode: {img.mode}")

        # Convert to RGB if necessary
        if img.mode != 'RGB':
            print(f"Converting from {img.mode} to RGB...")
            img = img.convert('RGB')

        # Resize if necessary
        if max_width or max_height:
            original_size = img.size
            if max_width and img.size[0] > max_width:
                aspect_ratio = img.size[1] / img.size[0]
                new_width = max_width
                new_height = int(max_width * aspect_ratio)
                img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)
                print(f"Resized from {original_size[0]}x{original_size[1]} to {new_width}x{new_height}")

            if max_height and img.size[1] > max_height:
                aspect_ratio = img.size[0] / img.size[1]
                new_height = max_height
                new_width = int(max_height * aspect_ratio)
                img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)
                print(f"Resized to {new_width}x{new_height}")

        width, height = img.size

        # Check PicoCalc limits (320x320)
        if width > 320 or height > 320:
            print(f"WARNING: Image is larger than display (320x320)")
            print(f"Image will be centered and cropped if necessary")

        # Open output file
        with open(output_path, 'wb') as f:
            # Write header (width and height in little-endian)
            f.write(struct.pack('<HH', width, height))

            # Convert and write pixels
            pixels = img.load()
            pixel_count = 0

            for y in range(height):
                for x in range(width):
                    r, g, b = pixels[x, y]
                    rgb565 = rgb888_to_rgb565(r, g, b)
                    f.write(struct.pack('<H', rgb565))
                    pixel_count += 1

                # Show progress
                if (y + 1) % 50 == 0:
                    print(f"Processed {y + 1}/{height} lines...")

        file_size = 4 + (pixel_count * 2)
        print(f"\n✓ Conversion completed!")
        print(f"  Output: {output_path}")
        print(f"  Dimensions: {width}x{height} pixels")
        print(f"  File size: {file_size} bytes ({file_size/1024:.2f} KB)")
        print(f"\nTo display on PicoCalc:")
        print(f"  1. Copy {output_path} to SD card")
        print(f"  2. Run: showimg {output_path.split('/')[-1]}")

        return True

    except FileNotFoundError:
        print(f"Error: File '{input_path}' not found.")
        return False
    except Exception as e:
        print(f"Error during conversion: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Convert images to RAW RGB565 format for PicoCalc',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('input', help='Input image file (PNG, JPG, BMP, etc.)')
    parser.add_argument('output', help='Output RAW file')
    parser.add_argument('--width', type=int, help='Maximum width (resize if necessary)')
    parser.add_argument('--height', type=int, help='Maximum height (resize if necessary)')

    args = parser.parse_args()

    # Check if PIL/Pillow is installed
    try:
        import PIL
    except ImportError:
        print("Error: Pillow is not installed.")
        print("Install with: pip install Pillow")
        sys.exit(1)

    # Convert image
    success = convert_image(args.input, args.output, args.width, args.height)
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
