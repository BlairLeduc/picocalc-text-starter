import struct
from xml.etree import ElementTree as ET
import argparse

def tmx_to_bin(input_tmx: str, output_bin: str, bitmap_start_id: int = None):
    """Converte un file .tmx in un file binario compatibile con Agon Light."""

    # Parsing file TMX
    tree = ET.parse(input_tmx)
    root = tree.getroot()

    # Leggi attributi principali
    map_width = int(root.attrib["width"])
    map_height = int(root.attrib["height"])
    tile_width = int(root.attrib["tilewidth"])
    tile_height = int(root.attrib["tileheight"])

    # Recupera i dati CSV della mappa
    layer = root.find("layer")
    data_tag = layer.find("data")
    csv_data = data_tag.text.strip().split(",")

    # Converti in interi
    tile_ids = [int(x.strip()) for x in csv_data if x.strip()]

    # Calcola o usa bitmap_start_id
    if bitmap_start_id is None:
        bitmap_start_id = min(tile_ids) if tile_ids else 0

    # Crea header binario
    header = b"JOBGFX"  # Firma
    header += struct.pack("<H", map_width)
    header += struct.pack("<H", map_height)
    header += struct.pack("B", tile_width)
    header += struct.pack("B", tile_height)
    header += struct.pack("<H", bitmap_start_id)

    # Crea contenuto binario della mappa (16 bit per ogni tile)
    tile_bytes = struct.pack("<" + "H" * len(tile_ids), *tile_ids)

    # Unisci header e dati
    output_data = header + tile_bytes

    # Scrivi file binario
    with open(output_bin, "wb") as f:
        f.write(output_data)

    print(f"File binario creato: {output_bin}")
    print(f"Dimensioni mappa: {map_width}x{map_height} tiles")
    print(f"Dimensioni tile: {tile_width}x{tile_height} pixel")
    print(f"Bitmap start ID: {bitmap_start_id}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Converte un file TMX in formato binario per Agon Light.")
    parser.add_argument("input_tmx", help="Percorso del file .tmx di input")
    parser.add_argument("output_bin", help="Percorso del file .bin di output")
    parser.add_argument("--bitmap_start_id", type=int, default=None, help="ID di partenza bitmap (opzionale)")

    args = parser.parse_args()
    tmx_to_bin(args.input_tmx, args.output_bin, args.bitmap_start_id)
