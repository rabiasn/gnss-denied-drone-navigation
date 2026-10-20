"""Small lossless RGB PNG writer for Gazebo's isolated Python (stdlib only)."""
from pathlib import Path
import struct
import zlib


def chunk(kind, payload):
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", zlib.crc32(kind + payload) & 0xffffffff)


def write_rgb_png(path, width, height, step, pixels):
    if not 0 < width <= 4096 or not 0 < height <= 4096 or step < width * 3 or len(pixels) < height * step:
        raise ValueError("Invalid bounded RGB image layout")
    raw = b"".join(b"\x00" + pixels[row * step:row * step + width * 3] for row in range(height))
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    payload = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 1)) + chunk(b"IEND", b"")
    with Path(path).open("xb") as stream:
        stream.write(payload)
