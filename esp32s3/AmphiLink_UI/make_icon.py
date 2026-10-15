"""Generate the desktop-shortcut icon from the brand mark, using only stdlib.

The brand asset ships as SVG, which WScript.Shell shortcuts cannot use, so the
same mark is rasterised here into a 32-bit ICO. Keeping this dependency-free
means the shortcut installer works on a bare Windows install.
"""
from pathlib import Path
import struct

SIZE = 64
BACKGROUND = (0x17, 0x32, 0x2D)   # #17322d, matches web/icon.svg
MARK = (0xA6, 0xEB, 0xC7)         # #a6ebc7
CORNER_RADIUS = 16
STROKE_WIDTH = 5.0


def blend(back, front, coverage):
    """Alpha-composite `front` over `back` at `coverage` in [0, 1]."""
    return tuple(round(b + (f - b) * coverage) for b, f in zip(back, front))


def segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    return ((px - (ax + t * dx)) ** 2 + (py - (ay + t * dy)) ** 2) ** 0.5


def rounded_square_coverage(x, y):
    """Coverage of the rounded-rect background at pixel centre (x, y)."""
    limit = SIZE - CORNER_RADIUS
    cx = min(max(x, CORNER_RADIUS), limit)
    cy = min(max(y, CORNER_RADIUS), limit)
    distance = ((x - cx) ** 2 + (y - cy) ** 2) ** 0.5
    # One-pixel smooth edge, so the corners do not alias badly.
    return max(0.0, min(1.0, CORNER_RADIUS + 0.5 - distance))


def mark_coverage(x, y):
    """Coverage of the A-mark strokes (the web/icon.svg path, drawn as segments)."""
    segments = [
        (15, 46, 32, 15),   # left leg
        (32, 15, 49, 46),   # right leg
        (22, 35, 42, 35),   # crossbar
    ]
    nearest = min(segment_distance(x, y, *segment) for segment in segments)
    return max(0.0, min(1.0, STROKE_WIDTH / 2 + 0.5 - nearest))


def pixel_rows():
    """RGBA rows, top-down."""
    rows = []
    for row in range(SIZE):
        line = []
        for column in range(SIZE):
            x, y = column + 0.5, row + 0.5
            shape = rounded_square_coverage(x, y)
            if shape <= 0:
                line.append((0, 0, 0, 0))
                continue
            colour = blend(BACKGROUND, MARK, mark_coverage(x, y) * shape)
            line.append((*colour, round(shape * 255)))
        rows.append(line)
    return rows


def dib_bytes(rows):
    """BITMAPINFOHEADER (height doubled, XOR then AND mask) plus BGRA pixels."""
    header = struct.pack('<IiiHHIIiiII', 40, SIZE, SIZE * 2, 1, 32, 0,
                         SIZE * SIZE * 4, 0, 0, 0, 0)
    xor = bytearray()
    for row in reversed(rows):                   # DIB rows are bottom-up
        for red, green, blue, alpha in row:
            xor += bytes((blue, green, red, alpha))
    mask_stride = ((SIZE + 31) // 32) * 4        # 1bpp AND mask, 4-byte aligned
    return header + bytes(xor) + bytes(mask_stride * SIZE)


def write_ico(path):
    image = dib_bytes(pixel_rows())
    icondir = struct.pack('<HHH', 0, 1, 1)
    entry = struct.pack('<BBBBHHII', SIZE, SIZE, 0, 0, 1, 32, len(image), 22)
    Path(path).write_bytes(icondir + entry + image)


if __name__ == '__main__':
    import sys
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name('rlyoulink.ico')
    write_ico(target)
    print(f'{target} ({target.stat().st_size} bytes)')
