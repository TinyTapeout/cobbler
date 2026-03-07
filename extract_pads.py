#!/usr/bin/env python3

import gdstk
import math

lib = gdstk.read_gds("ttgf0p1_v23.fill.gds")
ctop = next(cell for cell in lib.cells if cell.name == "tt_top")
(bx1, by1), (bx2, by2) = ctop.bounding_box()
r = lambda x: round(x, 9)
cx, cy, hw, hh = r((bx1+bx2)/2000), r((by1+by2)/2000), r((bx2-bx1)/2000), r((by2-by1)/2000)
pads = [p for p in ctop.polygons if p.layer == 81 and p.datatype == 10]
pad_centers = set((r((x1+x2)/2000-cx), r(cy-(y1+y2)/2000)) for ((x1, y1), (x2, y2)) in [p.bounding_box() for p in pads])
pad_centers = sorted(pad_centers, key=lambda p: math.atan2(*p))
first_index = min(range(len(pad_centers)), key=lambda i: 10*pad_centers[i][0]+pad_centers[i][1])
pad_centers = pad_centers[first_index:] + pad_centers[:first_index]

print(f"DIE_WIDTH = {hw*2}")
print(f"DIE_HEIGHT = {hh*2}")
print("DIE_ORIGIN_X = DIE_WIDTH / 2")
print("DIE_ORIGIN_Y = DIE_HEIGHT / 2")
print("PAD_CENTERS = [")
for x, y in pad_centers:
    print(f"    ({x}, {y}),")
print("]")

