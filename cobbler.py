#!/usr/bin/env python3

import itertools
from math import pi, cos, sin, tan, atan, atan2
from svgpathtools import Path, Line, CubicBezier, Arc

# Description of ttgf0p1 die & pads
# Warning: pad numbers don't correspond to QFN pin numbering
DIE_WIDTH = 3.932
DIE_HEIGHT = 5.122
DIE_ORIGIN_X = DIE_WIDTH / 2
DIE_ORIGIN_Y = DIE_HEIGHT / 2
PAD_CENTERS = [
    (-1.908, -2.0045), (-1.908, -1.7935), (-1.908, -1.5825), (-1.908, -1.3715), (-1.908, -1.1605), (-1.908, -0.9495),
    (-1.908, -0.7385), (-1.908, -0.5275), (-1.908, -0.3165), (-1.908, -0.1055), (-1.908, 0.1055), (-1.908, 0.3165),
    (-1.908, 0.5275), (-1.908, 0.7385), (-1.908, 0.9495), (-1.908, 1.1605), (-1.908, 1.3715), (-1.908, 1.5825), (-1.908, 1.7935),
    (-1.908, 2.0045), (-1.44, 2.503), (-1.26, 2.503), (-1.08, 2.503), (-0.9, 2.503), (-0.72, 2.503), (-0.54, 2.503), (-0.36, 2.503),
    (-0.18, 2.503), (0.0, 2.503), (0.18, 2.503), (0.36, 2.503), (0.54, 2.503), (0.72, 2.503), (0.9, 2.503), (1.08, 2.503),
    (1.26, 2.503), (1.44, 2.503), (1.908, 2.0045), (1.908, 1.7935), (1.908, 1.5825), (1.908, 1.3715), (1.908, 1.1605),
    (1.908, 0.9495), (1.908, 0.7385), (1.908, 0.5275), (1.908, 0.3165), (1.908, 0.1055), (1.908, -0.1055), (1.908, -0.3165),
    (1.908, -0.5275), (1.908, -0.7385), (1.908, -0.9495), (1.908, -1.1605), (1.908, -1.3715), (1.908, -1.5825), (1.908, -1.7935),
    (1.908, -2.0045), (1.44, -2.503), (1.26, -2.503), (1.08, -2.503), (0.9, -2.503), (0.72, -2.503), (0.54, -2.503), (0.36, -2.503),
    (0.18, -2.503), (0.0, -2.503), (-0.18, -2.503), (-0.36, -2.503), (-0.54, -2.503), (-0.72, -2.503), (-0.9, -2.503),
    (-1.08, -2.503), (-1.26, -2.503), (-1.44, -2.503),
]
GROUND_PADS = [2, 8, 16, 18, 28, 38, 46, 53, 55, 65]
PAD_SIZE = 0.06

# Description of footprint to generate
FP_WIDTH = 10
FP_HEIGHT = 10.5
FP_ORIGIN_X = FP_WIDTH / 2
FP_ORIGIN_Y = FP_HEIGHT / 2

# Description of pattern to generate
LANDING_PADS = 78
LP_PHI1 = 0.07 * pi
LP_PHI2 = 0.10 * pi
STRIPE_OFFSET = 2

# Bond wires
BOND_MAP = [
    *range(29, 49),
    *range(50, 67),
    *range(68, 78),
    *range(0, 10),
    *range(11, 28),
]

# Wires to footprint edge
TRACE_WIDTH = 0.2
TRACE_CONTROL1_RATIO = 0.1
TRACE_CONTROL2_RATIO = 0.1

EDGE_RASTER = 0.5
CORNER_OFFSET_H = 0
CORNER_OFFSET_V = 1
TOP_LEFT_PAD_OFFSETS = [(1.5, 1)]
LEFT_PAD_COUNT = 20
BOTTOM_LEFT_PAD_OFFSETS = [(1.5, 1)]
BOTTOM_PAD_COUNT = 17
BOTTOM_RIGHT_PAD_OFFSETS = [(1.5, 1)]
RIGHT_PAD_COUNT = 20
TOP_RIGHT_PAD_OFFSETS = [(1.5, 1)]
TOP_PAD_COUNT = 17

TOP_LEFT_PADS = [(-FP_ORIGIN_X+x*EDGE_RASTER, FP_ORIGIN_Y-y*EDGE_RASTER) for x, y in TOP_LEFT_PAD_OFFSETS]
LEFT_PADS = [(-FP_ORIGIN_X, ((LEFT_PAD_COUNT-1)/2-i)*EDGE_RASTER) for i in range(LEFT_PAD_COUNT)]
BOTTOM_LEFT_PADS = [(-FP_ORIGIN_X+x*EDGE_RASTER, -FP_ORIGIN_Y+y*EDGE_RASTER) for x, y in BOTTOM_LEFT_PAD_OFFSETS]
BOTTOM_PADS = [((i-(BOTTOM_PAD_COUNT-1)/2)*EDGE_RASTER, -FP_ORIGIN_Y) for i in range(BOTTOM_PAD_COUNT)]
BOTTOM_RIGHT_PADS = [(FP_ORIGIN_X-x*EDGE_RASTER, -FP_ORIGIN_Y+y*EDGE_RASTER) for x, y in BOTTOM_RIGHT_PAD_OFFSETS]
RIGHT_PADS = [(FP_ORIGIN_X, (i-(RIGHT_PAD_COUNT-1)/2)*EDGE_RASTER) for i in range(RIGHT_PAD_COUNT)]
TOP_RIGHT_PADS = [(FP_ORIGIN_X-x*EDGE_RASTER, FP_ORIGIN_Y-y*EDGE_RASTER) for x, y in TOP_RIGHT_PAD_OFFSETS]
TOP_PADS = [(((TOP_PAD_COUNT-1)/2-i)*EDGE_RASTER, FP_ORIGIN_Y) for i in range(TOP_PAD_COUNT)]
EDGE_PADS = [
    *TOP_LEFT_PADS,
    *LEFT_PADS,
    *BOTTOM_LEFT_PADS,
    *BOTTOM_PADS,
    *BOTTOM_RIGHT_PADS,
    *RIGHT_PADS,
    *TOP_RIGHT_PADS,
    *TOP_PADS,
]

EDGE_MAP = [
    *range(50, 78),
    *range(0, 50),
]

BOND_MAP_REVERSE = {j: i for i, j in enumerate(BOND_MAP)}
GROUND_FINGERS = {i for i in range(LANDING_PADS) if BOND_MAP_REVERSE.get(i) in GROUND_PADS + [None]}


def line_segment(dx, dy):
    return Line(start=0j, end=complex(dx, dy))

def bezier_segment(dx, dy, phi):
    theta = atan(tan(phi)*2/3)
    return CubicBezier(
        start=0j,
        control1=complex(dx/3*cos(theta)-dy/2*sin(theta), dy/3*cos(theta)+dx/2*sin(theta)),
        control2=complex(dx-dx/3*cos(theta)-dy/2*sin(theta), dy-dy/3*cos(theta)+dx/2*sin(theta)),
        end=complex(dx, dy))

def join_segments(*segments):
    pos = 0j
    path = Path()
    for segment in segments:
        segment = segment.translated(pos)
        pos = segment.end
        path.append(segment)
    return path

def rectangle(cx, cy, w, h):
    return join_segments(
        line_segment(-w, 0),
        line_segment(0, h),
        line_segment(w, 0),
        line_segment(0, -h)
    ).translated(complex(cx+w/2, cy-h/2))

def bezier_rectangle(cx, cy, w, h, r, phi1, phi2):
    return join_segments(
        bezier_segment(r*(sin(phi2)-cos(phi1)), r*(-cos(phi2)+sin(phi1)), (pi/2-phi1-phi2)/2),
        bezier_segment(-w-2*r*sin(phi2), 0, phi2),
        bezier_segment(r*(-cos(phi1)+sin(phi2)), r*(-sin(phi1)+cos(phi2)), (pi/2-phi1-phi2)/2),
        bezier_segment(0, h+2*r*sin(phi1), phi1),
        bezier_segment(r*(-sin(phi2)+cos(phi1)), r*(cos(phi2)-sin(phi1)), (pi/2-phi1-phi2)/2),
        bezier_segment(w+2*r*sin(phi2), 0, phi2),
        bezier_segment(r*(cos(phi1)-sin(phi2)), r*(sin(phi1)-cos(phi2)), (pi/2-phi1-phi2)/2),
        bezier_segment(0, -h-2*r*sin(phi1), phi1)
    ).translated(complex(cx+w/2+r*cos(phi1), cy-h/2-r*sin(phi1)))

def rounded_rectangle(cx, cy, w, h, r):
    # r is added to the rectangle size
    return bezier_rectangle(cx, cy, w, h, r, 0, 0)

def polygon(*points):
    path = Path()
    for p, q in itertools.pairwise(points):
        path.append(Line(start=p, end=q))
    return path

def circle(cx, cy, r):
    return Path(
        Arc(start=complex(cx+r, cy), radius=complex(r, r), rotation=0, large_arc=0, sweep=0, end=complex(cx, cy-r)),
        Arc(start=complex(cx, cy-r), radius=complex(r, r), rotation=0, large_arc=0, sweep=0, end=complex(cx-r, cy)),
        Arc(start=complex(cx-r, cy), radius=complex(r, r), rotation=0, large_arc=0, sweep=0, end=complex(cx, cy+r)),
        Arc(start=complex(cx, cy+r), radius=complex(r, r), rotation=0, large_arc=0, sweep=0, end=complex(cx+r, cy))
    )

def stroke_from_edges(edge1, edge2):
    return Path(
        edge1,
        Line(start=edge1.end, end=edge2.end),
        edge2.reversed(),
        Line(start=edge2.start, end=edge1.start),
    )


die_rect = rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT)
mask_ring_inner = rounded_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 0.15)
thermal_bridges_masked = [rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH+1.7, 0.3),
                          rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, 0.3, DIE_HEIGHT+1.7)]
thermal_bridges = [rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH+0.6, 0.3),
                   rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, 0.3, DIE_HEIGHT+0.6)]
ground_pad = rounded_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 0.3)
ground_ring_inner = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 0.5, LP_PHI1, LP_PHI2)
ground_ring_mid = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 0.65, LP_PHI1, LP_PHI2)
ground_ring_outer = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 0.8, LP_PHI1, LP_PHI2)
finger_ring_inner = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 1.0, LP_PHI1, LP_PHI2)
mask_ring_outer = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 1.1, LP_PHI1, LP_PHI2)
finger_ring_bond = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 1.4, LP_PHI1, LP_PHI2)
finger_ring_outer = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 1.7, LP_PHI1, LP_PHI2)
route_ring = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 1.85, LP_PHI1, LP_PHI2)
route_ring_epsilon = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 1.851, LP_PHI1, LP_PHI2)
glob_ring = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 1.95, LP_PHI1, LP_PHI2)
courtyard_ring = bezier_rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, DIE_WIDTH, DIE_HEIGHT, 2.2, LP_PHI1, LP_PHI2)
footprint_rect = rectangle(FP_ORIGIN_X, FP_ORIGIN_Y, FP_WIDTH, FP_HEIGHT)


def stripe_coord(ring, index):
    phi = (index+STRIPE_OFFSET)/(4*LANDING_PADS)*2*pi
    r = 10
    seg = line_segment(r*cos(phi), -r*sin(phi)).translated(complex(FP_ORIGIN_X, FP_ORIGIN_Y))
    isc = ring.intersect(seg)
    assert len(isc) == 1
    return seg.point(isc[0][1][0])
    
def fingers(ring_inner, ring_outer):
    f = []
    for i in range(LANDING_PADS):
        f.append(polygon(
            stripe_coord(ring_inner, 4*i-1),
            stripe_coord(ring_outer, 4*i-1),
            stripe_coord(ring_outer, 4*i+1),
            stripe_coord(ring_inner, 4*i+1),
        ))
    return f


landing_pads = []
for i in range(LANDING_PADS):
    landing_pads.append(stripe_coord(finger_ring_bond, 4*i))


fingers_no_mask = fingers(mask_ring_outer, finger_ring_outer)
fingers_ground_no_mask = fingers(ground_ring_mid, finger_ring_outer)
fingers_mask = fingers(finger_ring_inner, route_ring_epsilon)
fingers_ground_mask = fingers(ground_ring_mid, route_ring_epsilon)
fingers_ground_mask2 = fingers(ground_ring_mid, mask_ring_outer)

pad_centers = [complex(FP_ORIGIN_X+x, FP_ORIGIN_Y+y) for x, y in PAD_CENTERS]
edge_pads = [complex(FP_ORIGIN_X+x, FP_ORIGIN_Y-y) for x, y in EDGE_PADS]

edge_lines = []
center = complex(FP_ORIGIN_X, FP_ORIGIN_Y)
for i in range(LANDING_PADS):
    if EDGE_MAP[i] is None:
        continue
    end = edge_pads[EDGE_MAP[i]]
    start1 = stripe_coord(route_ring, 4*i-1)
    dir1 = (end-start1)/abs(end-start1)
    end1 = end + TRACE_WIDTH/2 * dir1 * 1j
    length1 = abs(end1-start1)
    c11 = start1 + (start1-center)/abs(start1-center) * length1 * TRACE_CONTROL1_RATIO
    c21 = end1 - dir1 * length1 * TRACE_CONTROL2_RATIO
    start2 = stripe_coord(route_ring, 4*i+1)
    dir2 = (end-start2)/abs(end-start2)
    end2 = end - TRACE_WIDTH/2 * dir2 * 1j
    length2 = abs(end2-start2)
    c12 = start2 + (start2-center)/abs(start2-center) * length2 * TRACE_CONTROL1_RATIO
    c22 = end2 - dir2 * length2 * TRACE_CONTROL2_RATIO
    edge1 = CubicBezier(start=start1, control1=c11, control2=c21, end=end1)
    edge2 = CubicBezier(start=start2, control1=c12, control2=c22, end=end2)
    line = stroke_from_edges(edge1, edge2)
    edge_lines.append(line)


with open("preview.svg", "w") as f:
    print(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {FP_WIDTH} {FP_HEIGHT}" width="{FP_WIDTH}mm" height="{FP_HEIGHT}mm">', file=f)
    print(f'<path d="{footprint_rect.d()}" fill="#008000" />', file=f)
    print(f'<path d="{finger_ring_outer.d()}" fill="#aa8800" />', file=f)
    print(f'<path d="{mask_ring_outer.d()}" fill="#008000" />', file=f)
    for i in range(LANDING_PADS):
        if i in GROUND_FINGERS:
            print(f'<path d="{fingers_ground_mask[i].d()}" fill="#55d400" />', file=f)
            print(f'<path d="{fingers_ground_no_mask[i].d()}" fill="#ffff00" />', file=f)
            print(f'<path d="{fingers_ground_mask2[i].d()}" fill="#55d400" />', file=f)
        else:
            print(f'<path d="{fingers_mask[i].d()}" fill="#55d400" />', file=f)
            print(f'<path d="{fingers_no_mask[i].d()}" fill="#ffff00" />', file=f)
    print(f'<path d="{ground_ring_outer.d()}" fill="#55d400" />', file=f)
    print(f'<path d="{ground_ring_inner.d()}" fill="#008000" />', file=f)
    for r in thermal_bridges_masked:
        print(f'<path d="{r.d()}" fill="#55d400" />', file=f)
    for r in thermal_bridges:
        print(f'<path d="{r.d()}" fill="#ffff00" />', file=f)
    print(f'<path d="{ground_pad.d()}" fill="#55d400" />', file=f)
    print(f'<path d="{mask_ring_inner.d()}" fill="#ffff00" />', file=f)
    print(f'<path d="{die_rect.d()}" fill="#999999" />', file=f)
    for i, j in enumerate(BOND_MAP):
        if j is not None:
            print(f'<circle cx="{pad_centers[i].real}" cy="{pad_centers[i].imag}" r="{PAD_SIZE/2}" fill="#333333" />', file=f)
            print(f'<circle cx="{landing_pads[j].real}" cy="{landing_pads[j].imag}" r="{PAD_SIZE/2}" fill="#333333" />', file=f)
            print(f'<line x1="{pad_centers[i].real}" y1="{pad_centers[i].imag}" x2="{landing_pads[j].real}" y2="{landing_pads[j].imag}" stroke="#333333" stroke-width="0.02" />', file=f)
    for pt in edge_pads:
        print(f'<circle cx="{pt.real}" cy="{pt.imag}" r="{TRACE_WIDTH/2}" fill="#55d400" />', file=f)
    for line in edge_lines:
        print(f'<path d="{line.d()}" fill="#55d400" />', file=f)
    print(f'<path d="{glob_ring.d()}" stroke="#ffffff" fill="none" stroke-width="0.12" />', file=f)
    print(f'<path d="{courtyard_ring.d()}" stroke="#ff00ff" fill="none" stroke-width="0.05" />', file=f)
    print('</svg>', file=f)

with open("copper.svg", "w") as f:
    print(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {FP_WIDTH} {FP_HEIGHT}" width="{FP_WIDTH}mm" height="{FP_HEIGHT}mm">', file=f)
    print(f'<path d="{footprint_rect.d()}" fill="#e0e0ff" />', file=f)
    for i in range(LANDING_PADS):
        if i in GROUND_FINGERS:
            print(f'<path d="{fingers_ground_mask[i].d()}" fill="#000000" />', file=f)
            print(f'<path d="{fingers_ground_mask2[i].d()}" fill="#000000" />', file=f)
        else:
            print(f'<path d="{fingers_mask[i].d()}" fill="#000000" />', file=f)
    print(f'<path d="{ground_ring_outer.d()}" fill="#000000" />', file=f)
    print(f'<path d="{ground_ring_inner.d()}" fill="#ffc0c0" />', file=f)
    print(f'<path d="{ground_pad.d()}" fill="#000000" />', file=f)
    for r in thermal_bridges_masked:
        print(f'<path d="{r.d()}" fill="#000000" />', file=f)
    for r in thermal_bridges:
        print(f'<path d="{r.d()}" fill="#000000" />', file=f)
    for pt in edge_pads:
        print(f'<circle cx="{pt.real}" cy="{pt.imag}" r="{TRACE_WIDTH/2}" fill="#000000" />', file=f)
    for line in edge_lines:
        print(f'<path d="{line.d()}" fill="#000000" />', file=f)
    print('</svg>', file=f)

with open("mask.svg", "w") as f:
    print(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {FP_WIDTH} {FP_HEIGHT}" width="{FP_WIDTH}mm" height="{FP_HEIGHT}mm">', file=f)
    print(f'<path d="{footprint_rect.d()}" fill="#e0e0ff" />', file=f)
    print(f'<path d="{finger_ring_outer.d()}" fill="#000000" />', file=f)
    print(f'<path d="{mask_ring_outer.d()}" fill="#ffc0c0" />', file=f)
    print(f'<path d="{mask_ring_inner.d()}" fill="#000000" />', file=f)
    print('</svg>', file=f)

with open("silkscreen.svg", "w") as f:
    print(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {FP_WIDTH} {FP_HEIGHT}" width="{FP_WIDTH}mm" height="{FP_HEIGHT}mm">', file=f)
    print(f'<path d="{footprint_rect.d()}" fill="#e0e0ff" />', file=f)
    print(f'<path d="{glob_ring.d()}" stroke="#000000" fill="none" stroke-width="0.12" />', file=f)
    print('</svg>', file=f)

with open("courtyard.svg", "w") as f:
    print(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {FP_WIDTH} {FP_HEIGHT}" width="{FP_WIDTH}mm" height="{FP_HEIGHT}mm">', file=f)
    print(f'<path d="{footprint_rect.d()}" fill="#e0e0ff" />', file=f)
    print(f'<path d="{courtyard_ring.d()}" stroke="#000000" fill="none" stroke-width="0.05" />', file=f)
    print('</svg>', file=f)

with open("user.svg", "w") as f:
    print(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {FP_WIDTH} {FP_HEIGHT}" width="{FP_WIDTH}mm" height="{FP_HEIGHT}mm">', file=f)
    print(f'<path d="{footprint_rect.d()}" fill="#e0e0ff" />', file=f)
    print(f'<path d="{die_rect.d()}" stroke="#000000" fill="none" stroke-width="0.02" stroke-linecap="round" stroke-linejoin="round" />', file=f)
    for i, j in enumerate(BOND_MAP):
        if j is not None:
            print(f'<circle cx="{pad_centers[i].real}" cy="{pad_centers[i].imag}" r="{PAD_SIZE/2}" fill="#000000" />', file=f)
            print(f'<circle cx="{landing_pads[j].real}" cy="{landing_pads[j].imag}" r="{PAD_SIZE/2}" fill="#000000" />', file=f)
            print(f'<line x1="{pad_centers[i].real}" y1="{pad_centers[i].imag}" x2="{landing_pads[j].real}" y2="{landing_pads[j].imag}" stroke="#000000" stroke-width="0.02" />', file=f)
    print('</svg>', file=f)

