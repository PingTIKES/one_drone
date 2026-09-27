#!/usr/bin/env python3
"""Rasterize the 3 m RMUC STL and VIO columns into a 2 m prior map.

This prior is loaded by Nav2 for global planning at the configured cruise height.
Coordinates are Gazebo world ENU after the +90 degree SDF model rotation.
"""
import hashlib
import json
import os
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

ROOT = Path(__file__).resolve().parents[1]
MESH = Path(os.environ.get('RM27_FIELD_MESH', ROOT / 'worlds/models/rmuc_2025/meshes/rmuc_2025.stl'))
WORLD = ROOT / 'worlds/rmuc_2025_3m_vio_columns.sdf'
OUT = ROOT / 'src/navigation/nav2_config/maps/rmuc_2025_prior.pgm'
RES = .1
ORIGIN = (-10., -16.)
WIDTH, HEIGHT = 200, 320
Z_MIN, Z_MAX = 1.5, 2.5


def clip_z(poly, threshold, keep_above):
    out = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        aa, bb = (a[2] >= threshold), (b[2] >= threshold)
        if not keep_above: aa, bb = not aa, not bb
        if aa: out.append(a)
        if aa != bb:
            fraction = (threshold - a[2]) / (b[2] - a[2])
            out.append(a + fraction * (b - a))
    return out


def add_vio_columns(draw):
    """Mark collision boxes intersecting the prior's flight-height band."""
    root = ET.parse(WORLD).getroot()
    count = 0
    for model in root.findall('./world/model'):
        if not model.get('name', '').startswith('vio_column_'):
            continue
        model_pose = [float(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()]
        for link in model.findall('link'):
            link_pose = [float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()]
            for collision in link.findall('collision'):
                box = collision.findtext('geometry/box/size')
                if box is None:
                    raise ValueError(f"{model.get('name')} must use a box collision")
                sx, sy, sz = [float(v) for v in box.split()]
                cp = [float(v) for v in collision.findtext('pose', '0 0 0 0 0 0').split()]
                if any(abs(v) > 1e-8 for v in (model_pose[3], model_pose[4], model_pose[5],
                                                link_pose[3], link_pose[4], link_pose[5],
                                                cp[3], cp[4], cp[5])):
                    raise ValueError('rotated VIO column collision requires polygon rasterization')
                x, y, z = (model_pose[i] + link_pose[i] + cp[i] for i in range(3))
                if z + sz/2 < Z_MIN or z - sz/2 > Z_MAX:
                    continue
                draw.rectangle([((x-sx/2-ORIGIN[0])/RES, (y-sy/2-ORIGIN[1])/RES),
                                ((x+sx/2-ORIGIN[0])/RES, (y+sy/2-ORIGIN[1])/RES)], fill=255)
                count += 1
    if count == 0:
        raise ValueError('no VIO columns intersect the prior flight-height band')
    return count


def main():
    with MESH.open('rb') as f:
        header = f.read(84)
        count = struct.unpack('<I', header[80:84])[0]
        triangles = np.fromfile(f, dtype=np.dtype([
            ('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')
        ]), count=count)['vertices']
    if len(triangles) != count:
        raise ValueError('truncated STL')
    # The SDF link raises the mesh by 0.2 m. The model rotates it +90 degrees:
    # mesh x -> world ENU north, mesh y -> world ENU west.
    triangles = triangles.astype(np.float64)
    triangles[:, :, 2] += .2
    layer = Image.new('L', (WIDTH, HEIGHT), 0)
    draw = ImageDraw.Draw(layer)
    for triangle in triangles:
        if np.max(triangle[:, 2]) < Z_MIN or np.min(triangle[:, 2]) > Z_MAX:
            continue
        poly = clip_z([v for v in triangle], Z_MIN, True)
        if len(poly) < 2: continue
        poly = clip_z(poly, Z_MAX, False)
        if len(poly) < 2: continue
        xy = [((-v[1] - ORIGIN[0]) / RES, (v[0] - ORIGIN[1]) / RES) for v in poly]
        if len(xy) >= 3: draw.polygon(xy, fill=255)
        else: draw.line(xy, fill=255, width=1)
    columns = add_vio_columns(draw)
    # Slightly thicken sub-cell mesh walls for a legible reference image.
    layer = layer.filter(ImageFilter.MaxFilter(3))
    occupied = np.asarray(layer) > 0
    grid = np.full((HEIGHT, WIDTH), -1, np.int8)
    xs = ORIGIN[0] + (np.arange(WIDTH) + .5) * RES
    ys = ORIGIN[1] + (np.arange(HEIGHT) + .5) * RES
    interior = ((abs(xs)[None, :] <= 8.07) & (abs(ys)[:, None] <= 14.58))
    grid[interior] = 0
    grid[occupied] = 100
    # Standard Nav2 map encoding: occupied black, free white, unknown gray.
    pixels = np.full(grid.shape, 205, dtype=np.uint8)
    pixels[grid == 100] = 0
    pixels[grid == 0] = 254
    OUT.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.flipud(pixels), mode='L').save(OUT)
    yaml_path = OUT.with_suffix('.yaml')
    yaml_path.write_text(
        f'image: {OUT.name}\nresolution: {RES}\norigin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n',
        encoding='utf-8')
    metadata = dict(source='rmuc_2025_3m.stl', source_sha256=hashlib.sha256(MESH.read_bytes()).hexdigest(),
                    world=WORLD.name, world_sha256=hashlib.sha256(WORLD.read_bytes()).hexdigest(),
                    vio_columns=columns, layer_m=[Z_MIN, Z_MAX], frame='world_enu',
                    resolution=RES, origin=list(ORIGIN))
    OUT.with_suffix('.metadata.json').write_text(json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
    print(f'{OUT}: free={np.sum(grid == 0)}, occupied={np.sum(grid == 100)}, unknown={np.sum(grid == -1)}')


if __name__ == '__main__': main()
