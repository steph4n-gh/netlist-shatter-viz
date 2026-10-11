#!/usr/bin/env python3
"""Independent copper/mask geometry comparison against KiCad's Gerber plotter.
Run with: uv run --with-requirements scripts/requirements-cam.txt scripts/verify_cam.py DIR --kicad-cli PATH
No upload or network operation. KiCad is an optional local verification dependency.
"""
import argparse, json, math, subprocess, warnings
from pathlib import Path
from gerbonara import GerberFile, ExcellonFile
from gerbonara import graphic_primitives as gp
from gerbonara.rs274x import UnknownStatementWarning
warnings.simplefilter("error", UnknownStatementWarning)
from gerbonara.utils import MM
from shapely.geometry import Point, LineString, Polygon
from shapely.ops import unary_union
from shapely.affinity import translate


def geometry(path):
    shapes = []
    for obj in GerberFile.open(path).objects:
        for p in obj.to_primitives(unit=MM):
            if isinstance(p, gp.Circle):
                shape = Point(p.x,p.y).buffer(p.r,quad_segs=64)
            elif isinstance(p,gp.Line):
                shape = LineString([(p.x1,p.y1),(p.x2,p.y2)]).buffer(p.width/2,quad_segs=64)
            elif isinstance(p,gp.Rectangle):
                c,s=math.cos(p.rotation),math.sin(p.rotation)
                shape = Polygon([(p.x+x*c-y*s,p.y+x*s+y*c) for x,y in [(-p.w/2,-p.h/2),(-p.w/2,p.h/2),(p.w/2,p.h/2),(p.w/2,-p.h/2)]])
            else:
                shape = Polygon(p.to_arc_poly().approximate_arcs(max_error=0.0001).outline)
            if not p.polarity_dark:
                raise ValueError(f"Unexpected negative primitive in {path}; refusing incomplete comparison")
            shapes.append(shape)
    return unary_union(shapes)


def main():
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('directory',type=Path);a.add_argument('--kicad-cli',default='kicad-cli');args=a.parse_args()
    d=args.directory;b=json.loads((d/'board.json').read_text());native=d/'kicad-verification';native.mkdir(exist_ok=True)
    pcb=next(d.glob('*.kicad_pcb'))
    subprocess.run([args.kicad_cli,'pcb','export','gerbers','-l','F.Cu,B.Cu,F.Mask,B.Mask','-o',str(native)+'/',str(pcb)],check=True)
    result={"reader":"gerbonara 1.6.3", "comparison":"KiCad plotted copper and mask versus native Rust CAM", "layers":{}}
    for layer,ext in [('F_Cu','gtl'),('B_Cu','gbl'),('F_Mask','gts'),('B_Mask','gbs')]:
        own=geometry(d/f'{layer}.gbr');gold=translate(geometry(next(native.glob(f'*.{ext}'))),yoff=b['height'])
        # KiCad rounds serialized geometry to 0.1um. Buffer comparison permits
        # at most 2um contour deviation, not missing copper or a mirrored board.
        missing=gold.difference(own.buffer(.002)).area;extra=own.difference(gold.buffer(.002)).area
        result['layers'][layer]={"missing_mm2":missing,"extra_mm2":extra,"pass":missing<1e-7 and extra<1e-7}
        (d/f'{layer}.svg').write_text(str(GerberFile.open(d/f'{layer}.gbr').to_svg()))
    drill=ExcellonFile.open(d/'drill.drl');actual=sorted(set((round(o.x,3),round(o.y,3),round(o.tool.diameter,3)) for o in drill.objects))
    expected=[]
    for v in b['vias']:expected.append((v['at']['x'],b['height']-v['at']['y'],round(v['drill'],3)))
    for f in b['footprints']:
        angle=math.radians(f['rotation']);c,s=math.cos(angle),math.sin(angle)
        for p in f['pads']:
            if 'ThroughHole' not in p['pad_type']:continue
            x,y=p['relative_pos']['x'],p['relative_pos']['y']
            expected.append((f['position']['x']+x*c-y*s,b['height']-(f['position']['y']+x*s+y*c),round(p['pad_type']['ThroughHole']['drill'],3)))
    result['drill']={"expected_hits":len(set(expected)),"actual_hits":len(actual),"pass":len(actual)==len(set(expected)) and all(any(abs(x-a)<.000501 and abs(y-b)<.000501 and abs(d-c)<.000001 for a,b,c in actual) for x,y,d in set(expected))}
    result['pass']=all(x['pass'] for x in result['layers'].values()) and result['drill']['pass']
    (d/'cam-verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['pass'] else 1)

if __name__=='__main__':main()
