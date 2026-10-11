#!/usr/bin/env python3
"""Create, place and route a new native PCB from an explicit design JSON and project rules."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import time
from build_board import find_kicad
from solve_kicad import native_worker_command,solve,ROOT


def validate_design(spec):
    if not isinstance(spec,dict) or set(spec)-{'board','components','attribution','planes','groups'}:
        raise ValueError('Expected board/components and optional attribution; unsupported design fields are rejected')
    board=spec.get('board',{})
    if not isinstance(board,dict) or set(board)-{'width','height','layers','thickness'}:
        raise ValueError('New-design placement currently supports rectangular board dimensions only')
    for key,low,high in [('width',5,300),('height',5,300),('thickness',.1,10)]:
        v=board.get(key,1.6 if key=='thickness' else None)
        if not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) or not low<=v<=high:
            raise ValueError('Invalid board '+key)
    if board.get('layers',2) not in (2,4,6,8):raise ValueError('Use 2, 4, 6 or 8 copper layers')
    parts=spec.get('components')
    if not isinstance(parts,list) or not 1<=len(parts)<=100:raise ValueError('Supply 1–100 components')
    refs=set()
    for p in parts:
        if not isinstance(p,dict) or set(p)-{'reference','value','footprint','nets','position','initial_position','rotation'}:
            raise ValueError('Unsupported component fields')
        ref=p.get('reference')
        if not isinstance(ref,str) or not ref or ref in refs:raise ValueError('Unique nonempty component references required')
        refs.add(ref)
        if not isinstance(p.get('value',''),str):raise ValueError('Component values must be strings')
        if not isinstance(p.get('footprint'),str) or not p['footprint'].lstrip().startswith('(footprint '):
            raise ValueError(ref+': embed an explicit KiCad footprint')
        nets=p.get('nets')
        if not isinstance(nets,dict) or any(not isinstance(pin,str) or not pin or (n is not None and (not isinstance(n,str) or not n)) for pin,n in nets.items()):
            raise ValueError(ref+': supply pad-number to net-name mappings; null means no-connect')
        if 'initial_position' in p and 'position' in p:raise ValueError(ref+': position is fixed; use only initial_position for a movable group')
        pos=p.get('position',[0,0]);rotation=p.get('rotation',0)
        if not isinstance(pos,list) or len(pos)!=2 or any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in pos+[rotation]):
            raise ValueError(ref+': position and rotation must be finite numbers')


    groups=spec.get('groups',[]);grouped=set();by_ref={p['reference']:p for p in parts}
    if not isinstance(groups,list):raise ValueError('groups must be lists of component references')
    for group in groups:
        if not isinstance(group,list) or not group:raise ValueError('Empty or invalid placement group')
        for ref in group:
            if not isinstance(ref,str) or ref not in refs or ref in grouped:raise ValueError('Unknown, repeated or overlapping group reference')
            grouped.add(ref);p=by_ref[ref]
            if 'position' in p:raise ValueError(ref+': grouped members must be movable, not fixed positions')
            pos=p.get('initial_position')
            finite=isinstance(pos,list) and len(pos)==2 and all(not isinstance(v,bool) and isinstance(v,(int,float)) for v in pos)
            if finite:
                try:finite=all(math.isfinite(v) for v in pos)
                except OverflowError:finite=False
            if not finite:
                raise ValueError(ref+': supply a finite relative initial_position for each group member')
        origin=by_ref[group[0]]['initial_position']
        if any(abs(v-o)>2147.483647 for ref in group for v,o in zip(by_ref[ref]['initial_position'],origin)):
            raise ValueError('Relative group initial_position exceeds native 32-bit nanometre coordinates')
    if any('initial_position' in p and p['reference'] not in grouped for p in parts):raise ValueError('initial_position requires a placement group')

    planes=spec.get('planes',[])
    if not isinstance(planes,list):raise ValueError('planes must be a list')
    names={n for p in parts for n in p['nets'].values() if n is not None}
    available={'F.Cu','B.Cu'}|{f'In{i}.Cu' for i in range(1,board.get('layers',2)-1)}
    seen=set()
    for plane in planes:
        if not isinstance(plane,dict) or set(plane)!={'net','layer','clearance_mm','edge_inset_mm'}:raise ValueError('Plane requires net, layer, clearance_mm and edge_inset_mm')
        if any(not isinstance(plane[k],str) or not plane[k] for k in ('net','layer')) or plane['net'] not in names or plane['layer'] not in available:raise ValueError('Plane net/layer must exist in the design')
        if plane['layer'] in seen:raise ValueError('One rectangular plane per layer is supported')
        seen.add(plane['layer'])
        for key in ('clearance_mm','edge_inset_mm'):
            v=plane[key]
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0:raise ValueError('Plane distances must be finite and positive')
        if 2*plane['edge_inset_mm']>=min(board['width'],board['height']):raise ValueError('Plane inset consumes the board')


def create_design(spec,out,kicad=None):
    started=time.perf_counter()
    spec,out=Path(spec).resolve(),Path(out).resolve()
    validate_design(json.loads(spec.read_text()))
    if out.exists() and any(out.iterdir()):raise ValueError('Creation output must be empty')
    project=spec.with_suffix('.kicad_pro')
    if not project.exists():raise ValueError('Matching .kicad_pro with explicit manufacturing rules is required')
    kicad=kicad or find_kicad()
    out.mkdir(parents=True,exist_ok=True)
    inputs=[spec,project]+([spec.with_suffix('.kicad_dru')] if spec.with_suffix('.kicad_dru').exists() else [])
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    try:
        with (out/'creation.log').open('w') as log:
            subprocess.run(native_worker_command(kicad,ROOT/'scripts/create_native_worker.py')+[str(spec),str(out)],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=90)
    except subprocess.CalledProcessError as error:
        raise ValueError('Creation failed: '+(out/'creation.log').read_text()[-1500:]) from error
    for p in inputs[1:]:shutil.copy2(p,out/p.name)
    shutil.copy2(spec,out/spec.name)
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    (out/'input-manifest.json').write_text(json.dumps(hashes,indent=2))
    report=json.loads((out/'creation.json').read_text())
    report.update(elapsed_seconds=time.perf_counter()-started,source_manifest=hashes)
    (out/'creation.json').write_text(json.dumps(report,indent=2))
    return out/(spec.stem+'.kicad_pcb')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('design',type=Path);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--dc-profile',type=Path,help='Electrical scenarios; defaults to the matching .dc.json when present')
    p.add_argument('--routing-profile',type=Path,help='Reference-plane rules; defaults to matching .routing.json')
    args=p.parse_args();board=create_design(args.design,args.out/'created')
    profile=args.dc_profile or args.design.with_suffix('.dc.json')
    routing_profile=args.routing_profile or args.design.with_suffix('.routing.json')
    if args.routing_profile and not routing_profile.is_file():p.error('Routing profile does not exist')
    if args.dc_profile and not profile.is_file():p.error('Electrical profile does not exist')
    result=solve(board,args.out/'native',creation=json.loads((board.parent/'creation.json').read_text()),dc_profile=profile if profile.exists() else None,routing_profile=routing_profile if routing_profile.exists() else None)
    print(json.dumps(result,indent=2));return 0 if result['review_pass'] else 1


if __name__=='__main__':raise SystemExit(main())
