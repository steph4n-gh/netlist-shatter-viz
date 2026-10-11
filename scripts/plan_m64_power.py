#!/usr/bin/env python3
"""Plan legal local ground stitches and conservative power-trace widening.
Geometry screens candidates; native KiCad DRC is the independent acceptance gate.
No trace topology or component position is changed.
"""
import json,math,argparse
from pathlib import Path
from shapely.geometry import Polygon,Point,LineString
from shapely.ops import unary_union

def shapes(rows):
    return unary_union([Point(*v['circle'][:2]).buffer(v['circle'][2]) if 'circle' in v else Polygon(v['shell'],v['holes']) for v in rows])

def plan(g):
    ground=next(int(n) for n,v in g['nets'].items() if v['name']=='GND')
    outline=shapes(g['outline']);via_allowed=outline.buffer(-.57)
    non_ground=unary_union([shapes(ps) for item in g['items'] if item['net']!=ground for ps in item['shapes'].values()])
    drill=shapes(g['holes']);pads=unary_union([shapes(ps) for item in g['items'] if item['kind']=='pad' for ps in item['shapes'].values()])
    per_layer={l:unary_union([shapes(item['shapes'][str(l)]) for item in g['items'] if item['net']!=ground and str(l) in item['shapes']]) for l in g['layers']}
    existing=[Point(v['at']) for v in g['vias'] if v['net']==ground]
    existing += [Point(p['at']) for p in g['pads'] if p['net']==ground and p['drill'][0]>0]
    stitches=[];misses=[]
    # Give populated decouplers and ground pins first access to scarce space.
    targets=sorted([p for p in g['pads'] if p['net']==ground and not p['dnp'] and p['ref']!='J1'],key=lambda p:(not p['ref'].startswith('C'),p['ref'],p['pin']))
    for pad in targets:
        start=Point(pad['at'])
        if existing and min(p.distance(start) for p in existing)<=1.3:continue
        layer=pad['layer']
        if layer not in (g['layers'][0],g['layers'][-1]):continue
        candidates=[]
        for r in (.6,.8,1.0,1.2,1.5,1.8,2.2):
            for angle in range(0,360,15):
                p=Point(round(start.x+r*math.cos(math.radians(angle)),3),round(start.y+r*math.sin(math.radians(angle)),3))
                if not via_allowed.contains(p):continue
                if p.distance(non_ground)<.447 or p.distance(drill)<.15+g['hole_to_hole']+.02:continue
                if p.distance(pads)<.4:continue # No open vias in paste lands.
                if any(p.distance(v)<.62 for v in existing):continue
                line=LineString([start,p])
                if line.distance(per_layer[layer])<.272:continue
                if not outline.buffer(-.395).covers(line):continue
                candidates.append((r,p));break
            if candidates:break
        if not candidates:misses.append({'ref':pad['ref'],'pin':pad['pin']});continue
        _,via=candidates[0];existing.append(via)
        stitches.append({'via':list(via.coords)[0],'pad':pad['at'],'layer':layer,'ref':pad['ref'],'pin':pad['pin']})
    return {'stitches':stitches,'unresolved_ground_pad_proximity':misses,'via_mm':.6,'drill_mm':.3,'track_mm':.25,'maximum_search_radius_mm':2.2}

def widths(g):
    power={'VCC_CART_3V3','VCC_3V3','VCCO3','VCC_1V2','VCC_SW_1V2','VDDOSC27','NetL1_1'}
    changes=[];summary={}
    for net,data in g['nets'].items():
        if data['name'] not in power:continue
        net=int(net);summary[data['name']]={'before_length_over_width':0,'after_length_over_width':0,'changed_segments':0}
        obstacles={l:unary_union([shapes(i['shapes'][str(l)]) for i in g['items'] if i['net']!=net and str(l) in i['shapes']]) for l in g['layers']}
        edge=shapes(g['outline']).boundary
        for t in g['tracks']:
            if t['net']!=net or t['arc']:continue
            line=LineString([t['start'],t['end']]);old=t['width']
            # Split remaining space with a neighboring trace that may also widen.
            allowed=old/2+max(0,line.distance(obstacles[t['layer']])-old/2-.147)/2
            allowed=min(allowed,max(old/2,line.distance(edge)-.27))
            width=max(old,min(1.,math.floor(allowed*2/.005)*.005))
            if width>old+.004:changes.append({'uuid':t['uuid'],'old_width':old,'width':round(width,3)});summary[data['name']]['changed_segments']+=1
            summary[data['name']]['before_length_over_width']+=line.length/old
            summary[data['name']]['after_length_over_width']+=line.length/width
    return {'width_changes':changes,'power_nets':summary,'note':'Length/width sum is a geometry metric, not loaded rail impedance or ampacity.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['stitch','width']);p.add_argument('geometry',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
    result=(plan if a.mode=='stitch' else widths)(json.loads(a.geometry.read_text()));a.output.write_text(json.dumps(result,indent=2)+'\n');print('planned',len(result.get('stitches',result.get('width_changes',[]))))
