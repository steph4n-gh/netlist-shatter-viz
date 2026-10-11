#!/usr/bin/env python3
"""Candidate-only polygon and exact-mask topology diagnostic; never physical UNSAT."""
import argparse
import ast
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time

import numpy as np
import shapely
from shapely.geometry import LineString,Point,Polygon
from scipy.ndimage import label

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'scripts/grid_route.py').exists())


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,value):p.write_text(json.dumps(value,indent=2)+'\n')


def shortest_transitions(edges,starts,goals,maximum=None):
    queue=deque((s,0) for s in starts);seen=set(starts)
    while queue:
        node,cost=queue.popleft()
        if node in goals:return cost
        if maximum is not None and cost>=maximum:continue
        for other in edges.get(node,()):
            if other not in seen:seen.add(other);queue.append((other,cost+1))
    return None


def boundary_attribution(region,layer,items,spec,data,padding=0,native_ids=None):
    """Attribute a measured model-pocket boundary to actual source primitives.

    A two-nanometre-wide boundary comparison band accounts only for Boolean
    roundoff. This is boundary support, not a certified minimum physical cut.
    """
    native_ids=native_ids or {};rows=[]
    for index,item in enumerate(items):
        if item['net']==spec.get('net') or layer not in item['geometry']:continue
        radius=max(spec['clearance'],item['clearance'])+spec['width']/2+padding
        obstacle=item['geometry'][layer].buffer(radius)
        support=region.boundary.intersection(obstacle.boundary.buffer(1e-6)).length
        if support<=1e-5:continue
        terminals=item.get('terminals',[]);primitives=[]
        if item['kind']=='copper':
            for primitive in data.get('tracks',[])+data.get('vias',[]):
                if primitive['net']!=item['net']:continue
                if item.get('uuid') and primitive.get('uuid')!=item['uuid']:continue
                if 'start' in primitive and primitive['layer']!=layer:continue
                points=[primitive['start'],primitive['end']] if 'start' in primitive else [primitive['at'],primitive['at']]
                if len(terminals)==2 and sorted(map(tuple,points))==sorted(map(tuple,terminals)):
                    primitives.append({'uuid':primitive['uuid'],'kind':'track' if 'start' in primitive else 'via','locked':primitive.get('locked',False)})
        else:
            for pad in data.get('pads',[]):
                if pad['net']!=item['net'] or pad['at'] not in terminals:continue
                for uuid,ident in native_ids.items():
                    if ident['kind']=='pad' and ident['net']==pad['net'] and ident.get('reference')==pad['ref'] and ident.get('pin')==pad['pin']:
                        primitives.append({'uuid':uuid,'kind':'pad','reference':pad['ref'],'pin':pad['pin'],'editable':False})
        name=data['nets'].get(str(item['net']),{}).get('name','no net')
        rows.append({'item_index':index,'net':item['net'],'name':name,'kind':item['kind'],
            'boundary_support_mm':support,'native_primitives':primitives,
            'editable_copper':bool(primitives) and (bool(item.get('uuid')) or len(primitives)==1)
                and all(p['kind'] in ('track','via') and not p.get('locked') for p in primitives)})
    totals={}
    for row in rows:
        entry=totals.setdefault(row['net'],{'net':row['net'],'name':row['name'],'boundary_support_mm':0,'editable_copper_uuids':[],'fixed_pad_uuids':[]})
        entry['boundary_support_mm']+=row['boundary_support_mm']
        entry['editable_copper_uuids'].extend(p['uuid'] for p in row['native_primitives'] if row['editable_copper'])
        entry['fixed_pad_uuids'].extend(p['uuid'] for p in row['native_primitives'] if p['kind']=='pad')
    return {'rows':rows,'boundary_net_groups':sorted(totals.values(),key=lambda x:(-x['boundary_support_mm'],x['net'])),
        'scope':'Actual exported native item boundaries. Editable UUIDs identify unlocked tracks/vias only; pad/keepout/outline/drill policy remains fixed. Net groups are boundary-support attribution, not a minimum physical cut.'}


def polygon_graph(domains,via_domain,groups,layers,maximum=None):
    parts={};edges={};contact=[]
    for layer,domain in zip(layers,domains):
        for index,part in enumerate(shapely.get_parts(domain)):
            if part.geom_type=='Polygon' and part.area>1e-12:
                node=(layer,index);parts[node]=part;edges[node]=set()
    nodes=list(parts)
    ports={n:p.intersection(via_domain) for n,p in parts.items()}
    via_edges=[]
    for i,a in enumerate(nodes):
        if ports[a].is_empty:continue
        for b in nodes[i+1:]:
            if a[0]==b[0] or ports[b].is_empty:continue
            common=ports[a].intersection(ports[b])
            if common.area>1e-12:
                edges[a].add(b);edges[b].add(a)
                via_edges.append({'from':a,'to':b,'legal_center_area_mm2':common.area})
    for group in groups:
        touched=set()
        for n,p in parts.items():
            if any(n[0] in item['geometry'] and item['geometry'][n[0]].intersection(p).area>1e-12 for item in group):touched.add(n)
        contact.append(touched)
    pairs=[]
    for i in range(len(contact)):
        for j in range(i+1,len(contact)):
            minimum=shortest_transitions(edges,contact[i],contact[j])
            pairs.append({'groups':[i,j],'minimum_new_vias':minimum,
                'connected_under_via_limit':minimum is not None and (maximum is None or minimum<=maximum)})
    contacted=set().union(*contact) if contact else set()
    return {'layer_components':{str(l):sum(n[0]==l for n in parts) for l in layers},
        'via_center_area_mm2':via_domain.area,'via_edges':via_edges,
        'terminal_contacts':[sorted(c) for c in contact],'terminal_pairs':pairs,
        'contacted_regions':[{'node':n,'area_mm2':parts[n].area,'bounds_mm':list(parts[n].bounds),
            'legal_via_center_area_mm2':ports[n].area,'via_neighbors':sorted(edges[n])} for n in sorted(contacted)]}


def grid_graph(domains,attachment_domains,via_domain,groups,layers,endpoints,bounds,step,maximum=None,via_domains=None):
    x0,y0,x1,y1=bounds;nx=math.ceil((x1-x0)/step)+1;ny=math.ceil((y1-y0)/step)+1
    xs,ys=np.meshgrid(x0+np.arange(nx)*step,y0+np.arange(ny)*step)
    masks=[shapely.contains_xy(d,xs,ys) for d in domains]
    labels=[];counts=[];edges={}
    for layer,mask in zip(layers,masks):
        labs,count=label(mask,structure=np.array([[0,1,0],[1,1,1],[0,1,0]]));labels.append(labs);counts.append(count)
        for i in range(1,count+1):edges[(layer,i)]=set()
    via_mask=np.logical_and.reduce([shapely.contains_xy(d,xs,ys) for d in via_domains]) if via_domains is not None else shapely.contains_xy(via_domain,xs,ys)
    via_edges=[]
    for i,layer in enumerate(layers):
        for j in range(i+1,len(layers)):
            valid=via_mask & masks[i] & masks[j]
            if not valid.any():continue
            pairs=np.unique(np.column_stack((labels[i][valid],labels[j][valid])),axis=0)
            for a,b in pairs:
                na,nb=(layer,int(a)),(layers[j],int(b));edges[na].add(nb);edges[nb].add(na);via_edges.append([na,nb])
    contacts=[];attachment_counts=[]
    for group in groups:
        found=set();states=set()
        for p,layer in endpoints(group,attachment_domains):
            l=layers.index(layer);x=round((p[0]-x0)/step);y=round((p[1]-y0)/step)
            for dx in range(-3,4):
                for dy in range(-3,4):
                    xx,yy=x+dx,y+dy
                    q=[round(x0+xx*step,6),round(y0+yy*step,6)]
                    if 0<=xx<nx and 0<=yy<ny and masks[l][yy,xx] and attachment_domains[l].covers(LineString([p,q])):
                        found.add((layer,int(labels[l][yy,xx])));states.add((l,yy,xx))
        contacts.append(found);attachment_counts.append(len(states))
    result=[]
    for i in range(len(contacts)):
        for j in range(i+1,len(contacts)):
            minimum=shortest_transitions(edges,contacts[i],contacts[j])
            result.append({'groups':[i,j],'minimum_new_vias':minimum,
                'connected_under_via_limit':minimum is not None and (maximum is None or minimum<=maximum)})
    return {'step_mm':step,'bounds_mm':bounds,'shape':[len(layers),ny,nx],
        'mask_components':dict(zip(map(str,layers),counts)),'legal_states':sum(int(m.sum()) for m in masks),
        'legal_via_states':int(via_mask.sum()),'via_edges':via_edges,
        'terminal_contacts':[sorted(c) for c in contacts],'attachment_states':attachment_counts,'terminal_pairs':result,
        'contacted_regions':[{'node':n,'state_count':int((labels[layers.index(n[0])]==n[1]).sum()),
            'legal_via_states':int((via_mask & (labels[layers.index(n[0])]==n[1])).sum())} for n in sorted(set().union(*contacts))],
        'same_layer_reachability_equivalence':'4-neighbor labels exactly match admitted 8-neighbor connectivity: every admitted diagonal requires both orthogonal cells, so each diagonal has a legal 4-neighbor replacement. This is reachability, not equal path cost.'}


def model_scope(data,profile,grid_path,preserve_source=False):
    """Load the exact frozen domain/components/endpoints policy without routing."""
    import grid_route
    if Path(grid_path).resolve()==Path(grid_route.__file__).resolve():
        return grid_route.routing_geometry(dict(data,routing_profile=profile),step=0,preserve_source=preserve_source)
    from grid_route import shape
    from routing_screen import corridors,inset
    layers=data['layers'];outline=shape(data['outline'])
    limits={r['net']:r for r in profile.get('net_limits',[])}
    specs={c:dict(s,width=max(s['width'],limits.get(s['name'],{}).get('min_width_mm',0))) for c,s in data['nets'].items()}
    items=[dict(i,geometry={int(l):shape(p) for l,p in i['shapes'].items()}) for i in data['items']]
    keepouts=[dict(k,geometry=shape(k['shapes'])) for k in data['keepouts']]
    drill_forbidden_lands=shapely.union_all([g for i in items if i.get('drill_forbidden') for g in i['geometry'].values()])
    holes=shape(data['holes']);drill_obstacles=[dict(h,geometry=shape(h['shapes'])) for h in data.get('drill_obstacles',[])]
    references=corridors(data,profile);reservations=[]
    for net,r in references.items():
        for track in data['tracks']:
            if track['net']==net and track['layer']==r['layer']:
                radius=track['width']/2+r['rule']['margin_mm']+.002
                reservations.append({'layer':r['reference_layer'],'net':r['reference_net'],'clearance':r['clearance_mm'],
                    'geometry':LineString([track['start'],track['end']]).buffer(radius/math.cos(math.pi/64),quad_segs=16)})
    # Execute the exact retained production formulas instead of making a second
    # obstacle policy. The diagnostic never executes the router or creates Cu.
    ast_tree=ast.parse(grid_path.read_text())
    scope=dict(data=data,layers=layers,outline=outline,specs=specs,limits=limits,items=items,keepouts=keepouts,
        holes=holes,drill_obstacles=drill_obstacles,drill_forbidden_lands=drill_forbidden_lands,references=references,reservations=reservations,
        Polygon=Polygon,Point=Point,LineString=LineString,shape=shape,shapely=shapely,math=math,inset=inset)
    for name in ('domains','components','endpoints'):
        node=next(n for n in ast.walk(ast_tree) if isinstance(n,ast.FunctionDef) and n.name==name)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(grid_path),'exec'),scope)
    return scope


def diagnose(source,out):
    out.mkdir(parents=True)
    if any(out.iterdir()):raise ValueError('Output must be empty')
    started=time.perf_counter();frozen=out/'frozen-scripts';frozen.mkdir()
    for p in (ROOT/'scripts').glob('*.py'):shutil.copy2(p,frozen/p.name)
    sys.path.insert(0,str(frozen))
    from grid_route import shape
    from routing_screen import corridors,inset
    from kicad_sexpr import sexpr
    case_source=source;selection=None
    if (source/'remaining-open-frontier-diagnosis.json').exists():
        paths={n:source/n for n in ('candidate.kicad_pcb','dc-geometry.json','candidate.routing.json','routed-drc.json','remaining-open-frontier-diagnosis.json')}
        retained_grid=json.loads((source/'remaining-open-native-blocker-geometry.json').read_text())['grid_route_sha256']
    else:
        experiment=json.loads((source/'experiment.json').read_text())
        eligible=[(i,t) for i,t in enumerate(experiment['trials']) if t.get('fixed_reference_pass') and t.get('native_drc',{}).get('unconnected')==1]
        index,trial=min(eligible,key=lambda pair:(pair[1]['native_drc']['total_findings'],-pair[1]['fixed_reference_passing_checks'],pair[0]))
        stage=next(i for i,s in enumerate(trial['stages']) if s.get('failed'))
        case_source=source/f'order-{index:02}'
        paths={n:case_source/n for n in ('candidate.kicad_pcb','dc-geometry.json','routed-drc.json')}
        paths.update({'candidate.routing.json':source/'candidate.routing.json','remaining-open-frontier-diagnosis.json':case_source/f'search-{stage}.json'})
        shutil.copy2(source/'frozen-grid_route.py',frozen/'grid_route.py')
        retained_grid=sha(source/'frozen-grid_route.py')
        selection={'experiment':str(source/'experiment.json'),'trial_index':index,'failed_stage_index':stage,'stage_order':trial['order'],'native_drc':trial['native_drc'],'fixed_passing':trial['fixed_reference_passing_checks']}
    manifest={str(path):sha(path) for path in paths.values()}
    for name,path in paths.items():shutil.copy2(path,out/name)
    write(out/'source-manifest.json',manifest)
    write(out/'frozen-script-manifest.json',{p.name:sha(p) for p in frozen.glob('*.py')})
    data=json.loads((out/'dc-geometry.json').read_text());profile=json.loads((out/'candidate.routing.json').read_text())
    data['routing_profile']=profile
    assert data['source_board_sha256']==sha(out/'candidate.kicad_pcb')
    frontier=json.loads((out/'remaining-open-frontier-diagnosis.json').read_text())
    assert frontier['source_board_sha256']==sha(out/'candidate.kicad_pcb')
    tree=sexpr((out/'candidate.kicad_pcb').read_text());native_ids={}
    def value(item,key):return next((v[1:] for v in item if isinstance(v,list) and v and v[0]==key),None)
    mapping={v[1]:v[2] for v in tree if isinstance(v,list) and len(v)==3 and v[0]=='net'}
    def identify(obj,reference=None):
        uuid=value(obj,'uuid');net=value(obj,'net')
        if not uuid or not net:return
        token=mapping.get(net[0],net[-1]);name=json.loads(token) if token.startswith('"') else token
        matches=[int(code) for code,spec in data['nets'].items() if spec['name']==name]
        if len(matches)==1:native_ids[json.loads(uuid[0])]={'net':matches[0],'name':name,'reference':reference,'kind':obj[0],
            'pin':json.loads(obj[1]) if obj[0]=='pad' and obj[1].startswith('"') else obj[1] if obj[0]=='pad' else None}
    for obj in tree:
        if not isinstance(obj,list) or not obj:continue
        if obj[0]=='footprint':
            reference=next((json.loads(v[2]) for v in obj if isinstance(v,list) and len(v)>2 and v[:2]==['property','"Reference"']),None)
            for pad in obj:
                if isinstance(pad,list) and pad and pad[0]=='pad':identify(pad,reference)
        elif obj[0] in ('segment','arc','via'):identify(obj)
    retained_codes={f['net'] for f in frontier['failed']}
    opens=json.loads((out/'routed-drc.json').read_text())['unconnected_items'];targets=[]
    for issue in opens:
        identified=[dict(native_ids[i['uuid']],uuid=i['uuid'],native_description=i['description'],native_position_mm=i['pos']) for i in issue['items'] if i['uuid'] in native_ids]
        codes={i['net'] for i in identified}
        if len(codes)==1 and codes<=retained_codes:targets.append({'net':next(iter(codes)),'native_items':identified})
    assert targets,'No UUID-resolved native frontier target'
    scope=model_scope(data,profile,frozen/'grid_route.py')
    layers=scope['layers'];specs=scope['specs'];limits=scope['limits'];items=scope['items']
    rows=[]
    for target in targets:
        net=target['net'];spec=specs[str(net)];groups=scope['components'](net)
        step=frontier.get('net_grid_mm',{}).get(str(net),frontier['grid_mm']);scope['step']=step
        maximum=limits.get(spec['name'],{}).get('max_vias')
        remaining=None if maximum is None else max(0,maximum-sum(v['net']==net for v in data['vias']))
        full=scope['domains'](net,spec['width']/2,padding=0)
        padded=scope['domains'](net,spec['width']/2)
        via_full=shapely.intersection_all(scope['domains'](net,spec['via_diameter']/2,True,padding=0))
        via_padded_layers=scope['domains'](net,spec['via_diameter']/2,True)
        via_padded=shapely.intersection_all(via_padded_layers)
        bounds=frontier.get('search_windows',{}).get(str(net),frontier['search_bounds_mm'])
        grid=grid_graph(padded,full,via_padded,groups,layers,scope['endpoints'],bounds,step,remaining,via_domains=via_padded_layers)
        row=dict(target,name=spec['name'],spec=spec,remaining_new_via_limit=remaining,native_copper_groups=len(groups),
            group_native_uuids=[[i['uuid'] for i in target['native_items'] if any(p.covers(Point(i['native_position_mm']['x'],i['native_position_mm']['y'])) for item in group for p in item['geometry'].values())] for group in groups],
            unpadded_polygon_graph=polygon_graph(full,via_full,groups,layers,remaining),
            router_padded_polygon_graph=polygon_graph(padded,via_padded,groups,layers,remaining),exact_grid_mask_graph=grid)
        row['boundary_attribution']={}
        for key,domains,padding in (('unpadded_polygon_graph',full,0),('router_padded_polygon_graph',padded,.003+step/math.sqrt(2))):
            row['boundary_attribution'][key]=[]
            if all(p['connected_under_via_limit'] for p in row[key]['terminal_pairs']):continue
            for region in row[key]['contacted_regions']:
                layer,index=region['node'];part=list(shapely.get_parts(domains[layers.index(layer)]))[index]
                measured=boundary_attribution(part,layer,items,dict(spec,net=net),data,padding,native_ids)
                row['boundary_attribution'][key].append(dict(node=region['node'],**measured))
        row['retained_astar_failure']=next(f for f in frontier['failed'] if f['net']==net)
        rows.append(row)
    result={'state':'done','candidate_only':True,'source_board_sha256':sha(out/'candidate.kicad_pcb'),
        'frozen_grid_route_sha256':sha(frozen/'grid_route.py'),'matches_retained_frontier_grid_code':sha(frozen/'grid_route.py')==retained_grid,
        'source_candidate_directory':str(case_source),'retained_trial_selection':selection,
        'profile_sha256':sha(out/'candidate.routing.json'),'target_selection':'Native DRC UUIDs resolved against board AST, intersected with retained failed-net metadata; no manually supplied net or coordinates.',
        'targets':rows,'elapsed_seconds':time.perf_counter()-started,'routes_generated':0,'fabrication_exported':False,
        'source_unchanged':all(sha(Path(n))==h for n,h in manifest.items()),
        'limitations':['Native exported polygons and Shapely buffer chord approximations; zero-area contacts excluded at1e-12mm2.',
            'Unpadded polygon graph is continuous free-space connectivity of the exported model, not physical infeasibility certification.',
            'Exact-mask graph preserves same-layer reachability, endpoint attachments and via centers; it ignores path-length/skew costs and deadlines. Via-count limit is enforced.',
            'Same-net terminal-region graph permits contact with its existing copper; it does not impose the router locked-tip-only endpoint preference. Exact-mask graph does impose current endpoints().',
            'Native final DRC and fixed108 checks remain required for any future generated candidate.']}
    write(out/'topology.json',result)
    print(json.dumps({'state':result['state'],'elapsed_seconds':result['elapsed_seconds'],'targets':[{'name':r['name'],'continuous':r['unpadded_polygon_graph']['terminal_pairs'],'padded':r['router_padded_polygon_graph']['terminal_pairs'],'grid':r['exact_grid_mask_graph']['terminal_pairs']} for r in rows]},indent=2))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();diagnose(a.source.resolve(),a.out.resolve())


if __name__=='__main__':main()
