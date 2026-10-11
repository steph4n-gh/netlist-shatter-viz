#!/usr/bin/env python3
"""Bounded multilayer A* fallback over polygons exported by KiCad.

This is a candidate generator. Only native KiCad DRC can approve its output.
It preserves placement and locked copper; it does not solve impedance or timing.
"""
import argparse
from collections import deque
import heapq
import json
import math
from pathlib import Path
import time

import numpy as np
import shapely
from shapely.geometry import Polygon, Point, LineString


def shape(polygons):
    return shapely.union_all([Point(*p['circle'][:2]).buffer(p['circle'][2]) if 'circle' in p
                             else Polygon(p['shell'], p.get('holes', [])) for p in polygons])


def mask_reachability(masks, via_mask, starts, goals, via_budget=None):
    """Exact reachability of this router's admitted moves, without path costs."""
    from scipy.ndimage import label
    labels, counts = zip(*(label(mask) for mask in masks))
    edges = {}
    for a in range(len(masks)):
        for b in range(a+1, len(masks)):
            valid = via_mask & masks[a] & masks[b]
            if not valid.any():continue
            base = counts[b]+1
            pairs = np.unique(labels[a][valid].astype(np.int64)*base+labels[b][valid])
            for pair in pairs:
                first, second = (a, int(pair//base)), (b, int(pair%base))
                edges.setdefault(first, set()).add(second)
                edges.setdefault(second, set()).add(first)
    def contacts(states):return {(l, int(labels[l][y,x])) for l,y,x in states}
    def distances(nodes):
        found = dict.fromkeys(nodes, 0);queue = deque(nodes)
        while queue:
            node = queue.popleft()
            for other in edges.get(node, ()):
                if other not in found:found[other] = found[node]+1;queue.append(other)
        return found
    source_nodes, target_nodes = contacts(starts), contacts(goals)
    source_distances = distances(source_nodes)
    minimum = min((source_distances[n] for n in target_nodes if n in source_distances), default=None)
    report = {'connected':minimum is not None and (via_budget is None or minimum<=via_budget),
              'minimum_new_vias':minimum,'mask_components':list(counts)}
    if report['connected']:return report, None
    # Four-neighbor labels preserve admitted eight-neighbor reachability:
    # each admitted diagonal requires both intervening orthogonal cells.
    sizes = [np.bincount(labs.ravel()) for labs in labels]
    def reached(distances):return {n for n,d in distances.items() if via_budget is None or d<=via_budget}
    source_reached, target_reached = reached(source_distances), reached(distances(target_nodes))
    size = lambda nodes:sum(int(sizes[l][i]) for l,i in nodes)
    nodes = source_reached if size(source_reached)<=size(target_reached) else target_reached
    report.update(diagnosed_terminal='source' if nodes is source_reached else 'target', reached_states=size(nodes))
    return report, [np.isin(labs, [i for ll,i in nodes if ll==l]) for l,labs in enumerate(labels)]


def routing_geometry(data, step=.1, preserve_source=False):
    """One obstacle/contact policy shared by grid, polygon and joint routing."""
    layers=data['layers']
    from routing_screen import corridors,inset
    references=corridors(data,data['routing_profile']) if data.get('routing_profile') else {}
    limits={r['net']:r for r in data.get('routing_profile',{}).get('net_limits',[])}
    specs={code:dict(spec,width=max(spec['width'],limits.get(spec['name'],{}).get('min_width_mm',0)))
           for code,spec in data['nets'].items()}
    outline = shape(data['outline'])
    items = [dict(i, geometry={int(l):shape(p) for l,p in i['shapes'].items()}) for i in data['items']]
    keepouts = [dict(k, geometry=shape(k['shapes'])) for k in data['keepouts']]
    drill_forbidden_lands = shapely.union_all([g for i in items if i.get('drill_forbidden')
                                              for g in i['geometry'].values()])
    source_copper=shapely.union_all([g for i in items for g in i['geometry'].values()]) if preserve_source else None
    state = {'holes':shape(data['holes'])}
    drill_obstacles=[dict(h,geometry=shape(h['shapes'])) for h in data.get('drill_obstacles',[])]
    reservations=[]
    def reserve(net,segments):
        if net not in references:return
        r=references[net]
        for t in segments:
            if t['layer']==r['layer']:
                radius=t['width']/2+r['rule']['margin_mm']+.002
                reservations.append({'layer':r['reference_layer'],'net':r['reference_net'],'clearance':r['clearance_mm'],
                    'geometry':LineString([t['start'],t['end']]).buffer(radius/math.cos(math.pi/64),quad_segs=16)})
    for net in references:reserve(net,[t for t in data.get('tracks',[]) if t['net']==net])
    # Native input polygons stay fixed while candidate paths append and roll
    # back. Reuse their expensive unions; every query still includes current
    # candidate copper, reference reservations and drill holes.
    fixed_items=len(items);fixed_reservations=len(reservations);base_obstacles={}
    def domains(net, radius, via=False, padding=None):
        if padding is None:padding=.003+step/math.sqrt(2)
        spec = specs[str(net)]
        allowed=limits.get(spec['name'],{}).get('allowed_layers')
        inside = outline.buffer(-data['edge_clearance']-radius-padding)
        result = []
        for layer in layers:
            if not via and allowed is not None and data['layer_names'][str(layer)] not in allowed:
                result.append(Polygon());continue
            key=(net,radius,via,padding,layer)
            if key not in base_obstacles:
                fixed = [i['geometry'][layer].buffer(max(spec['clearance'], i['clearance'])+radius+padding)
                         for i in items[:fixed_items] if i['net'] != net and layer in i['geometry']]
                fixed += [k['geometry'].buffer(radius+padding) for k in keepouts
                          if layer in k['layers'] and k['vias' if via else 'tracks']]
                fixed += [h['geometry'].buffer(radius+max(data['hole_clearance'],h.get('clearance',0))+padding)
                          for h in drill_obstacles if h['net']!=net]
                fixed += [r['geometry'].buffer(radius+max(spec['clearance'],r['clearance'])+padding)
                          for r in reservations[:fixed_reservations] if via or (r['layer']==layer and r['net']!=net)]
                if via:
                    fixed.append(drill_forbidden_lands.buffer((spec['via_drill']/2+padding)/math.cos(math.pi/64),quad_segs=16))
                    if source_copper is not None:
                        fixed.append(source_copper.buffer((spec['via_drill']/2+padding)/math.cos(math.pi/64),quad_segs=16))
                base_obstacles[key]=shapely.union_all(fixed)
            obstacles = [base_obstacles[key]]
            obstacles += [i['geometry'][layer].buffer(max(spec['clearance'], i['clearance'])+radius+padding)
                          for i in items[fixed_items:] if i['net'] != net and layer in i['geometry']]
            obstacles += [r['geometry'].buffer(radius+max(spec['clearance'],r['clearance'])+padding)
                          for r in reservations[fixed_reservations:] if via or (r['layer']==layer and r['net']!=net)]
            if via:
                obstacles.append(state['holes'].buffer(spec['via_drill']/2+data.get('hole_to_hole',data['hole_clearance'])+padding))
            domain=inside.difference(shapely.union_all(obstacles))
            if net in references:
                r=references[net]
                domain=domain.intersection(inset(r['domain'],radius+padding)) if not via and layer==r['layer'] else Polygon()
            physical_window=data.get('candidate_only_native_copper_windows',{}).get(str(net))
            if physical_window is not None:
                domain=domain.intersection(shape(physical_window).buffer(-radius-padding))
            result.append(domain)
        return result

    def components(net):
        net_items = [i for i in items if i['net']==net and i['terminals']]
        groups = []
        for item in net_items:
            matches = [g for g in groups if any(a['geometry'][l].intersects(item['geometry'][l])
                       for a in g for l in a['geometry'].keys() & item['geometry'].keys())]
            for g in matches:groups.remove(g)
            groups.append([item]+[a for g in matches for a in g])
        return groups

    def endpoints(group, legal=None):
        # Through-via barrel copper on unused layers is equipotential Cu, not
        # a dangling track tip. Match native via inventory and all-layer shape
        # coverage so ordinary (including zero-length) tracks stay track tips.
        via_bodies={(v['net'],tuple(v['at'])) for v in data.get('vias',[])
                    if v.get('through',True)}
        def via_body(item):
            return (item['kind']=='copper' and len(item['terminals'])==2
                    and item['terminals'][0]==item['terminals'][1]
                    and (item['net'],tuple(item['terminals'][0])) in via_bodies
                    and set(item['geometry'])==set(layers))
        points = [(tuple(p), l) for i in group for p in i['terminals'] for l in i['geometry']
                  if i['geometry'][l].covers(Point(p))]
        if legal is not None:
            # Edge fingers and large pads can have their centres inside a track
            # keepout while exposing legal copper at the inner end. Connectivity
            # is to the native copper shape, not necessarily the pad centre.
            for item in group:
                if item['kind']!='pad':continue
                for layer,polygon in item['geometry'].items():
                    contact=polygon.intersection(legal[layers.index(layer)])
                    if not contact.is_empty and contact.area>1e-10:
                        for region in getattr(contact,'geoms',[contact]):
                            if region.area<=1e-10:continue
                            p=region.representative_point();points.append(((p.x,p.y),layer))
        # Native item shapes carry UUIDs; their lock state is in the Cu inventory.
        locked = {v['uuid']:v.get('locked',False) for v in data.get('tracks',[])+data.get('vias',[]) if v.get('uuid')}
        # A locked dangling tip must be used, rather than bypassed at its pad.
        tips = [(tuple(p),l) for i in group if i['kind']=='copper' and locked.get(i.get('uuid'),i.get('locked',False)) and not via_body(i) for p in i['terminals'] for l in i['geometry']
                if not any(j is not i and l in j['geometry'] and j['geometry'][l].covers(Point(p)) for j in group)]
        return list(set(tips or points))

    return dict(data=data, layers=layers, outline=outline, specs=specs, limits=limits,
                items=items, keepouts=keepouts, holes=state['holes'], state=state,
                drill_obstacles=drill_obstacles, drill_forbidden_lands=drill_forbidden_lands,
                references=references, reservations=reservations, shape=shape,
                domains=domains, components=components, endpoints=endpoints, reserve=reserve)


def append_path(context,net,path,validate=True):
    """Quantize once and emit a connected candidate using the shared policy."""
    data=context['data'];layers=context['layers'];spec=context['specs'][str(net)]
    if any(layer not in layers or len(point)!=2 or not all(math.isfinite(float(v))for v in point) for layer,point in path):
        raise ValueError('Invalid native route coordinate or layer')
    compact=[(layer,[round(float(v)*1e6)/1e6 for v in point]) for layer,point in path]
    legal=context['domains'](net,spec['width']/2,padding=0) if validate else None
    via_legal=shapely.intersection_all(context['domains'](net,spec['via_diameter']/2,via=True,padding=0)) if validate and any(a[0]!=b[0]for a,b in zip(compact,compact[1:])) else None
    tracks=[];vias=[];geometry={};terminals=[]
    for (la,a),(lb,b) in zip(compact,compact[1:]):
        if la==lb:
            if a==b:continue
            line=LineString([a,b])
            if validate and not legal[layers.index(la)].covers(line):
                raise ValueError('Quantized complete trace leaves its legal domain')
            tracks.append({'start':a,'end':b,'layer':la,'width':spec['width'],'net':net})
            geometry.setdefault(la,[]).append(line.buffer(spec['width']/2))
        else:
            if a!=b or validate and not via_legal.covers(Point(a)):
                raise ValueError('Quantized layer transition has no legal through via')
            if validate and any(math.dist(a,v['at'])<spec['via_drill']+data.get('hole_to_hole',data['hole_clearance']) for v in vias):
                raise ValueError('Generated via drills violate hole spacing')
            vias.append({'at':a,'diameter':spec['via_diameter'],'drill':spec['via_drill'],'net':net})
            for layer in layers:geometry.setdefault(layer,[]).append(Point(a).buffer(spec['via_diameter']/2))
        terminals.extend([a,b])
    if not geometry:raise ValueError('Empty route cannot establish a connection')
    if validate:
        maximum=context['limits'].get(spec['name'],{}).get('max_vias')
        count=sum(v['net']==net for v in data.get('vias',[]))+sum(len(item.get('vias',[]))for item in context['items']if item['net']==net and item['kind']=='route')+len(vias)
        if maximum is not None and count>maximum:raise ValueError('Generated route exceeds supplied via limit')
        for item in context['items']:
            if item['kind']!='route':continue
            own=item['net']==net
            clearance=max(spec['clearance'],item['clearance'])
            if not own:
                for new in tracks:
                    line=LineString([new['start'],new['end']])
                    for old in item.get('tracks',[]):
                        if old['layer']==new['layer'] and line.distance(LineString([old['start'],old['end']]))<(old['width']+new['width'])/2+clearance:
                            raise ValueError('Quantized generated traces violate pair clearance')
                    for old in item.get('vias',[]):
                        if line.distance(Point(old['at']))<(new['width']+old['diameter'])/2+clearance:
                            raise ValueError('Quantized generated trace and via violate pair clearance')
                for new in vias:
                    point=Point(new['at'])
                    for old in item.get('tracks',[]):
                        if point.distance(LineString([old['start'],old['end']]))<(new['diameter']+old['width'])/2+clearance:
                            raise ValueError('Quantized generated via and trace violate pair clearance')
            for new in vias:
                for old in item.get('vias',[]):
                    distance=math.dist(new['at'],old['at'])
                    if distance<(new['drill']+old['drill'])/2+data.get('hole_to_hole',data['hole_clearance']):
                        raise ValueError('Quantized generated via drills violate pair spacing')
                    if not own and distance<(new['diameter']+old['diameter'])/2+clearance:
                        raise ValueError('Quantized generated vias violate copper clearance')
    context['state']['holes']=shapely.union_all([context['state']['holes'],*[Point(v['at']).buffer(v['drill']/2)for v in vias]])
    context['holes']=context['state']['holes']
    item={'net':net,'geometry':{layer:shapely.union_all(polys) for layer,polys in geometry.items()},
          'clearance':spec['clearance'],'terminals':terminals,'kind':'route','tracks':tracks,'vias':vias}
    context['reserve'](net,tracks);context['items'].append(item)
    return tracks,vias,item


def route(data, step=.1, seconds=30, max_states=8_000_000, only_nets=None, priority_nets=(), continuous=False, escape_prefix=False):
    if escape_prefix and (not continuous or only_nets is None or len(only_nets)!=1 or str(only_nets[0])not in data['nets']):
        raise ValueError('Escape prefix requires one explicit continuous native demand')
    if not math.isfinite(step) or step<=0 or not math.isfinite(seconds) or seconds<=0:
        raise ValueError('Grid spacing and time budget must be positive and finite')
    started = time.perf_counter()
    deadline = started + seconds
    layers = data['layers']
    context=routing_geometry(data,step)
    references=context['references'];limits=context['limits'];specs=context['specs'];outline=context['outline']
    x0, y0, x1, y1 = outline.bounds
    if only_nets:
        # A local repair should not rasterize an entire cartridge at fine pitch.
        # Keep all native obstacles, but bound the search around the selected
        # copper with a 5 mm escape margin. Failure remains a bounded-search miss.
        selected=[shape(polygons) for item in data['items'] if item['net'] in only_nets
                  for polygons in item['shapes'].values()]
        if selected:
            a,b,c,d=shapely.union_all(selected).bounds
            x0,y0,x1,y1=max(x0,a-5),max(y0,b-5),min(x1,c+5),min(y1,d+5)
    nx, ny = math.ceil((x1-x0)/step)+1, math.ceil((y1-y0)/step)+1
    if not continuous and not only_nets and nx*ny*len(layers) > max_states:
        raise ValueError('Board exceeds fallback grid budget; no claim of infeasibility')
    xs, ys = (None,None) if only_nets or continuous else np.meshgrid(x0+np.arange(nx)*step, y0+np.arange(ny)*step)
    search_windows={}
    items=context['items']
    components=context['components'];endpoints=context['endpoints'];reserve=context['reserve']
    tracks, vias, completed, failed = [], [], [], []
    requested_step=step;net_grid_mm={}

    def check_time():
        if time.perf_counter() >= deadline:
            raise TimeoutError('Fallback search budget exhausted; infeasibility is not established')

    def xy(state):
        return [round(x0+state[2]*step, 6), round(y0+state[1]*step, 6)]

    def domains(net,radius,via=False,padding=None):
        if padding is None:padding=.003+step/math.sqrt(2)
        return context['domains'](net,radius,via=via,padding=padding)

    continuous_trials=[]
    last_search_failure=None
    def find_path(sources, targets, legal, attachment_legal, via_mask, via_budget, preferred_layer=None, attempt_deadline=None, net=None):
        nonlocal last_search_failure
        last_search_failure=None
        masks = [shapely.contains_xy(g, xs, ys) for g in legal]
        def attachments(endpoints):
            found = {}
            for p,layer in endpoints:
                l=layers.index(layer); x=round((p[0]-x0)/step); y=round((p[1]-y0)/step)
                for dx in range(-3,4):
                    for dy in range(-3,4):
                        xx,yy=x+dx,y+dy
                        s=(l,yy,xx)
                        if 0<=xx<nx and 0<=yy<ny and masks[l][yy,xx] and attachment_legal[l].covers(LineString([p,xy(s)])):
                            found[s]=p
            return found
        starts, goals = attachments(sources), attachments(targets)
        def diagnose(states, reached_masks=None):
            # Attribute the boundary of the actually reached pocket to native
            # copper, rather than guessing from distance to either terminal.
            blockers={};primitive_ids={};bounds=[];boundary_layers=[]
            primitive_map={}
            for native in data.get('tracks',[])+data.get('vias',[]):
                points=[native['start'],native['end']] if 'start' in native else [native['at']]
                key=(native['net'],tuple(sorted(set(map(tuple,points)))))
                if native.get('uuid'):primitive_map.setdefault(key,set()).add(native['uuid'])
            for l,layer in enumerate(layers):
                reached=np.zeros((ny,nx),dtype=bool) if reached_masks is None else reached_masks[l]
                if reached_masks is None:
                    for s in states:
                        if s[0]==l:reached[s[1],s[2]]=True
                if not reached.any():continue
                if reached[0].any() or reached[-1].any() or reached[:,0].any() or reached[:,-1].any():
                    boundary_layers.append(layer)
                frontier=np.zeros_like(reached)
                frontier[1:]|=reached[:-1];frontier[:-1]|=reached[1:]
                frontier[:,1:]|=reached[:,:-1];frontier[:,:-1]|=reached[:,1:]
                yy,xx=np.where(frontier & ~masks[l])
                if not len(xx):continue
                stride=max(1,math.ceil(len(xx)/10000));xx,yy=xx[::stride],yy[::stride]
                px,py=x0+xx*step,y0+yy*step
                bounds.append({'layer':layer,'bounds_mm':[float(px.min()),float(py.min()),float(px.max()),float(py.max())]})
                spec=specs[str(net)]
                for item in items:
                    if item['net']==net or item['kind'] not in ('copper','route') or layer not in item['geometry']:continue
                    area=item['geometry'][layer].buffer(max(spec['clearance'],item['clearance'])+spec['width']/2+.003+step/math.sqrt(2))
                    count=int(shapely.intersects_xy(area,px,py).sum())*stride
                    if count:
                        blockers[item['net']]=blockers.get(item['net'],0)+count
                        key=(item['net'],tuple(sorted(set(map(tuple,item['terminals'])))))
                        primitive_ids.setdefault(item['net'],set()).update(primitive_map.get(key,()))
            return {'pocket_bounds':bounds,'touches_search_boundary':bool(boundary_layers),'search_boundary_layers':boundary_layers,
                    'blocking_copper':[{'net':n,'name':specs[str(n)]['name'],'frontier_contacts':count,'primitive_uuids':sorted(primitive_ids.get(n,()))}
                    for n,count in sorted(blockers.items(),key=lambda p:(-p[1],p[0]))]}
        if not starts or not goals:
            last_search_failure={'reason':'No legal grid attachment','source_states':len(starts),'target_states':len(goals)}
            return None
        topology, reached = mask_reachability(masks, via_mask, starts, goals, via_budget)
        if not topology['connected']:
            last_search_failure={'reason':'Disconnected legality masks or insufficient via budget',
                                 'expanded_states':0,'topology':topology,**diagnose((), reached)}
            return None
        check_time()
        # Search outward from the smaller contact set. A boxed-in supply pin
        # should exhaust its small pocket, not scan the entire connected rail.
        if len(starts)>len(goals):starts,goals=goals,starts
        target_xy=list({(g[1],g[2]) for g in goals})
        if len(target_xy)>16:
            from scipy.spatial import cKDTree
            goal_tree=cKDTree(target_xy)
        else:goal_tree=None
        def h(s):
            if goal_tree is not None:return float(goal_tree.query([s[1],s[2]])[0])
            return min(math.hypot(y-s[1],x-s[2]) for y,x in target_xy)
        heap=[];cost={};parent={}
        for s,p in starts.items():
            s=(*s,0)
            cost[s]=math.dist(p,xy(s))/step;heapq.heappush(heap,(cost[s]+h(s),cost[s],s))
        expanded=0
        while heap:
            _,distance,s=heapq.heappop(heap)
            if distance!=cost.get(s):continue
            if via_budget is not None and any(cost.get((*s[:3],v),math.inf)<=distance for v in range(s[3])):continue
            if s[:3] in goals:
                path=[s]
                while path[-1] in parent:path.append(parent[path[-1]])
                path.reverse()
                return path, starts[path[0][:3]], goals[s[:3]]
            expanded+=1
            if expanded%1024==0:
                if attempt_deadline is not None and time.perf_counter()>=attempt_deadline:
                    last_search_failure={'reason':'Target search budget exhausted','expanded_states':expanded,**diagnose(cost)}
                    return None
                check_time()
            l,y,x,used_vias=s
            moves=[]
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                xx,yy=x+dx,y+dy
                if 0<=xx<nx and 0<=yy<ny and masks[l][yy,xx]:
                    if dx and dy and not (masks[l][y,xx] and masks[l][yy,x]):continue
                    weight=1 if preferred_layer is None or layers[l]==preferred_layer else 3
                    moves.append(((l,yy,xx,used_vias),math.hypot(dx,dy)*weight))
            if (via_budget is None or used_vias<via_budget) and via_mask[y,x]:
                moves.extend(((ll,y,x,used_vias+1 if via_budget is not None else 0),2/step) for ll in range(len(layers)) if ll!=l and masks[ll][y,x])
            for dest,increment in moves:
                c=distance+increment
                if via_budget is not None and any(cost.get((*dest[:3],v),math.inf)<=c for v in range(dest[3]+1)):continue
                if c<cost.get(dest,math.inf):
                    cost[dest]=c;parent[dest]=s;heapq.heappush(heap,(c+h(dest),c,dest))
        last_search_failure={'reason':'No path on this grid','expanded_states':expanded,**diagnose(cost)}
        return None

    path_budgets={}
    for rule in data.get('routing_profile',{}).get('paths',[]):
        path_budgets[rule['net']]=min(path_budgets.get(rule['net'],math.inf),rule['max_length_mm'])
    path_nets={p['id']:p['net'] for p in data.get('routing_profile',{}).get('paths',[])}
    matched_nets={path_nets[p] for group in data.get('routing_profile',{}).get('matched_groups',[]) for p in group['paths']}
    preferred={}
    for group in data.get('routing_profile',{}).get('matched_groups',[]):
        names={path_nets[p] for p in group['paths']}
        codes={int(n) for n,s in specs.items() if s['name'] in names}
        region=shapely.union_all([poly for item in items if item['net'] in codes for poly in item['geometry'].values()]).envelope.buffer(1)
        allowed=set(layers)
        for name in names:
            if 'allowed_layers' in limits.get(name,{}):allowed&={l for l in layers if data['layer_names'][str(l)] in limits[name]['allowed_layers']}
        if allowed:
            # Give matched paths a common low-congestion layer preference while
            # permitting legal escapes. Final skew checks still decide acceptance.
            choice=min(allowed,key=lambda l:sum(poly.intersection(region).area for item in items if item['net'] not in codes for layer,poly in item['geometry'].items() if layer==l))
            preferred.update({n:choice for n in codes})
    for code,layer in data.get('preferred_layers',{}).items():
        net=int(code)
        if str(net) not in specs or layer not in layers:
            raise ValueError('Preferred layer must name an existing net and copper layer')
        allowed=limits.get(specs[str(net)]['name'],{}).get('allowed_layers')
        if allowed is not None and data['layer_names'][str(layer)] not in allowed:
            raise ValueError('Preferred layer conflicts with supplied allowed layers')
        preferred[net]=layer
    # Short constrained connections get their escape space before longer paths.
    order=sorted((int(n) for n in data['nets'] if only_nets is None or int(n) in only_nets),
                 key=lambda n:(n not in priority_nets,specs[str(n)]['name'] not in matched_nets,path_budgets.get(specs[str(n)]['name'],math.inf),len(components(n)),n))
    try:
        for net in order:
            check_time()
            if only_nets and not continuous:
                step=requested_step
                # Crop each repair net independently; distant faults must not
                # force a single oversized raster covering unrelated board area.
                a,b,c,d=data.get('repair_windows',{}).get(str(net)) or shapely.union_all([poly for item in items if item['net']==net for poly in item['geometry'].values()]).bounds
                ox0,oy0,ox1,oy1=outline.bounds
                x0,y0,x1,y1=max(ox0,a-5),max(oy0,b-5),min(ox1,c+5),min(oy1,d+5)
                nx,ny=math.ceil((x1-x0)/step)+1,math.ceil((y1-y0)/step)+1
                while nx*ny*len(layers)>max_states and step<.1:
                    step=min(.1,step*2)
                    nx,ny=math.ceil((x1-x0)/step)+1,math.ceil((y1-y0)/step)+1
                if nx*ny*len(layers)>max_states:
                    failed.append({'net':net,'reason':'Local repair exceeds grid budget'});continue
                net_grid_mm[str(net)]=step
                xs,ys=np.meshgrid(x0+np.arange(nx)*step,y0+np.arange(ny)*step)
                search_windows[str(net)]=[x0,y0,x1,y1]
            spec=specs[str(net)]
            # Track via count in search only when a limit is supplied.
            maximum=limits.get(spec['name'],{}).get('max_vias')
            existing=sum(v['net']==net for v in data.get('vias',[]))
            groups=components(net)
            while len(groups)>1:
                if continuous:
                    from polygon_path import find_path as find_polygon_path,first_via_prefix
                    attachment_legal=domains(net,spec['width']/2,padding=0)
                    groups.sort(key=lambda g:not any(i['kind']=='copper' for i in g))
                    sources=endpoints(groups[0],attachment_legal)
                    contacts=[(group,endpoints(group,attachment_legal)) for group in groups[1:]]
                    if not sources or any(not points for group,points in contacts):
                        failed.append({'net':net,'name':spec['name'],'remaining_components':len(groups),
                                       'search':{'reason':'No legal continuous endpoint attachment'}});break
                    target_groups=[group for group,points in sorted(contacts,key=lambda pair:min(math.dist(p,q) for p,_ in sources for q,_ in pair[1]))]
                    compact=None
                    for target in target_groups:
                        remaining=None if maximum is None else max(0,maximum-existing-sum(v['net']==net for v in vias))
                        frontier_detail={}
                        def record_frontier(regions):
                            from diagnose_routing_topology import boundary_attribution
                            blockers={};reached=[]
                            for layer,region in regions:
                                check_time()
                                boundary=boundary_attribution(region,layer,items,dict(spec,net=net),data)
                                reached.append({'layer':layer,'area_mm2':region.area,'bounds_mm':list(region.bounds)})
                                for row in boundary['boundary_net_groups']:
                                    if not row['editable_copper_uuids']:continue
                                    entry=blockers.setdefault(row['net'],{'net':row['net'],'name':row['name'],'boundary_support_mm':0,'primitive_uuids':[]})
                                    entry['boundary_support_mm']+=row['boundary_support_mm']
                                    entry['primitive_uuids'].extend(row['editable_copper_uuids'])
                            for entry in blockers.values():entry['primitive_uuids']=sorted(set(entry['primitive_uuids']))
                            frontier_detail.update(source_board_sha256=data.get('source_board_sha256'),reached_components=reached,
                                blocking_copper=sorted(blockers.values(),key=lambda row:(-row['boundary_support_mm'],row['net'])))
                        generator=first_via_prefix if escape_prefix else find_polygon_path
                        preference={} if escape_prefix or net not in preferred else {
                            'preferred_layer':preferred[net],
                            'via_spacing':spec['via_drill']+data.get('hole_to_hole',data['hole_clearance'])}
                        compact,detail=generator(lambda radius,**kw:domains(net,radius,**kw),endpoints,
                            groups[0],target,layers,spec['width'],spec['via_diameter']/2,remaining,step,check_time,record_frontier,**preference)
                        detail.update(frontier_detail)
                        continuous_trials.append({'net':net,**detail})
                        if compact:break
                    if compact is None:
                        failed.append({'net':net,'name':spec['name'],'remaining_components':len(groups),'search':detail});break
                else:
                    legal=domains(net,spec['width']/2)
                    attachment_legal=domains(net,spec['width']/2,padding=0)
                    via_legal=domains(net,spec['via_diameter']/2,True)
                    via_mask=np.logical_and.reduce([shapely.contains_xy(g,xs,ys) for g in via_legal])
                    # Start at an existing locked tip when present; otherwise grow the
                    # first connected component toward the nearest unconnected pad.
                    groups.sort(key=lambda g:not any(i['kind']=='copper' for i in g))
                    sources=endpoints(groups[0],attachment_legal)
                    target_groups=sorted(groups[1:],key=lambda g:min(math.dist(p,q) for p,_ in sources for q,_ in endpoints(g,attachment_legal)))
                    found=None
                    for target_index,target in enumerate(target_groups):
                        remaining=None if maximum is None else max(0,maximum-existing-sum(v['net']==net for v in vias))
                        now=time.perf_counter()
                        attempt_deadline=now+(deadline-now)/(len(target_groups)-target_index)
                        found=find_path(sources,endpoints(target,attachment_legal),legal,attachment_legal,via_mask,remaining,preferred.get(net),attempt_deadline,net)
                        if found:break
                    if not found:
                        failed.append({'net':net,'name':spec['name'],'remaining_components':len(groups),'search':last_search_failure})
                        break
                    path,start,end=found
                    # Collapse only collinear steps. No shortcut may cross an obstacle.
                    points=[(layers[path[0][0]],list(start))]+[(layers[s[0]],xy(s)) for s in path]+[(layers[path[-1][0]],list(end))]
                    compact=[]
                    for layer,p in points:
                        if compact and compact[-1]==(layer,p):continue
                        if len(compact)>=2 and compact[-1][0]==compact[-2][0]==layer:
                            a,b=compact[-2][1],compact[-1][1]
                            if abs((b[0]-a[0])*(p[1]-b[1])-(b[1]-a[1])*(p[0]-b[0]))<1e-9 and (b[0]-a[0])*(p[0]-b[0])+(b[1]-a[1])*(p[1]-b[1])>=0:
                                compact.pop()
                        compact.append((layer,p))
                    # Pull each same-layer path taut inside the checked native
                    # clearance domain. Keep every layer transition and attachment.
                    # This removes raster staircases before they become obstacles.
                    pulled=[];index=0
                    while index<len(compact):
                        layer,a=compact[index];pulled.append((layer,a))
                        last=index
                        while last+1<len(compact) and compact[last+1][0]==layer:last+=1
                        if last==index:index+=1;continue
                        end=last
                        while end>index+1 and not attachment_legal[layers.index(layer)].covers(LineString([a,compact[end][1]])):
                            end-=1
                        index=end
                    compact=pulled
                try:
                    new_tracks,new_vias,new_item=append_path(context,net,compact,validate=continuous)
                except ValueError as error:
                    failed.append({'net':net,'name':spec['name'],'remaining_components':len(groups),
                                   'search':{'reason':str(error)}});break
                tracks.extend(new_tracks);vias.extend(new_vias)
                if escape_prefix:break  # Partial candidate, never mark a complete net.
                groups[0].extend(target+[new_item]);groups.remove(target)
            else:
                if not escape_prefix:completed.append(net)
    except TimeoutError as e:
        failed.append({'reason':str(e)})
    return {'tracks':tracks,'vias':vias,'completed_nets':completed,'failed':failed,'grid_mm':requested_step,'net_grid_mm':net_grid_mm,
            'matched_layer_preferences':preferred,'source_board_sha256':data.get('source_board_sha256'),
            'search_bounds_mm':[x0,y0,x1,y1],'search_windows':search_windows,
            'elapsed_seconds':time.perf_counter()-started,'candidate_only':True,**({'continuous_trials':continuous_trials} if continuous else {}),
            **({'escape_prefix_only':True} if escape_prefix else {})}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path);p.add_argument('--seconds',type=float,default=30);p.add_argument('--step',type=float,default=.1)
    p.add_argument('--continuous',action='store_true',help='Generate guarded continuous polygon candidates using the same native domains')
    args=p.parse_args()
    data=json.loads((args.directory/'geometry.json').read_text())
    plan_path=args.directory/'route-nets.json'
    plan=json.loads(plan_path.read_text()) if plan_path.exists() else {}
    data['repair_windows']=plan.get('windows',{})
    for net,width in plan.get('widths',{}).items():
        data['nets'][net]['width']=max(data['nets'][net]['width'],width)
    result=route(data,args.step,args.seconds,only_nets=plan.get('nets'),continuous=args.continuous)
    (args.directory/'grid-route.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('tracks','vias')}))


if __name__=='__main__':main()
