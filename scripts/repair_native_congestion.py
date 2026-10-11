"""Bounded local rip-up/reroute, accepted only by native and profile checks."""
import json
import hashlib
from itertools import combinations,permutations
import math
from pathlib import Path
import shutil
import subprocess
import time
import shapely
from shapely.geometry import LineString,Point
from grid_route import route,shape
from native_project import copy_assets
from routing_screen import screen,nonregression


def repair_plans(g,drc,protected=()):
    """Rank small neighborhoods around native-reported missing connections."""
    names={int(c):n['name'] for c,n in g['nets'].items()}
    pads={c:[p for p in g['pads'] if p['net']==c] for c in names}
    locked={t['net'] for t in g['tracks']+g['vias'] if t.get('locked',True)}
    locked.update(c for c,name in names.items() if name in protected)
    targets={c for c,name in names.items() if c not in locked and any('['+name+']' in item['description']
             for issue in drc.get('unconnected_items',[]) for item in issue['items'])}
    plans=[]
    for target in sorted(targets,key=lambda c:(len(pads[c])>8,c)):
        points=[p['at'] for p in pads[target]] if len(pads[target])<=8 else [
            [item['pos']['x'],item['pos']['y']] for issue in drc.get('unconnected_items',[]) for item in issue['items']
            if item['description'].startswith('Pad ') and '['+names[target]+']' in item['description']]
        if not points:continue
        nearby={}
        for item in g['vias']+g['tracks']:
            neighbor=item['net']
            if neighbor==target or neighbor in locked or not 1<len(pads.get(neighbor,[])):continue
            geom=Point(item['at']) if 'at' in item else LineString([item['start'],item['end']])
            distance=min(geom.distance(Point(p)) for p in points)
            if distance<=3:nearby[neighbor]=min(nearby.get(neighbor,math.inf),distance)
        neighbors=sorted(nearby,key=lambda n:(nearby[n],n))[:6]
        # Interleave individual moves with groups: exhausting every single-net
        # move first can consume the budget while two blockers remain in place.
        groups=[(n,) for n in neighbors[:3]]+list(combinations(neighbors[:4],2))+[(n,) for n in neighbors[3:]]+list(combinations(neighbors[:4],3))
        plans.extend((target,group) for group in groups)
    return plans


def frontier_plans(g,target,search,neighbors=(),protected=()):
    """Expand a repair only with copper touching a failed search's frontier."""
    locked={t['net'] for t in g['tracks']+g['vias'] if t.get('locked',True)}
    locked.update(int(c) for c,n in g['nets'].items() if n['name'] in protected)
    pads={int(c):sum(p['net']==int(c) for p in g['pads']) for c in g['nets']}
    blockers=[b['net'] for b in search.get('blocking_copper',[]) if b['net'] not in locked
              and b['net']!=target and b['net'] not in neighbors and pads.get(b['net'],0)>1][:3]
    return [(target,tuple(sorted((*neighbors,*group)))) for count in range(1,len(blockers)+1)
            for group in combinations(blockers,count) if len(neighbors)+count<=5]


def routing_orders(target,neighbors):
    basic=[(target,*neighbors),(*neighbors,target)]
    # Exhaust the small joint escape, not every permutation of a large netlist.
    if len(neighbors)<=2:
        basic.extend(permutations((target,*neighbors)))
    return list(dict.fromkeys(basic))


def next_repair_option(options,attempts,neighborhood_attempts=None):
    """Give each open net a turn before repeating one on unchanged geometry."""
    neighborhood_attempts=neighborhood_attempts or {}
    return min(options,key=lambda option:(attempts.get(option[0],0),
               neighborhood_attempts.get((option[0],tuple(sorted(option[1]))),0)))


def diagnosed_primitives(frontiers,target,neighbor,source_sha256):
    """A stale/empty measured cut cannot become an unrestricted removal."""
    evidence=frontiers.get(target)
    if evidence is None:return None  # Existing proximity fallback has no cut.
    if evidence['source_board_sha256']!=source_sha256:return []
    return evidence['nets'].get(neighbor,[])


def whole_cut_preserved(before,after,nets,points,removed):
    return local_copper_preserved(before,after,nets,points,removed=removed)


def local_removals(g,net,points,primitive_uuids=None,retain_crossing=False):
    """Open a small escape window, preserving the rest of a large copper net."""
    window=shapely.union_all([Point(p).buffer(3) for p in points])
    result={'tracks':[],'vias':[],'remove':[]}
    for track in g['tracks']:
        if track['net']!=net or track.get('locked',True) or track['arc']:continue
        if primitive_uuids is not None and track['uuid'] not in primitive_uuids:continue
        line=LineString([track['start'],track['end']])
        if not line.intersects(window):continue
        if retain_crossing and not window.covers(line.buffer(track['width']/2,quad_segs=32)):continue
        result['remove'].append(track['uuid'])
        outside=line.difference(window)
        for part in shapely.get_parts(outside):
            if part.geom_type=='LineString' and part.length>1e-6:
                result['tracks'].append({k:track[k] for k in ('net','width','layer')}|
                    {'start':list(part.coords[0]),'end':list(part.coords[-1])})
    result['remove'].extend(v['uuid'] for v in g['vias'] if v['net']==net and not v.get('locked',True)
        and (primitive_uuids is None or v['uuid'] in primitive_uuids)
        and window.covers(Point(v['at']).buffer(v['diameter_mm']/2,quad_segs=32) if retain_crossing else Point(v['at'])))
    return result


def local_copper_preserved(before,after,nets,points,removed=None):
    """Check native round trips and final reroutes, per net AND physical layer."""
    window=shapely.union_all([Point(p).buffer(3) for p in points]);rows=[]
    old=before['tracks']+before['vias'];new=after['tracks']+after['vias']
    actual={i['uuid']:i for i in new};source={i['uuid']:i for i in old}
    if not before.get('layers') and any('at' in i and i['net'] in nets for i in old+new):
        return {'pass':False,'reason':'Via outside-Cu checks require the native layer stack','checks':[]}
    def physical(item):
        return (Point(item['at']).buffer(item['diameter_mm']/2,quad_segs=32) if 'at' in item else
                LineString([item['start'],item['end']]).buffer(item['width']/2,quad_segs=32))
    def native(p):return tuple(round(v*1e6) for v in p)
    for net in nets:
        originals=[i for i in old if i['net']==net];current=[i for i in new if i['net']==net]
        # Arcs cannot be reconstructed from endpoint metadata. Never edit them.
        unchanged=all(actual.get(i['uuid'])==i for i in originals if i.get('locked',True) or i.get('arc') or
                      not physical(i).intersects(window) or ('at' in i and not window.covers(physical(i))))
        unselected=removed is None or all(actual.get(i['uuid'])==i for i in originals if i['uuid'] not in removed)
        added=[i for i in current if i['uuid'] not in source]
        whole_added=removed is None or all(not i.get('arc') and window.covers(physical(i)) for i in added)
        exact=all(not i.get('arc') or source.get(i['uuid'])==i for i in current)
        for item in current:
            if 'at' in item or item.get('arc') or window.covers(physical(item)):continue
            a,b=native(item['start']),native(item['end'])
            def collinear(source):
                if 'at' in source or source.get('arc') or source['width']!=item['width'] or source['layer']!=item['layer']:return False
                p,q=native(source['start']),native(source['end']);dx,dy=q[0]-p[0],q[1]-p[1]
                return all((r[0]-p[0])*dy==(r[1]-p[1])*dx for r in (a,b))
            exact &= source.get(item['uuid'])==item or any(collinear(i) for i in originals)
        for item in originals:
            if 'at' in item or item.get('arc'):continue
            endpoints={native(p) for t in current if 'at' not in t and t['layer']==item['layer'] and t['width']==item['width'] for p in (t['start'],t['end'])}
            exact &= all(window.covers(Point(p)) or native(p) in endpoints for p in (item['start'],item['end']))
        layers=before.get('layers') or sorted({i['layer'] for i in originals+current if 'at' not in i})
        for layer in layers:
            def copper(items):return shapely.union_all([physical(i) for i in items if 'at' in i or i['layer']==layer]).difference(window)
            a,b=copper(originals),copper(current)
            missing=a.difference(b).area;added=b.difference(a).area
            rows.append({'net':net,'layer':layer,'missing_outside_mm2':missing,'added_outside_mm2':added,
                         'exact_native_lines_and_anchors':bool(exact),'unchanged_protected_primitives':unchanged,
                         'unselected_primitives_exact':unselected,'whole_cut_additions_inside':whole_added,
                         'pass':bool(exact and unchanged and unselected and whole_added and missing<=1e-10 and added<=1e-10)})
    return {'pass':all(r['pass'] for r in rows),'scope':'Track/via Cu outside the exact local window, layer-specific; refill caches excluded.',
            'polygon_area_tolerance_mm2':1e-10,'checks':rows}


def plane_partitions_preserved(before,after,nets,points):
    """Retained outside copper and fixed pad contacts cannot split or vanish."""
    from stitch_native_planes import plane_components
    window=shapely.union_all([Point(p).buffer(3) for p in points]);rows=[]
    for net in nets:
        old=plane_components(before,net);new=plane_components(after,net)
        for index,component in enumerate(old):
            linked=set();lost=0.0;contact_area=0.0;outside_area=0.0;pad_area=0.0;pad_lost=0.0
            for layer,poly in component.items():
                outside=poly.difference(window);outside_area+=outside.area
                # Retained filled copper also counts, so a zone-only island
                # cannot evade the proof merely by having no pad terminal.
                for j,group in enumerate(new):
                    if layer in group and outside.intersection(group[layer]).area>1e-10:linked.add(j)
                # Fixed pads and preserved track/via contacts must not vanish.
                contacts=shapely.union_all([shape(item['shapes'][str(layer)]).intersection(poly).difference(window)
                    for item in before['items'] if item['net']==net and str(layer) in item['shapes']])
                # Pad geometry is fixed even inside the edit window. A newly
                # split pad-connected source component must be restored.
                pads=shapely.union_all([shape(item['shapes'][str(layer)]).intersection(poly)
                    for item in before['items'] if item['net']==net and item.get('kind')=='pad' and str(layer) in item['shapes']])
                for j,group in enumerate(new):
                    if layer in group and pads.intersection(group[layer]).area>1e-10:linked.add(j)
                final=shapely.union_all([group[layer] for group in new if layer in group])
                contact_area+=contacts.area;pad_area+=pads.area
                lost+=contacts.difference(final).area;pad_lost+=pads.difference(final).area
            rows.append({'net':net,'source_component':index,'final_components':sorted(linked),
                         'outside_native_contact_mm2':contact_area,'lost_outside_contact_mm2':lost,'outside_source_copper_mm2':outside_area,
                         'fixed_pad_contact_mm2':pad_area,'lost_fixed_pad_contact_mm2':pad_lost,
                         'pass':(len(linked)==1 if outside_area>0 or pad_area>0 else not linked) and lost<=1e-10 and pad_lost<=1e-10})
    return {'pass':all(row['pass'] for row in rows),'scope':'Actual exported filled-Cu/via components; retained outside-window copper and all fixed pad contacts of each source component stay connected, merging allowed; not component-count equality.',
            'polygon_area_tolerance_mm2':1e-10,'checks':rows}


def repair(directory,worker,kicad,profile,seconds=180,protected=()):
    from solve_kicad import audit_preservation,summarize_drc
    directory=Path(directory);started=time.perf_counter();trials=[];seen=set();guided=[];diagnosed=set();layer_trials=[];attempts={};neighborhood_attempts={};frontier_ids={}
    (directory/'congestion-context.json').write_text(json.dumps({
        'source_board_sha256':hashlib.sha256((directory/'candidate.kicad_pcb').read_bytes()).hexdigest(),
        'selection':'reached copper frontier, layer backtracking, then proximity fallback',
        'timeout_seconds':seconds,'trial_limit':32,'protected_nets':list(protected),
        'script_sha256':{name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest()
                         for name in ('grid_route.py','repair_native_congestion.py')},
        'manual_trace_coordinates':False},indent=2)+'\n')
    def run(args,log):
        assets={suffix:(log.parent/('candidate'+suffix)).read_bytes() for suffix in ('.kicad_pro','.kicad_dru')
                if (log.parent/('candidate'+suffix)).exists()}
        try:
            with log.open('w') as f:subprocess.run(list(map(str,args)),stdout=f,stderr=subprocess.STDOUT,check=True,timeout=30)
        finally:
            for suffix,value in assets.items():(log.parent/('candidate'+suffix)).write_bytes(value)
    def geometry(d,label):
        run(worker+['dc-geometry',d],d/(label+'.log'))
        g=json.loads((d/'dc-geometry.json').read_text());g['routing_profile']=profile;return g
    while time.perf_counter()-started<seconds and len(trials)<32:
        g=geometry(directory,'congestion-geometry');before=screen(g,profile)
        drc=json.loads((directory/'routed-drc.json').read_text())
        names={int(c):n['name'] for c,n in g['nets'].items()}
        pads={c:[p for p in g['pads'] if p['net']==c] for c in names}
        proximity=repair_plans(g,drc,protected)
        for target in dict.fromkeys(t for t,_ in proximity):
            if target in diagnosed or len(pads[target])>8:continue
            remaining=seconds-(time.perf_counter()-started)
            if remaining<=0:break
            diagnosed.add(target)
            diagnosis=route(g,step=.025,seconds=min(10,remaining),only_nets=[target])
            (directory/('congestion-diagnosis-'+str(target)+'.json')).write_text(json.dumps(diagnosis,indent=2))
            for failure in diagnosis['failed']:
                if failure.get('net')==target:
                    search=failure.get('search') or {}
                    guided.extend(frontier_plans(g,target,search,protected=protected))
                    frontier_ids[target]={'source_board_sha256':hashlib.sha256((directory/'candidate.kicad_pcb').read_bytes()).hexdigest(),
                        'nets':{b['net']:b['primitive_uuids'] for b in search.get('blocking_copper',[]) if 'primitive_uuids' in b}}
        if time.perf_counter()-started>=seconds:break
        options=[option for option in layer_trials if option not in seen]+[
                 (target,neighbors,order,None) for target,neighbors in guided+proximity
                 for order in routing_orders(target,neighbors) if (target,neighbors,order,None) not in seen]
        if not options:break
        option=next_repair_option(options,attempts,neighborhood_attempts)
        target,neighbors,order,preferred_layer=option;seen.add(option)
        attempts[target]=attempts.get(target,0)+1
        neighborhood=(target,tuple(sorted(neighbors)))
        neighborhood_attempts[neighborhood]=neighborhood_attempts.get(neighborhood,0)+1
        trial_started=time.perf_counter()
        d=directory/f'congestion-{len(trials):02}';d.mkdir();copy_assets(directory,d)
        for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
            if (directory/('candidate'+suffix)).exists():shutil.copy2(directory/('candidate'+suffix),d/('candidate'+suffix))
        trial={'target':names[target],'displaced':', '.join(names[n] for n in neighbors),'displaced_nets':[names[n] for n in neighbors],'neighbors_first':order[0]!=target,'route_order':[names[n] for n in order],'preferred_layer':preferred_layer,'accepted':False};trials.append(trial)
        # Preserve neighbors' copper outside the obstructed escape. Removing
        # their complete routing discards usable anchors and creates avoidable
        # full-board routing demands, even for a two-pad net.
        rip=[target] if len(pads[target])<=8 else []
        (d/'route-nets.json').write_text(json.dumps({'nets':rip}))
        try:
            run(worker+['rip-nets',d],d/'remove.log')
            points=[p['at'] for p in pads[target]] if len(pads[target])<=8 else [
                [item['pos']['x'],item['pos']['y']] for issue in drc['unconnected_items'] for item in issue['items']
                if item['description'].startswith('Pad ') and '['+names[target]+']' in item['description']]
            removed_neighbor_ids=set();whole_cut=True
            for neighbor in neighbors:
                ids=diagnosed_primitives(frontier_ids,target,neighbor,hashlib.sha256((directory/'candidate.kicad_pcb').read_bytes()).hexdigest())
                whole_cut &= ids is not None
                removal=local_removals(g,neighbor,points,primitive_uuids=ids,retain_crossing=ids is not None)
                removed_neighbor_ids.update(removal['remove'])
                (d/('local-remove-'+str(neighbor)+'.json')).write_text(json.dumps(removal))
                (d/'grid-route.json').write_text(json.dumps(removal));run(worker+['apply-grid',d],d/('local-remove-'+str(neighbor)+'.log'))
            strict_ids=removed_neighbor_ids if whole_cut else None
            preparation=local_copper_preserved(g,geometry(d,'local-preservation'),neighbors,points,removed=strict_ids)
            (d/'local-preparation-preservation.json').write_text(json.dumps(preparation,indent=2))
            if not preparation['pass']:raise ValueError('Local cut changed preserved copper')
            window=shapely.union_all([Point(p).buffer(3) for p in points])
            window_shapes=[{'shell':list(p.exterior.coords),'holes':[list(h.coords) for h in p.interiors]}
                           for p in shapely.get_parts(window)]
            for index,net in enumerate(order):
                current=geometry(d,'geometry-'+str(index))
                # Propose only copper the final independent edit guard admits.
                current['candidate_only_native_copper_windows']={str(n):window_shapes for n in neighbors}
                if preferred_layer is not None:
                    allowed=next((limit.get('allowed_layers') for limit in profile.get('net_limits',[]) if limit['net']==names[net]),None)
                    if allowed is None or current['layer_names'][str(preferred_layer)] in allowed:
                        current['preferred_layers']={str(net):preferred_layer}
                if net!=target and any(z['net']==net and not z.get('teardrop') for z in current.get('zones',[])):
                    continue  # Refilled planes and native connectivity, below, decide this net.
                if index<len(order)-1:
                    # Leave via clearance beside displaced pads. A new via
                    # must not make the neighbor impossible to reconnect.
                    for neighbor in order[index+1:]:
                        spec=current['nets'][str(neighbor)]
                        guard=spec['width']/2+spec['clearance']
                        for pad in current['pads']:
                            if pad['net']!=neighbor:continue
                            for polygons in pad['shapes'].values():
                                area=shape(polygons).buffer(guard)
                                parts=list(area.geoms) if hasattr(area,'geoms') else [area]
                                current['keepouts'].append({'layers':current['layers'],'tracks':False,'vias':True,
                                    'shapes':[{'shell':list(p.exterior.coords),'holes':[list(h.coords) for h in p.interiors]} for p in parts]})
                remaining=seconds-(time.perf_counter()-started)
                if remaining<=0:raise ValueError('Congestion repair budget exhausted')
                budget=min(20,remaining/(len(order)-index))
                routed=route(current,step=.025,seconds=budget,only_nets=[net])
                if net not in routed['completed_nets']:
                    # Compose the continuous candidate with this local endpoint
                    # repair before its native gate, never adopt an equal-score
                    # intermediate merely to create a future routing opportunity.
                    (d/('search-'+str(index)+'-grid.json')).write_text(json.dumps(routed))
                    left=seconds-(time.perf_counter()-started)
                    if left>0:
                        polygon=route(current,step=.025,seconds=min(30,left/(len(order)-index)),only_nets=[net],continuous=True)
                        (d/('search-'+str(index)+'-continuous.json')).write_text(json.dumps(polygon))
                        if net in polygon['completed_nets']:routed=polygon
                clipped=any(f.get('net')==net and (f.get('search') or {}).get('touches_search_boundary')
                            for f in routed['failed'])
                if net not in routed['completed_nets'] and clipped:
                    # A finite search window is not a physical obstacle. Retain
                    # the first miss and try the board with the same rules.
                    (d/('search-'+str(index)+'-cropped.json')).write_text(json.dumps(routed))
                    current['repair_windows']={str(net):list(shape(current['outline']).bounds)}
                    remaining=seconds-(time.perf_counter()-started)
                    if remaining<=0:raise ValueError('Congestion repair budget exhausted')
                    budget=min(60,remaining/(len(order)-index))
                    expansion_started=time.perf_counter()
                    expanded=route(current,step=.075,seconds=min(20,budget),
                                   max_states=32_000_000,only_nets=[net])
                    if net not in expanded['completed_nets']:
                        (d/('search-'+str(index)+'-full-coarse.json')).write_text(json.dumps(expanded))
                        left=min(seconds-(time.perf_counter()-started),budget-(time.perf_counter()-expansion_started))
                        if left>0:
                            expanded=route(current,step=.025,seconds=left,
                                           max_states=32_000_000,only_nets=[net])
                    expanded['expanded_after_boundary_miss']=True;expanded['expanded_to_full_board']=True
                    routed=expanded
                (d/('search-'+str(index)+'.json')).write_text(json.dumps(routed))
                (d/'grid-route.json').write_text(json.dumps(routed))
                run(worker+['apply-grid',d],d/('apply-'+str(index)+'.log'))
                if net not in routed['completed_nets']:
                    trial['failed_net']=names[net]
                    for failure in routed['failed']:
                        if failure.get('net')!=net:continue
                        trial['search']=failure.get('search')
                        expansions=frontier_plans(g,target,failure.get('search') or {},neighbors,protected)
                        guided=expansions+guided
                        # A route just generated for another member can itself
                        # close this pocket. Try a different legal layer choice
                        # before ripping more unrelated existing copper.
                        blockers={b['net'] for b in (failure.get('search') or {}).get('blocking_copper',[])}
                        if preferred_layer is None and blockers.intersection(order[:index]):
                            reached_layers={p['layer'] for p in (failure.get('search') or {}).get('pocket_bounds',[])}
                            choices=sorted(g['layers'],key=lambda layer:layer in reached_layers)
                            orders=routing_orders(target,neighbors)
                            layer_trials.extend((target,neighbors,other_order,None) for other_order in orders)
                            layer_trials.extend((target,neighbors,other_order,layer)
                                                for other_order in orders for layer in choices)
                    raise ValueError(('Target' if net==target else 'Displaced neighbor')+' did not reconnect')
            run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'routed-drc.json',d/'candidate.kicad_pcb'],d/'drc.log')
            from stitch_native_planes import stitch
            trial['plane_stitching']=stitch(d,worker,kicad,profile=profile,protected=protected)
            final_geometry=geometry(d,'checked');after=screen(final_geometry,profile)
            local_preservation=local_copper_preserved(g,final_geometry,neighbors,points,removed=strict_ids)
            (d/'local-final-preservation.json').write_text(json.dumps(local_preservation,indent=2))
            plane_nets=sorted({z['net'] for z in g.get('zones',[]) if z['net'] and not z.get('teardrop')})
            partitions=plane_partitions_preserved(g,final_geometry,plane_nets,points)
            (d/'local-plane-partitions.json').write_text(json.dumps(partitions,indent=2))
            (d/'routing-screen.json').write_text(json.dumps(after,indent=2))
            a=summarize_drc(directory/'routed-drc.json');b=summarize_drc(d/'routed-drc.json')
            disconnected=json.loads((d/'routed-drc.json').read_text()).get('unconnected_items',[])
            neighbor_open=any('['+names[n]+']' in item['description'] for n in neighbors if n not in plane_nets for issue in disconnected for item in issue['items'])
            from optimize_kicad import unchanged_other_nets
            other_copper_preserved=unchanged_other_nets(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb',[names[n] for n in (target,*neighbors)])
            accepted=(other_copper_preserved and local_preservation['pass'] and partitions['pass'] and not neighbor_open and b['unconnected']<a['unconnected'] and b['total_findings']<a['total_findings'] and
                      all(count<=a['types'].get(kind,0) for kind,count in b['types'].items()) and
                      nonregression(before,after) and audit_preservation(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb')['pass'])
            trial.update(before=a,after=b,routing_nonregression=nonregression(before,after),other_nets_unchanged=other_copper_preserved,local_copper_preservation=local_preservation,plane_component_preservation=partitions,accepted=accepted)
            if accepted:
                for name in ('candidate.kicad_pcb','routed-drc.json'):shutil.copy2(d/name,directory/name)
                # The board and its opens changed. Discard plans derived from
                # the previous geometry rather than spending the rest of the
                # budget rerouting a now-complete target.
                guided.clear();layer_trials.clear();diagnosed.clear();seen.clear();attempts.clear();neighborhood_attempts.clear();frontier_ids.clear()
        except (ValueError,subprocess.SubprocessError) as error:trial['reason']=str(error)
        trial['elapsed_seconds']=time.perf_counter()-trial_started
        (directory/'congestion-repair.json').write_text(json.dumps(trials,indent=2)+'\n')
    return trials


def native_open_counts(geometry,drc):
    """Count enabled native missing connections by exact exported net name."""
    return {int(code):sum(any('['+spec['name']+']' in item['description']
            for item in issue['items']) for issue in drc.get('unconnected_items',[]))
            for code,spec in geometry['nets'].items() if spec['name']}


def no_new_native_findings(before,after):
    """A smaller count cannot hide a different physical error or warning."""
    from collections import Counter
    key=lambda v:json.dumps({k:v[k] for k in ('description','type','severity','items') if k in v},sort_keys=True)
    old=Counter(key(v) for v in before.get('violations',[])+before.get('schematic_parity',[]))
    new=Counter(key(v) for v in after.get('violations',[])+after.get('schematic_parity',[]))
    return not (new-old)


def exact_ring_cycle(ring):
    """Exact integer cycle; accept only cyclic start and whole reversal."""
    if len(ring)<3 or any(len(p)!=2 or any(type(v)is not int for v in p)for p in ring):
        raise ValueError('Invalid integer native ring')
    values=tuple(tuple(p)for p in ring)
    candidates=[]
    for sequence in (values,tuple(reversed(values))):
        least=min(sequence)
        candidates.extend(sequence[i:]+sequence[:i]for i,p in enumerate(sequence)if p==least)
    return min(candidates)


def exact_contact_polygon_row(a,b):
    if not isinstance(a,dict)or not isinstance(b,dict)or a.keys()!=b.keys():return False
    if any(a[k]!=b[k]for k in a if k!='polygon_nm'):return False
    x=a['polygon_nm'];y=b['polygon_nm']
    if len(x)!=len(y):return False
    for left,right in zip(x,y):
        if left.keys()!=right.keys()or set(left)!={'shell','holes'}:return False
        if len(left['holes'])!=len(right['holes']):return False
        if exact_ring_cycle(left['shell'])!=exact_ring_cycle(right['shell']):return False
        if any(exact_ring_cycle(p)!=exact_ring_cycle(q)for p,q in zip(left['holes'],right['holes'])):return False
    return True


def native_partition_preservation(before,after,removed,allowed):
    """Every original Cu/pad component's complete survivors must stay joined."""
    removed=set(removed);old=before['native_items'];new=after['native_items'];rows=[]
    cut_valid=all(uid in old and uid not in new and old[uid]['class'] in ('PCB_TRACK','PCB_VIA')
        and not old[uid].get('locked',True) and old[uid]['net'] in allowed for uid in removed)
    exact=all(new.get(uid)==record for uid,record in old.items() if uid not in removed)
    additions=all(record['class'] in ('PCB_TRACK','PCB_VIA') and record['net'] in allowed
        for uid,record in new.items() if uid not in old)
    physical=before.get('contact_schema')=='physical-Cu-islands/1'
    physical_exact=not physical or (all(exact_contact_polygon_row(after['contact_nodes'].get(node),row)
        for node,row in before['contact_nodes'].items() if row['uuid'] not in removed)
        and all(row['uuid'] not in old or node in before['contact_nodes']
            for node,row in after['contact_nodes'].items()))
    declared_ties_exact=before.get('declared_net_tie_contacts',[])==after.get('declared_net_tie_contacts',[])
    contacts_valid=(after.get('contact_schema')=='physical-Cu-islands/1'
        and before.get('contact_graph_unambiguous') is True and after.get('contact_graph_unambiguous') is True
        and physical_exact and declared_ties_exact) if physical else not any(r['class']=='ZONE' for r in list(old.values())+list(new.values()))
    for net,groups in before['groups'].items():
        for group in groups:
            if physical:
                keep={node for node in group['contact_nodes'] if before['contact_node_items'][node] not in removed}
                matches=[g for g in after['groups'].get(net,[]) if keep<=set(g.get('contact_nodes',[]))]
            else:
                keep=set(group['native_item_uuids'])-removed
                matches=[g for g in after['groups'].get(net,[]) if keep<=set(g['native_item_uuids'])]
            rows.append({'net':net,'surviving_item_count':len(keep),'pass':not keep or len(matches)==1})
    clean=not any(before.get(key) or after.get(key) for key in ('cross_net_contacts','unrepresented_net_items'))
    return {'pass':cut_valid and exact and additions and clean and contacts_valid and all(r['pass'] for r in rows),
        'measured_cuts_valid':cut_valid,'unselected_native_items_exact':exact,
        'allowed_additions_only':additions,'native_contacts_clean':clean,
        'physical_contact_schema_valid':contacts_valid,'surviving_contact_polygons_exact':physical_exact,
        'declared_net_tie_contacts_exact':declared_ties_exact,'partitions':rows}


def boundary_cut_choices(boundary,primitives,layers,locked,target,maximum):
    """Try connected same-owner boundary paths before isolated fragments."""
    groups=[];support={}
    for row in boundary['rows']:
        if not row['editable_copper']:continue
        for item in row['native_primitives']:
            uid=item['uuid'];primitive=primitives.get(uid)
            if primitive is None or primitive['net'] in locked|{target} or primitive.get('arc'):continue
            support[uid]=support.get(uid,0)+row['boundary_support_mm']
            points={(layer,*point) for layer in (layers if 'at' in primitive else [primitive['layer']])
                    for point in ([primitive['at']] if 'at' in primitive else [primitive['start'],primitive['end']])}
            matches=[group for group in groups if group['net']==primitive['net'] and group['points']&points]
            for group in matches:groups.remove(group)
            groups.append({'net':primitive['net'],'points':points.union(*(group['points'] for group in matches)),
                'ids':{uid}.union(*(group['ids'] for group in matches))})
    def rank(ids):
        # Retaining vias preserves native drill shadows and their contact Cu.
        return (any('at' in primitives[uid] for uid in ids),-sum(support[uid] for uid in ids),len(ids),sorted(ids))
    paths=sorted((frozenset(group['ids']) for group in groups if 1<len(group['ids'])<=maximum),key=rank)
    singles=sorted((frozenset([uid]) for uid in support),key=rank)
    return paths,singles


def measured_owner_cut(geometry,selection,source_sha256,protected=()):
    """Resolve only hash-bound, unlocked primitives of the measured owners."""
    if selection.get('source_sha256')!=source_sha256 or geometry.get('source_board_sha256',source_sha256)!=source_sha256:
        raise ValueError('Measured cut belongs to different source copper')
    primitives={p['uuid']:p for p in geometry['tracks']+geometry['vias']}
    codes={int(c) for c,n in geometry['nets'].items() if n['name'] in selection['selected_nets']}
    locked={p['net'] for p in primitives.values() if p.get('locked',True)}|{
        int(c) for c,n in geometry['nets'].items() if n['name'] in protected}
    ids=selection['peeled_native_uuids']
    if not ids or len(ids)!=len(set(ids)) or codes&locked or any(uid not in primitives or
        primitives[uid]['net'] not in codes or primitives[uid].get('locked',True) or primitives[uid].get('arc') for uid in ids):
        raise ValueError('Invalid measured owner cut')
    return list(ids)


def measured_component_cuts(geometry,drc,profile,protected=(),seconds=15,maximum=4,states=24):
    """Search small actual component-boundary cuts while retaining target Cu.

    Graph passage is only candidate selection. Complete displaced-owner
    restoration and native survivor partitions decide adoption separately.
    """
    import copy
    from collections import Counter,deque
    from diagnose_routing_topology import model_scope,polygon_graph,boundary_attribution
    primitives={p['uuid']:p for p in geometry['tracks']+geometry['vias']}
    names={int(c):n['name'] for c,n in geometry['nets'].items()}
    locked={p['net'] for p in primitives.values() if p.get('locked',True)}|{c for c,n in names.items() if n in protected}
    counts=native_open_counts(geometry,drc);started=time.perf_counter();rows=[];selected=[]
    targets=deque(c for c in sorted(counts,key=lambda c:(sum(p['net']==c for p in geometry['pads']),c))
                  if counts[c] and c not in locked)
    queues={target:deque([frozenset()]) for target in targets};seen={target:set() for target in targets}
    while targets and len(rows)<states and time.perf_counter()-started<seconds:
        target=targets.popleft();queue=queues[target]
        while queue and queue[0] in seen[target]:queue.popleft()
        if not queue:continue
        cut=queue.popleft();seen[target].add(cut);g=copy.deepcopy(geometry)
        targets.append(target)
        for field in ('items','tracks','vias'):g[field]=[p for p in g[field] if p.get('uuid') not in cut]
        holes=Counter(tuple([*primitives[u]['at'],primitives[u]['drill_mm']/2]) for u in cut if 'at' in primitives[u])
        retained=[]
        for hole in g['holes']:
            key=tuple(hole.get('circle',()))
            if holes[key]:holes[key]-=1
            else:retained.append(hole)
        if any(holes.values()):raise ValueError('Selected via has no unique matching native drill')
        g['holes']=retained;scope=model_scope(g,profile,Path(__file__).with_name('grid_route.py'),preserve_source=True)
        spec=scope['specs'][str(target)];groups=scope['components'](target)
        domains=scope['domains'](target,spec['width']/2,padding=0)
        via=shapely.intersection_all(scope['domains'](target,spec['via_diameter']/2,via=True,padding=0))
        via_budget=scope['limits'].get(spec['name'],{}).get('max_vias')
        if via_budget is not None:via_budget=max(0,via_budget-sum(v['net']==target for v in g['vias']))
        graph=polygon_graph(domains,via,groups,g['layers'],maximum=via_budget)
        bad=[p for p in graph['terminal_pairs'] if not p['connected_under_via_limit']]
        row={'target':target,'target_name':names[target],'removed_native_uuids':sorted(cut),
             'owner_nets':sorted({primitives[u]['net'] for u in cut}),'graph_reachable':not bad,
             'remaining_new_via_budget':via_budget,'boundary':[]}
        rows.append(row)
        if not bad:
            if cut:
                selected.append(row)
                if len(selected)>=2:break
            continue
        if len(cut)>=maximum:continue
        parts={(l,j):p for l,d in zip(g['layers'],domains) for j,p in enumerate(shapely.get_parts(d))
            if p.geom_type=='Polygon' and p.area>1e-12}
        contacts={tuple(n) for pair in bad for group in pair['groups'] for n in graph['terminal_contacts'][group]}
        if not contacts:continue
        node=min(contacts,key=lambda n:(parts[n].area,n))
        boundary=boundary_attribution(parts[node],node[0],scope['items'],dict(spec,net=target),g)
        row['boundary'].append({'node':node,'area_mm2':parts[node].area,**boundary})
        paths,singles=boundary_cut_choices(boundary,primitives,g['layers'],locked,target,maximum-len(cut))
        # A complete measured path is a useful next state even when its
        # individual segments cannot open the passage. Keep singleton
        # alternatives, and give another target its turn between states.
        queue.extendleft(reversed([cut|ids for ids in paths if ids-cut]))
        queue.extend(cut|ids for ids in singles if ids-cut and len(cut|ids)<=maximum)
    return {'target_copper_retained':True,'manual_trace_coordinates':False,'rows':rows,'selected':selected,
        'states_examined':len(rows),'elapsed_seconds':time.perf_counter()-started,
        'physical_infeasibility_claim':False}


def measured_owner_cluster(geometry,drc,profile,protected=(),maximum=5,seconds=15,excluded_targets=()):
    """Peel actual boundary UUIDs while retaining every other owner primitive.

    Whole-owner removal too early hides deeper separating copper. Peeling is
    diagnostic only: the joint candidate reconnects their complete surviving
    components, and final native/source/profile guards decide Cu adoption.
    """
    import copy
    from collections import Counter
    from diagnose_routing_topology import model_scope,polygon_graph,boundary_attribution
    names={int(c):n['name'] for c,n in geometry['nets'].items()}
    primitives=geometry['tracks']+geometry['vias']
    locked={p['net'] for p in primitives if p.get('locked',True)}|{c for c,n in names.items() if n in protected}
    counts=native_open_counts(geometry,drc);started=time.perf_counter()
    def without(ids):
        g=copy.deepcopy(geometry);removed=[p for p in primitives if p['uuid'] in ids]
        g['items']=[p for p in g['items'] if p.get('uuid') not in ids]
        g['tracks']=[p for p in g['tracks'] if p['uuid'] not in ids];g['vias']=[p for p in g['vias'] if p['uuid'] not in ids]
        circles=Counter(tuple([*p['at'],p['drill_mm']/2]) for p in removed if 'at' in p);holes=[]
        for h in g['holes']:
            circle=tuple(h.get('circle',()))
            if circles[circle]:circles[circle]-=1
            else:holes.append(h)
        g['holes']=holes;return g
    def demand(g,scope,net):
        spec=scope['specs'][str(net)];groups=scope['components'](net)
        domains=scope['domains'](net,spec['width']/2,padding=0)
        via=shapely.intersection_all(scope['domains'](net,spec['via_diameter']/2,via=True,padding=0))
        via_budget=scope['limits'].get(spec['name'],{}).get('max_vias')
        if via_budget is not None:via_budget=max(0,via_budget-sum(v['net']==net for v in g['vias']))
        graph=polygon_graph(domains,via,groups,g['layers'],maximum=via_budget)
        bad=[r for r in graph['terminal_pairs'] if not r['connected_under_via_limit']]
        boundary=None
        if bad:
            parts={(l,j):p for l,d in zip(g['layers'],domains) for j,p in enumerate(shapely.get_parts(d)) if p.geom_type=='Polygon' and p.area>1e-12}
            nodes={tuple(n) for row in bad for group in row['groups'] for n in graph['terminal_contacts'][group]}
            if nodes:
                node=min(nodes,key=lambda n:parts[n].area)
                boundary={'contact_pocket':list(node),**boundary_attribution(parts[node],node[0],scope['items'],dict(spec,net=net),g)}
        return graph,bad,boundary
    for target in sorted(counts,key=lambda c:(sum(p['net']==c for p in geometry['pads']),c)):
        if not counts[target] or target in locked or names[target] in excluded_targets or sum(p['net']==target for p in geometry['pads'])>8:continue
        selected={target};peeled=set();rounds=[]
        for iteration in range(16):
            if time.perf_counter()-started>=seconds:break
            g=without(peeled);scope=model_scope(g,profile,Path(__file__).with_name('grid_route.py'),preserve_source=True)
            graph,bad,boundary=demand(g,scope,target);blocked=target;graphs=[{'net':names[target],'graph':graph}]
            if not bad:
                # Independent pad-demand reachability is necessary; it is not
                # a claim that all these routes can be assigned simultaneously.
                graphs=[]
                for blocked in sorted(selected):
                    graph,bad,boundary=demand(g,scope,blocked);graphs.append({'net':names[blocked],'graph':graph})
                    if bad:break
                if not bad:
                    if len(selected)>1:return {'target':names[target],'selected_nets':[names[c] for c in sorted(selected)],'rounds':rounds,'demand_graphs':graphs,'peeled_native_uuids':sorted(peeled),'graph_reachable':True,'source_sha256':geometry.get('source_board_sha256')}
                    break
            if boundary is None:break
            rows=[r for r in boundary['rows'] if r['editable_copper'] and r['net'] not in locked]
            new=set();owners=set()
            for row in rows:
                if row['net'] not in selected|owners and len(selected|owners)>=maximum:continue
                owners.add(row['net']);new.update(p['uuid'] for p in row['native_primitives'])
            new-=peeled
            rounds.append({'selected_nets':[names[c] for c in sorted(selected)],'removed_native_uuids':sorted(peeled),'blocked_demand':names[blocked],'demand_graphs':graphs,'boundary':boundary,'new_boundary_native_uuids':sorted(new)})
            if not new:break
            selected.update(owners);peeled.update(new)
        if time.perf_counter()-started>=seconds:break
    return None


def chamfer_generated_hairpins(data,proposal,profile):
    """Correct new acute reversals/crossings inside the same guarded domain.

    Original copper/pads and generated vias are anchors. A same-width bridge
    between the two arms replaces redundant tails or fills a crossing notch;
    native checks remain
    the acceptance authority, including copper slivers and all pin demands.
    """
    import copy
    from diagnose_routing_topology import model_scope
    result=copy.deepcopy(proposal);tracks=result['tracks'];changes=[]
    if not data.get('source_board_sha256') or proposal.get('source_board_sha256')!=data['source_board_sha256']:return result
    padding={}
    for trial in proposal.get('continuous_trials',[]):
        value=trial.get('candidate_padding_mm',0)
        if value>0:padding[trial['net']]=min(padding.get(trial['net'],value),value)
    if not padding:return result
    scope=model_scope(data,profile,Path(__file__).with_name('grid_route.py'));domains={}
    def key(p):return tuple(round(v,6) for v in p)
    incident={}
    for index,t in enumerate(tracks):
        for end in ('start','end'):incident.setdefault((t['layer'],key(t[end])),[]).append((index,end))
    used=set()
    for (layer,junction),ends in sorted(incident.items()):
        if len(ends)!=2:continue
        (i,ie),(j,je)=ends
        if i==j or i in used or j in used:continue
        a,b=tracks[i],tracks[j];net=a['net'];width=a['width']
        if net not in padding or net!=b['net'] or width!=b['width']:continue
        at=Point(junction)
        if any(Point(v['at']).buffer(v['diameter']/2).covers(at) for v in proposal.get('vias',[])):continue
        if any(index not in (i,j) and t['layer']==layer and LineString([t['start'],t['end']]).buffer(t['width']/2).covers(at)
               for index,t in enumerate(tracks)):continue
        if any(str(layer) in item['shapes'] and shape(item['shapes'][str(layer)]).covers(Point(junction)) for item in data['items']):continue
        other_a=a['end' if ie=='start' else 'start'];other_b=b['end' if je=='start' else 'start']
        va=[other_a[k]-junction[k] for k in (0,1)];vb=[other_b[k]-junction[k] for k in (0,1)]
        la=math.hypot(*va);lb=math.hypot(*vb)
        if not la or not lb:continue
        cosine=max(-1,min(1,sum(x*y for x,y in zip(va,vb))/(la*lb)));angle=math.acos(cosine)
        if not 1e-8<angle<math.radians(20):continue
        distance=width/(2*math.sin(angle/2))
        if distance>=min(la,lb)-1e-6:continue
        p=[round(junction[k]+distance*va[k]/la,6) for k in (0,1)]
        q=[round(junction[k]+distance*vb[k]/lb,6) for k in (0,1)]
        domain_key=(net,width)
        if domain_key not in domains:domains[domain_key]=dict(zip(data['layers'],scope['domains'](net,width/2,padding=padding[net])))
        legal=domains[domain_key].get(layer)
        if legal is None or not all(legal.covers(LineString([key(x),key(y)])) for x,y in ((other_a,p),(p,q),(q,other_b))):continue
        a[ie]=p;b[je]=q;tracks.append({k:a[k] for k in ('net','layer','width')}|{'start':p,'end':q});used.update((i,j))
        changes.append({'net':net,'layer':layer,'original_junction_mm':list(junction),'included_angle_degrees':math.degrees(angle),'width_mm':width,'chamfer_mm':[p,q],'candidate_padding_mm':padding[net]})
    if changes:result['generated_hairpin_chamfers']=changes
    bridges=[];segments=list(tracks);splits={}
    for i,a in enumerate(segments):
        for j,b in enumerate(segments[i+1:],i+1):
            net=a['net'];width=a['width'];layer=a['layer']
            if net not in padding or (net,width,layer)!=(b['net'],b['width'],b['layer']):continue
            crossing=LineString([a['start'],a['end']]).intersection(LineString([b['start'],b['end']]))
            if crossing.geom_type!='Point':continue
            junction=list(crossing.coords[0])
            if any(Point(end).distance(crossing)<1e-6 for t in (a,b) for end in (t['start'],t['end'])):continue
            for end_a in (a['start'],a['end']):
                for end_b in (b['start'],b['end']):
                    va=[end_a[k]-junction[k] for k in (0,1)];vb=[end_b[k]-junction[k] for k in (0,1)]
                    la=math.hypot(*va);lb=math.hypot(*vb)
                    angle=math.acos(max(-1,min(1,sum(x*y for x,y in zip(va,vb))/(la*lb))))
                    if not 1e-8<angle<math.radians(20):continue
                    distance=width/(2*math.sin(angle/2))
                    if distance>=min(la,lb)-1e-6:continue
                    p=[round(junction[k]+distance*va[k]/la,6) for k in (0,1)];q=[round(junction[k]+distance*vb[k]/lb,6) for k in (0,1)]
                    domain_key=(net,width)
                    if domain_key not in domains:domains[domain_key]=dict(zip(data['layers'],scope['domains'](net,width/2,padding=padding[net])))
                    legal=domains[domain_key].get(layer)
                    if legal is None or not legal.covers(LineString([p,q])):continue
                    tracks.append({k:a[k] for k in ('net','layer','width')}|{'start':p,'end':q})
                    splits.setdefault(i,[]).append(p);splits.setdefault(j,[]).append(q)
                    bridges.append({'net':net,'layer':layer,'crossing_mm':junction,'included_angle_degrees':math.degrees(angle),'width_mm':width,'bridge_mm':[p,q],'candidate_padding_mm':padding[net]})
    if bridges:
        rebuilt=[]
        for index,t in enumerate(segments):
            line=LineString([t['start'],t['end']]);points=[t['start']]+sorted({tuple(p) for p in splits.get(index,[])},key=lambda p:line.project(Point(p)))+[t['end']]
            rebuilt.extend({k:t[k] for k in ('net','layer','width')}|{'start':list(p),'end':list(q)} for p,q in zip(points,points[1:]) if p!=q)
        result['tracks']=rebuilt+tracks[len(segments):]
        result['generated_acute_crossing_bridges']=bridges
    return result


def coupled_completion(directory,worker,kicad,profile,seconds=240,protected=()):
    """Measured joint escape plus append-only completion on one guarded fork.

    The input must have native-persisted UUIDs. No intermediate displaced open
    is adopted: only the final original-source/native/profile gates can copy
    copper back. Two measured joint selections and one finite append pass per
    selection are bounded independently and retain every miss.
    """
    from solve_kicad import audit_preservation,summarize_drc
    from optimize_kicad import unchanged_other_nets
    from joint_congestion_experiment import experiment
    from joint_route import route_joint
    directory=Path(directory);start=time.perf_counter();branch=directory/'coupled-completion';branch.mkdir(exist_ok=False)
    source=directory/'candidate.kicad_pcb';source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
    shutil.copy2(source,branch/'input-candidate.kicad_pcb')
    for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
        if source.with_suffix(suffix).exists():shutil.copy2(source.with_suffix(suffix),branch/('candidate'+suffix))
    copy_assets(directory,branch);profile_path=branch/'profile.json';profile_path.write_text(json.dumps(profile,indent=2)+'\n')
    def remaining():return seconds-(time.perf_counter()-start)
    def run(args,d,label):
        project={s:(d/('candidate'+s)).read_bytes() for s in ('.kicad_pro','.kicad_dru') if (d/('candidate'+s)).exists()}
        try:
            with (d/(label+'.log')).open('w') as log:subprocess.run(list(map(str,args)),check=True,timeout=max(1,min(45,remaining())),stdout=log,stderr=subprocess.STDOUT)
        finally:
            for suffix,value in project.items():(d/('candidate'+suffix)).write_bytes(value)
    def inspect(d):
        run(worker+['dc-geometry',d],d,'geometry');g=json.loads((d/'dc-geometry.json').read_text());g['routing_profile']=profile;return g
    def check(d):
        run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'routed-drc.json',d/'candidate.kicad_pcb'],d,'native-drc')
        return json.loads((d/'routed-drc.json').read_text())
    report={'source_sha256':source_hash,'accepted':False,'complete':False,'joints':[],
        'component_cut_trials':[],'append_trials':[],'protected_nets':list(protected),'manual_trace_coordinates':False}
    changed=set()
    attempted_clusters=set()
    attempted_targets={}
    def save():
        report['elapsed_seconds']=time.perf_counter()-start
        (branch/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    try:
        initial=inspect(branch);initial_drc=check(branch);initial_screen=screen(initial,profile);initial_counts=native_open_counts(initial,initial_drc)
        run(worker+['native-connectivity',branch],branch,'source-native-connectivity')
        original_contacts=json.loads((branch/'native-connectivity.json').read_text())
        final,final_drc,final_counts=initial,initial_drc,initial_counts
        (branch/'source-geometry.json').write_text(json.dumps(initial));(branch/'source-drc.json').write_text(json.dumps(initial_drc,indent=2))
        diagnosis=measured_component_cuts(initial,initial_drc,profile,protected,seconds=max(0,min(15,remaining()-15)))
        (branch/'measured-component-cuts.json').write_text(json.dumps(diagnosis,indent=2)+'\n')
        if diagnosis['selected'] and remaining()>15:
            for selection in diagnosis['selected']:
                owners=selection['owner_nets'];orders=list(dict.fromkeys([tuple(owners),tuple(reversed(owners))]))
                for owner_order in orders:
                    if remaining()<15:break
                    d=branch/f'component-cut-{len(report["component_cut_trials"]):02d}';d.mkdir();copy_assets(branch,d)
                    for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
                        if (branch/('candidate'+suffix)).exists():shutil.copy2(branch/('candidate'+suffix),d/('candidate'+suffix))
                    row={'target':selection['target_name'],'cut_native_uuids':selection['removed_native_uuids'],
                        'owner_order':list(owner_order),'target_copper_retained':True,'accepted':False,'stages':[]}
                    report['component_cut_trials'].append(row)
                    allowed={initial['nets'][str(c)]['name'] for c in (selection['target'],*owners)}
                    try:
                        cut={'tracks':[],'vias':[],'remove':selection['removed_native_uuids']}
                        cut['preserve_functional_fills']=True
                        (d/'grid-route.json').write_text(json.dumps(cut));run(worker+['apply-grid',d],d,'cut')
                        coordinated=None
                        if owner_order==tuple(owners) and remaining()>25:
                            g=inspect(d)
                            coordinated=route_joint(g,[selection['target'],*owners],seconds=max(1,min(60,remaining()-20)))
                            coordinated['preserve_functional_fills']=True
                            (d/'joint-route.json').write_text(json.dumps(coordinated,indent=2)+'\n')
                            row['coordinated']={k:v for k,v in coordinated.items()if k not in ('tracks','vias','joint_trials')}
                            if coordinated['complete_cluster']:
                                (d/'grid-route.json').write_text(json.dumps(coordinated));run(worker+['apply-grid',d],d,'coordinated')
                        for net in (() if coordinated and coordinated['complete_cluster'] else (selection['target'],*owner_order)):
                            g=inspect(d)
                            proposal=route(g,step=.025,seconds=max(1,min(30,remaining()-10)),only_nets=[net],continuous=True)
                            proposal=chamfer_generated_hairpins(g,proposal,profile)
                            if proposal.get('remove'):raise ValueError('Component restoration must only append copper')
                            row['stages'].append({'net':g['nets'][str(net)]['name'],'failed':proposal['failed'],
                                'tracks':len(proposal['tracks']),'vias':len(proposal['vias'])})
                            (d/f'append-{net}.json').write_text(json.dumps(proposal))
                            if proposal['failed']:break
                            if proposal['tracks'] or proposal['vias']:
                                (d/'grid-route.json').write_text(json.dumps(proposal));run(worker+['apply-grid',d],d,f'append-{net}')
                        if not(coordinated and coordinated['complete_cluster']) and (len(row['stages'])!=1+len(owners) or any(s['failed'] for s in row['stages'])):continue
                        after_drc=check(d);after=inspect(d)
                        run(worker+['native-connectivity',d],d,'final-native-connectivity')
                        contacts=json.loads((d/'native-connectivity.json').read_text())
                        partition=native_partition_preservation(original_contacts,contacts,selection['removed_native_uuids'],allowed)
                        preserve=audit_preservation(source,d/'candidate.kicad_pcb')
                        plane=plane_partitions_preserved(initial,after,sorted({z['net'] for z in initial.get('zones',[]) if z['net'] and not z.get('teardrop')}),[])
                        after_counts=native_open_counts(after,after_drc)
                        project_exact=all((branch/('candidate'+s)).read_bytes()==(d/('candidate'+s)).read_bytes()
                            for s in ('.kicad_pro','.kicad_dru') if (branch/('candidate'+s)).exists())
                        row.update(native_partitions=partition,source_preservation=preserve,plane_partitions=plane,
                            project_exact=project_exact,profile_nonregression=nonregression(initial_screen,screen(after,profile)),
                            no_new_native_findings=no_new_native_findings(initial_drc,after_drc),
                            no_displaced_open_regression=all(after_counts.get(c,0)<=n for c,n in initial_counts.items()),
                            foreign_copper=unchanged_other_nets(source,d/'candidate.kicad_pcb',allowed),
                            after=summarize_drc(d/'routed-drc.json'))
                        row['accepted']=all((partition['pass'],preserve['pass'],plane['pass'],project_exact,
                            row['profile_nonregression'],row['no_new_native_findings'],row['no_displaced_open_regression'],
                            row['foreign_copper'],len(after_drc['unconnected_items'])<len(initial_drc['unconnected_items'])))
                        if row['accepted']:
                            shutil.copy2(d/'candidate.kicad_pcb',branch/'candidate.kicad_pcb');changed.update(allowed)
                            final,final_drc,final_counts=after,after_drc,after_counts
                    except (ValueError,RuntimeError,subprocess.SubprocessError) as error:row['error']=str(error)
                    finally:
                        (d/'result.json').write_text(json.dumps(row,indent=2)+'\n');save()
                    if row['accepted']:break
                if report['component_cut_trials'] and report['component_cut_trials'][-1]['accepted']:break
        for iteration in range(2):
            if remaining()<20:break
            current=inspect(branch);native=check(branch)
            if not native['unconnected_items']:break
            current_hash=hashlib.sha256((branch/'candidate.kicad_pcb').read_bytes()).hexdigest()
            targets=attempted_targets.setdefault(current_hash,set())
            selection=measured_owner_cluster(current,native,profile,protected,seconds=min(15,remaining()),excluded_targets=targets)
            if selection is not None and remaining()>25:
                if selection.get('source_sha256') not in (None,current_hash):raise ValueError('Measured cut belongs to different source copper')
                selection['source_sha256']=current_hash
                targets.add(selection['target'])
                key=(selection['source_sha256'],tuple(sorted(selection['selected_nets'])))
                if key in attempted_clusters:
                    report.setdefault('unchanged_cluster_skips',[]).append({'iteration':iteration,'selected_nets':selection['selected_nets'],'source_sha256':selection['source_sha256']})
                    selection=None
                else:attempted_clusters.add(key)
            if selection is not None and remaining()>25:
                selection_path=branch/f'selection-{iteration:02d}.json';selection_path.write_text(json.dumps(selection,indent=2)+'\n')
                d=branch/f'coordinated-rebuild-{iteration:02d}';d.mkdir();copy_assets(branch,d)
                for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
                    if (branch/('candidate'+suffix)).exists():shutil.copy2(branch/('candidate'+suffix),d/('candidate'+suffix))
                codes={int(c)for c,n in current['nets'].items()if n['name']in selection['selected_nets']}
                cut_ids=measured_owner_cut(current,selection,current_hash,protected)
                joint_row={'target':selection['target'],'selected_nets':selection['selected_nets'],'removed_native_uuids':cut_ids,
                    'unpeeled_owner_copper_retained':True,'accepted':False}
                report.setdefault('coordinated_trials',[]).append(joint_row)
                try:
                    (d/'grid-route.json').write_text(json.dumps({'tracks':[],'vias':[],'remove':cut_ids,'preserve_functional_fills':True}))
                    run(worker+['apply-grid',d],d,'cut');g=inspect(d)
                    proposal=route_joint(g,sorted(codes),seconds=max(1,min(60,remaining()-20)))
                    proposal['preserve_functional_fills']=True
                    (d/'joint-route.json').write_text(json.dumps(proposal,indent=2)+'\n')
                    joint_row['search']={k:v for k,v in proposal.items()if k not in ('tracks','vias','joint_trials')}
                    if proposal['complete_cluster']:
                        (d/'grid-route.json').write_text(json.dumps(proposal));run(worker+['apply-grid',d],d,'apply')
                        after_drc=check(d);after=inspect(d);run(worker+['native-connectivity',d],d,'final-native-connectivity')
                        contacts=json.loads((d/'native-connectivity.json').read_text())
                        cuts=sorted(set(original_contacts['native_items'])-set(contacts['native_items']))
                        partition=native_partition_preservation(original_contacts,contacts,cuts,changed|set(selection['selected_nets']))
                        counts=native_open_counts(after,after_drc)
                        preserve=audit_preservation(source,d/'candidate.kicad_pcb')
                        foreign=unchanged_other_nets(branch/'candidate.kicad_pcb',d/'candidate.kicad_pcb',selection['selected_nets'])
                        joint_row.update(native_partitions=partition,source_preservation=preserve,foreign_copper=foreign,
                            profile_nonregression=nonregression(initial_screen,screen(after,profile)),
                            no_new_native_findings=no_new_native_findings(initial_drc,after_drc),
                            selected_native_closed=all(counts.get(c,0)==0 for c in codes))
                        joint_row['accepted']=all((partition['pass'],preserve['pass'],foreign,joint_row['profile_nonregression'],joint_row['no_new_native_findings'],joint_row['selected_native_closed'],len(after_drc['unconnected_items'])<len(native['unconnected_items'])))
                        if joint_row['accepted']:
                            shutil.copy2(d/'candidate.kicad_pcb',branch/'candidate.kicad_pcb');changed.update(selection['selected_nets'])
                            final,final_drc,final_counts=after,after_drc,counts
                except (ValueError,RuntimeError,subprocess.SubprocessError)as error:joint_row['error']=str(error)
                finally:(d/'result.json').write_text(json.dumps(joint_row,indent=2)+'\n');save()
                if joint_row['accepted']:continue
            if selection is not None and remaining()>25:
                joint=branch/f'joint-{iteration:02d}'
                result=experiment(branch/'candidate.kicad_pcb',branch,joint,profile_path,seconds=max(1,min(60,int(remaining()-15))),passes=8,protected=protected,selection_path=selection_path,rebuild_selected=True)
                report['joints'].append(result)
                if result.get('state')=='done':
                    candidate_drc=json.loads((joint/'routed-drc.json').read_text())
                    # An unfinished joint draft can continue privately, but it
                    # cannot bypass fixed physical/source/profile safeguards.
                    physical=(result['foreign_copper']['pass'] and result['preservation']['pass'] and result['project_exact'] and result['routing_nonregression'] and no_new_native_findings(native,candidate_drc))
                    if physical:
                        shutil.copy2(joint/'candidate.kicad_pcb',branch/'candidate.kicad_pcb');changed.update(selection['selected_nets'])
            tried=set()
            while remaining()>20:
                g=inspect(branch);native=check(branch);counts=native_open_counts(g,native);names={int(c):n['name'] for c,n in g['nets'].items()}
                locked={t['net'] for t in g['tracks']+g['vias'] if t.get('locked',True)}|{c for c,n in names.items() if n in protected}
                options=[c for c in counts if counts[c] and c not in tried|locked]
                if not options:break
                target=min(options,key=lambda c:(sum(p['net']==c for p in g['pads']),c));tried.add(target)
                d=branch/f'append-{len(report["append_trials"]):02d}';d.mkdir();copy_assets(branch,d)
                for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
                    if (branch/('candidate'+suffix)).exists():shutil.copy2(branch/('candidate'+suffix),d/('candidate'+suffix))
                row={'net':names[target],'accepted':False,'source_sha256':hashlib.sha256((branch/'candidate.kicad_pcb').read_bytes()).hexdigest()}
                try:
                    proposal=route(g,step=.025,seconds=max(1,min(30,remaining()-15)),only_nets=[target],continuous=True)
                    proposal=chamfer_generated_hairpins(g,proposal,profile)
                    (d/'grid-route.json').write_text(json.dumps(proposal));row['failed']=proposal.get('failed',[])
                    if proposal['tracks']:
                        run(worker+['apply-grid',d],d,'apply');after_drc=check(d);after=inspect(d);s=screen(after,profile)
                        planes=sorted({z['net'] for z in g.get('zones',[]) if z['net'] and not z.get('teardrop')})
                        plane_gate=plane_partitions_preserved(g,after,planes,[])
                        row.update(after=summarize_drc(d/'routed-drc.json'),foreign_copper=unchanged_other_nets(branch/'candidate.kicad_pcb',d/'candidate.kicad_pcb',[names[target]]),preservation=audit_preservation(branch/'candidate.kicad_pcb',d/'candidate.kicad_pcb'),plane_partitions=plane_gate,profile_nonregression=nonregression(screen(g,profile),s))
                        row['accepted']=row['foreign_copper'] and row['preservation']['pass'] and plane_gate['pass'] and row['profile_nonregression'] and no_new_native_findings(native,after_drc) and native_open_counts(after,after_drc).get(target,0)==0 and len(after_drc['unconnected_items'])<len(native['unconnected_items'])
                        if row['accepted']:
                            shutil.copy2(d/'candidate.kicad_pcb',branch/'candidate.kicad_pcb');changed.add(names[target])
                except (ValueError,RuntimeError,subprocess.SubprocessError) as error:
                    row['error']=str(error)
                    (d/'candidate-error.json').write_text(json.dumps(row,indent=2)+'\n')
                report['append_trials'].append(row);save()
            final=inspect(branch);final_drc=check(branch);final_counts=native_open_counts(final,final_drc)
            if not final_drc['unconnected_items']:break
        final_screen=screen(final,profile)
        preserve=audit_preservation(source,branch/'candidate.kicad_pcb');foreign=unchanged_other_nets(source,branch/'candidate.kicad_pcb',changed)
        planes=sorted({z['net'] for z in initial.get('zones',[]) if z['net'] and not z.get('teardrop')});partition=plane_partitions_preserved(initial,final,planes,[])
        project_exact=all(source.with_suffix(s).read_bytes()==(branch/('candidate'+s)).read_bytes() for s in ('.kicad_pro','.kicad_dru') if source.with_suffix(s).exists())
        no_displaced_regression=all(final_counts.get(c,0)<=count for c,count in initial_counts.items())
        run(worker+['native-connectivity',branch],branch,'final-native-connectivity')
        final_contacts=json.loads((branch/'native-connectivity.json').read_text())
        all_cuts=sorted(set(original_contacts['native_items'])-set(final_contacts['native_items']))
        source_partition=native_partition_preservation(original_contacts,final_contacts,all_cuts,changed)
        report['native_partitions']=source_partition
        report['removed_original_copper_uuids']=all_cuts
        report.update(final=summarize_drc(branch/'routed-drc.json'),source_preservation=preserve,foreign_copper=foreign,plane_partitions=partition,profile_pass=final_screen['pass'],profile_nonregression=nonregression(initial_screen,final_screen),project_exact=project_exact,no_new_native_findings=no_new_native_findings(initial_drc,final_drc),no_displaced_open_regression=no_displaced_regression,changed_nets=sorted(changed),source_unchanged=hashlib.sha256(source.read_bytes()).hexdigest()==source_hash)
        report['accepted']=source_partition['pass'] and preserve['pass'] and foreign and partition['pass'] and project_exact and report['profile_nonregression'] and report['no_new_native_findings'] and no_displaced_regression and report['source_unchanged'] and len(final_drc['unconnected_items'])<len(initial_drc['unconnected_items'])
        report['complete']=report['accepted'] and not final_drc['unconnected_items'] and final_screen['pass']
        (branch/'routing-screen.json').write_text(json.dumps(final_screen,indent=2)+'\n')
        if report['accepted']:
            shutil.copy2(branch/'candidate.kicad_pcb',directory/'candidate.kicad_pcb');shutil.copy2(branch/'routed-drc.json',directory/'routed-drc.json')
    except (ValueError,KeyError,RuntimeError,subprocess.SubprocessError) as error:report['error']=str(error)
    save();return report
