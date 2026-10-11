"""Join disconnected filled-plane islands with native-DRC-checked through vias."""
import json
import copy
import hashlib
import time
from pathlib import Path
import shutil
import subprocess
import shapely
from shapely.geometry import Point
from grid_route import shape
from grid_route import route
from shapely.ops import nearest_points
from native_project import copy_assets


def plane_components(g,net):
    """Connected components of actual filled copper, including via barrels."""
    nodes=[{int(l):shape(p) for l,p in i['shapes'].items()} for i in g['items'] if i['net']==net]
    for zone in g.get('zones',[]):
        if zone['net']==net and not zone.get('teardrop'):
            nodes.extend({zone['layer']:shape([p])} for p in zone['filled'])
    parent=list(range(len(nodes)))
    def root(i):
        while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
        return i
    for i,a in enumerate(nodes):
        for j,b in enumerate(nodes[:i]):
            if any(a[l].intersects(b[l]) for l in a.keys()&b.keys()):parent[root(i)]=root(j)
    grouped={}
    for i,node in enumerate(nodes):
        group=grouped.setdefault(root(i),{})
        for layer,poly in node.items():group.setdefault(layer,[]).append(poly)
    return [{l:shapely.union_all(polys) for l,polys in group.items()} for group in grouped.values()]


def plane_contacts(g,net):
    """Present equipotential components as contacts for a bounded bridge search."""
    groups=plane_components(g,net);result=copy.deepcopy(g)
    for item in result['items']:
        if item['net']==net:item['terminals']=[]
    for group in groups:
        points=[];polygons={}
        for layer,poly in group.items():
            parts=[p for p in shapely.get_parts(poly) if p.geom_type=='Polygon']
            polygons[str(layer)]=[{'shell':list(p.exterior.coords),'holes':[list(h.coords) for h in p.interiors]} for p in parts]
            points.extend(list(p.representative_point().coords)[0] for p in parts)
            for other in groups:
                if other is not group and layer in other:
                    points.append(list(nearest_points(poly,other[layer])[0].coords)[0])
        result['items'].append({'net':net,'shapes':polygons,'terminals':list(set(points)),
                                'clearance':0,'kind':'pad','synthetic_plane_contacts':True})
    return result,len(groups)


def fixed_plane_pair(current,net,pads,goal,original_items):
    """Anchor an evolving plane demand to fixed pads and retained main copper."""
    if any(not any(all(p.get(key)==pad.get(key) for key in ('ref','pin','net','at','shapes'))
                   for p in current['pads']) for pad in pads):
        raise ValueError('Fixed native source pad changed or disappeared')
    groups=plane_components(current,net)
    def contacts(polygons):
        return {i for i,group in enumerate(groups) if any(
            layer in group and group[layer].intersection(poly).area>1e-10
            for layer,poly in polygons.items())}
    source_groups=[contacts({int(l):shape(p) for l,p in pad['shapes'].items()}) for pad in pads]
    sources=set().union(*source_groups)
    goals=contacts(goal)
    if not pads or any(len(group)!=1 for group in source_groups) or len(sources)!=1 or len(goals)!=1:
        raise ValueError('Fixed source pads or retained main contacts are missing or split')
    model=copy.deepcopy(current)
    originals=[(i.get('uuid'),{k:v for k,v in i.items() if k!='uuid'}) for i in original_items]
    for item in model['items']:
        if item['net']!=net:continue
        physical={k:v for k,v in item.items() if k!='uuid'}
        # Older retained exports lack UUID metadata. Physical and role fields
        # still match exactly; two tagged, distinct objects remain distinct.
        if any(physical==old and (not uid or not item.get('uuid') or uid==item['uuid'])
               for uid,old in originals):item['terminals']=[]
    # A previously filled island can be split by the next neighbor route. Its
    # old polygon must not electrically glue the fixed pad to a detached finger.
    for pad in pads:
        model['items'].append({'net':net,'kind':'pad','clearance':0,
            'terminals':[pad['at']],'shapes':copy.deepcopy(pad['shapes']),
            'synthetic_plane_contacts':True,'fixed_pad_reference':pad['ref']+'.'+pad['pin']})
    region={layer:groups[next(iter(goals))][layer].intersection(poly)
            for layer,poly in goal.items() if layer in groups[next(iter(goals))]}
    polygons={};points=[]
    for layer,poly in region.items():
        parts=[p for p in shapely.get_parts(poly) if p.geom_type=='Polygon' and p.area>1e-10]
        polygons[str(layer)]=[{'shell':list(p.exterior.coords),'holes':[list(h.coords) for h in p.interiors]} for p in parts]
        points.extend(list(p.representative_point().coords)[0] for p in parts)
    model['items'].append({'net':net,'kind':'pad','clearance':0,'terminals':points,
                          'shapes':polygons,'synthetic_plane_contacts':True})
    return model,sources==goals


def proposals(g):
    if len(g['layers'])<2:return []
    candidates=[]
    for net in sorted({z['net'] for z in g.get('zones',[]) if z['net']}):
        spec=g['nets'].get(str(net))
        if not spec:continue
        groups=plane_components(g,net)
        radius=spec['via_diameter']/2+.003
        obstacles=[shape(p).buffer(max(spec['clearance'],item['clearance'])+radius)
                   for item in g['items'] if item['net']!=net for p in item['shapes'].values()]
        obstacles += [shape(k['shapes']).buffer(radius) for k in g['keepouts'] if k['vias']]
        obstacles += [shape(h['shapes']).buffer(radius+max(g['hole_clearance'],h.get('clearance',0))) for h in g.get('drill_obstacles',[]) if h['net']!=net]
        obstacles.append(shape(g['holes']).buffer(spec['via_drill']/2+g['hole_to_hole']+.003))
        legal=shape(g['outline']).buffer(-g['edge_clearance']-radius).difference(shapely.union_all(obstacles))
        for index,group in enumerate(groups):
            for other_group in groups[:index]:
                for layer,a in group.items():
                    for other,b in other_group.items():
                        if layer==other:continue
                        region=a.intersection(b).buffer(-radius).intersection(legal)
                        if region.is_empty:continue
                        p=region.representative_point()
                        candidates.append((region.area,{'net':net,'at':[round(p.x,6),round(p.y,6)],'diameter':spec['via_diameter'],'drill':spec['via_drill']}))
    return [v for _,v in sorted(candidates,key=lambda pair:-pair[0])]


def stitch(directory,worker,kicad,rounds=8,profile=None,protected=()):
    from solve_kicad import audit_preservation,summarize_drc
    from routing_screen import screen,nonregression
    directory=Path(directory);trials=[]
    def run(args,log):
        project=log.parent/'candidate.kicad_pro';original=project.read_bytes() if project.exists() else None
        try:
            with log.open('w') as f:subprocess.run(list(map(str,args)),stdout=f,stderr=subprocess.STDOUT,check=True,timeout=30)
        finally:
            if original is not None:project.write_bytes(original)
    for _ in range(rounds):
        before=json.loads((directory/'routed-drc.json').read_text())
        if not any(all(item['description'].startswith('Zone ') for item in issue['items']) for issue in before.get('unconnected_items',[])):break
        run(worker+['dc-geometry',directory],directory/'stitch-geometry.log')
        g=json.loads((directory/'dc-geometry.json').read_text())
        baseline=screen(g,profile) if profile is not None else None
        options=proposals(g);accepted=False
        for via in options[:4]:
            d=directory/f'stitch-{len(trials):02}';d.mkdir();copy_assets(directory,d)
            for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
                if (directory/('candidate'+suffix)).exists():shutil.copy2(directory/('candidate'+suffix),d/('candidate'+suffix))
            (d/'grid-route.json').write_text(json.dumps({'tracks':[],'vias':[via]}))
            run(worker+['apply-grid',d],d/'apply.log')
            run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'routed-drc.json',d/'candidate.kicad_pcb'],d/'drc.log')
            a=summarize_drc(directory/'routed-drc.json');b=summarize_drc(d/'routed-drc.json')
            profile_pass=True
            if profile is not None:
                run(worker+['dc-geometry',d],d/'checked-geometry.log')
                checked=screen(json.loads((d/'dc-geometry.json').read_text()),profile)
                (d/'routing-screen.json').write_text(json.dumps(checked,indent=2))
                profile_pass=nonregression(baseline,checked)
            accepted=(b['unconnected']<a['unconnected'] and b['total_findings']<a['total_findings'] and
                      all(count<=a['types'].get(kind,0) for kind,count in b['types'].items()) and
                      profile_pass and audit_preservation(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb')['pass'])
            trials.append({'via':via,'accepted':accepted,'before':a,'after':b,'routing_nonregression':profile_pass})
            if accepted:
                for name in ('candidate.kicad_pcb','routed-drc.json'):shutil.copy2(d/name,directory/name)
                break
        if not accepted:break
    if profile is not None:
        trials.extend(repair_plane_pockets(directory,worker,kicad,profile,protected=protected))
    (directory/'stitching.json').write_text(json.dumps(trials,indent=2)+'\n');return trials


def local_plane_selection(g,target,profile,protected=()):
    """Measure one actual filled island and its editable native boundary Cu."""
    from shapely.geometry import LineString
    from repair_native_congestion import local_removals
    components=plane_components(g,target)
    filled={layer:shapely.union_all([shape(z['filled']) for z in g.get('zones',[])
            if z['net']==target and z['layer']==layer and not z.get('teardrop')]) for layer in g['layers']}
    choices=[]
    for index,component in enumerate(components):
        area=sum(p.intersection(filled[l]).area for l,p in component.items())
        pads=[p for p in g['pads'] if p['net']==target and any(
            l in component and shape(polys).intersects(component[l]) for layer,polys in p['shapes'].items() for l in [int(layer)])]
        if area>1e-12 and pads:choices.append((area,index,pads))
    if len(components)<=1 or not choices:return None
    area,index,pads=min(choices,key=lambda row:(row[0],row[1]));island=components[index]
    main=max(range(len(components)),key=lambda i:sum(p.intersection(filled[l]).area for l,p in components[i].items()))
    if index==main:return None
    names={int(n):v['name'] for n,v in g['nets'].items()}
    excluded={t['net'] for t in g['tracks']+g['vias'] if t.get('locked',True)}
    excluded.update(n for n,name in names.items() if name in set(protected))
    if names[target] in protected:return None
    spec=g['nets'][str(target)];rows=[]
    for track in g['tracks']:
        if track['net']==target or track['net'] in excluded or track.get('locked',True) or track['arc'] or track['layer'] not in island:continue
        # Match the actual exported native polygon to its source primitive by
        # exact net/layer/endpoints. No guessed bounding rectangle is used.
        matches=[i for i in g['items'] if i['net']==track['net'] and i['kind']=='copper' and str(track['layer']) in i['shapes']
                 and len(i['terminals'])==2 and {tuple(p) for p in i['terminals']}=={tuple(track['start']),tuple(track['end'])}]
        if len(matches)!=1:continue
        item=matches[0];actual=shape(item['shapes'][str(track['layer'])])
        support=island[track['layer']].boundary.intersection(actual.buffer(max(spec['clearance'],item['clearance'])+spec['width']/2)).length
        if support>1e-9:rows.append({'net':track['net'],'uuid':track['uuid'],'boundary_support_mm':support})
    groups={}
    for row in rows:groups.setdefault(row['net'],[]).append(row)
    if not groups:return None
    neighbor=max(groups,key=lambda n:(sum(r['boundary_support_mm'] for r in groups[n]),-n))
    ids=[r['uuid'] for r in groups[neighbor]];points=[p['at'] for p in pads]
    removal=local_removals(g,neighbor,points,ids,retain_crossing=True)
    crossing=[t for t in g['tracks'] if t['uuid'] in ids and t['uuid'] not in removal['remove']]
    extension=None
    if crossing:
        support={r['uuid']:r['boundary_support_mm'] for r in groups[neighbor]}
        crossing=max(crossing,key=lambda t:(support[t['uuid']],t['uuid']))
        physical=LineString([crossing['start'],crossing['end']]).buffer(crossing['width']/2,quad_segs=32)
        midpoint=[(a+b)/2 for a,b in zip(crossing['start'],crossing['end'])]
        extended=[list(p) for p in dict.fromkeys(tuple(p) for p in [*points,crossing['start'],crossing['end']])]
        window=shapely.union_all([Point(p).buffer(3) for p in extended])
        if not window.covers(physical):
            extended.append(midpoint);window=shapely.union_all([Point(p).buffer(3) for p in extended])
        if not window.covers(physical):return None  # Never enlarge a radius or guess another anchor.
        extension={'source_primitive_uuid':crossing['uuid'],'source_primitive':crossing,'midpoint':midpoint}
        points=extended;removal=local_removals(g,neighbor,points,ids,retain_crossing=True)
        if crossing['uuid'] not in removal['remove']:return None
    if not removal['remove']:return None
    return {'target':target,'source_component':index,'goal_component':main,'filled_area_mm2':area,
            'fixed_pad_contacts':[{'reference':p['ref'],'pin':p['pin'],'at':p['at']} for p in pads],
            'neighbor':neighbor,'boundary':rows,'selected_boundary_uuids':ids,'points':points,'extension':extension,'removal':removal}


def local_plane_trial(directory,worker,kicad,profile,g,target,seconds,protected=()):
    """One native local boundary trial; a partial plane merge is not completion."""
    from repair_native_congestion import local_copper_preserved,plane_partitions_preserved
    from optimize_kicad import unchanged_other_nets
    from routing_screen import screen,nonregression
    from solve_kicad import audit_preservation,summarize_drc
    directory=Path(directory);deadline=time.perf_counter()+seconds;trials=[]
    selection=local_plane_selection(g,target,profile,protected)
    if selection is None:return trials
    source=hashlib.sha256((directory/'candidate.kicad_pcb').read_bytes()).hexdigest()
    if g.get('source_board_sha256')!=source:return [{'kind':'local_plane_boundary','accepted':False,'reason':'Stale native geometry source hash'}]
    neighbor=selection['neighbor'];points=selection['points'];removed=set(selection['removal']['remove'])
    window=shapely.union_all([Point(p).buffer(3) for p in points]);polygons=[{'shell':list(p.exterior.coords),'holes':[list(h.coords) for h in p.interiors]} for p in shapely.get_parts(window)]
    source_components=plane_components(g,target);island=source_components[selection['source_component']];main=source_components[selection['goal_component']]
    source_pads=[p for p in g['pads'] if p['net']==target and any(
        int(l) in island and shape(polys).intersection(island[int(l)]).area>1e-10
        for l,polys in p['shapes'].items())]
    goal={layer:poly.difference(window) for layer,poly in main.items()}
    # Fixed main-pad copper is also a stable contact when the whole small plane
    # happens to lie inside the edit window.
    for pad in g['pads']:
        if pad['net']!=target:continue
        for l,polys in pad['shapes'].items():
            layer=int(l);poly=shape(polys)
            if layer in main and poly.intersection(main[layer]).area>1e-10:
                goal[layer]=shapely.union_all([goal[layer],poly])
    goal={layer:poly for layer,poly in goal.items() if not poly.is_empty}
    original_target_items=[i for i in g['items'] if i['net']==target]
    project={suffix:(directory/('candidate'+suffix)).read_bytes() for suffix in ('.kicad_pro','.kicad_dru') if (directory/('candidate'+suffix)).exists()}
    asset_hashes=None
    before=screen(g,profile);plane_nets=sorted({z['net'] for z in g['zones'] if z['net'] and not z.get('teardrop')})
    tool_files=[Path(value) for value in [*worker,kicad] if Path(value).is_file()]
    code_files=list(Path(__file__).parent.glob('*.py'))
    execution_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in tool_files+code_files}
    profile_hash=hashlib.sha256(json.dumps(profile,sort_keys=True).encode()).hexdigest()
    def run(args,d,label):
        saved={suffix:(d/('candidate'+suffix)).read_bytes() for suffix in ('.kicad_pro','.kicad_dru') if (d/('candidate'+suffix)).exists()}
        try:
            with (d/(label+'.log')).open('w') as log:subprocess.run(list(map(str,args)),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
        finally:
            for suffix,value in saved.items():(d/('candidate'+suffix)).write_bytes(value)
    def geometry(d,label):
        run(worker+['dc-geometry',d],d,label);current=json.loads((d/'dc-geometry.json').read_text());current['routing_profile']=profile;return current
    def pair_model(current):
        return fixed_plane_pair(current,target,source_pads,goal,original_target_items)
    for order in ((target,neighbor),(neighbor,target)):
        if time.perf_counter()>=deadline:break
        d=directory/f'plane-local-{len(list(directory.glob("plane-local-*-trial"))):02}-trial';d.mkdir();assets=copy_assets(directory,d)
        for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
            if (directory/('candidate'+suffix)).exists():shutil.copy2(directory/('candidate'+suffix),d/('candidate'+suffix))
        if asset_hashes is None:asset_hashes={name:hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in assets}
        trial={'kind':'local_plane_boundary','target':g['nets'][str(target)]['name'],'displaced':g['nets'][str(neighbor)]['name'],
               'source_board_sha256':source,'selection':selection,'route_order':list(order),'accepted':False};trials.append(trial)
        (d/'selection.json').write_text(json.dumps(trial,indent=2))
        try:
            (d/'grid-route.json').write_text(json.dumps(selection['removal']));run(worker+['apply-grid',d],d,'local-remove');current=geometry(d,'local-prepared')
            preparation=local_copper_preserved(g,current,[neighbor],points,removed=removed)
            partitions=plane_partitions_preserved(g,current,plane_nets,points)
            if not preparation['pass'] or not partitions['pass']:raise ValueError('Local cut violates copper or plane preservation')
            run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'local-cut-drc.json',d/'candidate.kicad_pcb'],d,'cut-drc')
            cut_drc=json.loads((d/'local-cut-drc.json').read_text())
            neighbor_cut_open=any('['+g['nets'][str(neighbor)]['name']+']' in item['description'] for issue in cut_drc.get('unconnected_items',[]) for item in issue['items'])
            _,cut_merged=pair_model(current)
            pruning_only=cut_merged and not neighbor_cut_open
            trial.update(pruning_only=pruning_only,neighbor_native_connected_after_cut=not neighbor_cut_open)
            stages=[]
            for net in order:
                model,merged=pair_model(current) if net==target else (copy.deepcopy(current),False)
                if pruning_only:
                    stages.append({'net':net,'native_pruning_demand_satisfied':True,'new_tracks':0,'new_vias':0});continue
                model['candidate_only_native_copper_windows']={str(neighbor):polygons}
                left=deadline-time.perf_counter()
                if left<=0:raise ValueError('Local plane budget exhausted')
                proposal=route(model,step=.025,seconds=left,only_nets=[net],continuous=True)
                (d/f'route-{net}.json').write_text(json.dumps(proposal,indent=2))
                if net not in proposal['completed_nets']:raise ValueError('Selected plane pair or displaced neighbor did not reconnect')
                (d/'grid-route.json').write_text(json.dumps(proposal));run(worker+['apply-grid',d],d,f'apply-{net}');current=geometry(d,f'geometry-{net}')
                stages.append({'net':net,'completed_selected_demand':True,'new_tracks':len(proposal['tracks']),'new_vias':len(proposal['vias'])})
            _,merged=pair_model(current)
            if not merged:raise ValueError('Native refill did not merge the selected original source components')
            run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'routed-drc.json',d/'candidate.kicad_pcb'],d,'drc')
            current=geometry(d,'checked');after=screen(current,profile)
            _,merged=pair_model(current)  # Final native DRC/refill can re-box an earlier merge.
            local=local_copper_preserved(g,current,[neighbor],points,removed=removed);planes=plane_partitions_preserved(g,current,plane_nets,points)
            foreign=unchanged_other_nets(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb',[g['nets'][str(n)]['name'] for n in (target,neighbor)])
            structure=audit_preservation(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb');a=summarize_drc(directory/'routed-drc.json');b=summarize_drc(d/'routed-drc.json')
            project_exact=all((d/('candidate'+suffix)).read_bytes()==value for suffix,value in project.items())
            assets_exact=all(hashlib.sha256((d/name).read_bytes()).hexdigest()==digest for name,digest in asset_hashes.items())
            neighbor_open=any('['+g['nets'][str(neighbor)]['name']+']' in item['description'] for issue in json.loads((d/'routed-drc.json').read_text())['unconnected_items'] for item in issue['items'])
            unchanged_source=hashlib.sha256((directory/'candidate.kicad_pcb').read_bytes()).hexdigest()==source
            inputs_exact=(all((directory/('candidate'+suffix)).read_bytes()==value for suffix,value in project.items())
                          and all(hashlib.sha256((directory/name).read_bytes()).hexdigest()==digest for name,digest in asset_hashes.items()))
            execution_exact=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==digest for p,digest in execution_hashes.items())
            profile_exact=hashlib.sha256(json.dumps(profile,sort_keys=True).encode()).hexdigest()==profile_hash
            accepted=(merged and local['pass'] and planes['pass'] and foreign and structure['pass'] and project_exact and assets_exact and unchanged_source and inputs_exact and execution_exact and profile_exact and not neighbor_open
                      and nonregression(before,after) and b['unconnected']<a['unconnected'] and b['total_findings']<a['total_findings'] and all(v<=a['types'].get(k,0) for k,v in b['types'].items()))
            trial.update(accepted=accepted,before=a,after=b,stages=stages,fixed_checks=len(after['rules']),routing_nonregression=nonregression(before,after),
                         local_copper_preservation=local,all_source_plane_preservation=planes,foreign_copper_exact=foreign,structure=structure,project_exact=project_exact,
                         assets_exact=assets_exact,source_unchanged=unchanged_source,source_project_assets_exact=inputs_exact,execution_unchanged=execution_exact,profile_exact=profile_exact,selected_source_components_merged=merged,whole_net_complete=len(plane_components(current,target))<=1)
            (d/'routing-screen.json').write_text(json.dumps(after,indent=2))
            (d/'trial.json').write_text(json.dumps(trial,indent=2))
            if accepted:
                for name in ('candidate.kicad_pcb','routed-drc.json'):shutil.copy2(d/name,directory/name)
                return trials
        except (ValueError,subprocess.SubprocessError) as error:trial['reason']=str(error)
        (d/'trial.json').write_text(json.dumps(trial,indent=2))
    return trials


def repair_plane_pockets(directory,worker,kicad,profile,seconds=60,protected=()):
    """Bridge a boxed plane component, then reroute its measured small blocker."""
    from solve_kicad import audit_preservation,summarize_drc
    from routing_screen import screen,nonregression
    from repair_native_congestion import plane_partitions_preserved
    from optimize_kicad import unchanged_other_nets
    directory=Path(directory);started=time.perf_counter();trials=[]
    def remaining():return seconds-(time.perf_counter()-started)
    def run(args,d,label):
        project=d/'candidate.kicad_pro';original=project.read_bytes() if project.exists() else None
        try:
            with (d/(label+'.log')).open('w') as f:
                subprocess.run(list(map(str,args)),stdout=f,stderr=subprocess.STDOUT,check=True,timeout=30)
        finally:
            if original is not None:project.write_bytes(original)
    report=json.loads((directory/'routed-drc.json').read_text())
    if not report.get('unconnected_items'):return trials
    run(worker+['dc-geometry',directory],directory,'plane-pocket-geometry')
    g=json.loads((directory/'dc-geometry.json').read_text());before=screen(g,profile)
    names={int(n):s['name'] for n,s in g['nets'].items()}
    targets=[n for n in sorted({z['net'] for z in g['zones'] if z['net'] and not z.get('teardrop')})
             if names[n] not in protected and any('['+names[n]+']' in i['description'] for f in report['unconnected_items'] for i in f['items'])]
    excluded={t['net'] for t in g['tracks']+g['vias'] if t.get('locked',True)}
    excluded.update(n for n,name in names.items() if name in set(protected)|{v['net'] for v in profile.get('net_limits',[])})
    source_hash=hashlib.sha256((directory/'candidate.kicad_pcb').read_bytes()).hexdigest()
    plane_nets=sorted({z['net'] for z in g['zones'] if z['net'] and not z.get('teardrop')})
    for target in targets:
        if remaining()<=0:break
        contacts,count=plane_contacts(g,target)
        if count<=1:continue
        contacts['routing_profile']=profile
        contacts['repair_windows']={str(target):list(shape(g['outline']).bounds)}
        diagnosis=route(contacts,step=.075,seconds=min(10,remaining()),max_states=32_000_000,only_nets=[target])
        (directory/f'plane-pocket-diagnosis-{target}.json').write_text(json.dumps(diagnosis,indent=2))
        blockers=list(dict.fromkeys(b['net'] for f in diagnosis['failed'] for b in (f.get('search') or {}).get('blocking_copper',[])))
        choices=([None] if target in diagnosis.get('completed_nets',[]) else
                 [n for n in blockers if n!=target and n not in excluded and 1<sum(p['net']==n for p in g['pads'])<=8])
        if target not in diagnosis.get('completed_nets',[]) and remaining()>0:
            local_trials=local_plane_trial(directory,worker,kicad,profile,g,target,remaining(),protected)
            trials.extend(local_trials)
            if any(trial['accepted'] for trial in local_trials):return trials
        for neighbor in choices:
            if remaining()<=0:break
            d=directory/f'plane-pocket-{len(list(directory.glob("plane-pocket-*-trial"))):02}-trial';d.mkdir();copy_assets(directory,d)
            for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
                if (directory/('candidate'+suffix)).exists():shutil.copy2(directory/('candidate'+suffix),d/('candidate'+suffix))
            project=(d/'candidate.kicad_pro').read_bytes()
            trial={'kind':'plane_bridge' if neighbor is None else 'plane_escape','target':names[target],
                   'displaced':None if neighbor is None else names[neighbor],
                   'source_board_sha256':source_hash,'accepted':False};trials.append(trial)
            try:
                if neighbor is None:
                    bridge=diagnosis  # A completed search still needs the same native candidate gate.
                else:
                    (d/'route-nets.json').write_text(json.dumps({'nets':[neighbor]}));run(worker+['rip-nets',d],d,'rip')
                    run(worker+['dc-geometry',d],d,'after-rip')
                    current=json.loads((d/'dc-geometry.json').read_text())
                    # Keep the old component contacts: merely removing a blocker
                    # can refill the plane temporarily. An explicit bridge keeps
                    # that connection after the blocker is routed again.
                    model=copy.deepcopy(current)
                    for item in model['items']:
                        if item['net']==target:item['terminals']=[]
                    model['items'].extend(i for i in contacts['items'] if i.get('synthetic_plane_contacts'))
                    model['routing_profile']=profile;model['repair_windows']=contacts['repair_windows']
                    if remaining()<=0:raise ValueError('Plane escape budget exhausted')
                    bridge=route(model,step=.075,seconds=min(20,remaining()),max_states=32_000_000,only_nets=[target])
                (d/'bridge-route.json').write_text(json.dumps(bridge,indent=2))
                if target not in bridge['completed_nets']:raise ValueError('Plane bridge did not complete')
                (d/'grid-route.json').write_text(json.dumps(bridge));run(worker+['apply-grid',d],d,'bridge-apply')
                if neighbor is not None:
                    run(worker+['dc-geometry',d],d,'after-bridge')
                    current=json.loads((d/'dc-geometry.json').read_text());current['routing_profile']=profile
                    current['repair_windows']={str(neighbor):list(shape(g['outline']).bounds)}
                    if remaining()<=0:raise ValueError('Plane escape budget exhausted')
                    restored=route(current,step=.075,seconds=min(20,remaining()),max_states=32_000_000,only_nets=[neighbor])
                    (d/'blocker-route.json').write_text(json.dumps(restored,indent=2))
                    if neighbor not in restored['completed_nets']:raise ValueError('Displaced blocker did not reconnect')
                    (d/'grid-route.json').write_text(json.dumps(restored));run(worker+['apply-grid',d],d,'blocker-apply')
                (d/'candidate.kicad_pro').write_bytes(project)
                run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'routed-drc.json',d/'candidate.kicad_pcb'],d,'drc')
                run(worker+['dc-geometry',d],d,'checked-geometry');(d/'candidate.kicad_pro').write_bytes(project)
                current=json.loads((d/'dc-geometry.json').read_text());after=screen(current,profile)
                (d/'routing-screen.json').write_text(json.dumps(after,indent=2))
                a=summarize_drc(directory/'routed-drc.json');b=summarize_drc(d/'routed-drc.json')
                preserved=audit_preservation(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb')['pass']
                foreign=unchanged_other_nets(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb',[names[n] for n in (target,neighbor) if n is not None])
                planes=plane_partitions_preserved(g,current,plane_nets,[])
                accepted=(foreign and planes['pass'] and b['unconnected']<a['unconnected'] and b['total_findings']<a['total_findings'] and
                          all(v<=a['types'].get(k,0) for k,v in b['types'].items()) and nonregression(before,after) and preserved)
                trial.update(before=a,after=b,routing_nonregression=nonregression(before,after),preservation_pass=preserved,foreign_copper_exact=foreign,all_source_plane_preservation=planes,accepted=accepted)
                if accepted:
                    for name in ('candidate.kicad_pcb','routed-drc.json'):shutil.copy2(d/name,directory/name)
                    return trials  # Recompute any remaining pockets on the new board in the next call.
            except (ValueError,subprocess.SubprocessError) as error:trial['reason']=str(error)
    return trials
