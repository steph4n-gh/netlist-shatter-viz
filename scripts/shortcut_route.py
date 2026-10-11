"""Shorten existing same-layer chains without changing their attachment points."""
from collections import defaultdict
import math
import time
import shapely
from shapely.geometry import LineString,Point
from grid_route import shape


def shortcut(data, net, required_owners=None):
    from routing_screen import corridors,inset
    references=corridors(data,data['routing_profile']) if data.get('routing_profile') else {}
    tracks=[t for t in data['tracks'] if t['net']==net and
            (required_owners is None or t['uuid'] in required_owners)]
    by_id={t['uuid']:t for t in tracks}
    adj=defaultdict(list)
    def key(p,layer):return (layer,round(p[0],6),round(p[1],6))
    for t in tracks:
        for p in ('start','end'):adj[key(t[p],t['layer'])].append(t['uuid'])
    pads=[(int(l),shape(s)) for i in data['items'] if i['net']==net and i['kind']=='pad' for l,s in i['shapes'].items()]
    via_positions={tuple(v['at']) for v in data['vias'] if v['net']==net}
    def boundary(k):
        return (len(adj[k])!=2 or (k[1],k[2]) in via_positions
                or any(by_id[i]['locked'] or by_id[i]['arc'] for i in adj[k])
                or any(l==k[0] and g.covers(Point(k[1:])) for l,g in pads))
    outline=shape(data['outline']);legal={}
    def allowed(layer,width):
        cache=(layer,width)
        if cache not in legal:
            radius=width/2
            obstacles=[shape(i['shapes'][str(layer)]).buffer(max(data['nets'][str(net)]['clearance'],i['clearance'])+radius+.002)
                       for i in data['items'] if i['net']!=net and str(layer) in i['shapes']]
            obstacles += [shape(k['shapes']).buffer(radius+.002) for k in data['keepouts'] if k['tracks'] and layer in k['layers']]
            obstacles += [shape(h['shapes']).buffer(radius+data['hole_clearance']+.002) for h in data.get('drill_obstacles',[]) if h['net']!=net]
            legal[cache]=outline.buffer(-data['edge_clearance']-radius).difference(shapely.union_all(obstacles))
        if net in references:
            r=references[net]
            return legal[cache].intersection(inset(r['domain'],width/2)) if layer==r['layer'] else Point().buffer(0)
        return legal[cache]
    visited=set();new=[];remove=[];receipts=[]
    for start in list(adj):
        if not boundary(start):continue
        for first in adj[start]:
            if first in visited:continue
            chain=[];current=start;edge=first;original_points=[list(start[1:])]
            while True:
                t=by_id[edge]
                if t['locked'] or t['arc']:break
                visited.add(edge);chain.append(t)
                current=key(t['end'],t['layer']) if key(t['start'],t['layer'])==current else key(t['start'],t['layer'])
                original_points.append(list(current[1:]))
                if boundary(current):break
                following=[i for i in adj[current] if i!=edge]
                if not following or following[0] in visited:break
                edge=following[0]
            if len(chain)<2 or len({t['width'] for t in chain})!=1 or start==current:continue
            old=sum(math.dist(t['start'],t['end']) for t in chain)
            a,b=list(start[1:]),list(current[1:]);width=chain[0]['width']
            domain=allowed(start[0],width);points=[a,b];method='same-layer shortcut'
            if required_owners is not None and not domain.covers(LineString(points)):
                # Reuse the continuous router for a bounded local replacement;
                # both original endpoints, layer and width remain fixed.
                from polygon_path import triangle_path
                # Path vertices lie on polygon boundaries. Reserve two native
                # units before the one-time quantization, then check literally
                # against the original complete legal domain below.
                corridor=domain.buffer(-.000002).intersection(
                    LineString(original_points).buffer(max(.25,width*4)))
                parts=[p for p in shapely.get_parts(corridor) if p.geom_type=='Polygon'
                       and p.covers(Point(a)) and p.covers(Point(b))]
                if len(parts)!=1:continue
                deadline=time.perf_counter()+1
                def check_time():
                    if time.perf_counter()>deadline:raise TimeoutError('Local chain path deadline')
                try:points,_=triangle_path(parts[0],a,b,check_time,width=width*4)
                except (ValueError,TimeoutError):continue
                method='continuous same-layer shortening'
            points=[[round(v*1e6)/1e6 for v in p] for p in points]
            length=sum(math.dist(x,y) for x,y in zip(points,points[1:]))
            fewer_sections=len(points)-1<len(chain) and length<=old+1e-9
            improved=length<old-.01 or required_owners is not None and fewer_sections
            if not improved or any(not domain.covers(LineString([x,y])) for x,y in zip(points,points[1:])):continue
            if required_owners is not None and abs(length-old)<=1e-9:method='collinear consolidation'
            new.extend({'net':net,'start':x,'end':y,'layer':start[0],'width':width}
                       for x,y in zip(points,points[1:]))
            remove.extend(t['uuid'] for t in chain)
            receipts.append({'method':method,'removed_native_uuids':[t['uuid'] for t in chain],
                'start':a,'end':b,'layer':start[0],'width_mm':width,
                'old_length_mm':old,'new_length_mm':length,'old_sections':len(chain),
                'new_sections':len(points)-1})
    return {'tracks':new,'vias':[],'remove':remove,'candidate_only':True,'chains':receipts}


def consolidate_dangling_vias(data, via_uuids, protected=()):
    """Candidate-only genuine chain repairs around measured obsolete drills.

    Supply geometry from before removing the vias: nominal zero-Cu connector
    stubs carry exact endpoint adjacency for the local chain planner. Native
    DRC, immutable source partitions and supplied profiles decide adoption.
    """
    requested=set(via_uuids);locked={t['net'] for t in data['tracks']+data['vias'] if t.get('locked',True)}
    locked.update(int(code) for code,n in data['nets'].items() if n['name'] in protected)
    vias=[v for v in data['vias'] if v['uuid'] in requested and v['net'] not in locked
          and v.get('through') is True and v.get('drill_mm',0)>0]
    removed_vias={v['uuid'] for v in vias};affected={}
    for via in vias:
        owners={t['uuid'] for t in data['tracks'] if t['net']==via['net'] and not t.get('locked',True)
            and not t.get('arc',True) and LineString([t['start'],t['end']]).distance(Point(via['at']))
                <=(via['drill_mm']+t['width'])/2+1e-9}
        affected[via['uuid']]=owners
    required=set().union(*affected.values()) if affected else set()
    geometry=dict(data,vias=[v for v in data['vias'] if v['uuid'] not in removed_vias],
        items=[i for i in data['items'] if i.get('uuid') not in removed_vias])
    tracks=[];cuts=set(removed_vias);chains=[]
    for net in sorted({v['net'] for v in vias}):
        proposal=shortcut(geometry,net,required)
        tracks.extend(proposal['tracks']);cuts.update(proposal['remove']);chains.extend(proposal['chains'])
    return {'tracks':tracks,'vias':[],'remove':sorted(cuts),'chains':chains,'candidate_only':True,
        'source_board_sha256':data.get('source_board_sha256'),
        'proposed_removed_via_uuids':sorted(removed_vias),
        'skipped_via_uuids':sorted(requested-removed_vias),
        'drill_affected_native_uuids':sorted(required),
        'unrepaired_affected_native_uuids':sorted(required-cuts),
        'preserve_functional_fills':True}
