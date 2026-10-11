"""Bounded coordinated trunks and partial connections on native legal polygons.

No candidate is a native acceptance receipt. A failed cluster emits no copper.
"""
from itertools import combinations
import math
import time

import shapely
from shapely.geometry import LineString,Point
from shapely.ops import nearest_points

from grid_route import routing_geometry,append_path
from polygon_path import find_path,parallel_paths,triangle_path


def passage_sections(domain,start,end,count,spacing,check_time):
    """Find finite trunk proposals from literal connected cross sections."""
    span=(count-1)*spacing/2
    core=domain.buffer(-span-.001)
    choices=[p for p in shapely.get_parts(core) if p.geom_type=='Polygon' and p.area>1e-12]
    if not choices:return []
    first=min(choices,key=lambda p:p.distance(Point(start)))
    last=min(choices,key=lambda p:p.distance(Point(end)))
    # A neck too narrow for the bundle can split the eroded core. Projecting
    # both ends into one arbitrary side then creates an unrelated pocket leg
    # and consumes the endpoint search budget without reserving the passage.
    if first is not last:return []
    part=first
    a=nearest_points(Point(start),part)[1];b=nearest_points(Point(end),part)[1]
    points,_=triangle_path(part,[a.x,a.y],[b.x,b.y],check_time)
    line=LineString(points);rows=[];distance=0.
    extent=math.hypot(domain.bounds[2]-domain.bounds[0],domain.bounds[3]-domain.bounds[1])+spacing
    for a,b in zip(points,points[1:]):
        length=math.dist(a,b)
        if length<1e-9:continue
        normal=(-(b[1]-a[1])/length,(b[0]-a[0])/length)
        for i in range(max(2,math.ceil(length/spacing)+1)):
            check_time();t=i/(max(2,math.ceil(length/spacing)+1)-1)
            p=Point(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]))
            cross=LineString([(p.x-extent*normal[0],p.y-extent*normal[1]),
                              (p.x+extent*normal[0],p.y+extent*normal[1])])
            intervals=[g for g in shapely.get_parts(domain.intersection(cross)) if g.geom_type=='LineString' and g.distance(p)<1e-7]
            if len(intervals)!=1:continue
            interval=intervals[0];middle=interval.interpolate(.5,normalized=True)
            rows.append({'distance':distance+t*length,'width':interval.length,'at':[middle.x,middle.y]})
        distance+=length
    proposals=[];covered=[]
    # A cross section is a proposal seed, never a certificate of global capacity.
    for i in sorted(range(len(rows)),key=lambda i:(rows[i]['width'],rows[i]['distance'])):
        if any(a<=i<=b for a,b in covered):continue
        lo=hi=i;threshold=rows[i]['width']+spacing
        while lo and rows[lo-1]['width']<=threshold:lo-=1
        while hi+1<len(rows) and rows[hi+1]['width']<=threshold:hi+=1
        covered.append((max(0,lo-1),min(len(rows)-1,hi+1)))
        if math.dist(rows[lo]['at'],rows[hi]['at'])>=spacing:
            proposals.append((rows[lo]['at'],rows[hi]['at'],{'minimum_sampled_center_width_mm':rows[i]['width'],'sample_count':len(rows)}))
        if len(proposals)>=3:break
    proposals.append(([points[0][0],points[0][1]],[points[-1][0],points[-1][1]],{'whole_shared_leg':True}))
    return proposals


def route_joint(data,nets,seconds=60,maximum_states=24,ordinary_first=False):
    if not math.isfinite(seconds) or seconds<=0 or type(maximum_states)is not int or maximum_states<1:
        raise ValueError('Joint routing requires positive finite budgets')
    nets=sorted(set(nets))
    if not 1<=len(nets)<=5 or any(str(net)not in data['nets']for net in nets):
        raise ValueError('A joint cluster must contain one to five existing nets')
    started=time.perf_counter();deadline=started+seconds
    context=routing_geometry(data,step=.025,preserve_source=True)
    layers=context['layers'];tracks=[];vias=[];trials=[];states=0;best=None
    def check_time():
        if time.perf_counter()>=deadline:raise TimeoutError('Joint candidate budget exhausted; infeasibility is not established')
    def checkpoint():
        return len(context['items']),context['state']['holes'],len(context['reservations']),len(tracks),len(vias)
    def rollback(saved):
        items,holes,reservations,nt,nv=saved
        del context['items'][items:];del context['reservations'][reservations:]
        context['state']['holes']=holes;context['holes']=holes
        del tracks[nt:];del vias[nv:]
    def emit(net,path):
        nt,nv,item=append_path(context,net,path)
        tracks.extend(nt);vias.extend(nv)
        return item
    def remaining_vias(net):
        limit=context['limits'].get(context['specs'][str(net)]['name'],{}).get('max_vias')
        return None if limit is None else max(0,limit-sum(v['net']==net for v in data.get('vias',[])+vias))
    def connect(net,source,target,preference=None,candidate_seconds=15):
        spec=context['specs'][str(net)]
        until=min(deadline,time.perf_counter()+candidate_seconds)
        def bounded():
            check_time()
            if time.perf_counter()>=until:raise TimeoutError('Partial connection candidate budget exhausted')
        def domains(radius,via=False,padding=None):
            return context['domains'](net,radius,via=via,padding=max(.002002,padding or 0))
        return find_path(domains,context['endpoints'],source,target,layers,spec['width'],
                         spec['via_diameter']/2,remaining_vias(net),.025,bounded,
                         preferred_layer=preference,via_spacing=spec['via_drill']+data.get('hole_to_hole',data['hole_clearance']))
    def status():
        return {net:len(context['components'](net))for net in nets}
    def remember():
        nonlocal best
        counts=status();score=(sum(c<=1 for c in counts.values()),-sum(max(0,c-1)for c in counts.values()))
        if best is None or score>best['score']:
            best={'score':score,'components':counts,'tracks':list(tracks),'vias':list(vias)}
        return all(c<=1 for c in counts.values())
    def tasks():
        result=[]
        for net in nets:
            check_time()
            groups=context['components'](net)
            if len(groups)<=1:continue
            spec=context['specs'][str(net)];legal=context['domains'](net,spec['width']/2,padding=.002002)
            vd=shapely.intersection_all(context['domains'](net,spec['via_diameter']/2,via=True,padding=.002002))
            def scarcity(group):
                check_time()
                regions=[p for layer,domain in zip(layers,legal) for p in shapely.get_parts(domain)
                         if p.geom_type=='Polygon' and any(layer in item['geometry'] and p.intersection(item['geometry'][layer]).area>1e-12 for item in group)]
                area=sum(p.intersection(vd).area for p in regions)
                physical=sum(p.area for item in group for p in item['geometry'].values())
                return (area,physical)
            scores=[scarcity(group)for group in groups]
            contacts=[context['endpoints'](group,legal)for group in groups];seen=set()
            for index in sorted(range(len(groups)),key=lambda i:scores[i]):
                check_time();source=groups[index];sources=contacts[index]
                if not sources:continue
                options=[]
                for other,target in enumerate(groups):
                    if index==other or tuple(sorted((index,other)))in seen:continue
                    points=contacts[other]
                    if points:options.append((min(math.dist(a,b)for a,_ in sources for b,_ in points),other,target))
                for distance,other,target in sorted(options,key=lambda row:row[0])[:2]:
                    seen.add(tuple(sorted((index,other))))
                    result.append((scores[index],distance,net,source,target))
        return sorted(result,key=lambda row:(row[0],row[1],row[2]))
    def search(depth=0,state_limit=None):
        nonlocal states
        check_time()
        if remember():return True
        state_limit=maximum_states if state_limit is None else state_limit
        if states>=state_limit:return False
        available=tasks()
        for _,_,net,source,target in available:
            allowed=context['limits'].get(context['specs'][str(net)]['name'],{}).get('allowed_layers')
            eligible=[layer for layer in layers if allowed is None or data['layer_names'][str(layer)]in allowed]
            preferences=[None]+(eligible if len(eligible)>1 else [])
            for preference in preferences:
                check_time()
                if states>=state_limit:return False
                states+=1;saved=checkpoint();before=len(context['components'](net))
                row={'state':states,'depth':depth,'net':net,'preferred_layer':preference}
                trials.append(row)
                try:
                    path,detail=connect(net,source,target,preference);row['search']=detail
                    # A layer preference only changes the visit state; it
                    # cannot connect a disconnected native component graph.
                    if not path and detail.get('reason')=='Continuous native-domain components disconnected':
                        # With only two remaining groups there is no same-net
                        # third leg that could supply another attachment. New
                        # foreign copper only shrinks their legal graph, so
                        # restore the preceding route choice immediately.
                        if len(context['components'](net))==2:
                            row.update(branch_rejected_net=net,
                                       branch_rejection_reason='Two remaining component groups are disconnected under append-only copper')
                            return False
                        break
                    if path:
                        emit(net,path)
                        after=len(context['components'](net))
                        if after>=before:raise ValueError('Quantized path does not join both native components')
                        row.update(connection_pass=True,components_before=before,components_after=after)
                        if search(depth+1,state_limit):return True
                except (ValueError,TimeoutError) as error:row['error']=str(error)
                rollback(saved)
        return False
    def bundle_candidates():
        independent={};pairs={}
        # Seed the same scarce partial connection the ordinary search would
        # attempt. A three-component tree may need its first escape reserved
        # together with another net before its remote branch can be joined.
        for _,_,net,source,target in tasks():pairs.setdefault(net,(source,target))
        seed_deadline=started+seconds/3
        for net in pairs:
            source,target=pairs[net]
            allowed=context['limits'].get(context['specs'][str(net)]['name'],{}).get('allowed_layers')
            eligible=[layer for layer in layers if allowed is None or data['layer_names'][str(layer)]in allowed]
            seeded={}
            for preference in [None]+(eligible if len(eligible)>1 else []):
                check_time()
                if time.perf_counter()>=seed_deadline:break
                if preference in seeded:continue
                row={'seed_net':net,'seed_preferred_layer':preference};trials.append(row)
                try:
                    path,detail=connect(net,source,target,preference,
                                        seed_deadline-time.perf_counter())
                    row.update(search=detail,seed_path_found=bool(path))
                    if not path and detail.get('reason')=='Continuous native-domain components disconnected':break
                    if path:
                        # Minimum-transition paths may stay on the pad layer.
                        # Explicit visits also seed shared trunks on another
                        # allowed layer, including their legal endpoint vias.
                        for (la,a),(lb,b)in zip(path,path[1:]):
                            if la==lb and a!=b:seeded.setdefault(la,path)
                except (ValueError,TimeoutError)as error:row['error']=str(error)
            if seeded:independent[net]=seeded
        rank={net:index for index,net in enumerate(independent)}
        bundles=sorted((participants for size in range(min(3,len(independent)),1,-1)
                        for participants in combinations(independent,size)),
                       key=lambda participants:(min(rank[net]for net in participants),-len(participants),
                                                tuple(rank[net]for net in participants)))
        for participants in bundles:
            size=len(participants)
            specs=[context['specs'][str(net)]for net in participants]
            spacing=max((a['width']+b['width'])/2+max(a['clearance'],b['clearance'])for a,b in combinations(specs,2))+.002004
            legal={net:context['domains'](net,context['specs'][str(net)]['width']/2,padding=.002004)for net in participants}
            for index,layer in enumerate(layers):
                check_time();common=shapely.intersection_all([legal[net][index]for net in participants])
                if common.is_empty:continue
                chunks={}
                for net in participants:
                    path=independent[net].get(layer,[])
                    lines=[LineString([a,b])for (la,a),(lb,b)in zip(path,path[1:])if la==lb==layer and a!=b]
                    if not lines:break
                    chunks[net]=shapely.line_merge(shapely.union_all(lines))
                if len(chunks)!=size:continue
                parts=[p for p in shapely.get_parts(common)if p.geom_type=='Polygon' and all(chunks[net].intersection(p).length>spacing for net in participants)]
                for part in sorted(parts,key=lambda p:-min(chunks[net].intersection(p).length for net in participants))[:2]:
                    ends=[];reference=None
                    for net in participants:
                        pieces=[p for p in shapely.get_parts(shapely.line_merge(chunks[net].intersection(part)))if p.geom_type=='LineString' and p.length>spacing]
                        if not pieces:break
                        piece=max(pieces,key=lambda p:p.length);a=list(piece.coords[0]);b=list(piece.coords[-1])
                        vector=(b[0]-a[0],b[1]-a[1])
                        if reference is None:reference=vector
                        if vector[0]*reference[0]+vector[1]*reference[1]<0:a,b=b,a
                        ends.append((net,a,b))
                    if len(ends)!=size:continue
                    start=[sum(row[1][i]for row in ends)/size for i in (0,1)]
                    end=[sum(row[2][i]for row in ends)/size for i in (0,1)]
                    try:sections=passage_sections(part,start,end,size,spacing,check_time)
                    except ValueError:continue
                    for a,b,section in sections:
                        check_time()
                        try:paths,detail=parallel_paths(part.buffer(-.001),a,b,size,spacing,check_time,width=max(s['width']for s in specs))
                        except ValueError:continue
                        normal=(-(b[1]-a[1]),b[0]-a[0])
                        order=sorted(ends,key=lambda row:row[1][0]*normal[0]+row[1][1]*normal[1])
                        for assignment in (order,list(reversed(order))):
                            yield [(row[0],[(layer,p)for p in path])for row,path in zip(assignment,paths)],dict(detail,layer=layer,nets=list(participants),assignment=[row[0]for row in assignment],start=a,end=b,section=section)
    complete=False
    try:
        remember()
        # Measured copper cuts can expose direct component joins before
        # floating passage lanes consume the same finite search budget.
        if ordinary_first:complete=search()
        for lanes,detail in ([] if complete else bundle_candidates()):
            check_time();saved=checkpoint();row={'bundle':detail};trials.append(row)
            try:
                for net,path in lanes:emit(net,path)
                row['whole_lane_clearance_pass']=True
                # A reserved floating lane itself needs attachment. Keep the
                # ordinary alternative allowance, but admit every nominal
                # tree edge before switching allocations. Incidental joins
                # may merge extra groups; this is no physical lower bound.
                allowance=max(12,sum(max(0,len(context['components'](net))-1)for net in nets))
                if search(state_limit=min(maximum_states,states+allowance)):complete=True;break
            except (ValueError,TimeoutError)as error:row['error']=str(error)
            rollback(saved)
            if states>=maximum_states:break
            if time.perf_counter()>=deadline:break
        if not complete and not ordinary_first:
            complete=search()
    except (ValueError,TimeoutError)as error:trials.append({'error':str(error)})
    counts=status() if complete else best['components']
    return {'tracks':tracks if complete else [],'vias':vias if complete else [],
            'completed_nets':[net for net,c in counts.items()if c<=1]if complete else [],
            'failed':[]if complete else [{'net':net,'remaining_components':c}for net,c in counts.items()if c>1],
            'source_board_sha256':data.get('source_board_sha256'),'candidate_only':True,
            'joint_trials':trials,'elapsed_seconds':time.perf_counter()-started,
            'complete_cluster':complete,'states':states,'maximum_states':maximum_states,
            'best_partial_components':best['components']}
