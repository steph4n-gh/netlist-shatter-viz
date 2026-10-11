"""Continuous polygon candidate paths using the caller's native geometry policy.

Candidate generation only: native DRC and fixed profile checks decide acceptance.
"""
from collections import deque
import heapq
import math
import time
import numpy as np
import shapely
from shapely.geometry import Point,LineString
from shapely.ops import nearest_points

def _polish_path(domain,points,width,check_time):
    """Optional shortest visibility path in a width-bounded local corridor."""
    if width is None or not math.isfinite(width) or width<=0 or points[0]==points[-1]:return points
    deadline=time.perf_counter()+.5
    line=LineString(points);local=domain.intersection(line.buffer(width))
    choices=[p for p in shapely.get_parts(local) if p.geom_type=='Polygon' and p.covers(line)]
    if len(choices)!=1:return points
    corridor=choices[0];vertices=[tuple(points[0]),tuple(points[-1])]
    for ring in (corridor.exterior,*corridor.interiors):
        if len(vertices)+len(ring.coords)-1>256:return points
        vertices.extend(tuple(p) for p in list(ring.coords)[:-1])
    vertices=list(dict.fromkeys(vertices))
    if len(vertices)>256:return points
    adjacency=[[] for _ in vertices]
    for i,a in enumerate(vertices):
        check_time()
        if time.perf_counter()>=deadline:return points
        for j in range(i):
            b=vertices[j]
            if corridor.covers(LineString([a,b])):
                length=math.dist(a,b);adjacency[i].append((j,length));adjacency[j].append((i,length))
    distance={0:0.};parent={};queue=[(0.,0)]
    while queue:
        check_time()
        if time.perf_counter()>=deadline:return points
        cost,node=heapq.heappop(queue)
        if cost!=distance[node]:continue
        if node==1:break
        for other,length in adjacency[node]:
            value=cost+length
            if value<distance.get(other,math.inf):
                distance[other]=value;parent[other]=node;heapq.heappush(queue,(value,other))
    if 1 not in distance or distance[1]>=line.length-1e-9:return points
    chain=[1]
    while chain[-1]!=0:chain.append(parent[chain[-1]])
    result=[list(vertices[node]) for node in reversed(chain)]
    return result if all(domain.covers(LineString([a,b])) for a,b in zip(result,result[1:])) else points


def triangle_path(domain,start,end,check_time=lambda:None,width=None):
    """Triangle adjacency yields a covered path; visibility pulling shortens it."""
    if not domain.covers(Point(start)) or not domain.covers(Point(end)):raise ValueError('Endpoint outside exact legal component')
    if domain.covers(LineString([start,end])):return [list(start),list(end)],0
    check_time()
    triangles=list(shapely.get_parts(shapely.constrained_delaunay_triangles(domain)))
    if not triangles:raise ValueError('No constrained triangulation')
    if any(not domain.covers(t) for t in triangles):raise ValueError('Triangulation crosses domain boundary/hole')
    adjacency=[{} for _ in triangles];owners={}
    for index,triangle in enumerate(triangles):
        if index%1024==0:check_time()
        vertices=list(triangle.exterior.coords)[:-1]
        for a,b in zip(vertices,vertices[1:]+vertices[:1]):
            edge=tuple(sorted((a,b)))
            if edge in owners:
                other=owners[edge];mid=[(a[0]+b[0])/2,(a[1]+b[1])/2]
                adjacency[index][other]=mid;adjacency[other][index]=mid
            else:owners[edge]=index
    starts=np.flatnonzero(shapely.covers(triangles,Point(start)));goals=set(map(int,np.flatnonzero(shapely.covers(triangles,Point(end)))))
    centroids=[(t.centroid.x,t.centroid.y) for t in triangles]
    heap=[];cost={};parent={}
    for index in starts:
        index=int(index);cost[index]=math.dist(start,centroids[index]);heapq.heappush(heap,(cost[index],index))
    found=None
    while heap:
        check_time()
        c,node=heapq.heappop(heap)
        if c!=cost[node]:continue
        if node in goals:found=node;break
        for other in adjacency[node]:
            candidate=c+math.dist(centroids[node],centroids[other])
            if candidate<cost.get(other,math.inf):cost[other]=candidate;parent[other]=node;heapq.heappush(heap,(candidate,other))
    if found is None:raise ValueError('Triangle graph disconnected; no inferred bridge')
    chain=[found]
    while chain[-1] in parent:chain.append(parent[chain[-1]])
    chain.reverse();points=[list(start)]
    for a,b in zip(chain,chain[1:]):points.extend([list(centroids[a]),adjacency[a][b]])
    points.extend([list(centroids[chain[-1]]),list(end)])
    if any(not domain.covers(LineString([a,b])) for a,b in zip(points,points[1:])):raise ValueError('Triangle path leaves exact legal component')
    # Same line-coverage policy as the existing grid router's taut-path pass.
    result=[];index=0
    while index<len(points)-1:
        check_time()
        result.append(points[index]);other=len(points)-1
        while other>index+1 and not domain.covers(LineString([points[index],points[other]])):other-=1
        index=other
    result.append(points[-1])
    return _polish_path(domain,result,width,check_time),len(triangles)


def parallel_paths(domain,start,end,count,spacing,check_time=lambda:None,width=None):
    """Allocate a same-layer passage together; endpoint fanout remains separate.

    The caller supplies a shared legal center domain and required center spacing.
    Every offset and pair is checked literally. This produces finite candidates,
    including around bends; it never infers usable capacity from area alone.
    """
    if type(count)is not int or count<1 or not math.isfinite(spacing) or spacing<=0:
        raise ValueError('Lane count and spacing must be positive')
    # Retain a tiny construction guard above floating offset arithmetic; the
    # caller still quantizes once and checks the resulting native geometry.
    pitch=spacing+1e-9
    span=(count-1)*pitch/2
    center_domain=domain.buffer(-span) if span else domain
    parts=[p for p in shapely.get_parts(center_domain) if p.geom_type=='Polygon'
           and p.covers(Point(start)) and p.covers(Point(end))]
    if len(parts)!=1:raise ValueError('No shared passage for the requested lane bundle')
    center,triangles=triangle_path(parts[0],start,end,check_time,width)
    line=LineString(center);paths=[]
    for i in range(count):
        check_time();offset=i*pitch-span
        lane=line if offset==0 else line.offset_curve(offset,join_style='mitre')
        if lane.geom_type!='LineString' or not domain.covers(lane):
            raise ValueError('Offset lane leaves the shared legal passage')
        if any(lane.distance(LineString(other))<spacing for other in paths):
            raise ValueError('Offset lanes lose required clearance at a bend')
        paths.append([list(point) for point in lane.coords])
    return paths,{'lane_count':count,'center_span_mm':2*span,'triangle_count':triangles}


def find_path(domains,endpoints,source,target,layers,width,via_radius,via_budget,step,check_time,on_disconnected=None,preferred_layer=None,via_spacing=None):
    """Return layer/point pairs; never infer a bridge across disjoint legal regions.

The existing endpoint callback preserves locked dangling-tip preference. Native
pad contacts are computed inside the candidate domain by that same callback.
"""
    attachment=domains(width/2,padding=0)
    upper=.003+step/math.sqrt(2)
    def fallback():
        path,detail=find_path(domains,endpoints,source,target,layers,width,via_radius,via_budget,step,check_time,on_disconnected)
        used=bool(path) and any(la==lb==preferred_layer and a!=b for (la,a),(lb,b) in zip(path,path[1:]))
        return path,dict(detail,preferred_layer=preferred_layer,preferred_layer_used=used)
    def model(padding):
        check_time()
        legal=domains(width/2,padding=padding)
        via=shapely.intersection_all(domains(via_radius,via=True,padding=padding))
        parts={(layer,index):part for layer,domain in zip(layers,legal)
               for index,part in enumerate(shapely.get_parts(domain))
               if part.geom_type=='Polygon' and part.area>1e-12}
        ports={n:p.intersection(via) for n,p in parts.items()};edges={n:set() for n in parts};nodes=list(parts)
        for i,a in enumerate(nodes):
            check_time()
            if ports[a].is_empty:continue
            for b in nodes[i+1:]:
                if a[0]!=b[0] and not ports[b].is_empty and ports[a].intersection(ports[b]).area>1e-12:
                    edges[a].add(b);edges[b].add(a)
        def contacts(group):
            found={}
            for point,layer in sorted(endpoints(group,legal)):
                for node,part in parts.items():
                    if node[0]!=layer:continue
                    near=nearest_points(Point(point),part)[1];q=(near.x,near.y)
                    # GEOS can place a computed boundary projection a few ULPs
                    # outside its polygon. Move inward and check the connector;
                    # never enlarge the domain or infer a bridge.
                    if not part.covers(near):
                        interior=part.buffer(-1e-7)
                        if interior.is_empty:continue
                        near=nearest_points(Point(point),interior)[1];q=(near.x,near.y)
                        if not part.covers(near):continue
                    # Same local endpoint attachment reach as the 7x7 grid window;
                    # its short connector is checked in the exact native domain.
                    if math.dist(point,q)>3*step*math.sqrt(2)+1e-9:continue
                    if not attachment[layers.index(layer)].covers(LineString([point,q])):continue
                    found.setdefault(node,[]).append((point,q))
            return found
        starts,goals=contacts(source),contacts(target)
        if preferred_layer is not None:
            # A same-layer source and goal can share a component yet prefer an
            # out-and-back route. Keep the visit state, not just the component.
            queue=deque((n,n[0]==preferred_layer) for n in sorted(starts))
            parent={state:None for state in queue};depth={state:0 for state in queue};finish=None
            while queue:
                check_time();state=queue.popleft();node,visited=state
                if node in goals and visited:finish=state;break
                if via_budget is not None and depth[state]>=via_budget:continue
                for other in sorted(edges[node]):
                    next_state=(other,visited or other[0]==preferred_layer)
                    if next_state not in parent:
                        parent[next_state]=state;depth[next_state]=depth[state]+1;queue.append(next_state)
            if finish is None:return None
            chain=[finish]
            while parent[chain[-1]] is not None:chain.append(parent[chain[-1]])
            chain=[state[0] for state in reversed(chain)]
            start,end=min(((a,b) for a in starts[chain[0]] for b in goals[chain[-1]]),
                          key=lambda pair:(math.dist(pair[0][1],pair[1][1]),pair))
            return legal,via,parts,chain,start,end
        queue=deque(sorted(starts));parent={n:None for n in starts};depth={n:0 for n in starts};finish=None
        while queue:
            check_time();node=queue.popleft()
            if node in goals:finish=node;break
            if via_budget is not None and depth[node]>=via_budget:continue
            for other in sorted(edges[node]):
                if other not in parent:parent[other]=node;depth[other]=depth[node]+1;queue.append(other)
        if finish is None:
            if padding==0 and on_disconnected is not None:
                on_disconnected([(node[0],parts[node]) for node in sorted(parent)])
            return None
        chain=[finish]
        while parent[chain[-1]] is not None:chain.append(parent[chain[-1]])
        chain.reverse()
        start,end=min(((a,b) for a in starts[chain[0]] for b in goals[chain[-1]]),
                      key=lambda pair:(math.dist(pair[0][1],pair[1][1]),pair))
        return legal,via,parts,chain,start,end
    # Positive guard is measured from geometry, rather than removing clearance
    # globally. Disconnected native geometry remains a bounded candidate miss.
    initial=model(0)
    if initial is None:
        if preferred_layer is not None:return fallback()
        return None,{'reason':'Continuous native-domain components disconnected'}
    full=model(upper)
    low,high=(upper,upper) if full is not None else (0.,upper)
    if full is None:
        for _ in range(10):
            middle=(low+high)/2
            if model(middle) is not None:low=middle
            else:high=middle
    padding=low/4
    if padding<=1e-9:
        if preferred_layer is not None:return fallback()
        return None,{'reason':'No positive guarded continuous passage'}
    selected=model(padding)
    if selected is None:
        if preferred_layer is not None:return fallback()
        return None,{'reason':'Guarded continuous component graph disconnected'}
    legal,via,parts,chain,start,end=selected;via_points=[]
    for a,b in zip(chain,chain[1:]):
        check_time();common=parts[a].intersection(parts[b]).intersection(via)
        if common.area<=1e-12:raise ValueError('Via port lacks positive-area legal center region')
        if preferred_layer is None:
            circle=shapely.maximum_inscribed_circle(common,tolerance=.001);point=Point(circle.coords[0])
        else:
            # Distinct endpoint-oriented sites keep F/B/F from selecting the
            # same MIC twice. Existing holes remain in the caller's via domain.
            spacing=(2*via_radius if via_spacing is None else via_spacing)+padding
            for previous in via_points:
                common=common.difference(Point(previous).buffer(spacing/math.cos(math.pi/64),quad_segs=16))
            if common.is_empty or common.area<=1e-12:return fallback()
            interior=common.buffer(-1e-6)
            if interior.is_empty:return fallback()
            fraction=(len(via_points)/(len(chain)-2) if len(chain)>2 else
                      (0 if chain[0][0]!=preferred_layer else 1))
            anchor=[a+(b-a)*fraction for a,b in zip(start[1],end[1])]
            point=nearest_points(Point(anchor),interior)[1]
            if not common.covers(point):return fallback()
        if not common.covers(point):raise ValueError('Derived via center leaves native legal region')
        via_points.append((point.x,point.y))
    compact=[(chain[0][0],list(start[0]))];counts=[]
    for i,node in enumerate(chain):
        a=start[1] if i==0 else via_points[i-1];b=end[1] if i==len(chain)-1 else via_points[i]
        try:path,count=triangle_path(parts[node],a,b,check_time,width)
        except ValueError:
            if preferred_layer is not None:return fallback()
            raise
        counts.append(count)
        if any(not legal[layers.index(node[0])].covers(LineString([a,b])) for a,b in zip(path,path[1:])):
            raise ValueError('Proposed segment leaves guarded native domain')
        compact.extend((node[0],list(point)) for point in path)
    compact.append((chain[-1][0],list(end[0])))
    detail={'candidate_padding_mm':padding,'connected_padding_mm':low,'disconnected_padding_mm':high,
            ('minimum_new_vias' if preferred_layer is None else 'candidate_new_vias'):len(chain)-1,
            'triangle_counts':counts}
    if preferred_layer is not None:
        detail.update(preferred_layer=preferred_layer,
                      preferred_layer_used=any(la==lb==preferred_layer and a!=b
                          for (la,a),(lb,b) in zip(compact,compact[1:])))
    return compact,detail


def first_via_prefix(domains,endpoints,source,target,layers,width,via_radius,via_budget,step,check_time,on_disconnected=None):
    """One source-attached escape to an actual opposite-layer target portal.

    This is partial candidate copper, never a whole-net completion label. The
    caller supplies its unchanged native domains and fixed pad/main contacts.
    """
    if via_budget is not None and via_budget<1:return None,{'reason':'No remaining native via budget for escape prefix'}
    _,guard=find_path(domains,endpoints,source,target,layers,width,via_radius,1,step,check_time,on_disconnected)
    if 'candidate_padding_mm'not in guard:return None,guard
    padding=guard['candidate_padding_mm'];legal=domains(width/2,padding=padding)
    attachment=domains(width/2,padding=0);via=shapely.intersection_all(domains(via_radius,via=True,padding=padding))
    parts={(layer,index):part for layer,domain in zip(layers,legal)for index,part in enumerate(shapely.get_parts(domain))if part.geom_type=='Polygon'and part.area>1e-12};choices=[]
    for endpoint,layer in sorted(endpoints(source,legal)):
        check_time()
        for node,part in parts.items():
            if node[0]!=layer:continue
            near=nearest_points(Point(endpoint),part)[1];start=(near.x,near.y)
            if math.dist(endpoint,start)>3*step*math.sqrt(2)+1e-9 or not attachment[layers.index(layer)].covers(LineString([endpoint,start])):continue
            for other,op in parts.items():
                if other[0]==layer or not any(other[0]in item['geometry']and op.intersection(item['geometry'][other[0]]).area>1e-12 for item in target):continue
                common=part.intersection(op).intersection(via);interior=common.buffer(-1e-6)
                if interior.is_empty:continue
                q=nearest_points(Point(endpoint),interior)[1];point=[round(q.x,6),round(q.y,6)]
                if common.covers(Point(point)):choices.append((math.dist(endpoint,point),node,other,list(endpoint),list(start),point,common.area))
    if not choices:return None,{'reason':'No positive guarded source-to-opposite-layer target portal',**guard}
    _,node,other,endpoint,start,point,area=min(choices,key=lambda x:(x[0],x[1],x[2],x[3],x[5]));path,count=triangle_path(parts[node],start,point,check_time,width)
    compact=[(node[0],endpoint)]+[(node[0],p)for p in path]+[(other[0],point)]
    return compact,dict(guard,escape_prefix_only=True,source_part=node,goal_part=other,portal_area_mm2=area,finite_port_count=len(choices),triangle_count=count)
