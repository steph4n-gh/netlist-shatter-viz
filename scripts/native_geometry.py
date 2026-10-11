"""Serialize KiCad-owned copper geometry for the bounded fallback router."""
import pcbnew as k
import json
from fractions import Fraction
from pathlib import Path


def polygons(poly):
    def points(line):
        return [[k.ToMM(line.CPoint(i).x), k.ToMM(line.CPoint(i).y)] for i in range(line.PointCount())]
    return [{'shell': points(poly.COutline(i)), 'holes': [points(poly.CHole(i,j)) for j in range(poly.HoleCount(i))]}
            for i in range(poly.OutlineCount())]


def point(p):
    return [k.ToMM(p.x), k.ToMM(p.y)]


def export_native_connectivity(board):
    """Exact item signatures and bounded native physical Cu-island contacts."""
    board.BuildConnectivity(); connectivity=board.GetConnectivity()
    layers=list(board.GetEnabledLayers().CuStack())
    pads=[p for f in board.GetFootprints() for p in f.Pads()]
    zones=list(board.Zones())+[z for f in board.GetFootprints() for z in f.Zones()]
    graphics=list(board.GetDrawings())+[q for f in board.GetFootprints()
        for q in list(f.GraphicalItems())+[f.Reference(),f.Value()]]
    actual={q.m_Uuid.AsString():q for q in pads+list(board.GetTracks())+zones+graphics
        if hasattr(q,'GetNetname') and (isinstance(q,k.PAD) or any(q.IsOnLayer(l) for l in layers))}
    from kicad_sexpr import sexpr,zone_definition
    definitions={}
    def visit(node):
        if not isinstance(node,list):return
        uid=next((v[1] for v in node if isinstance(v,list) and v and v[0]=='uuid'),None)
        if uid:definitions[json.loads(uid) if uid.startswith('"') else uid]=node
        for child in node:visit(child)
    visit(sexpr(Path(board.GetFileName()).read_text()))
    ties={};tie_pads={}
    for fp in board.GetFootprints():
        owner=fp.m_Uuid.AsString()
        declarations=[json.loads(v) for node in definitions[owner]
            if isinstance(node,list) and node and node[0]=='net_tie_pad_groups' for v in node[1:]]
        if not declarations:continue
        pads=list(fp.Pads());by_number={}
        for pad in pads:by_number.setdefault(pad.GetNumber(),[]).append(pad)
        used=set()
        for declaration in declarations:
            numbers=[v.strip() for v in declaration.split(',')]
            if '\\' in declaration or len(numbers)<2 or any(not v or v not in by_number for v in numbers) \
                    or len(set(numbers))!=len(numbers) or used.intersection(numbers):
                raise ValueError('Unsupported or invalid source net-tie group: '+owner)
            members=[pad for number in numbers for pad in by_number[number]]
            if any(not pad.GetNetname() for pad in members) or any(
                    not a.SharesNetTieGroup(b) for a in members for b in members):
                raise ValueError('Native net-tie membership differs from source: '+owner)
            used.update(numbers)
            group={'footprint_uuid':owner,'pad_group':declaration,'pad_numbers':numbers,
                   'pad_uuids':sorted(pad.m_Uuid.AsString() for pad in members)}
            for pad in members:tie_pads[pad.m_Uuid.AsString()]=group
        ties[owner]=declarations
    xy=lambda v:[int(v.x),int(v.y)]
    def poly_nm(poly):
        points=lambda line:[xy(line.CPoint(i)) for i in range(line.PointCount())]
        return [{'shell':points(poly.COutline(i)),
                 'holes':[points(poly.CHole(i,j)) for j in range(poly.HoleCount(i))]}
                for i in range(poly.OutlineCount())]
    def signature(q):
        row={'class':q.GetClass(),'net':q.GetNetname(),
             'layers':[board.GetLayerName(l) for l in layers if q.IsOnLayer(l)]}
        if isinstance(q,k.PAD):
            row.update(ref=q.GetParentFootprint().GetReference(),pin=q.GetNumber(),
                at_nm=xy(q.GetPosition()),size_nm=xy(q.GetSize()),drill_nm=xy(q.GetDrillSize()),
                shape=int(q.GetShape()),attribute=int(q.GetAttribute()),rotation_deg=q.GetOrientationDegrees(),
                offset_nm=xy(q.GetOffset()),dnp=q.GetParentFootprint().IsDNP())
        elif isinstance(q,k.PCB_VIA):
            row.update(at_nm=xy(q.GetPosition()),drill_nm=int(q.GetDrillValue()),
                diameter_nm={board.GetLayerName(l):int(q.GetWidth(l)) for l in layers if q.IsOnLayer(l)},
                via_type=int(q.GetViaType()),locked=q.IsLocked())
        elif isinstance(q,k.PCB_TRACK):
            row.update(start_nm=xy(q.GetStart()),end_nm=xy(q.GetEnd()),width_nm=int(q.GetWidth()),locked=q.IsLocked())
            if isinstance(q,k.PCB_ARC):row['mid_nm']=xy(q.GetMid())
        elif isinstance(q,k.ZONE):
            row.update(zone_definition=zone_definition(definitions[q.m_Uuid.AsString()]),
                filled_nm={board.GetLayerName(l):poly_nm(q.GetFilledPolysList(l))
                           for l in layers if q.IsOnLayer(l)},rule_area=q.GetIsRuleArea())
        else:row['native_format']=k.PCB_IO_KICAD_SEXPR().Format(q)
        fp=q.GetParentFootprint()
        if fp and fp.m_Uuid.AsString() in ties:
            row['net_tie']={'footprint_uuid':fp.m_Uuid.AsString(),
                           'pad_groups':ties[fp.m_Uuid.AsString()]}
        uid=q.m_Uuid.AsString()
        if uid not in definitions:raise ValueError('Native Cu item missing from source: '+uid)
        row['native_definition']=zone_definition(definitions[uid]) if isinstance(q,k.ZONE) else definitions[uid]
        return row
    records={uid:signature(q) for uid,q in actual.items()}
    def bezier(node):
        return isinstance(node,list) and ((node and node[0] in ('gr_curve','fp_curve'))
            or any(bezier(child) for child in node))
    if any(bezier(record['native_definition']) for record in records.values()):
        raise ValueError('Unsupported Cu/custom-pad Bezier centerline approximation')
    collision_memo={'hits':0,'misses':0,'stores':0,'ineligible':0}
    inner=_copper_contact_groups(board,actual,layers,k.ERROR_INSIDE,poly_nm,tie_pads=tie_pads,collision_memo=collision_memo)
    outer=_copper_contact_groups(board,actual,layers,k.ERROR_OUTSIDE,poly_nm,tie_pads=tie_pads,collision_memo=collision_memo)
    canonical=lambda result:sorted(tuple(g['contact_nodes']) for groups in result['groups'].values() for g in groups)
    unambiguous=(canonical(inner)==canonical(outer) and inner['contact_node_items']==outer['contact_node_items']
        and inner['declared_net_tie_contacts']==outer['declared_net_tie_contacts'])
    refined=set();different=set(inner['contact_node_items'])^set(outer['contact_node_items'])
    common=set(inner['contact_node_items'])&set(outer['contact_node_items'])
    common_parts=lambda result:sorted(tuple(sorted(common&set(g['contact_nodes'])))
        for groups in result['groups'].values() for g in groups if common&set(g['contact_nodes']))
    # Refine only a small missing-fragment disagreement, not whole components.
    affected={inner['contact_node_items'].get(n,outer['contact_node_items'].get(n)) for n in different}
    if not unambiguous and different and len(affected)<=4 and common_parts(inner)==common_parts(outer) \
            and all(actual[u].GetClass()=='PCB_TRACK' for u in affected):
        refined=set(affected)
        for uid in affected:
            box=actual[uid].GetBoundingBox()
            for owner,drill in outer['drill_bounds_nm'].items():
                b=drill['bbox'];span=drill['layers']
                if any(actual[uid].IsOnLayer(layer) for layer in span) and not (
                    box.GetRight()<b[0] or b[2]<box.GetLeft() or box.GetBottom()<b[1] or b[3]<box.GetTop()):refined.add(owner)
        if len(refined)<=8:
            inner=_copper_contact_groups(board,actual,layers,k.ERROR_INSIDE,poly_nm,refined,tie_pads,collision_memo)
            outer=_copper_contact_groups(board,actual,layers,k.ERROR_OUTSIDE,poly_nm,refined,tie_pads,collision_memo)
            unambiguous=(canonical(inner)==canonical(outer) and inner['contact_node_items']==outer['contact_node_items']
                and inner['declared_net_tie_contacts']==outer['declared_net_tie_contacts'])
        else:refined=set()
    return dict(inner,native_items=records,cross_net_contacts=outer['cross_net_contacts'],
        contact_graph_unambiguous=unambiguous,contact_schema='physical-Cu-islands/1',
        contact_refinement={'coarse_error_nm':1000,'fine_error_nm':1,'native_uuids':sorted(refined)},
        native_unconnected_count=connectivity.GetUnconnectedCount(False),
        scope='Native integer Cu polygons; zones split by filled outline. Drill voids subtracted; '
        'Whole straight PCB_TRACK capsules exactly inside native round through-via drills '
        'are nominal empty Cu items; closed-hole tangency has no Cu area. '
        'Endpoint fragments require exact disk/halfplane absence; equal-width axial separators '
        'require integral ring crossings and exact compound geometry/area preservation. '
        'only plated via/PTH barrel contacts bridge layers. Unflashed/backdrilled plated layers '
        'fail closed. Primitive conversion requests 1um; native custom primitives use board MaxError '
        'and forward ERROR_LOC; Bezier centerline approximations reject. '
        'Inside/outside component agreement '
        'required. Full zone fills are exact survivor signatures; changed refills reject. '
        'Native DRC and existing independent physical/profile guards remain required.')


def _round_via_drill(via,layer,cache=None):
    if cache is not None:
        key=(id(via),layer)
        if key not in cache:cache[key]=_round_via_drill(via,layer)
        return cache[key]
    if not isinstance(via,k.PCB_VIA) or via.GetViaType()!=k.VIATYPE_THROUGH \
            or not via.IsOnLayer(layer) or not via.FlashLayer(layer) \
            or via.IsBackdrilledOrPostMachined(layer):return None
    diameter=int(via.GetDrillValue())
    if diameter<=0 or via.GetPrimaryDrillShape()!=k.PAD_DRILL_SHAPE_CIRCLE:return None
    center=via.GetPosition();hole=via.GetEffectiveHoleShape();segment=hole.GetSeg()
    if int(hole.GetWidth())!=diameter or any(int(p.x)!=int(center.x) or int(p.y)!=int(center.y)
            for p in (segment.A,segment.B)):return None
    return diameter,(int(center.x),int(center.y))


def _endpoint_disk_inside_drill(track,via,layer,point,round_drills=None):
    drill=_round_via_drill(via,layer,round_drills);width=int(track.GetWidth())
    return drill is not None and width>0 and drill[0]>=width and \
        4*((int(point.x)-drill[1][0])**2+(int(point.y)-drill[1][1])**2)<=(drill[0]-width)**2


def _straight_track_inside_round_via_drill(track,via,layer,round_drills=None):
    """Exact closed-hole containment; a tangent boundary has no Cu area."""
    return track.GetClass()=='PCB_TRACK' and track.IsOnLayer(layer) and all(
        _endpoint_disk_inside_drill(track,via,layer,p,round_drills) for p in (track.GetStart(),track.GetEnd()))


def _rings_nm(poly):
    return [[(int(line.CPoint(i).x),int(line.CPoint(i).y)) for i in range(line.PointCount())]
        for j in range(poly.OutlineCount()) for line in
        [poly.COutline(j)]+[poly.CHole(j,h) for h in range(poly.HoleCount(j))]]


def _twice_area_nm2(poly):
    def area(line):
        p=[(int(line.CPoint(i).x),int(line.CPoint(i).y)) for i in range(line.PointCount())]
        return abs(sum(p[i][0]*p[(i+1)%len(p)][1]-p[(i+1)%len(p)][0]*p[i][1]
                       for i in range(len(p))))
    return sum(area(poly.COutline(j))-sum(area(poly.CHole(j,h))
        for h in range(poly.HoleCount(j))) for j in range(poly.OutlineCount()))


def _empty_endpoint_fragment(track,via,layer,piece,round_drills=None):
    if track.GetClass()!='PCB_TRACK' or not track.IsOnLayer(layer) or piece.ArcCount():return False
    a=track.GetStart();z=track.GetEnd();axis=(int(z.x)-int(a.x),int(z.y)-int(a.y))
    if axis==(0,0):return False
    eligible=[(end,sign) for end,sign in ((a,1),(z,-1))
        if _endpoint_disk_inside_drill(track,via,layer,end,round_drills)]
    if not eligible:return False
    points=[p for ring in _rings_nm(piece) for p in ring]
    return bool(points) and any(all(
        sign*((p[0]-int(end.x))*axis[0]+(p[1]-int(end.y))*axis[1])<=0 for p in points)
        for end,sign in eligible)


def _axial_drill_separator(track,via,layer,round_drills=None):
    if track.GetClass()!='PCB_TRACK' or not track.IsOnLayer(layer):return None
    drill=_round_via_drill(via,layer,round_drills)
    if drill is None or int(track.GetWidth())!=drill[0]:return None
    a=track.GetStart();z=track.GetEnd();ends=((int(a.x),int(a.y)),(int(z.x),int(z.y)))
    return next(((i,drill[1][i]) for i in (0,1)
        if ends[0][1-i]==ends[1][1-i]==drill[1][1-i]
        and min(ends[0][i],ends[1][i])<drill[1][i]<max(ends[0][i],ends[1][i])),None)


def _exact_halfplane_split(poly,axis,coordinate):
    """Decline any integer clipping that would round or change compound Cu."""
    if poly.ArcCount() or _twice_area_nm2(poly)<=0:return None
    rings=_rings_nm(poly);original={p for ring in rings for p in ring};crossings=set()
    for ring in rings:
        for j,a in enumerate(ring):
            b=ring[(j+1)%len(ring)]
            if a[axis]==b[axis] or not min(a[axis],b[axis])<=coordinate<=max(a[axis],b[axis]):continue
            other=Fraction(a[1-axis])+Fraction((b[1-axis]-a[1-axis])*(coordinate-a[axis]),b[axis]-a[axis])
            if other.denominator!=1:return None
            p=[0,0];p[axis]=coordinate;p[1-axis]=int(other);crossings.add(tuple(p))
    box=poly.BBox();bounds=[box.GetLeft()-1,box.GetTop()-1,box.GetRight()+1,box.GetBottom()+1]
    def clipped(side):
        b=bounds[:];b[axis+2 if side==0 else axis]=coordinate
        x0,y0,x1,y1=b;window=k.SHAPE_POLY_SET();window.NewOutline()
        for p in ((x0,y0),(x1,y0),(x1,y1),(x0,y1)):window.Append(k.VECTOR2I(*p))
        result=poly.CloneDropTriangulation();result.BooleanIntersection(window);return result
    halves=[clipped(i) for i in (0,1)]
    if any(_twice_area_nm2(p)<=0 or any(v not in original|crossings
            for ring in _rings_nm(p) for v in ring) for p in halves):return None
    if sum(_twice_area_nm2(p) for p in halves)!=_twice_area_nm2(poly):return None
    combined=halves[0].CloneDropTriangulation();combined.BooleanAdd(halves[1])
    difference=poly.CloneDropTriangulation();difference.BooleanXor(combined)
    overlap=halves[0].CloneDropTriangulation();overlap.BooleanIntersection(halves[1])
    if difference.OutlineCount() or _twice_area_nm2(overlap)!=0 or halves[0].Collide(halves[1]):return None
    if any((side==0 and p[axis]>coordinate) or (side==1 and p[axis]<coordinate)
        for side,half in enumerate(halves) for ring in _rings_nm(half) for p in ring):return None
    return halves



def _collision_key(poly,rows):
    """Ordered exact state of fresh UnitSet pieces in this immutable export."""
    if poly.ArcCount():return None
    for i in range(poly.OutlineCount()):
        for line in [poly.COutline(i)]+[poly.CHole(i,h) for h in range(poly.HoleCount(i))]:
            if not line.IsClosed() or line.Width()!=0 or line.ArcCount() or any(
                    line.IsPtOnArc(j) for j in range(line.PointCount())) or any(
                    line.IsArcSegment(j) for j in range(line.SegmentCount())):return None
    return tuple((tuple(map(tuple,row['shell'])),tuple(tuple(map(tuple,h)) for h in row['holes'])) for row in rows)


def _memo_collide(x,y,memo):
    """Reuse ordered default Booleans only within one immutable contact export."""
    a=x['collision_key'];b=y['collision_key']
    if memo is None or a is None or b is None:
        if memo is not None:memo['ineligible']+=1
        return x['shape'].Collide(y['shape'])
    key=(a,b)
    if key in memo:
        memo['hits']+=1;return memo[key]
    memo['misses']+=1
    result=x['shape'].Collide(y['shape'])
    if x['shape'].IsTriangulationUpToDate() and y['shape'].IsTriangulationUpToDate():
        memo[key]=result;memo['stores']+=1
    return result


def _copper_contact_groups(board,actual,layers,error,poly_nm,refined=(),tie_pads=None,collision_memo=None):
    """One node per physical per-layer island; no parent-zone/pad-name joins."""
    nodes={};bylayer={};bridges=[];holes={};represented=set();nominal=set()
    exact_void_tracks=[];empty_endpoint_fragments=[];axial_splits=[]
    # Source wrappers stay held by actual throughout this immutable pass.
    # Cache validated integer descriptors, including unsupported None results.
    round_drills={}
    opposite=k.ERROR_OUTSIDE if error==k.ERROR_INSIDE else k.ERROR_INSIDE
    for uid,q in sorted(actual.items()):
        max_error=1 if uid in refined else k.FromMM(.001)
        hole=None;plated=False
        if isinstance(q,k.PAD) and q.GetDrillSize().x and q.GetDrillSize().y:
            hole=k.SHAPE_POLY_SET();q.TransformHoleToPolygon(hole,0,max_error,opposite)
            plated=q.GetAttribute()==k.PAD_ATTRIB_PTH
        elif isinstance(q,k.PCB_VIA):
            proxy=k.PAD(None);proxy.SetPosition(q.GetPosition())
            proxy.SetDrillSize(k.VECTOR2I(q.GetDrillValue(),q.GetDrillValue()))
            hole=k.SHAPE_POLY_SET();proxy.TransformHoleToPolygon(hole,0,max_error,opposite)
            plated=True
        if hole is not None:
            box=hole.BBox()
            holes[uid]=(hole,plated,
                layers if isinstance(q,k.PAD) else [l for l in layers if q.IsOnLayer(l)],
                [box.GetLeft(),box.GetTop(),box.GetRight(),box.GetBottom()])
    for uid,q in sorted(actual.items()):
        straight=q.GetClass()=='PCB_TRACK'
        max_error=1 if uid in refined else k.FromMM(.001)
        hole,plated=holes[uid][:2] if uid in holes else (None,False)
        barrel=[]
        for layer in layers:
            if not q.IsOnLayer(layer):continue
            if plated and (not q.FlashLayer(layer) or q.IsBackdrilledOrPostMachined(layer)):
                raise ValueError('Unsupported unflashed/backdrilled plated Cu layer: '+uid)
            if isinstance(q,k.ZONE):
                if q.GetIsRuleArea():continue
                poly=q.GetFilledPolysList(layer).CloneDropTriangulation()
            else:
                poly=k.SHAPE_POLY_SET()
                if hasattr(q,'TransformTextToPolySet'):q.TransformTextToPolySet(poly,0,max_error,error)
                else:q.TransformShapeToPolygon(poly,layer,0,max_error,error)
            if poly.OutlineCount():nominal.add(uid)
            # Only an exact entire straight capsule can bypass polygon clipping.
            # Retain its native signature and nominal empty-item accounting.
            covering=next((owner for owner in holes
                if _straight_track_inside_round_via_drill(q,actual[owner],layer,round_drills)),None) if straight else None
            if covering is not None:
                nominal.add(uid)
                exact_void_tracks.append({'track_uuid':uid,'layer':layer,
                                          'round_native_via_uuid':covering})
                continue
            box=poly.BBox();bounds=[box.GetLeft(),box.GetTop(),box.GetRight(),box.GetBottom()]
            for void,_,span,hbox in holes.values():
                if layer not in span:continue
                if bounds[2]<hbox[0] or hbox[2]<bounds[0] or bounds[3]<hbox[1] or hbox[3]<bounds[1]:continue
                poly.BooleanSubtract(void)
            labeled=[]
            separators=sorted({separator for owner in holes
                for separator in [_axial_drill_separator(q,actual[owner],layer,round_drills)] if separator is not None}) if straight else []
            for piece in (poly.UnitSet(i) for i in range(poly.OutlineCount())):
                covering=next((owner for owner in holes
                    if _empty_endpoint_fragment(q,actual[owner],layer,piece,round_drills)),None) if straight else None
                if covering is not None:
                    empty_endpoint_fragments.append({'track_uuid':uid,'layer':layer,
                        'round_native_via_uuid':covering,'twice_area_nm2':_twice_area_nm2(piece)})
                    continue
                parts=[((),piece)]
                for axis,coordinate in separators:
                    updated=[]
                    for tag,part in parts:
                        box=part.BBox();low=(box.GetLeft(),box.GetTop())[axis];high=(box.GetRight(),box.GetBottom())[axis]
                        if high<=coordinate:updated.append((tag+(0,),part))
                        elif low>=coordinate:updated.append((tag+(1,),part))
                        else:
                            halves=_exact_halfplane_split(part,axis,coordinate)
                            if halves is None:updated.append((tag+(2,),part))
                            else:
                                axial_splits.append({'track_uuid':uid,'layer':layer,'axis':axis,
                                    'coordinate_nm':coordinate,'twice_area_nm2':_twice_area_nm2(part)})
                                updated.extend((tag+(side,),half.UnitSet(i)) for side,half in enumerate(halves)
                                               for i in range(half.OutlineCount()))
                    parts=updated
                labeled.extend(parts)
            # Exact source separator regions precede bounds, keeping real halves distinct.
            pieces=[p for tag,p in sorted(labeled,key=lambda row:(row[0],row[1].BBox().GetLeft(),
                row[1].BBox().GetTop(),row[1].BBox().GetRight(),row[1].BBox().GetBottom()))]
            for i,piece in enumerate(pieces):
                node=uid+'/'+str(layer)+'/'+str(i);box=piece.BBox()
                rows=poly_nm(piece)
                nodes[node]={'uuid':uid,'net':q.GetNetname(),'layer':layer,'shape':piece,
                    'bbox':[box.GetLeft(),box.GetTop(),box.GetRight(),box.GetBottom()],
                    'polygon_nm':rows,'collision_key':_collision_key(piece,rows) if collision_memo is not None else None}
                bylayer.setdefault(layer,[]).append(node);represented.add(uid)
                # A remote custom land of a plated pad is not a barrel contact.
                if plated and piece.Collide(hole):barrel.append(node)
        if plated and barrel:bridges.append(barrel)
    parent={node:node for node in nodes}
    def root(node):
        while parent[node]!=node:parent[node]=parent[parent[node]];node=parent[node]
        return node
    def join(a,b):parent[root(b)]=root(a)
    foreign=[];contacts={node:set() for node in nodes}
    for layer,ids in bylayer.items():
        ordered=sorted(ids,key=lambda n:nodes[n]['bbox'][0])
        for i,a in enumerate(ordered):
            x=nodes[a]
            for b in ordered[i+1:]:
                y=nodes[b]
                if y['bbox'][0]>x['bbox'][2]:break
                if y['bbox'][1]>x['bbox'][3] or x['bbox'][1]>y['bbox'][3]:continue
                if not _memo_collide(x,y,collision_memo):continue
                contacts[a].add(b);contacts[b].add(a)
                if x['net']!=y['net']:foreign.append([a,b,x['net'],y['net']])
                else:join(a,b)
    declared=[];remaining=[];tie_pads=tie_pads or {}
    for contact in foreign:
        a,b=contact[:2];left=actual[nodes[a]['uuid']];right=actual[nodes[b]['uuid']]
        group=None
        if isinstance(left,k.PAD) and isinstance(right,k.PAD):
            candidate=tie_pads.get(nodes[a]['uuid'])
            if candidate and candidate==tie_pads.get(nodes[b]['uuid']) \
                    and left.SharesNetTieGroup(right):group=candidate
        else:
            pad_node,shape_node=(a,b) if isinstance(left,k.PAD) else (b,a)
            pad=actual[nodes[pad_node]['uuid']];shape=actual[nodes[shape_node]['uuid']]
            candidate=tie_pads.get(nodes[pad_node]['uuid'])
            fp=shape.GetParentFootprint()
            if isinstance(pad,k.PAD) and isinstance(shape,k.PCB_SHAPE) and not shape.GetNetname() \
                    and candidate and fp and fp.m_Uuid.AsString()==candidate['footprint_uuid']:
                touched={nodes[node]['uuid'] for node in contacts[shape_node]}
                if touched==set(candidate['pad_uuids']):group=candidate
        if group:declared.append(dict(group,contact_nodes=[a,b]))
        else:remaining.append(contact)
    foreign=remaining
    for bridge in bridges:
        for node in bridge[1:]:join(bridge[0],node)
    components={}
    for node,row in nodes.items():
        if row['net']:components.setdefault(row['net'],{}).setdefault(root(node),[]).append(node)
    groups={net:[{'contact_nodes':sorted(ids),
        'native_item_uuids':sorted({nodes[node]['uuid'] for node in ids})}
        for ids in parts.values()] for net,parts in components.items()}
    return {'groups':groups,'contact_node_items':{node:row['uuid'] for node,row in nodes.items()},
        'drill_bounds_nm':{uid:{'bbox':row[3],'layers':row[2]} for uid,row in holes.items()},
        'contact_nodes':{node:{k:v for k,v in row.items() if k not in ('shape','bbox','collision_key')} for node,row in nodes.items()},
        'empty_cu_items':sorted(nominal-represented),
        'exact_drill_void_tracks':exact_void_tracks,
        'empty_endpoint_fragments':empty_endpoint_fragments,'axial_drill_splits':axial_splits,
        'declared_net_tie_contacts':declared,'cross_net_contacts':foreign,'unrepresented_net_items':sorted(uid for uid,q in actual.items()
            if q.GetNetname() and uid not in nominal and not isinstance(q,k.ZONE))}


def export_geometry(board):
    layers = list(board.GetEnabledLayers().CuStack())
    outline = k.SHAPE_POLY_SET()
    if not board.GetBoardPolygonOutlines(outline, False):
        raise ValueError('Invalid outline')
    settings = board.GetDesignSettings()
    project=json.loads(Path(board.GetFileName()).with_suffix('.kicad_pro').read_text())
    classes=project.get('net_settings',{}).get('classes',[])
    items, holes, keepouts, nets, drill_obstacles = [], [], [], {}, []
    def add(item, terminals=(), net=0):
        shapes = {}
        for layer in layers:
            if item.IsOnLayer(layer):
                poly = k.SHAPE_POLY_SET()
                if hasattr(item,'TransformTextToPolySet'):
                    # Text's generic shape is an envelope; route around the
                    # actual native stroke copper instead.
                    item.TransformTextToPolySet(poly,0,k.FromMM(.001),k.ERROR_OUTSIDE)
                else:item.TransformShapeToPolygon(poly, layer, 0, k.FromMM(.001), k.ERROR_OUTSIDE)
                if poly.OutlineCount():shapes[str(layer)] = polygons(poly)
        if shapes:
            clearance=max(0, item.GetOwnClearance(k.UNDEFINED_LAYER)) if hasattr(item,'GetOwnClearance') else 0
            items.append({'uuid':item.m_Uuid.AsString(),'net':net,'shapes':shapes,'terminals':[point(p) for p in terminals],
                          'clearance':k.ToMM(clearance),'kind':'pad' if isinstance(item,k.PAD) else 'copper'})
            if isinstance(item,k.PAD):
                drill=item.GetDrillSize()
                items[-1]['drill_forbidden']=not drill.x and not drill.y
    for fp in board.GetFootprints():
        for pad in fp.Pads():
            net=pad.GetNetCode()
            add(pad,[pad.GetPosition()] if net else [],net)
            if pad.GetDrillSize().x:
                poly=k.SHAPE_POLY_SET()
                pad.TransformHoleToPolygon(poly,0,k.FromMM(.001),k.ERROR_OUTSIDE)
                holes.extend(polygons(poly))
                drill_obstacles.append({'net':net if pad.GetAttribute()!=k.PAD_ATTRIB_NPTH else 0,
                    'shapes':polygons(poly),
                    'clearance':k.ToMM(max(0,pad.GetOwnClearance(k.UNDEFINED_LAYER)))})
            if net and str(net) not in nets:
                # KiCad 10's shared_ptr NETCLASS binding is opaque. Resolve the
                # native class name against the supplied project, conservatively
                # using all classes if KiCad reports a composite class name.
                applicable=[c for c in classes if c['name']==pad.GetNetClassName()] or classes
                value=lambda key,minimum:max([k.ToMM(minimum)]+[c.get(key,0) or 0 for c in applicable])
                drill=value('via_drill',settings.m_MinThroughDrill)
                nets[str(net)]={'width':value('track_width',settings.m_TrackMinWidth),
                    'clearance':value('clearance',settings.m_MinClearance),
                    'via_diameter':max(value('via_diameter',settings.m_ViasMinSize),drill+2*k.ToMM(settings.m_ViasMinAnnularWidth)),
                    'via_drill':drill,
                    'name':pad.GetNetname()}
        for graphic in list(fp.GraphicalItems())+[fp.Reference(),fp.Value()]:add(graphic)
    for track in board.GetTracks():
        add(track,[track.GetStart(),track.GetEnd()],track.GetNetCode())
        if isinstance(track,k.PCB_VIA):
            p=point(track.GetPosition());r=k.ToMM(track.GetDrillValue())/2
            holes.append({'circle':[p[0],p[1],r]})
    for drawing in board.GetDrawings():add(drawing)
    for zone in list(board.Zones())+[z for fp in board.GetFootprints() for z in fp.Zones()]:
        if zone.GetIsRuleArea():
            keepouts.append({'layers':[v for v in layers if zone.IsOnLayer(v)],'shapes':polygons(zone.Outline()),
                             'tracks':zone.GetDoNotAllowTracks(),'vias':zone.GetDoNotAllowVias()})
    tracks=[{'uuid':t.m_Uuid.AsString(),'net':t.GetNetCode(),'start':point(t.GetStart()),'end':point(t.GetEnd()),
             'layer':t.GetLayer(),'width':k.ToMM(t.GetWidth()),'locked':t.IsLocked(),'arc':isinstance(t,k.PCB_ARC)}
            for t in board.GetTracks() if not isinstance(t,k.PCB_VIA)]
    vias=[{'net':t.GetNetCode(),'at':point(t.GetPosition()),'uuid':t.m_Uuid.AsString(),'locked':t.IsLocked()}
          for t in board.GetTracks() if isinstance(t,k.PCB_VIA)]
    return {'layers':layers,'outline':polygons(outline),'items':items,'holes':holes,'keepouts':keepouts,'nets':nets,'tracks':tracks,'vias':vias,
            'drill_obstacles':drill_obstacles,'hole_to_hole':k.ToMM(settings.m_HoleToHoleMin),
            'edge_clearance':k.ToMM(settings.m_CopperEdgeClearance),'hole_clearance':k.ToMM(settings.m_HoleClearance)}

def export_dc_geometry(board):
    """Native copper and terminal geometry for the explicit DC screening model."""
    retained_tracks=list(board.GetTracks());retained_footprints=list(board.GetFootprints())
    g=export_geometry(board)
    g['layer_names']={str(l):board.GetLayerName(l) for l in g['layers']}
    g['board_thickness_mm']=k.ToMM(board.GetDesignSettings().GetBoardThickness())
    g['zones']=[{'net':z.GetNetCode(),'layer':layer,'filled':polygons(z.GetFilledPolysList(layer)),'clearance_mm':k.ToMM(max(0,z.GetLocalClearance())),'teardrop':z.IsTeardropArea()} for z in board.Zones() if not z.GetIsRuleArea() for layer in g['layers'] if z.IsOnLayer(layer)]
    g['pads']=[]
    for fp in retained_footprints:
        for pad in fp.Pads():
            data={'ref':fp.GetReference(),'pin':pad.GetNumber(),'net':pad.GetNetCode(),'at':point(pad.GetPosition()),'layer':pad.GetLayer(),'dnp':fp.IsDNP(),'size':point(pad.GetSize()),'drill':point(pad.GetDrillSize()),'shapes':{}}
            for layer in g['layers']:
                if pad.IsOnLayer(layer):
                    poly=k.SHAPE_POLY_SET();pad.TransformShapeToPolygon(poly,layer,0,k.FromMM(.001),k.ERROR_OUTSIDE)
                    if poly.OutlineCount():data['shapes'][str(layer)]=polygons(poly)
            g['pads'].append(data)
    g['vias']=[{'net':v.GetNetCode(),'at':point(v.GetPosition()),'drill_mm':k.ToMM(v.GetDrillValue()),'diameter_mm':k.ToMM(v.GetWidth(k.F_Cu)),'uuid':v.m_Uuid.AsString(),'through':v.GetViaType()==k.VIATYPE_THROUGH,'locked':v.IsLocked()} for v in retained_tracks if isinstance(v,k.PCB_VIA)]
    return g
