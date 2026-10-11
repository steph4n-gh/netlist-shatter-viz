#!/usr/bin/env python3
"""Create an unrouted native board from explicit circuit, footprint and board data."""
import json
import math
from pathlib import Path
import subprocess
import sys
import pcbnew as k
import wx


def create(spec_path,out):
    spec=json.loads(spec_path.read_text());config=spec['board'];parts=spec['components']
    width,height=float(config['width']),float(config['height']);layers=config.get('layers',2)
    if not all(math.isfinite(v) and 5<=v<=300 for v in (width,height)) or layers not in (2,4,6,8):
        raise ValueError('Creation supports 5–300 mm rectangular boards with 2, 4, 6 or 8 copper layers')
    if not 1<=len(parts)<=100:raise ValueError('Supply 1–100 components')
    refs=[p['reference'] for p in parts]
    if len(set(refs))!=len(refs):raise ValueError('Duplicate component references')
    b=k.BOARD();b.SetCopperLayerCount(layers);b.GetDesignSettings().SetBoardThickness(k.FromMM(config.get('thickness',1.6)))
    vector=lambda p:k.VECTOR2I(k.FromMM(p[0]),k.FromMM(p[1]))
    for a,z in zip([(0,0),(width,0),(width,height),(0,height)],[(width,0),(width,height),(0,height),(0,0)]):
        edge=k.PCB_SHAPE();edge.SetShape(k.SHAPE_T_SEGMENT);edge.SetStart(vector(a));edge.SetEnd(vector(z));edge.SetLayer(k.Edge_Cuts);edge.SetWidth(k.FromMM(.05));b.Add(edge)
    net_names=sorted({n for p in parts for n in p['nets'].values() if n is not None})
    if any(not isinstance(n,str) or not n for n in net_names):raise ValueError('Net names must be nonempty strings or null for explicit no-connect')
    nets={}
    for name in net_names:
        n=k.NETINFO_ITEM(b,name);b.Add(n);nets[name]=n
    library=out/'parts.pretty';library.mkdir()
    footprints=[];fixed=[];occupied=[];groups=spec.get('groups',[]);group_seeds={}
    def bounds(fp):
        box=fp.GetBoundingBox(False,False)
        return [k.ToMM(box.GetLeft()),k.ToMM(box.GetTop()),k.ToMM(box.GetRight()),k.ToMM(box.GetBottom())]
    def closed_fab(shapes):
        if not shapes:return False
        edges=[]
        for shape in shapes:
            if shape.IsClosed():
                q=shape.GetBoundingBox()
                if q.GetRight()-q.GetLeft()<=shape.GetWidth() or q.GetBottom()-q.GetTop()<=shape.GetWidth():return False
                if shape.GetShape()==k.SHAPE_T_POLY and (not shape.IsPolyShapeValid() or shape.GetPolyShape().IsSelfIntersecting()):return False
            elif shape.GetShape()==k.SHAPE_T_SEGMENT:
                a,z=shape.GetStart(),shape.GetEnd();edges.append(((a.x,a.y),(z.x,z.y)))
            else:return False
        # Native coordinates are nanometres. Each segment end must meet one
        # other segment end within 1 nm; fragments and branches stay conservative.
        near=lambda a,z:(a[0]-z[0])**2+(a[1]-z[1])**2<=1
        if any(near(a,z) for a,z in edges):return False
        if any(sum(near(p,q) for j,edge in enumerate(edges) if j!=i for q in edge)!=1 for i,edge in enumerate(edges) for p in edge):return False
        pending=edges[:]
        while pending:
            start,at=pending.pop();contour=k.SHAPE_POLY_SET();contour.NewOutline();contour.Append(*start)
            while not near(at,start):
                contour.Append(*at)
                index,edge=next((i,e) for i,e in enumerate(pending) if near(at,e[0]) or near(at,e[1]))
                pending.pop(index);at=edge[1] if near(at,edge[0]) else edge[0]
            if contour.Area()<=0 or contour.IsSelfIntersecting():return False
        return True
    def edge_legal(fp):
        pads=[];fab=[];shapes=[]
        for pad in fp.Pads():
            q=pad.GetBoundingBox();pads.append([k.ToMM(q.GetLeft()),k.ToMM(q.GetTop()),k.ToMM(q.GetRight()),k.ToMM(q.GetBottom())])
        for shape in fp.GraphicalItems():
            if isinstance(shape,k.PCB_SHAPE) and shape.GetLayer()==k.F_Fab:
                shapes.append(shape);q=shape.GetBoundingBox();half=k.ToMM(shape.GetWidth())/2
                fab.append([k.ToMM(q.GetLeft())+half,k.ToMM(q.GetTop())+half,k.ToMM(q.GetRight())-half,k.ToMM(q.GetBottom())-half])
        if closed_fab(shapes):
            body=[min(q[0] for q in fab),min(q[1] for q in fab),max(q[2] for q in fab),max(q[3] for q in fab)]
        else:body=bounds(fp)
        return body[0]>=0 and body[1]>=0 and body[2]<=width and body[3]<=height and all(q[0]>=.4 and q[1]>=.4 and q[2]<=width-.4 and q[3]<=height-.4 for q in pads)
    def separated(a,q):
        return a[2]+.3<q[0] or a[0]-.3>q[2] or a[3]+.3<q[1] or a[1]-.3>q[3]
    def legal(fp):
        # Keep the original conservative native packing/courtyard envelopes.
        # Only boundary acceptance separates body/Fab from actual pad copper.
        box=bounds(fp)
        return edge_legal(fp) and all(separated(box,q) for q in occupied)
    # Footprints are embedded exactly by the caller; never infer a package from a part name.
    for index,p in enumerate(parts):
        path=library/f'part-{index}.kicad_mod';path.write_text(p['footprint'])
        fp=k.FootprintLoad(str(library),path.stem)
        if fp is None:raise ValueError('Invalid footprint for '+p['reference'])
        fp.SetReference(p['reference']);fp.SetValue(p.get('value',''));fp.SetFPIDAsString('')
        fp.Reference().SetLayer(k.F_Fab);fp.Value().SetLayer(k.F_Fab)
        fp.SetPosition(vector([0,0]));fp.SetOrientationDegrees(float(p.get('rotation',0)))
        numbers={pad.GetNumber() for pad in fp.Pads() if pad.GetNumber()}
        if numbers!=set(p['nets']):raise ValueError(p['reference']+': map every numbered pad exactly once; use null for no-connect')
        for pad in fp.Pads():
            name=p['nets'].get(pad.GetNumber())
            if name is not None:pad.SetNet(nets[name])
        b.Add(fp);footprints.append(fp)
    for fp,p in zip(footprints,parts):
        if 'position' in p:
            fp.SetPosition(vector(p['position']))
            if not legal(fp):raise ValueError(fp.GetReference()+': fixed placement overlaps or exceeds board')
            occupied.append(bounds(fp));fixed.append(fp.GetReference())
    grouped={ref:group for group in groups for ref in group};packed=set()
    by_ref={fp.GetReference():(fp,p) for fp,p in zip(footprints,parts)}
    for fp,p in zip(footprints,parts):
        if 'position' in p:continue
        if fp.GetReference() in grouped:
            group=grouped[fp.GetReference()]
            if group[0] in packed:continue
            packed.add(group[0]);members=[by_ref[ref][0] for ref in group]
            origin=by_ref[group[0]][1]['initial_position']
            try:relative={ref:[v-o for v,o in zip(by_ref[ref][1]['initial_position'],origin)] for ref in group}
            except OverflowError:raise ValueError('Relative group initial_position exceeds native 32-bit nanometre coordinates') from None
            if any(abs(v)>2147.483647 or not math.isfinite(v) for values in relative.values() for v in values):raise ValueError('Relative group initial_position exceeds native 32-bit nanometre coordinates')
            seeds={ref:k.VECTOR2I(round(values[0]*1e6),round(values[1]*1e6)) for ref,values in relative.items()}
            group_seeds.update({ref:(v.x,v.y) for ref,v in seeds.items()})
            for member in members:member.SetPosition(seeds[member.GetReference()])
            boxes=[bounds(member) for member in members]
            if any(not separated(box,q) for i,box in enumerate(boxes) for q in boxes[:i]):raise ValueError('Relative group envelopes overlap: '+group[0])
            placed=False
            for y in range(1,math.ceil(height)):
                for x in range(1,math.ceil(width)):
                    delta=vector([x,y])
                    if any(abs(seed.x+delta.x)>2147483647 or abs(seed.y+delta.y)>2147483647 for seed in seeds.values()):continue
                    for member in members:
                        seed=seeds[member.GetReference()];member.SetPosition(k.VECTOR2I(seed.x+delta.x,seed.y+delta.y))
                    boxes=[bounds(member) for member in members]
                    if all(box[0]>=.5 and box[1]>=.5 and box[2]<=width-.5 and box[3]<=height-.5 for box in boxes) and all(legal(member) for member in members):
                        occupied.extend(boxes);placed=True;break
                if placed:break
            if not placed:raise ValueError('No initial placement fits group '+group[0])
            continue
        placed=False
        for y in range(1,math.ceil(height)):
            for x in range(1,math.ceil(width)):
                fp.SetPosition(vector([x,y]));box=bounds(fp)
                # Retain the original conservative initial packing for automatic poses.
                if box[0]>=.5 and box[1]>=.5 and box[2]<=width-.5 and box[3]<=height-.5 and legal(fp):occupied.append(box);placed=True;break
            if placed:break
        if not placed:raise ValueError('No initial placement fits '+fp.GetReference()+'; increase board size or provide positions')
    # Reuse the existing placement optimizer with conservative native envelopes.
    # These proxy pads never enter the delivered PCB; native footprints remain intact.
    model={'width':width,'height':height,'thickness':config.get('thickness',1.6),'footprints':[], 'tracks':[],'vias':[],
           'net_names':{str(n.GetNetCode()):name for name,n in nets.items()}}
    for fp in footprints:
        pos=fp.GetPosition();px,py=k.ToMM(pos.x),k.ToMM(pos.y);pads=[]
        for i,pad in enumerate(fp.Pads()):
            q=pad.GetPosition()
            pads.append({'pin_number':str(i),'pin_name':'','relative_pos':{'x':k.ToMM(q.x)-px,'y':k.ToMM(q.y)-py},
                         'shape':{'Circle':{'diameter':.001}},'pad_type':{'Smd':{'layer':'FCu'}},'net':pad.GetNetCode() or None})
        x0,y0,x1,y1=bounds(fp)
        # Preserve the old 0.5um proxy-envelope reserve without inventing copper.
        corners=[(x0-.0005,y0-.0005),(x1+.0005,y0-.0005),(x1+.0005,y1+.0005),(x0-.0005,y1+.0005)]
        envelope=[({'x':x-px,'y':y-py},{'x':z-px,'y':w-py}) for (x,y),(z,w) in zip(corners,corners[1:]+corners[:1])]
        model['footprints'].append({'reference':fp.GetReference(),'value':fp.GetValue(),'package':'native-envelope',
            'position':{'x':px,'y':py},'rotation':0,'pads':pads,'silkscreen_lines':envelope})
    tool=Path(__file__).resolve().parents[1]/'target/release/preview_layout'
    result=subprocess.run([str(tool)],input=json.dumps({'board':model,'constraints':{'fixed':fixed,'groups':groups},'seed':1}),text=True,capture_output=True,timeout=30)
    if result.returncode:raise ValueError(result.stderr)
    preview=json.loads(result.stdout)
    if preview['report']['placement_issues']:raise ValueError('; '.join(preview['report']['placement_issues']))
    poses={f['reference']:f['position'] for f in preview['board']['footprints']}
    for fp in footprints:
        p=poses[fp.GetReference()];fp.SetPosition(vector([p['x'],p['y']]))
    group_translations={}
    for group in groups:
        shifts=[]
        for ref in group:
            pos=by_ref[ref][0].GetPosition();seed=group_seeds[ref];shifts.append((pos.x-seed[0],pos.y-seed[1]))
        if any(shift!=shifts[0] for shift in shifts):raise ValueError('Native relative group geometry changed: '+group[0])
        group_translations[group[0]]=[k.ToMM(v) for v in shifts[0]]
    for fp in footprints:
        if not edge_legal(fp):raise ValueError(fp.GetReference()+': native body/pad edge check failed after placement')
        fp.BuildCourtyardCaches()
    for i,fp in enumerate(footprints):
        for other in footprints[:i]:
            for side in (k.F_Cu,k.B_Cu):
                a,z=fp.GetCourtyard(side),other.GetCourtyard(side)
                if a.OutlineCount() and z.OutlineCount() and a.Collide(z):
                    raise ValueError(fp.GetReference()+' / '+other.GetReference()+': native courtyards overlap')
    for plane in spec.get('planes',[]):
        z=k.ZONE(b);z.SetLayer(b.GetLayerID(plane['layer']));z.SetNet(nets[plane['net']])
        z.SetLocalClearance(k.FromMM(plane['clearance_mm']));z.SetMinThickness(k.FromMM(.15));z.SetPadConnection(k.ZONE_CONNECTION_FULL)
        inset=plane['edge_inset_mm'];outline=z.Outline();outline.NewOutline()
        for x,y in [(inset,inset),(width-inset,inset),(width-inset,height-inset),(inset,height-inset)]:outline.Append(k.FromMM(x),k.FromMM(y))
        b.Add(z)
    # The routing worker fills after loading the matching project/custom rules.
    pcb=out/(spec_path.stem+'.kicad_pcb')
    k.SaveBoard(str(pcb),b)
    (out/'creation.json').write_text(json.dumps({'components':len(parts),'nets':len(nets),'pads':sum(len(list(f.Pads())) for f in footprints),
        'connections':{f.GetReference():{p.GetNumber():p.GetNetname() or None for p in f.Pads() if p.GetNumber()} for f in footprints},
        'declared_planes':spec.get('planes',[]),'plane_pad_connection':'solid','initial_tracks':len(list(b.GetTracks())),'fixed_components':fixed,'placement':preview['report'],'groups':groups,'group_translations_mm':group_translations,
        'native_footprints_embedded':True,'circuit_supplied_not_inferred':True},indent=2))


if __name__=='__main__':
    app=wx.App(False)
    create(Path(sys.argv[1]),Path(sys.argv[2]))
