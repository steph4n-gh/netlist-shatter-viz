"""Propose clearance-limited widening of existing straight supply traces.

Independent native DRC and the explicit DC scenario must approve every proposal.
"""
import math
from shapely.geometry import LineString
from grid_route import shape


def plan(data, names, factor, max_width_mm=2.0):
    if not math.isfinite(factor) or factor<=1:raise ValueError('Width factor must exceed one')
    if not math.isfinite(max_width_mm) or max_width_mm<=0:raise ValueError('Width ceiling must be positive')
    codes={int(k) for k,n in data['nets'].items() if n['name'] in names}
    outline=shape(data['outline']);changes=[]
    # Cache obstacle polygons once per net/layer instead of per trace.
    obstacles={}
    for t in data['tracks']:
        if t['net'] not in codes or t['locked'] or t['arc']:continue
        key=(t['net'],t['layer'])
        if key not in obstacles:
            clearance=data['nets'][str(t['net'])]['clearance']
            objects=[(shape(i['shapes'][str(t['layer'])]),max(clearance,i['clearance']))
                     for i in data['items'] if i['net']!=t['net'] and str(t['layer']) in i['shapes']]
            objects += [(shape(k['shapes']),0) for k in data['keepouts'] if k['tracks'] and t['layer'] in k['layers']]
            objects += [(shape(h['shapes']),data['hole_clearance']) for h in data.get('drill_obstacles',[]) if h['net']!=t['net']]
            obstacles[key]=objects
        line=LineString([t['start'],t['end']])
        if not outline.covers(line):continue
        def width_for(part):
            radius=part.distance(outline.boundary)-data['edge_clearance']-.002
            for geometry,clearance in obstacles[key]:radius=min(radius,part.distance(geometry)-clearance-.002)
            return max(t['width'],math.floor(min(t['width']*factor,max_width_mm,2*radius)*100+1e-9)/100)
        width=width_for(line)
        change={'uuid':t['uuid'],'old_width_mm':t['width'],'width_mm':width,'net':data['nets'][str(t['net'])]['name']}
        # A single nearby pad should constrain only its short neck, not the
        # entire supply trunk. Check the full polygon distance of each piece.
        if width<min(t['width']*factor,max_width_mm)-.01 and line.length>.5:
            count=math.ceil(line.length/.5);pieces=[]
            for i in range(count):
                a=list(line.interpolate(i/count,normalized=True).coords[0]);b=list(line.interpolate((i+1)/count,normalized=True).coords[0])
                w=width_for(LineString([a,b]))
                if pieces and pieces[-1]['width_mm']==w:pieces[-1]['end']=b
                else:pieces.append({'start':a,'end':b,'width_mm':w})
            change['width_mm']=max(p['width_mm'] for p in pieces)
            if len(pieces)>1:change['segments']=pieces
        if change['width_mm']>t['width']+.005:changes.append(change)
    return {'changes':changes,'factor':factor,'max_width_mm':max_width_mm,'candidate_only':True}
