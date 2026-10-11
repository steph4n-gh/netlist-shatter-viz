"""Clipped-cell sheet-resistance screen; not a field/thermal solver.

Each separate copper fragment in a square gets a separate node. Neighboring
fragments conduct only across a shared edge, never across a gap or a point.
Boundary and contact-cell approximations require a mesh refinement check.
"""
import math
import numpy as np
import shapely
from shapely.geometry import Point,LineString,box
from grid_route import shape


def add_sheet(network,domain,contacts,pitch,resistivity,thickness,label,max_cells=1000000):
    if domain.is_empty:raise ValueError('Empty copper sheet: '+label)
    if not domain.is_valid:raise ValueError('Invalid copper polygon: '+label)
    x0,y0,x1,y1=domain.bounds
    ix0,iy0=math.floor(x0/pitch),math.floor(y0/pitch)
    nx,ny=math.ceil(x1/pitch)-ix0,math.ceil(y1/pitch)-iy0
    if nx*ny>max_cells:raise ValueError('Sheet mesh exceeds 1000000 cells per layer; choose a coarser starting pitch')
    indices=[(x,y) for x in range(ix0,ix0+nx) for y in range(iy0,iy0+ny)]
    squares=shapely.box(np.array([x*pitch for x,y in indices]),np.array([y*pitch for x,y in indices]),np.array([(x+1)*pitch for x,y in indices]),np.array([(y+1)*pitch for x,y in indices]))
    shapely.prepare(domain)
    inside=shapely.covers(domain,squares);intersects=shapely.intersects(domain,squares)
    cells={};polys=[];nodes=[]
    for i,(x,y) in enumerate(indices):
        if not intersects[i]:continue
        clipped=squares[i] if inside[i] else domain.intersection(squares[i])
        for part in shapely.get_parts(clipped):
            if part.geom_type!='Polygon' or part.area<1e-12:continue
            node=('sheet',label,x,y,len(cells.get((x,y),[])))
            network.root(node);cells.setdefault((x,y),[]).append((node,part));polys.append(part);nodes.append(node)
    for (x,y),parts in cells.items():
        for neighbor in ((x+1,y),(x,y+1)):
            for a,pa in parts:
                for b,pb in cells.get(neighbor,[]):
                    face=pa.intersection(pb).length
                    if face>1e-10:network.add(a,b,resistivity*pitch/(thickness*face),'sheet:'+label)
    tree=shapely.STRtree(polys)
    for terminal,contact in contacts:
        # Contact cells are equipotential: deliberately exposed as a mesh-dependent approximation.
        for i in tree.query(contact,predicate='intersects'):
            if polys[i].intersection(contact).area>1e-12:network.join(terminal,nodes[i])
    return len(nodes)


def add_board_sheets(network,g,codes,terminals,pitch,rho,copper_mm,plating_mm,board_mm):
    layers=g['layers'];count=0
    holes=shape(g.get('holes',[]))
    for code in codes:
        objects={l:[] for l in layers};contacts={l:[] for l in layers}
        for z in g.get('zones',[]):
            if z['net']==code and z.get('filled'):objects[z['layer']].append(shape(z['filled']))
        for t in g['tracks']:
            if t['net']!=code:continue
            if t['arc']:raise ValueError('Arcs not supported by sheet DC screen')
            objects[t['layer']].append(LineString([t['start'],t['end']]).buffer(t['width']/2,quad_segs=16))
        for p in g['pads']:
            if p['net']!=code or not p['pin']:continue
            key=p['ref']+'.'+p['pin'];node=('pad',key);terminals[key]=node;network.root(node)
            for layer,polys in p['shapes'].items():
                poly=shape(polys).difference(holes);objects[int(layer)].append(poly);contacts[int(layer)].append((node,poly))
        for v in g['vias']:
            if v['net']!=code:continue
            if not v.get('through',True):raise ValueError('Blind/buried/microvia resistance is not supported by this DC screen')
            pt=Point(v['at']);poly=pt.buffer(v['diameter_mm']/2,quad_segs=16).difference(pt.buffer(v['drill_mm']/2,quad_segs=16))
            nodes={l:('via',v['uuid'],l) for l in layers}
            for l in layers:objects[l].append(poly);contacts[l].append((nodes[l],poly))
            area=math.pi*((v['drill_mm']/2+plating_mm)**2-(v['drill_mm']/2)**2)
            for a,b in zip(layers,layers[1:]):network.add(nodes[a],nodes[b],rho*board_mm/(len(layers)-1)/area,'via:'+v['uuid'])
        for layer,parts in objects.items():
            if not parts:continue
            domain=shapely.union_all(parts).difference(holes)
            count+=add_sheet(network,domain,contacts[layer],pitch,rho,copper_mm,f'{code}:{layer}')
    return count
