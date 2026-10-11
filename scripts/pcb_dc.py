#!/usr/bin/env python3
"""DC screening with a trace-centerline and via-barrel resistor network.
Pads/junctions are equipotential; optional filled-sheet and return models.
"""
import json,math,argparse
from pathlib import Path
from collections import defaultdict,deque
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve
from shapely.geometry import Point,LineString
from shapely.ops import nearest_points
from plan_m64_power import shapes

class Network:
    def __init__(self):self.parent={};self.edges=[]
    def root(self,x):
        self.parent.setdefault(x,x)
        node=x
        while self.parent[node]!=node:node=self.parent[node]
        while self.parent[x]!=x:
            parent=self.parent[x];self.parent[x]=node;x=parent
        return node
    def join(self,a,b):self.parent[self.root(a)]=self.root(b)
    def add(self,a,b,r,label):
        if r<0:raise ValueError('Negative resistance')
        if r==0:self.join(a,b)
        else:self.root(a);self.root(b);self.edges.append((a,b,r,label))
    def solve(self,sources,loads):
        source={}
        for n,v in sources.items():
            n=self.root(n)
            if n in source and abs(source[n]-v)>1e-9:raise ValueError('Shorted unlike ideal sources')
            source[n]=v
        load=defaultdict(float)
        for n,i in loads.items():load[self.root(n)]+=i
        edges=[(self.root(a),self.root(b),r,label) for a,b,r,label in self.edges if self.root(a)!=self.root(b)]
        adjacent=defaultdict(set)
        for a,b,_,_ in edges:adjacent[a].add(b);adjacent[b].add(a)
        reached=set(source);todo=deque(source)
        while todo:
            for n in adjacent[todo.popleft()]:
                if n not in reached:reached.add(n);todo.append(n)
        if set(load)-reached:raise ValueError('Disconnected loaded terminals: '+str(set(load)-reached))
        unknown=list(reached-set(source));index={n:i for i,n in enumerate(unknown)}
        rows=[];cols=[];data=[];rhs=np.array([-load[n] for n in unknown],dtype=float)
        def add(i,j,value):rows.append(i);cols.append(j);data.append(value)
        for a,b,r,_ in edges:
            if a not in reached:continue
            for n,m in ((a,b),(b,a)):
                if n not in index:continue
                i=index[n];add(i,i,1/r)
                if m in index:add(i,index[m],-1/r)
                else:rhs[i]+=source[m]/r
        matrix=coo_matrix((data,(rows,cols)),shape=(len(unknown),len(unknown))).tocsr()
        del rows,cols,data
        values=spsolve(matrix,rhs) if unknown else np.array([])
        if not np.all(np.isfinite(values)):raise ValueError('DC network solution is not finite')
        voltage={**source,**dict(zip(unknown,map(float,values)))}
        residual=float(np.max(np.abs(matrix@values-rhs))) if unknown else 0
        currents=[];source_current=defaultdict(float);sheet_summary={};loss_w=0;edge_count=0
        for a,b,r,label in edges:
            if a not in reached:continue
            current=(voltage[a]-voltage[b])/r
            loss=current*current*r;loss_w+=loss;edge_count+=1
            if label.startswith('sheet:'):
                item=sheet_summary.setdefault(label,{'edges':0,'loss_w':0,'max_cell_drop_v':0})
                item['edges']+=1;item['loss_w']+=loss;item['max_cell_drop_v']=max(item['max_cell_drop_v'],abs(current*r))
            else:currents.append({'label':label,'current_a':current,'resistance_ohm':r,'loss_w':loss,'drop_v':abs(current*r)})
            if a in source:source_current[a]+=current
            if b in source:source_current[b]-=current
        # A load directly on a source node still draws supply current.
        for n in source:source_current[n]+=load[n]
        mismatch=abs(sum(source_current.values())-sum(load.values()))
        if residual>1e-7 or mismatch>1e-7:raise ValueError('KCL residual exceeds tolerance')
        return voltage,{'nodes':len(reached),'edges':edge_count,'max_kcl_residual_a':residual,'current_balance_error_a':mismatch,'source_current_a':sum(source_current.values()),'loss_w':loss_w,'sheets':sheet_summary,'elements':sorted(currents,key=lambda x:-x['loss_w'])}


def build_network(g,net_names,temperature_c=85,copper_mm=.035,plating_mm=.025,board_mm=1.2,plane_mesh_mm=None,contact_lengths=False):
    """Uses uniform copper thickness and equally spaced layer Z as stated assumptions."""
    rho=1.724e-5*(1+.00393*(temperature_c-20)) # ohm mm
    netcodes={int(n) for n,d in g['nets'].items() if d['name'] in net_names}
    unknown=set(net_names)-{d['name'] for d in g['nets'].values()}
    if unknown:raise ValueError('Unknown DC nets: '+', '.join(sorted(unknown)))
    sheet_codes={z['net'] for z in g.get('zones',[]) if z['net'] in netcodes
                 and not (contact_lengths and z.get('teardrop') is True)}
    if sheet_codes and plane_mesh_mm is None:raise ValueError('Supply-zone resistance requires an explicit sheet/mesh configuration')
    trace_codes=netcodes-sheet_codes
    tracks=[t for t in g['tracks'] if t['net'] in trace_codes]
    if any(t['arc'] for t in tracks):raise ValueError('Arcs not supported by DC screen')
    network=Network();cuts=[{0.,LineString([t['start'],t['end']]).length} for t in tracks]
    lines=[LineString([t['start'],t['end']]) for t in tracks]
    links=[]
    def node(i,d):
        p=lines[i].interpolate(d);return ('xy',tracks[i]['net'],tracks[i]['layer'],round(p.x,6),round(p.y,6))
    def cut(i,p):d=lines[i].project(p);cuts[i].add(d);return node(i,d)
    # Branch endpoints and crossing/overlapping same-net copper need junctions.
    for i,t in enumerate(tracks):
        for j in range(i):
            u=tracks[j]
            if t['net']!=u['net'] or t['layer']!=u['layer']:continue
            if lines[i].distance(lines[j])>(t['width']+u['width'])/2+1e-6:continue
            cross=lines[i].intersection(lines[j])
            points=[]
            if not cross.is_empty:
                for part in (list(cross.geoms) if hasattr(cross,'geoms') else [cross]):
                    points.extend([Point(c) for c in part.coords])
            if points:
                for p in points:links.append((cut(i,p),cut(j,p)))
            else:
                a,b=nearest_points(lines[i],lines[j]);na,nb=cut(i,a),cut(j,b)
                if contact_lengths:
                    # Signal distance must not collapse a chain of short,
                    # overlapping segments into a zero-length copper junction.
                    network.add(na,nb,rho*a.distance(b),'contact:'+str(i)+'-'+str(j))
                else:links.append((na,nb))
    terminals={}
    for p in g['pads']:
        if p['net'] not in trace_codes or not p['pin']:continue
        key=p['ref']+'.'+p['pin'];terminal=('pad',key);terminals[key]=terminal;network.root(terminal)
        for i,t in enumerate(tracks):
            if t['net']!=p['net'] or str(t['layer']) not in p['shapes']:continue
            poly=shapes(p['shapes'][str(t['layer'])]);hit=lines[i].intersection(poly.buffer(t['width']/2))
            if hit.is_empty:continue
            for part in (list(hit.geoms) if hasattr(hit,'geoms') else [hit]):
                for c in part.coords:links.append((terminal,cut(i,Point(c))))
    layers=g['layers']
    for v in g['vias']:
        if v['net'] not in trace_codes:continue
        if not v.get('through',True):raise ValueError('Blind/buried/microvia resistance is not supported by this DC screen')
        nodes={l:('via',v['uuid'],l) for l in layers};pt=Point(v['at'])
        for i,t in enumerate(tracks):
            if t['net']==v['net'] and lines[i].distance(pt)<=(v['diameter_mm']+t['width'])/2+1e-6:
                links.append((nodes[t['layer']],cut(i,pt)))
        for p in g['pads']:
            if p['net']!=v['net'] or not p['pin']:continue
            for layer,polys in p['shapes'].items():
                if shapes(polys).distance(pt)<=v['diameter_mm']/2:links.append((nodes[int(layer)],terminals[p['ref']+'.'+p['pin']]))
        area=math.pi*((v['drill_mm']/2+plating_mm)**2-(v['drill_mm']/2)**2)
        for a,b in zip(layers,layers[1:]):network.add(nodes[a],nodes[b],rho*board_mm/(len(layers)-1)/area,'via:'+v['uuid'])
    for a,b in links:network.join(a,b)
    for i,t in enumerate(tracks):
        ds=sorted(cuts[i])
        for a,b in zip(ds,ds[1:]):
            if b-a>1e-7:network.add(node(i,a),node(i,b),rho*(b-a)/(t['width']*copper_mm),'track:'+t['uuid'])
    if sheet_codes:
        from pcb_sheet import add_board_sheets
        network.sheet_cells=add_board_sheets(network,g,sheet_codes,terminals,plane_mesh_mm,rho,copper_mm,plating_mm,board_mm)
    return network,terminals


def run_once(g,profile,pitch=None):
    n,term=build_network(g,profile['nets'],**profile.get('copper',{}),plane_mesh_mm=pitch)
    returns=profile.get('returns',{})
    requested=set(profile['sources_v'])|set(profile['loads_a'])|set(returns.values())
    requested.update(pin for group in profile.get('shorted_pins',[]) for pin in group)
    requested.update(link[end] for link in profile.get('components',[]) for end in ('a','b'))
    if requested-set(term):raise ValueError('Unknown DC terminals on selected nets: '+', '.join(sorted(requested-set(term))))
    for link in profile.get('components',[]):n.add(term[link['a']],term[link['b']],link['ohm'],link['reference'])
    for group in profile.get('shorted_pins',[]):
        for pin in group[1:]:n.join(term[group[0]],term[pin])
    sources={term[k]:v for k,v in profile['sources_v'].items()};loads={term[k]:v for k,v in profile['loads_a'].items()}
    for pin,return_pin in returns.items():loads[term[return_pin]]=loads.get(term[return_pin],0)-profile['loads_a'][pin]
    voltage,result=n.solve(sources,loads)
    result['terminals_v']={k:voltage[n.root(node)] for k,node in term.items() if n.root(node) in voltage}
    result['loads']={k:{'current_a':i,'voltage_v':result['terminals_v'][k]} for k,i in profile['loads_a'].items()}
    for pin,return_pin in returns.items():
        item=result['loads'][pin];item['supply_voltage_v']=item['voltage_v'];item['return_voltage_v']=result['terminals_v'][return_pin];item['return_terminal']=return_pin;item['voltage_v']-=item['return_voltage_v']
    result['return_path_modelled']=bool(returns);result['sheet_cells']=getattr(n,'sheet_cells',0)
    result['assumptions']=profile;result['screen_only']=True
    result['limitations']=['Ideal return ground; no ground-plane voltage loss.','Equipotential pads and copper junctions; through-hole pad barrels are ideal, unlike modeled via barrels. Trace centerlines omit local spreading resistance.','No transient, regulator-loop, thermal or signal-integrity simulation.','Uniform copper/plating and uniformly spaced layers are assumed, not fabricated measurements.','Current loads are explicit test scenarios, not recovered firmware power estimates.']
    if returns:result['limitations'][0]='Explicit return terminals included; each device load is measured supply-to-return.'
    if result['sheet_cells']:result['limitations'].append('Clipped Cartesian sheet cells approximate boundary/contact spreading resistance; refinement agreement is not an error bound.')
    return result

def run(g,profile):
    mesh=profile.get('plane_mesh')
    if mesh is None:return run_once(g,profile)
    coarse=run_once(g,profile,mesh['pitch_mm'])
    fine=run_once(g,profile,mesh['pitch_mm']/2)
    delta=max(abs(coarse['loads'][p]['voltage_v']-v['voltage_v']) for p,v in fine['loads'].items())
    fine['mesh_check']={'coarse_pitch_mm':mesh['pitch_mm'],'fine_pitch_mm':mesh['pitch_mm']/2,'max_loaded_voltage_difference_v':delta,'tolerance_v':mesh['voltage_tolerance_v'],'pass':delta<=mesh['voltage_tolerance_v'],'coarse_loads':coarse['loads'],'coarse_loss_w':coarse['loss_w']}
    return fine

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('geometry',type=Path);p.add_argument('profile',type=Path);p.add_argument('output',type=Path);a=p.parse_args();r=run(json.loads(a.geometry.read_text()),json.loads(a.profile.read_text()));a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k in ('nodes','edges','loads','loss_w','max_kcl_residual_a')},indent=2))
