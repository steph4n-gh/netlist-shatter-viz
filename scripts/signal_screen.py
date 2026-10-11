"""Explicit terminal-path and net limits; no inferred timing or impedance signoff."""
from collections import defaultdict
import heapq
import math
from pcb_dc import build_network

FIELDS={'paths','matched_groups','net_limits','unresolved','sources'}

def number(value,label):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
        raise ValueError(label+' must be finite and nonnegative')

def validate(profile):
    if not isinstance(profile,dict):raise ValueError('Routing profile must be an object')
    for field in FIELDS:
        if field in profile and not isinstance(profile[field],list):raise ValueError(field+' must be a list')
    ids=set()
    for p in profile.get('paths',[]):
        if not isinstance(p,dict) or set(p)-{'id','net','from','to','min_length_mm','max_length_mm'} or not {'id','net','from','to','max_length_mm'}<=set(p):raise ValueError('Invalid terminal-path rule')
        if any(not isinstance(p[k],str) or not p[k] for k in ('id','net','from','to')) or p['id'] in ids:raise ValueError('Path names must be nonempty and unique')
        if p['from']==p['to'] or any('.' not in p[k] for k in ('from','to')):raise ValueError('Use distinct reference.pin terminals')
        ids.add(p['id']);number(p['max_length_mm'],'Maximum path length');number(p.get('min_length_mm',0),'Minimum path length')
        if p.get('min_length_mm',0)>p['max_length_mm']:raise ValueError('Reversed path length interval')
    group_ids=set()
    for m in profile.get('matched_groups',[]):
        if not isinstance(m,dict) or set(m)!={'id','paths','max_skew_mm'} or not isinstance(m['id'],str) or not m['id'] or m['id'] in group_ids:raise ValueError('Invalid or duplicate matched-path group')
        group_ids.add(m['id'])
        if not isinstance(m['paths'],list) or any(not isinstance(p,str) for p in m['paths']) or len(m['paths'])<2 or len(set(m['paths']))!=len(m['paths']) or set(m['paths'])-ids:raise ValueError('Matched group needs distinct existing paths')
        number(m['max_skew_mm'],'Path skew')
    nets=set()
    for n in profile.get('net_limits',[]):
        if not isinstance(n,dict) or set(n)-{'net','min_width_mm','max_vias','allowed_layers'} or 'net' not in n or len(n)<2:raise ValueError('Invalid net limit')
        if not isinstance(n['net'],str) or not n['net'] or n['net'] in nets:raise ValueError('Net limit names must be unique');
        nets.add(n['net'])
        if 'min_width_mm' in n:number(n['min_width_mm'],'Minimum width')
        if 'max_vias' in n and (type(n['max_vias'])!=int or n['max_vias']<0):raise ValueError('Via limit must be a nonnegative integer')
        if 'allowed_layers' in n and (not isinstance(n['allowed_layers'],list) or not n['allowed_layers'] or any(not isinstance(l,str) or not l for l in n['allowed_layers'])):raise ValueError('Supply allowed copper layer names')
    for u in profile.get('unresolved',[]):
        if not isinstance(u,dict) or set(u)!={'id','reason','source'} or any(not isinstance(v,str) or not v for v in u.values()):raise ValueError('Unresolved requirements need id, reason and source')
    return profile


def path_lengths(g,paths):
    """Reuse native-contact graph; convert each resistive edge back to physical length.

    This measures routed centerline outside equipotential pad/junction regions,
    plus through-via barrels. It is not flight time or a package-delay model.
    """
    results={};names={n['name']:int(code) for code,n in g['nets'].items()}
    for name in sorted({p['net'] for p in paths}):
        selected=[p for p in paths if p['net']==name]
        try:
            if name not in names:raise ValueError('Unknown net '+name)
            thickness=g.get('board_thickness_mm')
            if any(v['net']==names[name] for v in g['vias']) and (thickness is None or thickness<=0):raise ValueError('Native board thickness required for via lengths')
            network,terminals=build_network(g,[name],temperature_c=20,copper_mm=1,plating_mm=.025,board_mm=thickness or 1.2,contact_lengths=True)
            widths={t['uuid']:t['width'] for t in g['tracks']};vias={v['uuid']:v for v in g['vias']};adj=defaultdict(list)
            for a,b,resistance,label in network.edges:
                kind,uid=label.split(':',1)
                if kind=='track':length=resistance*widths[uid]/1.724e-5
                elif kind=='contact':length=resistance/1.724e-5
                elif kind=='via':
                    v=vias[uid];area=math.pi*((v['drill_mm']/2+.025)**2-(v['drill_mm']/2)**2);length=resistance*area/1.724e-5
                else:raise ValueError('Unsupported signal geometry')
                a,b=network.root(a),network.root(b)
                if a!=b:adj[a].append((b,length));adj[b].append((a,length))
            for p in selected:
                if p['from'] not in terminals or p['to'] not in terminals:raise ValueError('Unknown or ambiguous terminal on '+name)
                # Duplicate references/pins must not collapse to a fictitious terminal.
                for key in (p['from'],p['to']):
                    if sum(pad['ref']+'.'+pad['pin']==key and pad['net']==names[name] for pad in g['pads'])!=1:raise ValueError('Ambiguous terminal '+key)
                start,end=network.root(terminals[p['from']]),network.root(terminals[p['to']]);dist={start:0.};queue=[(0.,0,start)];serial=0
                while queue:
                    length,_,node=heapq.heappop(queue)
                    if length>dist[node]:continue
                    if node==end:break
                    for neighbor,cost in adj[node]:
                        value=length+cost
                        if value<dist.get(neighbor,math.inf):
                            dist[neighbor]=value;serial+=1;heapq.heappush(queue,(value,serial,neighbor))
                results[p['id']]={'length_mm':dist.get(end),'error':None if end in dist else 'Disconnected terminal path'}
        except (ValueError,KeyError) as error:
            for p in selected:results[p['id']]={'length_mm':None,'error':str(error)}
    return results


def screen(g,profile):
    validate(profile);reports=[];lengths=path_lengths(g,profile.get('paths',[]));names={n['name']:int(c) for c,n in g['nets'].items()}
    for p in profile.get('paths',[]):
        m=lengths[p['id']];value=m['length_mm'];passed=value is not None and p.get('min_length_mm',0)-1e-6<=value<=p['max_length_mm']+1e-6
        reports.append(dict(m,id=p['id'],net=p['net'],kind='terminal_path',pass_=passed,limits_mm=[p.get('min_length_mm',0),p['max_length_mm']]))
    for m in profile.get('matched_groups',[]):
        values=[lengths[p]['length_mm'] for p in m['paths']];skew=None if any(v is None for v in values) else max(values)-min(values)
        reports.append({'id':m['id'],'net':m['id'],'kind':'path_matching','skew_mm':skew,'max_skew_mm':m['max_skew_mm'],'pass_':skew is not None and skew<=m['max_skew_mm']+1e-6})
    for n in profile.get('net_limits',[]):
        code=names.get(n['net']);tracks=[t for t in g['tracks'] if t['net']==code];vias=[v for v in g['vias'] if v['net']==code];issues=[]
        if code is None or not tracks:issues.append('Unknown net or no routed tracks')
        minimum=min((t['width'] for t in tracks),default=None);layers=sorted({g['layer_names'][str(t['layer'])] for t in tracks})
        if 'min_width_mm' in n and (minimum is None or minimum<n['min_width_mm']-1e-6):issues.append('Trace narrower than declared limit')
        if 'max_vias' in n and len(vias)>n['max_vias']:issues.append('Too many vias')
        if 'allowed_layers' in n and set(layers)-set(n['allowed_layers']):issues.append('Unapproved signal layer')
        reports.append({'net':n['net'],'kind':'net_limits','pass_':not issues,'violations':issues,'min_width_mm':minimum,'vias':len(vias),'layers':layers})
    for u in profile.get('unresolved',[]):reports.append(dict(u,net=u['id'],kind='unresolved_requirement',pass_=False))
    for r in reports:r['pass']=r.pop('pass_')
    return reports
