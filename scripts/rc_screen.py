"""Optional passive RC current-hold acceptance for native PCB jobs.

One regulator output is supported. This does not model its control loop,
inductance, switching, thermal behavior, or unspecified firmware loads.
"""
import math
from pcb_dc import build_network
from pcb_rc import reduce_ports,reservoir


def validate(profile):
    from dc_screen import number
    r=profile['rc_hold']
    keys={'source_pair','capacitors','events','voltage_limits_v','allowance_v','time_step_s','voltage_tolerance_v'}
    if not isinstance(r,dict) or set(r)!=keys:raise ValueError('rc_hold requires '+', '.join(sorted(keys)))
    pair=r['source_pair']
    if not isinstance(pair,list) or len(pair)!=2 or any(not isinstance(p,str) for p in pair) or pair[0]==pair[1]:raise ValueError('RC source_pair requires distinct supply and ground terminals')
    if set(pair)!=set(profile['sources_v']) or profile['sources_v'][pair[1]]!=0:raise ValueError('RC hold requires exactly one supply and one zero-volt return source')
    if profile['sources_v'][pair[0]]<=0:raise ValueError('RC supply source must be positive')
    if 'returns' not in profile:raise ValueError('RC hold requires explicit returns for every load')
    caps=r['capacitors']
    if not isinstance(caps,list) or not 1<=len(caps)<=64:raise ValueError('RC hold supports 1 to 64 capacitors')
    names=[]
    for c in caps:
        if not isinstance(c,dict) or set(c)!={'reference','a','b','farad','esr_ohm'}:raise ValueError('Each RC capacitor requires reference, a, b, farad, esr_ohm')
        if any(not isinstance(c[k],str) or not c[k] for k in ('reference','a','b')) or '.' not in c['a'] or '.' not in c['b'] or c['a']==c['b']:raise ValueError('Invalid RC capacitor terminals')
        names.append(c['reference']);number(c['farad'],'Effective capacitance',0,True);number(c['esr_ohm'],'Capacitor ESR',0,True)
    if len(names)!=len(set(names)):raise ValueError('Duplicate RC capacitor reference')
    for key in ('time_step_s','voltage_tolerance_v'):number(r[key],key,0,True)
    number(r['allowance_v'],'RC voltage allowance',0)
    limits=r['voltage_limits_v']
    if not isinstance(limits,dict) or set(limits)!=set(profile['loads_a']):raise ValueError('RC limits must cover exactly every DC load terminal')
    for bounds in limits.values():
        if not isinstance(bounds,list) or len(bounds)!=2:raise ValueError('RC voltage limits require [low, high]')
        for v in bounds:number(v,'RC voltage bound',0)
        if bounds[0]>=bounds[1]:raise ValueError('Invalid RC voltage interval')
    events=r['events']
    if not isinstance(events,list) or not 1<=len(events)<=16:raise ValueError('RC hold supports 1 to 16 explicit events')
    event_names=[]
    for e in events:
        if not isinstance(e,dict) or set(e)!={'name','before_a','after_a','source_v','duration_s'}:raise ValueError('RC event requires name, before_a, after_a, source_v, duration_s')
        if not isinstance(e['name'],str) or not e['name']:raise ValueError('RC event name must be nonempty')
        event_names.append(e['name'])
        for key in ('before_a','after_a'):
            if not isinstance(e[key],dict) or set(e[key])!=set(profile['loads_a']):raise ValueError('RC event currents must cover exactly every DC load')
            for v in e[key].values():number(v,'RC load current',0)
        for key in ('duration_s','source_v'):number(e[key],key,0,True)
        if e['duration_s']/r['time_step_s']>50000:raise ValueError('RC event exceeds 50000 coarse time steps')
    if len(event_names)!=len(set(event_names)):raise ValueError('Duplicate RC event name')
    return r


def evaluate(geometry,profile):
    r=validate(profile);source,ground=r['source_pair'];returns=profile['returns'];caps=r['capacitors']
    ports=list(dict.fromkeys([source,*profile['loads_a'],*returns.values(),*[p for c in caps for p in (c['a'],c['b'])]]));ports=[p for p in ports if p!=ground]
    mesh=profile.get('plane_mesh');pitches=[mesh['pitch_mm'],mesh['pitch_mm']/2] if mesh else [None]
    resolutions=[]
    for pitch in pitches:
        network,term=build_network(geometry,profile['nets'],**profile['copper'],plane_mesh_mm=pitch)
        required=set(ports)|{ground}|{p for c in profile.get('components',[]) for p in (c['a'],c['b'])}|{p for group in profile.get('shorted_pins',[]) for p in group}
        if required-set(term):raise ValueError('Unknown RC terminals on selected nets: '+', '.join(sorted(required-set(term))))
        for c in profile.get('components',[]):network.add(term[c['a']],term[c['b']],c['ohm'],c['reference'])
        for group in profile.get('shorted_pins',[]):
            for p in group[1:]:network.join(term[group[0]],term[p])
        model=reduce_ports(network,term,ports,ground);del network,term
        events={}
        for e in r['events']:
            loads=lambda key:[{'a':p,'b':returns[p],'amp':v} for p,v in e[key].items()]
            args=(model,caps,[source,ground],loads('before_a'),loads('after_a'),e['source_v'],e['duration_s'])
            coarse=reservoir(*args,r['time_step_s']);fine=reservoir(*args,coarse['step_s']/2)
            if len(coarse['history'])!=len(fine['history'][::2]):raise ValueError('RC time grids do not align')
            delta=0.
            for a,b in zip(coarse['history'],fine['history'][::2]):
                if not math.isclose(a['time_s'],b['time_s'],rel_tol=1e-12,abs_tol=1e-18):raise ValueError('RC time grids do not align')
                delta=max(delta,max(abs(a[p]-b[p]) for p in profile['loads_a']))
            envelopes={}
            for p in profile['loads_a']:
                values=[fine['initial_v'][p],coarse['initial_v'][p],*[row[p] for row in fine['history']],*[row[p] for row in coarse['history']]]
                low=min(values)-r['allowance_v'];high=max(values)+r['allowance_v'];bounds=r['voltage_limits_v'][p]
                envelopes[p]={'minimum_v':low,'maximum_v':high,'limits_v':bounds,'pass':bounds[0]<=low and high<=bounds[1]}
            events[e['name']]={'loads':envelopes,'pass_limits':all(v['pass'] for v in envelopes.values()),'time_check':{'difference_v':delta,'tolerance_v':r['voltage_tolerance_v'],'pass':delta<=r['voltage_tolerance_v']},'initial_v':fine['initial_v'],'source_hold_a':fine['source_hold_a'],'history':fine['history'],'step_s':fine['step_s'],'max_kcl_residual_a':fine['max_kcl_residual_a']}
        resolutions.append({'pitch_mm':pitch,'full_nodes':model['full_nodes'],'port_nodes':model['port_nodes'],'events':events})
    mesh_delta=0.
    if mesh:
        for name,e in resolutions[0]['events'].items():
            other=resolutions[1]['events'][name]
            if len(e['history'])!=len(other['history']):raise ValueError('RC mesh comparison times differ')
            mesh_delta=max(mesh_delta,max(abs(e['initial_v'][p]-other['initial_v'][p]) for p in profile['loads_a']),max(abs(a[p]-b[p]) for a,b in zip(e['history'],other['history']) for p in profile['loads_a']))
    converged=all(e['time_check']['pass'] for resolution in resolutions for e in resolution['events'].values()) and (not mesh or mesh_delta<=mesh['voltage_tolerance_v'])
    return {'pass_limits':converged and all(e['pass_limits'] for resolution in resolutions for e in resolution['events'].values()),'converged':converged,'resolutions':resolutions,'mesh_difference_v':mesh_delta if mesh else None,'scope':'Passive RC current-hold scenarios only. Effective C, ESR, load events and source voltage are supplied assumptions. No regulator-loop, inductive, switching, thermal or signal-integrity validation.'}


def nonregression(before,after):
    if not after['pass_limits'] or not before['converged'] or not after['converged']:return False
    if len(before['resolutions'])!=len(after['resolutions']):return False
    for a,b in zip(before['resolutions'],after['resolutions']):
        if a['pitch_mm']!=b['pitch_mm'] or a['events'].keys()!=b['events'].keys():return False
        for name,e in a['events'].items():
            candidate=b['events'][name]
            if e['loads'].keys()!=candidate['loads'].keys():return False
            for p,v in e['loads'].items():
                w=candidate['loads'][p]
                if w['minimum_v']<v['minimum_v']-1e-8 or w['maximum_v']>v['maximum_v']+1e-8:return False
    return True


def summary(report):
    return {k:v for k,v in report.items() if k!='resolutions'}|{'events':{name:{'loads':e['loads'],'time_check':e['time_check']} for name,e in report['resolutions'][-1]['events'].items()}}
