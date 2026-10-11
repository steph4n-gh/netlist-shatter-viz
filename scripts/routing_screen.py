"""Explicit reference-copper continuity, not impedance or field simulation."""
import math
import shapely
from shapely.geometry import LineString,Point
from grid_route import shape

GUARD_MM=.002  # Native polygon approximation guard, in addition to supplied margin.


def inset(polygon,distance):
    # Enlarge the erosion radius so chord approximations never grant extra
    # space around a concave corner. Straight-edge erosion is conservative.
    return polygon.buffer(-distance/math.cos(math.pi/64),quad_segs=16)


def validate(profile):
    from signal_screen import validate as validate_signal,FIELDS
    if not isinstance(profile,dict) or set(profile)-({'reference_planes'}|FIELDS):raise ValueError('Unknown routing profile field')
    validate_signal(profile)
    rules=profile.get('reference_planes',[])
    if not isinstance(rules,list) or not any(profile.get(k) for k in ('reference_planes','paths','net_limits','unresolved')):raise ValueError('Supply at least one routing rule')
    seen=set()
    for r in rules:
        keys={'net','signal_layer','reference_layer','reference_net','reference_terminal','margin_mm'}
        if not isinstance(r,dict) or set(r)!=keys:raise ValueError('Reference rule requires '+', '.join(sorted(keys)))
        if any(not isinstance(r[k],str) or not r[k] for k in keys-{'margin_mm'}):raise ValueError('Reference rule names must be nonempty strings')
        if r['net'] in seen:raise ValueError('One reference rule per signal net is supported')
        seen.add(r['net'])
        if r['net']==r['reference_net'] or r['signal_layer']==r['reference_layer']:raise ValueError('Signal and reference must use different nets and layers')
        if '.' not in r['reference_terminal']:raise ValueError('Reference terminal must use reference.pin notation')
        m=r['margin_mm']
        if isinstance(m,bool) or not isinstance(m,(int,float)) or not math.isfinite(m) or m<0:raise ValueError('Reference margin must be finite and nonnegative')
    return profile


def corridors(g,profile):
    validate(profile);names={n['name']:int(code) for code,n in g['nets'].items()};layers={name:int(code) for code,name in g['layer_names'].items()}
    holes=shapely.union_all([Point(*h['circle'][:2]).buffer(h['circle'][2]/math.cos(math.pi/64),quad_segs=16) if 'circle' in h else shape([h]) for h in g['holes']]);result={}
    for r in profile.get('reference_planes',[]):
        if {r['net'],r['reference_net']}-set(names):raise ValueError('Unknown signal/reference net')
        if {r['signal_layer'],r['reference_layer']}-set(layers):raise ValueError('Unknown signal/reference copper layer')
        signal,ref=layers[r['signal_layer']],layers[r['reference_layer']]
        if abs(g['layers'].index(signal)-g['layers'].index(ref))!=1:raise ValueError('Reference layer must be adjacent in the native copper stack')
        code,ground=names[r['net']],names[r['reference_net']]
        pads=[p for p in g['pads'] if p['ref']+'.'+p['pin']==r['reference_terminal']]
        if len(pads)!=1 or pads[0]['net']!=ground or str(ref) not in pads[0]['shapes']:raise ValueError('Reference terminal must identify one pad on the reference net and layer')
        zones=[shape(z['filled']) for z in g['zones'] if z['net']==ground and z['layer']==ref and z['filled']]
        if not zones:raise ValueError('Reference rule requires actual filled copper on its reference layer')
        copper=shapely.union_all(zones+[shape(i['shapes'][str(ref)]) for i in g['items'] if i['net']==ground and str(ref) in i['shapes']]).difference(holes)
        if not copper.is_valid:raise ValueError('Invalid reference copper polygons')
        anchor=shape(pads[0]['shapes'][str(ref)]).difference(holes)
        parts=[p for p in shapely.get_parts(copper) if p.geom_type=='Polygon' and p.intersection(anchor).area>1e-10]
        if len(parts)!=1:raise ValueError('Reference anchor does not touch exactly one connected copper region')
        # Isolated same-net islands and regions connected only through another
        # layer cannot stand in for a continuous local reference plane.
        result[code]={'layer':signal,'reference_layer':ref,'reference_net':ground,'clearance_mm':max([g['nets'][str(ground)]['clearance']]+[z.get('clearance_mm',0) for z in g['zones'] if z['net']==ground and z['layer']==ref]),'copper':parts[0],'domain':inset(parts[0],r['margin_mm']+GUARD_MM),'rule':r}
    return result


def screen(g,profile):
    domains=corridors(g,profile);reports=[]
    for code,entry in domains.items():
        r=entry['rule'];tracks=[t for t in g['tracks'] if t['net']==code];violations=[];uncovered=0.
        if not tracks:violations.append({'reason':'No routed signal traces'})
        if any(v['net']==code for v in g['vias']):violations.append({'reason':'Signal vias and reference-plane transitions are not supported'})
        for t in tracks:
            if t['arc'] or t['layer']!=entry['layer']:
                violations.append({'uuid':t['uuid'],'reason':'Signal arc or unapproved layer'});continue
            line=LineString([t['start'],t['end']]);legal=inset(entry['domain'],t['width']/2)
            if not legal.covers(line):
                missing=line.difference(legal);uncovered+=missing.length
                violations.append({'uuid':t['uuid'],'reason':'Signal corridor leaves connected reference copper','unsupported_centerline_mm':missing.length,'bounds_mm':list(missing.bounds)})
        reports.append({'net':r['net'],'pass':not violations,'unsupported_centerline_mm':uncovered,'violations':violations,'trace_count':len(tracks),'reference_layer':r['reference_layer'],'reference_terminal':r['reference_terminal'],'margin_mm':r['margin_mm']})
    from signal_screen import screen as signal_screen
    reports.extend(signal_screen(g,profile))
    return {'pass':all(r['pass'] for r in reports),'rules':reports,'assumptions':profile,'polygon_guard_mm':GUARD_MM,'scope':'Single-layer signal corridor over adjacent connected filled reference copper. Optional terminal paths use equipotential pads/junctions and through-via barrel lengths. No impedance, timing, return-current field, transition or EMI validation.'}


def nonregression(before,after):
    if before['assumptions']!=after['assumptions']:return False
    if len(before['rules'])!=len(after['rules']):return False
    # A failed original may be repaired one net at a time. Every changed rule
    # must pass outright; remaining failures must be identical, never traded.
    return all(a['net']==b['net'] and (b['pass'] or a==b) for a,b in zip(before['rules'],after['rules']))
