"""Propose clearance-checked length additions for explicitly matched terminal paths.

This matches measured path lengths only, not differential impedance or coupling.
Native DRC and the complete supplied profile must independently approve output.
"""
import math
import shapely
from shapely.geometry import LineString
from grid_route import shape
from signal_screen import screen


def tune(g,profile):
    before=screen(g,profile)
    failed=[r for r in before if r['kind']=='path_matching' and not r['pass'] and r['skew_mm'] is not None]
    paths={p['id']:p for p in profile.get('paths',[])}
    measured={r['id']:r['length_mm'] for r in before if r['kind']=='terminal_path'}
    names={n['name']:int(c) for c,n in g['nets'].items()}
    for failure in failed:
        group=next(m for m in profile['matched_groups'] if m['id']==failure['id'])
        shortest=min(group['paths'],key=lambda p:measured[p]);name=paths[shortest]['net'];net=names[name]
        addition=failure['skew_mm']-group['max_skew_mm']*.5
        relevant=[m for m in profile['matched_groups'] if any(paths[p]['net']==name for p in m['paths'])]
        ids={p for m in relevant for p in m['paths']}|{p['id'] for p in profile['paths'] if p['net']==name}
        sub={'paths':[p for p in profile['paths'] if p['id'] in ids], 'matched_groups':relevant,
             'net_limits':[n for n in profile.get('net_limits',[]) if n['net']==name]}
        prior=screen(g,sub);domains={}
        tracks=sorted((t for t in g['tracks'] if t['net']==net and not t['locked'] and not t['arc']),
                      key=lambda t:-math.dist(t['start'],t['end']))
        for track in tracks:
            a,b=track['start'],track['end'];length=math.dist(a,b);width=track['width'];layer=track['layer']
            if length<max(1,8*width):continue
            key=layer,width
            if key not in domains:
                radius=width/2+.003;clearance=g['nets'][str(net)]['clearance']
                obstacles=[shape(i['shapes'][str(layer)]).buffer(max(clearance,i['clearance'])+radius)
                           for i in g['items'] if i['net']!=net and str(layer) in i['shapes']]
                obstacles += [shape(k['shapes']).buffer(radius) for k in g['keepouts'] if k['tracks'] and layer in k['layers']]
                obstacles += [shape(h['shapes']).buffer(radius+g['hole_clearance']) for h in g.get('drill_obstacles',[]) if h['net']!=net]
                domains[key]=shape(g['outline']).buffer(-g['edge_clearance']-radius).difference(shapely.union_all(obstacles))
            u=[(b[i]-a[i])/length for i in range(2)];normal=[-u[1],u[0]]
            offset=min(length/4,max(2*width,.25))
            windows=[(offset,length-offset)]
            for span in (1.,2.,4.,8.):
                if span>=length-2*offset:continue
                for fraction in (.5,.25,.75):
                    center=length*fraction
                    if center-span/2>=offset and center+span/2<=length-offset:
                        windows.append((center-span/2,center+span/2))
            for begin,finish in windows:
                p=[a[i]+u[i]*begin for i in range(2)];q=[a[i]+u[i]*finish for i in range(2)]
                for scale in (1,.9,1.1,.8,1.2,.7,1.3):
                    height=(addition/2+width)*scale
                    for sign in (1,-1):
                        points=[a,p,[p[i]+normal[i]*height*sign for i in range(2)],
                                [q[i]+normal[i]*height*sign for i in range(2)],q,b]
                        if not domains[key].covers(LineString(points)):continue
                        new=[dict(track,uuid='matching-'+str(i),start=[round(v,6) for v in x],end=[round(v,6) for v in y]) for i,(x,y) in enumerate(zip(points,points[1:]))]
                        candidate=dict(g,tracks=[t for t in g['tracks'] if t['uuid']!=track['uuid']]+new)
                        after=screen(candidate,sub)
                        # Never exchange a passing rule for a failing one.
                        if any(a['pass'] and not b['pass'] for a,b in zip(prior,after)):continue
                        if sum(not r['pass'] for r in after)>=sum(not r['pass'] for r in prior):continue
                        return {'remove':[track['uuid']],'tracks':new,'vias':[], 'matched_group':group['id'],
                                'predicted_checks':after,'candidate_only':True}
    return {'remove':[],'tracks':[],'vias':[],'candidate_only':True}
