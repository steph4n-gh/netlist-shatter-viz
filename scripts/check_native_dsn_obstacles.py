"""Verify the DSN contains the native temporary-obstacle polygon vertices."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from dsn_constraints import parse,decode


def matching_vertices(expected,actual,tolerance=1.1):
    # Exported DSN closes outlines explicitly and rounds to six significant
    # digits (up to one micrometre on this board). Match every vertex once.
    if actual and actual[0]==actual[-1]:actual=actual[:-1]
    if expected and expected[0]==expected[-1]:expected=expected[:-1]
    if len(expected)!=len(actual):return False
    remaining=list(actual)
    for x,y in expected:
        nearest=min(range(len(remaining)),key=lambda i:math.hypot(x-remaining[i][0],y-remaining[i][1]))
        if math.hypot(x-remaining[nearest][0],y-remaining[nearest][1])>tolerance:return False
        remaining.pop(nearest)
    return True


def check(directory):
    evidence=json.loads((directory/'dsn-native-obstacles.json').read_text())
    tree=parse((directory/'input.dsn').read_text())
    scope=next(v for v in tree if isinstance(v,list) and v and v[0]=='structure')
    polygons=[n for v in scope if isinstance(v,list) and v and v[0]=='keepout'
              for n in v if isinstance(n,list) and n and n[0]=='polygon']
    checks=[]
    for obstacle in evidence['obstacles']:
        for layer in obstacle.get('layers',[obstacle.get('layer')]):
            name=evidence['layer_names'][str(layer)]
            for shape in obstacle['shapes']:
                expected=[(x*1000,-y*1000) for x,y in shape['shell']]
                found=any(decode(poly[1])==name and matching_vertices(expected,
                         [(float(poly[i]),float(poly[i+1])) for i in range(3,len(poly),2)]) for poly in polygons)
                checks.append({'kind':obstacle['kind'],'reference':obstacle.get('reference'),
                               'text':obstacle.get('text'),'layer':name,'vertices':len(expected),'pass':found})
    base_hash=hashlib.sha256((directory/'routing-base.kicad_pcb').read_bytes()).hexdigest()
    return {'pass':base_hash==evidence['routing_base_sha256'] and all(c['pass'] for c in checks),
            'routing_base_hash_matches':base_hash==evidence['routing_base_sha256'],
            'dsn_keepout_polygons':len(polygons),'checks':checks,
            'scope':'Native outer-contour vertices represented in temporary DSN keepouts; not route completion or electrical qualification.'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);args=p.parse_args()
    result=check(args.directory);(args.directory/'native-obstacle-checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'pass':result['pass'],'checks':len(result['checks']),'dsn_keepout_polygons':result['dsn_keepout_polygons']}))
    raise SystemExit(0 if result['pass'] else 1)
