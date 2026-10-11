from pathlib import Path
import json,argparse
from collections import Counter
NM=lambda x:int(round(x*1_000_000))
def pt(p):return tuple(map(NM,p))
def poly(p):
 def ring(r):
  v=[pt(x) for x in r];v=v[:-1] if v and v[0]==v[-1] else v
  return min(tuple(v[i:]+v[:i]) for v in (v,list(reversed(v))) for i in range(len(v))) if v else ()
 return (ring(p['shell']),tuple(sorted(ring(h) for h in p['holes'])))
def inventory(g):
 name=lambda n:g['nets'][str(n)]['name'] if n else ''
 layer=lambda n:g['layer_names'][str(n)]
 tracks=Counter((name(t['net']),layer(t['layer']),NM(t['width']),tuple(sorted([pt(t['start']),pt(t['end'])])),t['arc'],t.get('locked',False)) for t in g['tracks'])
 vias=Counter((name(v['net']),pt(v['at']),NM(v['diameter_mm']),NM(v['drill_mm']),v['through'],v.get('locked',False)) for v in g['vias'])
 pads=Counter((p['ref'],p['pin'],name(p['net']),pt(p['at']),pt(p['size']),pt(p['drill']),p['dnp'],tuple(sorted((layer(int(l)),tuple(sorted(poly(z) for z in shapes))) for l,shapes in p['shapes'].items()))) for p in g['pads'])
 outline=tuple(sorted(poly(z) for z in g['outline']))
 return {'tracks':tracks,'vias':vias,'pads':pads,'outline':outline,'board_thickness':NM(g['board_thickness_mm']),'layer_stack':tuple(layer(l) for l in g['layers']),'net_names':sorted(v['name'] for v in g['nets'].values()),'hole_to_hole':NM(g['hole_to_hole']),'hole_clearance':NM(g['hole_clearance']),'edge_clearance':NM(g['edge_clearance']),'zones':g['zones'],'keepouts':g['keepouts']}
def main():
 p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('candidate');p.add_argument('output');a=p.parse_args();A=inventory(json.loads(Path(a.source).read_text()));B=inventory(json.loads(Path(a.candidate).read_text()));checks={k:A[k]==B[k] for k in A};r={'pass':all(checks.values()),'nanometre_inventory_checks':checks,'source_counts':{k:sum(A[k].values()) for k in ('tracks','vias','pads')},'candidate_counts':{k:sum(B[k].values()) for k in ('tracks','vias','pads')}};Path(a.output).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r));assert r['pass']
if __name__=='__main__':main()
