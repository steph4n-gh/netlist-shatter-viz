#!/usr/bin/env python3
"""Freeze native geometric reference bounds; never infer electrical qualification."""
import argparse
from collections import Counter
from itertools import combinations
import json
from pathlib import Path
from signal_screen import path_lengths
from routing_screen import screen


def measure(g,unfinished_nets=()):
    unfinished=set(unfinished_nets)
    if unfinished-{n['name'] for n in g['nets'].values()}:raise ValueError('Unknown unfinished net')
    profile={'paths':[],'net_limits':[],'sources':[{'kind':'reference_nonregression',
        'sha256':g['source_board_sha256'],'meaning':'Measured copper paths, width, layers and vias; no impedance, timing or thermal qualification'}]}
    skipped=[]
    planes={z['net'] for z in g.get('zones',[]) if z.get('teardrop') is not True}
    for code,spec in sorted(g['nets'].items(),key=lambda item:item[1]['name']):
        net=int(code);tracks=[t for t in g['tracks'] if t['net']==net]
        if not tracks:continue
        limit={'net':spec['name'],'min_width_mm':min(t['width'] for t in tracks)}
        # An unfinished native connection has no observed complete via/layer
        # budget. Preserve its known paths and width without inventing a cap
        # that prevents adding the missing connection. Explicit profiles bypass
        # this automatic measurement and retain all their supplied constraints.
        if spec['name'] not in unfinished:
            limit.update(max_vias=sum(v['net']==net for v in g['vias']),
                allowed_layers=sorted({g['layer_names'][str(t['layer'])] for t in tracks}))
        profile['net_limits'].append(limit)
        if net in planes:
            skipped.append({'net':spec['name'],'reason':'Functional copper planes require a separate return/plane model'});continue
        pads=[p['ref']+'.'+p['pin'] for p in g['pads'] if p['net']==net]
        duplicate=[p for p,n in Counter(pads).items() if n>1]
        if duplicate:
            skipped.append({'net':spec['name'],'reason':'Ambiguous duplicate terminal names','terminals':duplicate});continue
        for a,b in combinations(sorted(pads),2):
            profile['paths'].append({'id':spec['name']+':'+a+'>'+b,'net':spec['name'],'from':a,'to':b,'max_length_mm':1e6})
    lengths=path_lengths(g,profile['paths']);measured=[]
    for p in profile['paths']:
        value=lengths[p['id']]
        if value['length_mm'] is None:
            skipped.append(dict(p,reason=value['error']));continue
        p['max_length_mm']=round(value['length_mm']+.002,6);measured.append(p)
    profile['paths']=measured
    checked=screen(g,profile)
    if not checked['pass']:raise ValueError('Measured contract does not pass its own reference')
    return profile,{'reference_board_sha256':g['source_board_sha256'],
        'checks':len(checked['rules']),'scope':'Every measurable non-plane pair of distinct named terminals plus routed net width/via/layer bounds'+
            ('; native unfinished nets retain known paths and width without inferred via/layer caps' if unfinished else ''),
        'unfinished_nets_without_observed_via_layer_caps':sorted(unfinished),
        'omissions':skipped,'electrical_qualification':False}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('geometry',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();profile,evidence=measure(json.loads(args.geometry.read_text()))
    args.output.write_text(json.dumps(profile,indent=2)+'\n')
    args.output.with_suffix('.evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(evidence['checks'],'reference checks;',len(evidence['omissions']),'explicit omissions')
