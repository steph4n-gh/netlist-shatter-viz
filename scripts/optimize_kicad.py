"""Measured, selective rerouting. The original board is always a candidate."""
from collections import Counter
from pathlib import Path
import json
import shutil
import sys
import time
from native_project import copy_assets


def findings(report, stable_unconnected_nets=()):
    # Keep identities and measured descriptions, not just category counts: trading
    # one clearance error for a different clearance error is a regression.
    result=Counter()
    for key in ('violations','unconnected_items','schematic_parity'):
        for v in report.get(key,[]):
            items=v.get('items',[])
            # KiCad's airwire spanning tree can pick different endpoint pairs
            # between identical islands. Normalize only for nets whose copper
            # is proved unchanged, with no zones, and never for the edited net.
            stable=[n for n in stable_unconnected_nets if items and all('['+n+']' in i.get('description','') for i in items)]
            if key=='unconnected_items' and len(stable)==1:
                identity={'type':v.get('type'),'severity':v.get('severity'),'unchanged_net':stable[0]}
            else:identity=dict(v,items=sorted(items,key=lambda i:json.dumps(i,sort_keys=True)))
            result[json.dumps(identity,sort_keys=True)]+=1
    return result


def improves(before, after):
    return (after['length_mm'] <= before['length_mm'] + 1e-6 and after['vias'] <= before['vias']
            and (after['length_mm'] < before['length_mm'] - .01 or after['vias'] < before['vias']))


def routing_nonregression(before, after, name):
    """Additional measured constraints, not a claim of electrical equivalence."""
    a=next(n for n in before['nets'].values() if n['name']==name)
    b=next(n for n in after['nets'].values() if n['name']==name)
    return (b['length_mm']<=a['length_mm']+1e-6 and b['vias']<=a['vias']
            and b['pads']==a['pads'] and b['min_width_mm'] is not None
            and b['min_width_mm']>=a['min_width_mm']-1e-6
            and set(b['layers']).issubset(a['layers']))


def unchanged_other_nets(before, after, allowed):
    from solve_kicad import sexpr
    allowed={allowed} if isinstance(allowed,str) else set(allowed)
    allowed={json.dumps(name,ensure_ascii=False) for name in allowed}
    def routing(path):
        tree=sexpr(path.read_text())
        mapping={v[1]:v[2] for v in tree if isinstance(v,list) and len(v)==3 and v[0]=='net'}
        result=[]
        for v in tree:
            if not isinstance(v,list) or not v or v[0] not in ('segment','arc','via'):continue
            token=next((i[1] for i in v if isinstance(i,list) and i and i[0]=='net'),None)
            token=mapping.get(token,token)
            if token not in allowed:result.append(json.dumps(v))
        return Counter(result)
    return routing(before)==routing(after)


def filled_planes_preserved(before, after):
    """Compare actual functional plane copper by net name and physical layer."""
    import shapely
    from grid_route import shape
    def planes(geometry):
        grouped={}
        for zone in geometry.get('zones',[]):
            if zone.get('teardrop'):continue
            name=geometry['nets'][str(zone['net'])]['name']
            layer=geometry['layer_names'][str(zone['layer'])]
            grouped.setdefault((name,layer),[]).append(shape(zone['filled']))
        return {key:shapely.union_all(polygons) for key,polygons in grouped.items()}
    old,new=planes(before),planes(after);rows=[]
    for name,layer in sorted(old.keys()|new.keys()):
        a=old.get((name,layer),shapely.GeometryCollection())
        b=new.get((name,layer),shapely.GeometryCollection())
        missing=a.difference(b).area;added=b.difference(a).area
        rows.append({'net':name,'layer':layer,'missing_mm2':missing,'added_mm2':added,
                     'pass':missing<=1e-10 and added<=1e-10})
    return {'pass':all(row['pass'] for row in rows),'checks':rows,
            'polygon_area_tolerance_mm2':1e-10,
            'scope':'Actual filled functional copper, including voids; net codes and polygon subdivision may differ.'}


def optimize(out, source, worker, kicad, run, save, audit, timeout, protected=(), attempts=8, allowed=None, dc_check=None, dc_before=None, routing_check=None, routing_before=None, routing_profile=None):
    scripts=Path(__file__).resolve().parent
    initial=json.loads((out/'source-metrics.json').read_text())
    unknown=set(protected)-{n['name'] for n in initial['nets'].values()}
    if unknown:raise ValueError('Unknown protected nets: '+', '.join(sorted(unknown)))
    if allowed is not None:
        unknown=set(allowed)-{n['name'] for n in initial['nets'].values()}
        if unknown:raise ValueError('Unknown editable nets: '+', '.join(sorted(unknown)))
        protected=set(protected)|{n['name'] for n in initial['nets'].values() if n['name'] and n['name'] not in allowed}
    if dc_before and not dc_before.get('mesh_check',{}).get('pass',True):raise ValueError('Sheet mesh did not meet refinement tolerance; reduce its pitch before optimization')
    if dc_before and not dc_before.get('rc_hold',{}).get('converged',True):raise ValueError('RC mesh/time refinement did not converge; refine the supplied profile before optimization')
    best=initial;best_dc=dc_before;best_routing=routing_before
    baseline=json.loads((out/'source-drc.json').read_text())
    source_planes=None
    if any(n['zone'] for n in initial['nets'].values()):
        run(worker+['dc-geometry',out],'source-plane-geometry.log')
        source_planes=json.loads((out/'dc-geometry.json').read_text())
        (out/'source-plane-geometry.json').write_text(json.dumps(source_planes))
    trials=[]
    shutil.copy2(out/'candidate.kicad_pcb',out/'original.kicad_pcb')
    shutil.copy2(out/'source-drc.json',out/'routed-drc.json')
    eligible=[(int(code),n) for code,n in initial['nets'].items() if int(code) and n['length_mm']>0
              and n['pads']>=2 and not n['locked'] and not n['zone'] and n['name'] not in protected]
    eligible.sort(key=lambda entry:entry[1]['length_mm'],reverse=True)
    deadline=time.perf_counter()+timeout
    def rules(directory):
        copy_assets(source.parent, directory)
        for suffix in ('.kicad_pro','.kicad_dru'):
            if source.with_suffix(suffix).exists():shutil.copy2(source.with_suffix(suffix),directory/('candidate'+suffix))
    def geometry_at(directory,log):
        run(worker+['dc-geometry' if routing_profile else 'geometry',directory],log)
        g=json.loads((directory/('dc-geometry.json' if routing_profile else 'geometry.json')).read_text())
        if routing_profile:
            g['routing_profile']=routing_profile
            (directory/'geometry.json').write_text(json.dumps(g))
        return g
    def plane_check(directory):
        if source_planes is None:return True
        run(worker+['dc-geometry',directory],directory.name+'/plane-geometry.log')
        result=filled_planes_preserved(source_planes,json.loads((directory/'dc-geometry.json').read_text()))
        (directory/'plane-preservation.json').write_text(json.dumps(result,indent=2)+'\n')
        return result['pass']
    for index,(code,net) in enumerate(eligible[:attempts]):
        if time.perf_counter()>=deadline:break
        d=out/f'optimize-{index+1:02d}';d.mkdir()
        save(stage=f'Improving {net["name"]}: candidate {index+1}/{min(attempts,len(eligible))}')
        # Freeze KiCad-assigned UUIDs for legacy fields in trial boards, while
        # retaining the byte-exact original as the unchanged fallback.
        start=out/'original-normalized.kicad_pcb' if best is initial else out/'candidate.kicad_pcb'
        shutil.copy2(start,d/'candidate.kicad_pcb')
        rules(d)
        trial={'net':net['name'],'directory':d.name,'accepted':False}
        try:
            geometry=geometry_at(d,d.name+'/geometry-before.log')
            from shortcut_route import shortcut
            # Net codes are process/file-local, especially across legacy saves.
            # Resolve the stable net name in the actual candidate being edited.
            code=next(int(n) for n,v in geometry['nets'].items() if v['name']==net['name'])
            (d/'route-nets.json').write_text(json.dumps({'nets':[code],'widths':{str(code):net['width_mm']}}))
            shortcuts=shortcut(geometry,code)
            if shortcuts['remove']:
                (d/'grid-route.json').write_text(json.dumps(shortcuts))
                search={'failed':[],'completed_nets':[code]}
                trial['method']='same-layer shortcuts'
            else:
                trial['method']='selective A*'
                run(worker+['rip-nets',d],d.name+'/remove.log');rules(d)
                geometry_at(d,d.name+'/geometry.log')
                remaining=max(.1,min(10,deadline-time.perf_counter()))
                run([sys.executable,scripts/'grid_route.py',d,'--step','.05','--seconds',remaining],d.name+'/search.log',remaining+10)
                search=json.loads((d/'grid-route.json').read_text())
            if search['failed'] or code not in search['completed_nets']:
                trial['reason']='Search did not complete the net'
            else:
                if source_planes is not None:
                    route_path=d/'grid-route.json'
                    route=json.loads(route_path.read_text())
                    route['preserve_functional_fills']=True
                    route_path.write_text(json.dumps(route))
                run(worker+['apply-grid',d],d.name+'/apply.log');rules(d)
                run(worker+['measure',d],d.name+'/measure.log')
                candidate=json.loads((d/'metrics.json').read_text())
                trial['metrics']={'length_mm':candidate['length_mm'],'vias':candidate['vias']}
                trial['filled_planes_preserved']=plane_check(d)
                if not trial['filled_planes_preserved']:trial['reason']='Functional filled plane copper changed; original retained'
                elif not improves(best,candidate):trial['reason']='No shorter-copper/fewer-via improvement without a tradeoff'
                elif not routing_nonregression(best,candidate,net['name']):trial['reason']='Net width, layer use, connectivity inventory, length or via count regressed'
                else:
                    run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors',
                         '-o',d/'routed-drc.json',d/'candidate.kicad_pcb'],d.name+'/drc.log')
                    report=json.loads((d/'routed-drc.json').read_text())
                    preservation=audit(out/'original-normalized.kicad_pcb',d/'candidate.kicad_pcb')
                    untouched=unchanged_other_nets(start,d/'candidate.kicad_pcb',net['name'])
                    stable={n['name'] for n in best['nets'].values() if n['name'] and n['name']!=net['name'] and not n['zone']} if untouched and preservation['pass'] else set()
                    new=list((findings(report,stable)-findings(baseline,stable)).elements())
                    (d/'preservation.json').write_text(json.dumps(preservation,indent=2))
                    trial['new_findings']=len(new)
                    trial['other_nets_unchanged']=untouched
                    if not preservation['pass'] or new or not untouched:trial['reason']='Structure/other nets changed or new DRC findings appeared'
                    else:
                        routing_ok=True
                        if routing_check:
                            from routing_screen import nonregression as routing_nonregression_check
                            candidate_routing=routing_check(d);routing_ok=routing_nonregression_check(best_routing,candidate_routing)
                            trial['reference_planes_pass']=candidate_routing['pass'];trial['reference_planes_nonregression']=routing_ok
                        dc_ok=True
                        if dc_check:
                            from dc_screen import nonregression
                            candidate_dc=dc_check(d);dc_ok=nonregression(best_dc,candidate_dc)
                            trial['dc_pass']=dc_ok;trial['dc_loads']=candidate_dc['loads'];trial['dc_violations']=candidate_dc['violations']
                        if not routing_ok:trial['reason']='Supplied routing checks regressed; inspect individual terminal paths, widths, layers and vias'
                        elif not dc_ok:trial['reason']='Supplied electrical limits or nonregression checks failed; inspect the candidate electrical evidence'
                        else:
                            shutil.copy2(d/'candidate.kicad_pcb',out/'candidate.kicad_pcb')
                            shutil.copy2(d/'routed-drc.json',out/'routed-drc.json')
                            best=candidate;trial['accepted']=True;trial['reason']='Shorter copper and/or fewer vias; no new DRC findings'
                            if dc_check:best_dc=candidate_dc;trial['reason']+='; supplied electrical limits and nonregression pass'
                            if routing_check:best_routing=candidate_routing;trial['reason']+='; supplied routing-rule nonregression passes'
        except Exception as error:
            trial['reason']=str(error)
        trials.append(trial)
        (out/'optimization.json').write_text(json.dumps(trials,indent=2))
    power_repaired=False
    if dc_check and not best_dc['pass_limits']:
        # Repair all eligible supplies together: partial fixes cannot bypass limits.
        names={n['name'] for _,n in eligible}&set(best_dc['assumptions']['nets'])
        start=out/'original-normalized.kicad_pcb' if best is initial else out/'candidate.kicad_pcb'
        from widen_power import plan as width_plan
        from dc_screen import nonregression
        seen=set()
        for factor in (1.5,2,3,4,8):
            if not names or time.perf_counter()>=deadline:break
            d=out/f'power-width-{factor:g}';d.mkdir()
            trial={'nets':sorted(names),'net':', '.join(sorted(names)),'directory':d.name,'method':'DC supply trace widening','accepted':False}
            save(stage='Repairing DC supply drop: clearance-limited trace widening')
            shutil.copy2(start,d/'candidate.kicad_pcb');rules(d)
            try:
                run(worker+['geometry',d],d.name+'/geometry.log')
                geometry=json.loads((d/'geometry.json').read_text());plan=width_plan(geometry,names,factor)
                signature=json.dumps(plan['changes'],sort_keys=True)
                if not plan['changes'] or signature in seen:
                    trial['reason']='No additional clearance-limited width available'
                    trials.append(trial);(out/'optimization.json').write_text(json.dumps(trials,indent=2));break
                seen.add(signature)
                if source_planes is not None:plan['preserve_functional_fills']=True
                (d/'width-plan.json').write_text(json.dumps(plan,indent=2))
                trial['changed_segments']=len(plan['changes']);trial['max_width_mm']=max(c['width_mm'] for c in plan['changes'])
                run(worker+['widen-power',d],d.name+'/apply.log');rules(d)
                run(worker+['measure',d],d.name+'/measure.log')
                candidate=json.loads((d/'metrics.json').read_text())
                trial['filled_planes_preserved']=plane_check(d)
                if not trial['filled_planes_preserved']:raise ValueError('Functional filled plane copper changed; original retained')
                if any(not routing_nonregression(best,candidate,n) for n in names):raise ValueError('Width repair changed routing inventory, layers or length')
                run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'routed-drc.json',d/'candidate.kicad_pcb'],d.name+'/drc.log')
                report=json.loads((d/'routed-drc.json').read_text())
                preservation=audit(out/'original-normalized.kicad_pcb',d/'candidate.kicad_pcb')
                untouched=unchanged_other_nets(start,d/'candidate.kicad_pcb',names)
                new=list((findings(report)-findings(baseline)).elements())
                (d/'preservation.json').write_text(json.dumps(preservation,indent=2))
                trial.update(new_findings=len(new),other_nets_unchanged=untouched)
                if not preservation['pass'] or new or not untouched:raise ValueError('Structure/other nets changed or new DRC findings appeared')
                candidate_dc=dc_check(d);dc_ok=nonregression(best_dc,candidate_dc)
                if routing_check:
                    from routing_screen import nonregression as routing_nonregression_check
                    candidate_routing=routing_check(d)
                    trial['reference_planes_pass']=routing_nonregression_check(best_routing,candidate_routing)
                    dc_ok=dc_ok and trial['reference_planes_pass']
                trial.update(dc_pass=dc_ok,dc_loads=candidate_dc['loads'],dc_violations=candidate_dc['violations'])
                if dc_ok:
                    shutil.copy2(d/'candidate.kicad_pcb',out/'candidate.kicad_pcb')
                    shutil.copy2(d/'routed-drc.json',out/'routed-drc.json')
                    best=candidate;best_dc=candidate_dc;power_repaired=True
                    trial.update(accepted=True,reason='Supply limits repaired; no modeled voltage/loss regression or new DRC findings')
                else:trial['reason']='Widening did not satisfy all supplied electrical limits and nonregression checks'
            except Exception as error:trial['reason']=str(error)
            trials.append(trial)
            (out/'optimization.json').write_text(json.dumps(trials,indent=2))
            if power_repaired:break
    return {'objective':'Reduce length/vias or repair supplied DC voltage limits without geometric or supplied electrical regression; no new DRC findings',
            'before':initial,'after':best,'improved':improves(initial,best) or power_repaired,'power_repaired':power_repaired,'trials':trials,
            'protected_nets':sorted(protected),'editable_nets':None if allowed is None else sorted(allowed),'eligible_nets':len(eligible),
            'comparison_scope':('Reference-plane rules: each changed result must pass; unchanged existing failures block export; ' if routing_check else '')+('Supplied DC limits, no lower loaded voltage and no higher resistive loss; optional RC limits, refinement and envelope nonregression; ' if dc_check else '')+'Geometric constraints: no new DRC findings, no longer traces, no added vias, no narrower minimum width, no new routing layers; unchanged other nets, actual filled functional planes and board structure',
            'original_retained':True,'electrical_performance_verified':False,'explicit_dc_screen_applied':dc_check is not None,'reference_planes_applied':routing_check is not None}
