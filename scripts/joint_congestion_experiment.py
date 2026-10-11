#!/usr/bin/env python3
"""Bounded joint routing of an automatically selected failed repair cluster.

Experimental candidate generator only. Foreign generated copper is protected in
DSN and checked independently after import. No fabrication files are emitted.
"""
import argparse
from collections import Counter,defaultdict
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


def native_replace(directory,source):
    """Import selected routes onto exact original native foreign copper."""
    import pcbnew
    import wx
    app=wx.App(False)
    d=Path(directory);selected=set(json.loads((d/'experiment.json').read_text())['selected_nets'])
    original=pcbnew.LoadBoard(str(source));routed=pcbnew.LoadBoard(str(d/'joint-import.kicad_pcb'))
    # Keep SWIG wrappers alive while mutating either native board.
    original_items=list(original.GetTracks());routed_items=list(routed.GetTracks());created=[]
    removed=added=0
    for t in original_items:
        if t.GetNetname() not in selected:continue
        if t.IsLocked():raise ValueError('Selected cluster contains locked original copper')
        original.Remove(t);removed+=1
    for t in routed_items:
        name=t.GetNetname()
        if name not in selected:continue
        if isinstance(t,pcbnew.PCB_VIA):
            item=pcbnew.PCB_VIA(original);item.SetPosition(t.GetPosition());item.SetWidth(t.GetWidth(pcbnew.F_Cu))
            item.SetDrill(t.GetDrillValue());item.SetViaType(t.GetViaType())
            item.SetLayerPair(t.TopLayer(),t.BottomLayer())
        elif isinstance(t,pcbnew.PCB_ARC):raise ValueError('Selected joint-router arc copying is unsupported')
        else:
            item=pcbnew.PCB_TRACK(original);item.SetStart(t.GetStart());item.SetEnd(t.GetEnd())
            item.SetLayer(t.GetLayer());item.SetWidth(t.GetWidth())
        net=original.FindNet(name)
        if net is None:raise ValueError('Imported selected net is absent from original')
        item.SetNetCode(net.GetNetCode());original.Add(item);created.append(item);added+=1
    original.BuildConnectivity();pcbnew.ZONE_FILLER(original).Fill(original.Zones())
    if not pcbnew.SaveBoard(str(d/'candidate.kicad_pcb'),original):raise RuntimeError('Cannot save selected replacement')
    (d/'selected-routing-replacement.json').write_text(json.dumps({'selected_nets':sorted(selected),
        'removed_selected_items':removed,'added_selected_items':added,'foreign_items_copied_from':'Exact original native board',
        'joint_import_retained':True},indent=2)+'\n')


def movable_seed(text,geometry,names):
    """Keep the selected generated routes as movable search context."""
    from dsn_constraints import add_seed,parse,serialize,decode
    text,record=add_seed(text,geometry,names);tree=parse(text)
    wiring=next(v for v in tree if isinstance(v,list) and v and v[0]=='wiring')
    for item in wiring[1:]:
        if not isinstance(item,list) or item[0] not in ('wire','via'):continue
        net=next((decode(v[1]) for v in item if isinstance(v,list) and v and v[0]=='net'),None)
        if net in names:item[:]=[v for v in item if v!=['type','protect']]
    record['scope']='Movable existing generated copper; foreign seed remains protected'
    return serialize(tree),record


def select_cluster(trial_root,geometry,protected=()):
    """Use observed successful-target/failed-neighbor evidence, not net names."""
    names={int(n):v['name'] for n,v in geometry['nets'].items()}
    codes={name:n for n,name in names.items()}
    locked={names[t['net']] for t in geometry['tracks']+geometry['vias'] if t.get('locked',True)}|set(protected)
    choices=[]
    for report in sorted(Path(trial_root).rglob('congestion-repair.json')):
        for index,trial in enumerate(json.loads(report.read_text())):
            cluster={trial['target'],*trial.get('displaced_nets',[])}
            if trial.get('accepted') or not 2<=len(cluster)<=5 or cluster&locked or not cluster<=codes.keys():continue
            directory=report.parent/f'congestion-{index:02d}'
            searches=[]
            for path in directory.glob('search-*.json'):
                if path.stem.endswith('-cropped'):continue
                searches.append(json.loads(path.read_text()))
            target=codes[trial['target']]
            if not any(target in s.get('completed_nets',[]) for s in searches):continue
            failed=trial.get('failed_net')
            if failed is None or failed not in cluster-{trial['target']}:continue
            choices.append((len(cluster),str(directory),sorted(cluster),trial))
    if not choices:raise ValueError('No unlocked small cluster has a completed target and failed displaced neighbor')
    _,directory,cluster,trial=min(choices,key=lambda item:item[:2])
    return cluster,{'trial_directory':directory,'failed_net':trial['failed_net'],'target':trial['target'],
                    'route_order':trial.get('route_order'),'selection':'Smallest observed successful-target/failed-neighbor cluster'}


def foreign_geometry(before,after,selected):
    """Compare native centreline geometry, widths/layers and drilled via stacks.

    Routing UUIDs and collinear segment splits are not physical changes. Exact
    native nanometre precision remains; a finite movement is not tolerated.
    """
    import shapely
    from shapely.geometry import LineString
    selected=set(selected)
    def inventory(g):
        names={int(n):v['name'] for n,v in g['nets'].items()}
        lines=defaultdict(list);vias=Counter();locks=Counter()
        for t in g['tracks']:
            name=names[t['net']]
            if name in selected:continue
            if t['arc']:raise ValueError('Foreign arc preservation requires a native arc comparator')
            key=(name,t['layer'],round(t['width'],6))
            lines[key].append(LineString([t['start'],t['end']]))
            if t.get('locked',True):locks[('track',name)]+=1
        for v in g['vias']:
            name=names[v['net']]
            if name in selected:continue
            vias[(name,*map(lambda x:round(x,6),v['at']),round(v['diameter_mm'],6),round(v['drill_mm'],6),v['through'])]+=1
            if v.get('locked',True):locks[('via',name)]+=1
        return {k:shapely.set_precision(shapely.union_all(v),1e-6) for k,v in lines.items()},vias,locks
    a,av,al=inventory(before);b,bv,bl=inventory(after);changed=[]
    for key in sorted(a.keys()|b.keys()):
        x=a.get(key,LineString());y=b.get(key,LineString())
        if not x.equals(y):changed.append({'net':key[0],'layer':key[1],'width_mm':key[2],
            'removed_centerline_mm':x.difference(y).length,'added_centerline_mm':y.difference(x).length})
    return {'pass':not changed and av==bv and al==bl,'changed_track_groups':changed,
            'removed_vias':[list(v) for v in (av-bv).elements()],
            'added_vias':[list(v) for v in (bv-av).elements()],
            'locked_inventory_unchanged':al==bl,'comparison_precision_mm':1e-6}


def experiment(source,trial_root,out,profile_path,seconds=90,passes=8,protected=(),plane_hints=False,selection_path=None,rebuild_selected=False):
    from dsn_constraints import add_seed,apply
    from native_project import copy_assets
    from routing_screen import screen,nonregression,validate
    from solve_kicad import ROOT,find_kicad,find_python,native_worker_command,summarize_drc,audit_preservation
    from optimize_kicad import unchanged_other_nets
    from build_board import fanout_setting
    source=Path(source).resolve();out=Path(out).resolve();out.mkdir(parents=True,exist_ok=False)
    start=time.perf_counter();commands=[]
    profile_bytes=Path(profile_path).read_bytes();profile=validate(json.loads(profile_bytes))
    result={'state':'running','review_pass':False,'fabrication_exported':False,
            'source_board':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'profile_sha256':hashlib.sha256(profile_bytes).hexdigest(),'router_timeout_seconds':seconds,
            'passes':passes,'manual_trace_coordinates':False,'candidate_only':True,
            'dsn_planes_are_search_hints_only':plane_hints,
            'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    def save():
        result['elapsed_seconds']=time.perf_counter()-start
        (out/'experiment.json').write_text(json.dumps(result,indent=2)+'\n')
    def run(command,label,timeout=90):
        command=list(map(str,command));commands.append(command)
        project={suffix:(out/('candidate'+suffix)).read_bytes() for suffix in ('.kicad_pro','.kicad_dru') if (out/('candidate'+suffix)).exists()}
        (out/'commands.json').write_text(json.dumps(commands,indent=2)+'\n');save()
        try:
            with (out/(label+'.log')).open('w') as log:
                subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=timeout)
        finally:
            for suffix,value in project.items():(out/('candidate'+suffix)).write_bytes(value)
    save()
    try:
        copy_assets(source.parent,out)
        for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
            if source.with_suffix(suffix).exists():shutil.copy2(source.with_suffix(suffix),out/('candidate'+suffix))
        shutil.copy2(source,out/'source.kicad_pcb')
        kicad=find_kicad();worker=native_worker_command(kicad,Path(__file__).with_name('native_kicad_worker.py'))
        run(worker+['dc-geometry',out],'source-geometry')
        before=json.loads((out/'dc-geometry.json').read_text());before['routing_profile']=profile
        (out/'source-geometry.json').write_text(json.dumps(before))
        if selection_path is None:selected,decision=select_cluster(trial_root,before,protected)
        else:
            decision=json.loads(Path(selection_path).read_text());selected=decision['selected_nets']
            known={n['name'] for n in before['nets'].values()}
            locked={before['nets'][str(t['net'])]['name'] for t in before['tracks']+before['vias'] if t.get('locked',True)}|set(protected)
            if not 1<=len(selected)<=5 or len(set(selected))!=len(selected) or not set(selected)<=known or set(selected)&locked:
                raise ValueError('External graph selection must be unique, bounded, known and unlocked')
            if decision.get('source_sha256')!=result['source_sha256']:raise ValueError('Graph selection source hash differs')
            result['selection_sha256']=hashlib.sha256(Path(selection_path).read_bytes()).hexdigest()
        result.update(selected_nets=selected,selection=decision)
        baseline=screen(before,profile);(out/'source-routing-screen.json').write_text(json.dumps(baseline,indent=2))
        run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',out/'source-drc.json',out/'candidate.kicad_pcb'],'source-drc')
        result['before']=summarize_drc(out/'source-drc.json')
        run(worker+['prepare-planes' if plane_hints else 'prepare',out],'prepare')
        info=json.loads((out/'inventory.json').read_text())
        foreign={n['name'] for n in before['nets'].values()}-set(selected)
        seed_geometry=dict(before,vias=[v for v in before['vias'] if not v.get('locked',True)])
        dsn,seed=add_seed((out/'input.dsn').read_text(),seed_geometry,foreign)
        if rebuild_selected:
            movable={'scope':'Selected generated copper omitted for measured joint rebuild','nets':sorted(selected),'tracks':0,'vias':0}
        else:dsn,movable=movable_seed(dsn,seed_geometry,set(selected))
        dsn,constraints=apply(dsn,profile,info['routing_clearance_margin_um'])
        (out/'input.dsn').write_text(dsn);(out/'seed.json').write_text(json.dumps(seed,indent=2))
        (out/'search-constraints.json').write_text(json.dumps(constraints,indent=2))
        result['protected_seed']=seed;result['movable_seed']=movable
        (out/'movable-seed.json').write_text(json.dumps(movable,indent=2))
        result['stage']='Joint neighborhood routing';save()
        java=os.getenv('JAVA') or shutil.which('java');jar=os.getenv('FREEROUTING_JAR')
        if not java or not jar:raise ValueError('Set JAVA and FREEROUTING_JAR to the external routing tools')
        route_start=time.perf_counter()
        run([java,'-Xmx2g','-jar',jar,'-de',out/'input.dsn','-do',out/'route.ses','-mp',passes,'-mt','1','-da',
             '--gui.enabled=false','--api_server.enabled=false','--router.automatic_neckdown=false',
             fanout_setting(jar),'--router.optimizer.enabled=false',
             f'--router.job_timeout=00:{seconds//60:02d}:{seconds%60:02d}',
             f'--router.copper_to_edge_clearance_um={info["copper_edge_clearance_um"]}',
             f'--router.hole_clearance_um={info["hole_clearance_um"]}',f'--user_data_path={out/"router-settings"}'],
             'routing',seconds+15)
        result['routing_seconds']=time.perf_counter()-route_start
        run(worker+['finish',out],'finish')
        run(worker+['dc-geometry',out],'checked-geometry')
        after=json.loads((out/'dc-geometry.json').read_text());after['routing_profile']=profile
        raw_compared=foreign_geometry(before,after,selected)
        (out/'raw-import-foreign-copper.json').write_text(json.dumps(raw_compared,indent=2))
        result['raw_import_foreign_copper']=raw_compared
        shutil.copy2(out/'candidate.kicad_pcb',out/'joint-import.kicad_pcb')
        run(native_worker_command(kicad,Path(__file__).resolve())+['--native-replace',out,'--source-board',source],'selected-replacement')
        run(worker+['dc-geometry',out],'replacement-geometry')
        after=json.loads((out/'dc-geometry.json').read_text());after['routing_profile']=profile
        compared=foreign_geometry(before,after,selected)
        compared['original_foreign_items_unchanged']=unchanged_other_nets(source,out/'candidate.kicad_pcb',selected)
        compared['pass']=compared['pass'] and compared['original_foreign_items_unchanged']
        (out/'foreign-copper-preservation.json').write_text(json.dumps(compared,indent=2))
        routing=screen(after,profile);(out/'routing-screen.json').write_text(json.dumps(routing,indent=2))
        run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',out/'routed-drc.json',out/'candidate.kicad_pcb'],'drc')
        preservation=audit_preservation(out/'source.kicad_pcb',out/'candidate.kicad_pcb')
        (out/'preservation.json').write_text(json.dumps(preservation,indent=2))
        result.update(after=summarize_drc(out/'routed-drc.json'),foreign_copper=compared,
                      preservation=preservation,routing_pass=routing['pass'],routing_checks=len(routing['rules']),
                      routing_nonregression=nonregression(baseline,routing))
        a,b=result['before'],result['after']
        disconnected=json.loads((out/'routed-drc.json').read_text()).get('unconnected_items',[])
        result['selected_complete']=not any('['+name+']' in item['description'] for name in selected for issue in disconnected for item in issue['items'])
        result['project_exact']=all((out/('candidate'+suffix)).read_bytes()==source.with_suffix(suffix).read_bytes() for suffix in ('.kicad_pro','.kicad_dru') if source.with_suffix(suffix).exists())
        result['candidate_improves']=(result['selected_complete'] and result['project_exact'] and compared['pass'] and preservation['pass'] and routing['pass'] and
            result['routing_nonregression'] and b['unconnected']<a['unconnected'] and
            b['total_findings']<a['total_findings'] and all(n<=a['types'].get(k,0) for k,n in b['types'].items()))
        result['review_pass']=result['candidate_improves'] and b['total_findings']==0
        result['state']='done'
    except (ValueError,RuntimeError,subprocess.SubprocessError) as error:
        result.update(state='failed',error=str(error))
    result['source_unchanged']=hashlib.sha256(source.read_bytes()).hexdigest()==result['source_sha256']
    save();return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path,nargs='?');p.add_argument('trial_root',type=Path,nargs='?');p.add_argument('out',type=Path,nargs='?')
    p.add_argument('--profile',type=Path);p.add_argument('--seconds',type=int,default=90)
    p.add_argument('--passes',type=int,default=8);p.add_argument('--protect-net',action='append',default=[])
    p.add_argument('--plane-hints',action='store_true',help='Use DSN plane outlines only as search hints; native gates remain unchanged')
    p.add_argument('--rebuild-selected',action='store_true',help='Omit selected generated copper for a fresh joint neighborhood rebuild')
    p.add_argument('--selection',type=Path,help='Recorded bounded graph selection matching the source board hash')
    p.add_argument('--native-replace',type=Path);p.add_argument('--source-board',type=Path)
    a=p.parse_args()
    if a.native_replace:
        if a.source_board is None:p.error('--source-board required for native selected replacement')
        native_replace(a.native_replace,a.source_board);return
    if any(v is None for v in (a.source,a.trial_root,a.out,a.profile)):p.error('Source, trial root, output and --profile are required')
    if not 1<=a.seconds<=300 or not 1<=a.passes<=30:p.error('Bound seconds1–300 and passes1–30')
    print(json.dumps(experiment(a.source,a.trial_root,a.out,a.profile,a.seconds,a.passes,a.protect_net,a.plane_hints,a.selection,a.rebuild_selected),indent=2))


if __name__=='__main__':main()
