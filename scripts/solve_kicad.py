#!/usr/bin/env python3
"""Route, repair or improve native KiCad copper under its supplied project rules."""
import argparse
from collections import Counter
import hashlib
import html
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import zipfile
from build_board import find_kicad, fanout_setting
from kicad_sexpr import sexpr, zone_definition
from native_project import copy_assets

ROOT = Path(__file__).resolve().parents[1]


def find_python(kicad):
    if os.getenv('KICAD_PYTHON'):
        return os.environ['KICAD_PYTHON']
    contents = Path(kicad).parent.parent
    matches = sorted(contents.glob('Frameworks/Python.framework/Versions/*/bin/python3.*'))
    return str(next((p for p in matches if re.fullmatch(r'python3\.\d+', p.name)), Path(sys.executable)))


def native_worker_command(kicad, script):
    """Keep macOS crash-state restoration from opening a modal dialog in a worker."""
    python = find_python(kicad)
    if sys.platform != 'darwin':
        return [python, script]
    # Cocoa reads the original process argv. Strip its option only from Python's
    # argv, and reproduce normal script imports before running the unchanged worker.
    launcher = ("import runpy,sys;from pathlib import Path;script=sys.argv[1];"
                "sys.path.insert(0,str(Path(script).resolve().parent));"
                "sys.argv=[script]+sys.argv[4:];runpy.run_path(script,run_name='__main__')")
    return [python, '-c', launcher, str(script), '-ApplePersistenceIgnoreState', 'YES']


def structure(path, removed_generated_zones=()):
    """Keep every non-routing field, including all footprint, outline and rule-area geometry."""
    tree = sexpr(path.read_text())
    routing_ids = {v[1] for item in tree if isinstance(item, list) and item and item[0] in ('segment','arc','via')
                   for v in item if isinstance(v, list) and len(v)>1 and v[0]=='uuid'}
    removals=Counter(removed_generated_zones)
    result = []
    for item in tree:
        if isinstance(item, list) and item:
            if item[0] in ('segment', 'arc', 'via'):
                continue
            if item[0] == 'zone':
                item=zone_definition(item)
                digest=hashlib.sha256(json.dumps(item,separators=(',',':')).encode()).hexdigest()
                if removals[digest]:removals[digest]-=1;continue
            if item[0] == 'group':
                item = [[v[0]] + [member for member in v[1:] if member not in routing_ids]
                        if isinstance(v,list) and v and v[0]=='members' else v for v in item]
                if ['members'] in item:
                    continue  # A group containing only replaced copper has no surviving members.
        result.append(item)
    return result


def audit_preservation(before, after, generated_zone_removals=None):
    allowed=[];invalid=[]
    if generated_zone_removals is not None:
        before_tree=sexpr(before.read_text());after_tree=sexpr(after.read_text())
        def field(item,name):
            return next((v for v in item if isinstance(v,list) and v and v[0]==name),None)
        def locked(item):
            value=field(item,'locked')
            return 'locked' in item or value is not None and value!=['locked','no']
        def digest(item):return hashlib.sha256(json.dumps(zone_definition(item),separators=(',',':')).encode()).hexdigest()
        zones={digest(v):v for v in before_tree if isinstance(v,list) and v and v[0]=='zone'}
        source_counts=Counter(digest(v) for v in before_tree if isinstance(v,list) and v and v[0]=='zone')
        locked_nets={tuple(field(v,'net') or ()) for v in before_tree if isinstance(v,list) and v and v[0] in ('segment','arc','via') and locked(v)}
        after_counts=Counter(digest(v) for v in after_tree if isinstance(v,list) and v and v[0]=='zone')
        if generated_zone_removals.get('normalized_source_sha256')!=hashlib.sha256(before.read_bytes()).hexdigest():
            invalid.append('Generated-zone manifest does not match the normalized source')
        native_nets=Counter(v['net_name'] for v in generated_zone_removals.get('removed_generated_zones',[]) if v.get('native_teardrop') is True)
        allowed_nets=Counter()
        for record in generated_zone_removals.get('removed_zone_definitions',[]):
            ident=record.get('definition_sha256');zone=zones.get(ident);attr=field(zone,'attr') if zone is not None else None
            marker=next((v for v in (attr or [])[1:] if isinstance(v,list) and v and v[0]=='teardrop'),None)
            # Source syntax and the native classification must independently
            # identify generated copper. No arbitrary zone deletion is allowed.
            kind=field(marker,'type') if marker is not None else None
            net=field(zone,'net') if zone is not None else None
            eligible=(zone is not None and record.get('native_teardrop') is True
                      and kind in (['type','padvia'],['type','trackend'])
                      and field(zone,'keepout') is None and not locked(zone)
                      and tuple(net or ()) not in locked_nets
                      and source_counts[ident]-after_counts[ident]>allowed.count(ident)
                      and net==['net',json.dumps(record.get('net_name'),ensure_ascii=False)])
            if not eligible:invalid.append('Invalid generated-zone removal: '+str(ident))
            else:allowed.append(ident);allowed_nets[record['net_name']]+=1
        if native_nets!=allowed_nets:invalid.append('Native and serialized generated-zone removal inventories differ')
    a, b = structure(before,allowed), structure(after)
    # KiCad may reorder equivalent top-level objects on save.
    canon = lambda tree: Counter(json.dumps(v, separators=(',', ':')) for v in tree)
    ca, cb = canon(a), canon(b)
    return {'pass': ca == cb and not invalid, 'removed_fields': list((ca - cb).elements()),
            'added_fields': list((cb - ca).elements()),'removed_generated_zone_definition_sha256':sorted(allowed),
            'invalid_generated_zone_removals':invalid}


def summarize_drc(path):
    report = json.loads(path.read_text())
    items = report.get('violations', []) + report.get('unconnected_items', []) + report.get('schematic_parity', [])
    return {'errors': sum(v.get('severity') == 'error' for v in items),
            'warnings': sum(v.get('severity') == 'warning' for v in items),
            'excluded': sum(v.get('severity') == 'exclusion' for v in items),
            'unconnected': len(report.get('unconnected_items', [])),
            'total_findings': len(items), 'types': dict(Counter(v.get('type', 'unknown') for v in items)),
            'ignored_checks': report.get('ignored_checks', [])}


def solve(source, out, timeout=180, passes=30, kicad=None, mode="route", protected=(), attempts=8, creation=None, allowed=None, dc_profile=None, routing_profile=None, plane_seed=False):
    if mode not in ('route','repair','improve'):raise ValueError('Mode must be route, repair or improve')
    if allowed is not None and mode!='improve':raise ValueError('Editable nets require improvement mode')
    if not 1<=attempts<=32:raise ValueError('Improvement attempts must be 1–32')
    source, out = Path(source).resolve(), Path(out).resolve()
    if source.suffix != '.kicad_pcb' or not source.is_file():
        raise ValueError('Input must be an existing .kicad_pcb file')
    if out.exists() and any(out.iterdir()):
        raise ValueError('Output directory must be empty; previous attempts are never overwritten')
    kicad = kicad or find_kicad()
    if not kicad:
        raise RuntimeError('KiCad CLI is required')
    out.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    result = {'state': 'running', 'stage': 'Preparing native board', 'review_pass': False,
              'mode':mode, 'hardware_validated': False, 'manual_trace_edits': 0, 'source': str(source)}
    result['protected_nets']=list(protected)
    if creation is not None:result['creation']=creation
    commands = []
    generated_zone_removals=None
    project_assets=[]

    def route_audit(before,after):
        return audit_preservation(before,after,generated_zone_removals)

    def save(**fields):
        result.update(fields)
        result['elapsed_seconds'] = time.perf_counter() - started + (creation or {}).get('elapsed_seconds',0)
        (out / 'status.json').write_text(json.dumps(result, indent=2) + '\n')

    def run(command, log, seconds=60):
        command = list(map(str, command))
        commands.append(command)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2))
        try:
            with (out / log).open('w') as f:
                r = subprocess.run(command, cwd=ROOT, stdout=f, stderr=subprocess.STDOUT, timeout=seconds)
        finally:
            target=Path(command[-1])
            directory=target.parent if target.suffix=='.kicad_pcb' else target
            if directory.is_dir() and directory.is_relative_to(out) and (directory/'candidate.kicad_pcb').is_file():
                for suffix in ('.kicad_pro','.kicad_dru'):
                    project=source.with_suffix(suffix)
                    if project.is_file():shutil.copy2(project,directory/('candidate'+suffix))
        if r.returncode:
            raise RuntimeError(f'{log}: command exited {r.returncode}')

    save()
    try:
        copied = {}
        project_assets = copy_assets(source.parent, out)
        for name in project_assets:
            copied[name] = hashlib.sha256((source.parent / name).read_bytes()).hexdigest()
        if creation is not None or (source.parent/'source-design.json').is_file():
            spec=source.with_suffix('.json') if creation is not None else source.parent/'source-design.json'
            shutil.copy2(spec,out/'source-design.json')
            copied[spec.name]=hashlib.sha256(spec.read_bytes()).hexdigest()
        for suffix in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
            f = source.with_suffix(suffix)
            if f.exists():
                shutil.copy2(f, out / ('candidate' + suffix))
                copied[f.name] = hashlib.sha256(f.read_bytes()).hexdigest()
        (out / 'source-manifest.json').write_text(json.dumps(copied, indent=2))
        if not source.with_suffix('.kicad_pro').exists():
            raise ValueError('Matching .kicad_pro required to preserve project rules and net classes')
        if not isinstance(json.loads(source.with_suffix('.kicad_pro').read_text()), dict):
            raise ValueError('KiCad project must be a JSON object')
        worker = native_worker_command(kicad, ROOT / 'scripts/native_kicad_worker.py')
        from native_restoration import NAME,snapshot,bind_contacts,audit as audit_restoration
        baseline=out/NAME
        snapshot(source,baseline,profiles={'.dc.json':dc_profile,'.routing.json':routing_profile})
        if not (baseline/'native-connectivity.json').is_file():
            save(stage='Freezing original native connectivity obligations')
            run(worker+['native-connectivity',baseline],NAME+'/native-connectivity.log')
            bind_contacts(baseline)
        dc_check=None;dc_before=None;profile_hash=None
        if dc_profile is not None:
            from dc_screen import validate,screen
            dc_profile=Path(dc_profile).resolve()
            profile_bytes=dc_profile.read_bytes();profile_hash=hashlib.sha256(profile_bytes).hexdigest()
            profile=validate(json.loads(profile_bytes))
            (out/'candidate.dc.json').write_bytes(profile_bytes)
            result['dc_screen']={'scope':'Explicit DC'+(' and passive RC current-hold scenarios' if 'rc_hold' in profile else ' scenario')+'; '+('supplied return paths' if profile.get('returns') else 'ideal ground')+'; no regulator-loop, inductive, thermal or SI validation','profile_sha256':profile_hash}
            def dc_check(directory):
                label=str(directory.relative_to(out)/'dc-geometry.log')
                run(worker+['dc-geometry',directory],label)
                report=screen(json.loads((directory/'dc-geometry.json').read_text()),profile)
                (directory/'dc-screen.json').write_text(json.dumps(report,indent=2)+'\n')
                return report
        routing_check=None;routing_before=None;routing_rules=None
        if routing_profile is not None:
            from routing_screen import validate as validate_routing,screen as screen_routing
            routing_profile=Path(routing_profile).resolve();routing_bytes=routing_profile.read_bytes()
            routing_hash=hashlib.sha256(routing_bytes).hexdigest();routing_rules=validate_routing(json.loads(routing_bytes))
            (out/'candidate.routing.json').write_bytes(routing_bytes)
        def check_routing(directory):
            from routing_screen import screen as screen_routing
            try:run(worker+['dc-geometry',directory],str(directory.relative_to(out)/'reference-geometry.log'))
            finally:restore_project(directory)
            report=screen_routing(json.loads((directory/'dc-geometry.json').read_text()),routing_rules)
            (directory/'routing-screen.json').write_text(json.dumps(report,indent=2)+'\n')
            return report
        if routing_rules is not None:routing_check=check_routing
        def routing_geometry(directory,log):
            run(worker+['dc-geometry' if routing_rules else 'geometry',directory],log)
            if routing_rules:
                g=json.loads((directory/'dc-geometry.json').read_text());g['routing_profile']=routing_rules
                (directory/'geometry.json').write_text(json.dumps(g))
        def restore_project(directory=out):
            # SaveBoard migrates old project files. Preserve the supplied bytes for
            # every independent check and the delivered project, not the migration.
            for suffix in ('.kicad_pro', '.kicad_dru'):
                f = source.with_suffix(suffix)
                if f.exists():
                    shutil.copy2(f, directory / ('candidate' + suffix))
        # Baseline includes pre-existing source defects and excluded findings.
        run([kicad, 'pcb', 'drc', '--format', 'json', '--severity-all', '--all-track-errors',
             '-o', out / 'source-drc.json', out / 'candidate.kicad_pcb'], 'source-drc.log')
        run(worker + ['inspect', out], 'inspect.log')
        restore_project()
        shutil.copy2(out / 'metrics.json', out / 'source-metrics.json')
        result['source_metrics'] = json.loads((out / 'source-metrics.json').read_text())
        result['source_drc'] = summarize_drc(out / 'source-drc.json')
        unknown=set(protected)-{n['name'] for n in result['source_metrics']['nets'].values()}
        if unknown:raise ValueError('Unknown protected nets: '+', '.join(sorted(unknown)))
        if mode in ('repair','improve') and routing_check is None and result['source_metrics']['length_mm']>0:
            from measure_routing_contract import measure as measure_contract
            save(stage='Freezing measurable terminal-path bounds from the supplied board')
            run(worker+['dc-geometry',out],'measured-contract-geometry.log')
            geometry=json.loads((out/'dc-geometry.json').read_text())
            unfinished=[]
            if mode=='repair':
                from repair_native_congestion import native_open_counts
                counts=native_open_counts(geometry,json.loads((out/'source-drc.json').read_text()))
                unfinished=[geometry['nets'][str(c)]['name'] for c,n in counts.items() if n]
            routing_rules,evidence=measure_contract(geometry,unfinished_nets=unfinished)
            routing_profile=out/'candidate.routing.json'
            routing_bytes=(json.dumps(routing_rules,indent=2)+'\n').encode()
            routing_profile.write_bytes(routing_bytes);routing_hash=hashlib.sha256(routing_bytes).hexdigest()
            (out/'measured-contract.json').write_text(json.dumps(evidence,indent=2)+'\n')
            result['measured_contract']=evidence;routing_check=check_routing
        if mode == 'improve':
            from optimize_kicad import optimize
            if dc_check:
                save(stage='Measuring the supplied electrical scenarios on the original board')
                dc_before=dc_check(out)
                shutil.copy2(out/'dc-screen.json',out/'source-dc-screen.json')
            if routing_check:
                routing_before=routing_check(out)
                shutil.copy2(out/'routing-screen.json',out/'source-routing-screen.json')
            result['inventory'] = json.loads((out / 'inventory.json').read_text())
            result['optimization'] = optimize(out,source,worker,kicad,run,save,audit_preservation,timeout,protected,attempts,allowed,dc_check,dc_before,routing_check,routing_before,routing_rules)
            restore_project()
            result['selected_router'] = 'power-width' if result['optimization'].get('power_repaired') else ('selective-grid' if result['optimization']['improved'] else 'original')
            result['routed_drc'] = summarize_drc(out / 'routed-drc.json')
        elif mode=='repair':
            shutil.copy2(source,out/'original.kicad_pcb')
            shutil.copy2(out/'original-normalized.kicad_pcb',out/'candidate.kicad_pcb')
            run([kicad, 'pcb', 'drc', '--format', 'json', '--severity-all', '--all-track-errors',
                 '-o', out / 'routed-drc.json', out / 'candidate.kicad_pcb'], 'repair-baseline-drc.log')
            result['inventory']=json.loads((out/'inventory.json').read_text())
            result['selected_router']='native-local-repair'
            result['routed_drc']=summarize_drc(out/'routed-drc.json')
            result['router_trials']=[{'engine':result['selected_router'],'drc':result['routed_drc']}]
        else:
            run(worker + ['prepare-planes' if plane_seed else 'prepare', out], 'prepare.log')
            generated_zone_removals=json.loads((out/'generated-zone-removals.json').read_text())
            result['generated_zone_removals']=generated_zone_removals
            prepared_audit=route_audit(out/'original-normalized.kicad_pcb',out/'routing-base.kicad_pcb')
            (out/'prepared-preservation.json').write_text(json.dumps(prepared_audit,indent=2)+'\n')
            if not prepared_audit['pass']:raise ValueError('Fresh route preparation changed functional board structure')
            run([sys.executable,ROOT/'scripts/check_native_dsn_obstacles.py',out],'dsn-obstacle-check.log')
            result['native_dsn_obstacles']=json.loads((out/'native-obstacle-checks.json').read_text())
            restore_project()
            result['inventory'] = json.loads((out / 'inventory.json').read_text())
            save(stage='Autorouting native KiCad geometry')
            java = os.getenv('JAVA') or str(ROOT / '.tools/jre/bin/java')
            jar = os.getenv('FREEROUTING_JAR') or str(ROOT / '.tools/freerouting.jar')
            route_start = time.perf_counter()
            try:
                if routing_rules and routing_rules.get('reference_planes'):
                    routing_geometry(out,'reference-route-geometry.log')
                    run([sys.executable,ROOT/'scripts/grid_route.py',out,'--seconds',min(timeout,30),'--step','.05'],'reference-route.log',min(timeout,30)+10)
                    run(worker+['apply-grid',out],'reference-route-apply.log')
                else:
                    if routing_rules and routing_rules.get('paths'):
                        from seed_constrained_routes import seed
                        save(stage='Routing constrained signals before the remaining board')
                        result['constraint_seed']=seed(out,worker,kicad,run,routing_rules,min(timeout,180))
                        save(stage='Routing remaining connections around checked signal routes')
                    if routing_rules and routing_rules.get('net_limits'):
                        from dsn_constraints import apply as constrain_dsn
                        dsn,applied=constrain_dsn((out/'input.dsn').read_text(),routing_rules,result['inventory'].get('routing_clearance_margin_um',0))
                        (out/'input-unconstrained.dsn').write_bytes((out/'input.dsn').read_bytes())
                        (out/'input.dsn').write_text(dsn)
                        (out/'search-constraints.json').write_text(json.dumps(applied,indent=2)+'\n')
                    run([java, '-Xmx2g', '-jar', jar, '-de', out / 'input.dsn', '-do', out / 'route.ses',
                         '-mp', str(passes), '-mt', '1', '-da', '--gui.enabled=false', '--api_server.enabled=false',
                         '--router.automatic_neckdown=false', fanout_setting(jar),
                         '--router.optimizer.enabled=false',
                         f'--router.job_timeout=00:{max(1,int(timeout*.85))//60:02d}:{max(1,int(timeout*.85))%60:02d}',
                         f'--router.copper_to_edge_clearance_um={result["inventory"]["copper_edge_clearance_um"]}',
                         f'--router.hole_clearance_um={result["inventory"]["hole_clearance_um"]}',
                         f'--user_data_path={out / "router-settings"}'], 'routing.log', timeout)
                    save(stage='Importing route and refilling original planes', routing_seconds=time.perf_counter()-route_start)
                    run(worker + ['finish', out], 'finish.log')
            except (RuntimeError, subprocess.TimeoutExpired) as error:
                result['primary_router_error'] = str(error)
                shutil.copy2(out / 'routing-base.kicad_pcb', out / 'candidate.kicad_pcb')
            restore_project()
            # Never weaken rules or delete findings to make a route pass.
            save(stage='Checking original project rules')
            run([kicad, 'pcb', 'drc', '--format', 'json', '--severity-all', '--all-track-errors',
                 '-o', out / 'routed-drc.json', out / 'candidate.kicad_pcb'], 'routed-drc.log')
            result['source_drc'] = summarize_drc(out / 'source-drc.json')
            result['routed_drc'] = summarize_drc(out / 'routed-drc.json')
            result['selected_router'] = ('reference-grid' if routing_rules and routing_rules.get('reference_planes') else 'freerouting') if 'primary_router_error' not in result else 'unrouted'
            result['router_trials'] = [{'engine': result['selected_router'], 'drc': result['routed_drc']}]
            if result['routed_drc']['types'].get('hole_to_hole'):
                from repair_native_vias import repair
                save(stage='Repairing local drill spacing while preserving routed connections')
                result['via_repair']=repair(out,worker,kicad,seconds=min(timeout,90))
                result['routed_drc']=summarize_drc(out/'routed-drc.json')
            if any(result['routed_drc']['types'].get(t) for t in ('via_dangling','track_dangling')):
                from prune_native_routing import prune
                result['pruning']=prune(out,worker,kicad,protected=protected,profile=routing_rules)
                result['routed_drc']=summarize_drc(out/'routed-drc.json')
        if mode=='repair' and any(result['routed_drc']['types'].get(t) for t in ('via_dangling','track_dangling')):
            from prune_native_routing import prune
            save(stage='Consolidating obsolete via copper under original source obligations')
            result['initial_pruning']=prune(out,worker,kicad,prefix='initial-prune',protected=protected,profile=routing_rules)
            result['routed_drc']=summarize_drc(out/'routed-drc.json')
        def grid_trials(modes):
            for grid_mode in modes:
                if not (result['routed_drc']['errors'] or result['routed_drc']['unconnected']
                        or result['routed_drc']['types'].get('track_dangling') or result['routed_drc']['types'].get('via_dangling')):
                    break
                reference_before=routing_check(out) if routing_check else None
                restore_project()
                save(stage='Searching continuous native-clearance polygons' if grid_mode=='repair-continuous' else 'Searching retained copper with bounded A*')
                grid = out / ('grid-' + grid_mode)
                grid.mkdir()
                copy_assets(source.parent, grid)
                shutil.copy2(out / ('candidate.kicad_pcb' if grid_mode.startswith('repair') else 'routing-base.kicad_pcb'), grid / 'candidate.kicad_pcb')
                for suffix in ('.kicad_pro', '.kicad_dru'):
                    if source.with_suffix(suffix).exists():
                        shutil.copy2(source.with_suffix(suffix), grid / ('candidate'+suffix))
                try:
                    if grid_mode.startswith('repair'):
                        shutil.copy2(out / 'routed-drc.json', grid / 'input-drc.json')
                        (grid/'route-nets.json').write_text(json.dumps({'protected_nets':list(protected),'preserve_dangling_vias':True}))
                        run(worker + ['prepare-repair', grid], 'grid-repair-prepare.log')
                        for suffix in ('.kicad_pro', '.kicad_dru'):
                            if source.with_suffix(suffix).exists():
                                shutil.copy2(source.with_suffix(suffix), grid / ('candidate'+suffix))
                    plan=json.loads((grid/'route-nets.json').read_text()) if (grid/'route-nets.json').exists() else None
                    if plan is not None and not plan.get('nets'):raise ValueError('No eligible unlocked, unprotected repair nets')
                    routing_geometry(grid,f'grid-{grid_mode}-geometry.log')
                    restore_project(grid)
                    search_seconds=min(timeout,60 if grid_mode=='repair-continuous' else (90 if grid_mode=='repair-cleanup' else 30))
                    run([sys.executable, ROOT / 'scripts/grid_route.py', grid, '--seconds', search_seconds, '--step', '.025' if grid_mode in ('repair-cleanup','repair-continuous') else ('.05' if grid_mode=='repair' else '.1')]+(['--continuous'] if grid_mode=='repair-continuous' else []), f'grid-{grid_mode}-search.log', search_seconds+10)
                    run(worker + ['apply-grid', grid], f'grid-{grid_mode}-apply.log')
                    for suffix in ('.kicad_pro', '.kicad_dru'):
                        if source.with_suffix(suffix).exists():
                            shutil.copy2(source.with_suffix(suffix), grid / ('candidate'+suffix))
                    run([kicad, 'pcb', 'drc', '--format', 'json', '--severity-all', '--all-track-errors',
                         '-o', grid / 'routed-drc.json', grid / 'candidate.kicad_pcb'], f'grid-{grid_mode}-drc.log')
                    grid_drc = summarize_drc(grid / 'routed-drc.json')
                    grid_audit = route_audit(out / 'original-normalized.kicad_pcb', grid / 'candidate.kicad_pcb')
                    (grid / 'preservation.json').write_text(json.dumps(grid_audit,indent=2))
                    reference_after=routing_check(grid) if routing_check else None
                    restore_project(grid)
                    from routing_screen import nonregression as grid_nonregression
                    reference_preserved=reference_before is None or grid_nonregression(reference_before,reference_after)
                    from optimize_kicad import unchanged_other_nets
                    selected_names={n['name'] for code,n in result['source_metrics']['nets'].items() if plan is not None and int(code) in plan['nets']}
                    other_copper_preserved=plan is None or unchanged_other_nets(out/'candidate.kicad_pcb',grid/'candidate.kicad_pcb',selected_names)
                    result['router_trials'].append({'engine':'native-grid-'+grid_mode,'drc':grid_drc,
                        'preservation_pass':grid_audit['pass'],'routing_nonregression':reference_preserved,
                        'other_nets_unchanged':other_copper_preserved})
                    score = lambda d: (d['errors'],d['unconnected'],d['total_findings'])
                    if grid_audit['pass'] and reference_preserved and other_copper_preserved and score(grid_drc) < score(result['routed_drc']):
                        for name in ('candidate.kicad_pcb','routed-drc.json','routed-inventory.json'):
                            if (out / name).exists():shutil.copy2(out / name, out / (result['selected_router']+'-'+name))
                            shutil.copy2(grid / name, out / name)
                        result['selected_router'], result['routed_drc'] = 'native-grid-'+grid_mode, grid_drc
                except (RuntimeError, subprocess.TimeoutExpired, ValueError) as error:
                    result['router_trials'].append({'engine':'native-grid-'+grid_mode,'error':str(error)})
        native_findings=json.loads((out/'routed-drc.json').read_text())
        physical_errors=any(v.get('severity')=='error' for v in
            native_findings.get('violations',[])+native_findings.get('schematic_parity',[]))
        repair_profile=routing_rules
        if result['mode'] in ('route','repair') and result['routed_drc']['unconnected'] and not repair_profile:
            run(worker+['dc-geometry',out],'completion-geometry.log')
            restore_project()
            geometry=json.loads((out/'dc-geometry.json').read_text())
            repair_profile={'net_limits':[{'net':n['name'],'min_width_mm':n['width']}
                for n in geometry['nets'].values() if n['name'] and n['width']>0]}
        if result['mode'] in ('route','repair') and result['routed_drc']['unconnected'] and not physical_errors:
            # Resolve measured owner cages before repeating the older grid
            # searches. Only the controller's final guarded candidate is adopted.
            from repair_native_congestion import coupled_completion
            save(stage='Completing measured routing obstructions under original board rules')
            result['coupled_completion']=coupled_completion(out,worker,kicad,repair_profile,min(timeout*2,360),protected)
            result['routed_drc']=summarize_drc(out/'routed-drc.json')
            if result['coupled_completion']['accepted']:
                result['selected_router']+=' + coupled-completion'
            result['router_trials'].append({'engine':'coupled-completion','drc':result['routed_drc'],
                'accepted':result['coupled_completion']['accepted'],'complete':result['coupled_completion']['complete']})
        if result['mode'] in ('route','repair'):
            grid_trials(('repair','repair-cleanup') if result['mode']=='repair' else ('repair','repair-cleanup','fresh'))
        if result['mode'] in ('route','repair') and result['routed_drc']['unconnected']:
            if any(result['routed_drc']['types'].get(t) for t in ('via_dangling','track_dangling')):
                from prune_native_routing import prune
                result['final_pruning']=prune(out,worker,kicad,prefix='final-prune',protected=protected,profile=repair_profile)
                result['routed_drc']=summarize_drc(out/'routed-drc.json')
            from repair_native_congestion import repair as repair_congestion
            save(stage='Rerouting local congestion while preserving supplied constraints')
            result['congestion_repair']=repair_congestion(out,worker,kicad,repair_profile,min(timeout,180),protected)
            result['routed_drc']=summarize_drc(out/'routed-drc.json')
            from stitch_native_planes import stitch
            save(stage='Checking disconnected ground-plane islands')
            result['plane_stitching']=stitch(out,worker,kicad,profile=repair_profile,protected=protected)
            result['routed_drc']=summarize_drc(out/'routed-drc.json')
            if result['routed_drc']['unconnected']:
                # Endpoint/blocker repair can expose passages hidden by raster
                # padding. Reuse the same native domains and acceptance gates.
                grid_trials(('repair-continuous',))
        if routing_check and result['mode'] in ('route','repair') and result['routed_drc']['total_findings']==0:
            # Re-route failing signal constraints once against final filled copper.
            initial_reference=routing_check(out)
            if not initial_reference['pass']:
                from routing_screen import nonregression as reference_nonregression
                d=out/'reference-repair';d.mkdir()
                copy_assets(source.parent, d)
                for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
                    if (out/('candidate'+suffix)).exists():shutil.copy2(out/('candidate'+suffix),d/('candidate'+suffix))
                trial={'accepted':False};result['reference_repair']=trial
                save(stage='Rerouting signals that fail the supplied routing constraints')
                try:
                    run(worker+['measure',d],'reference-repair/measure-before.log')
                    metrics=json.loads((d/'metrics.json').read_text())
                    failures=[r for r in initial_reference['rules'] if not r['pass']]
                    names={r['net'] for r in failures if 'reference_layer' in r or r.get('kind') in ('terminal_path','net_limits')}
                    paths={p['id']:p['net'] for p in routing_rules.get('paths',[])}
                    for group in routing_rules.get('matched_groups',[]):
                        if any(r.get('id')==group['id'] for r in failures):names.update(paths[p] for p in group['paths'])
                    names-=set(protected)
                    names-={n['name'] for n in metrics['nets'].values() if n['locked']}
                    if not names:raise ValueError('Failing reference nets are locked or protected')
                    routing_geometry(d,'reference-repair/geometry-before.log')
                    g=json.loads((d/'geometry.json').read_text());codes={code:n for code,n in g['nets'].items() if n['name'] in names}
                    widths={code:max([n['width']]+[t['width'] for t in g['tracks'] if t['net']==int(code)]) for code,n in codes.items()}
                    (d/'route-nets.json').write_text(json.dumps({'nets':list(map(int,codes)),'widths':widths}))
                    run(worker+['rip-nets',d],'reference-repair/remove.log')
                    routing_geometry(d,'reference-repair/geometry.log')
                    run([sys.executable,ROOT/'scripts/grid_route.py',d,'--seconds',min(timeout,30),'--step','.05'],'reference-repair/search.log',min(timeout,30)+10)
                    search=json.loads((d/'grid-route.json').read_text())
                    if search['failed']:raise ValueError('Reference repair search did not complete all selected nets')
                    run(worker+['apply-grid',d],'reference-repair/apply.log')
                    for suffix in ('.kicad_pro','.kicad_dru'):
                        if source.with_suffix(suffix).exists():shutil.copy2(source.with_suffix(suffix),d/('candidate'+suffix))
                    run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'routed-drc.json',d/'candidate.kicad_pcb'],'reference-repair/drc.log')
                    drc=summarize_drc(d/'routed-drc.json');checked=routing_check(d)
                    from optimize_kicad import unchanged_other_nets
                    preserved=route_audit(out/'original-normalized.kicad_pcb',d/'candidate.kicad_pcb')['pass']
                    untouched=unchanged_other_nets(out/'candidate.kicad_pcb',d/'candidate.kicad_pcb',names)
                    trial.update(nets=sorted(names),reference_pass=checked['pass'],drc=drc,structure_preserved=preserved,other_nets_unchanged=untouched)
                    if drc['total_findings'] or not preserved or not untouched or not reference_nonregression(initial_reference,checked):raise ValueError('Reference repair failed preservation, continuity or DRC checks')
                    for filename in ('candidate.kicad_pcb','routed-drc.json'):shutil.copy2(d/filename,out/filename)
                    result['routed_drc']=drc;result['selected_router']+=' + reference-repair';trial['accepted']=True
                except (ValueError,RuntimeError,subprocess.TimeoutExpired) as error:trial['reason']=str(error)
        if dc_check and result['mode'] in ('route','repair') and result['routed_drc']['total_findings']==0:
            routed_dc=dc_check(out)
            if not routed_dc['pass_limits'] and routed_dc.get('mesh_check',{}).get('pass',True) and routed_dc.get('rc_hold',{}).get('converged',True):
                from optimize_kicad import optimize
                save(stage='Repairing routed supply traces to meet supplied voltage limits')
                repair=out/'power-repair';repair.mkdir()
                copy_assets(source.parent, repair)
                for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
                    if (out/('candidate'+suffix)).exists():shutil.copy2(out/('candidate'+suffix),repair/('candidate'+suffix))
                run(worker+['inspect',repair],'power-repair/inspect.log')
                shutil.copy2(repair/'metrics.json',repair/'source-metrics.json')
                shutil.copy2(out/'routed-drc.json',repair/'source-drc.json')
                shutil.copy2(out/'dc-screen.json',out/'pre-repair-dc-screen.json')
                def repair_run(command,log,seconds=60):return run(command,'power-repair/'+log,seconds)
                # Reuse the same acceptance path; skip geometric search here.
                repair_routing=routing_check(out) if routing_check else None
                result['power_repair']=optimize(repair,source,worker,kicad,repair_run,save,audit_preservation,timeout,protected,0,profile['nets'],dc_check,routed_dc,routing_check,repair_routing,routing_rules)
                if result['power_repair']['power_repaired']:
                    shutil.copy2(repair/'candidate.kicad_pcb',out/'candidate.kicad_pcb')
                    shutil.copy2(repair/'routed-drc.json',out/'routed-drc.json')
                    result['routed_drc']=summarize_drc(out/'routed-drc.json')
                    result['selected_router']+=' + power-width'
                dc_before=routed_dc
        if result['mode'] in ('route','repair') and result['routed_drc']['errors']==0 and result['routed_drc']['unconnected']==0:
            # Reuse guarded optimization after routing: shorter legal copper can
            # also remove sharp filled-plane slivers without changing zone rules.
            save(stage='Shortening completed routes under the same acceptance gates')
            shortened=solve(out/'candidate.kicad_pcb',out/'route-shortening',timeout=min(timeout,60),
                kicad=kicad,mode='improve',protected=protected,attempts=8,
                dc_profile=dc_profile,routing_profile=routing_profile)
            result['route_shortening']={k:shortened[k] for k in ('state','review_pass','routed_drc','optimization','reference_planes','measured_contract','source_metrics','routed_metrics') if k in shortened}
            for name in ('routing-screen.json','measured-contract.json'):
                if (out/'route-shortening'/name).exists():shutil.copy2(out/'route-shortening'/name,out/('route-shortening-'+name))
            if shortened.get('review_pass'):
                for name in ('candidate.kicad_pcb','routed-drc.json','routed-inventory.json'):
                    if (out/'route-shortening'/name).exists():shutil.copy2(out/'route-shortening'/name,out/name)
                result['routed_drc']=summarize_drc(out/'routed-drc.json')
                result['selected_router']+=' + guarded-shortening'
        run(worker + ['measure', out], 'measure.log')
        restore_project()
        result['routed_metrics'] = json.loads((out / 'metrics.json').read_text())
        result['routed_inventory'] = result['routed_metrics']['inventory']
        identical_original=(out / 'candidate.kicad_pcb').read_bytes()==source.read_bytes()
        result['original_board_retained_exactly']=identical_original
        audit = ({'pass':True,'removed_fields':[],'added_fields':[]} if identical_original else
                 route_audit(out / 'original-normalized.kicad_pcb', out / 'candidate.kicad_pcb'))
        (out / 'preservation.json').write_text(json.dumps(audit, indent=2))
        result['preservation_pass'] = audit['pass']
        # Preserve locked copper exactly, independent of native import behavior.
        before_tree = sexpr((out / 'original-normalized.kicad_pcb').read_text())
        after_tree = sexpr((out / 'candidate.kicad_pcb').read_text())
        locked = [v for v in before_tree if isinstance(v, list) and v and v[0] in ('segment','arc','via') and ['locked','yes'] in v]
        result['locked_copper_preserved'] = identical_original or all(v in after_tree for v in locked)
        result['project_rules_preserved'] = all((out / ('candidate'+suffix)).read_bytes() == source.with_suffix(suffix).read_bytes()
            for suffix in ('.kicad_pro','.kicad_dru') if source.with_suffix(suffix).exists())
        result['source_unchanged'] = all(hashlib.sha256((source.parent / name).read_bytes()).hexdigest() == digest for name,digest in copied.items())
        if result['mode']=='repair':
            from optimize_kicad import unchanged_other_nets
            guarded=set(protected)|{n['name'] for n in result['source_metrics']['nets'].values() if n.get('locked')}
            editable_names={n['name'] for n in result['source_metrics']['nets'].values()}-guarded
            result['protected_copper_nets']=sorted(guarded)
            result['protected_copper_preserved']=identical_original or unchanged_other_nets(out/'original-normalized.kicad_pcb',out/'candidate.kicad_pcb',editable_names)
        passed = (audit['pass'] and result['locked_copper_preserved'] and result['project_rules_preserved']
                  and result['source_unchanged'] and result.get('protected_copper_preserved',True) and result['routed_drc']['total_findings'] == 0)
        save(stage='Checking all first-source connectivity obligations')
        run(worker+['native-connectivity',out],'restoration-connectivity.log')
        final_contacts=json.loads((out/'native-connectivity.json').read_text())
        if final_contacts['source_sha256']!=hashlib.sha256((out/'candidate.kicad_pcb').read_bytes()).hexdigest():
            raise ValueError('Final native contacts belong to a stale candidate')
        result['restoration']=audit_restoration(baseline,final_contacts,protected)
        (out/'restoration.json').write_text(json.dumps(result['restoration'],indent=2)+'\n')
        passed=passed and result['restoration']['pass']
        if dc_check:
            save(stage='Checking final loaded voltages against supplied limits')
            final_dc=dc_check(out)
            from dc_screen import nonregression
            unchanged=hashlib.sha256(dc_profile.read_bytes()).hexdigest()==profile_hash
            dc_pass=unchanged and final_dc['pass_limits'] and (dc_before is None or nonregression(dc_before,final_dc))
            result['dc_screen'].update(pass_limits=final_dc['pass_limits'],pass_nonregression=dc_before is None or nonregression(dc_before,final_dc),pass_all=dc_pass,profile_unchanged=unchanged,return_path_modelled=final_dc['return_path_modelled'],mesh_check=final_dc.get('mesh_check'),loads=final_dc['loads'],loss_w=final_dc['loss_w'],violations=final_dc['violations'],before_loads=None if dc_before is None else dc_before['loads'])
            if 'rc_hold' in final_dc:
                from rc_screen import summary as rc_summary
                result['dc_screen']['rc_hold']=rc_summary(final_dc['rc_hold'])
                result['dc_screen']['scope']='Explicit DC and passive RC current-hold scenarios; no regulator-loop, inductive, thermal or signal-integrity validation'
            passed=passed and dc_pass
        if routing_check:
            final_routing=routing_check(out)
            unchanged=hashlib.sha256(routing_profile.read_bytes()).hexdigest()==routing_hash
            result['reference_planes']=dict(final_routing,profile_sha256=routing_hash,profile_unchanged=unchanged)
            passed=passed and unchanged and final_routing['pass']
        # Native KiCad plots retain custom pads, arbitrary outlines and all copper layers.
        save(stage='Rendering native board preview')
        run([kicad, 'pcb', 'export', 'svg', '--mode-multi', '--layers', 'F.Cu,B.Cu,Edge.Cuts', '--page-size-mode', '2',
             '--exclude-drawing-sheet', '-o', str(out / 'preview') + '/', out / 'candidate.kicad_pcb'], 'preview.log')
        run([kicad, 'pcb', 'export', 'svg', '--mode-single', '--layers', '*.Cu,Edge.Cuts', '--page-size-mode', '2',
             '--exclude-drawing-sheet', '-o', out / 'preview/board.svg', out / 'candidate.kicad_pcb'], 'composite-preview.log')
        if passed:
            selected_export_sha256 = hashlib.sha256((out/'candidate.kicad_pcb').read_bytes()).hexdigest()
            run(worker + ['gerber-export', out], 'gerbers.log')
            gerber_precision = json.loads((out/'native-audit/gerber-precision.json').read_text())
            if gerber_precision['source_board_sha256']!=selected_export_sha256:
                raise ValueError('Native Gerber precision report differs from the selected board')
            result['native_gerber_export'] = {key:value for key,value in gerber_precision.items() if key!='files'}
            raw_drill = out/'native-audit/raw-drill'
            raw_drill.mkdir(parents=True,exist_ok=True)
            run([kicad, 'pcb', 'export', 'drill', '--excellon-separate-th', '-o', str(raw_drill) + '/', out / 'candidate.kicad_pcb'], 'drill.log')
            run(worker + ['assembly-export', out], 'assembly-export.log')
            assembly = json.loads((out/'assembly/native-assembly-frames.json').read_text())
            native_drills = json.loads((out/'native-audit/native-drill-holes.json').read_text())
            if assembly['source_board_sha256']!=selected_export_sha256 or hashlib.sha256((out/'candidate.kicad_pcb').read_bytes()).hexdigest()!=selected_export_sha256:
                raise ValueError('Selected native board changed during engineering export')
            from native_drill_precision import restore_tool_precision
            precision = restore_tool_precision(raw_drill,native_drills,selected_export_sha256,out/'cam')
            if hashlib.sha256((out/'candidate.kicad_pcb').read_bytes()).hexdigest()!=selected_export_sha256:
                raise ValueError('Selected native board changed during drill precision restoration')
            for files in precision['files']:
                for key in ('raw_file','restored_file'):files[key]=str(Path(files[key]).relative_to(out))
            (out/'native-audit/drill-precision.json').write_text(json.dumps(precision,indent=2)+'\n')
            result['native_drill_export'] = {key:value for key,value in precision.items() if key not in ('records','files')}
            result['assembly_export'] = {key:value for key,value in assembly.items() if key!='frames'}
            result['qualification_stage'] = 'engineering_review'
            result['manufacturing_release_approved'] = False
            if not (out/'original.kicad_pcb').exists():shutil.copy2(source,out/'original.kicad_pcb')
        save(state='done', stage='Checks pass — engineering review' if passed else 'Blocked — inspect findings', review_pass=passed,
             output_board=str(out / 'candidate.kicad_pcb'))
    except Exception as e:
        save(state='failed', stage=str(e))
    finally:
        if mode in ('repair','improve') and (out/'original.kicad_pcb').exists():
            with zipfile.ZipFile(out/'original.zip','w',zipfile.ZIP_DEFLATED) as z:
                z.write(out/'original.kicad_pcb','original.kicad_pcb')
                for suffix in ('.kicad_pro','.kicad_dru','.dc.json','.routing.json'):
                    if (out/('candidate'+suffix)).exists():z.write(out/('candidate'+suffix),'original'+suffix)
                for name in project_assets:
                    z.write(out/name,name)
                if (out/'source-design.json').is_file():z.write(out/'source-design.json','source-design.json')
            save(original_archive=True)
        from review_ui import native_review
        archive_files = []
        if result['review_pass']:
            from native_restoration import validate as validate_restoration
            original_files=set(validate_restoration(out/'restoration-source')['files'])|{'manifest.json'}
            for p in sorted(out.rglob('*')):
                if p.is_file() and (p.is_relative_to(out/'restoration-source') and str(p.relative_to(out/'restoration-source'))in original_files or p.name in ('restoration.json','native-connectivity.json') and p.parent==out or str(p.relative_to(out)) in project_assets or p.parent in (out/'cam',out/'assembly') or
                    p.parent==out/'native-audit/raw-drill' and p.suffix=='.drl' or
                    p in (out/'native-audit/native-drill-holes.json',out/'native-audit/drill-precision.json',out/'native-audit/gerber-precision.json') or (p.parent == out and p.name in
                    ('status.json','source-manifest.json','routed-drc.json','source-drc.json','preservation.json','source-design.json','original.kicad_pcb','original.zip','candidate.kicad_pcb','candidate.kicad_pro','candidate.kicad_dru','candidate.dc.json','dc-screen.json','source-dc-screen.json','pre-repair-dc-screen.json','candidate.routing.json','routing-screen.json','source-routing-screen.json','measured-contract.json','generated-zone-removals.json','prepared-preservation.json','dsn-native-obstacles.json','native-obstacle-checks.json','route-shortening-routing-screen.json','route-shortening-measured-contract.json'))):
                    archive_files.append(p)
            roles = {str(p.relative_to(out)): ('immutable_first_source_and_native_contacts' if p.is_relative_to(out/'restoration-source') else
                     'native_derived_tool_restored_CAM' if p.parent==out/'cam' and p.suffix=='.drl' else
                     'native_CAM' if p.parent==out/'cam' else
                     'raw_native_Excellon_audit_only' if p.parent==out/'native-audit/raw-drill' else
                     'exact_native_drill_geometry' if p.name=='native-drill-holes.json' and p.parent==out/'native-audit' else
                     'native_drill_tool_restoration_verification' if p.name=='drill-precision.json' and p.parent==out/'native-audit' else
                     'native_Gerber_curve_precision_verification' if p.name=='gerber-precision.json' and p.parent==out/'native-audit' else
                     'native_assembly_UNQUALIFIED' if p.parent==out/'assembly' else
                     'provided_project_asset' if str(p.relative_to(out)) in project_assets else 'native_design_or_verification')
                     for p in archive_files}
            roles.update({'READ_FIRST.md':'engineering_scope_and_qualification_holds',
                          'HANDOFF.json':'file_roles_and_qualification_stage','manifest.json':'SHA256_file_manifest'})
            handoff = {'qualification_stage':'engineering_review','physical_validation':False,
                       'manufacturing_release_approved':False,'firmware_validated':False,
                       'procurement_qualified':False,'assembly_machine_orientation_qualified':False,
                       'native_enabled_findings':result['routed_drc']['total_findings'],
                       'source_unchanged':result['source_unchanged'],'project_rules_preserved':result['project_rules_preserved'],
                       'original_connectivity_restoration':result['restoration'],
                       'selected_files':{'native_board':'candidate.kicad_pcb','native_project':'candidate.kicad_pro',
                         'original_native_board':'original.kicad_pcb',
                         'first_uploaded_native_board':'restoration-source/candidate.kicad_pcb',
                         'first_uploaded_native_project':'restoration-source/candidate.kicad_pro',
                         'immutable_first_source_manifest':'restoration-source/manifest.json',
                         'first_uploaded_assets':sorted('restoration-source/'+name for name in original_files),
                         'source_circuit_JSON':'source-design.json' if (out/'source-design.json').exists() else None,
                         'populated_BOM':'assembly/BOM.csv','DNP_BOM':'assembly/BOM-DNP.csv',
                         'populated_positions_UNQUALIFIED':'assembly/positions.csv',
                         'DNP_positions_UNQUALIFIED':'assembly/positions-DNP.csv',
                         'all_native_assembly_frames':'assembly/native-assembly-frames.json',
                         'native_CAM':sorted(str(p.relative_to(out)) for p in archive_files if p.parent==out/'cam'),
                         'exact_native_drill_metadata':'native-audit/native-drill-holes.json',
                         'native_Gerber_precision_verification':'native-audit/gerber-precision.json',
                         'drill_precision_verification':'native-audit/drill-precision.json',
                         'raw_native_drill_audit_only':sorted(str(p.relative_to(out)) for p in archive_files if p.parent==out/'native-audit/raw-drill'),
                         'provided_project_assets':project_assets},'file_roles':roles,
                       'native_drill_export':result['native_drill_export'],
                       'native_gerber_export':result['native_gerber_export'],
                       'qualification_holds':['Exact procurement selections and land patterns are unqualified.',
                         'Native anchor coordinates and angles require assembler package/origin/rotation conversion.',
                         'Firmware, electrical behaviour, thermal/mechanical fit and physical assembly are unvalidated.',
                         'Passing enabled native checks and native CAM exports do not approve manufacturing release.']}
            (out/'HANDOFF.json').write_text(json.dumps(handoff,indent=2)+'\n')
            (out/'READ_FIRST.md').write_text(
                '# Native engineering handoff\n\n'
                'This package contains one matched native design revision for engineering review. '
                'Manufacturing release, physical operation, firmware and procurement are unqualified.\n\n'
                'HANDOFF.json maps every archived file and identifies the native board/project, immediate input board, '
                'immutable first-uploaded board/project/assets and native contacts, circuit JSON when supplied, native CAM and assembly tables. '
                'original.kicad_pcb is the immediate input; restoration-source/ retains the first source across continuations. '
                'manifest.json gives the SHA-256 of every archived file except itself. '
                'Enabled native checks preserve the project\'s recorded exclusions.\n\n'
                'Gerbers include copper, mask, paste, silkscreen and outline; plated and unplated drill files are separate. '
                'Native plotting uses curve approximation at most 1 micrometre on a temporary loaded board, '
                'restores the original setting without saving or refilling, and retains the source/project/rules/fills. '
                'native-audit/gerber-precision.json records source and actual plotting approximation and output hashes. '
                'Final .drl files are native-derived tool-restored CAM, not byte-exact raw native exports. '
                'Only tool-diameter declarations are restored from exact native integer hole metadata; '
                'native-audit/raw-drill retains untouched native CLI drill files. Native coordinates and slot commands '
                'retain KiCad rounding. native-audit/native-drill-holes.json records the selected board SHA256, '
                'plating, layer spans and effective slot endpoints; native-audit/drill-precision.json records '
                'unique feature/tool matching and raw/restored hashes. '
                'Use the provided native project/rules with the PCB. This package does not establish circuit performance '
                'or independently qualify the CAM against manufacturing requirements.\n\n'
                'Assembly tables use millimetres from the native board origin, +X right and +Y down, '
                'and raw native KiCad orientation degrees. All machine orientations are UNQUALIFIED. '
                'Populated tables honor native BOM/position exclusions; DNP tables are review-only and retain explicit flags. '
                'The frame JSON retains every footprint, including excluded features. Manufacturer and MPN fields '
                'contain only literal native properties with those names; Values are never inferred purchasing codes.\n')
            archive_files += [out/'READ_FIRST.md',out/'HANDOFF.json']
        (out / 'report.html').write_text(native_review(result, out))
        manifest = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (archive_files if result['review_pass'] else out.rglob('*')) if p.is_file()}
        (out / 'manifest.json').write_text(json.dumps(manifest, indent=2))
        if result['review_pass']:
            with zipfile.ZipFile(out / 'prototype.zip', 'w', zipfile.ZIP_DEFLATED) as z:
                for p in sorted(archive_files+[out/'manifest.json']):z.write(p,str(p.relative_to(out)))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('board', type=Path)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--timeout', type=int, default=180)
    p.add_argument('--passes', type=int, default=30)
    p.add_argument('--mode', choices=['route','repair','improve'],default='route')
    p.add_argument('--protect-net',action='append',default=[])
    p.add_argument('--allow-net',action='append',default=None,help='Only these exact net names may change; repeat for multiple nets')
    p.add_argument('--attempts',type=int,default=8)
    p.add_argument('--dc-profile',type=Path,help='Explicit loads, voltage limits and copper assumptions; failed DC or optional RC checks prevent a prototype archive')
    p.add_argument('--routing-profile',type=Path,help='Explicit reference-plane, terminal-path, matching and net rules')
    p.add_argument('--plane-seed',action='store_true',help='Use zone outlines as routing search hints; final native filled-copper connectivity remains mandatory')
    args = p.parse_args()
    if not 1 <= args.timeout <= 3600 or not 1 <= args.passes <= 100:
        p.error('timeout must be 1–3600 seconds and passes 1–100')
    if args.allow_net is not None and args.mode!='improve':p.error('--allow-net requires --mode improve')
    result = solve(args.board, args.out, args.timeout, args.passes, mode=args.mode, protected=args.protect_net, attempts=args.attempts, allowed=args.allow_net, dc_profile=args.dc_profile,routing_profile=args.routing_profile,plane_seed=args.plane_seed)
    print(json.dumps(result, indent=2))
    return 0 if result['review_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
