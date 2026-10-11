"""Prune native dangling copper, retaining only independently checked improvements."""
import json
from pathlib import Path
import shutil
import subprocess
import hashlib
from native_project import copy_assets


def consolidate_vias(directory,worker,kicad,via_ids,rounds,prefix,protected,profile):
    """Stage a complete drill/chain cleanup before touching the accepted board."""
    from solve_kicad import audit_preservation,summarize_drc
    from shortcut_route import consolidate_dangling_vias
    from optimize_kicad import unchanged_other_nets,filled_planes_preserved,findings
    from routing_screen import screen,nonregression
    from native_restoration import NAME,audit,validate
    directory=Path(directory);d=directory/(prefix+'-via-consolidation');d.mkdir();copy_assets(directory,d)
    rules={}
    for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
        source=directory/('candidate'+suffix)
        if source.exists():
            shutil.copy2(source,d/source.name)
            if suffix!='.kicad_pcb':rules[source.name]=source.read_bytes()
    before=json.loads((directory/'routed-drc.json').read_text());row={'method':'dangling-via chain consolidation','accepted':False}
    baseline=directory/NAME
    try:
        validate(baseline)
        with (d/'consolidation.log').open('w') as log:
            def native(mode):
                try:subprocess.run(list(map(str,worker+[mode,d])),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
                finally:
                    for name,data in rules.items():(d/name).write_bytes(data)
            def drc():
                try:subprocess.run(list(map(str,[kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors',
                    '-o',d/'routed-drc.json',d/'candidate.kicad_pcb'])),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
                finally:
                    for name,data in rules.items():(d/name).write_bytes(data)
            native('dc-geometry');geometry=json.loads((d/'dc-geometry.json').read_text())
            expected=hashlib.sha256((directory/'candidate.kicad_pcb').read_bytes()).hexdigest()
            if geometry.get('source_board_sha256')!=expected:raise ValueError('Cleanup geometry belongs to a different board')
            geometry['routing_profile']=profile
            proposal=consolidate_dangling_vias(geometry,via_ids,protected)
            (d/'cleanup-proposal.json').write_text(json.dumps(proposal,indent=2)+'\n')
            if not proposal['proposed_removed_via_uuids']:raise ValueError('No eligible measured dangling vias')
            (d/'grid-route.json').write_text(json.dumps(proposal));native('apply-grid');drc()
            # Track-only tail pruning is part of this private candidate. The
            # immutable first source is checked after the complete operation.
            row['tail_prune']=prune(d,worker,kicad,rounds=rounds,prefix='tail-prune',protected=protected)
            native('native-connectivity');contacts=json.loads((d/'native-connectivity.json').read_text())
            if contacts.get('source_sha256')!=hashlib.sha256((d/'candidate.kicad_pcb').read_bytes()).hexdigest():
                raise ValueError('Cleanup native contacts belong to a different board')
            restoration=audit(baseline,contacts,protected)
            (d/'restoration.json').write_text(json.dumps(restoration,indent=2)+'\n')
            native('dc-geometry');after_geometry=json.loads((d/'dc-geometry.json').read_text())
            if after_geometry.get('source_board_sha256')!=hashlib.sha256((d/'candidate.kicad_pcb').read_bytes()).hexdigest():
                raise ValueError('Final cleanup geometry belongs to a different board')
        names={geometry['nets'][str(v['net'])]['name'] for v in geometry['vias']
               if v['uuid'] in proposal['proposed_removed_via_uuids']}
        structure=audit_preservation(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb')
        foreign=unchanged_other_nets(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb',names)
        stable={n['name'] for n in geometry['nets'].values() if n['name'] not in names} if foreign else set()
        after=json.loads((d/'routed-drc.json').read_text())
        findings_ok=not(findings(after,stable)-findings(before,stable))
        planes=filled_planes_preserved(geometry,after_geometry)
        profile_ok=not profile or nonregression(screen(geometry,profile),screen(after_geometry,profile))
        project_exact=all((d/name).read_bytes()==data for name,data in rules.items())
        row.update(restoration=restoration,source_structure_pass=structure['pass'],other_nets_unchanged=foreign,
            native_findings_nonregression=findings_ok,profile_nonregression=profile_ok,
            filled_planes_preserved=planes['pass'],project_exact=project_exact,
            before=summarize_drc(directory/'routed-drc.json'),after=summarize_drc(d/'routed-drc.json'),chains=proposal['chains'])
        row['accepted']=bool(restoration['pass'] and structure['pass'] and foreign and findings_ok
                             and profile_ok and planes['pass'] and project_exact)
        if row['accepted']:
            for name in ('candidate.kicad_pcb','routed-drc.json'):shutil.copy2(d/name,directory/name)
    except (ValueError,RuntimeError,OSError,subprocess.SubprocessError) as error:row['error']=str(error)
    (d/'consolidation.json').write_text(json.dumps(row,indent=2)+'\n')
    return row


def prune(directory,worker,kicad,rounds=6,prefix='prune',protected=(),profile=None):
    from solve_kicad import audit_preservation,summarize_drc
    directory=Path(directory);trials=[]
    for index in range(rounds):
        report=json.loads((directory/'routed-drc.json').read_text())
        vias={i['uuid'] for v in report.get('violations',[]) if v['type']=='via_dangling' for i in v.get('items',[])}
        if vias:
            trial=consolidate_vias(directory,worker,kicad,vias,rounds-index,f'{prefix}-{index:02}',protected,profile)
            trials.append(trial)
            if trial['accepted']:continue
            # An unsupported via cleanup cannot expose source copper through a
            # later plain deletion. Independently dangling tracks can still go.
        dangling=[v for v in report.get('violations',[]) if v['type']=='track_dangling']
        if not dangling:break
        d=directory/f'{prefix}-{index:02}';d.mkdir();copy_assets(directory,d)
        for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
            if (directory/('candidate'+suffix)).exists():shutil.copy2(directory/('candidate'+suffix),d/('candidate'+suffix))
        (d/'input-drc.json').write_text(json.dumps({'violations':dangling,'unconnected_items':[]}))
        (d/'route-nets.json').write_text(json.dumps({'protected_nets':list(protected)}))
        (d/'grid-route.json').write_text(json.dumps({'tracks':[],'vias':[]}))
        def restore_rules():
            for suffix in ('.kicad_pro','.kicad_dru'):
                if (directory/('candidate'+suffix)).exists():shutil.copy2(directory/('candidate'+suffix),d/('candidate'+suffix))
        with (d/'prune.log').open('w') as log:
            for mode in ('prepare-repair','apply-grid'):
                try:subprocess.run(list(map(str,worker+[mode,d])),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
                finally:restore_rules()
            try:subprocess.run(list(map(str,[kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'routed-drc.json',d/'candidate.kicad_pcb'])),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
            finally:restore_rules()
        before=summarize_drc(directory/'routed-drc.json');after=summarize_drc(d/'routed-drc.json')
        audit=audit_preservation(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb')
        source_pass=True
        from native_restoration import NAME,audit as source_audit
        if (directory/NAME).exists():
            with (d/'source-contact-check.log').open('w') as log:
                try:subprocess.run(list(map(str,worker+['native-connectivity',d])),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=30)
                finally:restore_rules()
            contacts=json.loads((d/'native-connectivity.json').read_text())
            source_pass=contacts.get('source_sha256')==hashlib.sha256((d/'candidate.kicad_pcb').read_bytes()).hexdigest()
            restoration=source_audit(directory/NAME,contacts,protected)
            (d/'restoration.json').write_text(json.dumps(restoration,indent=2)+'\n')
            source_pass &= restoration['pass']
        accepted=audit['pass'] and after['errors']<=before['errors'] and after['unconnected']<=before['unconnected'] and after['total_findings']<=before['total_findings'] and json.loads((d/'repair.json').read_text())['removed_unlocked_copper']>0
        accepted &= source_pass
        trials.append({'before':before,'after':after,'accepted':accepted,'preservation_pass':audit['pass'],'source_restoration_pass':source_pass})
        if not accepted:break
        for name in ('candidate.kicad_pcb','routed-drc.json'):shutil.copy2(d/name,directory/name)
    (directory/('pruning.json' if prefix=='prune' else prefix+'-summary.json')).write_text(json.dumps(trials,indent=2)+'\n');return trials
