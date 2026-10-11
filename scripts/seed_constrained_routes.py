"""Generate critical copper first; reserve only signals passing the supplied checks."""
import json
import shutil
from native_project import copy_assets
from grid_route import route
from shortcut_route import shortcut
from signal_screen import screen
from dsn_constraints import add_seed
from match_route import tune


def seed(out,worker,kicad,run,profile,seconds=120):
    d=out/'constraint-seed';d.mkdir()
    copy_assets(out,d)
    for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
        if (out/('candidate'+suffix)).exists():shutil.copy2(out/('candidate'+suffix),d/('candidate'+suffix))
    def native(mode,label=None):run(worker+[mode,d],'constraint-seed/'+(label or mode)+'.log')
    def geometry(label):
        native('dc-geometry',label)
        g=json.loads((d/'dc-geometry.json').read_text());g['routing_profile']=profile
        return g
    g=geometry('before');original=g
    existing={t['net'] for t in g['tracks']+g['vias']}
    path_names={p['net'] for p in profile.get('paths',[])}
    # A zero-via or single out-and-back budget also needs reserved escape space.
    path_names.update(n['net'] for n in profile.get('net_limits',[]) if n.get('max_vias',float('inf'))<=2)
    codes=[int(c) for c,s in g['nets'].items() if s['name'] in path_names and int(c) not in existing]
    if not codes:return {'nets':[],'reason':'No route-free constrained signal nets'}
    result=route(g,step=.025,seconds=seconds,only_nets=codes)
    (d/'grid-route.json').write_text(json.dumps(result));(d/'initial-search.json').write_text(json.dumps(result))
    native('apply-grid');g=geometry('routed')
    preliminary=screen(g,profile);shortcuts=[]
    # Remove raster detours on overlong paths before reserving their geometry.
    failed={r['net'] for r in preliminary if r['kind']=='terminal_path' and not r['pass']}
    for name in sorted(failed):
        code=next(int(c) for c,s in g['nets'].items() if s['name']==name)
        change=shortcut(g,code)
        if not change['remove']:continue
        (d/'grid-route.json').write_text(json.dumps(change));native('apply-grid','shortcut-'+str(code))
        g=geometry('measure-'+str(code));shortcuts.append(name)
    matching=[]
    for index in range(len(profile.get('matched_groups',[]))):
        change=tune(g,profile)
        if not change['remove']:break
        (d/'grid-route.json').write_text(json.dumps(change));native('apply-grid','match-'+str(index))
        g=geometry('match-measure-'+str(index));matching.append(change['matched_group'])
    reports=screen(g,profile)
    bad={r['net'] for r in reports if not r['pass'] and r['kind']!='path_matching'}
    paths={p['id']:p['net'] for p in profile.get('paths',[])}
    for group in profile.get('matched_groups',[]):
        if any(r.get('id')==group['id'] and not r['pass'] for r in reports):bad.update(paths[p] for p in group['paths'])
    selected={original['nets'][str(c)]['name'] for c in result['completed_nets']}-bad
    run([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'drc.json',d/'candidate.kicad_pcb'],'constraint-seed/drc.log')
    # Remaining board connections are deliberately unfinished. Do not reserve
    # copper implicated in a geometric finding; final full-board DRC is mandatory.
    drc=json.loads((d/'drc.json').read_text())
    implicated={name for name in selected for issue in drc.get('violations',[]) for item in issue.get('items',[]) if '['+name+']' in item.get('description','')}
    selected-=implicated
    (d/'routing-screen.json').write_text(json.dumps({'rules':reports,'pass':all(r['pass'] for r in reports)},indent=2))
    dsn,summary=add_seed((out/'input.dsn').read_text(),g,selected)
    (out/'input-before-seed.dsn').write_bytes((out/'input.dsn').read_bytes());(out/'input.dsn').write_text(dsn)
    summary.update(shortcut_nets=shortcuts,matched_groups=matching,not_reserved=sorted(path_names-selected),geometry_rejected=sorted(implicated),
        search_seconds=result['elapsed_seconds'],manual_trace_edits=0,source_copper_reused=False)
    (d/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary
