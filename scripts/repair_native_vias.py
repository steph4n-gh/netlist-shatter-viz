"""Repair local drill-spacing conflicts without ripping up complete routed nets."""
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import time
from native_project import copy_assets


def repair(directory,worker,kicad,seconds=90):
    directory=Path(directory);started=time.perf_counter();trials=[];changed=False
    original=json.loads((directory/'routed-drc.json').read_text());best=original
    def count(report):return len(report.get('violations',[]))+len(report.get('unconnected_items',[]))+len(report.get('schematic_parity',[]))
    def shortfall(report):
        total=0
        for issue in report.get("violations",[]):
            if issue.get("type")!="hole_to_hole":continue
            values=re.search(r"min ([0-9.]+) mm; actual ([0-9.]+) mm",issue.get("description",""))
            if values:total+=max(0,float(values[1])-float(values[2]))
        return total
    def other_findings(report):
        return sorted(json.dumps(v,sort_keys=True) for v in report.get("violations",[]) if v.get("type")!="hole_to_hole")
    for round_index in range(24):
        issues=[v for v in best.get('violations',[]) if v['type']=='hole_to_hole']
        if not issues or time.perf_counter()-started>=seconds:break
        accepted=False
        for issue in issues:
            values=re.search(r'min ([0-9.]+) mm; actual ([0-9.]+) mm',issue.get('description',''))
            distance=max(.005,float(values[1])-float(values[2])+.003) if values else .1
            items=issue.get('items',[])
            if len(items)!=2:continue
            for index,item in enumerate(items):
                if not item.get('description','').startswith('Via '):continue
                p=item['pos'];q=items[1-index]['pos'];angle=math.atan2(p['y']-q['y'],p['x']-q['x'])
                for rotation,paired,jog in ((r,pair,j) for r in (0,math.pi/6,-math.pi/6,math.pi/3,-math.pi/3) for pair in (True,False) for j in (True,False)):
                    if paired and (index or not items[1-index].get("description","").startswith("Via ")):continue
                    if time.perf_counter()-started>=seconds:break
                    # Compensate a rotated move so its separation-axis projection
                    # still fixes the measured shortfall.
                    radius=distance/max(.5,math.cos(rotation))/(2 if paired else 1)
                    at=[round(p['x']+math.cos(angle+rotation)*radius,6),round(p['y']+math.sin(angle+rotation)*radius,6)]
                    d=directory/f'via-repair-{len(trials):03}';d.mkdir();copy_assets(directory,d)
                    for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
                        if (directory/('candidate'+suffix)).exists():shutil.copy2(directory/('candidate'+suffix),d/('candidate'+suffix))
                    moves=[{'uuid':item['uuid'],'at':at,'jog':jog}]
                    if paired:moves.append({'uuid':items[1-index]['uuid'],'jog':jog,'at':[round(q['x']-(at[0]-p['x']),6),round(q['y']-(at[1]-p['y']),6)]})
                    (d/'via-move.json').write_text(json.dumps({'moves':moves}))
                    trial={'directory':d.name,'uuid':item['uuid'],'at':at,'paired':paired,'jog':jog,'accepted':False};trials.append(trial)
                    try:
                        with (d/'apply.log').open('w') as log:subprocess.run(list(map(str,worker+['move-via',d])),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=15)
                        for suffix in ('.kicad_pro','.kicad_dru'):
                            if (directory/('candidate'+suffix)).exists():shutil.copy2(directory/('candidate'+suffix),d/('candidate'+suffix))
                        with (d/'drc.log').open('w') as log:subprocess.run(list(map(str,[kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'routed-drc.json',d/'candidate.kicad_pcb'])),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=15)
                        report=json.loads((d/'routed-drc.json').read_text())
                        trial['findings']=count(report)
                        progress=count(report)<count(best) or (count(report)==count(best) and other_findings(report)==other_findings(best) and shortfall(report)<shortfall(best)-.001)
                        if progress and len(report.get('unconnected_items',[]))<=len(best.get('unconnected_items',[])):
                            from solve_kicad import audit_preservation
                            if not audit_preservation(directory/'candidate.kicad_pcb',d/'candidate.kicad_pcb')['pass']:raise ValueError('Via repair changed non-routing structure')
                            for name in ('candidate.kicad_pcb','routed-drc.json'):shutil.copy2(d/name,directory/name)
                            best=report;changed=True;accepted=True;trial['accepted']=True;break
                    except (ValueError,subprocess.SubprocessError) as error:trial['error']=str(error)
                if accepted:break
            if accepted:break
        if not accepted:break
    result={'changed':changed,'before_findings':count(original),'after_findings':count(best),'trials':trials,'elapsed_seconds':time.perf_counter()-started}
    (directory/'via-repair.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
