#!/usr/bin/env python3
"""Create an offline board inspector from the exported model and measured reports."""
import html,json,sys
from pathlib import Path
from review_ui import navigation

def render(d, retained=False):
    d=Path(d);b=json.loads((d/'board.json').read_text());r=json.loads((d/'report.json').read_text())
    v=json.loads((d/'verification.json').read_text()) if (d/'verification.json').exists() else {}
    handoff=json.loads((d/'HANDOFF.json').read_text()) if (d/'HANDOFF.json').is_file() else None
    assembly=json.loads((d/'assembly/native-assembly-frames.json').read_text()) if handoff else None
    el=[]
    for t in b['tracks']:
        a,z=t['start'],t['end'];net=html.escape(b['net_names'].get(str(t['net']),''))
        el.append(f'<line class="{t["layer"]}" x1="{a["x"]}" y1="{a["y"]}" x2="{z["x"]}" y2="{z["y"]}" stroke-width="{t["width"]}" stroke-linecap="round"><title>{net}</title></line>')
    for via in b['vias']:
        el.append(f'<g class="both-copper"><circle class="via" cx="{via["at"]["x"]}" cy="{via["at"]["y"]}" r="{via["size"]/2}"/><circle fill="#0b1d21" cx="{via["at"]["x"]}" cy="{via["at"]["y"]}" r="{via["drill"]/2}"/></g>')
    for f in b['footprints']:
        ref=html.escape(f['reference']);x,y=f['position']['x'],f['position']['y']
        el.append(f'<g class="part" tabindex="0" data-ref="{ref}" transform="translate({x},{y}) rotate({f["rotation"]})"><title>{ref} · {html.escape(f["value"])}</title>')
        if f['pads']:
            bounds=[]
            for p in f['pads']:
                shape=p['shape'];q=p['relative_pos']
                size=next(iter(shape.values()))
                w=h=size['diameter'] if 'Circle' in shape else None
                if w is None:w,h=size['width'],size['height']
                bounds.append((q['x']-w/2,q['y']-h/2,q['x']+w/2,q['y']+h/2))
            left=min(v[0] for v in bounds);top=min(v[1] for v in bounds)
            right=max(v[2] for v in bounds);bottom_edge=max(v[3] for v in bounds)
            # A component's center is often empty SVG space between its pads.
            # Give the inspector a hit area without adding displayed copper.
            el.append(f'<rect x="{left}" y="{top}" width="{right-left}" height="{bottom_edge-top}" fill="transparent" stroke="none" pointer-events="all"/>')
        for p in f['pads']:
            shape=p['shape'];x,y=p['relative_pos']['x'],p['relative_pos']['y'];title=f'<title>{ref}.{html.escape(p["pin_number"])} · {html.escape(b["net_names"].get(str(p["net"]),"Unassigned"))}</title>'
            layer=p['pad_type'].get('Smd',{}).get('layer','both-copper')
            el.append(f'<g class="{layer}">')
            if 'Circle' in shape:el.append(f'<circle class="pad" cx="{x}" cy="{y}" r="{shape["Circle"]["diameter"]/2}">{title}</circle>')
            else:
                q=next(iter(shape.values()));w,h=q['width'],q['height'];radius=min(w,h)/2 if 'Obround' in shape else 0
                el.append(f'<rect class="pad" x="{x-w/2}" y="{y-h/2}" width="{w}" height="{h}" rx="{radius}">{title}</rect>')
            if 'ThroughHole' in p['pad_type']:el.append(f'<circle fill="#0b1d21" cx="{x}" cy="{y}" r="{p["pad_type"]["ThroughHole"]["drill"]/2}"/>')
            el.append('</g>')
        el.append(f'<text class="label" x="0" y="-1.3">{ref}</text></g>')
    payload=json.dumps(b).replace('<','\\u003c')
    issues=[f'{i["component"]}: {i["message"]}' for i in r['electrical_issues']]+r['geometry_issues']
    status=('Engineering checks pass' if v.get('review_pass') else
            'Routing needs attention' if v.get('kicad_unconnected',0) else
            'Layout checks need attention' if v else 'Independent checks not yet run')
    artifacts=[('F_Cu.svg','Top copper'),('B_Cu.svg','Bottom copper'),('report.json','Design report'),
               ('verification.json','Independent checks'),('BOM.csv','Engineering parts list')]
    scope=''
    count=f"{len(b['footprints'])} components"
    if handoff:
        status='Checks pass — engineering review' if v.get('review_pass') else status
        selected=handoff['selected_files']
        artifacts=[entry for entry in artifacts if entry[0]!='BOM.csv']+[
            ('HANDOFF.json','Engineering file roles and holds'),('READ_FIRST.md','Read first'),
            (selected['populated_BOM'],'Native populated parts'),
            (selected['populated_positions_UNQUALIFIED'],'Native placement — orientation unqualified'),
            (selected['DNP_BOM'],'DNP parts — review only'),
            ('BOM.csv','Model source parts — unqualified'),('rules.json','Applied model rules')]
        count=f"{assembly['footprints']} native footprints · {assembly['populated_BOM_rows']} populated parts · {assembly['DNP_rows']} DNP"
        issues+=handoff['qualification_holds']
        scope='<p>'+html.escape(handoff.get('model_rules_scope','Applied model rules retained in rules.json'))+'. Native assembly tables retain raw frames; machine orientation,procurement,firmware,physical operation and manufacturing release remain unqualified.</p>'
    artifact_links=' · '.join('<a href="'+path+'">'+label+'</a>' for path,label in artifacts if (d/path).is_file())
    editable=(handoff or {}).get('selected_files',{})
    actions=[]
    if v.get('review_pass') and (d/'review-cam.zip').is_file():
        actions.append('<a class="button-link primary" href="review-cam.zip" download>Download engineering files ↓</a>')
    for path,label in [(editable.get('native_board','reference.kicad_pcb'),'Editable board'),
                       (editable.get('native_project','reference.kicad_pro'),'Project rules')]:
        if (d/path).is_file():actions.append('<a class="button-link" href="'+html.escape(path,quote=True)+'" download>'+label+'</a>')
    heading='Automatically routed board'
    if retained:
        heading='Retained layout evidence'
        actions=['<a class="button-link primary" href="review.html">Current board review →</a>']
        scope='<p>This view preserves an earlier routing experiment. Its model and recorded checks may differ from the selected revision. Use the current board review for matched designs, engineering files and qualification holds.</p>'
    bottom=any(t['layer']=='BCu' for t in b['tracks']) or any(p['pad_type'].get('Smd',{}).get('layer')=='BCu' for f in b['footprints'] for p in f['pads'])
    layer_controls='<button type="button" class="active" aria-pressed="true" data-layer="FCu">Top copper</button>'
    if bottom:layer_controls+='<button type="button" class="active" aria-pressed="true" data-layer="BCu">Bottom copper</button>'
    layer_controls+='<button type="button" class="active" aria-pressed="true" data-layer="label">Labels</button>'
    body=f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>{html.escape(d.name)} · Board review · Netlist Shatter</title><link rel="stylesheet" href="/style.css"><style>
.model-preview{{display:block;width:100%;max-height:65vh;background:#0b1d21;border:1px solid #2e4849;border-radius:8px}}.model-preview line.FCu{{stroke:#ed947e}}.model-preview line.BCu{{stroke:#6baedf}}.model-preview .via,.model-preview .pad{{fill:#e5ce8e;stroke:none}}.model-preview .label{{fill:#deebe9;stroke:none;font:1px monospace;text-anchor:middle;pointer-events:none}}.model-preview .part{{cursor:pointer;margin:0}}.model-preview .part:hover .pad{{fill:white}}#detail pre{{white-space:pre-wrap;overflow-wrap:anywhere;font:12px monospace;line-height:1.7}}</style>
{navigation('boards')}<main class="review-main"><p class="breadcrumb"><a href="/boards">← All board reviews</a></p><section class="review-intro"><small>LAYOUT EVIDENCE</small><h1>{heading}</h1><div class="review-status"><span class="status-pill{'' if v.get('review_pass') else ' pending'}">{status}</span><span class="muted">Physical hardware unvalidated</span></div>{scope}<div class="review-actions">{''.join(actions)}</div></section><div class="review-grid"><section class="review-panel">
<div class="review-toolbar"><h2>Board layout</h2><div class="toggles" role="group" aria-label="Visible board layers">{layer_controls}</div></div>
<svg class="model-preview" role="group" aria-label="Interactive board layout" viewBox="-2 -2 {b['width']+4} {b['height']+4}"><rect width="{b['width']}" height="{b['height']}" fill="none" stroke="#b6c8bf" stroke-width=".1"/>{''.join(el)}</svg>
<p class="preview-caption">Board-model view. Hover a pad for its net; select a component for its pin list. Actual CAM layers are linked below.</p>
<details><summary>Files, checks and run details</summary><div class="file-links">{artifact_links}</div><p>Run: <code>{html.escape(d.name)}</code></p></details>
</section><section class="review-panel"><h2>Routing result</h2><p>{count} · {len(b['tracks'])} tracks</p><p>{b['width']} × {b['height']} × {b['thickness']} mm<br>{len(r['drc']['unrouted_nets'])} unrouted nets · {len(b['vias'])} vias<br>KiCad errors: {v.get('kicad_errors','not checked')} · warnings: {v.get('kicad_warnings','not checked')} · open connections: {v.get('kicad_unconnected','not checked')}</p>
<div id="detail">Select a component to inspect its pins.</div><details><summary>Review notes and qualification limits</summary><ul>{''.join('<li>'+html.escape(s)+'</li>' for s in issues+r['limitations'])}</ul></details></section></div></main>
<script>const board={payload};const visible={{FCu:true,BCu:{str(bottom).lower()},label:true}};document.querySelectorAll('[data-layer]').forEach(button=>button.onclick=()=>{{const c=button.dataset.layer;visible[c]=!visible[c];button.classList.toggle('active',visible[c]);button.setAttribute('aria-pressed',String(visible[c]));document.querySelectorAll('.'+c).forEach(e=>e.style.display=visible[c]?'':'none');document.querySelectorAll('.both-copper').forEach(e=>e.style.display=visible.FCu||visible.BCu?'':'none')}});
document.querySelectorAll('.part').forEach(e=>{{const select=()=>{{const f=board.footprints.find(f=>f.reference===e.dataset.ref);const p=document.createElement('pre');p.textContent=f.reference+' · '+f.value+'\\n'+f.package+'\\n\\n'+f.pads.map(p=>p.pin_number+' '+p.pin_name+' → '+(board.net_names[p.net]||'unassigned')).join('\\n');document.getElementById('detail').replaceChildren(p)}};e.setAttribute('role','button');e.setAttribute('aria-label','Inspect '+e.dataset.ref);e.onclick=select;e.onkeydown=event=>{{if(event.key==='Enter'||event.key===' '){{event.preventDefault();select()}}}}}});</script></html>'''
    return body

def main():
    d=Path(sys.argv[1]);(d/'review.html').write_text(render(d))
if __name__=='__main__':main()
