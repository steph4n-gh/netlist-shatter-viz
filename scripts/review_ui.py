"""Shared navigation for the local workbench and its review pages."""
import html
import json
import re


def navigation(active=''):
    links = [('/', 'Design', 'design'), ('/native', 'Your board', 'native'),
             ('/boards', 'Board reviews', 'boards'), ('/guide', 'Guide', 'guide')]
    items = ''.join('<a href="' + url + '"' +
                    (' aria-current="page"' if key == active else '') +
                    '>' + label + '</a>' for url, label, key in links)
    return ('<header class="site-header"><a class="brand" href="/">'
            '<span class="mark" aria-hidden="true">⌘</span> NETLIST SHATTER</a>'
            '<nav class="app-nav" aria-label="Main navigation">' + items +
            '</nav><span class="local">Local workspace</span></header>')


def native_review(result, directory):
    """Keep the native report readable while retaining its complete evidence."""
    drc = result.get('routed_drc', {})
    job = directory.parent.name
    back = '/native?job=' + job if re.fullmatch(r'[a-f0-9]{32}', job) else '/native'
    state = 'Engineering checks pass' if result.get('review_pass') else 'Review needs attention'
    links = [('candidate.kicad_pcb', 'Editable board'), ('routed-drc.json', 'Native findings'),
             ('preservation.json', 'Source preservation')]
    if (directory/'restoration.json').is_file():links.append(('restoration.json','Original connectivity'))
    if (directory/'coupled-completion/result.json').is_file():links.append(('coupled-completion/result.json','Coordinated routing'))
    if (directory / 'HANDOFF.json').is_file():
        links += [('READ_FIRST.md', 'Package scope'), ('HANDOFF.json', 'Files and holds'),
                  ('assembly/BOM.csv', 'BOM'), ('assembly/positions.csv', 'Native placement frames')]
    actions = ''.join('<a href="' + path + '">' + label + '</a>'
                      for path, label in links if (directory / path).is_file())
    if result.get('review_pass') and (directory / 'prototype.zip').is_file():
        actions += '<a href="prototype.zip">Engineering files</a>'
    preview = ''
    restoration=result.get('restoration')
    restored=(' <div><strong>'+str(restoration['restored'])+'/'+str(restoration['required'])+
              '</strong><span>Original connections preserved</span></div>') if restoration else ''
    for path, label in [('preview/board.svg', 'Native copper and board outline'),
                        ('preview/candidate-F_Cu.svg', 'Front copper'),
                        ('preview/candidate-B_Cu.svg', 'Back copper')]:
        if (directory / path).is_file():
            preview = '<div class="preview-frame"><img alt="' + label + '" src="' + path + '"></div>'
            break
    return ('<!doctype html><html lang="en"><head><meta charset="utf-8">'
            '<meta name="viewport" content="width=device-width,initial-scale=1">'
            '<title>Project review · Netlist Shatter</title><link rel="stylesheet" href="/style.css"></head><body>' +
            navigation('native') + '<main class="review-main"><p class="breadcrumb"><a href="' + back + '">'
            '← Your board</a></p><section class="review-intro"><div class="review-status"><span class="status-pill' +
            ('' if result.get('review_pass') else ' pending') + '">' + state +
            '</span><span class="muted">Physical hardware unvalidated</span></div><h1>Project review</h1><p>' +
            html.escape(str(result.get('stage', state))) + '</p></section><div class="review-metrics">'
            '<div><strong>' + str(drc.get('unconnected', '—')) + '</strong><span>Open connections</span></div>'
            '<div><strong>' + str(drc.get('total_findings', '—')) + '</strong><span>Native findings</span></div>'
            + restored + '</div><section class="review-panel">' + preview +
            '<p>Supplied geometry, rules and declared constraints govern this review. These checks do not establish '
            'physical electrical performance or manufacturing qualification.</p><div class="file-links">' + actions +
            '</div><details><summary>Complete evidence and assumptions</summary><pre style="white-space:pre-wrap;overflow-wrap:anywhere">' +
            html.escape(json.dumps(result, indent=2)) + '</pre></details></section></main></body></html>')
