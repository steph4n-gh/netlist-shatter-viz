#!/usr/bin/env python3
"""Build and independently review software-defined PCB prototypes. No fabrication orders."""
import argparse, hashlib, json, os, shutil, subprocess, sys, zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
NAMES=['bringup','framework','pico','esp32','m64']

def fanout_setting(jar):
    # Only the pinned fix honors automatic_neckdown during fanout. Unknown
    # builds retain the conservative policy; see .tools/freerouting-legal-fanout.json.
    patched='52471120bb444ae01368f41d2842072ae42f507179014923d9623f9501c8cdc3'
    return '--router.fanout.enabled='+str(hashlib.sha256(Path(jar).read_bytes()).hexdigest()==patched).lower()

def find_kicad():
    candidates=[os.getenv('KICAD_CLI'),shutil.which('kicad-cli'),ROOT/'.tools/KiCad.app/Contents/MacOS/kicad-cli',Path('/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli'),Path('/Applications/KiCad.app/Contents/MacOS/kicad-cli')]
    return next((str(p) for p in candidates if p and Path(p).is_file()),None)

def run(cmd, **kwargs):
    print('+', ' '.join(map(str,cmd)),flush=True)
    return subprocess.run(list(map(str,cmd)),cwd=ROOT,**kwargs)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--board',choices=NAMES+['all'],default='all')
    p.add_argument('--route',choices=['cached','freerouting','native','none'],default='cached')
    p.add_argument('--kicad-cli',default=find_kicad())
    p.add_argument('--freerouting-jar',default=os.getenv('FREEROUTING_JAR'))
    p.add_argument('--java',default=os.getenv('JAVA','java'))
    args=p.parse_args()
    if args.route=='freerouting' and not args.freerouting_jar:p.error('--freerouting-jar is required')
    run(['cargo','build','--release','--bins'],check=True)
    binary=ROOT/'target/release/export_manufacturing';statuses={}
    for name in NAMES if args.board=='all' else [args.board]:
        d=ROOT/'build'/name;d.mkdir(parents=True,exist_ok=True)
        for stale in ['review-cam.zip','cam-verification.json','kicad-drc.json','verification.json']:
            (d/stale).unlink(missing_ok=True)
        base=[binary,'--board',name,'--out',d]
        run(base+['--draft'],check=True)
        status={'route':args.route,'independent_checks':'not run'}
        if args.route=='cached':
            ses=ROOT/'reference/routes'/f'{name}.ses'
            if ses.exists():
                digest=hashlib.sha256((d/f'{name}.dsn').read_bytes()).hexdigest()
                if digest!=(ses.with_suffix('.dsn.sha256')).read_text().strip():
                    raise RuntimeError(f'{name}: cached route is stale; regenerate it for this exact DSN')
                run(base+['--session',ses,'--draft'],check=True)
            else:status['route']='unrouted draft; no reviewed session'
        elif args.route=='native':
            routed=run(base+['--route','--draft'])
            status['router_exit_code']=routed.returncode
        elif args.route=='freerouting':
            ses=d/f'{name}.ses';ses.unlink(missing_ok=True)
            with (d/'routing.log').open('w') as log:
                routed=run([args.java,'-Xmx2g','-jar',args.freerouting_jar,'-de',d/f'{name}.dsn','-do',ses,'-mp','30','-mt','1','-da','--gui.enabled=false','--api_server.enabled=false','--router.automatic_neckdown=false',fanout_setting(args.freerouting_jar),'--router.optimizer.enabled=false','--router.job_timeout=00:09:00',f'--user_data_path={ROOT / "build/freerouting-data"}'],stdout=log,stderr=subprocess.STDOUT,timeout=600)
            status['router_exit_code']=routed.returncode
            if routed.returncode==0 and ses.exists():run(base+['--session',ses,'--cleanup-route','--draft'],check=True)
        report=json.loads((d/'report.json').read_text())
        status['software_checks_pass']=report['software_checks_pass']
        status['hardware_validated']=False
        if args.kicad_cli:
            run([args.kicad_cli,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'kicad-drc.json',d/f'{name}.kicad_pcb'],check=True)
            native=json.loads((d/'kicad-drc.json').read_text())
            status['kicad_errors']=sum(v['severity']=='error' for v in native['violations'])
            status['kicad_warnings']=sum(v['severity']=='warning' for v in native['violations'])
            status['kicad_unconnected']=len(native['unconnected_items'])
            cam=run([sys.executable,ROOT/'scripts/verify_cam.py',d,'--kicad-cli',args.kicad_cli])
            status['cam_pass']=cam.returncode==0
            status['independent_checks']='completed'
        status['review_pass']=status['software_checks_pass'] and status.get('cam_pass',False) and status.get('kicad_errors',1)==0 and status.get('kicad_warnings',1)==0 and status.get('kicad_unconnected',1)==0
        if status['review_pass']:
            with zipfile.ZipFile(d/'review-cam.zip','w',zipfile.ZIP_DEFLATED) as z:
                for f in sorted(d.iterdir()):
                    if f.suffix in ['.gbr','.drl'] or f.name in ['report.json','PROTOTYPE.txt']:z.write(f,f.name)
        status['files_sha256']={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(d.iterdir()) if f.is_file() and f.suffix in ['.dsn','.ses','.json','.kicad_pcb','.gbr','.drl','.csv','.zip']}
        (d/'verification.json').write_text(json.dumps(status,indent=2)+'\n');statuses[name]=status
        run([sys.executable,ROOT/'scripts/review_board.py',d],check=True)
    (ROOT/'build/summary.json').write_text(json.dumps(statuses,indent=2)+'\n')
    cards=[]
    for n,s in statuses.items():
        cards.append(f'<a class="card" href="{n}/review.html"><div class="tag">{'CHECKS PASS' if s["review_pass"] else 'DRAFT / BLOCKED'}</div><h2>{n.title()}</h2><img src="{n}/F_Cu.svg"><p>KiCad errors: {s.get("kicad_errors","—")} · unconnected items: {s.get("kicad_unconnected","—")}</p></a>')
    (ROOT/'build/index.html').write_text('<!doctype html><html><meta charset="utf-8"><title>Netlist Shatter — design reviews</title><style>body{background:#0b141d;color:#e4edf3;font:16px system-ui;margin:42px}h1{font-size:42px;margin-bottom:12px}p{color:#a5bac9;line-height:1.6}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(270px,1fr));gap:20px;margin-top:35px}.card{display:block;border:1px solid #344e5d;border-radius:12px;background:#15232d;padding:24px;text-decoration:none;color:inherit}.card:hover{border-color:#74d0b6}.tag{font-size:12px;letter-spacing:.12em;color:#88d1b9}img{width:100%;height:220px;object-fit:contain;background:#203d3c;border-radius:5px}small{color:#84a7bc}</style><small>SOFTWARE-DEFINED PCB WORKBENCH</small><h1>From netlist to inspected copper.</h1><p>Five generated designs. Independent KiCad and CAM verification.<br>Every design remains a physical hardware prototype; select a board to inspect its wiring and review findings.</p><div class="grid">'+''.join(cards)+'</div></html>')
    print('\n'+json.dumps({n:{k:v for k,v in s.items() if k!='files_sha256'} for n,s in statuses.items()},indent=2))
    return 0 if all(s['review_pass'] for s in statuses.values()) else 1
if __name__=='__main__':raise SystemExit(main())
