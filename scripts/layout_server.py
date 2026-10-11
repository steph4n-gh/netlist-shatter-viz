#!/usr/bin/env python3
"""Local interactive placement + fresh autorouting workbench. No cloud service or trace drawing."""
import argparse,base64,binascii,hashlib,io,json,math,os,shutil,stat,subprocess,sys,tempfile,threading,time,unicodedata,uuid,zipfile,zlib
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
from build_board import find_kicad, fanout_setting
from solve_kicad import native_worker_command,solve as solve_native
from create_kicad import create_design
ROOT=Path(__file__).resolve().parents[1]
CURRENT_DESIGNS=ROOT/'build/current-designs'
NAMES=['bringup','framework','pico','esp32','tca9548a']
GUIDE_FILES={name:ROOT/'docs'/name for name in ('QUICKSTART.md','LIMITATIONS.md','RELEASE_NOTES.md')}
GUIDE_FILES.update({name:ROOT/name for name in ('README.md','TOOLCHAIN.md','THIRD_PARTY_NOTICES.md')})
JOBS={};LOCK=threading.Lock();ROUTING=threading.BoundedSemaphore(1)

ZIP_COMPRESSED_LIMIT=16_000_000
ZIP_DECOMPRESSED_LIMIT=20_000_000
ZIP_ENTRY_LIMIT=512


def native_zip_files(upload):
    """Validate a root-level native project ZIP before it enters a route job."""
    from kicad_sexpr import sexpr
    from native_project import copy_assets
    if not isinstance(upload,dict) or set(upload)!={'name','base64'}:
        raise ValueError('Project ZIP requires a name and base64 content')
    name,payload=upload['name'],upload['base64']
    if not isinstance(name,str) or Path(name).name!=name or '/' in name or '\\' in name or not name.lower().endswith('.zip'):
        raise ValueError('Invalid project ZIP filename')
    if not isinstance(payload,str) or len(payload)>4*((ZIP_COMPRESSED_LIMIT+2)//3):
        raise ValueError('Project ZIP exceeds the 16 MB compressed limit')
    try:raw=base64.b64decode(payload,validate=True)
    except (ValueError,binascii.Error) as error:raise ValueError('Invalid project ZIP base64') from error
    if len(raw)>ZIP_COMPRESSED_LIMIT:raise ValueError('Project ZIP exceeds the 16 MB compressed limit')
    files=[];seen=set();actual=advertised=0
    licenses={'LICENSE','LICENSE.txt','LICENSE.md','NOTICE','NOTICE.txt'}
    try:
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            entries=archive.infolist()
            if not 1<=len(entries)<=ZIP_ENTRY_LIMIT:raise ValueError('Project ZIP requires 1–512 entries')
            for entry in entries:
                filename=entry.filename;parts=(filename[:-1] if entry.is_dir() else filename).split('/')
                if (not filename or '\\' in filename or '\x00' in entry.orig_filename or filename.startswith('/')
                    or any(part in ('','.','..') or ':' in part or any(ord(c)<32 for c in part) for part in parts)):
                    raise ValueError('Invalid project ZIP path')
                key=unicodedata.normalize('NFC','/'.join(parts)).casefold()
                if key in seen:raise ValueError('Duplicate project ZIP path')
                seen.add(key)
                kind=stat.S_IFMT(entry.external_attr>>16)
                if kind not in (0,stat.S_IFREG,stat.S_IFDIR) or (kind==stat.S_IFDIR and not entry.is_dir()):
                    raise ValueError('Project ZIP cannot contain symlinks or special files')
                if entry.flag_bits&1 or entry.compress_type not in (zipfile.ZIP_STORED,zipfile.ZIP_DEFLATED):
                    raise ValueError('Project ZIP supports unencrypted stored or deflated files only')
                advertised+=entry.file_size
                if entry.file_size<0 or advertised>ZIP_DECOMPRESSED_LIMIT:raise ValueError('Project ZIP exceeds the 20 MB decompressed limit')
                if entry.is_dir():
                    if entry.file_size:raise ValueError('Invalid ZIP directory entry')
                    continue
                with archive.open(entry) as stream:content=stream.read(ZIP_DECOMPRESSED_LIMIT-actual+1)
                actual+=len(content)
                if actual>ZIP_DECOMPRESSED_LIMIT or len(content)!=entry.file_size:raise ValueError('Project ZIP exceeds its decompressed size limit')
                try:files.append({'name':filename,'text':content.decode('utf-8')})
                except UnicodeDecodeError:
                    if Path(filename).name not in licenses:raise
                    files.append({'name':filename,'_bytes_base64':base64.b64encode(content).decode('ascii')})
    except (zipfile.BadZipFile,RuntimeError,NotImplementedError,UnicodeError,OSError,EOFError,zlib.error) as error:
        raise ValueError('Invalid project ZIP or UTF-8 file content') from error
    core=[f for f in files if '/' not in f['name'] and f['name'].endswith(('.kicad_pcb','.kicad_pro','.kicad_dru','.json'))]
    core,board_name=native_files({'files':core})
    if not board_name.endswith('.kicad_pcb'):raise ValueError('Project ZIP requires an existing KiCad board at its root')
    table=next((f for f in files if f['name']=='fp-lib-table'),None)
    if table:
        parsed=sexpr(table['text'])
        if not parsed or parsed[0]!='fp_lib_table':raise ValueError('Invalid project footprint library table')
        for entry in parsed[1:]:
            if not isinstance(entry,list) or not entry or entry[0]!='lib':continue
            uri=next((v[1] for v in entry if isinstance(v,list) and len(v)==2 and v[0]=='uri'),None)
            uri=json.loads(uri) if uri and uri.startswith('"') else uri
            if not uri or not uri.startswith('${KIPRJMOD}/'):raise ValueError('ZIP footprint libraries must use project-local ${KIPRJMOD}/ paths')
            relative=uri[len('${KIPRJMOD}/'):];parts=relative.split('/')
            if '\\' in relative or any(p in ('','.','..') or ':' in p for p in parts):raise ValueError('Project footprint libraries must remain inside the ZIP project')
    # Reuse the native project's parser and asset inventory, rather than making
    # a second library-copy policy. Only those referenced assets enter the job.
    try:
        with tempfile.TemporaryDirectory(prefix='pcb-project-upload-') as directory:
            source=Path(directory)/'source';source.mkdir()
            for f in files:
                path=source/f['name'];path.parent.mkdir(parents=True,exist_ok=True)
                if 'text' in f:path.write_text(f['text'],encoding='utf-8')
                else:path.write_bytes(base64.b64decode(f['_bytes_base64'],validate=True))
            assets=copy_assets(source,Path(directory)/'assets')
    except (OSError,ValueError,IndexError,json.JSONDecodeError) as error:raise ValueError('Invalid project ZIP assets: '+str(error)) from error
    approved={f['name'] for f in core}|set(assets)
    for f in files:
        if f['name'] not in approved or (f['name'] not in {x['name'] for x in core}|{'fp-lib-table'} and Path(f['name']).name not in licenses and not f['name'].endswith('.kicad_mod')):
            raise ValueError('ZIP contains unsupported or unreferenced project files: '+f['name'])
    return files,board_name


def native_files(data):
    if isinstance(data,dict) and 'project_zip' in data:
        if 'files' in data or 'source_job' in data:raise ValueError('Supply a project ZIP, loose files or a source job, not a combination')
        return native_zip_files(data['project_zip'])
    if not isinstance(data,dict) or not isinstance(data.get('files'),list):raise ValueError('Expected project files')
    files=data['files']
    if not 2<=len(files)<=5:raise ValueError('Select a board, matching project, optional custom rules and electrical/routing profiles')
    names=[]
    for f in files:
        if not isinstance(f,dict) or not isinstance(f.get('name'),str) or not isinstance(f.get('text'),str):raise ValueError('Invalid file')
        name=f['name'];p=Path(name)
        if p.name!=name or '/' in name or '\\' in name or p.suffix not in ['.kicad_pcb','.kicad_pro','.kicad_dru','.json']:raise ValueError('Invalid project filename')
        if name in names:raise ValueError('Duplicate file')
        names.append(name)
    boards=[n for n in names if n.endswith(('.kicad_pcb','.json')) and not n.endswith(('.dc.json','.routing.json'))]
    if len(boards)!=1:raise ValueError('Select exactly one board or new-design JSON')
    stem=Path(boards[0]).stem
    if stem+'.kicad_pro' not in names or any(Path(n).stem!=stem and n not in (stem+'.dc.json',stem+'.routing.json') for n in names):raise ValueError('Project, custom rules and DC profile must match the board filename')
    for f in files:
        if f['name'].endswith('.dc.json'):
            from dc_screen import validate
            validate(json.loads(f['text']))
        if f['name'].endswith('.routing.json'):
            from routing_screen import validate as validate_routing
            validate_routing(json.loads(f['text']))
    return files,boards[0]

def native_source_job(job):
    if not isinstance(job,str) or len(job)!=32 or any(c not in '0123456789abcdef' for c in job):
        raise ValueError('Invalid source job id')
    jobs=(ROOT/'build/layout-jobs').resolve();directory=(jobs/job).resolve()
    if directory.parent!=jobs:raise ValueError('Invalid source job path')
    try:
        result=json.loads((directory/'status.json').read_text())
        if not isinstance(result,dict) or not result.get('native') or result.get('state') not in ('done','failed'):
            raise ValueError('Source must be a terminal native job')
        native=(directory/'native').resolve()
        if native.parent!=directory:raise ValueError('Invalid native source path')
        board=(native/'candidate.kicad_pcb').resolve()
        if board.parent!=native or not board.is_file():raise ValueError('Retained native candidate is missing')
        original=result.get('source')
        if not original:
            retained=json.loads((native/'status.json').read_text())
            original=retained.get('source') if isinstance(retained,dict) else None
        if not isinstance(original,str):raise ValueError('Original native source is missing')
        original=Path(original).resolve()
        if original.suffix!='.kicad_pcb' or not (original.is_relative_to(directory/'source') or original.is_relative_to(directory/'created')):
            raise ValueError('Original source must belong to the retained job')
        if not original.with_suffix('.kicad_pro').is_file():raise ValueError('Original project rules are missing')
        for suffix in ('.kicad_pro','.kicad_dru','.dc.json','.routing.json'):
            path=original.with_suffix(suffix)
            if path.exists() and not path.resolve().is_relative_to(directory):raise ValueError('Original asset escapes its job')
        for name in ('candidate.dc.json','candidate.routing.json','source-design.json'):
            path=native/name
            if path.exists() and path.resolve().parent!=native:raise ValueError('Retained profile escapes its native directory')
        return native,original
    except (OSError,json.JSONDecodeError) as error:
        raise ValueError('Retained source job is incomplete') from error


def original_restoration_job(job):
    """Recover legacy continuation ancestry without adopting its partial board."""
    from native_restoration import NAME,validate
    seen=set()
    while job not in seen:
        seen.add(job)
        native,original=native_source_job(job)
        baseline=native/NAME
        if baseline.exists():
            validate(baseline)
            return baseline,None
        status=json.loads((native.parent/'status.json').read_text())
        if not status.get('source_job') and (native/'status.json').is_file():
            status=json.loads((native/'status.json').read_text())
        parent=status.get('source_job')
        if parent is None:
            try:
                manifest=json.loads((native/'source-manifest.json').read_text())
                if not isinstance(manifest,dict) or not {original.name,original.with_suffix('.kicad_pro').name}<=manifest.keys():
                    raise ValueError('Legacy original source hashes are missing')
                for name,digest in manifest.items():
                    relative=Path(name);path=(original.parent/relative).resolve()
                    if relative.is_absolute() or '..'in relative.parts or not path.is_relative_to(original.parent) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
                        raise ValueError('Legacy original source hash changed')
                for suffix,key in (('.dc.json','dc_screen'),('.routing.json','reference_planes')):
                    profile=original.with_suffix(suffix)
                    if not profile.exists():profile=native/('candidate'+suffix)
                    metadata=status.get(key,{})
                    if not isinstance(metadata,dict):raise ValueError('Legacy profile evidence is invalid')
                    digest=metadata.get('profile_sha256')
                    if profile.exists() or digest is not None:
                        if not profile.is_file() or hashlib.sha256(profile.read_bytes()).hexdigest()!=digest:
                            raise ValueError('Legacy original profile cannot be verified')
            except (OSError,json.JSONDecodeError,TypeError,ValueError)as error:
                raise ValueError('Original source-job chain cannot be verified; upload the original project')from error
            return None,original
        job=parent
    raise ValueError('Original source-job chain is cyclic; upload the original project')


def run_native(job,files,board_name,kicad,mode='route',protected=(),allowed=None,source_job=None):
    d=ROOT/'build/layout-jobs'/job
    try:
        source=d/'source';source.mkdir(parents=True)
        if source_job is not None:
            from native_project import copy_assets
            native,original=native_source_job(source_job)
            from native_restoration import NAME,snapshot,copy_baseline
            baseline,first_source=original_restoration_job(source_job)
            if baseline is not None:copy_baseline(baseline,source/NAME)
            else:
                first_native=first_source.parent.parent/'native'
                profiles={s:first_source.with_suffix(s) if first_source.with_suffix(s).is_file() else first_native/('candidate'+s) if (first_native/('candidate'+s)).is_file() else None for s in ('.dc.json','.routing.json')}
                snapshot(first_source,source/NAME,profiles=profiles)
            copy_assets(native,source)
            shutil.copy2(native/'candidate.kicad_pcb',source/'candidate.kicad_pcb')
            if (native/'source-design.json').is_file():
                shutil.copy2(native/'source-design.json',source/'source-design.json')
            for suffix in ('.kicad_pro','.kicad_dru'):
                path=original.with_suffix(suffix)
                if path.exists():shutil.copy2(path,source/('candidate'+suffix))
            for suffix in ('.dc.json','.routing.json'):
                path=native/('candidate'+suffix)
                if not path.exists():path=original.with_suffix(suffix)
                if path.exists():shutil.copy2(path,source/('candidate'+suffix))
        else:
            for f in files:
                path=source/f['name'];path.parent.mkdir(parents=True,exist_ok=True)
                if 'text' in f:path.write_text(f['text'],encoding='utf-8')
                else:path.write_bytes(base64.b64decode(f['_bytes_base64'],validate=True))
        board=source/board_name
        dc_profile=board.with_suffix('.dc.json');routing_profile=board.with_suffix('.routing.json')
        if board.suffix=='.json':board=create_design(board,d/'created',kicad)
        creation=json.loads((d/'created/creation.json').read_text()) if (d/'created/creation.json').exists() else None
        result=solve_native(board,d/'native',kicad=kicad,mode=mode,protected=protected,creation=creation,allowed=allowed,dc_profile=dc_profile if dc_profile.exists() else None,routing_profile=routing_profile if routing_profile.exists() else None)
        if source_job is not None:result['source_job']=source_job
        result.update(id=job,native=True,review_url=f'/jobs/{job}/native/report.html',archive_url=f'/jobs/{job}/native/prototype.zip' if result['review_pass'] and result.get('state')=='done' else None)
    except Exception as e:result={'id':job,'native':True,'state':'failed','stage':str(e),'review_pass':False}
    finally:
        with LOCK:JOBS[job]=result
        (d/'status.json').write_text(json.dumps(result,indent=2))
        ROUTING.release()

def tool(name):return ROOT/'target/release'/name

def call(cmd,timeout=30,**kwargs):
    r=subprocess.run(list(map(str,cmd)),cwd=ROOT,timeout=timeout,text=True,capture_output=True,**kwargs)
    if r.returncode:raise RuntimeError((r.stderr or r.stdout)[-3000:])
    return r

def load_board(name):
    if name not in NAMES:raise ValueError('Unknown design')
    directory=ROOT/'build'/name if name=='tca9548a' else CURRENT_DESIGNS/name
    return json.loads((directory/'board.json').read_text())

def validate_model_rules(rules):
    fields={'grid_resolution','default_trace_width','via_diameter','via_drill','clearance'}
    if not isinstance(rules,dict) or set(rules)!=fields:
        raise ValueError('Model rules require exactly grid_resolution,default_trace_width,via_diameter,via_drill,clearance')
    try:
        invalid=any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v)
                    or v<0 or (key!='clearance' and v==0) for key,v in rules.items())
    except OverflowError:invalid=True
    if invalid:
        raise ValueError('Model rule dimensions must be finite and positive; clearance may be zero')
    if rules['via_drill']>=rules['via_diameter']:raise ValueError('Model via drill must be smaller than its diameter')
    return rules

def preview(data):
    if not isinstance(data,dict):raise ValueError('Expected a JSON object')
    # Rust performs geometry and constraint validation, also for client-edited boards.
    return json.loads(call([tool('preview_layout')],input=json.dumps(data),timeout=10).stdout)

def complete_model_route(d,kicad,base,before):
    """One bounded native repair, with an exact supported model round trip."""
    from collections import Counter
    from solve_kicad import native_worker_command
    from native_cu_to_model import convert
    from audit_native_inventory import inventory
    proof={'accepted':False,'controller_budget_seconds':240,'manual_trace_coordinates':False}
    directory=d/'native-completion';directory.mkdir()
    def findings(report):
        # Rust reexport creates new native UUIDs. Reference/position/description
        # and multiplicity remain identity; only those regenerated IDs differ.
        return Counter(json.dumps({**v,'items':[{k:value for k,value in item.items() if k!='uuid'}
            for item in v.get('items',[])]},sort_keys=True)
            for v in report.get('violations',[])+report.get('schematic_parity',[]))
    def geometry(folder):
        project=(folder/'candidate.kicad_pro').read_bytes()
        try:call(native_worker_command(kicad,ROOT/'scripts/native_kicad_worker.py')+['dc-geometry',folder],timeout=45)
        finally:(folder/'candidate.kicad_pro').write_bytes(project)
        return json.loads((folder/'dc-geometry.json').read_text())
    try:
        for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
            p=d/('reference'+suffix)
            if p.exists():shutil.copy2(p,directory/('candidate'+suffix))
        shutil.copy2(d/'board.json',directory/'source-model.json')
        original_model=json.loads((directory/'source-model.json').read_text());project=(directory/'candidate.kicad_pro').read_bytes()
        native=geometry(directory)
        profile={'net_limits':[{'net':n['name'],'min_width_mm':n['width']} for n in native['nets'].values() if n['name'] and n['width']>0]}
        profile_path=directory/'candidate.routing.json';profile_path.write_text(json.dumps(profile,indent=2)+'\n')
        repaired=directory/'repair'
        result=solve_native(directory/'candidate.kicad_pcb',repaired,kicad=kicad,mode='repair',timeout=120,routing_profile=profile_path)
        proof['native_solver']=result
        if not (result.get('state')=='done' and result.get('routed_drc',{}).get('unconnected')==0 and result.get('routed_drc',{}).get('errors')==0
            and all(result.get(k) for k in ('preservation_pass','locked_copper_preserved','project_rules_preserved','source_unchanged'))
            and result.get('reference_planes',{}).get('pass')):
            raise ValueError('Native completion did not pass source/routing guards')
        fixed=geometry(repaired)
        if any(t.get('locked',False) for t in fixed['tracks']+fixed['vias']):raise ValueError('Model adapter cannot represent locked native copper')
        model,mapping=convert(directory/'source-model.json',repaired/'dc-geometry.json')
        if {k:v for k,v in model.items() if k not in ('tracks','vias')}!={k:v for k,v in original_model.items() if k not in ('tracks','vias')}:
            raise ValueError('Native adapter altered non-copper model fields')
        model_path=directory/'native-model.json';model_path.write_text(json.dumps(model,indent=2)+'\n')
        (directory/'native-model-mapping-proof.json').write_text(json.dumps(mapping,indent=2)+'\n')
        reexport=directory/'reexport';reexport.mkdir()
        command=list(base)
        if command.count('--input')!=1 or command.count('--out')!=1:raise ValueError('Expected exactly one model input/output')
        command[command.index('--input')+1]=model_path;command[command.index('--out')+1]=reexport
        call(command,timeout=60)
        if (reexport/'reference.kicad_pro').read_bytes()!=project:raise ValueError('Model reexport altered original project rules')
        for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
            p=reexport/('reference'+suffix)
            if p.exists():shutil.copy2(p,reexport/('candidate'+suffix))
        actual=geometry(reexport);expected_inventory=inventory(fixed);actual_inventory=inventory(actual)
        checks={k:expected_inventory[k]==actual_inventory[k] for k in expected_inventory}
        proof['native_inventory']={'pass':all(checks.values()),'checks':checks}
        if not all(checks.values()):raise ValueError('Native Cu/model roundtrip changed actual native inventory')
        call([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',reexport/'kicad-drc.json',reexport/'reference.kicad_pcb'])
        after=json.loads((reexport/'kicad-drc.json').read_text())
        if after.get('unconnected_items') or findings(after)-findings(before):raise ValueError('Reexport added native findings or missing connections')
        # The verification utility selects one PCB. The temporary native-worker
        # alias must not make that selection ambiguous or enter the final app.
        for suffix in ('.kicad_pcb','.kicad_pro','.kicad_dru'):
            alias=reexport/('candidate'+suffix)
            if alias.exists():alias.unlink()
        cam=subprocess.run([sys.executable,str(ROOT/'scripts/verify_cam.py'),str(reexport),'--kicad-cli',kicad],cwd=ROOT,capture_output=True,text=True,timeout=60)
        (reexport/'cam.log').write_text(cam.stdout+cam.stderr);proof['cam_pass']=cam.returncode==0
        if cam.returncode:raise ValueError('Completed model/native CAM comparison failed')
        for p in reexport.iterdir():
            if p.is_file():shutil.copy2(p,d/p.name)
        proof['accepted']=True
    except (ValueError,RuntimeError,KeyError,AssertionError,subprocess.SubprocessError) as error:proof['error']=str(error)
    (directory/'result.json').write_text(json.dumps(proof,indent=2)+'\n')
    return proof

def run_route(job,data,kicad,java,jar):
    d=ROOT/'build/layout-jobs'/job
    def update(**fields):
        with LOCK:JOBS[job].update(fields)
        (d/'status.json').write_text(json.dumps(JOBS[job],indent=2)+'\n')
    start=time.perf_counter()
    try:
        d.mkdir(parents=True,exist_ok=True)
        b=data['board'];b['tracks']=[];b['vias']=[]
        (d/'input.json').write_text(json.dumps(b))
        update(stage='Exporting fresh routing input')
        base=[tool('export_manufacturing'),'--board','reference','--input',d/'input.json','--out',d,'--draft']
        if 'rules' in data:
            rules=validate_model_rules(data['rules'])
            (d/'requested-rules.json').write_text(json.dumps(rules,indent=2,allow_nan=False)+'\n')
            provided_rules_SHA256=hashlib.sha256((d/'requested-rules.json').read_bytes()).hexdigest()
            base += ['--rules',d/'requested-rules.json']
        call(base)
        rules_scope='Supplied model rules' if 'rules' in data else 'Generic prototype rules; no source rules supplied'
        update(routing_rules=json.loads((d/'rules.json').read_text()),routing_rules_scope=rules_scope)
        update(stage='Autorouting — no existing traces reused')
        cmd=[java,'-Xmx1g','-jar',jar,'-de',d/'reference.dsn','-do',d/'route.ses','-mp','30','-mt','1','-da','--gui.enabled=false','--api_server.enabled=false','--router.automatic_neckdown=false',fanout_setting(jar),'--router.optimizer.enabled=false','--router.job_timeout=00:02:30',f'--user_data_path={d / "router-settings"}']
        (d/'routing-command.json').write_text(json.dumps(list(map(str,cmd)),indent=2))
        routing_start=time.perf_counter()
        with (d/'routing.log').open('w') as log:
            result=subprocess.run(list(map(str,cmd)),cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=180)
        if result.returncode or not (d/'route.ses').exists():raise RuntimeError('Autorouter failed; inspect routing.log')
        update(stage='Checking connectivity and clearances',routing_seconds=time.perf_counter()-routing_start)
        call(base+['--session',d/'route.ses','--cleanup-route'],timeout=60)
        report=json.loads((d/'report.json').read_text())
        call([kicad,'pcb','drc','--format','json','--severity-all','--all-track-errors','-o',d/'kicad-drc.json',d/'reference.kicad_pcb'])
        independent=json.loads((d/'kicad-drc.json').read_text())
        physical=independent.get('violations',[])+independent.get('schematic_parity',[])
        completion=None
        if independent.get('unconnected_items') and not any(v.get('severity')=='error' for v in physical):
            update(stage='Completing measured routing obstructions under original board rules')
            completion=complete_model_route(d,kicad,base,independent)
            if completion['accepted']:
                report=json.loads((d/'report.json').read_text());independent=json.loads((d/'kicad-drc.json').read_text())
        if 'rules' in data and (json.loads((d/'rules.json').read_text())!=rules
            or hashlib.sha256((d/'requested-rules.json').read_bytes()).hexdigest()!=provided_rules_SHA256):
            raise RuntimeError('Supplied model rules changed during routing or reexport')
        physical=independent.get('violations',[])+independent.get('schematic_parity',[])
        errors=sum(v['severity']=='error' for v in physical);warnings=sum(v['severity']=='warning' for v in physical);unconnected=len(independent['unconnected_items'])
        update(stage='Comparing generated copper, mask and drills')
        if completion and completion['accepted']:cam_pass=completion['cam_pass']
        else:
            cam=subprocess.run([sys.executable,str(ROOT/'scripts/verify_cam.py'),str(d),'--kicad-cli',kicad],cwd=ROOT,capture_output=True,text=True,timeout=60)
            (d/'cam.log').write_text(cam.stdout+cam.stderr);cam_pass=cam.returncode==0
        board=json.loads((d/'board.json').read_text())
        passed=report['software_checks_pass'] and errors==0 and warnings==0 and unconnected==0 and cam_pass
        verification={'software_checks_pass':report['software_checks_pass'],'kicad_errors':errors,'kicad_warnings':warnings,'kicad_unconnected':unconnected,'cam_pass':cam_pass,'review_pass':passed,'hardware_validated':False}
        if completion:verification['native_completion']=completion
        (d/'verification.json').write_text(json.dumps(verification,indent=2))
        # Always include source attribution for possible TCA-derived exports.
        for f in ['README.md','license.txt','provenance.json']:
            shutil.copy2(ROOT/'reference/adafruit-tca9548a'/f,d/('tca-upstream-'+f))
        (d/'TCA_MODIFICATIONS.txt').write_text('If derived from the Adafruit TCA9548A model: 0805 0R/DNP links replace copper solder jumpers; original pour/artwork omitted; fresh placement/routing. Derived hardware: CC BY-SA 3.0. See upstream attribution and license.\n')
        plots=d/'native-manufacturing'
        if passed:
            source_files={name:hashlib.sha256((d/name).read_bytes()).hexdigest() for name in
                          ('input.json','board.json','rules.json','reference.kicad_pcb','reference.kicad_pro')}
            if (d/'requested-rules.json').exists():source_files['requested-rules.json']=hashlib.sha256((d/'requested-rules.json').read_bytes()).hexdigest()
            selected_export_sha256=source_files['reference.kicad_pcb']
            plots.mkdir()
            raw_drill=d/'native-audit/raw-drill'
            raw_drill.mkdir(parents=True,exist_ok=True)
            call([kicad,'pcb','export','gerbers','-l','F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts','-o',str(plots)+'/',d/'reference.kicad_pcb'])
            call([kicad,'pcb','export','drill','--excellon-separate-th','-o',str(raw_drill)+'/',d/'reference.kicad_pcb'])
            # The native exporter reads candidate.kicad_pcb. Give it an exact
            # temporary copy so this job keeps its existing reference filenames.
            with tempfile.TemporaryDirectory(prefix='model-assembly-') as temporary:
                assembly_source=Path(temporary)
                for suffix in ('.kicad_pcb','.kicad_pro'):
                    shutil.copy2(d/('reference'+suffix),assembly_source/('candidate'+suffix))
                exported=call(native_worker_command(kicad,ROOT/'scripts/native_kicad_worker.py')+['assembly-export',assembly_source],timeout=45)
                (d/'assembly-export.log').write_text(exported.stdout+exported.stderr)
                shutil.copytree(assembly_source/'assembly',d/'assembly')
                shutil.copy2(assembly_source/'native-audit/native-drill-holes.json',d/'native-audit/native-drill-holes.json')
            assembly=json.loads((d/'assembly/native-assembly-frames.json').read_text())
            native_drills=json.loads((d/'native-audit/native-drill-holes.json').read_text())
            if assembly['source_board_sha256']!=selected_export_sha256 or hashlib.sha256((d/'reference.kicad_pcb').read_bytes()).hexdigest()!=selected_export_sha256:
                raise RuntimeError('Selected native board changed during engineering export')
            from native_drill_precision import restore_tool_precision
            precision=restore_tool_precision(raw_drill,native_drills,selected_export_sha256,plots)
            if hashlib.sha256((d/'reference.kicad_pcb').read_bytes()).hexdigest()!=selected_export_sha256:
                raise RuntimeError('Selected native board changed during drill precision restoration')
            for files in precision['files']:
                for key in ('raw_file','restored_file'):files[key]=str(Path(files[key]).relative_to(d))
            (d/'native-audit/drill-precision.json').write_text(json.dumps(precision,indent=2)+'\n')
            if any(hashlib.sha256((d/name).read_bytes()).hexdigest()!=digest for name,digest in source_files.items()):
                raise RuntimeError('Engineering export altered source design or rules')
        payload=[p for p in sorted(d.iterdir()) if p.is_file() and p.suffix in
                 ('.gbr','.drl','.kicad_pcb','.kicad_pro','.csv','.json','.txt','.html','.svg','.md')
                 and p.name not in ('status.json','manifest.json')]
        if d/'review.html' not in payload:payload.append(d/'review.html')
        if passed:
            payload.extend(p for p in sorted(plots.iterdir()) if p.is_file())
            payload.extend(p for p in sorted((d/'assembly').iterdir()) if p.is_file())
            payload.extend(sorted(raw_drill.glob('*.drl')))
            payload.extend([d/'native-audit/native-drill-holes.json',d/'native-audit/drill-precision.json'])
        if (d/'native-completion').exists():payload.extend(p for p in sorted((d/'native-completion').rglob('*'))
            if p.is_file() and p.suffix not in ('.zip','.kicad_prl') and p.name not in ('status.json','manifest.json'))
        if passed:
            assembly=json.loads((d/'assembly/native-assembly-frames.json').read_text())
            if assembly['source_board_sha256']!=source_files['reference.kicad_pcb']:
                raise RuntimeError('Native assembly export does not match the selected board')
            roles={str(p.relative_to(d)):('native_derived_tool_restored_CAM' if p.parent==plots and p.suffix=='.drl' else
                    'native_CAM' if p.parent==plots else
                    'raw_native_Excellon_audit_only' if p.parent==raw_drill else
                    'exact_native_drill_geometry' if p==d/'native-audit/native-drill-holes.json' else
                    'native_drill_tool_restoration_verification' if p==d/'native-audit/drill-precision.json' else
                    'native_assembly_UNQUALIFIED' if p.parent==d/'assembly' else
                    'model_CAM_comparison_only' if p.parent==d and p.suffix in ('.gbr','.drl') else
                    'model_circuit_and_rules' if p.name in ('input.json','board.json','rules.json','requested-rules.json') else
                    'model_assembly_source_UNQUALIFIED' if p.name in ('BOM.csv','CPL.csv') else
                    'native_design_or_verification') for p in payload}
            roles.update({'READ_FIRST.md':'engineering_scope_and_qualification_holds',
                          'HANDOFF.json':'file_roles_and_qualification_stage','manifest.json':'SHA256_file_manifest'})
            handoff={'qualification_stage':'engineering_review','physical_validation':False,
                     'manufacturing_release_approved':False,'firmware_validated':False,
                     'procurement_qualified':False,'assembly_machine_orientation_qualified':False,
                     'native_enabled_findings':errors+warnings+unconnected,'model_rules_scope':rules_scope,
                     'source_files_SHA256':source_files,
                     'selected_files':{'native_board':'reference.kicad_pcb','native_project':'reference.kicad_pro',
                       'editable_circuit_sources':['input.json','board.json'],'model_rules':'rules.json',
                       'provided_model_rules':'requested-rules.json' if (d/'requested-rules.json').exists() else None,
                       'populated_BOM':'assembly/BOM.csv','DNP_BOM':'assembly/BOM-DNP.csv',
                       'populated_positions_UNQUALIFIED':'assembly/positions.csv',
                       'DNP_positions_UNQUALIFIED':'assembly/positions-DNP.csv',
                       'all_native_assembly_frames':'assembly/native-assembly-frames.json',
                       'native_CAM':sorted(str(p.relative_to(d)) for p in payload if p.parent==plots),
                       'exact_native_drill_metadata':'native-audit/native-drill-holes.json',
                       'drill_precision_verification':'native-audit/drill-precision.json',
                       'raw_native_drill_audit_only':sorted(str(p.relative_to(d)) for p in payload if p.parent==raw_drill),
                       'model_assembly_source_UNQUALIFIED':['BOM.csv','CPL.csv']},'file_roles':roles,
                     'native_drill_export':{key:value for key,value in precision.items() if key not in ('records','files')},
                     'qualification_holds':['Exact procurement selections and land patterns are unqualified.',
                       'Native anchor coordinates and angles require assembler package/origin/rotation conversion.',
                       'Model native footprint IDs are assigned when KiCad loads this export; reconcile by reference,value,footprint and frame, not stable persisted UUIDs.',
                       'Firmware, electrical behaviour, thermal/mechanical fit and physical assembly are unvalidated.',
                       'Model input supports two layers and round plated drills; it does not represent plated slots or NPTH.',
                       'Passing enabled native checks and native CAM exports do not approve manufacturing release.']}
            (d/'HANDOFF.json').write_text(json.dumps(handoff,indent=2)+'\n')
            (d/'READ_FIRST.md').write_text(
                '# Model engineering handoff\n\n'
                'This package contains one matched model/native revision for engineering review. '
                'Manufacturing release, physical operation, firmware and procurement are unqualified.\n\n'
                'HANDOFF.json maps every archived file and selects the exact input/routed circuit JSON, model rules, '
                'native PCB/project, native CAM and assembly tables. manifest.json gives the SHA-256 of every '
                'archived file except itself. Native manufacturing files are in native-manufacturing/. '
                'Root model Gerbers/drills are comparison evidence, not the selected manufacturing files. '
                'Root BOM/CPL retain model source records; assembly/ contains the selected native rows.\n\n'
                'Model rules accept exactly grid_resolution,default_trace_width,clearance,via_diameter and '
                'via_drill. Additional source-project rules require the native-project workflow; richer '
                'rule records are rejected by this model interface.\n\n'
                'Native Gerbers include copper, mask, paste, silkscreen and outline; plated and unplated drill '
                'files are separate. Final .drl files are native-derived tool-restored CAM, not byte-exact raw native exports. '
                'Only tool-diameter declarations are restored from exact native integer hole metadata; '
                'native-audit/raw-drill retains untouched native CLI drill files. Native coordinates and slot commands '
                'retain KiCad rounding. native-audit/native-drill-holes.json records the selected board SHA256, '
                'plating, layer spans and effective slot endpoints; native-audit/drill-precision.json records '
                'unique feature/tool matching and raw/restored hashes. '
                'Model input remains two-layer geometry with round plated drills; '
                'the separate drill files do not add plated-slot or NPTH model support. Enabled native checks '
                'preserve the project\'s recorded exclusions.\n\n'
                'Assembly tables use millimetres from the native board origin, +X right and +Y down, and raw '
                'native KiCad orientation degrees. All machine orientations are UNQUALIFIED. Populated tables '
                'honor native BOM/position exclusions; DNP tables are review-only and retain flags. The frame '
                'JSON retains every footprint. Manufacturer and MPN contain only literal native properties '
                'with those names; Values are never inferred purchasing codes.\n\n'
                'Model native footprint IDs are assigned when KiCad loads this export. Reconcile by reference, '
                'value,footprint and frame, not stable persisted UUIDs.\n')
            payload += [d/'READ_FIRST.md',d/'HANDOFF.json']
        call([sys.executable,ROOT/'scripts/review_board.py',d])
        manifest={str(p.relative_to(d)):hashlib.sha256(p.read_bytes()).hexdigest() for p in payload}
        (d/'manifest.json').write_text(json.dumps(manifest,indent=2))
        if passed:
            with zipfile.ZipFile(d/'review-cam.zip','w',zipfile.ZIP_DEFLATED) as z:
                for f in payload+[d/'manifest.json']:z.write(f,f.relative_to(d))
            with zipfile.ZipFile(d/'review-cam.zip') as z:
                if z.testzip() is not None:raise RuntimeError('Engineering archive CRC failed')
                if len(z.namelist())!=len(set(z.namelist())) or set(z.namelist())!=set(manifest)|{'manifest.json'}:
                    raise RuntimeError('Engineering archive membership failed')
                if set(handoff['file_roles'])!=set(z.namelist()):raise RuntimeError('Engineering archive file roles failed')
                if any(hashlib.sha256(z.read(name)).hexdigest()!=digest for name,digest in manifest.items()):
                    raise RuntimeError('Engineering archive hash verification failed')
        length=sum(((t['start']['x']-t['end']['x'])**2+(t['start']['y']-t['end']['y'])**2)**.5 for t in board['tracks'])
        update(stage='Checks pass — engineering review' if passed else 'Blocked — review findings',state='done',board=board,verification=verification,qualification_stage='engineering_review' if passed else None,manufacturing_release_approved=False,assembly_export={key:value for key,value in assembly.items() if key!='frames'} if passed else None,native_drill_export={key:value for key,value in precision.items() if key not in ('records','files')} if passed else None,elapsed_seconds=time.perf_counter()-start,track_length_mm=length,vias=len(board['vias']),manual_trace_edits=0,cleanup=json.loads((d/'route-cleanup.json').read_text()),review_url=f'/jobs/{job}/review.html',archive_url=f'/jobs/{job}/review-cam.zip' if passed else None)
    except Exception as e:
        update(stage=str(e),state='failed',elapsed_seconds=time.perf_counter()-start)
    finally:ROUTING.release()

class Handler(BaseHTTPRequestHandler):
    def reply(self,status,payload,kind='application/json'):
        body=payload if isinstance(payload,bytes) else json.dumps(payload).encode()
        self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(body)
    def allowed(self):
        host=self.headers.get('Host','')
        if host not in [f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}']:return False
        origin=self.headers.get('Origin')
        return not origin or origin in [f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}']
    def do_GET(self):
        if not self.allowed():return self.reply(403,{'error':'Local origin required'})
        path=urlparse(self.path).path
        try:
            if path=='/api/boards':return self.reply(200,{'boards':NAMES,'routing_available':all(self.server.routing_tools)})
            if path.startswith('/api/board/'):
                name=path.rsplit('/',1)[1];b=load_board(name)
                c=None
                p=ROOT/'reference/layout-constraints'/f'{name}.json'
                if p.exists():c=json.loads(p.read_text())
                if name=='tca9548a':c=json.loads((ROOT/'reference/adafruit-tca9548a/constraints.json').read_text())
                directory=ROOT/'build'/name if name=='tca9548a' else CURRENT_DESIGNS/name
                rules_path=directory/'rules.json'
                rules=validate_model_rules(json.loads(rules_path.read_text())) if rules_path.is_file() else None
                return self.reply(200,{'board':b,'constraints':c,'name':name,'rules':rules,
                    'rules_scope':'Rules supplied with this source' if rules is not None else 'Generic prototype rules apply; no source rules supplied. Inspect rules.json in the result.'})
            if path.startswith('/api/job/'):
                job=path.rsplit('/',1)[1]
                if len(job)!=32 or any(c not in '0123456789abcdef' for c in job):raise ValueError('Invalid job id')
                with LOCK:result=dict(JOBS[job]) if job in JOBS else json.loads((ROOT/'build/layout-jobs'/job/'status.json').read_text())
                progress=ROOT/'build/layout-jobs'/job/'native/status.json'
                if result.get('native') and result['state']=='running' and progress.exists():
                    try:
                        result.update(json.loads(progress.read_text()))
                        result['state']='running'  # Wait for manifest/archive completion in the parent.
                    except json.JSONDecodeError:pass
                if result.get('native') and result.get('state') in ('done','failed'):
                    try:
                        native_source_job(job)
                        result['retained_candidate']=True
                    except ValueError:result['retained_candidate']=False
                    native=ROOT/'build/layout-jobs'/job/'native'
                    result['available_files']=[name for name in (
                        'report.html','candidate.kicad_pcb','prototype.zip',
                        'dc-screen.json','candidate.dc.json','routing-screen.json','candidate.routing.json',
                        'original.kicad_pcb','original.zip',
                        'preview/candidate-F_Cu.svg','preview/candidate-B_Cu.svg',
                        'preview/candidate-Edge_Cuts.svg','preview/board.svg') if (native/name).is_file()]
                    if 'prototype.zip' not in result['available_files']:result['archive_url']=None
                return self.reply(200,result)
            if path in ['/examples/'+stem+suffix for stem in ('native-led-bank','native-rc-led-bank','native-plane-led-bank','rp2040-bringup-minimal') for suffix in ('.json','.kicad_pro','.dc.json','.routing.json')]:
                f=ROOT/path.lstrip('/')
            elif path=='/boards':f=ROOT/'scripts/workbench/boards.html'
            elif path in ['/','/native','/guide','/app.js','/native.js','/style.css']:
                f=ROOT/'scripts/workbench'/({'/':'index.html','/native':'native.html','/guide':'guide.html'}.get(path,path[1:]))
            elif path.startswith('/documentation/') and path[len('/documentation/'):] in GUIDE_FILES:
                f=GUIDE_FILES[path[len('/documentation/'):]]
            elif path.startswith('/jobs/'):
                f=(ROOT/'build/layout-jobs'/path[len('/jobs/'):]).resolve()
                if not f.is_relative_to((ROOT/'build/layout-jobs').resolve()):raise ValueError('Invalid path')
            else:return self.reply(404,{'error':'Not found'})
            types={'.md':'text/plain; charset=utf-8','.html':'text/html; charset=utf-8','.js':'text/javascript','.css':'text/css','.svg':'image/svg+xml','.png':'image/png','.json':'application/json','.zip':'application/zip'}
            content=f.read_bytes()
            if path in ('/','/native','/guide','/boards'):
                from review_ui import navigation
                content=content.decode('utf-8').replace('<!-- app-navigation -->',
                    navigation({'/':'design','/native':'native','/guide':'guide','/boards':'boards'}[path])).encode('utf-8')
            elif path.startswith('/jobs/') and f.name=='report.html' and f.parent.name=='native':
                job=f.parent.parent.name
                status=f.parent.parent/'status.json'
                if len(job)==32 and all(c in '0123456789abcdef' for c in job) and status.is_file():
                    from review_ui import native_review
                    content=native_review(json.loads(status.read_text()),f.parent).encode('utf-8')
            elif path.startswith('/jobs/') and f.name=='review.html' and f.parent.parent==(ROOT/'build/layout-jobs').resolve() and (f.parent/'board.json').is_file() and (f.parent/'report.json').is_file():
                # Present retained model runs with today's controls without
                # changing their archived reports or manufacturing evidence.
                from review_board import render
                content=render(f.parent).encode('utf-8')
            elif path.startswith('/jobs/') and f.name=='routing-review.html' and b'function toggle(c)' in content and (f.parent/'board.json').is_file() and (f.parent/'report.json').is_file():
                from review_board import render
                content=render(f.parent,retained=True).encode('utf-8')
            return self.reply(200,content,types.get(f.suffix,'application/octet-stream'))
        except (ValueError,KeyError,FileNotFoundError) as e:self.reply(404,{'error':str(e)})
    def do_POST(self):
        if not self.allowed():return self.reply(403,{'error':'Local origin required'})
        try:
            n=int(self.headers.get('Content-Length','0'))
            limit=24_000_000 if self.path=='/api/native-route' else 4_000_000
            if not 0<n<=limit:raise ValueError(f'Request must be 1 byte–{limit} bytes')
            if self.headers.get('Content-Type','').split(';')[0]!='application/json':raise ValueError('JSON required')
            data=json.loads(self.rfile.read(n))
            if self.path=='/api/native-route':
                if not isinstance(data,dict):raise ValueError('Expected a JSON object')
                source_job=data.get('source_job')
                if source_job is not None:
                    if 'files' in data or 'project_zip' in data:raise ValueError('Supply uploaded files or a source job, not both')
                    native_source_job(source_job)
                    original_restoration_job(source_job)
                    files,board_name=[],'candidate.kicad_pcb'
                else:files,board_name=native_files(data)
                mode=data.get('mode','route');protected=data.get('protected_nets',[]);allowed=data.get('editable_nets')
                if mode not in ('route','repair','improve'):raise ValueError('Mode must be route, repair or improve')
                if source_job is not None and mode=='route':raise ValueError('A retained result requires repair or improvement')
                if board_name.endswith('.json') and mode!='route':raise ValueError('New-design JSON requires the new-design workflow')
                if not isinstance(protected,list) or any(not isinstance(n,str) for n in protected):raise ValueError('Protected nets must be a list of names')
                if allowed is not None and (not isinstance(allowed,list) or any(not isinstance(n,str) or not n for n in allowed)):raise ValueError('Editable nets must be a list of nonempty names')
                if allowed is not None and mode!='improve':raise ValueError('Editable nets apply only to improvement')
                if not self.server.routing_tools[0] or (mode=='route' and not all(self.server.routing_tools)):raise ValueError('Required routing tools are missing')
                if not ROUTING.acquire(blocking=False):return self.reply(409,{'error':'A route is already running'})
                job=uuid.uuid4().hex
                with LOCK:JOBS[job]={'id':job,'native':True,'state':'running','stage':'Preparing native project'}
                threading.Thread(target=run_native,args=(job,files,board_name,self.server.routing_tools[0],mode,protected,allowed,source_job),daemon=True).start()
                return self.reply(202,{'id':job})
            if self.path=='/api/preview':return self.reply(200,preview(data))
            if self.path=='/api/route':
                if not isinstance(data,dict):raise ValueError('Expected a JSON object')
                if not all(self.server.routing_tools):raise ValueError('KiCad, Java or Freerouting is missing')
                if 'rules' in data:validate_model_rules(data['rules'])
                # Validate without changing requested placement: pin every footprint.
                board=data['board'];validation=preview({'board':board,'constraints':{'fixed':[f['reference'] for f in board['footprints']]}})
                if validation['report']['placement_issues']:raise ValueError('; '.join(validation['report']['placement_issues']))
                if not ROUTING.acquire(blocking=False):return self.reply(409,{'error':'A route is already running; previews remain available'})
                job=uuid.uuid4().hex
                with LOCK:JOBS[job]={'id':job,'state':'running','stage':'Queued'}
                threading.Thread(target=run_route,args=(job,{'board':board,**({'rules':data['rules']} if 'rules' in data else {})},*self.server.routing_tools),daemon=True).start()
                return self.reply(202,{'id':job})
            self.reply(404,{'error':'Not found'})
        except (ValueError,KeyError,RuntimeError,subprocess.TimeoutExpired) as e:self.reply(400,{'error':str(e)})
    def log_message(self,*args):pass

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',type=int,default=8766);args=p.parse_args()
    call(['cargo','build','--release','--bins','--locked'],timeout=120)
    for name in NAMES[:-1]:
        # Rebuild current zero-copper designs after compiling the actual source.
        # Legacy exports and immutable routed handoffs remain historical evidence.
        call([tool('export_manufacturing'),'--board',name,'--out',CURRENT_DESIGNS/name,'--draft'])
    if not (ROOT/'build/tca9548a/board.json').exists():call([sys.executable,ROOT/'scripts/import_tca9548a.py'])
    java=os.getenv('JAVA') or (str(ROOT/'.tools/jre/bin/java') if (ROOT/'.tools/jre/bin/java').exists() else shutil.which('java'))
    jar=os.getenv('FREEROUTING_JAR') or str(ROOT/'.tools/freerouting.jar')
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler);server.routing_tools=(find_kicad(),java,jar if Path(jar).exists() else None)
    print(f'Layout workbench: http://127.0.0.1:{args.port}',flush=True)
    server.serve_forever()
if __name__=='__main__':main()
