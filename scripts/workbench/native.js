const $=id=>document.getElementById(id);
let displayedJob=null;
let nativeBusy=false;
function updateControls(){
 const mode=$('nativeMode').value;
 $('nativeRoute').textContent={route:'Create / route & check →',repair:'Finish connections & check →',improve:'Improve & check →'}[mode];
 $('nativeRoute').disabled=nativeBusy||!$('files').files.length;
 for(const id of ['nativeMode','files','exampleCircuit','createExample'])$(id).disabled=nativeBusy;
 $('editableNets').disabled=nativeBusy||mode!=='improve';
 $('protectedNets').disabled=nativeBusy||mode==='route';
 $('protectedNetsField').hidden=mode==='route';
 $('editableNetsField').hidden=mode!=='improve';
 for(const id of ['improveResult','continueResult'])$(id).disabled=nativeBusy||!displayedJob;
 $('createExample').textContent={led:'Create LED-bank example →',rp2040:'Create RP2040 example →'}[$('exampleCircuit').value]||'Create selected example →';
 const copper=$('copperView');if(copper)copper.disabled=nativeBusy;
 $('nativeEvidence').hidden=!($('nativeFindings').textContent||'').trim();
}
function busy(value){nativeBusy=value;updateControls()}
function clearResult(jobId){
 if(!/^[a-f0-9]{32}$/.test(jobId))throw Error('The server did not return a valid job link.');
 displayedJob=null;$('improveResult').hidden=true;$('continueResult').hidden=true;
 for(const id of ['nativeLinks','nativePreview','nativeMetrics','nativeComparison'])$(id).replaceChildren();
 $('nativeFindings').textContent='';updateControls();
}
function showError(error){$('nativeStatus').textContent=error.message+(displayedJob?' The previous result remains available.':'');$('nativeStatus').className='bad'}
const editableNets=()=>{const names=$('editableNets').value.split('\n').map(n=>n.trim()).filter(Boolean);return names.length?names:null};
$('nativeMode').onchange=updateControls;
$('exampleCircuit').onchange=updateControls;
const requestedOptions=new URLSearchParams(location.search);
const requestedMode=requestedOptions.get('mode');
if(['route','repair','improve'].includes(requestedMode))$('nativeMode').value=requestedMode;
if(['led','rp2040'].includes(requestedOptions.get('example')))$('exampleCircuit').value=requestedOptions.get('example');
$('nativeMode').onchange();
$('files').onchange=()=>{$('selection').textContent=Array.from($('files').files).map(f=>f.name).join(' · ')||'No files selected. Files stay in this local workspace.';updateControls()};
$('nativeRoute').onclick=async()=>{
 if(nativeBusy)return;
 try{
  const files=Array.from($('files').files);if(!files.length)throw Error('Select a project ZIP or the board and its matching project file.');
  const zip=files.find(f=>f.name.toLowerCase().endsWith('.zip'));
  if(zip&&files.length!==1)throw Error('Select one project ZIP on its own.');
  if(files.reduce((n,f)=>n+f.size,0)>(zip?16_000_000:20_000_000))throw Error('Files exceed the upload limit. See Files to include and upload limits.');
  busy(true);
  const body={mode:$('nativeMode').value,editable_nets:$('nativeMode').value==='improve'?editableNets():null,protected_nets:$('nativeMode').value!=='route'?$('protectedNets').value.split('\n').map(n=>n.trim()).filter(Boolean):[]};
  if(zip){
   const bytes=new Uint8Array(await zip.arrayBuffer());let binary='';
   for(let i=0;i<bytes.length;i+=32768)binary+=String.fromCharCode(...bytes.subarray(i,i+32768));
   body.project_zip={name:zip.name,base64:btoa(binary)};
  }else body.files=await Promise.all(files.map(async f=>({name:f.name,text:await f.text()})));
  $('nativeStatus').className='muted';$('nativeStatus').textContent='Preparing project…';
  const response=await fetch('/api/native-route',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const job=await response.json();if(!response.ok)throw Error(job.error);
  clearResult(job.id);$('nativeTitle').textContent=files.find(f=>f.name.endsWith('.kicad_pcb'))?.name||zip?.name||'Your project';
  history.replaceState(null,'','/native?job='+job.id);
  await showJob(job.id);
 }catch(e){showError(e)}finally{busy(false)}
};

async function showJob(jobId){
 if(!/^[a-f0-9]{32}$/.test(jobId))throw Error('Invalid job link');
 let result;
 do{
  const r=await fetch('/api/job/'+jobId);result=await r.json();if(!r.ok)throw Error(result.error);
  $('nativeStatus').textContent=result.stage;
  if(result.state==='running')await new Promise(resolve=>setTimeout(resolve,700));
 }while(result.state==='running');
 const hasOpenConnections=result.routed_drc?.unconnected>0;
 const available=path=>!Array.isArray(result.available_files)||result.available_files.includes(path);
 const canContinue=result.retained_candidate!==false;
 displayedJob=null;$('improveResult').hidden=true;$('continueResult').hidden=true;
 if(hasOpenConnections||result.mode==='repair'){$('nativeMode').value='repair';$('nativeMode').onchange();$('protectedNets').value=(result.optimization?.protected_nets||result.protected_nets||[]).join('\n')}
 if(result.state==='failed'){
  if(!result.retained_candidate)throw Error(result.stage);
  displayedJob=canContinue?jobId:null;$('continueResult').hidden=!canContinue||!hasOpenConnections;
  $('nativeStatus').className='bad';$('nativeStatus').textContent=result.stage+' · Retained candidate available for review.';
  $('nativeTitle').textContent=result.source?.split('/').pop()||'Retained native candidate';
  $('nativeFindings').textContent=JSON.stringify({stage:result.stage,routed_drc:result.routed_drc},null,2);
  if(result.review_url&&available('report.html')){const review=document.createElement('a');review.textContent='Full review ↗';review.href=result.review_url;review.target='_blank';review.rel='noopener';review.style='color:#b4f2d1;margin-right:20px';$('nativeLinks').append(review)}
  const a=document.createElement('a');a.textContent='Retained KiCad board ↗';a.href='/jobs/'+jobId+'/native/candidate.kicad_pcb';
  if(available('candidate.kicad_pcb'))$('nativeLinks').append(a);updateControls();return;
 }
 displayedJob=canContinue?jobId:null;$('improveResult').hidden=!canContinue;$('continueResult').hidden=!canContinue||result.review_pass||!hasOpenConnections;
 $('nativeTitle').textContent=result.source?.split('/').pop()||'Native project';
  // The worker status may finish a fraction before the parent writes its links.
  const base='/jobs/'+jobId+'/native/';
  $('nativeStatus').textContent=(result.review_pass?'ENGINEERING CHECKS PASS under supplied rules':'REVIEW BLOCKED — findings retained')+' · '+(result.elapsed_seconds||0).toFixed(1)+' s · 0 manual trace edits';
  $('nativeStatus').className=result.review_pass?'good':'bad';
  if(result.creation){const placed=(result.creation.fixed_components||[]).length===result.creation.components;
   $('nativeComparison').textContent='CREATED FROM CIRCUIT DATA · '+result.creation.components+' components · '+result.creation.nets+' nets · '+result.creation.initial_tracks+' input traces · '+(placed?'supplied placement and automatic routing.':'automatic placement and routing with supplied anchors.');}
  if(result.optimization){
   const o=result.optimization;
   $('nativeComparison').textContent=(o.improved?'IMPROVED':'ORIGINAL RETAINED')+' · Copper '+o.before.length_mm.toFixed(2)+' → '+o.after.length_mm.toFixed(2)+' mm · Vias '+o.before.vias+' → '+o.after.vias+' · '+o.trials.filter(t=>t.accepted).length+' accepted changes. Full electrical performance is not verified.';
   $('nativeMode').value=hasOpenConnections?'repair':'improve';$('nativeMode').onchange();$('protectedNets').value=o.protected_nets.join('\n');$('editableNets').value=(o.editable_nets||[]).join('\n');
  }

  const repair=result.power_repair||(result.optimization?.power_repaired?result.optimization:null);
  if(repair){const p=document.createElement('p');p.className=repair.power_repaired?'good':'bad';p.textContent=repair.power_repaired?'SUPPLY REPAIRED · Traces widened automatically; supplied voltage limits and independent DRC pass.':'SUPPLY REPAIR INCOMPLETE · Findings retained; inspect the electrical evidence.';$('nativeComparison').append(p)}
  if(result.dc_screen){const d=result.dc_screen,p=document.createElement('p');p.className=d.pass_all?'good':'bad';p.textContent=(d.pass_all?'SUPPLIED ELECTRICAL SCENARIOS PASS':'SUPPLIED ELECTRICAL SCENARIOS BLOCKED')+' · DC: '+Object.entries(d.loads||{}).map(([pin,v])=>pin+': '+v.voltage_v.toFixed(5)+' V').join(' · ')+' · '+(d.return_path_modelled?'Explicit return paths included':'Ideal ground')+'; supplied loads only.';$('nativeComparison').append(p);
   if(d.rc_hold){const rc=document.createElement('p');rc.className=d.rc_hold.pass_limits?'good':'bad';rc.textContent=(d.rc_hold.pass_limits?'RC CURRENT-HOLD PASS':'RC CURRENT-HOLD BLOCKED')+' · '+Object.entries(d.rc_hold.events).map(([name,e])=>name+': '+Math.min(...Object.values(e.loads).map(v=>v.minimum_v)).toFixed(5)+'–'+Math.max(...Object.values(e.loads).map(v=>v.maximum_v)).toFixed(5)+' V across '+Object.keys(e.loads).length+' loads').join(' · ')+' · '+(d.rc_hold.converged?'Refinement checks pass. ':'Refinement checks fail. ')+'Passive RC model; regulator loop, inductance, thermal and signal integrity remain unverified.';$('nativeComparison').append(rc)}
   else {const scope=document.createElement('p');scope.textContent='No transient, thermal or signal-integrity validation.';$('nativeComparison').append(scope)}
  }
  if(result.reference_planes){const r=result.reference_planes,p=document.createElement('p');p.className=r.pass&&r.profile_unchanged?'good':'bad';const failed=r.rules.filter(v=>!v.pass);p.textContent=(r.pass&&r.profile_unchanged?'ROUTING RULES PASS':'ROUTING RULES BLOCKED')+' · '+(r.rules.length-failed.length)+' / '+r.rules.length+' supplied rules pass'+(failed.length?' · Failing: '+failed.map(v=>v.id||v.net).join(', '):'')+' · Impedance and timing remain unverified.';$('nativeComparison').append(p)}
  for(const [name,value] of [['Copper layers',result.inventory?.source.copper_layers],['Source pads',result.inventory?.source.pads],['Unconnected',result.routed_drc?.unconnected],['DRC findings',result.routed_drc?.total_findings]]){const box=document.createElement('div'),label=document.createElement('small'),number=document.createElement('strong');label.textContent=name;number.textContent=value??'—';box.append(label,number);$('nativeMetrics').append(box)}
  for(const [label,path] of [['Full review','report.html'],...(result.dc_screen?[['Electrical evidence','dc-screen.json'],['Electrical profile','candidate.dc.json']]:[]),...(result.reference_planes?[['Routing evidence','routing-screen.json'],['Routing profile','candidate.routing.json']]:[]),...((result.optimization||result.mode==='repair')?[['Original board','original.kicad_pcb'],...(result.original_archive?[['Original project','original.zip']]:[])]:[]),['Editable KiCad board','candidate.kicad_pcb'],...(result.review_pass?[['Engineering files','prototype.zip']]:[])]){if(!available(path))continue;const a=document.createElement('a');a.textContent=label+' ↗';a.href=base+path;a.target='_blank';a.rel='noopener';a.style='color:#b4f2d1;margin-right:20px';$('nativeLinks').append(a)}
  if(result.routed_drc){
   const label=document.createElement('label'),select=document.createElement('select'),view=document.createElement('div');
   const has=path=>available('preview/'+path);
   const views=[
    ['faces','Front traces over back copper',has('candidate-F_Cu.svg')&&has('candidate-B_Cu.svg')],
    ['front','Front copper',has('candidate-F_Cu.svg')],
    ['back','Back copper',has('candidate-B_Cu.svg')],
    ['all','All copper layers (opaque pours may hide traces)',has('board.svg')]
   ].filter(v=>v[2]);
   if(views.length){
    label.textContent=views.length>1?'Copper view ':views[0][1];label.style='display:block;margin-top:20px';
    for(const [value,text] of views){const option=document.createElement('option');option.value=value;option.textContent=text;select.append(option)}
    select.value=views[0][0];select.id='copperView';select.disabled=nativeBusy;
    if(views.length>1)label.append(select);
    view.style='position:relative;background:#0b1d21;margin-top:12px';$('nativePreview').append(label,view);
    const draw=()=>{view.replaceChildren();
     const paths=select.value==='faces'?[['candidate-F_Cu.svg',1],['candidate-B_Cu.svg',.35]]:select.value==='all'?[['board.svg',1]]:[[select.value==='front'?'candidate-F_Cu.svg':'candidate-B_Cu.svg',1]];
     if(select.value!=='all'&&has('candidate-Edge_Cuts.svg'))paths.push(['candidate-Edge_Cuts.svg',1]);
     paths.forEach(([path,opacity],i)=>{const img=document.createElement('img');img.src=base+'preview/'+path;img.alt=path==='board.svg'?'All native copper layers':path==='candidate-F_Cu.svg'?'Front copper':path==='candidate-B_Cu.svg'?'Back copper':'Board outline';img.style=i?'position:absolute;inset:0;width:100%;height:100%;object-fit:contain;pointer-events:none':'position:relative;display:block;width:100%;max-height:600px;object-fit:contain';img.style.opacity=opacity;img.style.zIndex=select.value==='faces'?(i===1?0:1):i;view.append(img)})
    };select.onchange=draw;draw();
   }
  }
  $('nativeTitle').focus?.({preventScroll:true});
  $('nativeFindings').textContent=JSON.stringify({stage:result.stage,dc_screen:result.dc_screen,reference_planes:result.reference_planes,geometry_preserved:result.preservation_pass,project_rules_preserved:result.project_rules_preserved,source_unchanged:result.source_unchanged,selected_router:result.selected_router,router_trials:result.router_trials,source_drc:result.source_drc,routed_drc:result.routed_drc},null,2);
  updateControls();
}

const savedJob=new URLSearchParams(location.search).get('job');
if(savedJob){
 busy(true);$('nativeStatus').textContent='Loading saved result…';
 showJob(savedJob).catch(showError).finally(()=>busy(false));
}

$('createExample').onclick=async()=>{
 if(nativeBusy)return;busy(true);
 try{
  const names=$('exampleCircuit').value==='rp2040'?['rp2040-bringup-minimal.json','rp2040-bringup-minimal.kicad_pro']:
   ['native-plane-led-bank.json','native-plane-led-bank.kicad_pro','native-plane-led-bank.dc.json','native-plane-led-bank.routing.json'];
  const files=await Promise.all(names.map(async name=>{const response=await fetch('/examples/'+name);
   if(!response.ok)throw Error('Example file unavailable: '+name);return {name,text:await response.text()}}));
  const r=await fetch('/api/native-route',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({files,mode:'route'})});
  const job=await r.json();if(!r.ok)throw Error(job.error);
  clearResult(job.id);$('nativeMode').value='route';$('nativeMode').onchange();history.replaceState(null,'','/native?job='+job.id);await showJob(job.id);
 }catch(e){showError(e)}finally{busy(false)}
};

async function continueResult(mode){
 const previous=displayedJob;if(nativeBusy||!previous)return;busy(true);
 try{
  const protected_nets=$('protectedNets').value.split('\n').map(n=>n.trim()).filter(Boolean);
  const body={source_job:previous,mode,protected_nets};
  if(mode==='improve')body.editable_nets=editableNets();
  const response=await fetch('/api/native-route',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const job=await response.json();if(!response.ok)throw Error(job.error);
  clearResult(job.id);
  $('nativeMode').value=mode;$('nativeMode').onchange();
  history.replaceState(null,'','/native?job='+job.id);await showJob(job.id);
 }catch(e){showError(e)}finally{busy(false)}
}
$('improveResult').onclick=()=>continueResult('improve');
$('continueResult').onclick=()=>continueResult('repair');
