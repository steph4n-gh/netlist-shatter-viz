// Run with node: exercise the actual browser script without a network or DOM library.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const path=require('node:path');
const file=path.join(__dirname,'../scripts/workbench/native.js');
class Element{
 constructor(){this.value='';this.hidden=false;this.disabled=false;this.files=[];this.children=[];this.style={}}
 append(...nodes){this.children.push(...nodes)}
 replaceChildren(...nodes){this.children=nodes}
}
const nodes=new Map();
const element=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id)};
element('nativeMode').value='route';
const requests=[];let next=0,passing=false;
const result={state:'done',stage:'Blocked',elapsed_seconds:1,native:true,source:'/retained/candidate.kicad_pcb',
 inventory:{source:{pads:3,copper_layers:2}},routed_drc:{unconnected:1,total_findings:1}};
const context=vm.createContext({URLSearchParams,setTimeout,location:{search:''},history:{replaceState(){}},
 document:{getElementById:element,createElement:()=>new Element()},
 fetch:async(url,options)=>{
  if(url==='/api/native-route'){
   const body=JSON.parse(options.body);requests.push(body);const id=(++next).toString(16).repeat(32);
   return{ok:true,json:async()=>({id})};
  }
  // Actual retained-job responses include the effective protection list.
  if(url.startsWith('/api/job/'))return{ok:true,json:async()=>({...result,protected_nets:result.protected_nets??requests.at(-1)?.protected_nets??[],review_pass:passing})};
  throw Error('Unexpected file download: '+url);
 }});
vm.runInContext(fs.readFileSync(file,'utf8'),context,{filename:file});
(async()=>{
 await vm.runInContext("showJob('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')",context);
 assert.equal(element('continueResult').hidden,false,'Blocked native result exposes continuation');
 element('protectedNets').value='USB_D+\nUSB_D-';element('editableNets').value='LED';
 await element('continueResult').onclick();
 assert.deepEqual(requests[0],{source_job:'a'.repeat(32),mode:'repair',protected_nets:['USB_D+','USB_D-']});
 assert.equal(element('nativeMode').value,'repair');assert.equal(element('editableNets').disabled,true);
 assert.equal(element('protectedNets').disabled,false);
 passing=true;await element('improveResult').onclick();
 assert.deepEqual(requests[1],{source_job:'1'.repeat(32),mode:'improve',protected_nets:['USB_D+','USB_D-'],editable_nets:['LED']});
 assert.equal(element('continueResult').hidden,true,'Passing native result hides continuation');
 element('nativeMode').value='repair';element('nativeMode').onchange();
 element('files').files=[{name:'board.kicad_pcb',size:20,text:async()=>'(kicad_pcb)'},
                        {name:'board.kicad_pro',size:2,text:async()=>'{ }'}];
 await element('nativeRoute').onclick();
 assert.equal(requests[2].mode,'repair');assert.deepEqual(requests[2].protected_nets,['USB_D+','USB_D-']);
 assert.equal(requests[2].editable_nets,null);assert.equal(requests[2].files.length,2);
 assert.equal(requests[2].source_job,undefined);
 result.mode='repair';result.protected_nets=['CLOCK'];result.original_archive=true;passing=false;
 element('nativeLinks').replaceChildren(); // A loaded result begins with a fresh link list.
 await vm.runInContext("showJob('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')",context);
 assert.equal(element('nativeMode').value,'repair');assert.equal(element('protectedNets').value,'CLOCK');
 const links=element('nativeLinks').children;
 assert.ok(links.some(link=>link.textContent==='Original board ↗'&&link.href.endsWith('/original.kicad_pcb')));
 assert.ok(links.some(link=>link.textContent==='Original project ↗'&&link.href.endsWith('/original.zip')));
 assert.ok(!links.some(link=>link.textContent==='Prototype CAM ↗'),'Blocked repair has no CAM link');
 result.state='failed';result.retained_candidate=true;result.stage='Interrupted repair';
 await vm.runInContext("showJob('bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb')",context);
 assert.equal(element('continueResult').hidden,false,'Failed job with validated retained candidate can resume');
 assert.equal(element('improveResult').hidden,true,'Incomplete failed job offers finishing only');
 await element('continueResult').onclick();
 assert.deepEqual(requests[3],{source_job:'b'.repeat(32),mode:'repair',protected_nets:['CLOCK']});
 result.retained_candidate=false;
 await assert.rejects(vm.runInContext("showJob('cccccccccccccccccccccccccccccccc')",context),/Interrupted repair/);
 assert.equal(element('continueResult').hidden,true,'Missing retained candidate cannot resume a stale prior result');
 result.state='done';passing=false;result.retained_candidate=true;
 result.optimization={before:{length_mm:1,vias:0},after:{length_mm:1,vias:0},trials:[],protected_nets:['CLOCK']};
 await vm.runInContext("showJob('cccccccccccccccccccccccccccccccc')",context);
 assert.equal(element('nativeMode').value,'repair','An optimization result with opens retains Finish mode');
 result.state='done';result.review_pass=true;passing=true;
 result.available_files=['candidate.kicad_pcb','preview/candidate-F_Cu.svg','preview/candidate-B_Cu.svg','preview/candidate-Edge_Cuts.svg'];
 result.retained_candidate=false;result.optimization=undefined;
 element('nativeLinks').replaceChildren();element('nativePreview').replaceChildren();
 await vm.runInContext("showJob('dddddddddddddddddddddddddddddddd')",context);
 assert.equal(element('continueResult').hidden,true);
 assert.equal(element('improveResult').hidden,true,'Unavailable candidate cannot be continued');
 assert.ok(element('nativeLinks').children.every(link=>link.href.endsWith('/candidate.kicad_pcb')),'Missing review/archive/profile links are absent');
 const previewLabel=element('nativePreview').children[0];
 const viewSelector=previewLabel.children[0];
 assert.deepEqual(Array.from(viewSelector.children,o=>o.value),['faces','front','back'],'Missing combined preview is not an option');
 element('nativeMode').value='route';element('nativeMode').onchange();
 assert.equal(element('protectedNetsField').hidden,true);assert.equal(element('editableNetsField').hidden,true);
 element('nativeMode').value='repair';element('nativeMode').onchange();
 assert.equal(element('protectedNetsField').hidden,false);assert.equal(element('editableNetsField').hidden,true);
 element('nativeMode').value='improve';element('nativeMode').onchange();
 assert.equal(element('protectedNetsField').hidden,false);assert.equal(element('editableNetsField').hidden,false);
 console.log('Native continuation UI: blocked/pass visibility, repair/improve source_job bodies and uploaded repair passed');
})().catch(error=>{console.error(error);process.exitCode=1});
