const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
class Element{constructor(){this.value='';this.disabled=false;this.hidden=false;this.files=[];this.children=[];this.style={}}append(...x){this.children.push(...x)}replaceChildren(...x){this.children=x}}
const nodes=new Map(),element=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id)};element('nativeMode').value='repair';
const requests=[],binary=Uint8Array.from([80,75,3,4,0,127,128,255]);
const context=vm.createContext({URLSearchParams,Uint8Array,setTimeout,btoa:x=>Buffer.from(x,'binary').toString('base64'),location:{search:''},history:{replaceState(){}},document:{getElementById:element,createElement:()=>new Element()},fetch:async(url,options)=>{if(url==='/api/native-route'){requests.push(JSON.parse(options.body));return{ok:true,json:async()=>({id:'a'.repeat(32)})}}if(url.startsWith('/api/job/'))return{ok:true,json:async()=>({state:'done',stage:'Blocked',review_pass:false,elapsed_seconds:1})};throw Error(url)}});
const script=path.join(__dirname,'../scripts/workbench/native.js');vm.runInContext(fs.readFileSync(script,'utf8'),context,{filename:script});
(async()=>{
 element('protectedNets').value='CLOCK';element('files').files=[{name:'project.zip',size:binary.length,arrayBuffer:async()=>binary.buffer,text:async()=>{throw Error('Binary ZIP read as text')}}];await element('nativeRoute').onclick();
 assert.equal(requests.length,1);assert.deepEqual(requests[0],{mode:'repair',editable_nets:null,protected_nets:['CLOCK'],project_zip:{name:'project.zip',base64:Buffer.from(binary).toString('base64')}});
 element('files').files.push({name:'board.kicad_pcb',size:5,text:async()=>'(board)'});await element('nativeRoute').onclick();assert.equal(requests.length,1);assert.match(element('nativeStatus').textContent,/ZIP on its own/);
 element('files').files=[{name:'too-big.zip',size:16_000_001,arrayBuffer:async()=>{throw Error('Oversized ZIP read')}}];await element('nativeRoute').onclick();assert.equal(requests.length,1);assert.match(element('nativeStatus').textContent,/upload limit/);
 element('files').files=[{name:'board.kicad_pcb',size:5,text:async()=>'(board)'},{name:'board.kicad_pro',size:2,text:async()=>'{ }'}];await element('nativeRoute').onclick();assert.equal(requests.length,2);assert.equal(requests[1].project_zip,undefined);assert.deepEqual(requests[1].files,[{name:'board.kicad_pcb',text:'(board)'},{name:'board.kicad_pro',text:'{ }'}]);
 console.log('ZIP binary request, mixed/oversized rejection and unchanged loose upload passed');
})().catch(e=>{console.error(e);process.exitCode=1});
