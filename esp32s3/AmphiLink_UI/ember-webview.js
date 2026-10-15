'use strict';
// Browser shim for EmberProbe's unmodified sidebar and chart renderer.
// No firmware values are synthesized. All native messages use the protected PC relay.
(()=>{
 const config=window.__RF_WEBVIEW__,key='rf-ember-view-'+config.id+'-'+config.generation;
 let token='',sequence=Number(sessionStorage.getItem(key+'-cursor')||0),queue=Promise.resolve(),localState={};
 try{localState=JSON.parse(sessionStorage.getItem(key)||'{}');}catch(_){}
 const notify=(message,level='error')=>{parent.postMessage({type:'rf-ember-notice',message:String(message),level},location.origin);};
 async function post(message){
  if(message.type==='executeCommand'&&message.cmd==='mcu-vscode.openLiveWatch'){parent.postMessage({type:'rf-ember-chart'},location.origin);return;}
  if(message.type==='exportCsv'&&typeof message.csv==='string'){
   const url=URL.createObjectURL(new Blob([message.csv],{type:'text/csv;charset=utf-8'}));
   const link=document.createElement('a');link.href=url;link.download='emberprobe-history.csv';link.click();
   setTimeout(()=>URL.revokeObjectURL(url),60000);
   window.dispatchEvent(new MessageEvent('message',{data:{type:'exportCsvResult',ok:true,rowCount:Math.max(0,message.csv.trimEnd().split('\n').length-1),seriesCount:message.names?.length||0}}));return;
  }
  if(message.type==='copyText'){
   try{await navigator.clipboard.writeText(String(message.text||''));return;}catch(_){}
  }
  if(!token){const response=await fetch('/api/session');token=(await response.json()).token;}
  const response=await fetch('/api/development/web-message',{method:'POST',headers:{'Content-Type':'application/json','X-AmphiLink-Token':token},body:JSON.stringify({id:config.id,message})});
  const result=await response.json();if(!response.ok)throw Error(result.error||'原生操作失败');
 }
 window.acquireVsCodeApi=()=>({
  postMessage(message){
   // Preserve ordering of writes and watch edits. Slider coalescing remains native.
   const result=queue.then(()=>post(message));queue=result.catch(error=>notify(error.message));
  },
  getState(){return localState;},
  setState(value){localState=value;sessionStorage.setItem(key,JSON.stringify(value));return value;}
 });
 const events=new EventSource('/api/development/events?id='+encodeURIComponent(config.id)+'&after='+sequence);
 events.onmessage=event=>{
  const item=JSON.parse(event.data);if(item.sequence<=sequence)return;sequence=item.sequence;sessionStorage.setItem(key+'-cursor',String(sequence));
  const message=item.message;
  if(message.type==='workstationReload'){location.reload();return;}
  if(message.type==='workstationResync'){post({type:config.kind==='sidebar'?'initCheck':'ready'}).catch(error=>notify(error.message));return;}
  if(message.type==='workstationDownload'){
   const link=document.createElement('a');link.href='/api/development/download?id='+encodeURIComponent(message.id);link.download=message.name;link.click();return;
  }
  window.dispatchEvent(new MessageEvent('message',{data:message,origin:location.origin}));
  if(['commandError','liveError','writeVariableResult'].includes(message.type)&&message.error)notify(message.error);
 };
 events.onerror=()=>{document.documentElement.dataset.transport='reconnecting';};
 events.onopen=()=>{document.documentElement.dataset.transport='online';};
 window.addEventListener('beforeunload',()=>events.close());
 const groups={variables:['liveValuesSection'],peripherals:['peripheralSection'],rtos:['rtosSection'],chip:['chipInfoSection'],memory:['memoryAnalysisSection'],config:['mcuConfigSection','openocdSetup','probeDiagnosticSection']};
 let sectionName='';
 function section(name){
  if(sectionName===name)return;sectionName=name;
  document.body.dataset.workstationSection=name;
  const selected=groups[name]||groups.variables;
  for(const id of Object.values(groups).flat()){
   const element=document.getElementById(id);if(element){element.hidden=!selected.includes(id);if(selected.includes(id)&&element.tagName==='DETAILS')element.open=true;}
  }
  const memory=document.getElementById('memoryAnalysisSection');
  if(memory&&memory.parentElement.id==='flashDebugSection')document.body.append(memory);
  const nativeFlash=document.getElementById('flashDebugSection');if(nativeFlash)nativeFlash.hidden=true;
  if(name==='rtos')document.getElementById('rtosRefresh')?.click();
  if(name==='memory')document.getElementById('memoryRefresh')?.click();
  if(name==='peripherals')document.getElementById('peripheralRefresh')?.click();
  resize();
 }
 function resize(){parent.postMessage({type:'rf-ember-height',id:config.id,height:Math.min(1600,Math.max(420,document.body.scrollHeight))},location.origin);}
 window.addEventListener('message',event=>{
  if(event.source!==parent||event.origin!==location.origin)return;
  if(event.data?.type==='rf-ember-section')section(event.data.section);
  if(event.data?.type==='rf-ember-theme')document.documentElement.dataset.theme=event.data.theme;
 });
 window.addEventListener('DOMContentLoaded',()=>{
  document.body.classList.add('workstation-native',config.kind==='chart'?'workstation-chart':'workstation-sidebar');
  if(config.kind==='sidebar')section('variables');
  new ResizeObserver(resize).observe(document.body);
  parent.postMessage({type:'rf-ember-ready',id:config.id},location.origin);
 });
})();
