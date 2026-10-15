'use strict';
let progressRevision=0, progressPending=false;
function drawProgress(p){
  if(!p.kind || p.revision<=progressRevision)return;
  progressRevision=p.revision;
  const prefix=p.kind==='esp32'?'release':'flash';
  const bar=$(prefix+'-progress'), label=$(prefix+'-stage'), percent=$(prefix+'-percent');
  if(!bar)return;
  label.textContent=p.phase+(p.detail?' · '+p.detail:'');
  if(p.percent===null || p.percent===undefined){
    if(p.state==='failed'){bar.value=0;percent.textContent='未完成';}
    else{bar.removeAttribute('value');percent.textContent='进行中';}
  }else{bar.value=p.percent;percent.textContent=(p.state==='complete'?'完成 ': '本阶段 ')+p.percent+'%';}
  bar.setAttribute('aria-label',label.textContent);
}
setInterval(async()=>{
  if(location.protocol==='file:'||progressPending)return;
  progressPending=true;
  try{drawProgress(await service('progress'));}catch(_){}
  finally{progressPending=false;}
},500);
