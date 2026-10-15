'use strict';
const $ = id => document.getElementById(id);
const pages = {
 overview:['操控','操控',''], vision:['视觉','视觉',''], data:['数据','数据',''],
 debug:['开发','开发',''], network:['设备设置','设备设置','']
};
const state={connected:false,transport:'usb',halted:false,file:null,flashing:false,page:'overview',logs:[]};
let toastTimer;
function toast(message){$('toast').textContent=message;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,5200);}
function log(message,level='info'){
  const now=new Date();
  state.logs.push({time:now.toLocaleTimeString('zh-CN',{hour12:false}),timestamp:now.toISOString(),level,message:String(message)});
  state.logs=state.logs.slice(-2000);renderLogs();
  if(typeof renderJournal==='function')renderJournal();
}
function logRow(row,full=false){
  const div=document.createElement('div');div.className='log-row '+row.level;
  const time=document.createElement('time');time.textContent=full?row.timestamp.replace('T',' ').slice(0,23)+'Z':row.time;
  const level=document.createElement('span');level.className='log-level';level.textContent=row.level.toUpperCase();
  const text=document.createElement('span');text.textContent=row.message;div.append(time,level,text);return div;
}
function renderLogs(){$('log-count').textContent=state.logs.length;}
function showLogs(open=true){$('log-drawer-body').hidden=!open;$('toggle-log').setAttribute('aria-expanded',String(open));if(typeof renderJournal==='function')renderJournal();}

function go(page){
  if(page==='journal'){showLogs();return;}if(page==='flash'){page='debug';$('legacy-maintenance').open=true;}if(page==='settings'){page='network';document.querySelector('#page-network details').open=true;}if(page==='robot')page='overview';
  if(!pages[page])page='overview';state.page=page;document.body.dataset.workspace=page;
  document.querySelectorAll('.page').forEach(el=>el.hidden=el.id!==`page-${page}`);
  document.querySelectorAll('nav button').forEach(el=>el.classList.toggle('active',el.dataset.page===page));
  $('page-title').textContent=pages[page][0];document.title='机器鱼工作站 · '+pages[page][0];
  if(location.hash!==`#${page}`)history.replaceState(null,'',`#${page}`);
  window.dispatchEvent(new CustomEvent('stationpage',{detail:page}));
}
function robotTab(tab){go(({status:'overview',control:'overview',camera:'vision',debug:'debug'})[tab]||'overview');}
function applyHash(){const [page,tab]=location.hash.slice(1).split('/');if(page==='robot'&&tab)robotTab(tab);else go(page);}
function validIP(value){const parts=value.split('.');return parts.length===4&&parts.every(x=>/^\d{1,3}$/.test(x)&&+x<=255);}
function validAddress(value){return /^0x[\da-f]{1,8}$/i.test(value.trim());}
function setTransport(kind){
  if(state.connected||['paused','running'].includes(window.nativeDevelopment?.status?.state))return toast('请先断开当前目标，再切换连接方式');state.transport=kind;
  document.querySelectorAll('[data-transport]').forEach(el=>el.classList.toggle('selected',el.dataset.transport===kind));
  $('usb-fields').hidden=kind!=='usb';$('wifi-fields').hidden=kind!=='wifi';
  $('link-label').textContent=kind==='usb'?'USB CMSIS-DAP':'TCP / LOCAL NETWORK';
  $('connect-hint').textContent=kind==='usb'?'USB连接用于烧录和调试。设备配网仍需连接热点或局域网。':'电脑与探针需要网络互通；IP可能因DHCP变化。';
}
function selectFile(file){
  if(!file)return;if(!/\.(bin|elf|hex)$/i.test(file.name))return toast('请选择BIN、ELF或HEX文件');
  if(!file.size)return toast('不能使用空文件');if(state.flashing)return toast('烧录进行中，不能更换固件');
  state.file=file;$('file-name').textContent=file.name;$('file-size').textContent=`${(file.size/1024).toFixed(1)} KiB · 文件尚未写入`;
  const bin=/\.bin$/i.test(file.name);$('flash-address').disabled=!bin;
  $('flash-address').value=!bin?'由镜像决定':/^RoboFishF405_B\.bin$/i.test(file.name)?'0x08080000':/^RoboFishF405\.bin$/i.test(file.name)?'0x08010000':'0x08000000';
  $('flash-stage').textContent='固件已选择';log(`已选择固件 ${file.name}，地址与启动记录需核对`);renderFlashActions();
}
function downloadBlob(blob,name){const url=URL.createObjectURL(blob);const link=document.createElement('a');link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
function exportLogs(format='json'){
  const text=format==='csv'?'\ufefftimestamp,level,message\r\n'+state.logs.map(row=>[row.timestamp,row.level,row.message].map(v=>'"'+String(v).replace(/^[=+@-]/,"'$&").replaceAll('"','""')+'"').join(',')).join('\r\n'):JSON.stringify({time_zone:'UTC',entries:state.logs},null,2);
  downloadBlob(new Blob([text],{type:format==='csv'?'text/csv;charset=utf-8':'application/json'}),`robofish-log.${format}`);
}
document.querySelectorAll('[data-page]').forEach(el=>el.onclick=()=>go(el.dataset.page));
document.querySelectorAll('[data-go]').forEach(el=>el.onclick=()=>el.dataset.tab?robotTab(el.dataset.tab):go(el.dataset.go));
document.querySelectorAll('[data-transport]').forEach(el=>el.onclick=()=>setTransport(el.dataset.transport));
document.querySelectorAll('[data-firmware-tab]').forEach(el=>el.onclick=()=>{
  const tab=el.dataset.firmwareTab;document.querySelectorAll('[data-firmware-tab]').forEach(b=>b.classList.toggle('selected',b===el));
  $('firmware-stm32').hidden=tab!=='stm32';$('firmware-esp32').hidden=tab!=='esp32';
});
$('firmware-file').onchange=e=>selectFile(e.target.files[0]);
$('dropzone').ondragover=e=>{e.preventDefault();$('dropzone').classList.add('dragover');};
$('dropzone').ondragleave=()=>$('dropzone').classList.remove('dragover');
$('dropzone').ondrop=e=>{e.preventDefault();$('dropzone').classList.remove('dragover');selectFile(e.dataTransfer.files[0]);};
$('clear-log').onclick=()=>{state.logs=[];renderLogs();if(typeof renderJournal==='function')renderJournal();};
$('toggle-log').onclick=()=>showLogs($('log-drawer-body').hidden);
$('export-log').onclick=()=>exportLogs();
$('help').onclick=()=>$('info-dialog').showModal();$('close-dialog').onclick=$('dialog-confirm').onclick=()=>$('info-dialog').close();
$('theme-toggle').onclick=()=>{document.body.dataset.theme=document.body.dataset.theme==='dark'?'light':'dark';try{localStorage.setItem('robofish-theme',document.body.dataset.theme);}catch(_){}window.dispatchEvent(new Event('stationtheme'));};
try{document.body.dataset.theme=localStorage.getItem('robofish-theme')||'light';}catch(_){}
window.addEventListener('hashchange',applyHash);applyHash();log('机器鱼工作站已加载，等待本地服务。');
