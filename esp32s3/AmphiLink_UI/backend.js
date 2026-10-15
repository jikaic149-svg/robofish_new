'use strict';
let serviceToken='', liveState={connected:false}, liveBusy=false, lastLog='', liveEpoch=0, pollPending=false;
async function service(path,body) {
  if(location.protocol==='file:')throw Error('请使用 start.cmd 启动本地服务，再从 http://127.0.0.1:8765 打开上位机');
  if(body!==undefined&&!serviceToken)await service('session');
  const controller=new AbortController(), timer=setTimeout(()=>controller.abort(),path==='development/task'?180000:path==='vision/start'?30000:path==='probe-upgrade'?200000:(path==='flash'||path==='erase')?600000:path==='connect'?60000:((path==='links'||path==='state')?6000:20000));
  try{
    const response=await fetch('/api/'+path,{method:body===undefined?'GET':'POST',headers:{'Content-Type':'application/json','X-AmphiLink-Token':serviceToken},body:body===undefined?undefined:JSON.stringify(body),signal:controller.signal});
    const data=await response.json();
    if(!response.ok){if(data.logs)recordLogs(data.logs);throw Error(data.error||'设备服务请求失败');}
    if(data.token)serviceToken=data.token;
    return data;
  }catch(error){if(error.name==='AbortError')throw Error('请求超时，结果尚未确认；请检查连接与日志');if(error instanceof TypeError)throw Error('本地服务不可达，请双击桌面图标重新启动');throw error;}finally{clearTimeout(timer);}
}
function recordLogs(lines){const start=lastLog?lines.lastIndexOf(lastLog)+1:0;for(const line of lines.slice(start))log(line);lastLog=lines.at(-1)||lastLog;}
function drawLive(data){
  if('probe_enabled' in data)probeLinks.enabled=data.probe_enabled;
  targetReceived=Date.now();sharedBusy=false;
  liveState={...liveState,...data};
  if(!data.connected){liveState={...data};state.connected=false;state.halted=false;}
  else{state.connected=true;state.halted=data.target_state==='halted';}
  if(data.transport && data.session_active){
    state.transport=data.transport;
    document.querySelectorAll('[data-transport]').forEach(el=>el.classList.toggle('selected',el.dataset.transport===state.transport));
    $('usb-fields').hidden=state.transport!=='usb';$('wifi-fields').hidden=state.transport!=='wifi';
  }
  if(data.logs)recordLogs(data.logs);
  renderConnection();
}
function renderFlashActions(){
  const busy=liveBusy||state.flashing, connected=!!liveState.connected;
  $('start-flash').disabled=busy||!connected||!state.file;
  $('start-flash').textContent=state.flashing?'正在烧录…':!state.file?'选择固件后烧录':!connected?'连接目标后烧录':'开始烧录固件';
  $('erase-only').disabled=busy||!connected;
}
function renderConnection(){
  renderFlashActions();
  const c=!!liveState.connected, identity=liveState.identity||{}, target=liveState.target_state;
  const nativeState=window.nativeDevelopment?.status?.state;
  $('side-state').textContent=['paused','running'].includes(nativeState)?'STM32 · '+(nativeState==='paused'?'已暂停':'运行中'):c?'探针和目标已连接':'目标未连接';
  $('connection-badge').textContent=c?'真实连接':'未连接';
  $('connection-badge').classList.toggle('connected',c);
  $('connect-main').textContent=c?'断开并恢复运行':'连接目标';
  $('connect-card').textContent=c?'断开并恢复运行':'连接并短暂暂停识别';
  $('probe-stat').textContent=c?state.transport.toUpperCase()+' 已连接':'等待连接';
  $('probe-detail').textContent=c?'本地 OpenOCD · CMSIS-DAP':'选择 USB 或局域网 IP';
  $('target-stat').textContent=c?identity.name:'—';
  $('target-name').textContent=c?identity.name:'等待识别目标';
  $('target-description').textContent=c?((identity.uid||[]).length?'UID '+identity.uid.join(' '):identity.warning||'UID unavailable'):'连接后读取实际芯片信息';
  $('target-core').textContent=c?identity.core||'Cortex-M':'Cortex-M';
  $('target-id').textContent=c?identity.id:'—';
  $('target-flash').textContent=c?identity.flash_kib+' KiB':'—';
  $('target-badge').textContent=c?'实板读回':'未识别';
  const label=c?({halted:'已暂停',running:'运行中'}[target]||target):'未知';
  $('target-run').textContent=label;$('debug-state').textContent=label;
  $('latency-stat').textContent=c && liveState.latency_ms!==undefined?liveState.latency_ms+' ms':'—';
  renderLinks();
  renderRegisters();renderLiveBreaks();
  if(typeof syncDeveloperStatus==='function')syncDeveloperStatus();
};
function renderRegisters(){
  $('registers').replaceChildren();
  for(const name of ['r0','r1','r2','r3','r4','r5','r6','r7','r8','r9','r10','r11','r12','sp','lr','pc','xpsr']){
    const row=document.createElement('tr');
    for(const text of [name.toUpperCase(),state.halted?(liveState.registers?.[name]||'待读取'):'—','实板内核寄存器']){const cell=document.createElement('td');cell.textContent=text;row.append(cell);}
    $('registers').append(row);
  }
};
function renderLiveBreaks(){
  $('breakpoint-list').replaceChildren();
  for(const bp of liveState.breakpoints||[]){
    const row=document.createElement('div');row.className='breakpoint-row';
    const label=document.createElement('span');label.textContent=`0x${bp.address.toString(16)} · ${bp.kind==='write'?'写观察点':'硬件断点'}`;
    const remove=document.createElement('button');remove.className='text-button';remove.textContent='移除';remove.onclick=()=>operate('remove-breakpoint',{address:bp.address});row.append(label,remove);$('breakpoint-list').append(row);
  }
  if(!(liveState.breakpoints||[]).length)$('breakpoint-list').textContent='尚未添加实板断点';
}
async function operate(action,body={}){
  if(liveBusy)return toast('设备操作正在进行');
  liveBusy=true;liveEpoch++;renderConnection();
  try{
    if(!serviceToken){const initial=await service('state');drawLive(initial);}
    const result=await service(action,{...body,hardware:state.page==='debug'});
    if('connected'in result)drawLive(result);
    else if(result.registers){liveState.registers=result.registers;renderRegisters();}
    else if(result.words){$('memory-output').textContent=result.words.map((v,i)=>`0x${(result.address+i*4).toString(16).padStart(8,'0')}  0x${v.toString(16).padStart(8,'0')}`).join('\n');}
    log('实际操作完成：'+action,'ok');return result;
  }catch(e){
    toast(e.message);log(e.message,'error');
    if(e.name==='AbortError'||e instanceof TypeError)drawLive({connected:false,target_state:'unknown'});
    else{try{drawLive(await service('state'));}catch(_){drawLive({connected:false,target_state:'unknown'});}}
  }finally{liveBusy=false;renderConnection();}
}
async function connect(){
  return operate(liveState.session_active?'disconnect':'connect',{transport:state.transport,host:$('host-input').value.trim(),port:Number($('port-input').value),speed:parseInt($('adapter-speed').value)});
};
async function debug(action){return operate(action);}
async function flash(){
  if(!liveState.connected)return toast('请先连接并识别目标');
  if(!state.file)return toast('请先选择固件');
  if(state.file.size>4*1024*1024)return toast('固件文件不能大于 4 MiB');
  const erase=$('erase-mode').selectedIndex===1?'all':'covered';
  const scope=erase==='all'?'将擦除全部主 Flash，原程序及其中保存的数据都会清除，然后写入新固件。OTP 和选项字节不在擦除范围。':'仅擦除固件覆盖的扇区。';
  if(!confirm(`${liveState.identity?.name||'STM32'}：${state.file.name}\n${scope}\n继续吗？`))return;
  const bytes=new Uint8Array(await state.file.arrayBuffer());let binary='';
  for(let offset=0;offset<bytes.length;offset+=8192)binary+=String.fromCharCode(...bytes.subarray(offset,offset+8192));
  state.flashing=true;$('start-flash').disabled=true;$('flash-stage').textContent='OpenOCD 正在擦写，请查看实际日志';$('flash-progress').removeAttribute('value');$('flash-percent').textContent='进行中';
  try{
    const result=await operate('flash',{name:state.file.name,data:btoa(binary),address:$('flash-address').value,erase,verify:$('verify-flash').checked,run:$('run-after').checked});
    $('flash-progress').value=result?100:0;$('flash-percent').textContent=result?'100%':'未完成';$('flash-stage').textContent=result?($('verify-flash').checked?'写入并校验完成':'写入完成，未校验'):'烧录未确认完成，查看日志';
  }finally{state.flashing=false;renderFlashActions();}
};
$('connect-main').onclick=$('connect-card').onclick=()=>connect();
document.querySelectorAll('[data-debug]').forEach(el=>el.onclick=()=>debug(el.dataset.debug));
$('refresh-registers').onclick=()=>operate('registers');
$('memory-form').onsubmit=e=>{e.preventDefault();operate('memory',{address:$('memory-address').value});};
$('breakpoint-form').onsubmit=e=>{e.preventDefault();operate('breakpoint',{address:$('breakpoint-address').value,kind:$('breakpoint-type').selectedIndex?'write':'hardware'});};
$('start-flash').onclick=()=>flash();
$('erase-only').textContent='仅整片擦除（无需固件）';
$('erase-only').onclick=async()=>{
  if(liveBusy)return toast('设备操作正在进行');
  if(!liveState.connected)return toast('请先连接并识别目标');
  if(!confirm(`将擦除 ${liveState.identity?.name||'STM32'} 的全部主 Flash，原程序和其中的数据将被清除。\n不写入固件，不操作 OTP 和选项字节。擦除后保持暂停。\n确定执行吗？`))return;
  $('erase-only').disabled=true;
  try{await operate('erase',{});}finally{renderFlashActions();}
};
$('save-settings').onclick=()=>{try{localStorage.setItem('robofish-swd-speed',$('adapter-speed').value);}catch(_){}toast('SWD速率在下一次连接时应用；当前会话使用软件复位');};
try{const value=localStorage.getItem('robofish-swd-speed');if(value&&[...$('adapter-speed').options].some(o=>o.value===value))$('adapter-speed').value=value;}catch(_){}
$('erase-mode').options[1].disabled=false;
$('nrst-connected').disabled=true;$('quirk-mode').disabled=true;
$('quirk-mode').checked=false;
$('quirk-mode').parentElement.lastChild.textContent='额外适配器兼容处理（已关闭）';
$('probe-select').replaceChildren(new Option('XIAO ESP32-S3 · USB','xiao'));
$('host-input').value='';
try{const saved=localStorage.getItem('amphilink-probe-ip');if(validIP(saved||''))$('host-input').value=saved;}catch(_){}
setTransport($('host-input').value?'wifi':'usb');


$('info-dialog').querySelector('p').textContent='通过本地 OpenOCD 连接 ESP32 和 STM32。连接时短暂停核读取身份，然后恢复原状态；内存和寄存器读取需先暂停。调试和烧录会实际操作目标。多个窗口共用一个会话，断开会影响所有窗口并恢复运行。';
renderConnection();
if(location.protocol!=='file:')service('state').then(drawLive).catch(e=>log(e.message,'error'));
setInterval(async()=>{
  if(location.protocol==='file:'||pollPending)return;
  pollPending=true;const epoch=liveEpoch;
  try{
    if(liveBusy)recordLogs((await service('logs')).logs);
    else{const result=await service('state');if(epoch===liveEpoch&&!liveBusy)drawLive(result);}
  }catch(e){if(epoch===liveEpoch&&!liveBusy){if(e.message.includes('操作进行中')){sharedBusy=true;renderLinks();}else drawLive({connected:false,target_state:'unknown'});}}
  finally{pollPending=false;}
},2000);
