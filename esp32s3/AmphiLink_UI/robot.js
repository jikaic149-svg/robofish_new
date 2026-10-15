'use strict';
let robotState=null,robotIdentity=null,robotPending=false,robotError='',robotSeen=0;
let stationMode='mission',stationEpoch=0,modePending=false,robotStreaming=false,identityPending=false,identityEpoch=0;
const ROBOT_STALE_MS=5000;
const MODE_LABELS={mission:'任务 · LoRa',bench:'台架 · HTTP/SPI',simulation:'仿真 · PC模型'};
function robotAddress(value){
  const parts=String(value||'').trim().split(':');
  if(parts.length>2||!validIP(parts[0]||''))return '';
  if(parts.length===2&&(!/^\d+$/.test(parts[1])||+parts[1]<1||+parts[1]>65535))return '';
  return parts[0]+(parts.length===2?':'+Number(parts[1]):'');
}
function robotHost(){
  const typed=$('robot-host').value.trim();
  if(typed)return robotAddress(typed);
  const discovered=Object.keys(probeLinks.http||{}).filter(ip=>probeLinks.http[ip].state==='online');
  return robotAddress($('management-ip').value)||robotAddress($('host-input').value)||(discovered.length===1?discovered[0]:'');
}
function robotText(id,value){const el=$(id);if(el)el.textContent=value;}
function robotNumber(value,suffix='',digits=0){return typeof value==='number'&&Number.isFinite(value)?value.toFixed(digits)+suffix:'—';}
function robotBadge(id,label,tone='unknown'){const el=$(id);if(el){el.textContent=label;el.dataset.state=tone;}}
function robotMeter(id,value,min,max,centered=false){
  const el=$(id);if(!Number.isFinite(value)){el.style.width='0%';return;}
  const ratio=Math.max(0,Math.min(1,(value-min)/(max-min)));
  el.style.left=centered?(ratio<.5?ratio*100:50)+'%':'0';el.style.width=(centered?Math.abs(ratio-.5):ratio)*100+'%';
}
function currentRobot(){return robotState&&robotSeen&&Date.now()-robotSeen<=ROBOT_STALE_MS?robotState:null;}
function renderRobot(){
  const view=currentRobot(),valid=!!view?.online,battery=view?.battery||{},motion=view?.motion||{},act=valid?view.actuators||{}:{};
  const simulated=stationMode==='simulation',source=view?.telemetry_source||({mission:'lora',bench:'http_spi',simulation:'simulation'})[stationMode];
  $('source-label').textContent=MODE_LABELS[stationMode];
  $('source-description').textContent=robotError||(simulated?'离线测试，未连接实物。':stationMode==='bench'?'台架遥测；运动控制仍走LoRa。':'LoRa发送/上行未接入，实物控制不可用。');
  document.querySelector('.source-strip').dataset.source=source;
  robotText('source-age',robotSeen?new Date(robotSeen).toLocaleTimeString('zh-CN',{hour12:false}):'未收到数据');
  robotBadge('global-source',simulated?'仿真模式':stationMode==='bench'?'台架只读':'LoRa待接入',simulated?'warning':valid?'online':'unknown');
  const batteryValid=valid&&battery.valid;
  robotText('robot-battery-voltage',batteryValid?robotNumber(battery.voltage,' V',2):'—');
  robotText('robot-battery-age',batteryValid&&Number.isFinite(battery.age_ms)?battery.age_ms+' ms前':'没有有效采样');
  robotBadge('robot-battery-state',batteryValid?battery.condition_label:'未知',batteryValid?(battery.condition==='normal'?'online':'error'):'unknown');
  robotText('robot-action',valid?motion.action_label||'—':'—');
  robotText('robot-mode-sub',valid?`${motion.mode_label||'未知'} · 输入 ${robotNumber(motion.power,'%')}`:'等待有效遥测');
  robotText('owner-value',({none:'等待指令',lora:'LoRa',wifi:'2.4G',vision:'视觉',unknown:'未知'})[view?.owner||'unknown']);
  robotText('robot-link-detail',valid?`收 ${view.link?.received||0} / 发 ${view.link?.sent||0} 帧`:'LoRa › 2.4G › 视觉');
  const mode=view?.safety_mode||'UNKNOWN';
  robotText('robot-safety-verdict',({UNKNOWN:'未知',WAIT_COMMAND:'等待指令',RUNNING:'运行中',SURFACE:'浮力上浮',BATTERY_STOP:'电池停机'})[mode]||mode);
  robotText('robot-safety-detail',mode==='UNKNOWN'?'安全原因尚未由固件上报':mode==='SURFACE'?'双桨中立 · 舵机保持':mode==='BATTERY_STOP'?'全部中立 · 等待新指令':mode==='RUNNING'?'目标生效':'等待新的有效目标');
  robotText('reason-value',robotError||view?.reason||'等待状态上报');
  robotText('robot-link-verdict',valid?'遥测在线':view?.reason||'没有有效遥测');
  robotBadge('robot-failsafe-state',mode==='UNKNOWN'?'状态未知':mode==='SURFACE'||mode==='BATTERY_STOP'?'保护生效':'正常',mode==='UNKNOWN'?'unknown':mode==='SURFACE'||mode==='BATTERY_STOP'?'warning':'online');
  robotBadge('robot-link-flag',valid?(simulated?'仿真数据':'台架遥测'):'无有效数据',valid?'online':'unknown');
  for(const [id,tone] of [['robot-hero-battery',batteryValid?(battery.condition==='normal'?'good':'bad'):'warn'],['robot-hero-motion',valid?'good':'warn'],['robot-hero-link',view?.owner&&view.owner!=='unknown'?'good':'warn'],['robot-hero-safety',mode==='BATTERY_STOP'?'bad':mode==='UNKNOWN'||mode==='SURFACE'?'warn':'good']])$(id).dataset.tone=tone;
  for(const [key,label,unit] of [['left_fin','left-fin','°'],['right_fin','right-fin','°'],['tail1','tail1','°'],['tail2','tail2','°'],['left_esc','left-esc','%'],['right_esc','right-esc','%']]){
    robotText('robot-'+label,robotNumber(act[key],unit));robotMeter('robot-'+label+'-meter',act[key],unit==='°'?0:-100,unit==='°'?180:100,unit==='%');
  }
  robotText('robot-pitch',robotNumber(act.pitch,'°'));
  robotText('robot-age',valid?Math.max(0,Math.floor((Date.now()-robotSeen)/1000))+'秒前':'—');
  robotText('lora-state',simulated?(view?.links?.lora?'可用 · 仿真':'断开 · 仿真'):'待接入');
  robotText('wifi-state',simulated?(view?.links?.wifi?'可用 · 仿真':'断开 · 仿真'):'备用控制待接入');
  robotText('spi-state',simulated?(view?.links?.spi?'可用 · 仿真':'断开 · 仿真'):stationMode==='bench'?(valid?'在线':'离线 / 未知'):'台架模式可检查');
  document.querySelectorAll('[data-axis]').forEach(el=>el.disabled=!simulated||modePending||$('control-source').value==='vision');
  $('control-stop').disabled=$('robot-control-mode').disabled=!simulated||modePending;
  robotBadge('robot-gate-badge',simulated?'仿真可控制':'LoRa发送端待接入',simulated?'warning':'unknown');
  robotText('robot-gate-note',simulated?'离线测试：输出由STM32原分配算法计算。':'LoRa发送端未接入，未向实物发送。');

  $('robot-sim-panel').hidden=!simulated;
  if(simulated&&view){for(const key of ['lora','wifi','spi','upgrading'])$('sim-'+key).checked=!!view.links[key];}
  renderRobotSession();
  if(typeof renderControl==='function')renderControl();
  if(typeof renderCapture==='function')renderCapture();
}
function renderRobotSession(){
  const host=robotHost();robotText('robot-device-host',host||'—');robotText('robot-session-host',stationMode==='simulation'?'SIMULATION / PC模型':host||'未选择管理设备');
  if(!host){robotBadge('robot-device-badge','找不到设备','error');robotText('robot-device-detail','未自动发现设备；连接热点后填写IP，或使用测试模式。');}
  else if(!robotIdentity){robotBadge('robot-device-badge','未连接','unknown');robotText('robot-device-detail','找不到设备，或尚未检测；点击“检测”检查此地址。');}
  else{robotBadge('robot-device-badge',robotIdentity.authenticated?'已登录':'设备在线',robotIdentity.authenticated?'online':'unknown');robotText('robot-device-detail',robotIdentity.authenticated?'管理会话已建立；运动控制仍走LoRa。':'设备可达，请输入管理密钥登录。');}
  for(const [id,key] of [['robot-version','version'],['robot-slot','running_slot'],['robot-ssid','ssid']])robotText(id,robotIdentity?.[key]||'—');
  $('login-saved').disabled=!robotIdentity?.key_saved;
}
async function pollIdentity(){
  if(identityPending)return;const host=robotHost(),epoch=identityEpoch;
  if(stationMode==='simulation'){robotIdentity=null;renderRobotSession();return;}
  if(!host){robotIdentity=null;renderRobotSession();return;}
  identityPending=true;
  try{const reply=await service('device/identity?host='+encodeURIComponent(host));if(epoch===identityEpoch)robotIdentity=reply;}
  catch(e){if(epoch===identityEpoch){robotIdentity=null;robotText('robot-login-state',e.message);}}
  finally{identityPending=false;renderRobotSession();}
}
async function pollRobot(){
  if(location.protocol==='file:'||robotPending)return;robotPending=true;const epoch=stationEpoch;
  try{const view=await service('station');if(epoch!==stationEpoch)return;robotState=view;stationMode=view.mode||stationMode;$('station-mode').value=stationMode;robotError='';robotSeen=Date.now();}
  catch(e){if(epoch===stationEpoch){robotState=null;robotSeen=0;robotError=e.message;}}
  finally{robotPending=false;renderRobot();}
}
async function switchStationMode(mode){
  if(modePending)return;modePending=true;stationEpoch++;releaseControl();robotState=null;robotSeen=0;renderRobot();
  try{const view=await service('station/mode',{mode,host:robotHost()});stationMode=mode;robotState=view;robotSeen=Date.now();robotError='';log('切换工作模式：'+MODE_LABELS[mode],mode==='simulation'?'warn':'info');}
  catch(e){toast(e.message);log(e.message,'warn');}
  finally{modePending=false;$('station-mode').value=stationMode;renderRobot();await pollRobot();}
}
async function robotLogin(useSaved=false){
  const host=robotHost();if(!host)return toast('请先填写有效设备IP');const password=$('robot-password').value;
  if(!useSaved&&!password)return toast('请输入设备管理密钥');
  try{await service('device/login',{host,password,use_saved:useSaved,remember:$('robot-remember').checked});robotText('robot-login-state','管理登录成功');log('已登录管理设备 '+host,'ok');await pollIdentity();}
  catch(e){robotText('robot-login-state',e.message);toast(e.message);}
  finally{$('robot-password').value='';}
}

const controlSession=crypto.randomUUID();let controlSequence=0,controlQueue=Promise.resolve(),desiredAxes={surge:0,yaw:0,heave:0},desiredSource='lora',inputVersion=0,sentVersion=0,targetPending=false,lastRenewal=0;
const heldKeys=new Set(),heldPointers=new Map();
const axisBindings={forward:[1,0,0],back:[-1,0,0],left:[0,-1,0],right:[0,1,0],up:[0,0,1],down:[0,0,-1],'front-left':[1,-1,0],'front-right':[1,1,0],'back-left':[-1,-1,0],'back-right':[-1,1,0]};
const keyboardAxes={w:'forward',s:'back',a:'left',d:'right',r:'up',f:'down'};
let robotHistory=[];
function renderControl(){
 const actual=currentRobot()?.control?.axes;
 robotText('control-axis-value',actual?`已确认：前后 ${actual.surge} · 转向 ${actual.yaw} · 浮沉 ${actual.heave}`:'目标尚未确认');
 document.querySelectorAll('[data-axis]').forEach(button=>button.classList.toggle('pressed',[...heldKeys,...heldPointers.values()].includes(button.dataset.axis)));
}
function queueControl(callback){const result=controlQueue.then(callback);controlQueue=result.catch(()=>{});return result;}
async function configureControl(action,value){
 try{const result=await queueControl(()=>service('station/control',{action,value,session:controlSession,sequence:++controlSequence}));robotText('control-configuration-state',result.message);log(result.message,'ok');await pollRobot();}
 catch(error){robotText('control-configuration-state',error.message);toast(error.message);$('robot-control-mode').value=String(currentRobot()?.control?.mode||3);$('control-source').value=currentRobot()?.control?.source||'lora';}
}
function updateAxes(sourceOverride){
 const total=[0,0,0];for(const key of [...heldKeys,...heldPointers.values()]){const vector=axisBindings[key];if(vector)vector.forEach((value,i)=>total[i]+=value);}
 desiredAxes={surge:Math.sign(total[0])*Number($('robot-control-power').value),yaw:Math.sign(total[1])*Number($('robot-control-strength').value),heave:Math.sign(total[2])*100};desiredSource=sourceOverride||$('control-source').value;inputVersion++;pumpTarget();renderControl();
}
function releaseControl(sourceOverride){heldKeys.clear();heldPointers.clear();updateAxes(sourceOverride);}
async function pumpTarget(){
 if(targetPending||modePending)return;
 const active=Object.values(desiredAxes).some(value=>value!==0),now=Date.now();
 if(inputVersion===sentVersion&&(!active||now-lastRenewal<150))return;
 targetPending=true;const version=inputVersion,axes={...desiredAxes},source=desiredSource,epoch=stationEpoch;
 try{const result=await queueControl(()=>version!==inputVersion?{skipped:true}:service('station/target',{axes,source,session:controlSession,sequence:++controlSequence}));
   if(result.skipped)return;
   if(epoch===stationEpoch){sentVersion=version;lastRenewal=Date.now();robotText('robot-command-result',`目标已确认（仿真） #${result.id}`);if(version===inputVersion)await pollRobot();}
 }catch(error){sentVersion=version;heldKeys.clear();heldPointers.clear();desiredAxes={surge:0,yaw:0,heave:0};robotText('robot-command-result',error.message);log(error.message,'warn');renderControl();}
 finally{targetPending=false;if(inputVersion!==sentVersion)pumpTarget();}
}
function renderRobotHistory(){$('robot-history').replaceChildren(...robotHistory.slice(-12).reverse().map(entry=>{const row=document.createElement('div');row.textContent=entry;return row;}));}
$('station-mode').onchange=e=>switchStationMode(e.target.value);
$('robot-login').onclick=()=>robotLogin();$('login-saved').onclick=()=>robotLogin(true);$('robot-refresh').onclick=pollIdentity;
$('forget-key').onclick=async()=>{try{await service('device/forget',{host:robotHost()});robotIdentity=null;robotText('robot-login-state','已删除该设备的本机密钥');await pollIdentity();}catch(error){toast(error.message);}};
let hostTimer;$('robot-host').addEventListener('input',()=>{identityEpoch++;robotIdentity=null;clearTimeout(hostTimer);hostTimer=setTimeout(pollIdentity,400);renderRobotSession();});
$('robot-control-mode').onchange=e=>configureControl('mode',Number(e.target.value));
$('control-source').onchange=e=>{if(heldKeys.size||heldPointers.size)releaseControl(desiredSource);configureControl('source',e.target.value);};
for(const [id,output] of [['robot-control-power','robot-power-value'],['robot-control-strength','robot-strength-value']])$(id).oninput=()=>{robotText(output,$(id).value+'%');if(heldKeys.size||heldPointers.size)updateAxes();};
document.querySelectorAll('[data-axis]').forEach(button=>{
 button.onpointerdown=event=>{event.preventDefault();button.setPointerCapture(event.pointerId);heldPointers.set(event.pointerId,button.dataset.axis);updateAxes();};
 const end=event=>{if(heldPointers.delete(event.pointerId))updateAxes();};button.onpointerup=end;button.onpointercancel=end;button.onlostpointercapture=end;
});
$('control-stop').onclick=()=>{releaseControl($('control-source').value==='vision'?'lora':undefined);robotHistory.push(new Date().toLocaleTimeString()+' 全停');renderRobotHistory();};
$('robot-clear-history').onclick=()=>{robotHistory=[];renderRobotHistory();};
window.addEventListener('keydown',event=>{if(state.page!=='overview'||/INPUT|TEXTAREA|SELECT/.test(event.target.tagName))return;if(event.code==='Space'){event.preventDefault();if(!event.repeat)$('control-stop').click();return;}const key=keyboardAxes[event.key.toLowerCase()];if(key&&!event.repeat&&stationMode==='simulation'&&$('control-source').value!=='vision'){event.preventDefault();heldKeys.add(key);updateAxes();}});
window.addEventListener('keyup',event=>{const key=keyboardAxes[event.key.toLowerCase()];if(key&&heldKeys.delete(key))updateAxes();});
window.addEventListener('blur',()=>{if(heldKeys.size||heldPointers.size)releaseControl();});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&(heldKeys.size||heldPointers.size))releaseControl();});
window.addEventListener('stationpage',event=>{if(event.detail!=='overview'&&(heldKeys.size||heldPointers.size))releaseControl();if(event.detail==='network')pollIdentity();});
for(const key of ['lora','wifi','spi','upgrading'])$('sim-'+key).onchange=async event=>{try{robotState=await service('station/simulate',{changes:{[key]:event.target.checked}});robotSeen=Date.now();renderRobot();}catch(error){toast(error.message);}};
$('robot-sim-fault').onchange=async event=>{try{robotState=await service('station/simulate',{changes:{battery:event.target.value}});robotSeen=Date.now();renderRobot();}catch(error){toast(error.message);}};
setInterval(()=>{if(stationMode==='simulation')pumpTarget();},75);
setInterval(pollRobot,1000);setInterval(()=>{renderRobot();if(state.page==='network')pollIdentity();},4000);
renderRobot();pollRobot();
