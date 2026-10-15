'use strict';
let probeLinks={}, linksReceived=0, targetReceived=0, sharedBusy=false, linksPending=false;
function renderLinks(){
  const now=Date.now(), fresh=now-linksReceived<7000, busy=liveBusy||sharedBusy;
  const targetFresh=now-targetReceived<7000 && !!liveState.checked_at;
  const transport=liveState.session_active?liveState.transport:state.transport;
  const host=liveState.session_active?liveState.host:$('host-input').value.trim();
  const stamp=probeLinks.wifi?.[host];
  let pc=transport==='usb'?(probeLinks.usb||{}):{
    state:stamp && now-stamp*1000<10000?'online':'unknown',checked_at:stamp,
    detail:stamp && now-stamp*1000<10000?`Wi-Fi · ${host} · 收到探针广播`:`Wi-Fi · ${host||'请填写 IP'} · 未收到近期广播，可能离线或广播受限`
  };
  if(transport==='wifi'&&probeLinks.wifi_error)pc={state:'unknown',detail:'广播检测不可用：'+probeLinks.wifi_error};
  const http=probeLinks.http?.[host];
  if(transport==='wifi' && http && now-http.checked_at*1000<10000){
    if(http.state==='online')pc={...http,detail:`Wi-Fi · ${host} · ${http.ssid||'局域网'} · HTTP 已连接`};
    else if(pc.state!=='online')pc={...http,detail:`Wi-Fi · ${host} · 找不到设备：${http.detail}`};
  }
  if(targetFresh && liveState.connected)pc={state:'online',checked_at:liveState.checked_at,detail:`${transport==='usb'?'USB':'Wi-Fi · '+host} · SWD 命令应答确认`};
  if(!fresh)pc={state:'unknown',detail:'本地服务未响应，连接状态未知'};
  if(pc.checked_at && now-pc.checked_at*1000>10000)pc.state='unknown';
  const paused=fresh && probeLinks.enabled===false;
  if(paused)pc={state:'disconnected',detail:'已停止上位机连接与检测；ESP32 仍连接路由器'};
  const ts=!fresh?'unknown':busy&&targetFresh&&liveState.connected?'online':busy?'busy':!targetFresh?'unknown':liveState.target_link||'unverified';
  const labels={online:'已连接',offline:'未检测到',unknown:'状态未知',unverified:'未检测',error:'通信异常',busy:'操作中 · 待更新',disconnected:'已手动断开'};
  $('probe-link-toggle').textContent=paused?'重新连接':'断开连接';
  $('probe-link-toggle').disabled=!fresh||busy;
  $('probe-link-toggle').title='对所有上位机窗口生效；断开时结束调试并恢复目标运行';
  $('connect-main').disabled=$('connect-card').disabled=paused||busy;
  if(paused)$('management-result').textContent='探针已手动断开，点击实时连接卡片中的“重新连接”恢复自动检测。';
  $('pc-link').dataset.state=pc.state||'unknown';
  $('pc-link-state').textContent=labels[pc.state]||'检测中';
  $('pc-link-detail').textContent=pc.detail||'正在检测 USB';
  $('stm-link').dataset.state=ts;
  $('stm-link-state').textContent=ts==='online'?'已连接 · '+(busy?'执行中':liveState.target_state==='halted'?'已暂停':'运行中'):labels[ts];
  $('stm-link-detail').textContent=ts==='online'?`${liveState.identity?.name||'STM32'} · SWD 轮询成功`:ts==='busy'?'烧录或调试操作正在进行，暂不轮询目标':ts==='error'?liveState.target_error:ts==='unverified'?'尚未建立调试会话，请点击“连接目标”；不能据此判断接线断开':'检测结果不可用，等待服务更新';
  if(busy&&ts==='online')$('stm-link-detail').textContent='正在执行命令 · 保持当前会话，完成后更新状态';
  const timeLabel=stamp=>stamp?`${new Date(stamp*1000).toLocaleTimeString('zh-CN',{hour12:false})} · ${Math.max(0,Math.floor((now-stamp*1000)/1000))} 秒前`:'尚无记录';
  $('pc-link-time').textContent='最近检测：'+timeLabel(pc.checked_at);
  $('stm-link-time').textContent='最近检测：'+timeLabel(liveState.checked_at);
  $('monitor-service').textContent=fresh?'本地服务在线 · 每 2 秒刷新':'本地服务未连接 / 状态已过期';
  $('probe-stat').textContent=(transport==='usb'?'USB':'Wi-Fi')+' · '+(labels[pc.state]||'检测中');
  $('probe-detail').textContent=pc.detail||'等待检测';
  $('connection-badge').textContent='SWD · '+labels[ts];
  $('connection-badge').classList.toggle('connected',ts==='online');
  $('side-state').textContent=`ESP32 ${labels[pc.state]||'检测中'} / STM32 ${labels[ts]}`;
  $('connect-main').textContent=liveState.session_active?'结束调试连接':'连接目标';
  $('connect-card').textContent=liveState.session_active?'结束调试连接':'连接并短暂暂停识别';
  if(ts!=='online'){$('target-run').textContent=labels[ts];$('debug-state').textContent=labels[ts];}
  if(typeof syncDeveloperStatus==='function')syncDeveloperStatus();
}
async function pollLinks(){
  if(linksPending||location.protocol==='file:')return;
  linksPending=true;
  const epoch=liveEpoch;
  try{
    const host=liveState.session_active?liveState.host:$('host-input').value.trim();
    const result=await service('links'+(validIP(host||'')?'?host='+encodeURIComponent(host):''));
    if(epoch!==liveEpoch)return;
    probeLinks=result;linksReceived=Date.now();
    syncProbeAddress();
  }catch(_){}finally{linksPending=false;renderLinks();}
}
let addressEdited=false;
function syncProbeAddress(){
  if(probeLinks.enabled===false)return;
  const now=Date.now(), current=$('host-input').value.trim();
  const available=Object.entries(probeLinks.http||{}).filter(([ip,x])=>x.state==='online'&&now-x.checked_at*1000<10000);
  let selected=available.find(([ip])=>ip===current);
  if(!selected && !liveState.session_active && !addressEdited && available.length===1)selected=available[0];
  if(!selected){if(!current&&!Object.keys(probeLinks.wifi||{}).length)$('management-result').textContent='找不到设备：请连接探针热点，或手动填写局域网IP。';return;}
  const [ip,info]=selected;
  if(!liveState.session_active && !addressEdited)$('host-input').value=ip;
  if(!$('management-ip').matches(':focus'))$('management-ip').value=ip;
  $('management-result').textContent=`已自动发现ESP32：${info.ssid||'局域网'} · ${ip}。登录后可直接在本页配网。`;
  try{localStorage.setItem('amphilink-probe-ip',ip);}catch(_){}
}
$('host-input').addEventListener('input',()=>{addressEdited=true;});
$('probe-link-toggle').onclick=()=>operate(probeLinks.enabled===false?'probe-reconnect':'probe-disconnect');
setTimeout(pollLinks,0);setInterval(pollLinks,2000);setInterval(renderLinks,1000);
